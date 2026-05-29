#define _CRT_SECURE_NO_WARNINGS 1

#include <iostream>
#include <sstream>
#include <chrono>

#include <omp.h>

#include "../stb/stb_image_write.h"
#include "../stb/stb_image.h"

#include "../math/geometry.h"
#include "../geometry/scene.h"
#include "../io/obj_parser.h"

#include "../accel/bvh.h"
#include "../accel/parallel_bvh.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif

std::map<std::string, std::string> readConfig(const std::string& config_file) {
	std::map<std::string, std::string> config;
	std::ifstream file(config_file);
	std::string line;
	while (std::getline(file, line)) {
		if (line.empty()) continue;

		std::stringstream ss(line);
		std::string k, v;
		if (std::getline(ss, k, '=') && std::getline(ss, v)) {
			config[k] = v;
			std::cout << "Found " << k << " = " << v << "\n";
		}
	}
	return config;
}

std::vector<Ray> randomRaysIntoBbox(BoundingBox& bbox, int count) {
	double radius = (bbox.Bmax - bbox.Bmin).norm() * 2;
	std::vector<Ray> ret(count);
	for (int i = 0; i < count; i++) {
		Vector O = bbox.center() + Vector::randomUnit() * radius;
		Vector dir = (bbox.randomPoint() - O);
		dir.normalize();

		ret[i] = Ray(O, dir);
	}
	return ret;
}

void benchmark(std::string filename) {
	TriangleMesh<ParallelBVH> mesh(Vector(1., 1., 1.));
	readOBJ(filename, mesh);
	mesh.scale_translate(2., Vector(0., 0., 0.));
	mesh.updateBoundingBox();

	std::cout << "Loaded " << filename <<  " with " << mesh.vertices.size() << " vertices " << mesh.indices.size() << " triangles\n";

	Scene scene;
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10, 20, 40);
	scene.light_intensity = 1E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;
	scene.max_light_bounce = 5;

	scene.addObject(&mesh);

	std::vector<int> thread_cnts = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};
	const int iteration_bvh = 20;
	const int ray_count = 1e5;

	std::vector<Ray> rays = randomRaysIntoBbox(mesh.box, ray_count);

	for (int num_thread : thread_cnts) {
		std::cout << "Benchmarking " << num_thread << " thread(s)...\n";

		// warmup
		mesh.buildAccel(num_thread);

		auto start_bvh = std::chrono::steady_clock::now();
		for (int it = 0; it < iteration_bvh; it++) {
			mesh.buildAccel(num_thread);
		}
		auto end_bvh = std::chrono::steady_clock::now();
		std::cout << "Average BVH build: " << std::chrono::duration_cast<std::chrono::milliseconds> (end_bvh - start_bvh).count() / iteration_bvh << "ms\n";

		auto start_render = std::chrono::steady_clock::now();
		for (auto ray : rays) {
			scene.getColor(ray, 0);
		}
		auto end_render = std::chrono::steady_clock::now();	
		std::cout << "Average scene render: " << std::chrono::duration_cast<std::chrono::milliseconds> (end_render - start_render).count() << "ms for " << ray_count << " rays\n";

		std::cout << std::endl;
	}
}

int main() {
	std::string config_file = "config.txt";
	auto config = readConfig(config_file);

	std::cout << "Reading from file " << config["obj_file"] << "\n";
	
	benchmark(config["obj_file"]);
	return 0;
}
