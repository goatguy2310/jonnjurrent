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

void benchmark(std::string obj_file) {
	TriangleMesh<ParallelBVH> mesh(Vector(1., 1., 1.));

	std::cout << "Reading from file " << obj_file << "\n";
	readOBJ(obj_file, mesh);
	mesh.scale_translate(2., Vector(0., 0., 0.));
	mesh.updateBoundingBox();

	std::cout << "Loaded " << obj_file <<  " with " << mesh.vertices.size() << " vertices " << mesh.indices.size() << " triangles\n";

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
	std::vector<TriangleIndices> backup_indices = mesh.indices;

	int best_thread_cnt = 1, best_time = 1e9;
	for (int num_thread : thread_cnts) {
		std::cout << "Benchmarking " << num_thread << " thread(s)...\n";

		// warmup
		mesh.indices = backup_indices;
		mesh.buildAccel(num_thread);

		auto bvh_time = 0LL;
		for (int it = 0; it < iteration_bvh; it++) {
			mesh.indices = backup_indices;
	
			auto start_bvh = std::chrono::steady_clock::now();
			mesh.buildAccel(num_thread);
			auto end_bvh = std::chrono::steady_clock::now();

			bvh_time += std::chrono::duration_cast<std::chrono::milliseconds>(end_bvh - start_bvh).count();
		}
		bvh_time /= iteration_bvh;
		std::cout << "Average BVH build: " << bvh_time << "ms\n";

		if (bvh_time < best_time) best_thread_cnt = num_thread, best_time = bvh_time;

		auto start_render = std::chrono::steady_clock::now();
		for (auto ray : rays) {
			scene.getColor(ray, 0);
		}
		auto end_render = std::chrono::steady_clock::now();	
		auto time = std::chrono::duration_cast<std::chrono::milliseconds> (end_render - start_render).count();
		std::cout << "Average scene render: " << time << "ms for " << ray_count << " rays (" << (double) time / ray_count << "ms per ray)\n";

		std::cout << std::endl;
	}

	std::cout << "Going forward with " << best_thread_cnt << " thread count\n\n";

	int best_thres = 1024;
	best_time = 1e9;
	std::vector<int> parallel_thres = {256, 512, 1024, 2048, 4096, 8192};
	for (int thres : parallel_thres) {
		Config::set("parallel_threshold", std::to_string(thres));

		std::cout << "Benchmarking with " << thres << " parallel thres cutoff...\n";

		// warmup
		mesh.indices = backup_indices;
		mesh.buildAccel(best_thread_cnt);

		auto bvh_time = 0LL;
		for (int it = 0; it < iteration_bvh; it++) {
			mesh.indices = backup_indices;

			auto start_bvh = std::chrono::steady_clock::now();
			mesh.buildAccel(best_thread_cnt);
			auto end_bvh = std::chrono::steady_clock::now();

			bvh_time += std::chrono::duration_cast<std::chrono::milliseconds>(end_bvh - start_bvh).count();
		}
		bvh_time /= iteration_bvh;
		std::cout << "Average BVH build: " << bvh_time << "ms\n";
		
		if (bvh_time < best_time) best_thres = thres, best_time = bvh_time;

		std::cout << std::endl;
	}

	std::cout << "Best threshold: " << best_thres << "\n\n";
	Config::set("parallel_threshold", std::to_string(best_thres));
}

int main() {
	Config::load("config.txt");
	std::string obj_file = Config::get("obj_file");

	benchmark(obj_file);
	return 0;
}
