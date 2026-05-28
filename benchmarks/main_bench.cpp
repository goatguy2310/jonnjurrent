#define _CRT_SECURE_NO_WARNINGS 1

#include <iostream>
#include <chrono>

#include <omp.h>

#include "../stb/stb_image_write.h"
#include "../stb/stb_image.h"

#include "../math/geometry.h"
#include "../geometry/object.h"
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

int main() {
	Sphere center_sphere(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8));
	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));

	std::string filename = "assets/Maria_C.obj";

	TriangleMesh<ParallelBVH> mesh(Vector(1., 1., 1.));
	readOBJ(filename, mesh);
//	mesh.rotateX(-M_PI * 0.5);
//	mesh.readTexture("assets/Maria_C_Maria_O.png");
	mesh.scale_translate(15., Vector(0., -25., 0.));
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

	std::vector<int> thread_cnts = {1, 2, 4, 8, 12, 16};
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

	return 0;
}
