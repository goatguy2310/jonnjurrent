#define _CRT_SECURE_NO_WARNINGS 1

#include <iostream>
#include <chrono>

#include <omp.h>

#include "stb/stb_image_write.h"
#include "stb/stb_image.h"

#include "math/geometry.h"
#include "geometry/object.h"
#include "geometry/triangle_mesh.h"
#include "geometry/scene.h"
#include "io/obj_parser.h"
#include "render/renderer.h"
#include "accel/bvh.h"

#ifndef M_PI
#define M_PI 3.14159265358979323856
#endif

int main() {
	std::chrono::steady_clock::time_point begin_prep = std::chrono::steady_clock::now();

	Sphere center_sphere(Vector(0, 0, 0), 10., Vector(0.8, 0.8, 0.8));
	Sphere wall_left(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere wall_right(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere wall_front(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere wall_behind(Vector(0, 0, 1000), 940, Vector(0.8, 0.2, 0.9));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.7));

	TriangleMesh<BVH> cat(Vector(1., 1., 1.));
	readOBJ("assets/cat.obj", cat);
	cat.rotateX(-M_PI * 0.5);
//	cat.readTexture("assets/cat_diff.png");
	cat.scale_translate(0.6, Vector(0., -5., 0.));
	cat.updateBoundingBox();

	cat.buildAccel();

	Scene scene;
	scene.camera_center = Vector(0, 0, 55);
	scene.light_position = Vector(-10,20,40);
	scene.light_intensity = 1E7;
	scene.fov = 60 * M_PI / 180.;
	scene.gamma = 2.2;
	scene.max_light_bounce = 5;

	// scene.addObject(&center_sphere);

	scene.addObject(&wall_left);
	scene.addObject(&wall_right);
	scene.addObject(&wall_front);
	scene.addObject(&wall_behind);
	scene.addObject(&ceiling);
	scene.addObject(&floor);

	scene.addObject(&cat);

	Renderer r;
	r.W = 512;
	r.H = 512;
	r.sample_count = 32;

	std::chrono::steady_clock::time_point end_prep = std::chrono::steady_clock::now();
	std::cout << "Finished preparing objects and building BVHs in " << std::chrono::duration_cast<std::chrono::milliseconds> (end_prep - begin_prep).count() << "ms" << std::endl;

	std::chrono::steady_clock::time_point begin_render = std::chrono::steady_clock::now();

	r.render("build/image.png", scene);

	std::chrono::steady_clock::time_point end_render = std::chrono::steady_clock::now();
	std::cout << "Finished rendering in " << std::chrono::duration_cast<std::chrono::milliseconds> (end_render - begin_render).count() << "ms" << std::endl;

	return 0;
}
