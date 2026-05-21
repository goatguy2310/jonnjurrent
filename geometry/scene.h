#pragma once

#include <algorithm>

#include "object.h"
#include "../math/utils.h"

class Scene {
public:
	Scene() {};
	void addObject(const Object* obj) {
		objects.push_back(obj);
	}

	std::optional<Intersection> intersect(const Ray& ray, int &object_id) const  {
		std::optional<Intersection> best_hit = std::nullopt;
		for (int i = 0; i < (int) objects.size(); i++) {
			auto cur_hit = objects[i]->intersect(ray);

			if (cur_hit.has_value()) {
				if (!best_hit.has_value() || cur_hit->t < best_hit->t) {
					best_hit = cur_hit;
					object_id = i;
				}
			}
		}

		return best_hit;
	}

	Vector getColor(const Ray& ray, int recursion_depth) {
		if (recursion_depth >= max_light_bounce) return Vector(0, 0, 0);

		int object_id = -1;
		auto hit = intersect(ray, object_id);
		if (hit.has_value() && object_id >= 0) {
			Vector P = hit->P, N = hit->N;

			if (objects[object_id]->mirror) {
				// return getColor in the reflected direction, with recursion_depth+1 (recursively)
				Ray mirror_ray(P + eps * N, ray.u - 2 * dot(ray.u, N) * N);
				return getColor(mirror_ray, recursion_depth + 1);
			} // else

			if (objects[object_id]->transparent) { // optional
				// return getColor in the refraction direction, with recursion_depth+1 (recursively)
			} // else

			Vector albedo = objects[object_id]->getAlbedo(hit->uv[0], hit->uv[1]);

			// test if there is a shadow by sending a new ray
			// if there is no shadow, compute the formula with dot products etc.
			Vector omega = (light_position - P);
			double dist_to_light = omega.norm();
			omega.normalize();

			Ray shadow_ray(P + eps * N, omega);

			int sha_ID;
			auto shadowHit = intersect(shadow_ray, sha_ID);

			Vector L0(0., 0., 0.);
			if (!shadowHit.has_value() || shadowHit->t > dist_to_light) {
				L0 = light_intensity * albedo * std::max(0., dot(N, omega)) / (4 * M_PI * M_PI * dist_to_light * dist_to_light);
			}
			double r1 = getRandom();
			double r2 = getRandom();

			double x = std::sqrt(1 - r2) * std::cos(2 * M_PI * r1);
			double y = std::sqrt(1 - r2) * std::sin(2 * M_PI * r1);
			double z = std::sqrt(r2);

			double N_abs[3] = {std::abs(N[0]), std::abs(N[1]), std::abs(N[2])};
			double N_min = *std::min_element(N_abs, N_abs + 3);

			Vector T1;
			if (N_abs[0] == N_min) T1 = Vector(0, -N[2], N[1]);
			else if (N_abs[1] == N_min) T1 = Vector(-N[2], 0, N[0]);
			else T1 = Vector(-N[1], N[0], 0);

			T1.normalize();
			Vector T2 = cross(N, T1);
			
			Vector omega_i = x * T1 + y * T2 + z * N;
			omega_i.normalize();

			Ray random_ray(P + N * eps, omega_i);
			Vector Li = albedo * getColor(random_ray, recursion_depth + 1);

			return L0 + Li;
		}

		return Vector(0, 0, 0);
	}

	std::vector<const Object*> objects;

	Vector camera_center, light_position;
	double fov, gamma, light_intensity;
	int max_light_bounce;
};


