#pragma once

#include "../math/geometry.h"

class BoundingBox {
public:
	Vector Bmin, Bmax;

	bool intersect(const Ray& ray, double tmax = std::numeric_limits<double>::max()) const {
		double t0_max = std::numeric_limits<double>::lowest();
		double t1_min = std::numeric_limits<double>::max();
		for (int i = 0; i < 3; i++) {
			double t0 = (Bmin[i] - ray.O[i]) / ray.u[i];
			double t1 = (Bmax[i] - ray.O[i]) / ray.u[i];

			if (t0 > t1) std::swap(t0, t1);

			t0_max = std::max(t0_max, t0);
			t1_min = std::min(t1_min, t1);
		}
		return t1_min >= t0_max && t1_min >= 0. && t0_max < tmax;
	}

	void merge(const BoundingBox& other) {
		for (int i = 0; i < 3; i++) {
			Bmin[i] = std::min(Bmin[i], other.Bmin[i]);
			Bmax[i] = std::max(Bmax[i], other.Bmax[i]);
		}
	}
};