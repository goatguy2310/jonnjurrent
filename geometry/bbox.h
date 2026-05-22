#pragma once

#include <optional>
#include "../math/geometry.h"

class BoundingBox {
public:
	Vector Bmin, Bmax;

	static BoundingBox init() {
		BoundingBox ret;
		double dmax = std::numeric_limits<double>::max();
		double dmin = std::numeric_limits<double>::lowest();

		ret.Bmin = Vector(dmax, dmax, dmax);
		ret.Bmax = Vector(dmin, dmin, dmin);

		return ret;
	}

	std::optional<double> intersect(const Ray& ray, double tmax = std::numeric_limits<double>::max()) const {
		double t0_max = std::numeric_limits<double>::lowest();
		double t1_min = std::numeric_limits<double>::max();
		for (int i = 0; i < 3; i++) {
			double inv_u = 1.0 / ray.u[i];
			double t0 = (Bmin[i] - ray.O[i]) * inv_u;
			double t1 = (Bmax[i] - ray.O[i]) * inv_u;

			if (t0 > t1) std::swap(t0, t1);

			t0_max = std::max(t0_max, t0);
			t1_min = std::min(t1_min, t1);
		}
		if (t1_min >= t0_max && t1_min >= 0. && t0_max < tmax) {
			return std::max(0., t0_max);
		}
		return std::nullopt;
	}

	void merge(const BoundingBox& other) {
		for (int i = 0; i < 3; i++) {
			Bmin[i] = std::min(Bmin[i], other.Bmin[i]);
			Bmax[i] = std::max(Bmax[i], other.Bmax[i]);
		}
	}

	void merge(const Vector& point) {
		for (int i = 0; i < 3; i++) {
			Bmin[i] = std::min(Bmin[i], point[i]);
			Bmax[i] = std::max(Bmax[i], point[i]);
		}
	}

	double surfaceArea() const {
		Vector d = Bmax - Bmin;
		if (d[0] < 0 || d[1] < 0 || d[2] < 0) return 0.;
		return 2. * (d[0] * d[1] + d[1] * d[2] + d[2] * d[0]);
	}
};
