#pragma once

#include <optional>

#include "../math/geometry.h"

struct Intersection {
	Vector P;
	Vector N;
	Vector uv;
	double t;
};

class Object {
public:
	Object(const Vector& albedo, bool mirror = false, bool transparent = false) : albedo(albedo), mirror(mirror), transparent(transparent) {};
	virtual ~Object() = default;

	virtual std::optional<Intersection> intersect(const Ray& ray) const = 0;

	virtual Vector getAlbedo(double u, double v) const {
		return albedo;
	}

	Vector albedo;
	bool mirror, transparent;
};

class Sphere : public Object {
public:
	Sphere(const Vector& center, double radius, const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent), C(center), R(radius) {};

	std::optional<Intersection> intersect(const Ray& ray) const {
		Vector O_to_C = ray.O - C;
		double uC = dot(ray.u, O_to_C);
		double delta = uC * uC - (O_to_C.norm2() - R * R);

		if (delta < 0) return std::nullopt;

		double sqrt_del = std::sqrt(delta);
		double t1 = -uC - sqrt_del, t2 = -uC + sqrt_del;
		if (t2 < 0) return std::nullopt;

		double t = (t1 >= 0 ? t1 : t2);

		Vector P = ray.O + t * ray.u;

		Vector N = P - C;
		N.normalize();
		return Intersection{P, N, Vector(0, 0, 0), t};
	}

	Vector C;
	double R;
};


