#pragma once

#include <algorithm>
#include <string>

#include "../stb/stb_image_write.h"
#include "../stb/stb_image.h"

#include "../math/geometry.h"
#include "triangle_indices.h"
#include "object.h"
#include "bbox.h"

template<typename AccelStructure>
class TriangleMesh : public Object {
public:
	TriangleMesh(const Vector& albedo, bool mirror = false, bool transparent = false) : ::Object(albedo, mirror, transparent) {};

	void calcTriangleData() {
		for (auto& tri : indices) {
			tri.A = vertices[tri.vtx[0]];
			tri.e1 = vertices[tri.vtx[1]] - tri.A;
			tri.e2 = vertices[tri.vtx[2]] - tri.A;

			tri.N = cross(tri.e1, tri.e2);
			tri.centroid = tri.A + (tri.e1 + tri.e2) / 3;

			tri.bbox = BoundingBox::init();
			for (int i = 0; i < 3; i++) tri.bbox.merge(vertices[tri.vtx[i]]);
		}
	}

	void buildAccel(int num_threads) {
		accel.build(vertices, indices, num_threads);
	}

	// first scale and then translate the current object
	void scale_translate(double s, const Vector& t) {
		for (int i = 0; i < (int) vertices.size(); i++) {
			vertices[i] = vertices[i] * s + t;
		}
		calcTriangleData();
	}

	// read an .obj file
	void readTexture(std::string filename) {
		int channels;
		unsigned char* data = stbi_load(filename.c_str(), &tex_width, &tex_height, &channels, 3);
		if (data) {
			texture.assign(data, data + tex_width * tex_height * 3);
			free(data);
		}
	}
	

	std::optional<Intersection> intersect(const Ray& ray) const {
		if (!box.intersect(ray)) return std::nullopt;

		bool found = false;

		Intersection best_hit;
		best_hit.t = std::numeric_limits<double>::max();
		found = accel.intersect(ray, vertices, indices, normals, uvs, best_hit);

		if (found) return best_hit;
		return std::nullopt;
	}


	Vector getAlbedo(double u, double v) const {
		if (texture.empty()) return albedo;

		int i = u * (tex_width - 1);
		int j = (tex_height - 1) - v * (tex_height - 1);

		i = std::clamp(i, 0, tex_width - 1);
		j = std::clamp(j, 0, tex_height - 1);

		int pixel = (j * tex_width + i) * 3;
		return Vector(
			std::pow(texture[pixel] / 255., 2.2),
			std::pow(texture[pixel + 1] / 255., 2.2),
			std::pow(texture[pixel + 2] / 255., 2.2)
		);
	}

	// bounding box of indices in either vertices or indices, denoted by vtx
	BoundingBox calculateBoundingBox(int start, int end, bool vtx = true) {
		BoundingBox ret;

		double dmax = std::numeric_limits<double>::max();
		double dmin = std::numeric_limits<double>::lowest();

		ret.Bmin = Vector(dmax, dmax, dmax);
		ret.Bmax = Vector(dmin, dmin, dmin);
		if (vtx) {
			for (int i = start; i < end; i++) {
				const Vector& vert = vertices[i];
				for (int j = 0; j < 3; j++) {
					ret.Bmin[j] = std::min(ret.Bmin[j], vert[j]);
					ret.Bmax[j] = std::max(ret.Bmax[j], vert[j]);
				}
			}
		} else {
			for (int i = start; i < end; i++) {
				const TriangleIndices& ind = indices[i];
				for (int j = 0; j < 3; j++) { 
					const Vector& vert = vertices[ind.vtx[j]];

					for (int k = 0; k < 3; k++) {
						ret.Bmin[k] = std::min(ret.Bmin[k], vert[k]);
						ret.Bmax[k] = std::max(ret.Bmax[k], vert[k]);
					}
				}
			}

		}
		return ret;
	}

	void updateBoundingBox() {
		box = calculateBoundingBox(0, vertices.size());
	}

	void rotateX(float angle) {
		float c = std::cos(angle);
		float s = std::sin(angle);

		Vector center = (box.Bmin + box.Bmax) * 0.5;

		for (auto& v : vertices) {
		Vector p = v - center;
		v = Vector(
		    p[0],
		    c * p[1] - s * p[2],
		    s * p[1] + c * p[2]
		) + center;
		}

		for (auto& n : normals) {
		n = Vector(
		    n[0],
		    c * n[1] - s * n[2],
		    s * n[1] + c * n[2]
		);
		}

		updateBoundingBox();
		calcTriangleData();
	}	

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	BoundingBox box;

	std::vector<unsigned char> texture;
	int tex_width, tex_height;

	AccelStructure accel;
};


