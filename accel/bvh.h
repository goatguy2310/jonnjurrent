#pragma once

#include <algorithm>

#include "../math/geometry.h"
#include "../geometry/object.h"
#include "../geometry/triangle_indices.h"
#include "../geometry/bbox.h"

class BVHNode {
public:
	BoundingBox box;
	int start_idx, end_idx; // indices of the range in the original indices array
	int left, right; // indices of the left and right child in the bvh_node array
	bool has_child = false;
};

class BVH {
public:
	void build(const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices) {
		bvhnodes.clear();
		bvhnodes.reserve(indices.size() * 2 - 1); // Perfect reserve for a full binary tree
		bvhnodes.emplace_back();
		buildBVHNode(vertices, indices, 0, 0, indices.size());
	}

	// node_idx: index in the node vector; start_idx, end_idx: index in the vertices vector
	void buildBVHNode(const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, int node_idx, int start_idx, int end_idx) {
		bvhnodes[node_idx].start_idx = start_idx;
		bvhnodes[node_idx].end_idx = end_idx;
		if (end_idx - start_idx < 5) {
			bvhnodes[node_idx].box = calculateBBoxIndices(vertices, indices, start_idx, end_idx);
		}

		Vector diag = bvhnodes[node_idx].box.Bmax - bvhnodes[node_idx].box.Bmin;
		Vector center = bvhnodes[node_idx].box.Bmin + diag * 0.5;

		int longest_ax = std::distance(diag.data, std::max_element(diag.data, diag.data + 3));
		int pivot_idx = start_idx;

		for (int i = start_idx; i < end_idx; i++) {
			if (indices[i].centroid[longest_ax] < center[longest_ax]) {
				std::swap(indices[i], indices[pivot_idx]);
				pivot_idx++;
			}
		}

		if (end_idx - start_idx < 5) return;

		// Fallback to equal-count split when spatial split degenerates.
		if (pivot_idx <= start_idx || pivot_idx >= end_idx) {
			pivot_idx = start_idx + (end_idx - start_idx) / 2;
		}

		if (pivot_idx <= start_idx || pivot_idx >= end_idx) return;

		bvhnodes[node_idx].left = bvhnodes.size();
		bvhnodes.emplace_back();
		bvhnodes[node_idx].right = bvhnodes.size();
		bvhnodes.emplace_back();
		bvhnodes[node_idx].has_child = true;

		buildBVHNode(vertices, indices, bvhnodes[node_idx].left, start_idx, pivot_idx);
		buildBVHNode(vertices, indices, bvhnodes[node_idx].right, pivot_idx, end_idx);

		bvhnodes[node_idx].box = bvhnodes[bvhnodes[node_idx].left].box;
		bvhnodes[node_idx].box.merge(bvhnodes[bvhnodes[node_idx].right].box);
	}

	BoundingBox calculateBBoxIndices(const std::vector<Vector>& vertices, const std::vector<TriangleIndices>& indices, int start, int end) {
		BoundingBox ret;

		double dmax = std::numeric_limits<double>::max();
		double dmin = std::numeric_limits<double>::lowest();

		ret.Bmin = Vector(dmax, dmax, dmax);
		ret.Bmax = Vector(dmin, dmin, dmin);
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
		return ret;
	}

	bool intersect(const Ray& ray, const std::vector<Vector>& vertices, const std::vector<TriangleIndices>& indices, const std::vector<Vector>& normals, const std::vector<Vector>& uvs, Intersection& best_hit, int idx = 0) const {
		if (bvhnodes.empty()) return false;

		const BVHNode& node = bvhnodes[idx];
		bool found = false;
		if (node.has_child) {
			bool left_hit = false, right_hit = false;
			if (bvhnodes[node.left].box.intersect(ray, best_hit.t)) left_hit = intersect(ray, vertices, indices, normals, uvs, best_hit, node.left);
			if (bvhnodes[node.right].box.intersect(ray, best_hit.t)) right_hit = intersect(ray, vertices, indices, normals, uvs, best_hit, node.right);
			found = left_hit || right_hit;
		} else {
			for (int i = node.start_idx; i < node.end_idx; i++) {
				const TriangleIndices& tri = indices[i];

				const Vector& A = tri.A;
				const Vector& e1 = tri.e1;
				const Vector& e2 = tri.e2;
				const Vector& N = tri.N;

				double uN = dot(ray.u, N);
				if (std::abs(uN) < eps) continue;

				Vector Axu = cross(A - ray.O, ray.u);

				double beta = dot(e2, Axu) / uN;
				double gamma = -dot(e1, Axu) / uN;
				double alpha = 1 - beta - gamma;
				double t_cur = dot(A - ray.O, N) / uN;

				if (t_cur >= best_hit.t || t_cur < eps || alpha < 0 || beta < 0 || gamma < 0) continue;

				best_hit.t = t_cur;
				best_hit.P = A + beta * e1 + gamma * e2;
				best_hit.N = alpha * normals[tri.n[0]] + beta * normals[tri.n[1]] + gamma * normals[tri.n[2]];
				best_hit.N.normalize();

				if (!uvs.empty()) best_hit.uv = alpha * uvs[tri.uv[0]] + beta * uvs[tri.uv[1]] + gamma * uvs[tri.uv[2]];

				found = true;
			}
		}
		return found;	
	}

	std::vector<BVHNode> bvhnodes;
};
