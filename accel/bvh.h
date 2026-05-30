#pragma once

#include <algorithm>
#include <numeric>

#include "../math/geometry.h"
#include "../geometry/object.h"
#include "../geometry/triangle_indices.h"
#include "../geometry/bbox.h"
#include "../core/config.h"
#include "utils.h"

class BVHNode {
public:
	BoundingBox box;
	int start, end; // indices of the range in the original indices array
	int left, right; // indices of the left and right child in the bvh_node array
	bool has_child = false;
};

class BVH {
public:
	void build(const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, int num_threads) {
		bvh_nodes.clear();
		bvh_nodes.reserve(indices.size() * 2); // reserve for binary tree
		bvh_nodes.emplace_back();

		std::vector<int> index_map(indices.size());
		std::iota(index_map.begin(), index_map.end(), 0);

		buildBVHNode(vertices, indices, index_map, 0, 0, indices.size());

		std::vector<TriangleIndices> temp(indices.size());
		for (size_t i = 0; i < indices.size(); i++) {
			temp[i] = std::move(indices[index_map[i]]);
		}
		indices = std::move(temp);

		if (Config::getInt("flatten")) flatten();
	}

	// node_idx: index in the node vector; start, end: index in the vertices vector
	void buildBVHNode(const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, std::vector<int>& index_map, int node_idx, int start, int end) {
		bvh_nodes[node_idx].start = start;
		bvh_nodes[node_idx].end = end;

		bvh_nodes[node_idx].box = computeBounds(indices, index_map, start, end);
		BoundingBox& bounds = bvh_nodes[node_idx].box;
		if (end - start <= 2) return;

		int best_axis = -1;
		int best_split_index = -1;

		// setup global bins for binned sah & reciprocal of total for each axis
		Bin global_bins[3][BINS_COUNT];
		double scales[3];
		for (int axis = 0; axis < 3; axis++) {
			scales[axis] = BINS_COUNT / (bounds.Bmax[axis] - bounds.Bmin[axis]);
		}

		// sequential binning
		for (int i = start; i < end; i++) {
			int idx = index_map[i];
			for (int axis = 0; axis < 3; axis++) {
				if (bounds.Bmax[axis] - bounds.Bmin[axis] < eps) continue;
				double centroid = indices[idx].centroid[axis];
				int bin_idx = std::clamp((int)((centroid - bounds.Bmin[axis]) * scales[axis]), 0, BINS_COUNT - 1);
				global_bins[axis][bin_idx].count++;
				global_bins[axis][bin_idx].bounds.merge(indices[idx].bbox);
			}
		}

		// evaluate sah cost to find best split
		evaluateSAH(global_bins, bounds, end - start, best_axis, best_split_index);

		// if no split is better than parent, make it a leaf
		if (best_axis == -1) {
			bvh_nodes[node_idx].box = bounds;
			return;
		}

		// sequential partition array based on best split
		double scale = BINS_COUNT / (bounds.Bmax[best_axis] - bounds.Bmin[best_axis]);
		int pivot_idx = start;
		for (int i = start; i < end; i++) {
			double centroid = indices[index_map[i]].centroid[best_axis];
			int bin_idx = std::clamp((int)((centroid - bounds.Bmin[best_axis]) * scale), 0, BINS_COUNT - 1);
			
			if (bin_idx <= best_split_index) {
				std::swap(index_map[i], index_map[pivot_idx]);
				pivot_idx++;
			}
		}

		int left_idx = bvh_nodes.size();
		bvh_nodes.emplace_back();
		int right_idx = bvh_nodes.size();
		bvh_nodes.emplace_back();

		bvh_nodes[node_idx].left = left_idx;
		bvh_nodes[node_idx].right = right_idx;
		bvh_nodes[node_idx].has_child = true;

		buildBVHNode(vertices, indices, index_map, left_idx, start, pivot_idx);
		buildBVHNode(vertices, indices, index_map, right_idx, pivot_idx, end);
	}

	BoundingBox computeBounds(const std::vector<TriangleIndices>& indices, const std::vector<int>& index_map, int start, int end) {
		BoundingBox ret = BoundingBox::init();

		for (int i = start; i < end; i++) ret.merge(indices[index_map[i]].bbox);
		return ret;
	}

	void flatten() {
		if (bvh_nodes.empty()) return;

		std::vector<BVHNode> bvh_nodes_flat;
		bvh_nodes_flat.reserve(bvh_nodes.size());

		bvh_nodes_flat.push_back(bvh_nodes[0]);

		flattenNode(0, 0, bvh_nodes_flat);

		bvh_nodes = std::move(bvh_nodes_flat);
	}

	void flattenNode(int old_idx, int new_idx, std::vector<BVHNode>& bvh_nodes_flat) {
		if (!bvh_nodes[old_idx].has_child) return;

		auto& old_node = bvh_nodes[old_idx];
		int l_old = old_node.left, r_old = old_node.right;

		int l_new = bvh_nodes_flat.size();
		bvh_nodes_flat.push_back(bvh_nodes[l_old]);

		int r_new = bvh_nodes_flat.size();
		bvh_nodes_flat.push_back(bvh_nodes[r_old]);

		bvh_nodes_flat[new_idx].left = l_new;
		bvh_nodes_flat[new_idx].right = r_new;

		flattenNode(l_old, l_new, bvh_nodes_flat);
		flattenNode(r_old, r_new, bvh_nodes_flat);
	}

	bool intersect(const Ray& ray, const std::vector<Vector>& vertices, const std::vector<TriangleIndices>& indices, const std::vector<Vector>& normals, const std::vector<Vector>& uvs, Intersection& best_hit, int idx = 0) const {
		if (bvh_nodes.empty()) return false;

		const BVHNode& node = bvh_nodes[idx];
		bool found = false;
		if (node.has_child) {
			auto t_left = bvh_nodes[node.left].box.intersect(ray, best_hit.t);
			auto t_right = bvh_nodes[node.right].box.intersect(ray, best_hit.t);

			int first = node.left, second = node.right;

			// test the nearer child first so may allow early pruning
			if (t_left && t_right && *t_right < *t_left) {
				std::swap(first, second);
				std::swap(t_left, t_right);
			}

			if (t_left && intersect(ray, vertices, indices, normals, uvs, best_hit, first)) {
				found = true;
			}
			if (t_right && *t_right < best_hit.t && intersect(ray, vertices, indices, normals, uvs, best_hit, second)) {
				found = true;
			}
		} else {
			for (int i = node.start; i < node.end; i++) {
				const TriangleIndices& tri = indices[i];

				const Vector& A = tri.A;
				const Vector& e1 = tri.e1;
				const Vector& e2 = tri.e2;
				const Vector& N = tri.N;

				double uN = dot(ray.u, N);
				if (std::abs(uN) < eps) continue;

				Vector AO = A - ray.O;
				Vector Axu = cross(AO, ray.u);

				double inv_uN = 1.0 / uN;

				double beta = dot(e2, Axu) * inv_uN;
				double gamma = -dot(e1, Axu) * inv_uN;
				double alpha = 1 - beta - gamma;
				double t_cur = dot(AO, N) * inv_uN;

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

	std::vector<BVHNode> bvh_nodes;
};
