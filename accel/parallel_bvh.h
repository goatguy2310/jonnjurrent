#pragma once

#include <algorithm>
#include <numeric>
#include <thread>
#include <cmath>
#include <atomic>

#include "../math/geometry.h"
#include "../geometry/object.h"
#include "../geometry/triangle_indices.h"
#include "../geometry/bbox.h"
#include "utils.h"

#include "bvh.h"

class ParallelBVH {
public:
	void build(const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, int num_threads) {
		bvh_nodes.clear();
		bvh_nodes.resize(indices.size() * 2); // resize all for binary tree

		std::vector<int> index_map (indices.size()); // storing indirect index for more efficient swapping
		std::iota(index_map.begin(), index_map.end(), 0);
		
		std::vector<int> temp_index_map(indices.size());
		std::vector<uint8_t> flags(indices.size());

		std::atomic<int> node_counter(1);

		buildBVHNode(vertices, indices, index_map, temp_index_map, flags, 0, 0, indices.size(), num_threads, node_counter);

		// forward permutation: reorder indices in-place using double copy
		std::vector<TriangleIndices> temp(indices.size());
		for (size_t i = 0; i < indices.size(); i++) {
			temp[i] = std::move(indices[index_map[i]]);
		}
		indices = std::move(temp);
	}

	// node_idx: index in the node vector; start, end: index in the vertices vector
	void buildBVHNode(const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, std::vector<int>& index_map, std::vector<int>& temp_index_map, std::vector<uint8_t>& flags, int node_idx, int start, int end, int num_threads, std::atomic<int>& node_counter) {
		bvh_nodes[node_idx].start = start;
		bvh_nodes[node_idx].end = end;

		bvh_nodes[node_idx].box = computeBounds(indices, index_map, start, end, num_threads);
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

		int len = end - start;
		if (num_threads <= 1 || len < 1024) {
			// fallback to sequential binning
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
		} else {
			// parallel map-reduce binning
			int block_sz = len / num_threads;
			std::vector<std::vector<std::vector<Bin>>> local_bins(num_threads, std::vector<std::vector<Bin>>(3, std::vector<Bin>(BINS_COUNT)));
			
			auto binMap = [&](int thread_id, int start_bin, int end_bin) {
				// map phase: populate thread-local bins
				for (int i = start_bin; i < end_bin; i++) {
					int idx = index_map[i];	

					for (int axis = 0; axis < 3; axis++) {
						if (bounds.Bmax[axis] - bounds.Bmin[axis] < eps) continue;
						double centroid = indices[idx].centroid[axis];
						int bin_idx = std::clamp((int)((centroid - bounds.Bmin[axis]) * scales[axis]), 0, BINS_COUNT - 1);
						local_bins[thread_id][axis][bin_idx].count++;
						local_bins[thread_id][axis][bin_idx].bounds.merge(indices[idx].bbox);
					}
				}
			};

			// spawn workers for map phase
			std::vector<std::thread> workers(num_threads - 1);
			int start_blk = start;
			for (int i = 0; i < num_threads - 1; i++) {
				int end_blk = start_blk + block_sz;
				workers[i] = std::thread(binMap, i, start_blk, end_blk);
				start_blk = end_blk;
			}
			binMap(num_threads - 1, start_blk, end);
			for (auto& w : workers) w.join();

			// reduce phase: merge thread-local bins into global bins
			for (int t = 0; t < num_threads; t++) {
				for (int axis = 0; axis < 3; axis++) {
					for (int b = 0; b < BINS_COUNT; b++) {
						global_bins[axis][b].count += local_bins[t][axis][b].count;
						global_bins[axis][b].bounds.merge(local_bins[t][axis][b].bounds);
					}
				}
			}
		}

		// evaluate sah cost to find best split
		evaluateSAH(global_bins, bounds, end - start, best_axis, best_split_index);

		// if no split is better than parent, make it a leaf
		if (best_axis == -1) {
			bvh_nodes[node_idx].box = bounds;
			return;
		}

		// parallel partition array based on best split
		double scale = BINS_COUNT / (bounds.Bmax[best_axis] - bounds.Bmin[best_axis]);
		int pivot_idx = parallelPartition(index_map, temp_index_map, flags, indices, start, end, best_axis, best_split_index, bounds.Bmin[best_axis], scale, BINS_COUNT, num_threads);

		int left_idx = node_counter.fetch_add(1);
		int right_idx = node_counter.fetch_add(1);

		bvh_nodes[node_idx].left = left_idx;
		bvh_nodes[node_idx].right = right_idx;
		bvh_nodes[node_idx].has_child = true;

		// proportional thread allocation based on child workload size
		int left_threads = 1, right_threads = 1;
		if (num_threads > 1) {
			int total_size = end - start;
			int left_size = pivot_idx - start;
			left_threads = std::max(1, (int) std::round((double) left_size / total_size * num_threads));
			left_threads = std::clamp(left_threads, 1, num_threads - 1);
			right_threads = num_threads - left_threads;
		}

		if (num_threads > 1) {
			std::thread left_worker([&]() {
				buildBVHNode(vertices, indices, index_map, temp_index_map, flags, left_idx, start, pivot_idx, left_threads, node_counter);
			});
			buildBVHNode(vertices, indices, index_map, temp_index_map, flags, right_idx, pivot_idx, end, right_threads, node_counter);
			left_worker.join();
		} else {
			buildBVHNode(vertices, indices, index_map, temp_index_map, flags, left_idx, start, pivot_idx, 1, node_counter);
			buildBVHNode(vertices, indices, index_map, temp_index_map, flags, right_idx, pivot_idx, end, 1, node_counter);
		}
	}

	BoundingBox computeBounds(const std::vector<TriangleIndices>& indices, const std::vector<int>& index_map, int start, int end, int num_threads) {
		BoundingBox ret = BoundingBox::init();

		auto boundsThread = [&](int start, int end, BoundingBox& bbox) {
			BoundingBox ret = BoundingBox::init();
			for (int i = start; i < end; i++) ret.merge(indices[index_map[i]].bbox);
			bbox = ret;
		};	

		int len = end - start;
		if (num_threads <= 1 || len <= 1024) {
			boundsThread(start, end, ret);
			return ret;
		}

		int block_sz = len / num_threads;
		std::vector<BoundingBox> bbox_vec(num_threads);
		std::vector<std::thread> workers(num_threads - 1);
		int start_blk = start;

		for (int i = 0; i < num_threads - 1; i++) {
			int end_blk = start_blk + block_sz;
			workers[i] = std::thread(boundsThread, start_blk, end_blk, std::ref(bbox_vec[i]));
			start_blk = end_blk;
		}
		boundsThread(start_blk, end, bbox_vec.back());
		for (auto& w : workers) w.join();

		for (auto& bbox : bbox_vec) ret.merge(bbox);

		return ret;
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
