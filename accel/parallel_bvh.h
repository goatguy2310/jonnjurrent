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
#include "../core/config.h"
#include "utils.h"

#include "bvh.h"

class ParallelBVH {
public:
	void build(const std::vector<Vector>& vertices, std::vector<TriangleIndices>& indices, int num_threads) {
		p_vertices = &vertices;
		p_indices = &indices;
		node_counter.store(1);

		bvh_nodes.clear();
		bvh_nodes.resize(indices.size() * 2); // resize all for binary tree

		index_map.resize(indices.size()); // storing indirect index for more efficient swapping
		std::iota(index_map.begin(), index_map.end(), 0);
		
		temp_index_map.resize(indices.size());
		flags.resize(indices.size());

		buildBVHNode(0, 0, indices.size(), Config::getInt("adaptive_split"), Config::getInt("parallel_threshold"), num_threads);

		// forward permutation: reorder indices in-place using double copy
		indices_temp.resize(indices.size());
		#pragma omp parallel for schedule(static)
		for (size_t i = 0; i < indices.size(); i++) {
			indices_temp[i] = indices[index_map[i]];
		}
		std::swap(indices, indices_temp);

		if (Config::getInt("flatten")) flatten();
	}

	// node_idx: index in the node vector; start, end: index in the vertices vector
	void buildBVHNode(int node_idx, int start, int end, bool adaptive_split, int parallel_threshold, int num_threads) {
		auto& indices = *p_indices;

		bvh_nodes[node_idx].start = start;
		bvh_nodes[node_idx].end = end;

		bvh_nodes[node_idx].box = computeBounds(start, end, parallel_threshold, num_threads);
		BoundingBox& bounds = bvh_nodes[node_idx].box;

		if (end - start <= 2) return;

		int best_axis = -1;
		double max_extent = -1.0;
		for (int i = 0; i < 3; i++) {
			double extent = bounds.Bmax[i] - bounds.Bmin[i];
			if (extent > max_extent) {
				max_extent = extent;
				best_axis = i;
			}
		}

		if (max_extent < eps) {
			bvh_nodes[node_idx].box = bounds;
			return;
		}

		int len = end - start;
		
		// Find centroid bounds for tight bin mapping (required for single-axis SAH)
		double centroid_min = std::numeric_limits<double>::infinity();
		double centroid_max = -std::numeric_limits<double>::infinity();
		
		if (num_threads <= 1 || len < parallel_threshold) {
			for (int i = start; i < end; i++) {
				double c = indices[index_map[i]].centroid[best_axis];
				centroid_min = std::min(centroid_min, c);
				centroid_max = std::max(centroid_max, c);
			}
		} else {
			std::vector<std::thread> workers(num_threads - 1);
			std::vector<double> lmin(num_threads, std::numeric_limits<double>::infinity());
			std::vector<double> lmax(num_threads, -std::numeric_limits<double>::infinity());
			int block_sz = len / num_threads;
			
			auto minmax_worker = [&](int id, int s, int e) {
				for (int i = s; i < e; i++) {
					double c = indices[index_map[i]].centroid[best_axis];
					lmin[id] = std::min(lmin[id], c);
					lmax[id] = std::max(lmax[id], c);
				}
			};

			for (int i = 0; i < num_threads - 1; i++) {
				workers[i] = std::thread(minmax_worker, i, start + i * block_sz, start + (i + 1) * block_sz);
			}
			minmax_worker(num_threads - 1, start + (num_threads - 1) * block_sz, end);

			for (auto& w : workers) w.join();
			for (int i = 0; i < num_threads; i++) {
				centroid_min = std::min(centroid_min, lmin[i]);
				centroid_max = std::max(centroid_max, lmax[i]);
			}
		}

		double centroid_extent = centroid_max - centroid_min;
		if (centroid_extent < eps) {
			bvh_nodes[node_idx].box = bounds;
			return;
		}

		int best_split_index = -1;

		// setup global bins for binned sah
		Bin global_bins[BINS_COUNT];
		double scale = BINS_COUNT / centroid_extent;

		if (num_threads <= 1 || len < parallel_threshold) {
			// fallback to sequential binning
			for (int i = start; i < end; i++) {
				int idx = index_map[i];	
				double centroid = indices[idx].centroid[best_axis];
				int bin_idx = (int)((centroid - centroid_min) * scale);
				if (bin_idx < 0) bin_idx = 0;
				else if (bin_idx >= BINS_COUNT) bin_idx = BINS_COUNT - 1;

				global_bins[bin_idx].count++;
				global_bins[bin_idx].bounds.merge(indices[idx].bbox);
			}
		} else {
			// parallel map-reduce binning
			int block_sz = len / num_threads;
			std::vector<std::vector<Bin>> local_bins(num_threads, std::vector<Bin>(BINS_COUNT));
			
			auto binMap = [&](int thread_id, int start_bin, int end_bin) {
				// map phase: populate thread-local bins
				for (int i = start_bin; i < end_bin; i++) {
					int idx = index_map[i];	
					double centroid = indices[idx].centroid[best_axis];
					int bin_idx = (int)((centroid - centroid_min) * scale);
					if (bin_idx < 0) bin_idx = 0;
					else if (bin_idx >= BINS_COUNT) bin_idx = BINS_COUNT - 1;

					local_bins[thread_id][bin_idx].count++;
					local_bins[thread_id][bin_idx].bounds.merge(indices[idx].bbox);
				}
			};

			std::vector<std::thread> workers(num_threads - 1);
			for (int i = 0; i < num_threads - 1; i++) {
				workers[i] = std::thread(binMap, i, start + i * block_sz, start + (i + 1) * block_sz);
			}
			binMap(num_threads - 1, start + (num_threads - 1) * block_sz, end);
			for (auto& w : workers) w.join();

			// reduce phase: merge thread-local bins into global bins
			for (int t = 0; t < num_threads; t++) {
				for (int b = 0; b < BINS_COUNT; b++) {
					global_bins[b].count += local_bins[t][b].count;
					global_bins[b].bounds.merge(local_bins[t][b].bounds);
				}
			}
		}

		// evaluate sah cost to find best split
		evaluateSAH(global_bins, bounds, end - start, best_split_index);

		// if no split is better than parent, make it a leaf
		if (best_split_index == -1) {
			bvh_nodes[node_idx].box = bounds;
			return;
		}

		// two pointer partitioning based on best split
		double split_plane = centroid_min + centroid_extent * ((best_split_index + 1.0) / BINS_COUNT);
		
		int left = start;
		int right = end - 1;
		while (left <= right) {
			if (indices[index_map[left]].centroid[best_axis] <= split_plane) {
				left++;
			} else {
				std::swap(index_map[left], index_map[right]);
				right--;
			}
		}

		int pivot_idx = std::clamp(left, start + 1, end - 1);

		int left_idx = node_counter.fetch_add(2);
		int right_idx = left_idx + 1;

		bvh_nodes[node_idx].left = left_idx;
		bvh_nodes[node_idx].right = right_idx;
		bvh_nodes[node_idx].has_child = true;

		// proportional thread allocation based on child workload size
		int left_threads = 1, right_threads = 1;
		if (num_threads > 1) {
			if (adaptive_split) {
				int total_size = end - start;
				int left_size = pivot_idx - start;
				left_threads = std::max(1, (int) std::round((double) left_size / total_size * num_threads));
				left_threads = std::clamp(left_threads, 1, num_threads - 1);
				right_threads = num_threads - left_threads;
			} else {
				left_threads = num_threads / 2;
				right_threads = num_threads - left_threads;
			}

			std::thread left_worker([&]() {
				buildBVHNode(left_idx, start, pivot_idx, adaptive_split, parallel_threshold, left_threads);
			});
			buildBVHNode(right_idx, pivot_idx, end, adaptive_split, parallel_threshold, right_threads);
			left_worker.join();
		} else {
			buildBVHNode(left_idx, start, pivot_idx, adaptive_split, parallel_threshold, 1);
			buildBVHNode(right_idx, pivot_idx, end, adaptive_split, parallel_threshold, 1);
		}
	}

	BoundingBox computeBounds(int start, int end, int parallel_threshold, int num_threads) {
		auto& indices = *p_indices;
		BoundingBox ret = BoundingBox::init();

		auto boundsThread = [&](int start, int end, BoundingBox& bbox) {
			BoundingBox ret = BoundingBox::init();
			for (int i = start; i < end; i++) ret.merge(indices[index_map[i]].bbox);
			bbox = ret;
		};	

		int len = end - start;
		if (num_threads <= 1 || len <= parallel_threshold) {
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

	void flatten() {
		if (bvh_nodes.empty()) return;

		bvh_nodes_flat.clear();
		bvh_nodes_flat.reserve(bvh_nodes.size());

		bvh_nodes_flat.push_back(bvh_nodes[0]);

		flattenNode(0, 0, bvh_nodes_flat);

		std::swap(bvh_nodes, bvh_nodes_flat);
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
private:


	const std::vector<Vector>* p_vertices = nullptr;
	std::vector<TriangleIndices>* p_indices = nullptr;
	std::atomic<int> node_counter{1};
	std::vector<int> index_map;
	std::vector<int> temp_index_map;
	std::vector<uint8_t> flags;
	std::vector<TriangleIndices> indices_temp;
	std::vector<BVHNode> bvh_nodes_flat;
};
