#pragma once

#include <vector>
#include <algorithm>
#include <thread>
#include "../geometry/triangle_indices.h"
#include "../geometry/bbox.h"

const int BINS_COUNT = 16;

struct Bin {
	BoundingBox bounds = BoundingBox::init();
	int count = 0;
};

inline void evaluateSAH(const Bin global_bins[3][BINS_COUNT], const BoundingBox& bounds, int indices_cnt, int& best_axis, int& best_split_index) {
	best_axis = -1;
	best_split_index = -1;
	double min_cost = indices_cnt * 1.5;

	for (int axis = 0; axis < 3; axis++) {
		if (bounds.Bmax[axis] - bounds.Bmin[axis] < eps) continue;

		int left_cnt[BINS_COUNT - 1], right_cnt[BINS_COUNT - 1];
		double left_areas[BINS_COUNT - 1], right_areas[BINS_COUNT - 1];

		// sweep: accumulate area and count from left to right (and vice versa)
		int cur_cnt = 0;
		BoundingBox current_box = BoundingBox::init();
		for (int i = 0; i < BINS_COUNT - 1; i++) {
			cur_cnt += global_bins[axis][i].count;
			current_box.merge(global_bins[axis][i].bounds);

			left_cnt[i] = cur_cnt;
			left_areas[i] = current_box.surfaceArea();
		}

		cur_cnt = 0;
		current_box = BoundingBox::init();
		for (int i = BINS_COUNT - 1; i > 0; i--) {
			cur_cnt += global_bins[axis][i].count;
			current_box.merge(global_bins[axis][i].bounds);

			right_cnt[i - 1] = cur_cnt;
			right_areas[i - 1] = current_box.surfaceArea();
		}

		// calculate sah cost for all BINS_COUNT splits
		double node_area = bounds.surfaceArea();
		for (int i = 0; i < BINS_COUNT - 1; i++) {
			if (left_cnt[i] == 0 || right_cnt[i] == 0) continue;
			
			double cost = 1. + (left_cnt[i] * left_areas[i] + right_cnt[i] * right_areas[i]) / node_area;
			if (cost < min_cost) {
				min_cost = cost;
				best_axis = axis;
				best_split_index = i;
			}
		}
	}
}

struct PartitionInfo {
	int left_start;
	int right_start;
};

inline int parallelPartition(std::vector<int>& index_map, std::vector<int>& temp_index_map, std::vector<uint8_t>& is_left, const std::vector<TriangleIndices>& indices, int start, int end, int best_axis, int best_split_index, double min_bound, double scale, int num_bins, int num_threads) {
	int len = end - start;

	// fallback to sequential partition for small arrays
	if (num_threads <= 1 || len < 1024) {
		int pivot = start;
		for (int i = start; i < end; i++) {
			double centroid = indices[index_map[i]].centroid[best_axis];
			int bin_idx = std::clamp((int)((centroid - min_bound) * scale), 0, num_bins - 1);

			if (bin_idx <= best_split_index) {
				std::swap(index_map[i], index_map[pivot]);
				pivot++;
			}
		}
		return pivot;
	}

	std::vector<int> local_left_cnt(num_threads, 0);
	int block_sz = len / num_threads;

	// map: evaluate split condition and store in is_left
	auto mapThread = [&](int thread_id, int start_t, int end_t) {
		int count = 0;
		for (int j = start_t; j < end_t; j++) {
			double centroid = indices[index_map[j]].centroid[best_axis];
			int bin_idx = std::clamp((int)((centroid - min_bound) * scale), 0, num_bins - 1);

			bool left = (bin_idx <= best_split_index);
			is_left[j] = left ? 1 : 0;
			if (left) count++;
		}
		local_left_cnt[thread_id] = count;
	};

	std::vector<std::thread> workers(num_threads - 1);
	int start_blk = start;
	for (int i = 0; i < num_threads - 1; i++) {
		int end_blk = start_blk + block_sz;
		workers[i] = std::thread(mapThread, i, start_blk, end_blk);
		start_blk = end_blk;
	}
	mapThread(num_threads - 1, start_blk, end);
	for (auto& w : workers) w.join();

	// scan: compute prefix sums to find global memory offsets
	std::vector<PartitionInfo> pinfo(num_threads);
	int total_left = 0;
	for (int i = 0; i < num_threads; i++) {
		pinfo[i].left_start = start + total_left;
		total_left += local_left_cnt[i];
	}
	int global_pivot = start + total_left;

	int current_right = global_pivot;
	for (int i = 0; i < num_threads; i++) {
		pinfo[i].right_start = current_right;
		int chunk_size = (i != num_threads - 1) ? block_sz : (len - i * block_sz);
		current_right += (chunk_size - local_left_cnt[i]);
	}

	// scatter: move elements to temporary buffer without data races
	auto scatterThread = [&](int thread_id, int start_t, int end_t) {
		int l_cursor = pinfo[thread_id].left_start;
		int r_cursor = pinfo[thread_id].right_start;
		for (int j = start_t; j < end_t; j++) {
			if (is_left[j]) {
				temp_index_map[l_cursor++] = index_map[j];
			} else {
				temp_index_map[r_cursor++] = index_map[j];
			}
		}
	};

	start_blk = start;
	for (int i = 0; i < num_threads - 1; i++) {
		int end_blk = start_blk + block_sz;
		workers[i] = std::thread(scatterThread, i, start_blk, end_blk);
		start_blk = end_blk;
	}
	scatterThread(num_threads - 1, start_blk, end);
	for (auto& w : workers) w.join();

	// copy temp buffer back to original array
	auto copyThread = [&](int start_t, int end_t) {
		for (int j = start_t; j < end_t; j++) {
			index_map[j] = temp_index_map[j];
		}
	};

	start_blk = start;
	for (int i = 0; i < num_threads - 1; i++) {
		int end_blk = start_blk + block_sz;
		workers[i] = std::thread(copyThread, start_blk, end_blk);
		start_blk = end_blk;
	}
	copyThread(start_blk, end);
	for (auto& w : workers) w.join();

	return global_pivot;
}
