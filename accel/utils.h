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
