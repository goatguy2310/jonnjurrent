#pragma once

#include <random>
#include <omp.h>

inline double getRandom() {
	thread_local std::default_random_engine engine(67 + omp_get_thread_num());
	thread_local std::uniform_real_distribution<double> uniform(0, 1);

	return uniform(engine);
}

inline double getRandom(double l, double r) {
	thread_local std::default_random_engine engine(67 + omp_get_thread_num());
	thread_local std::uniform_real_distribution<double> uniform(0, 1);

	return l + uniform(engine) * (r - l);
}
