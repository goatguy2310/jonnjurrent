.PHONY: all bench clean

CXX = g++
CXXFLAGS = -O3 -flto=auto -march=native -std=c++23 -fopenmp -Wall -MMD -MP

TARGET = build/main
SRCS = main.cpp stb/stb.cpp
OBJS = $(SRCS:%.cpp=build/%.o)

BENCH_TARGET = build/bench
BENCH_SRCS = benchmarks/main_bench.cpp stb/stb.cpp
BENCH_OBJS = $(BENCH_SRCS:%.cpp=build/%.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

bench: $(BENCH_TARGET)

$(BENCH_TARGET): $(BENCH_OBJS)
	$(CXX) $(CXXFLAGS) -o $(BENCH_TARGET) $(BENCH_OBJS)

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(OBJS:.o=.d) $(BENCH_OBJS:.o=.d)

clean:
	rm -rf build/ image.png
