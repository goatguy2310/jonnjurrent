# Jonnjurrent

A highly concurrent C++23 research repository exploring multiple parallel Bounding Volume Hierarchy (BVH) builder implementations for a Physically Based Rendering (PBR) path tracer.

## Repository Structure

* `accel/` - **Core Focus.** Parallel algorithms for BVH construction and traversal (e.g., SAH binning, spatial splits).
* `benchmark/` - Performance and scalability tests for the BVH builders.
* `geometry/` - Ray-primitive intersections, triangle meshes, and scene.
* `render/` - Minimal path tracing renderer.
* `math/` - High-performance vector and ray math structures.
* `io/` - Lightweight `.obj` mesh parser.
* `stb/` - Image writing utilities (vendor).

## BVH Algorithms

* `bvh.h`: The original sequential BVH + SAH algorithm
* `parallel_bvh.h`: Parallel BVH + SAH algorithm, consisting of parallel bounding box calc, parallel binning, parallel partitioning using mainly `std::thread`
* `omp_bvh.h`: Parallel BVH + SAH implemented with OpenMP

By default, the benchmarks use ParallelBVH.

## Build & Run

To run the renderer,
```bash
make
./build/main
```

To run the benchmark,
```bash
make bench
./build/bench
```

For modifying parameters, you can modify `config.txt` which will affect policy globally. In particular,
`obj_file` (string): Path to the target OBJ file of the 3D mesh
`flatten` (0/1): Apply post-processing flattening to the BVHs for cache
`num_threads` (int): Maximum thread pool size allowed for parallel building, mainly to be used with `./build/main`
`parallel_threshold` (int): Threshold of primitive for a node to switch from parallel to sequential
`adaptive_split` (0/1): Toggle adaptive BVH children split based on number of primitives in children
