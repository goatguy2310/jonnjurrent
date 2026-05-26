use core::f64;
use criterion::Criterion;
use std::hint::black_box;

use crate::common;
use renderer::larp::{Boundable, Bvh};

const RAY_COUNT: usize = 100_000;

pub fn seq_cat(c: &mut Criterion) {
    let mesh = common::cat().build_soup();
    let bvh = Bvh::build(&mesh);
    let root_bbox = bvh.bounding_box();

    let rays = common::rays_into_bbox(root_bbox, RAY_COUNT);

    c.bench_function("<Sequential BVH Traversal: Cat>", |b| {
        b.iter(|| {
            for ray in &rays {
                black_box(bvh.intersect_ray(ray, f64::EPSILON, f64::INFINITY));
            }
        });
    });
}

pub fn seq_lucky(c: &mut Criterion) {
    let mesh = common::lucky().build_soup();
    let bvh = Bvh::build(&mesh);
    let root_bbox = bvh.bounding_box();

    let rays = common::rays_into_bbox(root_bbox, RAY_COUNT);

    c.bench_function("<Sequential BVH Traversal: Lucky>", |b| {
        b.iter(|| {
            for ray in &rays {
                black_box(bvh.intersect_ray(ray, f64::EPSILON, f64::INFINITY));
            }
        });
    });
}

pub fn seq_maria(c: &mut Criterion) {
    let mesh = common::maria().build_soup();
    let bvh = Bvh::build(&mesh);
    let root_bbox = bvh.bounding_box();

    let rays = common::rays_into_bbox(root_bbox, RAY_COUNT);

    c.bench_function("<Sequential BVH Traversal: Maria>", |b| {
        b.iter(|| {
            for ray in &rays {
                black_box(bvh.intersect_ray(ray, f64::EPSILON, f64::INFINITY));
            }
        });
    });
}
