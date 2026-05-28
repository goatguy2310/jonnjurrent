use criterion::{criterion_group, criterion_main};

mod common;

mod larp;
use crate::larp::*;

criterion_group!(
    benches,
    build::seq_cat,
    // build::seq_lucky,
    build::seq_maria,
    build::par_cat,
    // traversal::seq_cat,
    // traversal::seq_lucky,
    traversal::seq_maria,
);
criterion_main!(benches);
