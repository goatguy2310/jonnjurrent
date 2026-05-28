use crate::larp::BoundingBox;

pub trait FlattenableBvh {
    fn flatten(&self) -> FlatBvh;
}

enum FlatBvhNode {
    Internal {
        offset: u32,
    },
    Leaf {},
}

struct FlatBvh {}
