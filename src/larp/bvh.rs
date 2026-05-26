use crate::{
    larp::{Boundable, BoundingBox},
    math::{Ray, Vector},
};

#[derive(Debug, Clone)]
pub enum BvhNode {
    Internal {
        bbox: BoundingBox,
        left: u32,
        right: u32,
    },

    Leaf {
        bbox: BoundingBox,
        start: u32,
        end: u32,
    },
}

impl BvhNode {
    fn new_internal(bbox: BoundingBox, left: usize, right: usize) -> Self {
        Self::Internal {
            bbox,
            left: left as u32,
            right: right as u32,
        }
    }

    fn new_leaf(bbox: BoundingBox, start: usize, end: usize) -> Self {
        Self::Leaf {
            bbox,
            start: start as u32,
            end: end as u32,
        }
    }
}

impl Boundable for BvhNode {
    fn bounding_box(&self) -> BoundingBox {
        match self {
            Self::Internal { bbox, .. } => bbox.clone(),
            Self::Leaf { bbox, .. } => bbox.clone(),
        }
    }
}

#[derive(Debug, Clone)]
struct CachedPrimitive {
    bounding_box: BoundingBox,
    center: Vector,
}

impl CachedPrimitive {
    #[inline]
    #[must_use]
    fn with_primitive<T: Boundable>(primitive: &T) -> Self {
        let bounding_box = primitive.bounding_box();
        let center = bounding_box.center();

        CachedPrimitive {
            bounding_box,
            center,
        }
    }
}

#[derive(Debug, Clone)]
struct Bucket {
    count: usize,
    bbox: BoundingBox,
}

impl Bucket {
    pub const COUNT: usize = 12;

    #[inline]
    #[must_use]
    const fn new() -> Self {
        Self {
            count: 0,
            bbox: BoundingBox::EMPTY,
        }
    }

    #[inline]
    const fn add(&mut self, bbox: &BoundingBox) {
        self.count += 1;
        self.bbox.union_mut(bbox);
    }
}

#[derive(Debug, Clone)]
pub struct Bvh {
    nodes: Vec<BvhNode>,
    primitive_indices: Vec<usize>,
}

impl Bvh {
    #[inline(always)]
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            nodes: Vec::new(),
            primitive_indices: Vec::new(),
        }
    }

    pub fn build<T: Boundable>(primitives: &[T]) -> Self {
        let primitive_count = primitives.len();
        let mut nodes = Vec::with_capacity(2 * primitive_count);
        let mut primitive_indices: Vec<usize> = (0..primitive_count).collect();

        let cached_primitives = primitives
            .iter()
            .map(CachedPrimitive::with_primitive)
            .collect::<Vec<CachedPrimitive>>();

        Self::build_recursive(
            &cached_primitives,
            &mut primitive_indices,
            0,
            primitive_count,
            &mut nodes,
        );

        Bvh {
            nodes,
            primitive_indices,
        }
    }

    fn build_recursive(
        primitives: &[CachedPrimitive],
        primitive_indices: &mut [usize],
        start: usize,
        end: usize,
        nodes: &mut Vec<BvhNode>,
    ) -> usize {
        let bbox = Self::compute_bbox(primitives, &primitive_indices[start..end]);
        let node_index = nodes.len();

        if end - start <= 4 {
            nodes.push(BvhNode::new_leaf(bbox, start, end));
            return node_index;
        }

        nodes.push(BvhNode::Internal {
            bbox: bbox.clone(),
            left: 0,
            right: 0,
        });

        let split_position = Self::find_sah_split(primitives, primitive_indices, start, end, &bbox);

        let Some(split_position) = split_position else {
            nodes[node_index] = BvhNode::new_leaf(bbox, start, end);
            return node_index;
        };

        let left =
            Self::build_recursive(primitives, primitive_indices, start, split_position, nodes);
        let right =
            Self::build_recursive(primitives, primitive_indices, split_position, end, nodes);

        nodes[node_index] = BvhNode::new_internal(bbox, left, right);

        node_index
    }

    fn compute_bbox(primitives: &[CachedPrimitive], primitive_indices: &[usize]) -> BoundingBox {
        primitive_indices.iter().fold(
            BoundingBox::new(Vector::INFINITY, Vector::NEG_INFINITY),
            |mut acc, &index| {
                acc.union_mut(&primitives[index].bounding_box);
                acc
            },
        )
    }

    pub fn intersect_ray(&self, ray: &Ray, t_min: f64, t_max: f64) -> Vec<usize> {
        let mut hits = Vec::new();
        if self.nodes.is_empty() {
            return hits;
        }

        let mut stack: Vec<u32> = Vec::with_capacity(64);
        stack.push(0);

        while let Some(node_index) = stack.pop() {
            let node = &self.nodes[node_index as usize];

            if !node.bounding_box().is_intersecting(ray, t_min, t_max) {
                continue;
            }

            match node {
                BvhNode::Leaf { start, end, .. } => {
                    for i in *start..*end {
                        hits.push(self.primitive_indices[i as usize]);
                    }
                }
                BvhNode::Internal { left, right, .. } => {
                    stack.push(*right);
                    stack.push(*left);
                }
            }
        }

        hits
    }

    fn find_split(
        primitives: &[CachedPrimitive],
        primitive_indices: &mut [usize],
        start: usize,
        end: usize,
        bbox: &BoundingBox,
    ) -> usize {
        let diag = bbox.diagonal();
        let axis = diag.axis();

        let mid = bbox.min[axis] + diag[axis] * 0.5;
        let mut left = start;
        let mut right = end - 1;

        while left < right {
            let pri_idx = primitive_indices[left];
            let center = primitives[pri_idx].center[axis];

            if center < mid {
                left += 1;
            } else {
                primitive_indices.swap(left, right);
                right -= 1;
            }
        }

        left.max(start + 1).min(end - 1)
    }

    fn find_sah_split(
        cached: &[CachedPrimitive],
        primitive_indices: &mut [usize],
        start: usize,
        end: usize,
        bbox: &BoundingBox,
    ) -> Option<usize> {
        const TRAVERSAL_COST: f64 = 1.0;
        const INTERSECTION_COST: f64 = 1.0;
        const EPS: f64 = 1e-12;

        let primitive_count = end - start;

        if primitive_count <= 2 {
            return Some(start + primitive_count / 2);
        }

        let axis = bbox.diagonal().axis();

        let mut centroid_min = f64::INFINITY;
        let mut centroid_max = f64::NEG_INFINITY;

        for &primitive_index in &primitive_indices[start..end] {
            let center = cached[primitive_index].center[axis];

            centroid_min = centroid_min.min(center);
            centroid_max = centroid_max.max(center);
        }

        let centroid_extent = centroid_max - centroid_min;

        if centroid_extent.abs() < EPS {
            return None;
        }

        let mut buckets: [Bucket; Bucket::COUNT] = std::array::from_fn(|_| Bucket::new());

        for &primitive_index in &primitive_indices[start..end] {
            let center = cached[primitive_index].center[axis];

            let mut bucket_index =
                (((center - centroid_min) / centroid_extent) * Bucket::COUNT as f64) as usize;

            if bucket_index >= Bucket::COUNT {
                bucket_index = Bucket::COUNT - 1;
            }

            buckets[bucket_index].add(&cached[primitive_index].bounding_box);
        }

        let mut left_count = [0usize; Bucket::COUNT];

        let mut left_bbox: [BoundingBox; Bucket::COUNT] =
            std::array::from_fn(|_| BoundingBox::EMPTY);

        let mut accumulated_count = 0usize;

        let mut accumulated_bbox = BoundingBox::EMPTY;

        for i in 0..Bucket::COUNT {
            accumulated_count += buckets[i].count;

            left_count[i] = accumulated_count;

            accumulated_bbox.union_mut(&buckets[i].bbox);

            left_bbox[i] = accumulated_bbox.clone();
        }

        let mut right_count = [0usize; Bucket::COUNT];

        let mut right_bbox: [BoundingBox; Bucket::COUNT] =
            std::array::from_fn(|_| BoundingBox::new(Vector::INFINITY, Vector::NEG_INFINITY));

        accumulated_count = 0;

        accumulated_bbox = BoundingBox::new(Vector::INFINITY, Vector::NEG_INFINITY);

        for i in (0..Bucket::COUNT).rev() {
            accumulated_count += buckets[i].count;

            right_count[i] = accumulated_count;

            accumulated_bbox.union_mut(&buckets[i].bbox);

            right_bbox[i] = accumulated_bbox.clone();
        }

        let total_area = bbox.surface_area();

        if total_area <= EPS {
            return None;
        }

        let leaf_cost = INTERSECTION_COST * primitive_count as f64;

        let mut best_cost = f64::INFINITY;
        let mut best_bucket = None;

        for i in 0..Bucket::COUNT - 1 {
            let left_primitive_count = left_count[i];
            let right_primitive_count = right_count[i + 1];

            if left_primitive_count == 0 || right_primitive_count == 0 {
                continue;
            }

            let left_area = left_bbox[i].surface_area();
            let right_area = right_bbox[i + 1].surface_area();

            let cost = TRAVERSAL_COST
                + INTERSECTION_COST
                    * ((left_area / total_area) * left_primitive_count as f64
                        + (right_area / total_area) * right_primitive_count as f64);

            if cost < best_cost {
                best_cost = cost;
                best_bucket = Some(i);
            }
        }

        if best_bucket.is_none() || best_cost >= leaf_cost {
            return None;
        }

        let split_plane = centroid_min
            + centroid_extent * ((best_bucket.unwrap() + 1) as f64 / Bucket::COUNT as f64);

        let mut left = start;
        let mut right = end - 1;

        while left < right {
            let primitive_index = primitive_indices[left];

            let center = cached[primitive_index].center[axis];

            if center <= split_plane {
                left += 1;
            } else {
                primitive_indices.swap(left, right);
                right -= 1;
            }
        }

        let split = left.max(start + 1).min(end - 1);

        if split == start || split == end {
            None
        } else {
            Some(split)
        }
    }
}

impl Boundable for Bvh {
    fn bounding_box(&self) -> BoundingBox {
        self.nodes[0].bounding_box()
    }
}
