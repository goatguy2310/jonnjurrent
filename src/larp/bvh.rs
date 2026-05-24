use crate::{
    geometry::TriangleSoup,
    larp::BoundingBox,
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
    pub fn bbox(&self) -> &BoundingBox {
        match self {
            Self::Internal { bbox, .. } => bbox,
            Self::Leaf { bbox, .. } => bbox,
        }
    }

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

#[derive(Debug, Clone)]
pub struct BvhTree {
    nodes: Vec<BvhNode>,
    triangle_indices: Vec<usize>,
}

impl BvhTree {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            triangle_indices: Vec::new(),
        }
    }

    pub fn build<C: BVHConfig>(mesh: &Vec<TriangleSoup>) -> Self {
        let triangle_count = mesh.len();
        let mut triangle_indices: Vec<usize> = (0..triangle_count).collect();
        let mut nodes = Vec::with_capacity(2 * triangle_count);

        Self::build_recursive::<C>(mesh, &mut triangle_indices, 0, triangle_count, &mut nodes);

        BvhTree {
            nodes,
            triangle_indices,
        }
    }

    fn build_recursive<C: BVHConfig>(
        mesh: &Vec<TriangleSoup>,
        triangle_indices: &mut [usize],
        start: usize,
        end: usize,
        nodes: &mut Vec<BvhNode>,
    ) -> usize {
        let bbox = Self::compute_bbox(mesh, &triangle_indices[start..end]);

        if end - start <= 4 {
            let node_index = nodes.len();
            nodes.push(BvhNode::new_leaf(bbox, start, end));
            return node_index;
        }

        let node_index = nodes.len();
        nodes.push(BvhNode::Internal {
            bbox: bbox.clone(),
            left: 0,
            right: 0,
        });

        let split_position = if C::USE_SAH {
            Self::find_sah_split(mesh, triangle_indices, start, end, &bbox)
        } else {
            Self::find_split(mesh, triangle_indices, start, end, &bbox)
        };

        if split_position == start || split_position == end {
            nodes[node_index] = BvhNode::new_leaf(bbox, start, end);
            return node_index;
        }

        let left = Self::build_recursive::<C>(mesh, triangle_indices, start, split_position, nodes);
        let right = Self::build_recursive::<C>(mesh, triangle_indices, split_position, end, nodes);

        nodes[node_index] = BvhNode::new_internal(bbox, left, right);

        node_index
    }

    fn compute_bbox(mesh: &[TriangleSoup], triangle_indices: &[usize]) -> BoundingBox {
        let mut min = Vector::INFINITY;
        let mut max = Vector::NEG_INFINITY;

        for v in triangle_indices
            .iter()
            .flat_map(|&index| mesh[index].vtx.iter())
        {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);

            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }

        BoundingBox::new(min, max)
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

            if !node.bbox().is_intersecting(ray, t_min, t_max) {
                continue;
            }

            match node {
                BvhNode::Leaf { start, end, .. } => {
                    for i in *start..*end {
                        hits.push(self.triangle_indices[i as usize]);
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
        mesh: &[TriangleSoup],
        triangle_indices: &mut [usize],
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
            let tri_idx = triangle_indices[left];
            let center = mesh[tri_idx].center()[axis];

            if center < mid {
                left += 1;
            } else {
                triangle_indices.swap(left, right);
                right -= 1;
            }
        }

        left.max(start + 1).min(end - 1)
    }

    /// ChatGPT generated
    fn find_sah_split(
        mesh: &[TriangleSoup],
        triangle_indices: &mut [usize],
        start: usize,
        end: usize,
        bbox: &BoundingBox,
    ) -> usize {
        const BUCKET_COUNT: usize = 12;
        let n = end - start;
        if n <= 2 {
            return start + n / 2;
        }

        let diag = bbox.diagonal();
        let axis = diag.axis();

        let mut centroids = Vec::with_capacity(n);
        let mut cmin = f64::INFINITY;
        let mut cmax = f64::NEG_INFINITY;
        for &i in &triangle_indices[start..end] {
            let c = mesh[i].center()[axis];
            centroids.push((i, c));
            cmin = cmin.min(c);
            cmax = cmax.max(c);
        }

        if (cmax - cmin).abs() < 1e-12 {
            return start + n / 2;
        }

        #[derive(Clone)]
        struct Bucket {
            count: usize,
            bbox: Option<BoundingBox>,
        }
        let mut buckets = vec![
            Bucket {
                count: 0,
                bbox: None
            };
            BUCKET_COUNT
        ];

        for (tri_idx, c) in &centroids {
            let mut b = (((*c - cmin) / (cmax - cmin)) * (BUCKET_COUNT as f64)) as usize;
            if b == BUCKET_COUNT {
                b = BUCKET_COUNT - 1;
            }

            let bucket = &mut buckets[b];
            bucket.count += 1;
            let tri_bbox = Self::compute_bbox(mesh, &[*tri_idx]);
            bucket.bbox = match &bucket.bbox {
                Some(existing) => Some(BoundingBox::new(
                    Vector::new(
                        existing.min.x.min(tri_bbox.min.x),
                        existing.min.y.min(tri_bbox.min.y),
                        existing.min.z.min(tri_bbox.min.z),
                    ),
                    Vector::new(
                        existing.max.x.max(tri_bbox.max.x),
                        existing.max.y.max(tri_bbox.max.y),
                        existing.max.z.max(tri_bbox.max.z),
                    ),
                )),
                None => Some(tri_bbox),
            };
        }

        let mut left_count = [0usize; BUCKET_COUNT];
        let mut left_bbox: Vec<Option<BoundingBox>> = vec![None; BUCKET_COUNT];
        let mut count = 0usize;
        let mut bbox_acc: Option<BoundingBox> = None;
        for i in 0..BUCKET_COUNT {
            count += buckets[i].count;
            left_count[i] = count;
            bbox_acc = match (&bbox_acc, &buckets[i].bbox) {
                (None, None) => None,
                (Some(b1), None) => Some(b1.clone()),
                (None, Some(b2)) => Some(b2.clone()),
                (Some(b1), Some(b2)) => Some(BoundingBox::new(
                    Vector::new(
                        b1.min.x.min(b2.min.x),
                        b1.min.y.min(b2.min.y),
                        b1.min.z.min(b2.min.z),
                    ),
                    Vector::new(
                        b1.max.x.max(b2.max.x),
                        b1.max.y.max(b2.max.y),
                        b1.max.z.max(b2.max.z),
                    ),
                )),
            };
            left_bbox[i] = bbox_acc.clone();
        }

        let mut right_count = [0usize; BUCKET_COUNT];
        let mut right_bbox: Vec<Option<BoundingBox>> = vec![None; BUCKET_COUNT];
        count = 0;
        bbox_acc = None;
        for i in (0..BUCKET_COUNT).rev() {
            count += buckets[i].count;
            right_count[i] = count;
            bbox_acc = match (&bbox_acc, &buckets[i].bbox) {
                (None, None) => None,
                (Some(b1), None) => Some(b1.clone()),
                (None, Some(b2)) => Some(b2.clone()),
                (Some(b1), Some(b2)) => Some(BoundingBox::new(
                    Vector::new(
                        b1.min.x.min(b2.min.x),
                        b1.min.y.min(b2.min.y),
                        b1.min.z.min(b2.min.z),
                    ),
                    Vector::new(
                        b1.max.x.max(b2.max.x),
                        b1.max.y.max(b2.max.y),
                        b1.max.z.max(b2.max.z),
                    ),
                )),
            };
            right_bbox[i] = bbox_acc.clone();
        }

        let surface_area = |b: &BoundingBox| {
            let d = &b.max - &b.min;
            2.0 * (d.x * d.y + d.y * d.z + d.z * d.x)
        };

        let mut best_cost = f64::INFINITY;
        let mut best_bucket_split = 0usize;
        let total_area = surface_area(bbox);
        for i in 0..BUCKET_COUNT - 1 {
            let left_n = left_count[i] as f64;
            let right_n = right_count[i + 1] as f64;
            if left_n == 0.0 || right_n == 0.0 {
                continue;
            }
            let left_area = left_bbox[i].as_ref().map(&surface_area).unwrap_or(0.0);
            let right_area = right_bbox[i + 1].as_ref().map(&surface_area).unwrap_or(0.0);

            // cost = traversal_cost + intersection_cost * ( (left_area/total_area)*left_n + (right_area/total_area)*right_n )
            // use constants (t_trav = 1.0, t_isect = 1.0)
            let cost =
                1.0 + (left_area / total_area) * left_n + (right_area / total_area) * right_n;
            if cost < best_cost {
                best_cost = cost;
                best_bucket_split = i;
            }
        }

        // If no beneficial split found, fallback to median-by-count
        if best_cost.is_infinite() {
            return start + n / 2;
        }

        // Partition triangle_indices by bucket split (stable partitioning by recomputing bucket)
        let split_centroid =
            cmin + (cmax - cmin) * ((best_bucket_split as f64 + 1.0) / BUCKET_COUNT as f64);
        let mut l = start;
        let mut r = end - 1;
        while l <= r {
            let tri_idx = triangle_indices[l];
            let c = mesh[tri_idx].center()[axis];
            if c <= split_centroid {
                l += 1;
            } else {
                triangle_indices.swap(l, r);
                if r == 0 {
                    break;
                } // safety
                r -= 1;
            }
        }

        l.max(start + 1).min(end - 1)
    }
}

pub trait BVHConfig {
    const USE_SAH: bool;
}
