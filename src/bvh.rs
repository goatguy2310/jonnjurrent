use crate::{ray::Ray, vector::Vector};

#[derive(Debug, Clone)]
pub struct BoundingBox {
    min: Vector,
    max: Vector,
}

impl BoundingBox {
    pub fn new(min: Vector, max: Vector) -> Self {
        Self { min, max }
    }

    pub fn update(&mut self, min: Vector, max: Vector) {
        self.min = min;
        self.max = max;
    }

    pub fn is_intersecting(&self, ray: &Ray, t_min: f64, t_max: f64) -> bool {
        let mut t_min = t_min;
        let mut t_max = t_max;

        for i in 0..3 {
            let origin = ray.origin[i];
            let dir = ray.direction[i];
            let inv_d = dir.recip();

            let mut t0 = (self.min[i] - origin) * inv_d;
            let mut t1 = (self.max[i] - origin) * inv_d;

            if inv_d.is_sign_negative() {
                core::mem::swap(&mut t0, &mut t1);
            }

            t_min = t_min.max(t0);
            t_max = t_max.min(t1);

            if t_max <= t_min {
                return false;
            }
        }

        true
    }
}

// #[derive(Debug, Clone)]
// pub struct Bvh {}
