use crate::math::{Ray, Vector};

#[derive(Debug, Clone)]
pub struct BoundingBox {
    pub min: Vector,
    pub max: Vector,
}

impl BoundingBox {
    #[inline(always)]
    #[must_use]
    pub const fn new(min: Vector, max: Vector) -> Self {
        Self { min, max }
    }

    #[inline]
    #[must_use]
    pub fn diagonal(&self) -> Vector {
        &self.max - &self.min
    }

    #[inline]
    #[must_use]
    pub fn is_intersecting(&self, ray: &Ray, mut t_min: f64, mut t_max: f64) -> bool {
        for i in 0..3 {
            let origin = ray.origin[i];
            let dir = ray.direction[i];

            let (t0, t1) = if dir.abs() < f64::EPSILON {
                if origin < self.min[i] || origin > self.max[i] {
                    return false;
                }
                (f64::NEG_INFINITY, f64::INFINITY)
            } else {
                let inv = 1.0 / dir;
                let mut t0 = (self.min[i] - origin) * inv;
                let mut t1 = (self.max[i] - origin) * inv;
                if t0 > t1 {
                    core::mem::swap(&mut t0, &mut t1);
                }
                (t0, t1)
            };

            t_min = t_min.max(t0);
            t_max = t_max.min(t1);

            if t_max < t_min {
                return false;
            }
        }

        true
    }
}
