use crate::math::{Ray, Vector, VolumeSampleable};

pub trait Boundable {
    fn bounding_box(&self) -> BoundingBox;
}

#[derive(Debug, Clone)]
pub struct BoundingBox {
    pub min: Vector,
    pub max: Vector,
}

impl BoundingBox {
    pub const EMPTY: Self = Self::new(Vector::INFINITY, Vector::NEG_INFINITY);
    pub const UNIVERSE: Self = Self::new(Vector::NEG_INFINITY, Vector::INFINITY);

    #[inline(always)]
    #[must_use]
    pub const fn new(min: Vector, max: Vector) -> Self {
        Self { min, max }
    }

    #[inline]
    #[must_use]
    pub fn center(&self) -> Vector {
        (&self.min + &self.max) * 0.5
    }

    #[inline]
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        let (dx, dy, dz) = self.diagonal().into();
        2.0 * (dx * dy + dy * dz + dz * dx)
    }

    #[inline]
    #[must_use]
    pub fn diagonal(&self) -> Vector {
        &self.max - &self.min
    }

    #[inline]
    #[must_use]
    pub fn extend(self, value: f64) -> Self {
        let value_vector = Vector::splat(value);
        Self::new(self.min - &value_vector, self.max + value_vector)
    }

    #[inline]
    #[allow(unused)]
    pub fn extend_mut(&mut self, value: f64) {
        let value_vector = Vector::splat(value);
        self.min -= &value_vector;
        self.max += value_vector;
    }

    #[inline(always)]
    #[must_use]
    #[allow(unused)]
    pub const fn union(&self, other: &Self) -> Self {
        Self::new(self.min.infimum(&other.min), self.max.supremum(&other.max))
    }

    #[inline(always)]
    pub const fn union_mut(&mut self, other: &Self) {
        self.min = self.min.infimum(&other.min);
        self.max = self.max.supremum(&other.max);
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

impl VolumeSampleable for BoundingBox {
    fn volume_sample(&self) -> Vector {
        Vector::new(
            fastrand::f64() * (self.max.x - self.min.x) + self.min.x,
            fastrand::f64() * (self.max.y - self.min.y) + self.min.y,
            fastrand::f64() * (self.max.z - self.min.z) + self.min.z,
        )
    }
}
