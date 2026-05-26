use crate::math::Vector;

#[derive(Debug)]
pub struct Ray {
    pub origin: Vector,
    pub direction: Vector,
}

impl Ray {
    #[inline(always)]
    #[must_use]
    pub const fn new(origin: Vector, direction: Vector) -> Self {
        Self { origin, direction }
    }

    #[inline]
    #[must_use]
    pub fn at(&self, t: f64) -> Vector {
        &self.origin + t * &self.direction
    }

    #[inline]
    pub const fn update_direction(&mut self, x: f64, y: f64, z: f64) {
        self.direction.x = x;
        self.direction.y = y;
        self.direction.z = z;
    }
}

impl Default for Ray {
    #[inline(always)]
    fn default() -> Self {
        Self {
            origin: Vector::ZERO,
            direction: Vector::ZERO,
        }
    }
}
