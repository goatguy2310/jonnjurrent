use std::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

#[repr(C)]
#[derive(Debug, Clone)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector {
    pub const ZERO: Self = Self::splat(0.0);
    pub const ONE: Self = Self::splat(1.0);

    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    pub const Z: Self = Self::new(0.0, 0.0, 1.0);

    pub const NEG_X: Self = Self::new(-1.0, 0.0, 0.0);
    pub const NEG_Y: Self = Self::new(0.0, -1.0, 0.0);
    pub const NEG_Z: Self = Self::new(0.0, 0.0, -1.0);

    pub const AXES: [Self; 3] = [Self::X, Self::Y, Self::Z];

    pub const MIN: Self = Self::splat(f64::MIN);
    pub const MAX: Self = Self::splat(f64::MAX);

    pub const NAN: Self = Self::splat(f64::NAN);
    pub const INFINITY: Self = Self::splat(f64::INFINITY);
    pub const NEG_INFINITY: Self = Self::splat(f64::NEG_INFINITY);

    #[inline(always)]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    #[must_use]
    pub const fn splat(v: f64) -> Self {
        Self { x: v, y: v, z: v }
    }

    #[inline]
    #[must_use]
    pub const fn norm2(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline]
    #[must_use]
    pub fn norm(&self) -> f64 {
        self.norm2().sqrt()
    }

    #[inline]
    #[must_use]
    pub fn normalize(self) -> Self {
        let n = self.norm().recip();
        self.mul(n)
    }

    #[inline]
    #[must_use]
    pub const fn dot(&self, rhs: &Self) -> f64 {
        (self.x * rhs.x) + (self.y * rhs.y) + (self.z * rhs.z)
    }

    #[inline]
    #[must_use]
    pub const fn cross(&self, rhs: &Self) -> Self {
        Self {
            x: self.y * rhs.z - rhs.y * self.z,
            y: self.z * rhs.x - rhs.z * self.x,
            z: self.x * rhs.y - rhs.x * self.y,
        }
    }

    #[inline]
    #[must_use]
    pub fn random(min: f64, max: f64) -> Self {
        Self {
            x: min + (max - min) * fastrand::f64(),
            y: min + (max - min) * fastrand::f64(),
            z: min + (max - min) * fastrand::f64(),
        }
    }

    #[inline]
    #[must_use]
    pub fn random_unit() -> Self {
        loop {
            let p = Vector::random(-1.0, 1.0);
            let p_length2 = p.norm2();
            if f64::EPSILON < p_length2 && p_length2 < 1. {
                return p.normalize();
            }
        }
    }

    #[inline]
    #[must_use]
    pub const fn axis(&self) -> usize {
        let ax = self.x.abs();
        let ay = self.y.abs();
        let az = self.z.abs();

        if ax > ay && ax > az {
            0
        } else if ay > az {
            1
        } else {
            2
        }
    }

    #[inline]
    #[must_use]
    pub fn reflect(&self, normal: &Vector) -> Vector {
        self - 2. * self.dot(normal) * normal
    }

    #[inline]
    #[must_use]
    pub const fn maximum_component(&self) -> f64 {
        self.x.max(self.y).max(self.z)
    }

    #[inline]
    #[must_use]
    pub const fn minimum_component(&self) -> f64 {
        self.x.min(self.y).min(self.z)
    }

    #[inline]
    #[must_use]
    pub const fn supremum(&self, other: &Self) -> Self {
        Self::new(
            self.x.max(other.x),
            self.y.max(other.y),
            self.z.max(other.z),
        )
    }

    #[inline]
    #[must_use]
    pub const fn infimum(&self, other: &Self) -> Self {
        Self::new(
            self.x.min(other.x),
            self.y.min(other.y),
            self.z.min(other.z),
        )
    }
}

impl Default for Vector {
    #[inline(always)]
    fn default() -> Self {
        Self::ZERO
    }
}

impl Add for Vector {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::Output::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Add<&Self> for Vector {
    type Output = Self;
    #[inline]
    fn add(self, rhs: &Self) -> Self::Output {
        Self::Output::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Add<Vector> for &Vector {
    type Output = Vector;
    #[inline]
    fn add(self, rhs: Vector) -> Self::Output {
        Self::Output::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Add for &Vector {
    type Output = Vector;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::Output::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vector {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x.add_assign(rhs.x);
        self.y.add_assign(rhs.y);
        self.z.add_assign(rhs.z);
    }
}

impl AddAssign<&Self> for Vector {
    #[inline]
    fn add_assign(&mut self, rhs: &Self) {
        self.x.add_assign(rhs.x);
        self.y.add_assign(rhs.y);
        self.z.add_assign(rhs.z);
    }
}

impl Sub for Vector {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::Output::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Sub<&Self> for Vector {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: &Self) -> Self::Output {
        Self::Output::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Sub<Vector> for &Vector {
    type Output = Vector;
    #[inline]
    fn sub(self, rhs: Vector) -> Self::Output {
        Self::Output::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Sub for &Vector {
    type Output = Vector;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::Output::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vector {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x.sub_assign(rhs.x);
        self.y.sub_assign(rhs.y);
        self.z.sub_assign(rhs.z);
    }
}

impl SubAssign<&Self> for Vector {
    #[inline]
    fn sub_assign(&mut self, rhs: &Self) {
        self.x.sub_assign(rhs.x);
        self.y.sub_assign(rhs.y);
        self.z.sub_assign(rhs.z);
    }
}

impl Mul for Vector {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self::Output::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl Mul<&Self> for Vector {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: &Self) -> Self::Output {
        Self::Output::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl Mul<Vector> for &Vector {
    type Output = Vector;
    #[inline]
    fn mul(self, rhs: Vector) -> Self::Output {
        Self::Output::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl Mul for &Vector {
    type Output = Vector;
    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self::Output::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }
}

impl MulAssign for Vector {
    fn mul_assign(&mut self, rhs: Self) {
        self.x.mul_assign(rhs.x);
        self.y.mul_assign(rhs.y);
        self.z.mul_assign(rhs.z);
    }
}

impl MulAssign<&Self> for Vector {
    fn mul_assign(&mut self, rhs: &Self) {
        self.x.mul_assign(rhs.x);
        self.y.mul_assign(rhs.y);
        self.z.mul_assign(rhs.z);
    }
}

impl MulAssign<Vector> for &mut Vector {
    fn mul_assign(&mut self, rhs: Vector) {
        self.x.mul_assign(rhs.x);
        self.y.mul_assign(rhs.y);
        self.z.mul_assign(rhs.z);
    }
}

impl MulAssign for &mut Vector {
    fn mul_assign(&mut self, rhs: Self) {
        self.x.mul_assign(rhs.x);
        self.y.mul_assign(rhs.y);
        self.z.mul_assign(rhs.z);
    }
}

impl Mul<Vector> for f64 {
    type Output = Vector;
    #[inline]
    fn mul(self, rhs: Vector) -> Self::Output {
        Self::Output::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

impl Mul<&Vector> for f64 {
    type Output = Vector;
    #[inline]
    fn mul(self, rhs: &Vector) -> Self::Output {
        Self::Output::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

impl Mul<f64> for Vector {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Self::Output::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<f64> for &Vector {
    type Output = Vector;
    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Self::Output::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl MulAssign<f64> for Vector {
    fn mul_assign(&mut self, rhs: f64) {
        self.x.mul_assign(rhs);
        self.y.mul_assign(rhs);
        self.z.mul_assign(rhs);
    }
}

impl MulAssign<f64> for &mut Vector {
    fn mul_assign(&mut self, rhs: f64) {
        self.x.mul_assign(rhs);
        self.y.mul_assign(rhs);
        self.z.mul_assign(rhs);
    }
}

impl Div<f64> for Vector {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Self::Output::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl Div<f64> for &Vector {
    type Output = Vector;
    #[inline]
    fn div(self, rhs: f64) -> Self::Output {
        Self::Output::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl DivAssign<f64> for Vector {
    fn div_assign(&mut self, rhs: f64) {
        self.x.div_assign(rhs);
        self.y.div_assign(rhs);
        self.z.div_assign(rhs);
    }
}

impl DivAssign<f64> for &mut Vector {
    fn div_assign(&mut self, rhs: f64) {
        self.x.div_assign(rhs);
        self.y.div_assign(rhs);
        self.z.div_assign(rhs);
    }
}

impl Neg for Vector {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::Output::new(-self.x, -self.y, -self.z)
    }
}

impl Neg for &Vector {
    type Output = Vector;

    fn neg(self) -> Self::Output {
        Self::Output::new(-self.x, -self.y, -self.z)
    }
}

impl Index<usize> for Vector {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!(),
        }
    }
}

impl IndexMut<usize> for Vector {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!(),
        }
    }
}

impl From<[f64; 3]> for Vector {
    #[inline]
    fn from(value: [f64; 3]) -> Self {
        Self::new(value[0], value[1], value[2])
    }
}

impl From<Vector> for [f64; 3] {
    #[inline]
    fn from(value: Vector) -> Self {
        [value.x, value.y, value.z]
    }
}

impl From<(f64, f64, f64)> for Vector {
    #[inline]
    fn from(value: (f64, f64, f64)) -> Self {
        Self::new(value.0, value.1, value.2)
    }
}

impl From<Vector> for (f64, f64, f64) {
    #[inline]
    fn from(value: Vector) -> Self {
        (value.x, value.y, value.z)
    }
}
