use core::f64;
use std::marker::PhantomData;

use crate::{larp::BoundingBox, math::Vector};

#[derive(Debug, Clone)]
pub struct MortonEncoder<EncodeTo> {
    min: Vector,
    inv_extent: Vector,
    phantom: PhantomData<EncodeTo>,
}

impl<EncodeTo> MortonEncoder<EncodeTo> {
    #[inline(always)]
    #[must_use]
    pub fn new(bounds: &BoundingBox) -> Self {
        let extent = bounds.diagonal();

        Self {
            min: bounds.min.clone(),
            inv_extent: extent.recip(),
            phantom: PhantomData::default(),
        }
    }
}

impl MortonEncoder<u32> {
    pub const BITS_PER_AXIS: usize = 10;
    pub const MAX_COORD: f64 = ((1 << Self::BITS_PER_AXIS) - 1) as f64;

    #[inline]
    #[must_use]
    pub fn encode_u32(&self, p: &Vector) -> u32 {
        let n = ((p - &self.min) * &self.inv_extent).clamp_scalar(0.0, 1.);
        let (x, y, z) = n
            .map(|elem| (elem * Self::MAX_COORD).round().clamp(0., Self::MAX_COORD))
            .as_u32();

        Self::morton_u32(x, y, z)
    }

    #[inline]
    #[must_use]
    fn morton_u32(x: u32, y: u32, z: u32) -> u32 {
        Self::voodoo_u32(x) | (Self::voodoo_u32(y) << 1) | (Self::voodoo_u32(z) << 2)
    }

    // Taken from Stack Overflow
    // Morton encoding for 3 10-bit into u32
    #[inline]
    #[must_use]
    fn voodoo_u32(mut v: u32) -> u32 {
        v &= 0x0000_03ff;

        v = (v | (v << 16)) & 0x0300_00ff;
        v = (v | (v << 8)) & 0x0300_f00f;
        v = (v | (v << 4)) & 0x030c_30c3;
        v = (v | (v << 2)) & 0x0924_9249;

        v
    }
}

impl MortonEncoder<u64> {
    pub const BITS_PER_AXIS: usize = 21;
    pub const MAX_COORD: f64 = ((1 << Self::BITS_PER_AXIS) - 1) as f64;

    #[inline]
    #[must_use]
    pub fn encode_u64(&self, p: &Vector) -> u64 {
        let n = ((p - &self.min) * &self.inv_extent).clamp_scalar(0.0, 1.);
        let (x, y, z) = n
            .map(|elem| (elem * Self::MAX_COORD).round().clamp(0., Self::MAX_COORD))
            .as_u64();

        Self::morton_u64(x, y, z)
    }

    #[inline]
    #[must_use]
    fn morton_u64(x: u64, y: u64, z: u64) -> u64 {
        Self::voodoo_u64(x) | (Self::voodoo_u64(y) << 1) | (Self::voodoo_u64(z) << 2)
    }

    // Taken from Stack Overflow
    // Morton encoding for 3 21-bit into u64
    #[inline]
    #[must_use]
    fn voodoo_u64(mut v: u64) -> u64 {
        v &= 0x1fffff;

        v = (v | (v << 32)) & 0x01f00000000ffff;
        v = (v | (v << 16)) & 0x01f0000ff0000ff;
        v = (v | (v << 8)) & 0x100f00f00f00f00f;
        v = (v | (v << 4)) & 0x10c30c30c30c30c3;
        v = (v | (v << 2)) & 0x1249249249249249;

        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BBOX: BoundingBox = BoundingBox::new(Vector::ZERO, Vector::splat(10.));

    // ============ u32 Tests ============

    #[test]
    fn test_encode_u32_origin() {
        let encoder = MortonEncoder::<u32>::new(&BBOX);
        let origin = Vector::ZERO;

        let code = encoder.encode_u32(&origin);
        assert_eq!(code, 0, "Origin should encode to 0");
    }

    #[test]
    fn test_encode_u32_max_corner() {
        let encoder = MortonEncoder::<u32>::new(&BBOX);
        let max_point = Vector::splat(10.);

        let code = encoder.encode_u32(&max_point);
        let expected_max = (1 << (3 * MortonEncoder::<u32>::BITS_PER_AXIS)) - 1;
        assert_eq!(
            code, expected_max as u32,
            "Max corner should encode to max value"
        );
    }

    #[test]
    fn test_encode_u32_center() {
        let encoder = MortonEncoder::<u32>::new(&BBOX);
        let center = Vector::splat(5.);

        let code = encoder.encode_u32(&center);
        let max_code = (1 << (3 * MortonEncoder::<u32>::BITS_PER_AXIS)) - 1;
        assert!(
            code > 0 && code < max_code,
            "Center should encode to middle range"
        );
    }

    #[test]
    fn test_encode_u32_single_axis() {
        let encoder = MortonEncoder::<u32>::new(&BBOX);

        let p1 = Vector::X * 5.;
        let p2 = Vector::Y * 5.;
        let p3 = Vector::Z * 5.;

        let c1 = encoder.encode_u32(&p1);
        let c2 = encoder.encode_u32(&p2);
        let c3 = encoder.encode_u32(&p3);

        assert!(
            c1 != c2 && c2 != c3 && c1 != c3,
            "Different axes should produce different codes"
        );
    }

    #[test]
    fn test_encode_u32_deterministic() {
        let encoder = MortonEncoder::<u32>::new(&BBOX);
        let point = Vector::new(3.7, 2.5, 8.1);

        let code1 = encoder.encode_u32(&point);
        let code2 = encoder.encode_u32(&point);

        assert_eq!(code1, code2, "Same input should always produce same output");
    }

    #[test]
    fn test_encode_u32_in_range() {
        let encoder = MortonEncoder::<u32>::new(&BBOX);

        for i in 0..10 {
            let x = i as f64;
            let y = (9 - i) as f64;
            let point = Vector::new(x, y, 5.0);

            let code = encoder.encode_u32(&point);
            let max_code = (1u64 << (3 * MortonEncoder::<u32>::BITS_PER_AXIS)) - 1;
            assert!(code as u64 <= max_code, "Code should fit in allocated bits");
        }
    }

    // ============ u64 Tests ============

    #[test]
    fn test_encode_u64_origin() {
        let encoder = MortonEncoder::<u64>::new(&BBOX);
        let origin = Vector::ZERO;

        let code = encoder.encode_u64(&origin);
        assert_eq!(code, 0, "Origin should encode to 0");
    }

    #[test]
    fn test_encode_u64_max_corner() {
        let encoder = MortonEncoder::<u64>::new(&BBOX);
        let max_point = Vector::splat(10.);

        let code = encoder.encode_u64(&max_point);
        let expected_max = (1u128 << (3 * MortonEncoder::<u64>::BITS_PER_AXIS)) - 1;
        assert_eq!(
            code, expected_max as u64,
            "Max corner should encode to max value"
        );
    }

    #[test]
    fn test_encode_u64_center() {
        let encoder = MortonEncoder::<u64>::new(&BBOX);
        let center = Vector::splat(5.);

        let code = encoder.encode_u64(&center);
        let max_code = (1u128 << (3 * MortonEncoder::<u64>::BITS_PER_AXIS)) - 1;
        assert!(
            code > 0 && code < (max_code as u64),
            "Center should encode to middle range"
        );
    }

    #[test]
    fn test_encode_u64_deterministic() {
        let encoder = MortonEncoder::<u64>::new(&BBOX);
        let point = Vector::new(3.7, 2.5, 8.1);

        let code1 = encoder.encode_u64(&point);
        let code2 = encoder.encode_u64(&point);

        assert_eq!(code1, code2, "Same input should always produce same output");
    }

    // ============ Edge Cases ============

    #[test]
    fn test_non_origin_bounds() {
        let min = Vector::splat(5.);
        let max = Vector::splat(15.);

        let bounds = BoundingBox::new(min.clone(), max.clone());
        let encoder = MortonEncoder::<u32>::new(&bounds);

        let code_min = encoder.encode_u32(&min);
        assert_eq!(code_min, 0, "Min corner should encode to 0");

        let code_max = encoder.encode_u32(&max);
        let expected_max = (1 << (3 * MortonEncoder::<u32>::BITS_PER_AXIS)) - 1;
        assert_eq!(
            code_max, expected_max as u32,
            "Max corner should encode to max"
        );
    }

    #[test]
    fn test_asymmetric_bounds() {
        let bounds = BoundingBox::new(Vector::ZERO, Vector::new(20., 10., 5.));
        let encoder = MortonEncoder::<u32>::new(&bounds);

        let point = Vector::new(10.0, 5.0, 2.5);
        let code = encoder.encode_u32(&point);

        let max_code = (1u64 << (3 * MortonEncoder::<u32>::BITS_PER_AXIS)) - 1;
        assert!(code as u64 <= max_code);
    }

    #[test]
    fn test_spatial_locality() {
        let encoder = MortonEncoder::<u32>::new(&BBOX);

        let p1 = Vector::splat(5.0);
        let p2 = Vector::new(5.1, 5.0, 5.0);

        let c1 = encoder.encode_u32(&p1);
        let c2 = encoder.encode_u32(&p2);

        let xor = c1 ^ c2;
        let hamming_distance = xor.count_ones();

        println!("Hamming distance: {}", hamming_distance);
        assert!(
            hamming_distance <= 20,
            "Nearby points should have small Hamming distance"
        );
    }
}
