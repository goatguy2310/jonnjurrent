use crate::math::Vector;

/// TODO: Move to math
#[allow(unused)]
pub trait Sampleable {
    fn sample(&self) -> (Vector, Vector);
}
