use crate::math::Vector;

#[allow(unused)]
pub trait Sampleable {
    fn sample(&self) -> (Vector, Vector);
}
