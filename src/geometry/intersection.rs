use crate::math::{Ray, Vector};

#[derive(Debug)]
pub struct Intersection<T> {
    pub intersection: Vector,
    pub distance: f64,
    pub normal: Vector,
    pub u: f64,
    pub v: f64,
    pub index: T,
}

impl<T> Intersection<T> {
    pub fn new(
        intersection: Vector,
        distance: f64,
        normal: Vector,
        uv: (f64, f64),
        index: T,
    ) -> Self {
        Self {
            intersection,
            distance,
            normal,
            u: uv.0,
            v: uv.1,
            index,
        }
    }
}

pub trait ComputeIntersection {
    type Index;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>>;
    fn shadow_intersect(&self, ray: &Ray) -> Option<f64>;
}
