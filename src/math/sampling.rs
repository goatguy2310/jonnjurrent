use crate::math::Vector;

#[allow(unused)]
pub trait SurfaceSampleable {
    fn sample(&self) -> (Vector, Vector);
}

pub trait VolumeSampleable {
    fn volume_sample(&self) -> Vector;
}
