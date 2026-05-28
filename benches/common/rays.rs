use renderer::{
    larp::BoundingBox,
    math::{Ray, Vector, VolumeSampleable},
};

pub fn rays_into_bbox(bbox: BoundingBox, count: usize) -> Vec<Ray> {
    fastrand::seed(42);

    let center = &bbox.center();
    let diagonal = bbox.diagonal();

    let radius = diagonal.norm().max(1e-6) * 2.0;

    (0..count)
        .map(|_| {
            let origin = center + Vector::random_unit() * radius;
            let target = bbox.volume_sample();
            let direction = (target - &origin).normalize();

            Ray::new(origin, direction)
        })
        .collect()
}
