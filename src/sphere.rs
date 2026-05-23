use crate::{
    material::MaterialIndex,
    object::{ComputeIntersection, Intersection, Object, Sampling},
    ray::Ray,
    vector::Vector,
};

#[derive(Debug)]
pub struct Sphere {
    center: Ray,
    radius: f64,

    material_index: MaterialIndex,
}

impl ComputeIntersection for Sphere {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        let current_center = self.center.at(ray.time);
        let origin_center = &ray.origin - &current_center;
        let u_dot_origin_center = ray.direction.dot(&origin_center);

        let delta = u_dot_origin_center.powi(2) - (origin_center.norm2() - self.radius.powi(2));

        if delta < 0. {
            return None;
        }

        let delta_sqrt = delta.sqrt();
        let t_1 = -u_dot_origin_center - delta_sqrt;
        let t_2 = -u_dot_origin_center + delta_sqrt;

        if t_2 < 0. {
            return None;
        }

        let t = if t_1 >= 0. { t_1 } else { t_2 };

        let intersection_point = ray.at(t);
        let intersection_to_center = &intersection_point - current_center;
        let unit_normal = intersection_to_center.clone() / intersection_to_center.norm();

        Some(Intersection::new(
            intersection_point,
            t,
            unit_normal,
            self.material_index,
        ))
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        let current_center = self.center.at(ray.time);
        let origin_center = &ray.origin - &current_center;
        let u_dot_origin_center = ray.direction.dot(&origin_center);

        let delta = u_dot_origin_center.powi(2) - (origin_center.norm2() - self.radius.powi(2));

        if delta < 0. {
            return None;
        }

        let delta_sqrt = delta.sqrt();
        let t_1 = -u_dot_origin_center - delta_sqrt;
        let t_2 = -u_dot_origin_center + delta_sqrt;

        if t_2 < 0. {
            return None;
        }

        let t = if t_1 >= 0. { t_1 } else { t_2 };

        Some(t)
    }
}

impl Sampling for Sphere {
    fn sample(&self) -> (Vector, Vector) {
        let dir = Vector::random_unit();
        let center = self.center.origin.clone();
        let point = &center + self.radius * &dir;

        (point, dir)
    }
}

#[derive(Debug)]
pub struct SphereBuilder {
    center: Ray,
    radius: f64,

    material_index: MaterialIndex,
}

impl SphereBuilder {
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            center: Ray::default(),
            radius: 0.,
            material_index: MaterialIndex(0),
        }
    }

    #[inline]
    #[must_use]
    pub const fn build(self) -> Object {
        Object::Sphere(Sphere {
            center: self.center,
            radius: self.radius,
            material_index: self.material_index,
        })
    }

    #[inline]
    #[must_use]
    pub const fn center(mut self, x: f64, y: f64, z: f64) -> Self {
        self.center.origin.x = x;
        self.center.origin.y = y;
        self.center.origin.z = z;
        self
    }

    #[inline]
    #[must_use]
    pub const fn radius(mut self, radius: f64) -> Self {
        self.radius = radius;
        self
    }

    #[inline]
    #[must_use]
    pub const fn final_position(mut self, x: f64, y: f64, z: f64) -> Self {
        self.center.direction.x = x;
        self.center.direction.y = y;
        self.center.direction.z = z;
        self
    }

    #[inline]
    #[must_use]
    pub const fn material(mut self, index: MaterialIndex) -> Self {
        self.material_index = index;
        self
    }
}
