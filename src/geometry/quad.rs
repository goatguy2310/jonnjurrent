use crate::{
    geometry::{ComputeIntersection, Intersection, Object, Sampleable},
    material::MaterialIndex,
    math::{Ray, Vector},
};

const EPS: f64 = 1e-4;

#[derive(Debug)]
pub struct Quad {
    corner: Vector,
    u: Vector,
    v: Vector,

    material_index: MaterialIndex,
}

impl ComputeIntersection for Quad {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        let a_minus_o = &self.corner - &ray.origin;
        let n = self.u.cross(&self.v);

        let denom = ray.direction.dot(&n);
        if denom.abs() <= EPS {
            return None;
        }

        let inv_denom = 1. / denom;
        let t = a_minus_o.dot(&n) * inv_denom;

        if t <= EPS {
            return None;
        }

        let intersection = ray.at(t);
        let hit_point = &intersection - &self.corner;

        let w = &n / n.dot(&n);
        let alpha = w.dot(&hit_point.cross(&self.v));
        let beta = w.dot(&self.u.cross(&hit_point));

        if !(-EPS..=1. + EPS).contains(&alpha) || !(-EPS..=1. + EPS).contains(&beta) {
            return None;
        }

        Some(Intersection::new(
            intersection,
            t,
            n.normalize(),
            (alpha, beta),
            self.material_index,
        ))
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        let a_minus_o = &self.corner - &ray.origin;
        let n = self.u.cross(&self.v);

        let denom = ray.direction.dot(&n);
        if denom.abs() <= EPS {
            return None;
        }

        let inv_denom = 1. / denom;
        let t = a_minus_o.dot(&n) * inv_denom;

        if t <= EPS {
            return None;
        }

        let intersection = ray.at(t);
        let hit_point = &intersection - &self.corner;

        let w = &n / n.dot(&n);
        let alpha = w.dot(&hit_point.cross(&self.v));
        let beta = w.dot(&self.u.cross(&hit_point));

        if !(-EPS..=1. + EPS).contains(&alpha) || !(-EPS..=1. + EPS).contains(&beta) {
            return None;
        }

        Some(t)
    }
}

impl Sampleable for Quad {
    fn sample(&self) -> (Vector, Vector) {
        let u = fastrand::f64();
        let v = fastrand::f64();

        let point = &self.corner + u * &self.u + v * &self.v;
        let normal = self.u.cross(&self.v).normalize();

        (point, normal)
    }
}

impl crate::larp::Boundable for Quad {
    fn bounding_box(&self) -> crate::larp::BoundingBox {
        const PADDING: f64 = 1e-6;

        let p1 = &self.corner;
        let p2 = p1 + &self.u;
        let p3 = p1 + &self.v;
        let p4 = &p2 + &p3 - p1;

        let min = p1.infimum(&p2).infimum(&p3).infimum(&p4);
        let max = p1.supremum(&p2).supremum(&p3).supremum(&p4);

        crate::larp::BoundingBox::new(min, max).extend(PADDING)
    }
}

pub struct QuadBuilder {
    corner: Vector,
    u: Vector,
    v: Vector,

    material_index: MaterialIndex,
}

impl QuadBuilder {
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        QuadBuilder::default()
    }

    #[inline]
    #[must_use]
    pub const fn build(self) -> Object {
        Object::Quad(Quad {
            corner: self.corner,
            u: self.u,
            v: self.v,
            material_index: self.material_index,
        })
    }

    #[inline]
    #[must_use]
    pub const fn corner(mut self, x: f64, y: f64, z: f64) -> Self {
        self.corner.x = x;
        self.corner.y = y;
        self.corner.z = z;
        self
    }

    #[inline]
    #[must_use]
    pub const fn u(mut self, x: f64, y: f64, z: f64) -> Self {
        self.u.x = x;
        self.u.y = y;
        self.u.z = z;
        self
    }

    #[inline]
    #[must_use]
    pub const fn v(mut self, x: f64, y: f64, z: f64) -> Self {
        self.v.x = x;
        self.v.y = y;
        self.v.z = z;
        self
    }

    #[inline]
    #[must_use]
    pub const fn material(mut self, index: MaterialIndex) -> Self {
        self.material_index = index;
        self
    }
}

impl Default for QuadBuilder {
    fn default() -> Self {
        Self {
            corner: Vector::ZERO,
            u: Vector::Y,
            v: Vector::Z,
            material_index: MaterialIndex::default(),
        }
    }
}
