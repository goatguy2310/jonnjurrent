use crate::material::MaterialIndex;
use crate::{mesh::TriangleMesh, quad::Quad, sphere::Sphere};
use crate::{ray::Ray, vector::Vector};

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

#[allow(unused)]
pub trait Sampling {
    fn sample(&self) -> (Vector, Vector);
}

#[derive(Debug)]
pub enum Object {
    Sphere(Sphere),
    Quad(Quad),
    TriangleMesh(TriangleMesh),
}

impl Object {}

impl ComputeIntersection for Object {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        match self {
            Object::Sphere(sphere) => sphere.intersect(ray),
            Object::Quad(quad) => quad.intersect(ray),
            Object::TriangleMesh(mesh) => mesh.intersect(ray),
        }
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        match self {
            Object::Sphere(sphere) => sphere.shadow_intersect(ray),
            Object::Quad(quad) => quad.shadow_intersect(ray),
            Object::TriangleMesh(mesh) => mesh.shadow_intersect(ray),
        }
    }
}

impl Sampling for Object {
    fn sample(&self) -> (Vector, Vector) {
        match self {
            Object::Sphere(sphere) => sphere.sample(),
            Object::Quad(quad) => quad.sample(),
            Object::TriangleMesh(_) => todo!(),
        }
    }
}
