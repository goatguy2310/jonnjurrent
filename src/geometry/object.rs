use crate::geometry::Sampleable;
use crate::geometry::{ComputeIntersection, Intersection};
use crate::geometry::{Quad, Sphere, TriangleMesh};
use crate::material::MaterialIndex;
use crate::math::{Ray, Vector};

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

impl Sampleable for Object {
    fn sample(&self) -> (Vector, Vector) {
        match self {
            Object::Sphere(sphere) => sphere.sample(),
            Object::Quad(quad) => quad.sample(),
            Object::TriangleMesh(_) => todo!(),
        }
    }
}
