use crate::{
    material::MaterialIndex, mesh::TriangleMesh, ray::Ray, sphere::Sphere, vector::Vector,
};

#[derive(Debug)]
pub struct Intersection<T> {
    pub intersection: Vector,
    pub distance: f64,
    pub normal: Vector,
    pub index: T,
}

impl<T> Intersection<T> {
    pub fn new(intersection: Vector, distance: f64, normal: Vector, index: T) -> Self {
        Self {
            intersection,
            distance,
            normal,
            index,
        }
    }
}

pub trait ComputeIntersection {
    type Index;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>>;
    fn shadow_intersect(&self, ray: &Ray) -> Option<f64>;
}

#[derive(Debug)]
pub enum Object {
    Sphere(Sphere),
    TriangleMesh(TriangleMesh),
}

impl Object {}

impl ComputeIntersection for Object {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        match self {
            Object::Sphere(sphere) => sphere.intersect(ray),
            Object::TriangleMesh(mesh) => mesh.intersect(ray),
        }
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        match self {
            Object::Sphere(sphere) => sphere.shadow_intersect(ray),
            Object::TriangleMesh(mesh) => mesh.shadow_intersect(ray),
        }
    }
}
