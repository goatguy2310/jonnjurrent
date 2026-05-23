use crate::{object::Intersection, ray::Ray, texture::Texture, vector::Vector};

const EPS: f64 = 1e-4;

#[derive(Debug, Default, Clone, Copy)]
pub struct MaterialIndex(pub usize);

#[allow(unused)]
pub trait MaterialLike {
    fn emit(&self) -> Vector;
    fn scatter(&self, ray: &Ray, intersect: &Intersection<MaterialIndex>) -> Ray;
}

#[derive(Debug)]
pub enum Material {
    Lambertian(Texture),
    Metallic(f64),
    Dielectric(f64),
    Light(Vector),
}

impl Material {
    #[inline]
    #[must_use]
    pub fn lambertian(color: impl Into<Vector>) -> Self {
        let texture = Texture::solid_color(color.into());
        Material::Lambertian(texture)
    }

    #[inline]
    #[must_use]
    pub const fn lambertian_with_texture(texture: Texture) -> Self {
        Material::Lambertian(texture)
    }

    #[inline]
    #[must_use]
    pub fn lambertian_from_path(path: impl AsRef<std::path::Path> + std::fmt::Debug) -> Self {
        let texture = Texture::image_from_path(path);
        Material::lambertian_with_texture(texture)
    }

    #[inline]
    #[must_use]
    pub const fn mirror(fuzz: f64) -> Self {
        Material::Metallic(fuzz.clamp(0., 1.))
    }

    #[inline]
    #[must_use]
    pub const fn transparent(refraction_index: f64) -> Self {
        Material::Dielectric(refraction_index)
    }
}

impl MaterialLike for Material {
    fn emit(&self) -> Vector {
        match self {
            Material::Light(emission) => emission.clone(),
            _ => Vector::ZERO,
        }
    }

    fn scatter(&self, ray: &Ray, intersection: &Intersection<MaterialIndex>) -> Ray {
        match self {
            Material::Metallic(fuzz) => {
                let intersection_origin = &intersection.intersection + &intersection.normal * EPS;
                let reflected: Vector = Vector::reflect(&ray.direction, &intersection.normal)
                    .normalize()
                    + (*fuzz * Vector::random_unit());

                Ray::with_time(intersection_origin, reflected, ray.time)
            }

            Material::Dielectric(refraction_index) => {
                let mut n_1: f64 = 1.0;
                let mut n_2: f64 = *refraction_index;
                let mut cos_i = ray.direction.dot(&intersection.normal);
                let mut normal = intersection.normal.clone();

                if cos_i > 0. {
                    normal = -normal;
                    cos_i = -cos_i;
                    core::mem::swap(&mut n_1, &mut n_2);
                }

                let eta = n_1 / n_2;
                let k = 1.0 - eta.powi(2) * (1.0 - cos_i.powi(2));

                let r0 = ((n_1 - n_2) / (n_1 + n_2)).powi(2);
                let schlick = r0 + (1.0 - r0) * (1.0 - cos_i.abs()).powi(5);

                if k < 0. || fastrand::f64() < schlick {
                    Ray::with_time(
                        &intersection.intersection + &normal * EPS,
                        Vector::reflect(&ray.direction, &intersection.normal),
                        ray.time,
                    )
                } else {
                    Ray::with_time(
                        &intersection.intersection - &normal * EPS,
                        eta * (&ray.direction - cos_i * &normal) - k.sqrt() * &normal,
                        ray.time,
                    )
                }
            }

            _ => unreachable!(),
        }
    }
}
