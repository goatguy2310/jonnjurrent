use core::f64;

use crate::geometry::{ComputeIntersection, Intersection, Object};
use crate::material::{Material, MaterialIndex, MaterialLike};
use crate::math::{Ray, Vector};
use crate::texture::TextureLike;

const EPS: f64 = 1e-4;

#[derive(Debug)]
pub struct Scene {
    objects: Vec<Object>,
    lights: Vec<usize>,
    materials: Vec<Material>,

    camera_center: Vector,
    light_position: Vector,

    fov: f64,
    gamma: f64,
    light_intensity: f64,

    max_light_bounce: u32,
}

impl Scene {
    #[inline]
    pub fn add_object(&mut self, object: Object) {
        self.objects.push(object);
    }

    #[inline]
    pub fn add_light(&mut self, object: Object) {
        let object_index = self.objects.len();

        self.add_object(object);
        self.lights.push(object_index);
    }

    #[inline]
    #[must_use]
    pub fn add_material(&mut self, material: Material) -> MaterialIndex {
        let index = self.materials.len();
        self.materials.push(material);
        MaterialIndex(index)
    }

    #[inline]
    #[must_use]
    pub fn add_lambertian(&mut self, color: impl Into<Vector>) -> MaterialIndex {
        self.add_material(Material::lambertian(color))
    }

    #[inline]
    #[must_use]
    pub fn get_camera_center(&self) -> Vector {
        self.camera_center.clone()
    }

    #[inline]
    #[must_use]
    pub const fn get_fov(&self) -> f64 {
        self.fov
    }

    #[inline]
    #[must_use]
    pub const fn get_gamma(&self) -> f64 {
        self.gamma
    }

    #[inline]
    #[must_use]
    fn visibility(&self, distance: Option<f64>, distance_to_light: f64) -> f64 {
        match distance {
            Some(t) => f64::from(t > distance_to_light - EPS),
            None => 1.,
        }
    }

    pub fn get_color<CONFIG: SceneConfig>(
        &self,
        ray: &Ray,
        depth: u32,
        last_bounce_diffuse: bool,
    ) -> Vector {
        if depth >= self.max_light_bounce {
            return Vector::ZERO;
        }

        let intersection = match self.intersect(ray) {
            Some(intersect) => intersect,
            None => return Vector::ZERO,
        };

        let material = &self.materials[intersection.index.0];

        match material {
            Material::Light(albedo) => {
                if last_bounce_diffuse {
                    Vector::ZERO
                } else {
                    albedo.clone()
                }
            }

            Material::Lambertian(texture) => {
                let albedo = texture.value(intersection.u, intersection.v);

                let light_dir = &self.light_position - &intersection.intersection;
                let distance_to_light = light_dir.norm();
                let omega_i = light_dir / distance_to_light;
                let cos_term = intersection.normal.dot(&omega_i).max(0.0);

                let intersection_origin = &intersection.intersection + &intersection.normal * EPS;

                let shadow_ray = Ray::new(intersection_origin.clone(), omega_i.clone());

                let visibility =
                    self.visibility(self.shadow_intersect(&shadow_ray), distance_to_light);

                let distance_squared = distance_to_light.powi(2);
                let intensity_falloff =
                    self.light_intensity / (4.0 * f64::consts::PI * distance_squared);
                let albedo_factor = &albedo * f64::consts::PI.recip();

                let mut direct_light = visibility * intensity_falloff * albedo_factor * cos_term;

                if CONFIG::ENABLE_INDIRECT_LIGHTING {
                    let r1 = fastrand::f64();
                    let r2 = fastrand::f64();

                    let x = f64::cos(2. * f64::consts::PI * r1) * f64::sqrt(1. - r2);
                    let y = f64::sin(2. * f64::consts::PI * r1) * f64::sqrt(1. - r2);
                    let z = r2.sqrt();

                    let axis = if intersection.normal.x.abs() < intersection.normal.y.abs() {
                        Vector::X
                    } else {
                        Vector::Y
                    };

                    let t_1 = intersection.normal.cross(&axis).normalize();
                    let t_2 = intersection.normal.cross(&t_1);

                    let v = (x * t_1 + y * t_2 + z * &intersection.normal).normalize();

                    let random_ray = Ray::new(intersection_origin, v);
                    let indirect_light =
                        albedo * self.get_color::<CONFIG>(&random_ray, depth + 1, true);

                    direct_light += indirect_light;
                }

                direct_light
            }

            Material::Metallic(..) | Material::Dielectric(..) => {
                self.get_color::<CONFIG>(&material.scatter(ray, &intersection), depth + 1, false)
            }
        }
    }
}

impl ComputeIntersection for Scene {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        self.objects
            .iter()
            .flat_map(|object| object.intersect(ray))
            .min_by(|a, b| a.distance.total_cmp(&b.distance))
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        self.objects
            .iter()
            .flat_map(|object| object.shadow_intersect(ray))
            .min_by(|t_1, t_2| t_1.total_cmp(t_2))
    }
}

pub trait SceneConfig {
    const ENABLE_INDIRECT_LIGHTING: bool;
}

#[derive(Debug)]
pub struct SceneBuilder {
    camera_center: Vector,
    light_position: Vector,

    fov: f64,
    gamma: f64,
    light_intensity: f64,

    max_light_bounce: u32,
}

impl SceneBuilder {
    pub fn new() -> Self {
        Self {
            camera_center: Vector::default(),
            light_position: Vector::default(),
            fov: 0.,
            gamma: 0.,
            light_intensity: 0.,
            max_light_bounce: 0,
        }
    }

    pub fn build(self) -> Scene {
        let mut scene = Scene {
            objects: Vec::new(),
            lights: Vec::new(),
            materials: Vec::new(),
            camera_center: self.camera_center,
            light_position: self.light_position,
            fov: self.fov,
            gamma: self.gamma,
            light_intensity: self.light_intensity,
            max_light_bounce: self.max_light_bounce,
        };

        let _ = scene.add_material(Material::lambertian([0.8, 0.8, 0.8]));

        scene
    }

    pub fn camera_center(mut self, x: f64, y: f64, z: f64) -> Self {
        self.camera_center.x = x;
        self.camera_center.y = y;
        self.camera_center.z = z;
        self
    }

    pub fn light_position(mut self, x: f64, y: f64, z: f64) -> Self {
        self.light_position.x = x;
        self.light_position.y = y;
        self.light_position.z = z;
        self
    }

    pub fn fov(mut self, fov: f64) -> Self {
        self.fov = fov;
        self
    }

    pub fn gamma(mut self, gamma: f64) -> Self {
        self.gamma = gamma;
        self
    }

    pub fn light_intensity(mut self, light_intensity: f64) -> Self {
        self.light_intensity = light_intensity;
        self
    }

    pub fn max_light_bounce(mut self, max_light_bounce: u32) -> Self {
        self.max_light_bounce = max_light_bounce;
        self
    }
}

impl Default for SceneBuilder {
    fn default() -> Self {
        Self::new()
    }
}
