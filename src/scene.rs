use core::f64;

use crate::material::{Material, MaterialIndex};
use crate::object::{ComputeIntersection, Intersection, Object};
use crate::ray::Ray;
use crate::vector::Vector;

#[derive(Debug)]
pub struct Scene {
    objects: Vec<Object>,
    materials: Vec<Material>,

    camera_center: Vector,
    light_position: Vector,

    fov: f64,
    gamma: f64,
    light_intensity: f64,

    max_light_bounce: u32,
}

impl Scene {
    pub fn add_object(&mut self, object: Object) {
        self.objects.push(object);
    }

    pub fn add_material(&mut self, material: Material) -> MaterialIndex {
        let index = self.materials.len();
        self.materials.push(material);
        MaterialIndex(index)
    }

    pub fn get_camera_center(&self) -> Vector {
        self.camera_center.clone()
    }

    pub fn get_fov(&self) -> f64 {
        self.fov
    }

    pub fn get_gamma(&self) -> f64 {
        self.gamma
    }

    pub fn get_simple_color(&self, ray: &Ray) -> Vector {
        match self.intersect(ray) {
            Some(_) => Vector::ONE,
            None => Vector::ZERO,
        }
    }

    pub fn get_color<CONFIG: SceneConfig>(&self, ray: &Ray, depth: u32) -> Vector {
        const EPS: f64 = 1e-4;

        if depth >= self.max_light_bounce {
            return Vector::ZERO;
        }

        match self.intersect(ray) {
            Some(intersection) => {
                let material = &self.materials[intersection.index.0];

                if material.is_transparent() {
                    let mut n_1: f64 = 1.0;
                    let mut n_2: f64 = 1.5;
                    let mut cos_i = ray.direction.dot(&intersection.normal);
                    let mut normal = intersection.normal.clone();

                    if cos_i > 0. {
                        normal = -normal;
                        cos_i = -cos_i;
                        core::mem::swap(&mut n_1, &mut n_2);
                    }

                    let eta = n_1 / n_2;
                    let k = 1.0 - eta.powi(2) * (1.0 - cos_i.powi(2));

                    let reflected_ray = Ray::with_time(
                        &intersection.intersection + &normal * EPS,
                        &ray.direction - 2. * ray.direction.dot(&normal) * &normal,
                        ray.time,
                    );
                    let reflected_color = self.get_color::<CONFIG>(&reflected_ray, depth + 1);

                    if k < 0. {
                        return reflected_color;
                    }

                    let refracted_ray = Ray::with_time(
                        &intersection.intersection - &normal * EPS,
                        eta * (&ray.direction - cos_i * &normal) - k.sqrt() * &normal,
                        ray.time,
                    );
                    let refracted_color = self.get_color::<CONFIG>(&refracted_ray, depth + 1);

                    let r0 = ((n_1 - n_2) / (n_1 + n_2)).powi(2);
                    let schlick = r0 + (1.0 - r0) * (1.0 - cos_i.abs()).powi(5);

                    return schlick * reflected_color + (1.0 - schlick) * refracted_color;
                }

                let intersection_origin = &intersection.intersection + &intersection.normal * EPS;

                if material.is_mirror() {
                    let reflected_ray = Ray::with_time(
                        intersection_origin,
                        &ray.direction
                            - 2. * ray.direction.dot(&intersection.normal) * &intersection.normal,
                        ray.time,
                    );

                    return self.get_color::<CONFIG>(&reflected_ray, depth + 1);
                }

                let albedo = material.get_albedo();

                let light_dir = &self.light_position - &intersection.intersection;
                let distance_to_light = light_dir.norm();
                let omega_i = light_dir / distance_to_light;
                let cos_term = intersection.normal.dot(&omega_i).max(0.0);

                let shadow_ray =
                    Ray::with_time(intersection_origin.clone(), omega_i.clone(), ray.time);

                let visibility = match self.shadow_intersect(&shadow_ray) {
                    None => 1.0,
                    Some(t) => {
                        if t > distance_to_light {
                            1.0
                        } else {
                            0.0
                        }
                    }
                };

                let distance_squared = distance_to_light.powi(2);
                let intensity_falloff =
                    self.light_intensity / (4.0 * f64::consts::PI * distance_squared);
                let albedo_factor = albedo * f64::consts::PI.recip();

                let direct_light = intensity_falloff * albedo_factor * visibility * cos_term;

                if CONFIG::ENABLE_INDIRECT_LIGHTING {
                    let mut rng = fastrand::Rng::new();
                    let r1 = rng.f64();
                    let r2 = rng.f64();

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
                    let indirect_light = self.get_color::<CONFIG>(&random_ray, depth + 1) * albedo;

                    direct_light + indirect_light
                } else {
                    direct_light
                }
            }

            None => Vector::ZERO,
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
        Scene {
            objects: Vec::new(),
            materials: Vec::new(),
            camera_center: self.camera_center,
            light_position: self.light_position,
            fov: self.fov,
            gamma: self.gamma,
            light_intensity: self.light_intensity,
            max_light_bounce: self.max_light_bounce,
        }
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
