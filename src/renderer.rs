use core::f64;

use rayon::prelude::*;

use crate::{SceneConfig, ray::Ray, scene::Scene, vector::Vector};

#[derive(Debug)]
pub struct ImageRenderer {}

impl ImageRenderer {
    pub fn render<RC: RenderConfig, SC: SceneConfig, const IMAGE_SIZE: usize>(
        scene: Scene,
    ) -> [u8; IMAGE_SIZE] {
        let mut image: [u8; IMAGE_SIZE] = [0; IMAGE_SIZE];

        let gamma_correction = |channel: f64| -> u8 {
            (255.0 * (channel / 255.0).powf(1.0 / scene.get_gamma())).clamp(0.0, 255.0) as u8
        };

        let camera_center = scene.get_camera_center();
        let w = RC::WIDTH as f64;
        let h = RC::HEIGHT as f64;
        let focal_distance = (w / 2.0) / (scene.get_fov() / 2.0).tan();

        image
            .par_chunks_mut(RC::WIDTH * 3)
            .enumerate()
            .for_each(|(i, row)| {
                let mut rng = fastrand::Rng::new();

                for j in 0..RC::WIDTH {
                    let mut color = Vector::default();

                    for _ in 0..RC::SAMPLE_PER_PIXEL {
                        let muller = f64::sqrt(-2. * rng.f64().log(2.))
                            * f64::cos(2. * f64::consts::PI * rng.f64());
                        let dx = muller * 0.5;
                        let dy = muller * 0.5;

                        let pixel = Vector::new(
                            (j as f64) - w / 2.0 + 0.5 + dx,
                            h / 2. - (i as f64) - 0.5 + dy,
                            -focal_distance,
                        );

                        let ray = Ray::with_time(
                            camera_center.clone(),
                            pixel.normalize(),
                            rng.f64_inclusive(),
                        );
                        color += scene.get_color::<SC>(&ray, 0, false);
                    }

                    color /= RC::SAMPLE_PER_PIXEL as f64;

                    let index = j * 3;
                    row[index] = gamma_correction(color[0]);
                    row[index + 1] = gamma_correction(color[1]);
                    row[index + 2] = gamma_correction(color[2]);
                }
            });

        image
    }
}

pub trait RenderConfig {
    const WIDTH: usize;
    const HEIGHT: usize;
    const IMAGE_SIZE: usize;

    const SAMPLE_PER_PIXEL: usize;
}
