use core::f64::consts::PI;

use renderer::{ImageRenderer, Material, SceneBuilder, SphereBuilder, TriangleMeshBuilder};
use renderer::{RenderConfig, SceneConfig, render_config, scene_config};

render_config!(RConf, width: 1024, height: 1024, samples: 100);
scene_config!(SConf);

fn main() {
    let mut scene = SceneBuilder::new()
        .camera_center(0., 0., 55.)
        .light_position(-10., 40., 40.)
        .light_intensity(3E7)
        .fov(60. * PI / 180.)
        .gamma(2.2)
        .max_light_bounce(8)
        .build();

    let white = scene.add_material(Material::lambertian([0.8, 0.8, 0.8]));

    let queen_marika = TriangleMeshBuilder::new()
        .read_obj_file(&mut scene, "assets/marika/base.obj")
        .scale_translate(25., [0., 0., 0.])
        .build();

    let background = SphereBuilder::new()
        .center(0., 0., -1000.)
        .radius(940.)
        .material(white)
        .build();

    scene.add_object(queen_marika);
    scene.add_object(background);

    let img = ImageRenderer::render::<RConf, SConf, { RConf::IMAGE_SIZE }>(scene);

    image::save_buffer(
        "render.png",
        &img,
        RConf::WIDTH as u32,
        RConf::HEIGHT as u32,
        image::ExtendedColorType::Rgb8,
    )
    .unwrap();
}
