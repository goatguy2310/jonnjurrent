use core::f64::consts::PI;

use renderer::{ImageRenderer, Material, QuadBuilder, SceneBuilder, TriangleMeshBuilder};
use renderer::{RenderConfig, SceneConfig, render_config, scene_config};

render_config!(RConf, width: 512, height: 512, samples: 30);
scene_config!(SConf);

fn main() {
    let mut scene = SceneBuilder::new()
        .camera_center(0., 0., 55.)
        .light_position(0., 30., 0.)
        .light_intensity(1E7)
        .fov(60. * PI / 180.)
        .gamma(2.2)
        .max_light_bounce(5)
        .build();

    let white = scene.add_material(Material::lambertian([0.8, 0.8, 0.8]));
    let green = scene.add_material(Material::lambertian([0.12, 0.45, 0.15]));
    let red = scene.add_material(Material::lambertian([0.65, 0.05, 0.05]));
    // let transparent = scene.add_material(Material::transparent(1.5));

    let queen_marika = TriangleMeshBuilder::new()
        .read_obj_file(&mut scene, "assets/marika/base.obj")
        .scale_translate(30., [0., -13., -40.])
        .build();

    scene.add_object(queen_marika);

    let floor = QuadBuilder::new()
        .corner(-200.0, -40.0, -200.0)
        .u(0.0, 0.0, 400.0)
        .v(400.0, 0.0, 0.0)
        .material(white)
        .build();

    let ceiling = QuadBuilder::new()
        .corner(-200.0, 40.0, -200.0)
        .u(400.0, 0.0, 0.0)
        .v(0.0, 0.0, 400.0)
        .material(white)
        .build();

    let left_wall = QuadBuilder::new()
        .corner(-40.0, -200.0, -200.0)
        .u(0.0, 400.0, 0.0)
        .v(0.0, 0.0, 400.0)
        .material(green)
        .build();

    let right_wall = QuadBuilder::new()
        .corner(40.0, -200.0, -200.0)
        .u(0.0, 0.0, 400.0)
        .v(0.0, 400.0, 0.0)
        .material(red)
        .build();

    let front_wall = QuadBuilder::new()
        .corner(-200.0, -200.0, -70.0)
        .u(400.0, 0.0, 0.0)
        .v(0.0, 400.0, 0.0)
        .material(white)
        .build();

    let back_wall = QuadBuilder::new()
        .corner(-200.0, -200.0, 70.0)
        .u(400.0, 0.0, 0.0)
        .v(0.0, 400.0, 0.0)
        .material(white)
        .build();

    scene.add_object(floor);
    scene.add_object(ceiling);
    scene.add_object(left_wall);
    scene.add_object(right_wall);
    scene.add_object(front_wall);
    scene.add_object(back_wall);

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
