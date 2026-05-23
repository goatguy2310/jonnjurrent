use core::f64::consts::PI;

use renderer::{ImageRenderer, Material, SceneBuilder, SphereBuilder, Texture};
use renderer::{RenderConfig, SceneConfig, render_config, scene_config};

render_config!(RConf, width: 1024, height: 1024, samples: 10);
scene_config!(SConf);

fn main() {
    let mut scene = SceneBuilder::new()
        .camera_center(0., 0., 55.)
        .light_position(-10., 20., 40.)
        .light_intensity(1E7)
        .fov(60. * PI / 180.)
        .gamma(2.2)
        .max_light_bounce(8)
        .build();

    let earth_texture = Texture::image_from_path("assets/textures/8k_earth_daymap.jpg");
    let sun_texture = Texture::image_from_path("assets/textures/8k_sun.jpg");
    let checker = Texture::checker(1000., sun_texture.into(), earth_texture.into());

    let earth = scene.add_material(Material::lambertian_with_texture(checker));

    let color2 = scene.add_material(Material::lambertian([0.5, 0.8, 0.1]));
    let color3 = scene.add_material(Material::lambertian([0.9, 0.2, 0.3]));
    let color4 = scene.add_material(Material::lambertian([0.1, 0.6, 0.7]));
    let color5 = scene.add_material(Material::lambertian([0.8, 0.2, 0.9]));
    let color6 = scene.add_material(Material::lambertian([0.3, 0.5, 0.3]));
    let color7 = scene.add_material(Material::lambertian([0.6, 0.5, 0.7]));

    let transp = scene.add_material(Material::transparent(1.5));
    let mirror = scene.add_material(Material::mirror(0.001));

    let center_sphere = SphereBuilder::new()
        .center(0., 0., 0.)
        .radius(10.)
        .material(earth)
        .build();

    let left_sphere = SphereBuilder::new()
        .center(-20., 0., 0.)
        .radius(10.)
        .material(mirror)
        .build();

    let right_sphere = SphereBuilder::new()
        .center(20., 0., 0.)
        .radius(10.)
        .material(transp)
        .build();

    let wall_left = SphereBuilder::new()
        .center(-1000., 0., 0.)
        .radius(940.)
        .material(color2)
        .build();

    let wall_right = SphereBuilder::new()
        .center(1000., 0., 0.)
        .radius(940.)
        .material(color3)
        .build();

    let wall_front = SphereBuilder::new()
        .center(0., 0., -1000.)
        .radius(940.)
        .material(color4)
        .build();

    let wall_behind = SphereBuilder::new()
        .center(0., 0., 1000.)
        .radius(940.)
        .material(color5)
        .build();

    let ceiling = SphereBuilder::new()
        .center(0., 1000., 0.)
        .radius(940.)
        .material(color6)
        .build();

    let floor = SphereBuilder::new()
        .center(0., -1000., 0.)
        .radius(990.)
        .material(color7)
        .build();

    scene.add_object(center_sphere);
    scene.add_object(left_sphere);
    scene.add_object(right_sphere);
    scene.add_object(wall_left);
    scene.add_object(wall_right);
    scene.add_object(wall_front);
    scene.add_object(wall_behind);
    scene.add_object(ceiling);
    scene.add_object(floor);

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
