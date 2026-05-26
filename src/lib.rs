mod material;
mod renderer;
mod scene;
mod texture;

pub mod geometry;
pub mod larp;
pub mod math;

pub use geometry::QuadBuilder;
pub use geometry::SphereBuilder;
pub use geometry::TriangleMeshBuilder;

pub use material::Material;
pub use texture::Texture;

pub use scene::SceneBuilder;
pub use scene::SceneConfig;

pub use renderer::ImageRenderer;
pub use renderer::RenderConfig;

#[macro_export]
macro_rules! scene_config {
    ($name:ident) => {
        pub struct $name;

        impl SceneConfig for $name {
            const ENABLE_INDIRECT_LIGHTING: bool = true;
        }
    };

    ($name:ident, indirect_lighting: $ind_light:expr) => {
        pub struct $name;

        impl SceneConfig for $name {
            const ENABLE_INDIRECT_LIGHTING: bool = $ind_light;
        }
    };
}

#[macro_export]
macro_rules! render_config {
    ($name:ident, width: $width:expr, height: $height:expr, samples: $samples:expr) => {
        pub struct $name;

        impl RenderConfig for $name {
            const WIDTH: usize = $width;
            const HEIGHT: usize = $height;
            const IMAGE_SIZE: usize = $width * $height * 3;
            const SAMPLE_PER_PIXEL: usize = $samples;
        }
    };
}
