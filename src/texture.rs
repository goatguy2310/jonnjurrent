use crate::math::Vector;

pub trait TextureLike {
    fn value(&self, u: f64, v: f64) -> Vector;
}

#[derive(Debug, Clone)]
pub enum Texture {
    SolidColor(Vector),
    Image(ImageTexture),
    Checker(CheckerTexture),
}

impl Texture {
    #[inline]
    #[must_use]
    pub fn solid_color(color: impl Into<Vector>) -> Texture {
        Texture::SolidColor(color.into())
    }

    #[inline]
    #[must_use]
    pub const fn image(data: Vec<u8>, width: u32, height: u32) -> Texture {
        Texture::Image(ImageTexture {
            data,
            width,
            height,
        })
    }

    #[inline]
    #[must_use]
    pub fn image_from_path(texture_path: impl AsRef<std::path::Path> + std::fmt::Debug) -> Texture {
        let image = image::open(&texture_path)
            .unwrap_or_else(|_| panic!("Failed to load texture: {:?}", texture_path))
            .to_rgb8();

        let (width, height) = image.dimensions();

        Texture::image(image.into_raw(), width, height)
    }

    #[inline]
    #[must_use]
    pub const fn checker(scale: f64, even: Box<Texture>, odd: Box<Texture>) -> Texture {
        Texture::Checker(CheckerTexture { scale, even, odd })
    }
}

impl TextureLike for Texture {
    fn value(&self, u: f64, v: f64) -> Vector {
        match self {
            Texture::SolidColor(color) => color.clone(),
            Texture::Image(image) => image.value(u, v),
            Texture::Checker(checker) => checker.value(u, v),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ImageTexture {
    data: Vec<u8>,
    width: u32,
    height: u32,
}

impl TextureLike for ImageTexture {
    fn value(&self, u: f64, v: f64) -> Vector {
        let u = u.clamp(0.0, 1.0);
        let v = 1.0 - v.clamp(0.0, 1.0);

        let x = u * (self.width - 1) as f64;
        let y = v * (self.height - 1) as f64;

        let x0 = x.floor() as usize;
        let y0 = y.floor() as usize;

        let x1 = (x0 + 1).min(self.width as usize - 1);
        let y1 = (y0 + 1).min(self.height as usize - 1);

        let tx = x - x0 as f64;
        let ty = y - y0 as f64;

        let fetch = |x: usize, y: usize| -> Vector {
            let idx = (y * self.width as usize + x) * 3;

            Vector::new(
                self.data[idx] as f64 / 255.0,
                self.data[idx + 1] as f64 / 255.0,
                self.data[idx + 2] as f64 / 255.0,
            )
        };

        let c00 = fetch(x0, y0);
        let c10 = fetch(x1, y0);
        let c01 = fetch(x0, y1);
        let c11 = fetch(x1, y1);

        let c0 = c00 * (1.0 - tx) + c10 * tx;
        let c1 = c01 * (1.0 - tx) + c11 * tx;

        c0 * (1.0 - ty) + c1 * ty
    }
}

#[derive(Debug, Clone)]
pub struct CheckerTexture {
    scale: f64,
    even: Box<Texture>,
    odd: Box<Texture>,
}

impl TextureLike for CheckerTexture {
    fn value(&self, u: f64, v: f64) -> Vector {
        let x = (u * self.scale).floor() as i32;
        let y = (v * self.scale).floor() as i32;

        if (x + y) % 2 == 0 {
            self.even.value(u, v)
        } else {
            self.odd.value(u, v)
        }
    }
}
