#![allow(unused)]
use crate::vector::Vector;

pub trait TextureLike {
    fn value(&self, u: f64, v: f64) -> Vector;
}

#[derive(Debug, Clone)]
pub enum Texture {
    SolidColor(Vector),
    Image(ImageTexture),
    Checker(CheckerTexture),
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
    width: usize,
    height: usize,
}

impl TextureLike for ImageTexture {
    fn value(&self, u: f64, v: f64) -> Vector {
        let u = u.clamp(0.0, 1.0);
        let v = 1.0 - v.clamp(0.0, 1.0);

        let x = (u * (self.width - 1) as f64) as usize;
        let y = (v * (self.height - 1) as f64) as usize;
        let idx = (y * self.width + x) * 3;

        Vector::new(
            self.data[idx] as f64 / 255.0,
            self.data[idx + 1] as f64 / 255.0,
            self.data[idx + 2] as f64 / 255.0,
        )
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
        todo!()
    }
}
