use crate::vector::Vector;

#[derive(Debug, Clone, Copy)]
pub struct MaterialIndex(pub usize);

#[derive(Debug)]
pub enum Material {
    Lambertian(Vector),
    Mirror,
    Glass,
}

impl Material {
    pub fn new_lambertian(color: impl Into<Vector>) -> Self {
        Material::Lambertian(color.into())
    }

    pub fn new_mirror() -> Self {
        Material::Mirror
    }

    pub fn new_transparent() -> Self {
        Material::Glass
    }

    pub fn get_albedo(&self) -> &Vector {
        match self {
            Material::Lambertian(vector) => vector,
            Material::Mirror => panic!(),
            Material::Glass => panic!(),
        }
    }

    pub fn is_mirror(&self) -> bool {
        match self {
            Material::Lambertian(_vector) => false,
            Material::Mirror => true,
            Material::Glass => false,
        }
    }

    pub fn is_transparent(&self) -> bool {
        match self {
            Material::Lambertian(_vector) => false,
            Material::Mirror => false,
            Material::Glass => true,
        }
    }
}
