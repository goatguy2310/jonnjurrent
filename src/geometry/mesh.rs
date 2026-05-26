use core::f64;
use std::{collections::HashMap, mem::take};

use crate::{
    Material,
    geometry::{ComputeIntersection, Intersection, Object},
    larp::Bvh,
    material::MaterialIndex,
    math::{Ray, Vector},
    scene::Scene,
    texture::Texture,
};

const EPS: f64 = 1e-9;

#[derive(Debug, Default, Clone)]
pub struct TriangleSoup {
    pub vtx: [Vector; 3],
    normal: [Vector; 3],
    uv: [(f64, f64); 3],
    material_index: MaterialIndex,
}

impl TriangleSoup {
    #[inline(always)]
    #[must_use]
    pub const fn new(
        vtx: [Vector; 3],
        normal: [Vector; 3],
        uv: [(f64, f64); 3],
        material_index: MaterialIndex,
    ) -> Self {
        TriangleSoup {
            vtx,
            normal,
            uv,
            material_index,
        }
    }

    #[inline(always)]
    #[must_use]
    pub const fn with_default_material(
        vtx: [Vector; 3],
        normal: [Vector; 3],
        uv: [(f64, f64); 3],
    ) -> Self {
        TriangleSoup {
            vtx,
            normal,
            uv,
            material_index: MaterialIndex(0),
        }
    }

    #[inline]
    pub fn on_each_vertex(&mut self, map: &dyn Fn(Vector) -> Vector) {
        self.vtx[0] = map(take(&mut self.vtx[0]));
        self.vtx[1] = map(take(&mut self.vtx[1]));
        self.vtx[2] = map(take(&mut self.vtx[2]));
    }
}

impl crate::larp::Boundable for TriangleSoup {
    fn bounding_box(&self) -> crate::larp::BoundingBox {
        const PADDING: f64 = 1e-6;

        let a = &self.vtx[0];
        let b = &self.vtx[1];
        let c = &self.vtx[2];

        let min = a.infimum(b).infimum(c);
        let max = a.supremum(b).supremum(c);

        crate::larp::BoundingBox::new(min, max).extend(PADDING)
    }
}

#[derive(Debug)]
pub struct TriangleMesh {
    triangles: Vec<TriangleSoup>,
    bvh: Bvh,
}

impl TriangleMesh {
    #[inline(always)]
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            triangles: Vec::new(),
            bvh: Bvh::empty(),
        }
    }

    #[inline(always)]
    #[must_use]
    pub const fn with_triangles(triangles: Vec<TriangleSoup>) -> Self {
        Self {
            triangles,
            bvh: Bvh::empty(),
        }
    }
}

impl ComputeIntersection for TriangleMesh {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        let candidates = self.bvh.intersect_ray(ray, f64::EPSILON, f64::INFINITY);

        let hit = candidates
            .into_iter()
            .filter_map(|index| {
                let triangle = &self.triangles[index];
                let vertex_a = &triangle.vtx[0];
                let vertex_b = &triangle.vtx[1];
                let vertex_c = &triangle.vtx[2];

                let e_1 = vertex_b - vertex_a;
                let e_2 = vertex_c - vertex_a;

                let a_minus_o = vertex_a - &ray.origin;
                let a_minus_o_cross_u = a_minus_o.cross(&ray.direction);

                let n = e_1.cross(&e_2);
                let denom = ray.direction.dot(&n);
                let inv_denom = 1. / denom;

                if denom.abs() <= EPS {
                    return None;
                }

                let beta = e_2.dot(&a_minus_o_cross_u) * inv_denom;
                let gamma = -(e_1.dot(&a_minus_o_cross_u) * inv_denom);

                if !(-EPS..=1. + EPS).contains(&beta)
                    || !(-EPS..=1. + EPS).contains(&gamma)
                    || 1. - beta - gamma < -EPS
                {
                    return None;
                }

                let t = a_minus_o.dot(&n) * inv_denom;

                if t <= EPS {
                    return None;
                }

                Some((index, t, beta, gamma))
            })
            .min_by(|t_1, t_2| t_1.1.total_cmp(&t_2.1))?;

        let (index, t, beta, gamma) = hit;
        let alpha = 1. - beta - gamma;
        let intersection = ray.at(t);

        let tri_normal = &self.triangles[index].normal;
        let normal_a = &tri_normal[0];
        let normal_b = &tri_normal[1];
        let normal_c = &tri_normal[2];

        let normal = (normal_a * alpha + normal_b * beta + normal_c * gamma).normalize();

        let tri_uv = &self.triangles[index].uv;
        let uv_a = &tri_uv[0];
        let uv_b = &tri_uv[1];
        let uv_c = &tri_uv[2];

        let u = uv_a.0 * alpha + uv_b.0 * beta + uv_c.0 * gamma;
        let v = uv_a.1 * alpha + uv_b.1 * beta + uv_c.1 * gamma;

        Some(Intersection::new(
            intersection,
            t,
            normal,
            (u, v),
            self.triangles[index].material_index,
        ))
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        let candidates = self.bvh.intersect_ray(ray, f64::EPSILON, f64::INFINITY);

        candidates
            .into_iter()
            .filter_map(|index| {
                let triangle = &self.triangles[index];
                let vertex_a = &triangle.vtx[0];
                let vertex_b = &triangle.vtx[1];
                let vertex_c = &triangle.vtx[2];

                let e_1 = vertex_b - vertex_a;
                let e_2 = vertex_c - vertex_a;

                let n = e_1.cross(&e_2);
                let denom = ray.direction.dot(&n);
                if denom.abs() <= EPS {
                    return None;
                }

                let inv_denom = 1. / denom;

                let a_minus_o = vertex_a - &ray.origin;
                let a_minus_o_cross_u = a_minus_o.cross(&ray.direction);

                let beta = e_2.dot(&a_minus_o_cross_u) * inv_denom;
                let gamma = -(e_1.dot(&a_minus_o_cross_u) * inv_denom);

                if !(-EPS..=1. + EPS).contains(&beta)
                    || !(-EPS..=1. + EPS).contains(&gamma)
                    || 1. - beta - gamma < -EPS
                {
                    return None;
                }

                let t = a_minus_o.dot(&n) * inv_denom;

                if t <= EPS {
                    return None;
                }

                Some(t)
            })
            .min_by(|t_1, t_2| t_1.total_cmp(t_2))
    }
}

#[derive(Debug)]
pub struct TriangleMeshBuilder {
    triangles: Vec<TriangleSoup>,
    material_index: MaterialIndex,
}

impl TriangleMeshBuilder {
    pub fn new() -> Self {
        TriangleMeshBuilder {
            triangles: Vec::new(),
            material_index: MaterialIndex(0),
        }
    }

    pub fn build(self) -> Object {
        let mut mesh = TriangleMesh {
            triangles: self.triangles,
            bvh: Bvh::empty(),
        };
        mesh.bvh = Bvh::build(&mesh.triangles);

        Object::TriangleMesh(mesh)
    }

    pub fn build_soup(self) -> Vec<TriangleSoup> {
        self.triangles
    }

    pub fn fallback_material(mut self, index: MaterialIndex) -> Self {
        self.material_index = index;
        self
    }

    pub fn scale_translate(mut self, scale: f64, translate: impl Into<Vector>) -> Self {
        let translate_ref = &translate.into();
        self.triangles.iter_mut().for_each(|tri| {
            tri.on_each_vertex(&|vtx| vtx * scale + translate_ref);
        });

        self
    }

    pub fn read_obj_file(
        mut self,
        scene: &mut Scene,
        obj: impl AsRef<std::path::Path> + std::fmt::Debug,
    ) -> Self {
        let (models, materials) =
            tobj::load_obj(&obj, &tobj::GPU_LOAD_OPTIONS).expect("Failed to load OBJ file");

        let default_material_index = self.material_index;

        let mut material_map = HashMap::<usize, MaterialIndex>::new();

        for (idx, material) in materials.iter().flatten().enumerate() {
            if let Some(diffuse_texture_path) = &material.diffuse_texture {
                let texture_path = obj
                    .as_ref()
                    .parent()
                    .unwrap_or(std::path::Path::new(""))
                    .join(diffuse_texture_path);

                let image_texture = Texture::image_from_path(texture_path);
                let diffuse_material = Material::lambertian_with_texture(image_texture);

                let material_index = scene.add_material(diffuse_material);
                material_map.insert(idx, material_index);
                continue;
            }

            if let Some(diffuse) = material.diffuse {
                let material_index = scene.add_material(Material::lambertian(Vector::new(
                    diffuse[0], diffuse[1], diffuse[2],
                )));

                material_map.insert(idx, material_index);
                continue;
            }

            material_map.insert(idx, default_material_index);
        }

        for model in models {
            let mesh = &model.mesh;

            let vertices = mesh
                .positions
                .chunks_exact(3)
                .map(|c| Vector::new(c[0], c[1], c[2]))
                .collect();

            let normals: Vec<Vector> = if !mesh.normals.is_empty() {
                mesh.normals
                    .chunks_exact(3)
                    .map(|c| Vector::new(c[0], c[1], c[2]))
                    .collect()
            } else {
                Vec::new()
            };

            let uvs: Vec<(f64, f64)> = mesh
                .texcoords
                .chunks_exact(2)
                .map(|c| (c[0], c[1]))
                .collect();

            for chunk in mesh.indices.chunks_exact(3) {
                let idx0 = chunk[0] as usize;
                let idx1 = chunk[1] as usize;
                let idx2 = chunk[2] as usize;

                let take_index = |a: &Vec<Vector>| -> [Vector; 3] {
                    if !a.is_empty() {
                        [a[idx0].clone(), a[idx1].clone(), a[idx2].clone()]
                    } else {
                        [Vector::ZERO; 3]
                    }
                };

                let vtx = take_index(&vertices);

                let normal = if !normals.is_empty() {
                    [
                        normals[idx0].clone(),
                        normals[idx1].clone(),
                        normals[idx2].clone(),
                    ]
                } else {
                    let e_1 = &vtx[1] - &vtx[0];
                    let e_2 = &vtx[2] - &vtx[0];
                    let n = e_1.cross(&e_2).normalize();
                    [n.clone(), n.clone(), n]
                };

                let uv = if !uvs.is_empty() {
                    [uvs[idx0], uvs[idx1], uvs[idx2]]
                } else {
                    [(0., 0.); 3]
                };

                let material_index = mesh
                    .material_id
                    .and_then(|idx| material_map.get(&idx).copied())
                    .unwrap_or(default_material_index);

                self.triangles
                    .push(TriangleSoup::new(vtx, normal, uv, material_index));
            }
        }

        self
    }

    pub fn soup_from_obj(mut self, obj: impl AsRef<std::path::Path> + std::fmt::Debug) -> Self {
        let (models, _materials) =
            tobj::load_obj(&obj, &tobj::GPU_LOAD_OPTIONS).expect("Failed to load OBJ file");

        for model in models {
            let mesh = &model.mesh;

            let vertices = mesh
                .positions
                .chunks_exact(3)
                .map(|c| Vector::new(c[0], c[1], c[2]))
                .collect();

            let normals: Vec<Vector> = if !mesh.normals.is_empty() {
                mesh.normals
                    .chunks_exact(3)
                    .map(|c| Vector::new(c[0], c[1], c[2]))
                    .collect()
            } else {
                Vec::new()
            };

            let uvs: Vec<(f64, f64)> = mesh
                .texcoords
                .chunks_exact(2)
                .map(|c| (c[0], c[1]))
                .collect();

            for chunk in mesh.indices.chunks_exact(3) {
                let idx0 = chunk[0] as usize;
                let idx1 = chunk[1] as usize;
                let idx2 = chunk[2] as usize;

                let take_index = |a: &Vec<Vector>| -> [Vector; 3] {
                    if !a.is_empty() {
                        [a[idx0].clone(), a[idx1].clone(), a[idx2].clone()]
                    } else {
                        [Vector::ZERO; 3]
                    }
                };

                let vtx = take_index(&vertices);

                let normal = if !normals.is_empty() {
                    [
                        normals[idx0].clone(),
                        normals[idx1].clone(),
                        normals[idx2].clone(),
                    ]
                } else {
                    let e_1 = &vtx[1] - &vtx[0];
                    let e_2 = &vtx[2] - &vtx[0];
                    let n = e_1.cross(&e_2).normalize();
                    [n.clone(), n.clone(), n]
                };

                let uv = if !uvs.is_empty() {
                    [uvs[idx0], uvs[idx1], uvs[idx2]]
                } else {
                    [(0., 0.); 3]
                };

                self.triangles
                    .push(TriangleSoup::with_default_material(vtx, normal, uv));
            }
        }

        self
    }
}

impl Default for TriangleMeshBuilder {
    fn default() -> Self {
        Self::new()
    }
}
