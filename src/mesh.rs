use std::mem::take;

use crate::{
    bvh::{BVHConfig, BvhTree},
    material::MaterialIndex,
    object::{ComputeIntersection, Intersection, Object},
    ray::Ray,
    vector::Vector,
};

#[derive(Debug, Default, Clone)]
pub struct TriangleSoup {
    pub vtx: [Vector; 3],
    normal: [Vector; 3],
    uv: [Vector; 3],
    // #[allow(unused)]
    // material_index: MaterialIndex,
}

impl TriangleSoup {
    #[inline(always)]
    #[must_use]
    pub const fn new(vtx: [Vector; 3], normal: [Vector; 3], uv: [Vector; 3]) -> Self {
        TriangleSoup {
            vtx,
            normal,
            uv,
            // material_index: MaterialIndex(0),
        }
    }

    #[inline]
    #[must_use]
    pub fn center(&self) -> Vector {
        (&self.vtx[0] + &self.vtx[1] + &self.vtx[2]) * (1.0 / 3.0)
    }

    #[inline]
    pub fn on_each_vertex(&mut self, map: &dyn Fn(Vector) -> Vector) {
        self.vtx[0] = map(take(&mut self.vtx[0]));
        self.vtx[1] = map(take(&mut self.vtx[1]));
        self.vtx[2] = map(take(&mut self.vtx[2]));
    }
}

#[derive(Debug)]
pub struct TriangleMesh {
    triangles: Vec<TriangleSoup>,

    material_index: MaterialIndex,
    bvh: BvhTree,
}

impl ComputeIntersection for TriangleMesh {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        const EPS: f64 = 1e-9;

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

        Some(Intersection::new(
            intersection,
            t,
            normal,
            self.material_index,
        ))
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        const EPS: f64 = 1e-9;

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

    pub fn build<C: BVHConfig>(self) -> Object {
        let mut mesh = TriangleMesh {
            triangles: self.triangles,
            material_index: self.material_index,
            bvh: BvhTree::new(),
        };
        mesh.bvh = BvhTree::build::<C>(&mesh.triangles);

        Object::TriangleMesh(mesh)
    }

    pub fn material(mut self, index: MaterialIndex) -> Self {
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

    pub fn read_obj_file(mut self, obj: impl AsRef<std::path::Path> + std::fmt::Debug) -> Self {
        let (models, _materials) =
            tobj::load_obj(obj, &tobj::GPU_LOAD_OPTIONS).expect("Failed to load OBJ file");

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

            let uvs: Vec<Vector> = mesh
                .texcoords
                .chunks_exact(2)
                .map(|c| Vector::new(c[0], c[1], 0.0))
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

                let uv = take_index(&uvs);

                self.triangles.push(TriangleSoup::new(vtx, normal, uv));
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
