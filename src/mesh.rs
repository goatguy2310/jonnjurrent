use crate::{
    bvh::BoundingBox,
    material::MaterialIndex,
    object::{ComputeIntersection, Intersection, Object},
    ray::Ray,
    vector::Vector,
};

#[derive(Debug, Default, Clone)]
struct TriangleIndices {
    vtx_indices: [usize; 3],
    #[allow(unused)]
    normal_indices: [usize; 3],

    #[allow(unused)]
    uv_indices: [usize; 3],
}

impl TriangleIndices {
    pub fn new(
        vtx_indices: [usize; 3],
        uv_indices: [usize; 3],
        normal_indices: [usize; 3],
    ) -> Self {
        TriangleIndices {
            vtx_indices,
            normal_indices,
            uv_indices,
        }
    }
}

#[derive(Debug)]
pub struct TriangleMesh {
    indices: Vec<TriangleIndices>,
    vertices: Vec<Vector>,
    #[allow(unused)]
    normals: Vec<Vector>,

    #[allow(unused)]
    uvs: Vec<Vector>,
    #[allow(unused)]
    vertex_colors: Vec<Vector>,

    material_index: MaterialIndex,
    bbox: BoundingBox,
}

impl TriangleMesh {
    pub fn update_bbox(&mut self) {
        let mut min = self.vertices[0].clone();
        let mut max = self.vertices[0].clone();

        for v in self.vertices.iter().skip(1) {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);

            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }

        self.bbox.update(min, max);
    }

    pub fn scale_translate(&mut self, scale: f64, translate: &Vector) {
        self.vertices.iter_mut().for_each(|vertex| {
            *vertex = &*vertex * scale + translate;
        });

        self.update_bbox();
    }
}

impl ComputeIntersection for TriangleMesh {
    type Index = MaterialIndex;

    fn intersect(&self, ray: &Ray) -> Option<Intersection<Self::Index>> {
        const EPS: f64 = 1e-9;

        if !self.bbox.is_intersecting(ray, f64::EPSILON, f64::INFINITY) {
            return None;
        }

        let hit = self
            .indices
            .iter()
            .enumerate()
            .filter_map(|(index, indices)| {
                let vertex_a = &self.vertices[indices.vtx_indices[0]];
                let vertex_b = &self.vertices[indices.vtx_indices[1]];
                let vertex_c = &self.vertices[indices.vtx_indices[2]];

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

                Some((index, t, beta, gamma, n))
            })
            .min_by(|t_1, t_2| t_1.1.total_cmp(&t_2.1))?;

        let (_index, t, beta, gamma, _n) = hit;

        let _alpha = 1. - beta - gamma;
        let intersection = ray.at(t);

        // let indices = &self.indices[index];
        // let normal_a = &self.normals[indices.normal_indices[0]];
        // let normal_b = &self.normals[indices.normal_indices[1]];
        // let normal_c = &self.normals[indices.normal_indices[2]];
        //
        // let normal = (normal_a * alpha + normal_b * beta + normal_c * gamma).normalize();
        //
        // Some(Intersection::new(intersection, t, normal, self.material_index))
        Some(Intersection::new(
            intersection,
            t,
            _n.normalize(),
            self.material_index,
        ))
    }

    fn shadow_intersect(&self, ray: &Ray) -> Option<f64> {
        const EPS: f64 = 1e-9;

        if !self.bbox.is_intersecting(ray, f64::EPSILON, f64::INFINITY) {
            return None;
        }

        self.indices
            .iter()
            .filter_map(|indices| {
                let vertex_a = &self.vertices[indices.vtx_indices[0]];
                let vertex_b = &self.vertices[indices.vtx_indices[1]];
                let vertex_c = &self.vertices[indices.vtx_indices[2]];

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

                Some(t)
            })
            .min_by(|t_1, t_2| t_1.total_cmp(t_2))
    }
}

#[derive(Debug)]
pub struct TriangleMeshBuilder {
    indices: Vec<TriangleIndices>,
    vertices: Vec<Vector>,
    normals: Vec<Vector>,
    uvs: Vec<Vector>,
    vertex_colors: Vec<Vector>,

    material_index: MaterialIndex,
    bbox: BoundingBox,
}

impl TriangleMeshBuilder {
    pub fn new() -> Self {
        TriangleMeshBuilder {
            indices: Vec::new(),
            vertices: Vec::new(),
            normals: Vec::new(),
            uvs: Vec::new(),
            vertex_colors: Vec::new(),
            material_index: MaterialIndex(0),
            bbox: BoundingBox::new(Vector::default(), Vector::default()),
        }
    }

    pub fn build(self) -> Object {
        let mut mesh = TriangleMesh {
            indices: self.indices,
            vertices: self.vertices,
            normals: self.normals,
            uvs: self.uvs,
            vertex_colors: self.vertex_colors,
            material_index: self.material_index,
            bbox: self.bbox,
        };
        mesh.update_bbox();

        Object::TriangleMesh(mesh)
    }

    pub fn material(mut self, index: MaterialIndex) -> Self {
        self.material_index = index;
        self
    }

    pub fn scale_translate(mut self, scale: f64, translate: impl Into<Vector>) -> Self {
        let translate_ref = &translate.into();
        self.vertices.iter_mut().for_each(|vertex| {
            *vertex = &*vertex * scale + translate_ref;
        });

        self
    }

    // #[cfg(feature = "obj-support")]
    pub fn read_obj_file(mut self, obj: impl AsRef<std::path::Path> + std::fmt::Debug) -> Self {
        let (models, _materials) =
            tobj::load_obj(obj, &tobj::LoadOptions::default()).expect("Failed to load OBJ file");

        for model in models {
            let mesh = &model.mesh;

            for i in (0..mesh.positions.len()).step_by(3) {
                self.vertices.push(Vector::new(
                    mesh.positions[i] as f64,
                    mesh.positions[i + 1] as f64,
                    mesh.positions[i + 2] as f64,
                ));
            }

            for i in (0..mesh.normals.len()).step_by(3) {
                self.normals.push(Vector::new(
                    mesh.normals[i] as f64,
                    mesh.normals[i + 1] as f64,
                    mesh.normals[i + 2] as f64,
                ));
            }

            for i in (0..mesh.texcoords.len()).step_by(2) {
                self.uvs.push(Vector::new(
                    mesh.texcoords[i] as f64,
                    mesh.texcoords[i + 1] as f64,
                    0.0,
                ));
            }

            for i in (0..mesh.indices.len()).step_by(3) {
                let vtx_idx = [
                    mesh.indices[i] as usize,
                    mesh.indices[i + 1] as usize,
                    mesh.indices[i + 2] as usize,
                ];

                let uv_idx = if !mesh.texcoord_indices.is_empty() {
                    [
                        mesh.texcoord_indices[i] as usize,
                        mesh.texcoord_indices[i + 1] as usize,
                        mesh.texcoord_indices[i + 2] as usize,
                    ]
                } else {
                    [0; 3]
                };

                let normal_idx = if !mesh.normal_indices.is_empty() {
                    [
                        mesh.normal_indices[i] as usize,
                        mesh.normal_indices[i + 1] as usize,
                        mesh.normal_indices[i + 2] as usize,
                    ]
                } else {
                    [0; 3]
                };

                self.indices
                    .push(TriangleIndices::new(vtx_idx, uv_idx, normal_idx));
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
