mod intersection;
mod mesh;
mod object;
mod quad;
mod sampling;
mod sphere;

pub use intersection::{ComputeIntersection, Intersection};

use sampling::Sampleable;

pub use object::Object;

use mesh::TriangleMesh;
use quad::Quad;
use sphere::Sphere;

pub use mesh::TriangleMeshBuilder;
pub use quad::QuadBuilder;
pub use sphere::SphereBuilder;
