//! Cross-language geometry library with Point, Color, and Vector types.
//! Supports JSON serialization for interoperability between Rust, Python, and C++.

// Module declarations - makes modules publicly accessible
// Usage: session_rust::point::Point
#![allow(static_mut_refs)]

pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/session_proto.rs"));
}

pub mod boundingbox;
pub mod bvh;
#[cfg(test)]
mod bvh_test;
pub mod closest;
pub mod color;
pub mod edge;
pub mod encoders;
pub mod graph;
pub mod intersection;
#[cfg(test)]
mod intersection_test;
pub mod knot;
pub mod knot_test;
pub mod line;
pub mod mesh;
pub mod nurbscurve;
pub mod nurbssurface;
pub mod obj;
pub mod trimmedsurface;
pub mod trimmedsurface_test;
pub mod objects;
pub mod plane;
pub mod point;
pub mod pointcloud;
pub mod polyline;
pub mod primitives;
pub mod quaternion;
pub mod session;
pub mod tolerance;
pub mod tree;
pub mod treenode;
pub mod vector;
pub mod vertex;
pub mod xform;
pub mod mini_test;
pub mod color_test;
pub mod point_test;
pub mod vector_test;
pub mod tolerance_test;
pub mod line_test;
pub mod polyline_test;
pub mod plane_test;
pub mod pointcloud_test;
pub mod xform_test;
pub mod mesh_test;
pub mod nurbscurve_test;
pub mod nurbssurface_test;
pub mod primitives_test;

pub use boundingbox::BoundingBox;
pub use bvh::BVH;
pub use closest::Closest;
pub use color::Color;
pub use edge::Edge;
pub use graph::Graph;
pub use line::Line;
pub use mesh::Mesh;
pub use nurbscurve::NurbsCurve;
pub use nurbssurface::NurbsSurface;
pub use trimmedsurface::TrimmedSurface;
pub use obj::{read_obj, write_obj};
pub use objects::Objects;
pub use plane::Plane;
pub use point::Point;
pub use pointcloud::PointCloud;
pub use polyline::Polyline;
pub use primitives::Primitives;
pub use quaternion::Quaternion;
pub use session::{Geometry, Session};
pub use tolerance::Tolerance;
pub use tree::Tree;
pub use treenode::TreeNode;
pub use vector::Vector;
pub use vertex::Vertex;
pub use xform::Xform;
