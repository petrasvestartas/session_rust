//! Cross-language geometry library with Point, Color, and Vector types.
//! Supports JSON serialization for interoperability between Rust, Python, and C++.

// Module declarations - makes modules publicly accessible
// Usage: session_rust::point::Point
#![allow(static_mut_refs)]

pub mod proto {
    include!("proto/session_proto.rs");
}


pub mod prelude {
    pub use crate::session::{Geometry, RayHit, Session};
    pub use crate::{
        Color, Line, Mesh, NurbsCurve, NurbsSurface, Plane, Point, PointCloud,
        Polyline, Vector, OBB,
    };
}

pub mod aabb;
pub mod aabb_test;
pub mod guid_serde;
pub mod obb;
pub mod brep;
pub mod brep_test;
pub mod element;
pub mod element_test;
pub mod element_beam_test;
pub mod element_column_test;
pub mod element_plate_test;
pub mod spatial_aabbtree;
pub mod spatial_aabbtree_test;
pub mod spatial_bvh;
pub mod spatial_bvh_test;
pub mod closest;
pub mod closest_test;
pub mod color;
pub mod file_encoders;
pub mod file_encoders_test;
pub mod graph;
pub mod intersection;
pub mod intersection_test;
pub mod nurbsknot;
pub mod nurbsknot_test;
pub mod line;
pub mod instance_ref;
pub mod mesh;
pub mod render_mesh;
pub mod nurbscurve;
pub mod nurbssurface;
#[cfg(all(feature = "pdf", not(target_arch = "wasm32")))]
pub mod pdf;
pub mod file_obj;
pub mod file_obj_test;
pub mod io;
pub mod io_test;
pub mod remesh_cdt;
pub mod remesh_cdt_test;
pub mod nurbssurface_trimmed;
pub mod nurbssurface_trimmed_test;
pub mod objects;
pub mod plane;
pub mod point;
pub mod pointcloud;
pub mod polyline;
pub mod boolean_polyline;
pub mod primitives;
pub mod quaternion;
pub mod remesh_nurbssurface_adaptive;
pub mod remesh_nurbssurface_adaptive_test;
pub mod remesh_nurbssurface_grid;
pub mod remesh_nurbssurface_grid_test;
pub mod spatial_rtree;
pub mod spatial_rtree_test;
pub mod session;
pub mod session_config;
pub mod tolerance;
pub mod tree;
pub mod vector;
pub mod xform;
pub mod mini_test;
pub mod color_test;
pub mod point_test;
pub mod vector_test;
pub mod session_config_test;
pub mod tolerance_test;
pub mod line_test;
pub mod instance_ref_test;
pub mod polyline_test;
pub mod plane_test;
pub mod pointcloud_test;
pub mod xform_test;
pub mod mesh_test;
pub mod nurbscurve_test;
pub mod nurbssurface_test;
pub mod primitives_test;
pub mod session_test;
pub mod quaternion_test;
pub mod obb_test;
pub mod graph_test;
pub mod objects_test;
pub mod tree_test;
pub mod matrix;
pub mod matrix_test;
pub mod convex_hull;
pub mod convex_hull_test;
pub mod mesh_offset;
pub mod mesh_offset_test;
pub mod spatial_kdtree;
pub mod spatial_kdtree_test;
pub mod spatial_octree;
pub mod spatial_octree_test;
pub mod boolean_polyline_test;
pub mod picking_test;

pub use aabb::AABB;
pub use obb::OBB;
pub use brep::BRep;
pub use spatial_aabbtree::SpatialAABBTree;
pub use spatial_bvh::SpatialBVH;
pub use closest::Closest;
pub use color::Color;
pub use element::Element;
pub use graph::{Edge, Graph};
pub use line::Line;
pub use instance_ref::InstanceRef;
pub use mesh::{Mesh, LoftPanel, LoftWallFace};
pub use render_mesh::{RenderMesh, RenderVertex, GpuMesh, GpuCache};
pub use nurbscurve::NurbsCurve;
pub use nurbssurface::NurbsSurface;
pub use nurbssurface_trimmed::NurbsSurfaceTrimmed;
pub use file_obj::{read_file_obj, write_file_obj, read_file_obj_polylines};
pub use io::{read_xyz, write_xyz, read_xyz_from_str, write_xyz_to_string};
pub use objects::{Objects, Component};
pub use plane::Plane;
pub use point::Point;
pub use pointcloud::PointCloud;
pub use polyline::Polyline;
pub use primitives::Primitives;
pub use quaternion::Quaternion;
pub use remesh_nurbssurface_adaptive::RemeshNurbsSurfaceAdaptive;
pub use remesh_nurbssurface_grid::remesh_nurbssurface_grid;
pub use spatial_rtree::SpatialRTree;
pub use session::{Geometry, Session};
pub use session_config::SessionConfig;
pub use tolerance::Tolerance;
pub use tree::{Tree, TreeNode};
pub use vector::Vector;
pub use graph::Vertex;
pub use xform::Xform;
pub use matrix::Matrix;
pub use convex_hull::ConvexHull;
pub use mesh_offset::{MeshOffset, MeshOffsetLayers};
pub use spatial_kdtree::SpatialKDTree;
pub use spatial_octree::SpatialOctree;
