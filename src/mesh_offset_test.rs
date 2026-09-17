use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_mesh_offset_from_mesh() -> TestResult {
    MINI_TEST!("From Mesh", {
        use crate::Mesh;
        use crate::MeshOffset;
        use crate::Point;
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(points, vec![vec![0, 1, 2, 3]]);
        let result = MeshOffset::from_mesh(&mesh, 1.0);
        let copy = result.clone();

        MINI_CHECK!(result.is_valid());
        MINI_CHECK!(result.is_closed());
        MINI_CHECK!(result == copy);
        MINI_CHECK!(!(result != copy));
        MINI_CHECK!(result.number_of_vertices() == 8);
        MINI_CHECK!(result.number_of_faces() == 6);
    })
}

REGISTER_MINI_TEST!(
    "MeshOffset",
    "From Mesh",
    crate::mesh_offset_test::run_mesh_offset_from_mesh
);

pub fn run_mesh_offset_from_mesh_grid() -> TestResult {
    MINI_TEST!("From Mesh Grid", {
        use crate::Mesh;
        use crate::MeshOffset;
        use crate::Point;
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
        ];
        let faces = vec![
            vec![0, 1, 4, 3],
            vec![1, 2, 5, 4],
            vec![3, 4, 7, 6],
            vec![4, 5, 8, 7],
        ];
        let mesh = Mesh::from_vertices_and_faces(points, faces);
        let result = MeshOffset::from_mesh(&mesh, 2.0);

        MINI_CHECK!(result.is_valid());
        MINI_CHECK!(result.is_closed());
        MINI_CHECK!(result.number_of_vertices() == 18);
        MINI_CHECK!(result.number_of_faces() == 16);
    })
}

REGISTER_MINI_TEST!(
    "MeshOffset",
    "From Mesh Grid",
    crate::mesh_offset_test::run_mesh_offset_from_mesh_grid
);

pub fn run_mesh_offset_from_mesh_layers() -> TestResult {
    MINI_TEST!("From Mesh Layers", {
        use crate::Mesh;
        use crate::MeshOffset;
        use crate::Point;
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(points, vec![vec![0, 1, 2, 3]]);
        let layers = MeshOffset::from_mesh_layers(&mesh, 1.0);

        MINI_CHECK!(layers.bottom.is_valid());
        MINI_CHECK!(layers.top.is_valid());
        MINI_CHECK!(layers.sides.is_valid());
        MINI_CHECK!(layers.bottom.number_of_vertices() == 4);
        MINI_CHECK!(layers.bottom.number_of_faces() == 1);
        MINI_CHECK!(layers.top.number_of_vertices() == 4);
        MINI_CHECK!(layers.top.number_of_faces() == 1);
        MINI_CHECK!(layers.sides.number_of_faces() == 4);
    })
}

REGISTER_MINI_TEST!(
    "MeshOffset",
    "From Mesh Layers",
    crate::mesh_offset_test::run_mesh_offset_from_mesh_layers
);

pub fn run_mesh_offset_offset_planes() -> TestResult {
    MINI_TEST!("Offset Planes", {
        use crate::Mesh;
        use crate::MeshOffset;
        use crate::Point;
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(points, vec![vec![0, 1, 2, 3]]);
        let planes = MeshOffset::offset_planes(&mesh, 1.0);

        MINI_CHECK!(planes.len() == 1);
        let plane = &planes[&0];

        MINI_CHECK!(TOLERANCE.is_close(plane.a(), 0.0));
        MINI_CHECK!(TOLERANCE.is_close(plane.b(), 0.0));
        MINI_CHECK!(TOLERANCE.is_close(plane.c(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(plane.d(), -1.0));
        MINI_CHECK!(TOLERANCE.is_close(plane.origin()[2], 1.0));
    })
}

REGISTER_MINI_TEST!(
    "MeshOffset",
    "Offset Planes",
    crate::mesh_offset_test::run_mesh_offset_offset_planes
);

pub fn run_mesh_offset_offset_vertices() -> TestResult {
    MINI_TEST!("Offset Vertices", {
        use crate::Mesh;
        use crate::MeshOffset;
        use crate::Point;
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
        ];
        let faces = vec![
            vec![0, 1, 4, 3],
            vec![1, 2, 5, 4],
            vec![3, 4, 7, 6],
            vec![4, 5, 8, 7],
        ];
        let mesh = Mesh::from_vertices_and_faces(points.clone(), faces);
        let planes = MeshOffset::offset_planes(&mesh, 2.0);
        let offsets = MeshOffset::offset_vertices(&mesh, &planes);

        MINI_CHECK!(planes.len() == 4);
        MINI_CHECK!(offsets.len() == 9);
        for vkey in 0..9 {
            MINI_CHECK!(TOLERANCE.is_close(offsets[&vkey][0], points[vkey][0]));
            MINI_CHECK!(TOLERANCE.is_close(offsets[&vkey][1], points[vkey][1]));
            MINI_CHECK!(TOLERANCE.is_close(offsets[&vkey][2], 2.0));
        }
    })
}

REGISTER_MINI_TEST!(
    "MeshOffset",
    "Offset Vertices",
    crate::mesh_offset_test::run_mesh_offset_offset_vertices
);

pub fn run_mesh_offset_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Mesh;
        use crate::MeshOffset;
        use crate::Point;
        use std::path::PathBuf;
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(points, vec![vec![0, 1, 2, 3]]);
        let result = MeshOffset::from_mesh(&mesh, 1.0);
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization")
            .join("test_mesh_offset.json");
        result.file_json_dump(filename.to_str().unwrap()).unwrap();
        let loaded = Mesh::file_json_load(filename.to_str().unwrap()).unwrap();

        MINI_CHECK!(loaded == result);
        MINI_CHECK!(loaded.number_of_vertices() == 8);
        MINI_CHECK!(loaded.number_of_faces() == 6);
    })
}

REGISTER_MINI_TEST!(
    "MeshOffset",
    "Json Roundtrip",
    crate::mesh_offset_test::run_mesh_offset_json_roundtrip
);

pub fn run_mesh_offset_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Mesh;
        use crate::MeshOffset;
        use crate::Point;
        use std::path::PathBuf;
        let points = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(points, vec![vec![0, 1, 2, 3]]);
        let result = MeshOffset::from_mesh(&mesh, 1.0);
        let filename = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("serialization")
            .join("test_mesh_offset.bin");
        result.pb_dump(filename.to_str().unwrap());
        let loaded = Mesh::pb_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded == result);
        MINI_CHECK!(loaded.number_of_vertices() == 8);
        MINI_CHECK!(loaded.number_of_faces() == 6);
    })
}

REGISTER_MINI_TEST!(
    "MeshOffset",
    "Protobuf Roundtrip",
    crate::mesh_offset_test::run_mesh_offset_protobuf_roundtrip
);
