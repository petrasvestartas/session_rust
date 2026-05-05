use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_mesh_offset_from_mesh() -> TestResult {
    MINI_TEST!("from_mesh", {
        use crate::mesh_offset::MeshOffset;
        use crate::Mesh;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2, 3]]);
        let result = MeshOffset::from_mesh(&mesh, 1.0);
        let copy = result.clone();
        MINI_CHECK!(result.is_valid());
        MINI_CHECK!(result == copy);
        MINI_CHECK!(!(result != copy));
        MINI_CHECK!(result.number_of_vertices() == 8);
        MINI_CHECK!(result.number_of_faces() == 6);
    })
}
REGISTER_MINI_TEST!("MeshOffset", "from_mesh", crate::mesh_offset_test::run_mesh_offset_from_mesh);

pub fn run_mesh_offset_from_mesh_grid() -> TestResult {
    MINI_TEST!("from_mesh_grid", {
        use crate::mesh_offset::MeshOffset;
        use crate::Mesh;
        use crate::Point;
        let pts = vec![
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
        let mesh = Mesh::from_vertices_and_faces(pts, faces);
        let result = MeshOffset::from_mesh(&mesh, 2.0);
        MINI_CHECK!(result.is_valid());
        MINI_CHECK!(result.number_of_vertices() == 18);
        MINI_CHECK!(result.number_of_faces() == 16);
    })
}
REGISTER_MINI_TEST!("MeshOffset", "from_mesh_grid", crate::mesh_offset_test::run_mesh_offset_from_mesh_grid);

pub fn run_mesh_offset_from_mesh_layers() -> TestResult {
    MINI_TEST!("from_mesh_layers", {
        use crate::mesh_offset::MeshOffset;
        use crate::Mesh;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2, 3]]);
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
REGISTER_MINI_TEST!("MeshOffset", "from_mesh_layers", crate::mesh_offset_test::run_mesh_offset_from_mesh_layers);

pub fn run_mesh_offset_file_json_dump() -> TestResult {
    MINI_TEST!("file_json_dump", {
        use crate::mesh_offset::MeshOffset;
        use crate::Mesh;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2, 3]]);
        let result = MeshOffset::from_mesh(&mesh, 1.0);
        result.file_json_dump("mesh_offset_test_dump.json");
        let loaded = Mesh::file_json_load("mesh_offset_test_dump.json").unwrap();
        MINI_CHECK!(loaded.is_valid());
        MINI_CHECK!(loaded.number_of_vertices() == result.number_of_vertices());
        MINI_CHECK!(loaded.number_of_faces() == result.number_of_faces());
    })
}
REGISTER_MINI_TEST!("MeshOffset", "file_json_dump", crate::mesh_offset_test::run_mesh_offset_file_json_dump);

pub fn run_mesh_offset_file_json_load() -> TestResult {
    MINI_TEST!("file_json_load", {
        use crate::Mesh;
        let loaded = Mesh::file_json_load("mesh_offset_test_dump.json").unwrap();
        MINI_CHECK!(loaded.is_valid());
        MINI_CHECK!(loaded.number_of_vertices() == 8);
        MINI_CHECK!(loaded.number_of_faces() == 6);
    })
}
REGISTER_MINI_TEST!("MeshOffset", "file_json_load", crate::mesh_offset_test::run_mesh_offset_file_json_load);

pub fn run_mesh_offset_to_proto() -> TestResult {
    MINI_TEST!("to_proto", {
        use crate::mesh_offset::MeshOffset;
        use crate::Mesh;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2, 3]]);
        let result = MeshOffset::from_mesh(&mesh, 1.0);
        result.pb_dump("mesh_offset_test.pb");
        let loaded = Mesh::pb_load("mesh_offset_test.pb");
        MINI_CHECK!(loaded.is_valid());
        MINI_CHECK!(loaded.number_of_vertices() == result.number_of_vertices());
        MINI_CHECK!(loaded.number_of_faces() == result.number_of_faces());
    })
}
REGISTER_MINI_TEST!("MeshOffset", "to_proto", crate::mesh_offset_test::run_mesh_offset_to_proto);

pub fn run_mesh_offset_from_proto() -> TestResult {
    MINI_TEST!("from_proto", {
        use crate::Mesh;
        let loaded = Mesh::pb_load("mesh_offset_test.pb");
        MINI_CHECK!(loaded.is_valid());
        MINI_CHECK!(loaded.number_of_vertices() == 8);
        MINI_CHECK!(loaded.number_of_faces() == 6);
    })
}
REGISTER_MINI_TEST!("MeshOffset", "from_proto", crate::mesh_offset_test::run_mesh_offset_from_proto);
