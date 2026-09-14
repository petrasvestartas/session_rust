use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_file_obj_read_bunny() -> TestResult {
    MINI_TEST!("Read Bunny", {
        use crate::read_file_obj;
        use std::path::PathBuf;
        let bunny_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .join("session_data")
            .join("bunny.obj");
        if !bunny_path.exists() {
            return Ok(());
        }
        let mesh = read_file_obj(bunny_path.to_str().unwrap()).unwrap();

        MINI_CHECK!(mesh.number_of_vertices() == 2503);
        MINI_CHECK!(mesh.number_of_faces() == 4968);
        let (vertices, faces) = mesh.to_vertices_and_faces();
        MINI_CHECK!(vertices.len() == 2503);
        MINI_CHECK!(faces.len() == 4968);
        let mut has_non_zero = false;
        for v in vertices.iter() {
            if v[0] != 0.0 || v[1] != 0.0 || v[2] != 0.0 {
                has_non_zero = true;
            }
        }
        MINI_CHECK!(has_non_zero);
        let mut all_polygons = true;
        for f in faces.iter() {
            if f.len() < 3 {
                all_polygons = false;
            }
        }
        MINI_CHECK!(all_polygons);
    })
}

pub fn run_file_obj_write_read_roundtrip() -> TestResult {
    MINI_TEST!("Write Read Roundtrip", {
        use crate::read_file_obj;
        use crate::write_file_obj;
        use crate::Mesh;
        use crate::Point;
        use std::path::PathBuf;
        std::fs::create_dir_all("./serialization").unwrap();
        let mut original_mesh = Mesh::new();
        let v0 = original_mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = original_mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = original_mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let v3 = original_mesh.add_vertex(Point::new(0.0, 0.0, 1.0), None);
        original_mesh.add_face(vec![v0, v1, v2], None);
        original_mesh.add_face(vec![v0, v1, v3], None);

        MINI_CHECK!(original_mesh.number_of_vertices() == 4);
        MINI_CHECK!(original_mesh.number_of_faces() == 2);
        let temp_file = "./serialization/test_temp_roundtrip.obj";
        write_file_obj(&original_mesh, temp_file).unwrap();
        MINI_CHECK!(PathBuf::from(temp_file).exists());
        let loaded_mesh = read_file_obj(temp_file).unwrap();
        MINI_CHECK!(loaded_mesh.number_of_vertices() == original_mesh.number_of_vertices());
        MINI_CHECK!(loaded_mesh.number_of_faces() == original_mesh.number_of_faces());
        std::fs::remove_file(temp_file).unwrap();
    })
}

pub fn run_file_obj_string_roundtrip() -> TestResult {
    MINI_TEST!("String Roundtrip", {
        use crate::read_file_obj_from_str;
        use crate::write_file_obj_to_string;
        use crate::Mesh;
        use crate::Point;
        let mut original_mesh = Mesh::new();
        let v0 = original_mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = original_mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = original_mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let v3 = original_mesh.add_vertex(Point::new(0.0, 0.0, 1.0), None);
        original_mesh.add_face(vec![v0, v1, v2], None);
        original_mesh.add_face(vec![v0, v1, v3], None);
        let s = write_file_obj_to_string(&original_mesh);
        let loaded_mesh = read_file_obj_from_str(&s);

        MINI_CHECK!(loaded_mesh.number_of_vertices() == original_mesh.number_of_vertices());
        MINI_CHECK!(loaded_mesh.number_of_faces() == original_mesh.number_of_faces());
        MINI_CHECK!(TOLERANCE.is_close(loaded_mesh.area(), original_mesh.area()));
    })
}

REGISTER_MINI_TEST!(
    "FileObj",
    "Read Bunny",
    crate::file_obj_test::run_file_obj_read_bunny
);
REGISTER_MINI_TEST!(
    "FileObj",
    "Write Read Roundtrip",
    crate::file_obj_test::run_file_obj_write_read_roundtrip
);
REGISTER_MINI_TEST!(
    "FileObj",
    "String Roundtrip",
    crate::file_obj_test::run_file_obj_string_roundtrip
);
