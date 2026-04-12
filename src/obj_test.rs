use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;


pub fn run_obj_read_bunny() -> TestResult {
    MINI_TEST!("Read Bunny", {
        // load Stanford Bunny (real-world OBJ: 2503 vertices, 4968 faces)
        use std::path::PathBuf;
        use crate::obj::read_obj;
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let bunny_path = src_dir.parent().unwrap().join("session_data").join("bunny.obj");
        if !bunny_path.exists() {
            return Ok(());
        }
        let mesh = read_obj(bunny_path.to_str().unwrap()).unwrap();

        MINI_CHECK!(mesh.number_of_vertices() == 2503);
        MINI_CHECK!(mesh.number_of_faces() == 4968);
        let (vertices, faces) = mesh.to_vertices_and_faces();
        MINI_CHECK!(vertices.len() == 2503);
        MINI_CHECK!(faces.len() == 4968);
        let has_non_zero = vertices.iter().any(|v| v[0] != 0.0 || v[1] != 0.0 || v[2] != 0.0);
        MINI_CHECK!(has_non_zero);
        MINI_CHECK!(faces.iter().all(|f| f.len() >= 3));
    })
}

pub fn run_obj_write_read_roundtrip() -> TestResult {
    MINI_TEST!("Write Read Roundtrip", {
        // build a small mesh (4 verts, 2 faces), write to OBJ, read back, compare counts
        use crate::{Mesh, Point};
        use crate::obj::{read_obj, write_obj};
        use std::path::PathBuf;
        let mut original_mesh = Mesh::new();
        let v0 = original_mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = original_mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = original_mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let v3 = original_mesh.add_vertex(Point::new(0.0, 0.0, 1.0), None);
        let _ = original_mesh.add_face(vec![v0, v1, v2], None);
        let _ = original_mesh.add_face(vec![v0, v1, v3], None);

        MINI_CHECK!(original_mesh.number_of_vertices() == 4);
        MINI_CHECK!(original_mesh.number_of_faces() == 2);
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let temp_file = src_dir.join("serialization").join("test_temp_roundtrip.obj");
        let temp_str = temp_file.to_str().unwrap();
        write_obj(&original_mesh, temp_str).unwrap();
        MINI_CHECK!(temp_file.exists());
        let loaded_mesh = read_obj(temp_str).unwrap();
        MINI_CHECK!(loaded_mesh.number_of_vertices() == original_mesh.number_of_vertices());
        MINI_CHECK!(loaded_mesh.number_of_faces() == original_mesh.number_of_faces());
        let _ = std::fs::remove_file(&temp_file);
    })
}

REGISTER_MINI_TEST!("OBJ", "Read Bunny", crate::obj_test::run_obj_read_bunny);
REGISTER_MINI_TEST!("OBJ", "Write Read Roundtrip", crate::obj_test::run_obj_write_read_roundtrip);
