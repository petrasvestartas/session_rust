use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_mesh_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Mesh;

        let mesh = Mesh::new();

        let num_vertices = mesh.number_of_vertices();
        let num_faces = mesh.number_of_faces();
        let num_edges = mesh.number_of_edges();
        let is_empty = mesh.is_empty();
        let euler = mesh.euler();

        MINI_CHECK!(num_vertices == 0);
        MINI_CHECK!(num_faces == 0);
        MINI_CHECK!(num_edges == 0);
        MINI_CHECK!(is_empty);
        MINI_CHECK!(euler == 0);
        MINI_CHECK!(mesh.name == "my_mesh");
        MINI_CHECK!(!mesh.guid.is_empty());
    })
}

pub fn run_mesh_add_vertex() -> TestResult {
    MINI_TEST!("Add_vertex", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(1.0, 2.0, 3.0), None);

        MINI_CHECK!(mesh.number_of_vertices() == 1);
        MINI_CHECK!(!mesh.is_empty());

        let pos = mesh.vertex_position(v0).unwrap();
        MINI_CHECK!(pos[0] == 1.0 && pos[1] == 2.0 && pos[2] == 3.0);

        let v1 = mesh.add_vertex(Point::new(4.0, 5.0, 6.0), Some(42));
        MINI_CHECK!(v1 == 42);
        MINI_CHECK!(mesh.number_of_vertices() == 2);
    })
}

pub fn run_mesh_add_face() -> TestResult {
    MINI_TEST!("Add_face", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);

        let f = mesh.add_face(vec![v0, v1, v2], None);
        MINI_CHECK!(f.is_some());
        MINI_CHECK!(mesh.number_of_faces() == 1);
        MINI_CHECK!(mesh.number_of_edges() == 3);
        MINI_CHECK!(mesh.euler() == 1);

        let invalid1 = mesh.add_face(vec![v0, v1], None);
        MINI_CHECK!(invalid1.is_none());

        let invalid2 = mesh.add_face(vec![v0, v1, 999], None);
        MINI_CHECK!(invalid2.is_none());

        let invalid3 = mesh.add_face(vec![v0, v1, v0], None);
        MINI_CHECK!(invalid3.is_none());
    })
}

pub fn run_mesh_face_vertices() -> TestResult {
    MINI_TEST!("Face_vertices", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);

        let f = mesh.add_face(vec![v0, v1, v2], None).unwrap();
        let vertices = mesh.face_vertices(f).unwrap();

        MINI_CHECK!(vertices.len() == 3);
        MINI_CHECK!(vertices[0] == v0);
        MINI_CHECK!(vertices[1] == v1);
        MINI_CHECK!(vertices[2] == v2);
    })
}

pub fn run_mesh_vertex_neighbors() -> TestResult {
    MINI_TEST!("Vertex_neighbors", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);

        mesh.add_face(vec![v0, v1, v2], None);

        let neighbors = mesh.vertex_neighbors(v0);
        MINI_CHECK!(neighbors.len() == 2);
        MINI_CHECK!(neighbors.contains(&v1));
        MINI_CHECK!(neighbors.contains(&v2));
    })
}

pub fn run_mesh_vertex_faces() -> TestResult {
    MINI_TEST!("Vertex_faces", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let v3 = mesh.add_vertex(Point::new(1.0, 1.0, 0.0), None);

        let f1 = mesh.add_face(vec![v0, v1, v2], None).unwrap();
        let f2 = mesh.add_face(vec![v1, v3, v2], None).unwrap();

        let faces = mesh.vertex_faces(v1);
        MINI_CHECK!(faces.len() == 2);
        MINI_CHECK!(faces.contains(&f1));
        MINI_CHECK!(faces.contains(&f2));
    })
}

pub fn run_mesh_is_vertex_on_boundary() -> TestResult {
    MINI_TEST!("Is_vertex_on_boundary", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);

        mesh.add_face(vec![v0, v1, v2], None);

        MINI_CHECK!(mesh.is_vertex_on_boundary(v0));
        MINI_CHECK!(mesh.is_vertex_on_boundary(v1));
        MINI_CHECK!(mesh.is_vertex_on_boundary(v2));
    })
}

pub fn run_mesh_face_normal() -> TestResult {
    MINI_TEST!("Face_normal", {
        use crate::Mesh;
        use crate::Point;
        use crate::tolerance::TOLERANCE;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);

        let f = mesh.add_face(vec![v0, v1, v2], None).unwrap();
        let normal = mesh.face_normal(f).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(normal[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(normal[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(normal[1], 0.0));
    })
}

pub fn run_mesh_face_area() -> TestResult {
    MINI_TEST!("Face_area", {
        use crate::Mesh;
        use crate::Point;
        use crate::tolerance::TOLERANCE;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);

        let f = mesh.add_face(vec![v0, v1, v2], None).unwrap();
        let area = mesh.face_area(f).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(area, 0.5));
    })
}

pub fn run_mesh_from_polygons() -> TestResult {
    MINI_TEST!("From_polygons", {
        use crate::Mesh;
        use crate::Point;

        let triangle = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];

        let mesh = Mesh::from_polygons(vec![triangle], None);
        MINI_CHECK!(mesh.number_of_vertices() == 3);
        MINI_CHECK!(mesh.number_of_faces() == 1);
        MINI_CHECK!(mesh.number_of_edges() == 3);

        let tri1 = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let tri2 = vec![
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ];

        let mesh2 = Mesh::from_polygons(vec![tri1, tri2], None);
        MINI_CHECK!(mesh2.number_of_vertices() == 4);
        MINI_CHECK!(mesh2.number_of_faces() == 2);
    })
}

pub fn run_mesh_clear() -> TestResult {
    MINI_TEST!("Clear", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);

        MINI_CHECK!(!mesh.is_empty());

        mesh.clear();

        MINI_CHECK!(mesh.is_empty());
        MINI_CHECK!(mesh.number_of_vertices() == 0);
        MINI_CHECK!(mesh.number_of_faces() == 0);
    })
}

pub fn run_mesh_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Mesh;
        use crate::Point;
        use crate::Xform;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);

        mesh.xform = Xform::translation(10.0, 20.0, 30.0);
        let mesh_transformed = mesh.transformed();
        mesh.transform();

        let pos0 = mesh.vertex_position(v0).unwrap();
        MINI_CHECK!(pos0[0] == 10.0);
        MINI_CHECK!(pos0[1] == 20.0);
        MINI_CHECK!(pos0[2] == 30.0);
        MINI_CHECK!(mesh.xform == Xform::identity());
        MINI_CHECK!(mesh_transformed.xform == Xform::identity());
    })
}

pub fn run_mesh_json_roundtrip() -> TestResult {
    MINI_TEST!("Json_roundtrip", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        mesh.name = "test_mesh".to_string();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);

        //   json_dumps()    │ String       │ to JSON string
        //   json_loads(s)   │ String       │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        let filename = "serialization/test_mesh.json";
        mesh.json_dump(filename).unwrap();
        let loaded = Mesh::json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == mesh.name);
        MINI_CHECK!(loaded.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded.number_of_faces() == mesh.number_of_faces());
    })
}

pub fn run_mesh_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf_roundtrip", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        mesh.name = "test_mesh_proto".to_string();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);

        let filename = "serialization/test_mesh.bin";
        mesh.pb_dump(filename);
        let loaded = Mesh::pb_load(filename);

        MINI_CHECK!(loaded.name == mesh.name);
        MINI_CHECK!(loaded.number_of_vertices() == mesh.number_of_vertices());
        MINI_CHECK!(loaded.number_of_faces() == mesh.number_of_faces());
        MINI_CHECK!(loaded.guid == mesh.guid);
    })
}

pub fn run_mesh_vertex_position() -> TestResult {
    MINI_TEST!("Vertex_position", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(1.0, 2.0, 3.0), None);

        let pos = mesh.vertex_position(v0).unwrap();
        MINI_CHECK!(pos[0] == 1.0);
        MINI_CHECK!(pos[1] == 2.0);
        MINI_CHECK!(pos[2] == 3.0);
        MINI_CHECK!(mesh.vertex_position(999).is_none());
    })
}

pub fn run_mesh_vertex_normal() -> TestResult {
    MINI_TEST!("Vertex_normal", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let v3 = mesh.add_vertex(Point::new(1.0, 1.0, 0.0), None);

        mesh.add_face(vec![v0, v1, v3], None);
        mesh.add_face(vec![v0, v3, v2], None);

        let normal = mesh.vertex_normal(v0).unwrap();
        MINI_CHECK!(normal[2].abs() == 1.0);
    })
}

pub fn run_mesh_to_vertices_and_faces() -> TestResult {
    MINI_TEST!("To_vertices_and_faces", {
        use crate::Mesh;
        use crate::Point;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);

        mesh.add_face(vec![v0, v1, v2], None);

        let (vertices, faces) = mesh.to_vertices_and_faces();

        MINI_CHECK!(vertices.len() == 3);
        MINI_CHECK!(faces.len() == 1);
        MINI_CHECK!(faces[0].len() == 3);
    })
}

// Register tests with the shared registry
REGISTER_MINI_TEST!("Mesh", "Constructor", crate::mesh_test::run_mesh_constructor);
REGISTER_MINI_TEST!("Mesh", "Add_vertex", crate::mesh_test::run_mesh_add_vertex);
REGISTER_MINI_TEST!("Mesh", "Add_face", crate::mesh_test::run_mesh_add_face);
REGISTER_MINI_TEST!("Mesh", "Face_vertices", crate::mesh_test::run_mesh_face_vertices);
REGISTER_MINI_TEST!("Mesh", "Vertex_neighbors", crate::mesh_test::run_mesh_vertex_neighbors);
REGISTER_MINI_TEST!("Mesh", "Vertex_faces", crate::mesh_test::run_mesh_vertex_faces);
REGISTER_MINI_TEST!("Mesh", "Is_vertex_on_boundary", crate::mesh_test::run_mesh_is_vertex_on_boundary);
REGISTER_MINI_TEST!("Mesh", "Face_normal", crate::mesh_test::run_mesh_face_normal);
REGISTER_MINI_TEST!("Mesh", "Face_area", crate::mesh_test::run_mesh_face_area);
REGISTER_MINI_TEST!("Mesh", "From_polygons", crate::mesh_test::run_mesh_from_polygons);
REGISTER_MINI_TEST!("Mesh", "Clear", crate::mesh_test::run_mesh_clear);
REGISTER_MINI_TEST!("Mesh", "Transformation", crate::mesh_test::run_mesh_transformation);
REGISTER_MINI_TEST!("Mesh", "Vertex_position", crate::mesh_test::run_mesh_vertex_position);
REGISTER_MINI_TEST!("Mesh", "Vertex_normal", crate::mesh_test::run_mesh_vertex_normal);
REGISTER_MINI_TEST!("Mesh", "To_vertices_and_faces", crate::mesh_test::run_mesh_to_vertices_and_faces);
REGISTER_MINI_TEST!("Mesh", "Json_roundtrip", crate::mesh_test::run_mesh_json_roundtrip);
REGISTER_MINI_TEST!("Mesh", "Protobuf_roundtrip", crate::mesh_test::run_mesh_protobuf_roundtrip);
