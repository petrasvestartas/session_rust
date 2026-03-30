use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_session_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Session;
        let session = Session::default();

        MINI_CHECK!(session.name == "my_session");
        MINI_CHECK!(!session.guid().is_empty());
    })
}

pub fn run_session_jsondump() -> TestResult {
    MINI_TEST!("Jsondump", {
        use crate::{Session, Point};
        use crate::encoders::json_dump;
        let mut session = Session::default();
        let point1 = Point::new(1.0, 2.0, 3.0);
        let point2 = Point::new(4.0, 5.0, 6.0);
        let _ = (point1.guid(), point2.guid());
        session.add_point(point1.clone());
        session.add_point(point2.clone());
        session.add_edge(point1.guid(), point2.guid(), "connection");
        let json_str = session.jsondump().unwrap();
        let data: serde_json::Value = serde_json::from_str(&json_str).unwrap();

        MINI_CHECK!(data["name"] == "my_session");
        MINI_CHECK!(!data["guid"].is_null());
        MINI_CHECK!(data["objects"]["points"].as_array().unwrap().len() == 2);
        MINI_CHECK!(data["graph"]["vertices"].as_array().unwrap().len() == 2);
        MINI_CHECK!(data["graph"]["edges"].as_array().unwrap().len() == 1);
        json_dump(&session, "serialization/test_session.json", true).unwrap();
    })
}

pub fn run_session_jsonload() -> TestResult {
    MINI_TEST!("Jsonload", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point1 = Point::new(1.0, 2.0, 3.0);
        let point2 = Point::new(4.0, 5.0, 6.0);
        let _ = (point1.guid(), point2.guid());
        session.add_point(point1.clone());
        session.add_point(point2.clone());
        session.add_edge(point1.guid(), point2.guid(), "connection");
        let json_str = session.jsondump().unwrap();
        let session2 = Session::jsonload(&json_str).unwrap();

        MINI_CHECK!(session2.name == "my_session");
        MINI_CHECK!(session2.lookup.len() == 2);
        MINI_CHECK!(session2.graph.number_of_vertices() == 2);
    })
}

pub fn run_session_file_io() -> TestResult {
    MINI_TEST!("File Io", {
        use crate::{Session, Point};
        use std::fs;
        let mut session = Session::default();
        let point1 = Point::new(1.0, 2.0, 3.0);
        let point2 = Point::new(4.0, 5.0, 6.0);
        session.add_point(point1.clone());
        session.add_point(point2.clone());
        session.add_edge(point1.guid(), point2.guid(), "connection");
        let filename = "serialization/test_session_roundtrip.json";
        let json_str = session.jsondump().unwrap();
        fs::write(filename, &json_str).unwrap();
        let loaded_str = fs::read_to_string(filename).unwrap();
        let loaded_session = Session::jsonload(&loaded_str).unwrap();

        MINI_CHECK!(loaded_session.name == session.name);
        MINI_CHECK!(loaded_session.lookup.len() == session.lookup.len());
        MINI_CHECK!(loaded_session.graph.number_of_vertices() == session.graph.number_of_vertices());
        fs::remove_file(filename).ok();
    })
}

pub fn run_session_add_point() -> TestResult {
    MINI_TEST!("Add Point", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let _ = point.guid();
        session.add_point(point.clone());

        MINI_CHECK!(session.objects.points.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(point.guid()));
        MINI_CHECK!(session.graph.has_node(point.guid()));
    })
}

pub fn run_session_add_edge() -> TestResult {
    MINI_TEST!("Add Edge", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point1 = Point::new(1.0, 2.0, 3.0);
        let point2 = Point::new(4.0, 5.0, 6.0);
        session.add_point(point1.clone());
        session.add_point(point2.clone());
        session.add_edge(point1.guid(), point2.guid(), "connection");

        MINI_CHECK!(session.graph.has_edge((point1.guid(), point2.guid())));
    })
}

pub fn run_session_get_object() -> TestResult {
    MINI_TEST!("Get Object", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let _ = point.guid();
        session.add_point(point.clone());
        let retrieved = session.get_object(point.guid());

        MINI_CHECK!(retrieved.is_some());
        MINI_CHECK!(retrieved.unwrap().guid() == point.guid());
    })
}

pub fn run_session_file_io_comprehensive() -> TestResult {
    MINI_TEST!("File Io Comprehensive", {
        use crate::{Session, Point};
        use crate::encoders::{json_dump, json_load};
        use std::fs;
        let mut session = Session::new("./serialization/test_session");
        let point1 = Point::new(1.0, 2.0, 3.0);
        let point2 = Point::new(4.0, 5.0, 6.0);
        session.add_point(point1.clone());
        session.add_point(point2.clone());
        session.add_edge(point1.guid(), point2.guid(), "./serialization/test_connection");
        let filename = "serialization/test_session_comprehensive.json";
        json_dump(&session, filename, true).unwrap();
        let loaded_session: Session = json_load(filename).unwrap();

        MINI_CHECK!(loaded_session.name == session.name);
        MINI_CHECK!(loaded_session.objects.points.len() == session.objects.points.len());
        MINI_CHECK!(loaded_session.graph.number_of_vertices() == session.graph.number_of_vertices());
        MINI_CHECK!(loaded_session.graph.number_of_edges() == session.graph.number_of_edges());
        fs::remove_file(filename).ok();
    })
}

pub fn run_session_tree_transformation_hierarchy() -> TestResult {
    MINI_TEST!("Tree Transformation Hierarchy", {
        use crate::{Session, Point, Vector, Mesh, Plane, Xform};
        use std::f64::consts::PI;
        let mut scene = Session::new("tree_transformation_test");

        let create_box = |cx: f64, cy: f64, cz: f64, size: f64| -> Mesh {
            let mut mesh = Mesh::new();
            let h = size * 0.5;
            let vkeys = [
                mesh.add_vertex(Point::new(cx - h, cy - h, cz - h), None),
                mesh.add_vertex(Point::new(cx + h, cy - h, cz - h), None),
                mesh.add_vertex(Point::new(cx + h, cy + h, cz - h), None),
                mesh.add_vertex(Point::new(cx - h, cy + h, cz - h), None),
                mesh.add_vertex(Point::new(cx - h, cy - h, cz + h), None),
                mesh.add_vertex(Point::new(cx + h, cy - h, cz + h), None),
                mesh.add_vertex(Point::new(cx + h, cy + h, cz + h), None),
                mesh.add_vertex(Point::new(cx - h, cy + h, cz + h), None),
            ];
            mesh.add_face(vec![vkeys[0], vkeys[1], vkeys[2], vkeys[3]], None);
            mesh.add_face(vec![vkeys[4], vkeys[7], vkeys[6], vkeys[5]], None);
            mesh.add_face(vec![vkeys[0], vkeys[4], vkeys[5], vkeys[1]], None);
            mesh.add_face(vec![vkeys[2], vkeys[6], vkeys[7], vkeys[3]], None);
            mesh.add_face(vec![vkeys[0], vkeys[3], vkeys[7], vkeys[4]], None);
            mesh.add_face(vec![vkeys[1], vkeys[5], vkeys[6], vkeys[2]], None);
            mesh
        };

        let mut box1 = create_box(0.0, 0.0, 0.0, 2.0);
        let box1_node = scene.add_mesh(box1.clone());
        let mut box2 = create_box(0.0, 0.0, 0.0, 2.0);
        let box2_node = scene.add_mesh(box2.clone());
        let mut box3 = create_box(0.0, 0.0, 0.0, 2.0);
        let box3_node = scene.add_mesh(box3.clone());

        scene.add(&box1_node, None);
        scene.add(&box2_node, &box1_node);
        scene.add(&box3_node, &box2_node);

        let box1_top = Point::new(0.0, 0.0, 1.0);
        let x = Vector::new(1.0, 0.0, 0.0);
        let y = Vector::new(0.0, 1.0, 0.0);
        let plane_from = Plane::new(Point::new(0.0, 0.0, 0.0), x.clone(), y.clone());
        let plane_to = Plane::new(box1_top, x.clone(), y.clone());
        let xy_to_top = Xform::plane_to_plane(&plane_from, &plane_to);
        box1.xform = Xform::rotation_z(PI / 1.5) * xy_to_top;
        box2.xform = Xform::translation(2.0, 0.0, 0.0) * Xform::rotation_z(PI / 6.0);
        box3.xform = Xform::translation(2.0, 0.0, 0.0);

        scene.objects.meshes[0].xform = box1.xform.clone();
        scene.objects.meshes[1].xform = box2.xform.clone();
        scene.objects.meshes[2].xform = box3.xform.clone();

        let transformed = scene.get_geometry();

        MINI_CHECK!(transformed.meshes.len() == 3);

        let expected_box1: [[f64; 3]; 8] = [
            [1.36603, -0.366025, 0.0], [0.366025, 1.36603, 0.0],
            [-1.36603, 0.366025, 0.0], [-0.366025, -1.36603, 0.0],
            [1.36603, -0.366025, 2.0], [0.366025, 1.36603, 2.0],
            [-1.36603, 0.366025, 2.0], [-0.366025, -1.36603, 2.0],
        ];
        let expected_box2: [[f64; 3]; 8] = [
            [0.366025, 2.09808, 0.0], [-1.36603, 3.09808, 0.0],
            [-2.36603, 1.36603, 0.0], [-0.633975, 0.366025, 0.0],
            [0.366025, 2.09808, 2.0], [-1.36603, 3.09808, 2.0],
            [-2.36603, 1.36603, 2.0], [-0.633975, 0.366025, 2.0],
        ];
        let expected_box3: [[f64; 3]; 8] = [
            [-1.36603, 3.09808, 0.0], [-3.09808, 4.09808, 0.0],
            [-4.09808, 2.36603, 0.0], [-2.36603, 1.36603, 0.0],
            [-1.36603, 3.09808, 2.0], [-3.09808, 4.09808, 2.0],
            [-4.09808, 2.36603, 2.0], [-2.36603, 1.36603, 2.0],
        ];

        let m1 = &transformed.meshes[0];
        let mut vkeys1: Vec<usize> = m1.vertex.keys().copied().collect();
        vkeys1.sort();
        for i in 0..8 {
            let v = &m1.vertex[&vkeys1[i]];
            MINI_CHECK!((v.x - expected_box1[i][0]).abs() < 1e-4);
            MINI_CHECK!((v.y - expected_box1[i][1]).abs() < 1e-4);
            MINI_CHECK!((v.z - expected_box1[i][2]).abs() < 1e-4);
        }

        let m2 = &transformed.meshes[1];
        let mut vkeys2: Vec<usize> = m2.vertex.keys().copied().collect();
        vkeys2.sort();
        for i in 0..8 {
            let v = &m2.vertex[&vkeys2[i]];
            MINI_CHECK!((v.x - expected_box2[i][0]).abs() < 1e-4);
            MINI_CHECK!((v.y - expected_box2[i][1]).abs() < 1e-4);
            MINI_CHECK!((v.z - expected_box2[i][2]).abs() < 1e-4);
        }

        let m3 = &transformed.meshes[2];
        let mut vkeys3: Vec<usize> = m3.vertex.keys().copied().collect();
        vkeys3.sort();
        for i in 0..8 {
            let v = &m3.vertex[&vkeys3[i]];
            MINI_CHECK!((v.x - expected_box3[i][0]).abs() < 1e-4);
            MINI_CHECK!((v.y - expected_box3[i][1]).abs() < 1e-4);
            MINI_CHECK!((v.z - expected_box3[i][2]).abs() < 1e-4);
        }

        for mesh in &[m1, m2, m3] {
            MINI_CHECK!(mesh.face.len() == 6);
        }
    })
}

REGISTER_MINI_TEST!("Session", "Constructor", crate::session_test::run_session_constructor);
REGISTER_MINI_TEST!("Session", "Jsondump", crate::session_test::run_session_jsondump);
REGISTER_MINI_TEST!("Session", "Jsonload", crate::session_test::run_session_jsonload);
REGISTER_MINI_TEST!("Session", "File Io", crate::session_test::run_session_file_io);
REGISTER_MINI_TEST!("Session", "Add Point", crate::session_test::run_session_add_point);
REGISTER_MINI_TEST!("Session", "Add Edge", crate::session_test::run_session_add_edge);
REGISTER_MINI_TEST!("Session", "Get Object", crate::session_test::run_session_get_object);
REGISTER_MINI_TEST!("Session", "File Io Comprehensive", crate::session_test::run_session_file_io_comprehensive);
REGISTER_MINI_TEST!("Session", "Tree Transformation Hierarchy", crate::session_test::run_session_tree_transformation_hierarchy);
