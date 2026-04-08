use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::PI;

pub fn run_session_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Session;
        let session = Session::default();
        let named = Session::new("my_named_session");

        MINI_CHECK!(session.name == "my_session");
        MINI_CHECK!(!session.guid().is_empty());
        MINI_CHECK!(named.name == "my_named_session");
    })
}

pub fn run_session_add_point() -> TestResult {
    MINI_TEST!("Add Point", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);

        MINI_CHECK!(session.objects.points.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
        MINI_CHECK!(session.graph.has_node(&guid));
    })
}

pub fn run_session_add_line() -> TestResult {
    MINI_TEST!("Add Line", {
        use crate::{Session, Line};
        let mut session = Session::default();
        let line = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let guid = line.guid().to_string();
        session.add_line(line, None);

        MINI_CHECK!(session.objects.lines.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_plane() -> TestResult {
    MINI_TEST!("Add Plane", {
        use crate::{Session, Plane};
        let mut session = Session::default();
        let plane = Plane::xy_plane();
        let guid = plane.guid().to_string();
        session.add_plane(plane, None);

        MINI_CHECK!(session.objects.planes.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_polyline() -> TestResult {
    MINI_TEST!("Add Polyline", {
        use crate::{Session, Polyline, Point};
        let mut session = Session::default();
        let pl = Polyline::new(vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0)]);
        let guid = pl.guid().to_string();
        session.add_polyline(pl, None);

        MINI_CHECK!(session.objects.polylines.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_pointcloud() -> TestResult {
    MINI_TEST!("Add Pointcloud", {
        use crate::{Session, PointCloud, Point};
        let mut session = Session::default();
        let pc = PointCloud::new(vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0)], vec![], vec![]);
        let guid = pc.guid().to_string();
        session.add_pointcloud(pc, None);

        MINI_CHECK!(session.objects.pointclouds.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_mesh() -> TestResult {
    MINI_TEST!("Add Mesh", {
        use crate::{Session, Mesh, Point};
        let mut session = Session::default();
        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0,0.0,0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0,0.0,0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0,1.0,0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);
        let guid = mesh.guid().to_string();
        session.add_mesh(mesh, None);

        MINI_CHECK!(session.objects.meshes.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_element() -> TestResult {
    MINI_TEST!("Add Element", {
        use crate::{Session, Point};
        use crate::element::Element;
        let mut session = Session::default();
        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(2.0,0.0,0.0), Point::new(2.0,2.0,0.0), Point::new(0.0,2.0,0.0)];
        let plate = Element::plate(polygon, 0.2, "p1");
        let guid = plate.guid().to_string();
        session.add_element(plate, None);

        MINI_CHECK!(session.objects.elements.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
        MINI_CHECK!(session.graph.has_node(&guid));
    })
}

pub fn run_session_add_group() -> TestResult {
    MINI_TEST!("Add Group", {
        use crate::Session;
        let mut session = Session::default();
        let group = session.add_group("my_group");

        MINI_CHECK!(group.borrow().name == "my_group");
    })
}

pub fn run_session_add_edge() -> TestResult {
    MINI_TEST!("Add Edge", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let p1 = Point::new(1.0, 2.0, 3.0);
        let p2 = Point::new(4.0, 5.0, 6.0);
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_point(p1, None);
        session.add_point(p2, None);
        session.add_edge(&g1, &g2, "connection");

        MINI_CHECK!(session.graph.has_edge((&g1, &g2)));
    })
}

pub fn run_session_add_hierarchy() -> TestResult {
    MINI_TEST!("Add Hierarchy", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let p1 = Point::new(0.0,0.0,0.0);
        let p2 = Point::new(1.0,0.0,0.0);
        let n1 = session.add_point(p1, None);
        let n2 = session.add_point(p2, None);
        session.add(&n1, None);
        session.add(&n2, None);
        let g1 = n1.borrow().guid().to_string();
        let g2 = n2.borrow().guid().to_string();
        let ok = session.add_hierarchy(&g1, &g2);

        MINI_CHECK!(ok);
    })
}

pub fn run_session_get_children() -> TestResult {
    MINI_TEST!("Get Children", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let p1 = Point::new(0.0,0.0,0.0);
        let p2 = Point::new(1.0,0.0,0.0);
        let n1 = session.add_point(p1, None);
        let n2 = session.add_point(p2, None);
        session.add(&n1, None);
        session.add(&n2, None);
        let g1 = n1.borrow().guid().to_string();
        let g2 = n2.borrow().guid().to_string();
        session.add_hierarchy(&g1, &g2);

        let children = session.get_children(&g1);

        MINI_CHECK!(children.len() == 1);
        MINI_CHECK!(children[0] == g2);
    })
}

pub fn run_session_add_relationship() -> TestResult {
    MINI_TEST!("Add Relationship", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let p1 = Point::new(0.0,0.0,0.0);
        let p2 = Point::new(1.0,0.0,0.0);
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_point(p1, None);
        session.add_point(p2, None);
        session.add_relationship(&g1, &g2, "connects_to");

        MINI_CHECK!(session.graph.has_edge((&g1, &g2)));
    })
}

pub fn run_session_get_neighbours() -> TestResult {
    MINI_TEST!("Get Neighbours", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let p1 = Point::new(0.0,0.0,0.0);
        let p2 = Point::new(1.0,0.0,0.0);
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_point(p1, None);
        session.add_point(p2, None);
        session.add_edge(&g1, &g2, "connection");

        let neighbours = session.get_neighbours(&g1);

        MINI_CHECK!(neighbours.len() == 1);
        MINI_CHECK!(neighbours[0] == g2);
    })
}

pub fn run_session_get_object() -> TestResult {
    MINI_TEST!("Get Object", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);
        let retrieved = session.get_object(&guid);

        MINI_CHECK!(retrieved.is_some());
    })
}

pub fn run_session_remove_object() -> TestResult {
    MINI_TEST!("Remove Object", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);
        let removed = session.remove_object(&guid);

        MINI_CHECK!(removed);
        MINI_CHECK!(!session.lookup.contains_key(&guid));
    })
}

pub fn run_session_get_geometry() -> TestResult {
    MINI_TEST!("Get Geometry", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        session.add_point(point, None);

        let geom = session.get_geometry();

        MINI_CHECK!(geom.points.len() == 1);
    })
}

pub fn run_session_compute_face_to_face() -> TestResult {
    MINI_TEST!("Compute Face To Face", {
        use crate::{Session, Point};
        use crate::element::Element;
        let mut session = Session::default();
        let p1 = Element::plate(vec![Point::new(0.0,0.0,0.0), Point::new(1.0,0.0,0.0), Point::new(1.0,1.0,0.0), Point::new(0.0,1.0,0.0)], 0.2, "p1");
        let p2 = Element::plate(vec![Point::new(0.0,0.0,-0.2), Point::new(1.0,0.0,-0.2), Point::new(1.0,1.0,-0.2), Point::new(0.0,1.0,-0.2)], 0.2, "p2");
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_element(p1, None);
        session.add_element(p2, None);
        session.compute_face_to_face(5.0, 0.001);

        MINI_CHECK!(session.objects.elements.len() == 2);
        MINI_CHECK!(session.graph.has_edge((&g1, &g2)));
    })
}

pub fn run_session_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let p1 = Point::new(1.0, 2.0, 3.0);
        let p2 = Point::new(4.0, 5.0, 6.0);
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_point(p1, None);
        session.add_point(p2, None);
        session.add_edge(&g1, &g2, "connection");

        //   jsondump()      │ String       │ to JSON string (internal use)
        //   jsonload(s)     │ String       │ from JSON string (internal use)
        //   json_dumps()    │ String       │ to JSON string
        //   json_loads(s)   │ String       │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        let fname = "serialization/test_session.json";
        session.json_dump(fname);
        let loaded = Session::json_load(fname);

        MINI_CHECK!(loaded.name == session.name);
        MINI_CHECK!(loaded.lookup.len() == session.lookup.len());
        MINI_CHECK!(loaded.graph.number_of_vertices() == session.graph.number_of_vertices());
    })
}

pub fn run_session_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::{Session, Point};
        let mut session = Session::default();
        let p1 = Point::new(1.0, 2.0, 3.0);
        let p2 = Point::new(4.0, 5.0, 6.0);
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_point(p1, None);
        session.add_point(p2, None);
        session.add_edge(&g1, &g2, "connection");

        let fname = "serialization/test_session.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);

        MINI_CHECK!(loaded.name == session.name);
        MINI_CHECK!(loaded.lookup.len() == session.lookup.len());
    })
}

pub fn run_session_tree_transformation_hierarchy() -> TestResult {
    MINI_TEST!("Tree Transformation Hierarchy", {
        use crate::{Session, Point, Vector, Mesh, Plane, Xform};
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
        let box1_node = scene.add_mesh(box1.clone(), None);
        let mut box2 = create_box(0.0, 0.0, 0.0, 2.0);
        let box2_node = scene.add_mesh(box2.clone(), None);
        let mut box3 = create_box(0.0, 0.0, 0.0, 2.0);
        let box3_node = scene.add_mesh(box3.clone(), None);

        scene.add(&box1_node, None);
        scene.add(&box2_node, &box1_node);
        scene.add(&box3_node, &box2_node);

        let box1_top = Point::new(0.0, 0.0, 1.0);
        let x = Vector::new(1.0, 0.0, 0.0);
        let y = Vector::new(0.0, 1.0, 0.0);
        let plane_from = Plane::new(Point::new(0.0, 0.0, 0.0), x.clone(), y.clone());
        let plane_to = Plane::new(box1_top, x.clone(), y.clone());
        let xy_to_top = Xform::plane_to_plane(&plane_from, &plane_to);
        box1.xform = Xform::rotation_z(PI / 1.5, false) * xy_to_top;
        box2.xform = Xform::translation(2.0, 0.0, 0.0) * Xform::rotation_z(PI / 6.0, false);
        box3.xform = Xform::translation(2.0, 0.0, 0.0);

        scene.objects.meshes[0].xform = box1.xform.clone();
        scene.objects.meshes[1].xform = box2.xform.clone();
        scene.objects.meshes[2].xform = box3.xform.clone();

        let transformed = scene.get_geometry();

        MINI_CHECK!(transformed.meshes.len() == 3);
    })
}

REGISTER_MINI_TEST!("Session", "Constructor", crate::session_test::run_session_constructor);
REGISTER_MINI_TEST!("Session", "Add Point", crate::session_test::run_session_add_point);
REGISTER_MINI_TEST!("Session", "Add Line", crate::session_test::run_session_add_line);
REGISTER_MINI_TEST!("Session", "Add Plane", crate::session_test::run_session_add_plane);
REGISTER_MINI_TEST!("Session", "Add Polyline", crate::session_test::run_session_add_polyline);
REGISTER_MINI_TEST!("Session", "Add Pointcloud", crate::session_test::run_session_add_pointcloud);
REGISTER_MINI_TEST!("Session", "Add Mesh", crate::session_test::run_session_add_mesh);
REGISTER_MINI_TEST!("Session", "Add Element", crate::session_test::run_session_add_element);
REGISTER_MINI_TEST!("Session", "Add Group", crate::session_test::run_session_add_group);
REGISTER_MINI_TEST!("Session", "Add Edge", crate::session_test::run_session_add_edge);
REGISTER_MINI_TEST!("Session", "Add Hierarchy", crate::session_test::run_session_add_hierarchy);
REGISTER_MINI_TEST!("Session", "Get Children", crate::session_test::run_session_get_children);
REGISTER_MINI_TEST!("Session", "Add Relationship", crate::session_test::run_session_add_relationship);
REGISTER_MINI_TEST!("Session", "Get Neighbours", crate::session_test::run_session_get_neighbours);
REGISTER_MINI_TEST!("Session", "Get Object", crate::session_test::run_session_get_object);
REGISTER_MINI_TEST!("Session", "Remove Object", crate::session_test::run_session_remove_object);
REGISTER_MINI_TEST!("Session", "Get Geometry", crate::session_test::run_session_get_geometry);
REGISTER_MINI_TEST!("Session", "Compute Face To Face", crate::session_test::run_session_compute_face_to_face);
REGISTER_MINI_TEST!("Session", "Json Roundtrip", crate::session_test::run_session_json_roundtrip);
REGISTER_MINI_TEST!("Session", "Protobuf Roundtrip", crate::session_test::run_session_protobuf_roundtrip);
REGISTER_MINI_TEST!("Session", "Tree Transformation Hierarchy", crate::session_test::run_session_tree_transformation_hierarchy);
