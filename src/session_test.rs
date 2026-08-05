use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
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

pub fn run_session_add_obb() -> TestResult {
    MINI_TEST!("Add OBB", {
        use crate::{Session, OBB, Point, Vector};
        let mut session = Session::default();
        let obb = OBB::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(1.0, 1.0, 1.0),
        );
        let guid = obb.guid().to_string();
        session.add_obb(obb);

        MINI_CHECK!(session.objects.bboxes.len() == 1);
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

pub fn run_session_add_nurbscurve() -> TestResult {
    MINI_TEST!("Add Nurbscurve", {
        use crate::{Session, NurbsCurve, Point};
        let mut session = Session::default();
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
        ];
        let nc = NurbsCurve::create(false, 2, &pts);
        let guid = nc.guid().to_string();
        session.add_nurbscurve(nc, None);

        MINI_CHECK!(session.objects.nurbscurves.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_nurbssurface() -> TestResult {
    MINI_TEST!("Add Nurbssurface", {
        use crate::{Session, NurbsSurface, Point};
        let mut session = Session::default();
        let pts = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0), Point::new(0.0, 2.0, 0.0), Point::new(0.0, 3.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(1.0, 2.0, 0.0), Point::new(1.0, 3.0, 0.0),
            Point::new(2.0, 0.0, 0.0), Point::new(2.0, 1.0, 0.0), Point::new(2.0, 2.0, 0.0), Point::new(2.0, 3.0, 0.0),
            Point::new(3.0, 0.0, 0.0), Point::new(3.0, 1.0, 0.0), Point::new(3.0, 2.0, 0.0), Point::new(3.0, 3.0, 0.0),
        ];
        let ns = NurbsSurface::create(false, false, 3, 3, 4, 4, &pts).unwrap();
        let guid = ns.guid().to_string();
        session.add_nurbssurface(ns, None);

        MINI_CHECK!(session.objects.nurbssurfaces.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_brep() -> TestResult {
    MINI_TEST!("Add Brep", {
        use crate::Session;
        use crate::brep::BRep;
        let mut session = Session::default();
        let brep = BRep::create_box(1.0, 1.0, 1.0);
        let guid = brep.guid().to_string();
        session.add_brep(brep, None);

        MINI_CHECK!(session.objects.breps.len() == 1);
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

pub fn run_session_get_collisions() -> TestResult {
    MINI_TEST!("Get Collisions", {
        use crate::{Session, OBB, Point, Vector};
        let mut session = Session::default();
        let obb1 = OBB::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(2.0, 2.0, 2.0),
        );
        let obb2 = OBB::new(
            Point::new(1.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(2.0, 2.0, 2.0),
        );
        session.add_obb(obb1);
        session.add_obb(obb2);
        let pairs = session.get_collisions();

        MINI_CHECK!(pairs.len() >= 1);
    })
}

pub fn run_session_ray_cast() -> TestResult {
    MINI_TEST!("Ray Cast", {
        use crate::{Session, Mesh, Point, Vector, Xform};
        let mut session = Session::default();
        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(-1.0, -1.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, -1.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);
        session.add_mesh(mesh, None);
        let hits = session.ray_cast(&Point::new(0.0, 0.0, 2.0), &Vector::new(0.0, 0.0, -1.0), 1e-3);

        MINI_CHECK!(hits.len() >= 1);

        let mut placed = Mesh::new();
        let p0 = placed.add_vertex(Point::new(-1.0, -1.0, 0.0), None);
        let p1 = placed.add_vertex(Point::new(1.0, -1.0, 0.0), None);
        let p2 = placed.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        placed.add_face(vec![p0, p1, p2], None);
        placed.xform = Xform::translation(100.0, 0.0, 0.0);
        session.add_mesh(placed, None);
        let hits2 = session.ray_cast(&Point::new(100.0, 0.0, 2.0), &Vector::new(0.0, 0.0, -1.0), 1e-3);

        MINI_CHECK!(hits2.len() >= 1);
        MINI_CHECK!(TOLERANCE.is_close(hits2[0].point[0], 100.0));
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

        let polygon = vec![Point::new(0.0,0.0,0.0), Point::new(2.0,0.0,0.0), Point::new(2.0,2.0,0.0), Point::new(0.0,2.0,0.0)];
        let plate = crate::element::Element::plate(polygon, 0.2, "p1");
        let eguid = plate.guid().to_string();
        session.add_element(plate, None);
        let eremoved = session.remove_object(&eguid);

        let fname = "serialization/test_session_remove.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);

        MINI_CHECK!(removed);
        MINI_CHECK!(!session.lookup.contains_key(&guid));
        MINI_CHECK!(eremoved);
        MINI_CHECK!(session.objects.elements.is_empty());
        MINI_CHECK!(!loaded.lookup.contains_key(&eguid)); // removed objects must not resurrect on save/load
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
        //   file_json_dumps()    │ String       │ to JSON string
        //   file_json_loads(s)   │ String       │ from JSON string
        //   file_json_dump(path) │ file         │ write to file
        //   file_json_load(path) │ file         │ read from file

        let fname = "serialization/test_session.json";
        session.file_json_dump(fname);
        let loaded = Session::file_json_load(fname);

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

pub fn run_session_lookup_mutation_roundtrip() -> TestResult {
    MINI_TEST!("Lookup Mutation Roundtrip", {
        use crate::{Session, Line, Geometry};
        let mut session = Session::default();
        let line = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let guid = line.guid().to_string();
        session.add_line(line, None);

        if let Some(Geometry::Line(l)) = session.lookup.get_mut(&guid) { std::rc::Rc::make_mut(l).width = 5.0; }

        let fname = "serialization/test_session_lookup.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);

        MINI_CHECK!(loaded.objects.lines[0].width == 5.0);
        MINI_CHECK!(matches!(loaded.lookup.get(&guid), Some(Geometry::Line(l)) if l.width == 5.0));
    })
}

pub fn run_session_order() -> TestResult {
    MINI_TEST!("Order", {
        use crate::{Session, Line, Point};
        let mut session = Session::default();
        let line = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let point = Point::new(1.0, 2.0, 3.0);
        let line_guid = line.guid().to_string();
        let point_guid = point.guid().to_string();
        session.add_line(line, None);
        session.add_point(point, None);

        let order = session.order();

        let fname = "serialization/test_session_order.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);

        MINI_CHECK!(order.len() == 2);
        MINI_CHECK!(order[0] == point_guid);
        MINI_CHECK!(order[1] == line_guid);
        MINI_CHECK!(loaded.order() == order);
    })
}

pub fn run_session_tree_transformation_hierarchy() -> TestResult {
    MINI_TEST!("Tree Transformation Hierarchy", {
        use crate::{Session, Geometry, Point, Vector, Mesh, Plane, Xform};
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

        for (i, xf) in [&box1.xform, &box2.xform, &box3.xform].iter().enumerate() {
            let guid = scene.objects.meshes[i].guid().to_string();
            if let Some(Geometry::Mesh(m)) = scene.lookup.get_mut(&guid) {
                std::rc::Rc::make_mut(m).xform = (*xf).clone(); // lookup is the mutable truth
            }
        }

        let transformed = scene.get_geometry();

        MINI_CHECK!(transformed.meshes.len() == 3);
    })
}

pub fn run_session_add_component() -> TestResult {
    MINI_TEST!("Add Component", {
        // add_component stores a custom domain object in session.objects.components
        // and registers it in the graph under its guid.
        // The lookup is NOT in the geometry lookup (components are not geometry)
        // but they ARE in the tree and graph.
        use crate::{Session, Component};

        let mut session = Session::default();

        let mut extra = std::collections::HashMap::new();
        extra.insert("size".to_string(),   serde_json::json!(3000));
        extra.insert("height".to_string(), serde_json::json!(650));

        let guid = uuid::Uuid::new_v4().to_string();
        let c = Component {
            type_name: "FloorBuilder".to_string(),
            guid: guid.clone(),
            name: "floor_builder".to_string(),
            extra,
        };

        session.add_component(c, None);

        MINI_CHECK!(session.objects.components.len() == 1);
        MINI_CHECK!(session.graph.has_node(&guid));
        MINI_CHECK!(session.objects.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(session.objects.components[0].extra["size"] == serde_json::json!(3000));
    })
}

pub fn run_session_component_json_roundtrip() -> TestResult {
    MINI_TEST!("Component Json Roundtrip", {
        // A session with a component serialises to JSON and back.
        // All custom fields in `extra` must survive the round-trip.
        use crate::{Session, Component};
        use crate::file_encoders::{file_json_dump, file_json_load};

        let mut session = Session::default();

        let mut extra = std::collections::HashMap::new();
        extra.insert("size".to_string(),   serde_json::json!(3000));
        extra.insert("height".to_string(), serde_json::json!(650));
        extra.insert("rise".to_string(),   serde_json::json!(453));

        let guid = uuid::Uuid::new_v4().to_string();
        session.add_component(Component {
            type_name: "FloorBuilder".to_string(),
            guid: guid.clone(),
            name: "floor_builder".to_string(),
            extra,
        }, None);

        file_json_dump(&session, "serialization/test_session_component.json", false).unwrap();
        let loaded = file_json_load::<Session>("serialization/test_session_component.json").unwrap();

        MINI_CHECK!(loaded.objects.components.len() == 1);
        MINI_CHECK!(loaded.objects.components[0].guid == guid);
        MINI_CHECK!(loaded.objects.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.objects.components[0].extra["size"]   == serde_json::json!(3000));
        MINI_CHECK!(loaded.objects.components[0].extra["rise"]   == serde_json::json!(453));
    })
}

REGISTER_MINI_TEST!("Session", "Constructor", crate::session_test::run_session_constructor);
REGISTER_MINI_TEST!("Session", "Add Point", crate::session_test::run_session_add_point);
REGISTER_MINI_TEST!("Session", "Add Line", crate::session_test::run_session_add_line);
REGISTER_MINI_TEST!("Session", "Add Plane", crate::session_test::run_session_add_plane);
REGISTER_MINI_TEST!("Session", "Add OBB", crate::session_test::run_session_add_obb);
REGISTER_MINI_TEST!("Session", "Add Polyline", crate::session_test::run_session_add_polyline);
REGISTER_MINI_TEST!("Session", "Add Pointcloud", crate::session_test::run_session_add_pointcloud);
REGISTER_MINI_TEST!("Session", "Add Mesh", crate::session_test::run_session_add_mesh);
REGISTER_MINI_TEST!("Session", "Add Nurbscurve", crate::session_test::run_session_add_nurbscurve);
REGISTER_MINI_TEST!("Session", "Add Nurbssurface", crate::session_test::run_session_add_nurbssurface);
REGISTER_MINI_TEST!("Session", "Add Brep", crate::session_test::run_session_add_brep);
REGISTER_MINI_TEST!("Session", "Add Element", crate::session_test::run_session_add_element);
REGISTER_MINI_TEST!("Session", "Add Group", crate::session_test::run_session_add_group);
REGISTER_MINI_TEST!("Session", "Add Edge", crate::session_test::run_session_add_edge);
REGISTER_MINI_TEST!("Session", "Add Hierarchy", crate::session_test::run_session_add_hierarchy);
REGISTER_MINI_TEST!("Session", "Get Children", crate::session_test::run_session_get_children);
REGISTER_MINI_TEST!("Session", "Add Relationship", crate::session_test::run_session_add_relationship);
REGISTER_MINI_TEST!("Session", "Get Neighbours", crate::session_test::run_session_get_neighbours);
REGISTER_MINI_TEST!("Session", "Get Collisions", crate::session_test::run_session_get_collisions);
REGISTER_MINI_TEST!("Session", "Ray Cast", crate::session_test::run_session_ray_cast);
REGISTER_MINI_TEST!("Session", "Get Object", crate::session_test::run_session_get_object);
REGISTER_MINI_TEST!("Session", "Remove Object", crate::session_test::run_session_remove_object);
REGISTER_MINI_TEST!("Session", "Get Geometry", crate::session_test::run_session_get_geometry);
REGISTER_MINI_TEST!("Session", "Compute Face To Face", crate::session_test::run_session_compute_face_to_face);
REGISTER_MINI_TEST!("Session", "Json Roundtrip", crate::session_test::run_session_json_roundtrip);
REGISTER_MINI_TEST!("Session", "Protobuf Roundtrip", crate::session_test::run_session_protobuf_roundtrip);
REGISTER_MINI_TEST!("Session", "Lookup Mutation Roundtrip", crate::session_test::run_session_lookup_mutation_roundtrip);
REGISTER_MINI_TEST!("Session", "Order", crate::session_test::run_session_order);
REGISTER_MINI_TEST!("Session", "Tree Transformation Hierarchy", crate::session_test::run_session_tree_transformation_hierarchy);
REGISTER_MINI_TEST!("Session", "Add Component",              crate::session_test::run_session_add_component);
REGISTER_MINI_TEST!("Session", "Component Json Roundtrip",   crate::session_test::run_session_component_json_roundtrip);
