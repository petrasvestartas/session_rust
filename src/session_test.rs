use crate::mini_test::TestResult;
use crate::tolerance::PI;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

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

pub fn run_session_copy() -> TestResult {
    MINI_TEST!("Copy", {
        use crate::Element;
        use crate::Point;
        use crate::Session;
        use crate::Xform;

        let mut session = Session::new("original");
        let point = Point::new(1.0, 2.0, 3.0);
        let element = Element::new("plate");
        let point_guid = point.guid().to_string();
        let element_guid = element.guid().to_string();
        let group = session.add_group("Group");
        session.add_point(point, Some(&group));
        session.add_element(element, Some(&group));
        session.add_edge(&point_guid, &element_guid, "touching");
        session.set_xform(&point_guid, Xform::translation(1.0, 0.0, 0.0));
        let guid = session.guid().to_string();

        let mut copy = session.clone();

        MINI_CHECK!(copy.name == session.name);
        MINI_CHECK!(copy.guid() == guid);
        MINI_CHECK!(copy.history.depth() == 0);
        MINI_CHECK!(copy.objects.points.len() == 1);
        MINI_CHECK!(copy.objects.elements.len() == 1);
        MINI_CHECK!(copy.lookup.len() == session.lookup.len());
        MINI_CHECK!(copy.graph.number_of_edges() == 1);
        MINI_CHECK!(copy.xforms.len() == 1);
        MINI_CHECK!(
            copy.tree.root().unwrap().borrow().descendants().len()
                == session.tree.root().unwrap().borrow().descendants().len()
        );

        MINI_CHECK!(!std::ptr::eq(&copy.objects.points, &session.objects.points));
        MINI_CHECK!(!std::ptr::eq(
            &copy.objects.elements,
            &session.objects.elements
        ));
        MINI_CHECK!(!std::rc::Rc::ptr_eq(
            &copy.tree.root().unwrap(),
            &session.tree.root().unwrap()
        ));
        MINI_CHECK!(!std::rc::Rc::ptr_eq(
            &copy.objects.points[0],
            &session.objects.points[0]
        ));
        MINI_CHECK!(copy.objects.points[0].guid() == point_guid);
        MINI_CHECK!(copy.objects.elements[0].guid() == element_guid);
        MINI_CHECK!(copy.lookup.contains_key(&point_guid));

        copy.objects.points.clear();
        let copied_nodes = copy.tree.nodes();

        MINI_CHECK!(copied_nodes.len() > 1);
        copy.tree.remove(&copied_nodes[1]);

        MINI_CHECK!(session.objects.points.len() == 1);
        MINI_CHECK!(copy.objects.points.is_empty());
        MINI_CHECK!(!session
            .tree
            .root()
            .unwrap()
            .borrow()
            .descendants()
            .is_empty());
        MINI_CHECK!(copy.tree.root().unwrap().borrow().descendants().is_empty());
    })
}

pub fn run_session_add_point() -> TestResult {
    MINI_TEST!("Add Point", {
        use crate::Point;
        use crate::Session;

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
        use crate::Line;
        use crate::Session;

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
        use crate::Plane;
        use crate::Session;

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
        use crate::Point;
        use crate::Session;
        use crate::Vector;
        use crate::OBB;

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
        use crate::Point;
        use crate::Polyline;
        use crate::Session;

        let mut session = Session::default();
        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ]);
        let guid = pl.guid().to_string();
        session.add_polyline(pl, None);

        MINI_CHECK!(session.objects.polylines.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_select_by_type() -> TestResult {
    MINI_TEST!("Select By Type", {
        use crate::Mesh;
        use crate::Point;
        use crate::Polyline;
        use crate::Session;

        let mut session = Session::default();
        let g0 = session.add_group("g0");
        let g1 = session.add_group("g1");
        let g2 = session.add_group("g2");

        session.add_polyline(
            Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]),
            Some(&g0),
        );
        session.add_polyline(
            Polyline::new(vec![Point::new(0.0, 1.0, 0.0), Point::new(1.0, 1.0, 0.0)]),
            Some(&g0),
        );
        session.add_polyline(
            Polyline::new(vec![Point::new(0.0, 2.0, 0.0), Point::new(1.0, 2.0, 0.0)]),
            Some(&g1),
        );
        session.add_point(Point::new(9.0, 9.0, 9.0), Some(&g2));

        let groups = session.select_by_type::<Polyline>();

        MINI_CHECK!(groups.len() == 2);
        MINI_CHECK!(groups[0].len() == 2);
        MINI_CHECK!(groups[1].len() == 1);
        MINI_CHECK!(TOLERANCE.is_close(groups[1][0].get_point(0).unwrap()[1], 2.0));

        MINI_CHECK!(session.select_by_type::<Mesh>().is_empty());
    })
}

pub fn run_session_add_pointcloud() -> TestResult {
    MINI_TEST!("Add Pointcloud", {
        use crate::Point;
        use crate::PointCloud;
        use crate::Session;

        let mut session = Session::default();
        let pc = PointCloud::new(
            vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)],
            vec![],
            vec![],
        );
        let guid = pc.guid().to_string();
        session.add_pointcloud(pc, None);

        MINI_CHECK!(session.objects.pointclouds.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_mesh() -> TestResult {
    MINI_TEST!("Add Mesh", {
        use crate::Mesh;
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);
        let guid = mesh.guid().to_string();
        session.add_mesh(mesh, None);

        MINI_CHECK!(session.objects.meshes.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
    })
}

pub fn run_session_add_nurbscurve() -> TestResult {
    MINI_TEST!("Add Nurbscurve", {
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Session;

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
        use crate::NurbsSurface;
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(1.0, 3.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(2.0, 3.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(3.0, 3.0, 0.0),
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
        use crate::BRep;
        use crate::Session;

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
        use crate::Element;
        use crate::Session;

        let mut session = Session::default();
        let plate = Element::new("p1");
        let guid = plate.guid().to_string();
        session.add_element(plate, None);

        MINI_CHECK!(session.objects.elements.len() == 1);
        MINI_CHECK!(session.lookup.contains_key(&guid));
        MINI_CHECK!(session.graph.has_node(&guid));
    })
}

pub fn run_session_add_empty_geometry() -> TestResult {
    MINI_TEST!("Add Empty Geometry", {
        use crate::BRep;
        use crate::Mesh;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::Point;
        use crate::PointCloud;
        use crate::Polyline;
        use crate::Session;

        let mut session = Session::default();
        let group = session.add_group("empty");

        MINI_CHECK!(session
            .add_polyline(Polyline::new(vec![Point::new(0.0, 0.0, 0.0)]), Some(&group))
            .is_none());
        MINI_CHECK!(session
            .add_pointcloud(PointCloud::new(vec![], vec![], vec![]), Some(&group))
            .is_none());
        MINI_CHECK!(session.add_mesh(Mesh::new(), Some(&group)).is_none());
        MINI_CHECK!(session
            .add_nurbscurve(NurbsCurve::default(), Some(&group))
            .is_none());
        MINI_CHECK!(session
            .add_nurbssurface(NurbsSurface::default(), Some(&group))
            .is_none());
        MINI_CHECK!(session.add_brep(BRep::new(), Some(&group)).is_none());

        let mut vertices_only = Mesh::new();
        vertices_only.add_vertex(Point::new(0.0, 0.0, 0.0), None);

        MINI_CHECK!(session.add_mesh(vertices_only, Some(&group)).is_none());

        MINI_CHECK!(session.lookup.is_empty());
        MINI_CHECK!(session.order().is_empty());
        MINI_CHECK!(group.borrow().children().is_empty());
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
        use crate::Point;
        use crate::Session;

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
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 0.0, 0.0);
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
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 0.0, 0.0);
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
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 0.0, 0.0);
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_point(p1, None);
        session.add_point(p2, None);
        session.add_relationship(&g1, &g2, "connects_to");

        MINI_CHECK!(session.graph.has_edge((&g1, &g2)));
    })
}

/// A test-only implementor: a named interaction with no state of its own.
#[derive(Debug, Clone, Default)]
struct NamedInteraction {
    guid: std::sync::OnceLock<String>, // Lazily minted guid.
    name: String,                      // What joins the pair.
}

impl NamedInteraction {
    /// Construct from a name.
    fn new(name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
        }
    }
}

impl crate::Interaction for NamedInteraction {
    /// Return whether the lazy guid has been created.
    fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    fn set_guid(&mut self, guid: String) {
        self.guid = std::sync::OnceLock::from(guid);
    }

    /// Return the name.
    fn name(&self) -> &str {
        &self.name
    }

    /// Set the name.
    fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }

    /// Return the registered type name.
    fn interaction_type_name(&self) -> &str {
        "NamedInteraction"
    }

    /// Return no state.
    fn interaction_data_dumps(&self) -> Vec<u8> {
        Vec::new()
    }

    /// Return a copy with the same guid.
    fn clone_box(&self) -> Box<dyn crate::Interaction> {
        self.guid();

        Box::new(self.clone())
    }
}

/// Build a NamedInteraction from its data.
fn named_interaction(_data: &[u8]) -> Option<Box<dyn crate::Interaction>> {
    Some(Box::new(NamedInteraction::default()))
}

pub fn run_session_add_interaction() -> TestResult {
    MINI_TEST!("Add Interaction", {
        use crate::Element;
        use crate::Session;

        let mut session = Session::default();
        session.add_element(Element::new("a"), None);
        session.add_element(Element::new("b"), None);
        let a = session.objects.elements[0].clone();
        let b = session.objects.elements[1].clone();
        let absent = Element::new("absent");
        session.add_edge(a.guid(), b.guid(), "authored");
        let glue = session
            .add_interaction(&a, &b, Box::new(NamedInteraction::new("glue")))
            .unwrap()
            .clone_box();
        let id = session.graph.edges[a.guid()][b.guid()].guid().to_string();
        let screw = session
            .add_interaction(&b, &a, Box::new(NamedInteraction::new("screw")))
            .unwrap()
            .clone_box();

        MINI_CHECK!(session.interactions.len() == 1);
        MINI_CHECK!(session.interactions[&id].len() == 2);
        MINI_CHECK!(session.interactions[&id][0].guid() == glue.guid());
        MINI_CHECK!(session.interactions[&id][1].guid() == screw.guid());
        MINI_CHECK!(session.graph.number_of_edges() == 1);
        MINI_CHECK!(session.graph.edges[b.guid()][a.guid()].guid() == id);
        MINI_CHECK!(session.graph.edges[a.guid()][b.guid()].attribute == "authored");

        let missing_rejected = session
            .add_interaction(&a, &absent, Box::new(NamedInteraction::default()))
            .is_err();
        let self_rejected = session
            .add_interaction(&a, &a, Box::new(NamedInteraction::default()))
            .is_err();

        MINI_CHECK!(missing_rejected);
        MINI_CHECK!(self_rejected);
        MINI_CHECK!(session.graph.number_of_edges() == 1);
    })
}

pub fn run_session_get_interaction() -> TestResult {
    MINI_TEST!("Get Interaction", {
        use crate::Element;
        use crate::Interaction;
        use crate::Session;

        <dyn Interaction>::register_type("NamedInteraction", named_interaction);
        let mut session = Session::default();
        session.add_element(Element::new("a"), None);
        session.add_element(Element::new("b"), None);
        session.add_element(Element::new("c"), None);
        let a = session.objects.elements[0].clone();
        let b = session.objects.elements[1].clone();
        let c = session.objects.elements[2].clone();
        session.add_edge(a.guid(), c.guid(), "authored");
        let before = session.get_interaction(&a, &b).to_vec();
        let bare = session.get_interaction(&a, &c).to_vec();
        let glue = session
            .add_interaction(&a, &b, Box::new(NamedInteraction::new("glue")))
            .unwrap()
            .clone_box();
        let id = session.graph.edges[a.guid()][b.guid()].guid().to_string();
        let mut duplicate = session.clone();
        duplicate.interactions.get_mut(&id).unwrap()[0].set_name("screw");
        let loaded_b = Session::pb_loads(&session.pb_dumps()).unwrap();
        let loaded_j = Session::file_json_loads(&session.file_json_dumps());

        MINI_CHECK!(before.is_empty());
        MINI_CHECK!(bare.is_empty());
        MINI_CHECK!(session.get_interaction(&a, &b)[0].guid() == glue.guid());
        MINI_CHECK!(session.get_interaction(&b, &a)[0].guid() == glue.guid());
        MINI_CHECK!(session.get_interaction(&a, &b)[0].name() == "glue");
        MINI_CHECK!(duplicate.get_interaction(&b, &a)[0].name() == "screw");
        MINI_CHECK!(duplicate.get_interaction(&b, &a)[0].guid() == glue.guid());
        MINI_CHECK!(*loaded_b.get_interaction(&b, &a)[0] == *glue);
        MINI_CHECK!(loaded_b.get_interaction(&b, &a)[0].guid() == glue.guid());
        MINI_CHECK!(*loaded_j.get_interaction(&b, &a)[0] == *glue);
    })
}

pub fn run_session_has_interaction() -> TestResult {
    MINI_TEST!("Has Interaction", {
        use crate::Element;
        use crate::Interaction;
        use crate::Session;

        <dyn Interaction>::register_type("NamedInteraction", named_interaction);
        let mut session = Session::default();
        session.add_element(Element::new("a"), None);
        session.add_element(Element::new("b"), None);
        let a = session.objects.elements[0].clone();
        let b = session.objects.elements[1].clone();
        let absent = Element::new("absent");
        let before = session.has_interaction(&a, &b);
        session
            .add_interaction(&a, &b, Box::new(NamedInteraction::default()))
            .unwrap();
        let loaded = Session::pb_loads(&session.pb_dumps()).unwrap();

        MINI_CHECK!(!before);
        MINI_CHECK!(session.has_interaction(&a, &b));
        MINI_CHECK!(session.has_interaction(&b, &a));
        MINI_CHECK!(!session.has_interaction(&a, &absent));
        MINI_CHECK!(loaded.has_interaction(&b, &a));
    })
}

pub fn run_session_remove_interaction() -> TestResult {
    MINI_TEST!("Remove Interaction", {
        use crate::Element;
        use crate::Session;

        let mut session = Session::default();
        session.add_element(Element::new("a"), None);
        session.add_element(Element::new("b"), None);
        session.add_element(Element::new("c"), None);
        let a = session.objects.elements[0].clone();
        let b = session.objects.elements[1].clone();
        let c = session.objects.elements[2].clone();
        session
            .add_interaction(&a, &b, Box::new(NamedInteraction::new("glue")))
            .unwrap();
        session
            .add_interaction(&a, &c, Box::new(NamedInteraction::default()))
            .unwrap();
        session.remove_interaction(&b, &a);
        session.remove_interaction(&b, &a);

        MINI_CHECK!(!session.has_interaction(&a, &b));
        MINI_CHECK!(session.has_interaction(&a, &c));
        MINI_CHECK!(session.get_interaction(&a, &b).is_empty());
        MINI_CHECK!(session.interactions.len() == 1);
        MINI_CHECK!(session.graph.number_of_edges() == 1);
        MINI_CHECK!(session.graph.has_node(b.guid()));
    })
}

pub fn run_session_undo_remove_interaction() -> TestResult {
    MINI_TEST!("Undo Remove Interaction", {
        use crate::Element;
        use crate::Session;

        let mut session = Session::default();
        session.add_element(Element::new("a"), None);
        session.add_element(Element::new("b"), None);
        let a = session.objects.elements[0].clone();
        let b = session.objects.elements[1].clone();
        let glue = session
            .add_interaction(&a, &b, Box::new(NamedInteraction::new("glue")))
            .unwrap()
            .clone_box();
        let id = session.graph.edges[a.guid()][b.guid()].guid().to_string();

        session.begin("remove");
        session.remove_object(b.guid());
        session.commit();
        let dropped = !session.interactions.contains_key(&id);
        session.undo();

        MINI_CHECK!(dropped);
        MINI_CHECK!(session.graph.edges[a.guid()][b.guid()].guid() == id);
        MINI_CHECK!(session.get_interaction(&a, &b).len() == 1);
        MINI_CHECK!(session.get_interaction(&a, &b)[0].guid() == glue.guid());
        MINI_CHECK!(session.get_interaction(&a, &b)[0].name() == "glue");

        session.redo();

        MINI_CHECK!(!session.interactions.contains_key(&id));
    })
}

pub fn run_session_get_neighbours() -> TestResult {
    MINI_TEST!("Get Neighbours", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 0.0, 0.0);
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
        use crate::Point;
        use crate::Session;
        use crate::Vector;
        use crate::OBB;

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

        MINI_CHECK!(!pairs.is_empty());
    })
}

pub fn run_session_ray_cast() -> TestResult {
    MINI_TEST!("Ray Cast", {
        use crate::Mesh;
        use crate::Point;
        use crate::Session;
        use crate::Vector;
        use crate::Xform;

        let mut session = Session::default();
        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(-1.0, -1.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, -1.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2], None);
        session.add_mesh(mesh, None);
        let hits = session.ray_cast(
            &Point::new(0.0, 0.0, 2.0),
            &Vector::new(0.0, 0.0, -1.0),
            1e-3,
        );

        MINI_CHECK!(!hits.is_empty());

        let mut placed = Mesh::new();
        let p0 = placed.add_vertex(Point::new(-1.0, -1.0, 0.0), None);
        let p1 = placed.add_vertex(Point::new(1.0, -1.0, 0.0), None);
        let p2 = placed.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        placed.add_face(vec![p0, p1, p2], None);
        let placed_guid = placed.guid().to_string();
        session.add_mesh(placed, None);
        session.set_xform(&placed_guid, Xform::translation(100.0, 0.0, 0.0));
        let hits2 = session.ray_cast(
            &Point::new(100.0, 0.0, 2.0),
            &Vector::new(0.0, 0.0, -1.0),
            1e-3,
        );

        MINI_CHECK!(!hits2.is_empty());
        MINI_CHECK!(TOLERANCE.is_close(hits2[0].hit_point[0], 100.0));
    })
}

pub fn run_session_get_object() -> TestResult {
    MINI_TEST!("Get Object", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);
        let retrieved = session.get_object(&guid);

        MINI_CHECK!(retrieved.is_some());
        MINI_CHECK!(retrieved.unwrap().guid() == guid);
    })
}

pub fn run_session_remove_object() -> TestResult {
    MINI_TEST!("Remove Object", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);
        let removed = session.remove_object(&guid);

        let plate = crate::element::Element::new("p1");
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
        MINI_CHECK!(session.objects.elements.number_of_slots() == 1);
        MINI_CHECK!(session.objects.points.number_of_dead() == 1);
        MINI_CHECK!(!session.graph.has_node(&eguid));
        MINI_CHECK!(!loaded.lookup.contains_key(&eguid));
        MINI_CHECK!(loaded.objects.points.number_of_slots() == 0);
    })
}

pub fn run_session_get_geometry() -> TestResult {
    MINI_TEST!("Get Geometry", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        session.add_point(point, None);

        let geom = session.get_geometry();

        MINI_CHECK!(geom.points.len() == 1);
    })
}

pub fn run_session_get_geometry_is_pure() -> TestResult {
    MINI_TEST!("Get Geometry Is Pure", {
        use crate::Point;
        use crate::Session;
        use crate::Xform;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point.clone(), None);
        session.set_xform(&guid, Xform::translation(10.0, 0.0, 0.0));

        let first = session.get_geometry().points[0].clone();
        let second = session.get_geometry().points[0].clone();

        MINI_CHECK!(TOLERANCE.is_close(first[0], 11.0));
        MINI_CHECK!(TOLERANCE.is_close(second[0], 11.0));
        MINI_CHECK!(TOLERANCE.is_close(point[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(session.objects.points[0][0], 1.0));
    })
}

pub fn run_session_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let p1 = Point::new(1.0, 2.0, 3.0);
        let p2 = Point::new(4.0, 5.0, 6.0);
        let g1 = p1.guid().to_string();
        let g2 = p2.guid().to_string();
        session.add_point(p1, None);
        session.add_point(p2, None);
        session.add_edge(&g1, &g2, "connection");

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
        use crate::Point;
        use crate::Session;

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
        let converted = Session::from_proto(session.to_proto()).unwrap();

        MINI_CHECK!(loaded.name == session.name);
        MINI_CHECK!(loaded.lookup.len() == session.lookup.len());
        MINI_CHECK!(converted.lookup.len() == session.lookup.len());
        MINI_CHECK!(converted.graph.has_edge((&g1, &g2)));
    })
}

pub fn run_session_lookup_mutation_roundtrip() -> TestResult {
    MINI_TEST!("Lookup Mutation Roundtrip", {
        use crate::Geometry;
        use crate::Line;
        use crate::Session;

        let mut session = Session::default();
        let line = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        let guid = line.guid().to_string();
        session.add_line(line, None);

        if let Some(Geometry::Line(l)) = session.lookup.get_mut(&guid) {
            std::rc::Rc::make_mut(l).width = 5.0;
        }

        let fname = "serialization/test_session_lookup.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);

        MINI_CHECK!(loaded.objects.lines[0].width == 5.0);
        MINI_CHECK!(matches!(loaded.lookup.get(&guid), Some(Geometry::Line(l)) if l.width == 5.0));
    })
}

pub fn run_session_order() -> TestResult {
    MINI_TEST!("Order", {
        use crate::Line;
        use crate::Point;
        use crate::Session;

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

pub fn run_session_set_xform() -> TestResult {
    MINI_TEST!("Set Xform", {
        use crate::Point;
        use crate::Session;
        use crate::Xform;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);

        let shift = Xform::translation(5.0, 0.0, 0.0);
        session.set_xform(&guid, shift.clone());

        MINI_CHECK!(session.xform(&guid) == shift);
        MINI_CHECK!(session.world_xform(&guid) == shift);
        MINI_CHECK!(session.world_xforms()[&guid] == shift);
        MINI_CHECK!(session.xform("missing") == Xform::identity());
        MINI_CHECK!(session.remove_xform(&guid));
        MINI_CHECK!(session.xform(&guid) == Xform::identity());
    })
}

pub fn run_session_world_xform_hierarchy() -> TestResult {
    MINI_TEST!("World Xform Hierarchy", {
        use crate::Point;
        use crate::Session;
        use crate::Xform;

        let mut session = Session::default();
        let a = Point::new(0.0, 0.0, 0.0);
        let b = Point::new(0.0, 0.0, 0.0);
        let c = Point::new(0.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        let b_guid = b.guid().to_string();
        let c_guid = c.guid().to_string();
        let a_node = session.add_point(a, None);
        let b_node = session.add_point(b, None);
        let c_node = session.add_point(c, None);

        session.add(&a_node, None);
        session.add(&b_node, &a_node);
        session.add(&c_node, &b_node);

        let a_xform = Xform::rotation_z(PI / 2.0, false);
        let b_xform = Xform::translation(2.0, 0.0, 0.0);
        let c_xform = Xform::rotation_z(PI / 2.0, false);
        session.set_xform(&a_guid, a_xform.clone());
        session.set_xform(&b_guid, b_xform.clone());
        session.set_xform(&c_guid, c_xform.clone());

        let world = session.world_xforms();

        MINI_CHECK!(session.world_xform(&a_guid) == a_xform);
        MINI_CHECK!(session.world_xform(&b_guid) == &a_xform * &b_xform);
        MINI_CHECK!(session.world_xform(&c_guid) == &(&a_xform * &b_xform) * &c_xform);
        MINI_CHECK!(world[&c_guid] == session.world_xform(&c_guid));
    })
}

pub fn run_session_xform_roundtrip() -> TestResult {
    MINI_TEST!("Xform Roundtrip", {
        use crate::Point;
        use crate::Session;
        use crate::Xform;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);
        session.set_xform(&guid, Xform::translation(7.0, 8.0, 9.0));

        let fname = "serialization/test_session_xform.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);
        let json_loaded = Session::file_json_loads(&session.file_json_dumps());

        MINI_CHECK!(loaded.xform(&guid) == session.xform(&guid));
        MINI_CHECK!(loaded.xforms.len() == 1);
        MINI_CHECK!(json_loaded.xform(&guid) == session.xform(&guid));
        MINI_CHECK!(json_loaded.xforms.len() == 1);
    })
}

/// A cube mesh of the given size centred on center.
fn create_box(center: &crate::Point, size: f64) -> crate::Mesh {
    use crate::Mesh;
    use crate::Point;

    let mut mesh = Mesh::new();
    let h = size * 0.5;
    let verts = [
        Point::new(center[0] - h, center[1] - h, center[2] - h),
        Point::new(center[0] + h, center[1] - h, center[2] - h),
        Point::new(center[0] + h, center[1] + h, center[2] - h),
        Point::new(center[0] - h, center[1] + h, center[2] - h),
        Point::new(center[0] - h, center[1] - h, center[2] + h),
        Point::new(center[0] + h, center[1] - h, center[2] + h),
        Point::new(center[0] + h, center[1] + h, center[2] + h),
        Point::new(center[0] - h, center[1] + h, center[2] + h),
    ];
    let mut keys = Vec::new();

    for vert in &verts {
        keys.push(mesh.add_vertex(vert.clone(), None));
    }

    let faces = [
        [0, 1, 2, 3],
        [4, 7, 6, 5],
        [0, 4, 5, 1],
        [2, 6, 7, 3],
        [0, 3, 7, 4],
        [1, 5, 6, 2],
    ];

    for f in faces {
        mesh.add_face(vec![keys[f[0]], keys[f[1]], keys[f[2]], keys[f[3]]], None);
    }

    mesh
}

pub fn run_session_tree_transformation_hierarchy() -> TestResult {
    MINI_TEST!("Tree Transformation Hierarchy", {
        use crate::Plane;
        use crate::Point;
        use crate::Session;
        use crate::Vector;
        use crate::Xform;

        let mut scene = Session::new("tree_transformation_test");

        let box1 = create_box(&Point::new(0.0, 0.0, 0.0), 2.0);
        let box1_guid = box1.guid().to_string();
        let box1_node = scene.add_mesh(box1, None).unwrap();
        let box2 = create_box(&Point::new(0.0, 0.0, 0.0), 2.0);
        let box2_guid = box2.guid().to_string();
        let box2_node = scene.add_mesh(box2, None).unwrap();
        let box3 = create_box(&Point::new(0.0, 0.0, 0.0), 2.0);
        let box3_guid = box3.guid().to_string();
        let box3_node = scene.add_mesh(box3, None).unwrap();

        scene.add(&box1_node, None);
        scene.add(&box2_node, &box1_node);
        scene.add(&box3_node, &box2_node);

        let box1_top = Point::new(0.0, 0.0, 1.0);
        let x = Vector::new(1.0, 0.0, 0.0);
        let y = Vector::new(0.0, 1.0, 0.0);
        let plane_from = Plane::new(Point::new(0.0, 0.0, 0.0), x.clone(), y.clone());
        let plane_to = Plane::new(box1_top, x.clone(), y.clone());
        let xy_to_top = Xform::plane_to_plane(&plane_from, &plane_to);
        scene.set_xform(&box1_guid, Xform::rotation_z(PI / 1.5, false) * xy_to_top);
        scene.set_xform(
            &box2_guid,
            Xform::translation(2.0, 0.0, 0.0) * Xform::rotation_z(PI / 6.0, false),
        );
        scene.set_xform(&box3_guid, Xform::translation(2.0, 0.0, 0.0));

        let world3 = scene.world_xform(&box3_guid);
        let expected = world3.transform_point(&Point::new(-1.0, -1.0, -1.0));
        let transformed = scene.get_geometry();
        let baked = transformed.meshes[2].vertex_point(0).unwrap();

        MINI_CHECK!(transformed.meshes.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(baked[0], expected[0]));
        MINI_CHECK!(TOLERANCE.is_close(baked[1], expected[1]));
        MINI_CHECK!(TOLERANCE.is_close(baked[2], expected[2]));
    })
}

pub fn run_session_add_component() -> TestResult {
    MINI_TEST!("Add Component", {
        use crate::Component;
        use crate::Session;

        let mut session = Session::default();

        let mut extra = std::collections::HashMap::new();
        extra.insert("size".to_string(), serde_json::json!(3000));
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
        MINI_CHECK!(session.component_lookup.contains_key(&guid));
        MINI_CHECK!(session.graph.has_node(&guid));
    })
}

pub fn run_session_component_json_roundtrip() -> TestResult {
    MINI_TEST!("Component Json Roundtrip", {
        use crate::file_encoders::file_json_dump;
        use crate::file_encoders::file_json_load;
        use crate::Component;
        use crate::Session;

        let mut original = Session::default();
        let mut extra = std::collections::HashMap::new();
        extra.insert("size".to_string(), serde_json::json!(3000));
        extra.insert("height".to_string(), serde_json::json!(650));
        extra.insert("rise".to_string(), serde_json::json!(453));
        let guid = uuid::Uuid::new_v4().to_string();
        let c = Component {
            type_name: "FloorBuilder".to_string(),
            guid: guid.clone(),
            name: "floor_builder".to_string(),
            extra,
        };
        original.add_component(c, None);

        let filename = "serialization/test_session_component.json";
        file_json_dump(&original, filename, false).unwrap();
        let loaded = file_json_load::<Session>(filename).unwrap();

        MINI_CHECK!(loaded.objects.components.len() == 1);
        MINI_CHECK!(loaded.objects.components[0].type_name == "FloorBuilder");
        MINI_CHECK!(loaded.objects.components[0].extra["size"] == serde_json::json!(3000));
        MINI_CHECK!(loaded.objects.components[0].guid == guid);
    })
}

pub fn run_session_document_workflow() -> TestResult {
    MINI_TEST!("Document Workflow", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let a = Point::new(1.0, 0.0, 0.0);
        let b = Point::new(2.0, 0.0, 0.0);
        let c = Point::new(3.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        let b_guid = b.guid().to_string();
        let c_guid = c.guid().to_string();
        session.add_point(a, None);
        session.add_point(b, None);
        session.add_point(c, None);

        session.replace(
            &b_guid,
            Geometry::Point(Rc::new(Point::new(20.0, 0.0, 0.0))),
        );
        session.remove_object(&c_guid);
        let shift = Xform::translation(0.0, 5.0, 0.0);
        session.set_xform(&a_guid, shift.clone());

        let fname = "serialization/test_session_document.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);

        MINI_CHECK!(loaded.lookup.len() == 2);
        MINI_CHECK!(loaded.lookup.contains_key(&a_guid));
        MINI_CHECK!(loaded.lookup.contains_key(&b_guid));
        MINI_CHECK!(!loaded.lookup.contains_key(&c_guid));
        MINI_CHECK!(TOLERANCE.is_close(
            Point::from_geometry(&loaded.lookup[&b_guid]).unwrap()[0],
            20.0
        ));
        MINI_CHECK!(loaded.xform(&a_guid) == shift);
        MINI_CHECK!(loaded.history.depth() == 0);
        MINI_CHECK!(session.objects.points.len() == 2);
        MINI_CHECK!(session.objects.points.number_of_slots() == 3);
    })
}

pub fn run_session_undo_remove() -> TestResult {
    MINI_TEST!("Undo Remove", {
        use crate::Point;
        use crate::Session;
        use crate::Xform;

        let mut session = Session::default();
        let group = session.add_group("g");
        let a = Point::new(1.0, 0.0, 0.0);
        let b = Point::new(2.0, 0.0, 0.0);
        let c = Point::new(3.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        let b_guid = b.guid().to_string();
        let c_guid = c.guid().to_string();
        session.add_point(a, Some(&group));
        let b_node = session.add_point(b, Some(&group));
        session.add_point(c, Some(&b_node));
        session.add_edge(&a_guid, &b_guid, "connection");
        let shift = Xform::translation(0.0, 5.0, 0.0);
        session.set_xform(&b_guid, shift.clone());
        let stored = std::rc::Rc::clone(&session.objects.points[1]);

        session.begin("remove");
        session.remove_object(&b_guid);
        session.commit();
        let gone = !session.lookup.contains_key(&b_guid) && group.borrow().children().len() == 1;
        session.undo();

        MINI_CHECK!(gone);
        MINI_CHECK!(session.lookup.contains_key(&b_guid));
        MINI_CHECK!(session.objects.points[1].guid() == b_guid);
        MINI_CHECK!(std::rc::Rc::ptr_eq(&session.objects.points[1], &stored));
        MINI_CHECK!(group.borrow().children()[1].borrow().name == b_guid);
        MINI_CHECK!(std::rc::Rc::ptr_eq(&group.borrow().children()[1], &b_node));
        MINI_CHECK!(
            group.borrow().children()[1].borrow().children()[0]
                .borrow()
                .name
                == c_guid
        );
        MINI_CHECK!(session.graph.has_edge((&a_guid, &b_guid)));
        MINI_CHECK!(
            session.graph.edge_label(&a_guid, &b_guid, None) == Some("connection".to_string())
        );
        MINI_CHECK!(session.xform(&b_guid) == shift);

        session.redo();

        MINI_CHECK!(!session.lookup.contains_key(&b_guid));
        MINI_CHECK!(session.objects.points.len() == 2);
        MINI_CHECK!(group.borrow().children().len() == 1);
        MINI_CHECK!(!session.graph.has_edge((&a_guid, &b_guid)));
        MINI_CHECK!(session.xform(&b_guid) == Xform::identity());
    })
}

pub fn run_session_undo_add() -> TestResult {
    MINI_TEST!("Undo Add", {
        use crate::session::FromGeometry;
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let group = session.add_group("g");
        session.add_point(Point::new(0.0, 0.0, 0.0), Some(&group));
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();

        session.begin("add");
        session.add_point(point, Some(&group));
        session.commit();
        session.undo();
        let gone = !session.lookup.contains_key(&guid) && session.objects.points.len() == 1;
        session.redo();

        MINI_CHECK!(gone);
        MINI_CHECK!(session.lookup.contains_key(&guid));
        MINI_CHECK!(session.objects.points[1].guid() == guid);
        MINI_CHECK!(session.objects.points.number_of_slots() == 2);
        MINI_CHECK!(group.borrow().children()[1].borrow().name == guid);
        MINI_CHECK!(session.graph.has_node(&guid));
        MINI_CHECK!(TOLERANCE.is_close(
            Point::from_geometry(&session.lookup[&guid]).unwrap()[2],
            3.0
        ));
    })
}

pub fn run_session_undo_replace() -> TestResult {
    MINI_TEST!("Undo Replace", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);

        session.begin("replace");
        session.replace(&guid, Geometry::Point(Rc::new(Point::new(9.0, 9.0, 9.0))));
        session.commit();
        let replaced = Point::from_geometry(&session.lookup[&guid]).unwrap()[0];
        session.undo();
        let restored = Point::from_geometry(&session.lookup[&guid]).unwrap()[0];
        session.redo();

        MINI_CHECK!(TOLERANCE.is_close(replaced, 9.0));
        MINI_CHECK!(TOLERANCE.is_close(restored, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(
            Point::from_geometry(&session.lookup[&guid]).unwrap()[0],
            9.0
        ));
        MINI_CHECK!(session.objects.points[0].guid() == guid);
        MINI_CHECK!(session.objects.points.len() == 1);
        MINI_CHECK!(session.objects.points.number_of_slots() == 1);
    })
}

pub fn run_session_undo_xform() -> TestResult {
    MINI_TEST!("Undo Xform", {
        use crate::Point;
        use crate::Session;
        use crate::Xform;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();
        session.add_point(point, None);
        let shift = Xform::translation(5.0, 0.0, 0.0);

        session.begin("move");
        session.set_xform(&guid, shift.clone());
        session.commit();
        session.undo();
        let cleared = session.xform(&guid) == Xform::identity();
        session.redo();

        session.begin("reset");
        session.remove_xform(&guid);
        session.commit();
        session.undo();

        MINI_CHECK!(cleared);
        MINI_CHECK!(session.xform(&guid) == shift);
        MINI_CHECK!(session.xforms.len() == 1);
        MINI_CHECK!(session.history.undo_stack[0].ops[0].kind() == "xform");
    })
}

pub fn run_session_history_purged_on_save() -> TestResult {
    MINI_TEST!("History Purged On Save", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();

        session.begin("add");
        session.add_point(Point::new(0.0, 0.0, 0.0), None);
        session.commit();
        let before_pb = session.history.depth();
        session.pb_dumps();
        let after_pb = session.history.depth();

        session.begin("add");
        session.add_point(Point::new(1.0, 0.0, 0.0), None);
        session.commit();
        let before_json = session.history.depth();
        session.file_json_dumps();

        MINI_CHECK!(before_pb == 1);
        MINI_CHECK!(after_pb == 0);
        MINI_CHECK!(before_json == 1);
        MINI_CHECK!(session.history.depth() == 0);
        MINI_CHECK!(!session.undo());
        MINI_CHECK!(session.objects.points.len() == 2);
    })
}

pub fn run_session_history_capacity() -> TestResult {
    MINI_TEST!("History Capacity", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();

        for i in 0..70 {
            session.begin("add");
            session.add_point(Point::new(i as f64, 0.0, 0.0), None);
            session.commit();
        }

        let depth = session.history.depth();

        while session.undo() {}

        MINI_CHECK!(depth == 64);
        MINI_CHECK!(!session.history.can_undo());
        MINI_CHECK!(session.objects.points.len() == 6);
        MINI_CHECK!(session.objects.points.number_of_slots() == 70);
        MINI_CHECK!(session.history.dropped == 6);
        MINI_CHECK!(TOLERANCE.is_close(session.objects.points[5][0], 5.0));
    })
}

pub fn run_session_str_hierarchy() -> TestResult {
    MINI_TEST!("Str Hierarchy", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::new("blocks");
        let group = session.add_group("Group");
        session.add_point(Point::new(0.0, 0.0, 0.0), Some(&group));
        session.add_point(Point::new(1.0, 0.0, 0.0), Some(&group));
        let text = session.str();

        MINI_CHECK!(text.contains("Spatial Hierarchy"));
        MINI_CHECK!(text.contains("Element Interactions"));
        MINI_CHECK!(text.contains("\u{2514}\u{2500}\u{2500} "));
        MINI_CHECK!(text.contains("<Tree with "));
        MINI_CHECK!(text.contains("<Graph with "));
        MINI_CHECK!(session.repr().starts_with("Session(name=blocks"));
    })
}

pub fn run_session_add_definition() -> TestResult {
    MINI_TEST!("Add Definition", {
        use crate::Geometry;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let bx = Geometry::Mesh(Rc::new(create_box(&Point::new(0.0, 0.0, 0.0), 2.0)));
        let guid = session.add_definition(bx.clone());
        let again = session.add_definition(bx.clone());
        session.set_xform(&guid, Xform::translation(1.0, 0.0, 0.0));
        let point = Point::new(1.0, 2.0, 3.0);
        let point_guid = point.guid().to_string();
        session.add_point(point, None);
        let taken = session.add_definition(session.lookup[&point_guid].clone());

        MINI_CHECK!(guid == bx.guid());
        MINI_CHECK!(again == guid);
        MINI_CHECK!(taken.is_empty());
        MINI_CHECK!(session.definitions.meshes.len() == 1);
        MINI_CHECK!(session.definition_lookup.contains_key(&guid));
        MINI_CHECK!(!session.lookup.contains_key(&guid));
        MINI_CHECK!(session.order().len() == 1);
        MINI_CHECK!(!session.graph.has_node(&guid));
        MINI_CHECK!(session.tree.get_node_by_name(&guid).is_none());
        MINI_CHECK!(session.xforms.is_empty());
    })
}

pub fn run_session_add_instance() -> TestResult {
    MINI_TEST!("Add Instance", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let group = session.add_group("bay");
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let mut instance = InstanceRef::new(&definition, Xform::translation(0.0, 0.0, 3.0));
        instance.name = "column".to_string();
        let guid = instance.guid().to_string();
        let node = session
            .add_instance(instance, Xform::translation(10.0, 0.0, 0.0), Some(&group))
            .unwrap();
        let orphan = session.add_instance(
            InstanceRef::new("missing", Xform::identity()),
            Xform::identity(),
            None,
        );

        MINI_CHECK!(node.borrow().name == guid);
        MINI_CHECK!(group.borrow().children()[0].borrow().name == guid);
        MINI_CHECK!(session.graph.node_label(&guid, None) == Some("instance_column".to_string()));
        MINI_CHECK!(session.objects.instances.len() == 1);
        MINI_CHECK!(session.instance_lookup[&guid].xform == Xform::identity());
        MINI_CHECK!(session.xform(&guid) == Xform::translation(10.0, 0.0, 3.0));
        MINI_CHECK!(orphan.is_none());
        MINI_CHECK!(session.order().is_empty());
    })
}

pub fn run_session_definition_of() -> TestResult {
    MINI_TEST!("Definition Of", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Mesh;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let instance = InstanceRef::new(&definition, Xform::identity());
        let guid = instance.guid().to_string();
        session.add_instance(instance, Xform::identity(), None);
        let found = session.definition_of(&guid);

        MINI_CHECK!(found.is_some());
        MINI_CHECK!(Mesh::from_geometry(found.as_ref().unwrap()).unwrap().guid() == definition);
        MINI_CHECK!(session.definition_of(&definition).is_none());
        MINI_CHECK!(session.definition_of("missing").is_none());
    })
}

pub fn run_session_instances_of() -> TestResult {
    MINI_TEST!("Instances Of", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let first = InstanceRef::new(&definition, Xform::identity());
        let second = InstanceRef::new(&definition, Xform::identity());
        let first_guid = first.guid().to_string();
        let second_guid = second.guid().to_string();
        session.add_instance(first, Xform::identity(), None);
        session.add_instance(second, Xform::identity(), None);
        let guids = session.instances_of(&definition);

        MINI_CHECK!(guids.len() == 2);
        MINI_CHECK!(guids[0] == first_guid);
        MINI_CHECK!(guids[1] == second_guid);
        MINI_CHECK!(session.instances_of("missing").is_empty());
    })
}

pub fn run_session_world_geometry() -> TestResult {
    MINI_TEST!("World Geometry", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Mesh;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let mut instance = InstanceRef::new(&definition, Xform::identity());
        instance.name = "box".to_string();
        let guid = instance.guid().to_string();
        session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0), None);
        let point = Point::new(1.0, 2.0, 3.0);
        let point_guid = point.guid().to_string();
        session.add_point(point, None);
        session.set_xform(&point_guid, Xform::translation(0.0, 0.0, 5.0));

        let resolved = session.world_geometry(&guid).unwrap();
        let placed = session.world_geometry(&point_guid).unwrap();
        let mesh = Mesh::from_geometry(&resolved).unwrap();
        let moved = Point::from_geometry(&placed).unwrap();
        let local = Mesh::from_geometry(&session.definition_lookup[&definition]).unwrap();

        MINI_CHECK!(mesh.guid() == guid);
        MINI_CHECK!(mesh.name == "box");
        MINI_CHECK!(TOLERANCE.is_close(mesh.vertex_point(0).unwrap()[0], 9.0));
        MINI_CHECK!(TOLERANCE.is_close(local.vertex_point(0).unwrap()[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(moved[2], 8.0));
        MINI_CHECK!(TOLERANCE.is_close(
            Point::from_geometry(&session.lookup[&point_guid]).unwrap()[2],
            3.0
        ));
        MINI_CHECK!(session.world_geometry("missing").is_none());
    })
}

pub fn run_session_get_geometry_resolves_instances() -> TestResult {
    MINI_TEST!("Get Geometry Resolves Instances", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let group = session.add_group("row");
        session.set_xform("row", Xform::translation(0.0, 5.0, 0.0));
        session.add_instance(
            InstanceRef::new(&definition, Xform::identity()),
            Xform::translation(10.0, 0.0, 0.0),
            Some(&group),
        );
        session.add_instance(
            InstanceRef::new(&definition, Xform::identity()),
            Xform::translation(20.0, 0.0, 0.0),
            Some(&group),
        );

        let geometry = session.get_geometry();
        let corner = geometry.meshes[1].vertex_point(0).unwrap();

        MINI_CHECK!(geometry.instances.is_empty());
        MINI_CHECK!(geometry.meshes.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(corner[0], 19.0));
        MINI_CHECK!(TOLERANCE.is_close(corner[1], 4.0));
        MINI_CHECK!(session.objects.instances.len() == 2);
        MINI_CHECK!(session.objects.meshes.is_empty());
    })
}

pub fn run_session_replace_definition() -> TestResult {
    MINI_TEST!("Replace Definition", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Mesh;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let first = InstanceRef::new(&definition, Xform::identity());
        let second = InstanceRef::new(&definition, Xform::identity());
        let second_guid = second.guid().to_string();
        session.add_instance(first, Xform::translation(10.0, 0.0, 0.0), None);
        session.add_instance(second, Xform::translation(20.0, 0.0, 0.0), None);

        let replaced = session.replace_definition(
            &definition,
            Geometry::Mesh(Rc::new(create_box(&Point::new(0.0, 0.0, 0.0), 4.0))),
        );
        let missing = session.replace_definition(
            "missing",
            Geometry::Mesh(Rc::new(create_box(&Point::new(0.0, 0.0, 0.0), 4.0))),
        );
        let resolved = session.world_geometry(&second_guid).unwrap();
        let mesh = Mesh::from_geometry(&resolved).unwrap();

        MINI_CHECK!(replaced);
        MINI_CHECK!(!missing);
        MINI_CHECK!(session.definitions.meshes.len() == 1);
        MINI_CHECK!(session.definitions.meshes[0].guid() == definition);
        MINI_CHECK!(TOLERANCE.is_close(mesh.vertex_point(0).unwrap()[0], 18.0));
    })
}

pub fn run_session_remove_definition() -> TestResult {
    MINI_TEST!("Remove Definition", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let instance = InstanceRef::new(&definition, Xform::identity());
        let guid = instance.guid().to_string();
        session.add_instance(instance, Xform::identity(), None);

        let refused = !session.remove_definition(&definition);
        session.remove_object(&guid);
        let removed = session.remove_definition(&definition);

        MINI_CHECK!(refused);
        MINI_CHECK!(removed);
        MINI_CHECK!(session.definitions.meshes.is_empty());
        MINI_CHECK!(session.definitions.meshes.number_of_dead() == 1);
        MINI_CHECK!(session.definition_lookup.is_empty());
        MINI_CHECK!(session.objects.instances.is_empty());
        MINI_CHECK!(!session.remove_definition("missing"));
    })
}

pub fn run_session_to_instance() -> TestResult {
    MINI_TEST!("To Instance", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::Mesh;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let group = session.add_group("bay");
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let point = Point::new(0.0, 0.0, 0.0);
        let point_guid = point.guid().to_string();
        let mut bx = create_box(&Point::new(5.0, 0.0, 0.0), 2.0);
        bx.name = "column".to_string();
        let guid = bx.guid().to_string();
        session.add_point(point, Some(&group));
        session.add_mesh(bx, Some(&group));
        session.add_edge(&point_guid, &guid, "contact");
        session.set_xform(&guid, Xform::translation(0.0, 0.0, 1.0));

        let placed = session.world_geometry(&guid).unwrap();
        let before = Mesh::from_geometry(&placed)
            .unwrap()
            .vertex_point(0)
            .unwrap();
        session.begin("to instance");
        let converted = session.to_instance(&guid, &definition, Xform::translation(5.0, 0.0, 0.0));
        session.commit();
        let resolved = session.world_geometry(&guid).unwrap();
        let after = Mesh::from_geometry(&resolved)
            .unwrap()
            .vertex_point(0)
            .unwrap();

        MINI_CHECK!(converted);
        MINI_CHECK!(session.objects.meshes.is_empty());
        MINI_CHECK!(session.objects.meshes.number_of_slots() == 1);
        MINI_CHECK!(session.history.can_undo());
        MINI_CHECK!(session.instance_lookup[&guid].name == "column");
        MINI_CHECK!(session.instance_lookup[&guid].definition_guid == definition);
        MINI_CHECK!(group.borrow().children()[1].borrow().name == guid);
        MINI_CHECK!(session.graph.has_edge((&point_guid, &guid)));
        MINI_CHECK!(session.graph.node_label(&guid, None) == Some("instance_column".to_string()));
        MINI_CHECK!(TOLERANCE.is_close(before[0], after[0]));
        MINI_CHECK!(TOLERANCE.is_close(before[2], after[2]));
        MINI_CHECK!(!session.to_instance(&guid, &definition, Xform::identity()));
    })
}

pub fn run_session_explode() -> TestResult {
    MINI_TEST!("Explode", {
        use crate::element::ElementFeature;
        use crate::session::FromGeometry;
        use crate::Element;
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Polyline;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Element(Rc::new(Element::from_mesh(
            create_box(&Point::new(0.0, 0.0, 0.0), 2.0),
            "plate",
        ))));
        let mut instance = InstanceRef::new(&definition, Xform::identity());
        instance.name = "deck".to_string();
        instance.features.push(ElementFeature::new(
            "contact",
            0,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
            ])],
            "",
        ));
        let guid = instance.guid().to_string();
        let feature = instance.features[0].guid().to_string();
        let point = Point::new(0.0, 0.0, 0.0);
        let point_guid = point.guid().to_string();
        session.add_point(point, None);
        session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0), None);
        session.add_edge(&point_guid, &guid, "contact");

        let exploded = session.explode(&guid);
        let element = Element::from_geometry(&session.lookup[&guid]).unwrap();

        MINI_CHECK!(exploded);
        MINI_CHECK!(session.objects.instances.is_empty());
        MINI_CHECK!(session.objects.instances.number_of_dead() == 1);
        MINI_CHECK!(element.name == "deck");
        MINI_CHECK!(element.features().len() == 1);
        MINI_CHECK!(element.features()[0].guid() == feature);
        MINI_CHECK!(session.xform(&guid) == Xform::translation(10.0, 0.0, 0.0));
        MINI_CHECK!(session.graph.has_edge((&point_guid, &guid)));
        MINI_CHECK!(session.graph.node_label(&guid, None) == Some("element_deck".to_string()));
        MINI_CHECK!(session.definitions.elements.len() == 1);
        MINI_CHECK!(!session.explode(&guid));
    })
}

pub fn run_session_undo_instance() -> TestResult {
    MINI_TEST!("Undo Instance", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let group = session.add_group("bay");
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let point = Point::new(0.0, 0.0, 0.0);
        let point_guid = point.guid().to_string();
        let bx = create_box(&Point::new(0.0, 0.0, 0.0), 2.0);
        let guid = bx.guid().to_string();
        session.add_point(point, Some(&group));
        session.add_mesh(bx, Some(&group));
        session.add_edge(&point_guid, &guid, "contact");
        let edge = session.graph.edges[&point_guid][&guid].guid().to_string();
        let order = session.order();
        let tree = session.tree.str();
        let label = session.graph.node_label(&guid, None);

        session.begin("to instance");
        session.to_instance(&guid, &definition, Xform::identity());
        session.commit();
        session.begin("explode");
        session.explode(&guid);
        session.commit();
        session.undo();
        let instanced = session.instance_lookup.contains_key(&guid) && session.tree.str() == tree;
        session.undo();

        let instance = InstanceRef::new(&definition, Xform::identity());
        let added = instance.guid().to_string();
        session.begin("add");
        session.add_instance(instance, Xform::translation(1.0, 0.0, 0.0), Some(&group));
        session.commit();
        session.undo();
        let gone = session.instance_lookup.is_empty() && session.xforms.is_empty();
        session.redo();

        MINI_CHECK!(instanced);
        MINI_CHECK!(gone);
        MINI_CHECK!(session.objects.meshes[0].guid() == guid);
        MINI_CHECK!(session.objects.meshes.number_of_slots() == 2);
        MINI_CHECK!(session.objects.instances.len() == 1);
        MINI_CHECK!(session.graph.has_edge((&point_guid, &guid)));
        MINI_CHECK!(session.graph.edges[&guid][&point_guid].guid() == edge);
        MINI_CHECK!(session.graph.node_label(&guid, None) == label);
        MINI_CHECK!(session.order() == order);
        MINI_CHECK!(session.xform(&added) == Xform::translation(1.0, 0.0, 0.0));
        MINI_CHECK!(group.borrow().children()[2].borrow().name == added);
    })
}

pub fn run_session_instance_json_roundtrip() -> TestResult {
    MINI_TEST!("Instance Json Roundtrip", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let instance = InstanceRef::new(&definition, Xform::identity());
        let guid = instance.guid().to_string();
        session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0), None);

        let fname = "serialization/test_session_instance.json";
        session.file_json_dump(fname);
        let loaded = Session::file_json_load(fname);
        let mut data: serde_json::Value =
            serde_json::from_str(&session.jsondump().unwrap()).unwrap();
        data["objects"]["instances"][0]["xform"] =
            serde_json::to_value(Xform::translation(0.0, 0.0, 1.0)).unwrap();
        let folded = Session::jsonload(&data.to_string()).unwrap();

        MINI_CHECK!(loaded.definitions.meshes.len() == 1);
        MINI_CHECK!(loaded.instance_lookup.contains_key(&guid));
        MINI_CHECK!(loaded.definition_of(&guid).is_some());
        MINI_CHECK!(loaded.xform(&guid) == Xform::translation(10.0, 0.0, 0.0));
        MINI_CHECK!(folded.xform(&guid) == Xform::translation(10.0, 0.0, 1.0));
        MINI_CHECK!(folded.instance_lookup[&guid].xform == Xform::identity());
    })
}

pub fn run_session_instance_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Instance Protobuf Roundtrip", {
        use crate::element::ElementFeature;
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Polyline;
        use crate::Session;
        use crate::Xform;
        use prost::Message;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let mut instance = InstanceRef::new(&definition, Xform::identity());
        instance.features.push(ElementFeature::new(
            "contact",
            0,
            vec![Polyline::new(vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
            ])],
            "",
        ));
        let guid = instance.guid().to_string();
        let feature = instance.features[0].guid().to_string();
        session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0), None);

        let fname = "serialization/test_session_instance.bin";
        session.pb_dump(fname);
        let loaded = Session::pb_load(fname);
        let plain =
            crate::proto::Session::decode(Session::default().pb_dumps().as_slice()).unwrap();

        MINI_CHECK!(loaded.definitions.meshes.len() == 1);
        MINI_CHECK!(loaded.instance_lookup[&guid].features.len() == 1);
        MINI_CHECK!(loaded.instance_lookup[&guid].features[0].guid() == feature);
        MINI_CHECK!(loaded.definition_of(&guid).is_some());
        MINI_CHECK!(loaded.xform(&guid) == Xform::translation(10.0, 0.0, 0.0));
        MINI_CHECK!(plain.definitions.is_none());
    })
}

pub fn run_session_get_collisions_instances() -> TestResult {
    MINI_TEST!("Get Collisions Instances", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let first = InstanceRef::new(&definition, Xform::identity());
        let second = InstanceRef::new(&definition, Xform::identity());
        let third = InstanceRef::new(&definition, Xform::identity());
        let first_guid = first.guid().to_string();
        let second_guid = second.guid().to_string();
        let third_guid = third.guid().to_string();
        session.add_instance(first, Xform::identity(), None);
        session.add_instance(second, Xform::translation(1.0, 0.0, 0.0), None);
        session.add_instance(third, Xform::translation(100.0, 0.0, 0.0), None);

        let pairs = session.get_collisions();

        MINI_CHECK!(pairs.len() == 1);
        MINI_CHECK!(session.graph.has_edge((&first_guid, &second_guid)));
        MINI_CHECK!(!session.graph.has_edge((&first_guid, &third_guid)));
    })
}

pub fn run_session_ray_cast_instance() -> TestResult {
    MINI_TEST!("Ray Cast Instance", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Vector;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let instance = InstanceRef::new(&definition, Xform::identity());
        let guid = instance.guid().to_string();
        session.add_instance(instance, Xform::translation(100.0, 0.0, 0.0), None);

        let hits = session.ray_cast(
            &Point::new(100.0, 0.0, 5.0),
            &Vector::new(0.0, 0.0, -1.0),
            1e-3,
        );

        MINI_CHECK!(hits.len() == 1);
        MINI_CHECK!(hits[0].guid == guid);
        MINI_CHECK!(TOLERANCE.is_close(hits[0].hit_point[0], 100.0));
        MINI_CHECK!(TOLERANCE.is_close(hits[0].hit_point[2], 1.0));
    })
}

pub fn run_session_get_node() -> TestResult {
    MINI_TEST!("Get Node", {
        use crate::Point;
        use crate::Session;
        use crate::Tree;
        use crate::TreeNode;
        use std::rc::Rc;

        let mut session = Session::default();
        let node = session.add_point(Point::new(0.0, 0.0, 0.0), None);
        let guid = node.borrow().name.clone();
        let child = session.add_point(Point::new(1.0, 0.0, 0.0), Some(&node));
        let child_guid = child.borrow().name.clone();
        let found = session.get_node(&guid);
        session.begin("remove");
        session.remove_object(&guid);
        session.commit();
        let removed = session.get_node(&guid);
        let orphaned = session.get_node(&child_guid);
        session.undo();
        let restored = session.get_node(&guid);
        let reattached = session.get_node(&child_guid);
        let indexed = session
            .node_lookup
            .get(&child_guid)
            .is_some_and(|n| Rc::ptr_eq(n, &child));
        let mut tree = Tree::new("swapped");
        tree.add(&TreeNode::new("root"), None);
        let root = tree.root().unwrap();
        let swapped = TreeNode::new(&guid);
        tree.add(&swapped, Some(&root));
        session.tree = tree;
        let searched = session.get_node(&guid);
        session.reindex();

        MINI_CHECK!(found.is_some_and(|n| Rc::ptr_eq(&n, &node)));
        MINI_CHECK!(removed.is_none());
        MINI_CHECK!(orphaned.is_some_and(|n| Rc::ptr_eq(&n, &child)));
        MINI_CHECK!(restored.is_some_and(|n| Rc::ptr_eq(&n, &node)));
        MINI_CHECK!(reattached.is_some_and(|n| Rc::ptr_eq(&n, &child)) && indexed);
        MINI_CHECK!(searched.is_some_and(|n| Rc::ptr_eq(&n, &swapped)));
        MINI_CHECK!(Rc::ptr_eq(&session.node_lookup[&guid], &swapped));
        MINI_CHECK!(session.get_node("missing").is_none());
    })
}

pub fn run_session_remove_keeps_slot() -> TestResult {
    MINI_TEST!("Remove Keeps Slot", {
        use crate::history::Op;
        use crate::Point;
        use crate::Session;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let g = session.add_group("g");
        let a = Point::new(1.0, 0.0, 0.0);
        let b = Point::new(2.0, 0.0, 0.0);
        let c = Point::new(3.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        let b_guid = b.guid().to_string();
        let c_guid = c.guid().to_string();
        session.add_point(a, Some(&g));
        let b_node = session.add_point(b, Some(&g));
        session.add_point(c, Some(&g));
        session.set_xform(&b_guid, Xform::translation(0.0, 1.0, 0.0));
        let stored = Rc::clone(&session.objects.points[1]);

        session.begin("remove");
        session.remove_object(&b_guid);
        session.commit();
        let pinned = match &session.history.undo_stack[0].ops[0] {
            Op::Remove(t) => t.tomb.node.as_ref().is_some_and(|n| Rc::ptr_eq(n, &b_node)),
            _ => false,
        };

        MINI_CHECK!(session.objects.points.len() == 2);
        MINI_CHECK!(session.objects.points.number_of_slots() == 3);
        MINI_CHECK!(Rc::ptr_eq(session.objects.points.get_item(1), &stored));
        MINI_CHECK!(!session.lookup.contains_key(&b_guid));
        MINI_CHECK!(!session.xforms.contains_key(&b_guid));
        MINI_CHECK!(!session.graph.has_node(&b_guid));
        MINI_CHECK!(session.get_node(&b_guid).is_none());
        MINI_CHECK!(pinned);

        session.undo();
        let names: Vec<String> = g
            .borrow()
            .children()
            .iter()
            .map(|n| n.borrow().name.clone())
            .collect();

        MINI_CHECK!(Rc::ptr_eq(session.objects.points.get_item(1), &stored));
        MINI_CHECK!(session.objects.points.get_slot(&b_guid) == Some(1));
        MINI_CHECK!(session.order() == vec![a_guid.clone(), b_guid.clone(), c_guid.clone()]);
        MINI_CHECK!(names == vec![a_guid, b_guid, c_guid]);
        MINI_CHECK!(Rc::ptr_eq(&g.borrow().children()[1], &b_node));
        MINI_CHECK!(b_node.borrow().at() == 1);
    })
}

pub fn run_session_redo_add_keeps_node() -> TestResult {
    MINI_TEST!("Redo Add Keeps Node", {
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let a = Point::new(1.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();

        session.begin("add");
        let node = session.add_point(a, None);
        session.commit();
        let node_guid = node.borrow().guid().to_string();
        let stored = Rc::clone(&session.objects.points[0]);
        session.undo();
        let gone = session.get_node(&a_guid).is_none() && session.objects.points.is_empty();
        session.redo();
        let found = session.get_node(&a_guid);

        MINI_CHECK!(gone);
        MINI_CHECK!(found.is_some_and(|n| Rc::ptr_eq(&n, &node)));
        MINI_CHECK!(node.borrow().guid() == node_guid);
        MINI_CHECK!(Rc::ptr_eq(&session.objects.points[0], &stored));
        MINI_CHECK!(session.objects.points.get_slot(&a_guid) == Some(0));
        MINI_CHECK!(session.tree.root().unwrap().borrow().children().len() == 1);
    })
}

pub fn run_session_replace_shares_geometry() -> TestResult {
    MINI_TEST!("Replace Shares Geometry", {
        use crate::history::Op;
        use crate::Geometry;
        use crate::Item;
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let a = Point::new(1.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        session.add_point(a, None);
        let original = Rc::clone(&session.objects.points[0]);
        let mut p2 = Point::new(9.0, 9.0, 9.0);
        p2.set_guid(a_guid.clone());
        let p2 = Rc::new(p2);

        session.begin("replace");
        session.replace(&a_guid, Geometry::Point(Rc::clone(&p2)));
        session.commit();
        let shared = match &session.history.undo_stack[0].ops[0] {
            Op::Replace(op) => match (&op.before, &op.after) {
                (Item::Geometry(Geometry::Point(x)), Item::Geometry(Geometry::Point(y))) => {
                    Rc::ptr_eq(x, &original) && Rc::ptr_eq(y, &p2)
                }
                _ => false,
            },
            _ => false,
        };
        let swapped = Rc::ptr_eq(&session.objects.points[0], &p2);
        session.undo();
        let restored = Rc::ptr_eq(&session.objects.points[0], &original)
            && session.objects.points.get_slot(&a_guid) == Some(0);
        session.redo();

        MINI_CHECK!(shared);
        MINI_CHECK!(swapped);
        MINI_CHECK!(restored);
        MINI_CHECK!(Rc::ptr_eq(&session.objects.points[0], &p2));
        MINI_CHECK!(matches!(&session.lookup[&a_guid], Geometry::Point(p) if Rc::ptr_eq(p, &p2)));
    })
}

pub fn run_session_replace_across_types() -> TestResult {
    MINI_TEST!("Replace Across Types", {
        use crate::Geometry;
        use crate::Line;
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let a = Point::new(1.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        let node = session.add_point(a, None);
        let mut line = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        line.name = "edge".to_string();

        session.begin("replace");
        let replaced = session.replace(&a_guid, Geometry::Line(Rc::new(line)));
        session.commit();
        let label = session.graph.node_label(&a_guid, None);
        let same_node = session
            .get_node(&a_guid)
            .is_some_and(|n| Rc::ptr_eq(&n, &node));
        let counts = (session.objects.points.len(), session.objects.lines.len());
        session.undo();
        let undone = (session.objects.points.len(), session.objects.lines.len());
        let restored = session.graph.node_label(&a_guid, None);
        session.redo();

        MINI_CHECK!(replaced);
        MINI_CHECK!(counts == (0, 1));
        MINI_CHECK!(same_node);
        MINI_CHECK!(label == Some("line_edge".to_string()));
        MINI_CHECK!(undone == (1, 0));
        MINI_CHECK!(restored == Some("point_my_point".to_string()));
        MINI_CHECK!(session.objects.lines.len() == 1);
        MINI_CHECK!(session.objects.points.is_empty());
        MINI_CHECK!(matches!(session.lookup[&a_guid], Geometry::Line(_)));
        MINI_CHECK!(session
            .get_node(&a_guid)
            .is_some_and(|n| Rc::ptr_eq(&n, &node)));
    })
}

pub fn run_session_undo_restores_graph() -> TestResult {
    MINI_TEST!("Undo Restores Graph", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let a = Point::new(0.0, 0.0, 0.0);
        let b = Point::new(1.0, 0.0, 0.0);
        let c = Point::new(2.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        let b_guid = b.guid().to_string();
        let c_guid = c.guid().to_string();
        session.add_point(a, None);
        session.add_point(b, None);
        session.add_point(c, None);
        session.graph.set_vertex_attribute(&a_guid, "mass", 2.0);
        session.add_edge(&a_guid, &b_guid, "joint");
        session
            .graph
            .set_edge_attribute((&a_guid, &b_guid), "load", 1.5);
        session.add_edge(&a_guid, &c_guid, "contact");
        session.add_edge(&b_guid, &c_guid, "contact");
        let before = session.graph.jsondump().unwrap();

        session.begin("remove");
        session.remove_object(&a_guid);
        session.commit();
        let taken = !session.graph.has_node(&a_guid) && session.graph.number_of_edges() == 1;
        session.undo();
        let after = session.graph.jsondump().unwrap();
        session.redo();

        MINI_CHECK!(taken);
        MINI_CHECK!(before == after);
        MINI_CHECK!(!session.graph.has_node(&a_guid));
        MINI_CHECK!(session.graph.has_edge((&b_guid, &c_guid)));
        MINI_CHECK!(session.graph.number_of_vertices() == 2);

        let c_node = session.get_node(&c_guid).unwrap();
        session.tree.remove(&c_node);
        session.remove_object(&c_guid);

        MINI_CHECK!(!session.graph.has_node(&c_guid));
    })
}

pub fn run_session_undo_restores_interactions() -> TestResult {
    MINI_TEST!("Undo Restores Interactions", {
        use crate::Element;
        use crate::Session;

        let mut session = Session::default();
        session.add_element(Element::new("a"), None);
        session.add_element(Element::new("b"), None);
        session.add_element(Element::new("c"), None);
        let a = session.objects.elements[0].clone();
        let b = session.objects.elements[1].clone();
        let c = session.objects.elements[2].clone();
        let glue = session
            .add_interaction(&a, &b, Box::new(NamedInteraction::new("glue")))
            .unwrap()
            .guid()
            .to_string();
        let nail = session
            .add_interaction(&a, &c, Box::new(NamedInteraction::new("nail")))
            .unwrap()
            .guid()
            .to_string();
        let ab = session.graph.edges[a.guid()][b.guid()].guid().to_string();
        let ac = session.graph.edges[a.guid()][c.guid()].guid().to_string();

        session.begin("remove");
        session.remove_object(a.guid());
        session.commit();
        let parked = session.interactions.is_empty();
        session.undo();
        session.redo();
        session.undo();

        MINI_CHECK!(parked);
        MINI_CHECK!(session.interactions.len() == 2);
        MINI_CHECK!(session.graph.edges[a.guid()][b.guid()].guid() == ab);
        MINI_CHECK!(session.graph.edges[a.guid()][c.guid()].guid() == ac);
        MINI_CHECK!(session.get_interaction(&a, &b)[0].guid() == glue);
        MINI_CHECK!(session.get_interaction(&a, &c)[0].guid() == nail);
        MINI_CHECK!(session.get_interaction(&a, &c)[0].name() == "nail");
    })
}

pub fn run_session_dead_guid_reused() -> TestResult {
    MINI_TEST!("Dead Guid Reused", {
        use crate::session::FromGeometry;
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let x = Point::new(1.0, 0.0, 0.0);
        let x_guid = x.guid().to_string();
        session.add_point(x, None);
        let mut again = Point::new(9.0, 0.0, 0.0);
        again.set_guid(x_guid.clone());

        session.begin("reuse");
        session.remove_object(&x_guid);
        session.add_point(again, None);
        session.commit();
        let first = Point::from_geometry(&session.lookup[&x_guid]).unwrap()[0];
        let slots = (
            session.objects.points.is_dead(0),
            session.objects.points.get_slot(&x_guid),
        );
        let count = session.objects.points.len();
        session.undo();
        let second = Point::from_geometry(&session.lookup[&x_guid]).unwrap()[0];
        let undone = (
            session.objects.points.is_dead(1),
            session.objects.points.get_slot(&x_guid),
        );
        let still = session.objects.points.len();
        session.redo();

        MINI_CHECK!(TOLERANCE.is_close(first, 9.0));
        MINI_CHECK!(slots == (true, Some(1)));
        MINI_CHECK!(count == 1);
        MINI_CHECK!(TOLERANCE.is_close(second, 1.0));
        MINI_CHECK!(undone == (true, Some(0)));
        MINI_CHECK!(still == 1);
        MINI_CHECK!(session.objects.points.get_slot(&x_guid) == Some(1));
        MINI_CHECK!(session.objects.points.len() == 1);
    })
}

pub fn run_session_remove_keeps_lookup_edit() -> TestResult {
    MINI_TEST!("Remove Keeps Lookup Edit", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let a = Point::new(1.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        session.add_point(a, None);

        if let Some(Geometry::Point(point)) = session.lookup.get_mut(&a_guid) {
            Rc::make_mut(point)[0] = 7.0;
        }

        session.begin("remove");
        session.remove_object(&a_guid);
        session.commit();
        session.undo();
        let json = session.jsondump().unwrap();

        MINI_CHECK!(TOLERANCE.is_close(
            Point::from_geometry(&session.lookup[&a_guid]).unwrap()[0],
            7.0
        ));
        MINI_CHECK!(TOLERANCE.is_close(session.objects.points[0][0], 7.0));
        MINI_CHECK!(json.contains("7.0"));
    })
}

pub fn run_session_tree_ops() -> TestResult {
    MINI_TEST!("Tree Ops", {
        use crate::Color;
        use crate::Point;
        use crate::Session;
        use crate::TreeNode;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        session.add_group("A");
        let root = session.tree.root().unwrap();
        let mut snapshots = vec![session.tree.str()];

        session.begin("group");
        let node = session.add_group("L");
        session.commit();
        snapshots.push(session.tree.str());

        session.begin("rename");
        let renamed = session.rename_node(&node, "M");
        session.commit();
        snapshots.push(session.tree.str());

        session.begin("colour");
        let coloured = session.set_node_color(&node, Some(Color::new(1.0, 0.0, 0.0, 1.0)));
        session.commit();
        snapshots.push(session.tree.str());

        session.begin("remove");
        let removed = session.remove_group(&node);
        session.commit();
        let dead = node.borrow().is_dead() && root.borrow().children().len() == 1;
        let mut restored = Vec::new();

        for i in (0..4).rev() {
            session.undo();
            restored.push(session.tree.str() == snapshots[i]);
        }

        let absent = node.borrow().is_dead() && session.tree.get_node_by_name("L").is_none();
        let mut redone = Vec::new();

        for i in 1..=4 {
            session.redo();
            redone.push(i == 4 || session.tree.str() == snapshots[i]);
        }

        MINI_CHECK!(renamed && coloured && removed);
        MINI_CHECK!(dead);
        MINI_CHECK!(restored == vec![true, true, true, true]);
        MINI_CHECK!(absent);
        MINI_CHECK!(redone == vec![true, true, true, true]);
        MINI_CHECK!(node.borrow().is_dead());
        MINI_CHECK!(node.borrow().name == "M");
        MINI_CHECK!(node.borrow().at() == 1);
        MINI_CHECK!(node.borrow().color.is_some());
        MINI_CHECK!(Rc::ptr_eq(&session.tree.root().unwrap(), &root));
        MINI_CHECK!(session.history.undo_stack[3].ops[0].kind() == "tree");

        let point = Point::new(0.0, 0.0, 0.0);
        let guid = point.guid().to_string();
        let held = session.add_point(point, None);
        session.tree.remove(&held);
        session.set_xform(&guid, Xform::translation(1.0, 0.0, 0.0));
        session.begin("adopt");
        session.add(&TreeNode::new(&guid), None);
        session.commit();
        session.undo();

        MINI_CHECK!(session.get_node(&guid).is_none());
        MINI_CHECK!(session.xforms.contains_key(&guid));
    })
}

pub fn run_session_move_node() -> TestResult {
    MINI_TEST!("Move Node", {
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let g1 = session.add_group("g1");
        let g2 = session.add_group("g2");
        session.add_point(Point::new(0.0, 0.0, 0.0), Some(&g1));
        let x = session.add_point(Point::new(1.0, 0.0, 0.0), Some(&g1));
        let y = session.add_point(Point::new(2.0, 0.0, 0.0), Some(&x));
        let count = session.tree.nodes().len();

        session.begin("move");
        session.add(&x, Some(&g2));
        session.commit();
        let queued = g1.borrow().is_queued();
        let moved = g1.borrow().children().len() == 1
            && g2
                .borrow()
                .children()
                .last()
                .is_some_and(|n| Rc::ptr_eq(n, &x))
            && Rc::ptr_eq(&y.borrow().parent().unwrap(), &x);
        let walked = session.tree.nodes().len();
        session.undo();
        let back = g1
            .borrow()
            .children()
            .get(1)
            .is_some_and(|n| Rc::ptr_eq(n, &x))
            && g2.borrow().children().is_empty();
        let text = session.tree.str();
        session.redo();

        MINI_CHECK!(moved);
        MINI_CHECK!(queued);
        MINI_CHECK!(g2.borrow().is_queued());
        MINI_CHECK!(walked == count);
        MINI_CHECK!(back);
        MINI_CHECK!(session.tree.nodes().len() == count);
        MINI_CHECK!(!text.contains("TreeNode(, "));
        MINI_CHECK!(g2.borrow().children().len() == 1);
        MINI_CHECK!(g1.borrow().children().len() == 1);
        MINI_CHECK!(Rc::ptr_eq(&x.borrow().parent().unwrap(), &g2));
    })
}

pub fn run_session_deleted_parent_orphans_children() -> TestResult {
    MINI_TEST!("Deleted Parent Orphans Children", {
        use crate::Element;
        use crate::Point;
        use crate::Polyline;
        use crate::Session;
        use crate::TreeNode;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let element = Element::new("E");
        let e_guid = element.guid().to_string();
        let e_node = session.add_element(element, None);
        let attributes = TreeNode::new("attributes");
        session.add(&attributes, Some(&e_node));
        let q = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
        let q_guid = q.guid().to_string();
        let q_node = session.add_polyline(q, Some(&attributes)).unwrap();
        session.set_xform(&e_guid, Xform::translation(1.0, 0.0, 0.0));
        session.set_xform(&q_guid, Xform::translation(0.0, 1.0, 0.0));
        let composed = session.world_xform(&q_guid);

        session.begin("remove");
        session.remove_object(&e_guid);
        session.commit();
        let names: Vec<String> = session
            .tree
            .nodes()
            .iter()
            .map(|n| n.borrow().name.clone())
            .collect();

        MINI_CHECK!(session.lookup.contains_key(&q_guid));
        MINI_CHECK!(session
            .get_node(&q_guid)
            .is_some_and(|n| Rc::ptr_eq(&n, &q_node)));
        MINI_CHECK!(q_node
            .borrow()
            .parent()
            .is_some_and(|n| Rc::ptr_eq(&n, &attributes)));
        MINI_CHECK!(attributes
            .borrow()
            .parent()
            .is_some_and(|n| Rc::ptr_eq(&n, &e_node)));
        MINI_CHECK!(e_node.borrow().parent().is_none());
        MINI_CHECK!(session.world_xform(&q_guid) == session.xform(&q_guid));
        MINI_CHECK!(session.world_xforms().contains_key(&q_guid));
        MINI_CHECK!(names == vec!["my_session".to_string()]);

        session.undo();

        MINI_CHECK!(session.world_xform(&q_guid) == composed);
        MINI_CHECK!(session.tree.nodes().len() == 4);
    })
}

pub fn run_session_copy_drops_dead() -> TestResult {
    MINI_TEST!("Copy Drops Dead", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let g = session.add_group("g");
        let a = Point::new(1.0, 0.0, 0.0);
        let b = Point::new(2.0, 0.0, 0.0);
        let b_guid = b.guid().to_string();
        session.add_point(a, Some(&g));
        session.add_point(b, Some(&g));

        session.begin("remove");
        session.remove_object(&b_guid);
        session.commit();
        let copy = session.clone();

        MINI_CHECK!(copy.objects.points.number_of_slots() == copy.objects.points.len());
        MINI_CHECK!(copy.objects.points.number_of_dead() == 0);
        MINI_CHECK!(copy.objects.points.len() == 1);
        MINI_CHECK!(copy.history.depth() == 0);
        MINI_CHECK!(copy.order() == session.order());
        MINI_CHECK!(copy.tree.str() == session.tree.str());
        MINI_CHECK!(copy.graph.number_of_vertices() == 1);
        MINI_CHECK!(session.undo());
        MINI_CHECK!(session.objects.points.len() == 2);
        MINI_CHECK!(copy.objects.points.len() == 1);
    })
}

pub fn run_session_live_views() -> TestResult {
    MINI_TEST!("Live Views", {
        use crate::Geometry;
        use crate::InstanceRef;
        use crate::Point;
        use crate::Session;
        use crate::Vector;
        use crate::Xform;
        use std::rc::Rc;

        let mut session = Session::default();
        let g = session.add_group("gone_group");
        let kept = session.add_group("kept");
        let definition = session.add_definition(Geometry::Mesh(Rc::new(create_box(
            &Point::new(0.0, 0.0, 0.0),
            2.0,
        ))));
        let point = Point::new(0.0, 0.0, 0.0);
        let point_guid = point.guid().to_string();
        let mesh = create_box(&Point::new(5.0, 0.0, 0.0), 2.0);
        let mesh_guid = mesh.guid().to_string();
        let instance = InstanceRef::new(&definition, Xform::identity());
        let instance_guid = instance.guid().to_string();
        session.add_point(point, Some(&g));
        session.add_mesh(mesh, Some(&g));
        session.add_instance(instance, Xform::translation(10.0, 0.0, 0.0), Some(&g));
        session.add_point(Point::new(20.0, 0.0, 0.0), Some(&kept));
        session.set_xform(&point_guid, Xform::translation(0.0, 1.0, 0.0));
        session.set_xform(&mesh_guid, Xform::translation(0.0, 1.0, 0.0));
        session.set_xform("gone_group", Xform::translation(0.0, 0.0, 1.0));

        session.begin("remove");
        session.remove_object(&point_guid);
        session.remove_object(&mesh_guid);
        session.remove_object(&instance_guid);
        session.remove_group(&g);
        session.commit();
        let gone = [
            point_guid.as_str(),
            mesh_guid.as_str(),
            instance_guid.as_str(),
            "gone_group",
        ];
        let world = session.world_xforms();
        let groups: Vec<String> = session
            .tree
            .root()
            .unwrap()
            .borrow()
            .children()
            .iter()
            .map(|n| n.borrow().name.clone())
            .collect();
        let geometry = session.get_geometry();
        let collisions = session.get_collisions();
        let hits = session.ray_cast(
            &Point::new(5.0, 0.0, -10.0),
            &Vector::new(0.0, 0.0, 1.0),
            0.01,
        );
        let json = session.jsondump().unwrap();
        let derived = serde_json::to_string(&session).unwrap();
        let text = format!("{}{}", session.str(), session.repr());

        MINI_CHECK!(session
            .order()
            .iter()
            .all(|guid| !gone.contains(&guid.as_str())));
        MINI_CHECK!(gone.iter().all(|name| !world.contains_key(*name)));
        MINI_CHECK!(session.select_by_type::<Point>().len() == 1);
        MINI_CHECK!(groups == vec!["kept".to_string()]);
        MINI_CHECK!(geometry.points.len() == 1 && geometry.meshes.is_empty());
        MINI_CHECK!(session.instances_of(&definition).is_empty());
        MINI_CHECK!(collisions.is_empty());
        MINI_CHECK!(hits.is_empty());
        MINI_CHECK!(gone.iter().all(|name| !json.contains(*name)));
        MINI_CHECK!(gone.iter().all(|name| !derived.contains(*name)));
        MINI_CHECK!(gone.iter().all(|name| !text.contains(*name)));
        MINI_CHECK!(session.undo());
        MINI_CHECK!(session.order().len() == 3);
    })
}

pub fn run_session_unrecorded_remove() -> TestResult {
    MINI_TEST!("Unrecorded Remove", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let a = Point::new(1.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        session.add_point(a, None);
        session.add_point(Point::new(2.0, 0.0, 0.0), None);
        let removed = session.remove_object(&a_guid);

        MINI_CHECK!(removed);
        MINI_CHECK!(!session.lookup.contains_key(&a_guid));
        MINI_CHECK!(session.order().len() == 1);
        MINI_CHECK!(session.tree.nodes().len() == 2);
        MINI_CHECK!(session.objects.points.number_of_dead() == 1);
        MINI_CHECK!(session.objects.points.get_tomb(0).is_none());
        MINI_CHECK!(session.history.dropped == 1);
        MINI_CHECK!(!session.undo());
    })
}

REGISTER_MINI_TEST!(
    "Session",
    "Constructor",
    crate::session_test::run_session_constructor
);
REGISTER_MINI_TEST!("Session", "Copy", crate::session_test::run_session_copy);
REGISTER_MINI_TEST!(
    "Session",
    "Add Point",
    crate::session_test::run_session_add_point
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Line",
    crate::session_test::run_session_add_line
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Plane",
    crate::session_test::run_session_add_plane
);
REGISTER_MINI_TEST!(
    "Session",
    "Add OBB",
    crate::session_test::run_session_add_obb
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Polyline",
    crate::session_test::run_session_add_polyline
);
REGISTER_MINI_TEST!(
    "Session",
    "Select By Type",
    crate::session_test::run_session_select_by_type
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Pointcloud",
    crate::session_test::run_session_add_pointcloud
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Mesh",
    crate::session_test::run_session_add_mesh
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Nurbscurve",
    crate::session_test::run_session_add_nurbscurve
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Nurbssurface",
    crate::session_test::run_session_add_nurbssurface
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Brep",
    crate::session_test::run_session_add_brep
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Element",
    crate::session_test::run_session_add_element
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Empty Geometry",
    crate::session_test::run_session_add_empty_geometry
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Group",
    crate::session_test::run_session_add_group
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Edge",
    crate::session_test::run_session_add_edge
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Hierarchy",
    crate::session_test::run_session_add_hierarchy
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Children",
    crate::session_test::run_session_get_children
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Relationship",
    crate::session_test::run_session_add_relationship
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Interaction",
    crate::session_test::run_session_add_interaction
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Interaction",
    crate::session_test::run_session_get_interaction
);
REGISTER_MINI_TEST!(
    "Session",
    "Has Interaction",
    crate::session_test::run_session_has_interaction
);
REGISTER_MINI_TEST!(
    "Session",
    "Remove Interaction",
    crate::session_test::run_session_remove_interaction
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Remove Interaction",
    crate::session_test::run_session_undo_remove_interaction
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Neighbours",
    crate::session_test::run_session_get_neighbours
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Collisions",
    crate::session_test::run_session_get_collisions
);
REGISTER_MINI_TEST!(
    "Session",
    "Ray Cast",
    crate::session_test::run_session_ray_cast
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Object",
    crate::session_test::run_session_get_object
);
REGISTER_MINI_TEST!(
    "Session",
    "Remove Object",
    crate::session_test::run_session_remove_object
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Geometry",
    crate::session_test::run_session_get_geometry
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Geometry Is Pure",
    crate::session_test::run_session_get_geometry_is_pure
);
REGISTER_MINI_TEST!(
    "Session",
    "Json Roundtrip",
    crate::session_test::run_session_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Session",
    "Protobuf Roundtrip",
    crate::session_test::run_session_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Session",
    "Lookup Mutation Roundtrip",
    crate::session_test::run_session_lookup_mutation_roundtrip
);
REGISTER_MINI_TEST!("Session", "Order", crate::session_test::run_session_order);
REGISTER_MINI_TEST!(
    "Session",
    "Set Xform",
    crate::session_test::run_session_set_xform
);
REGISTER_MINI_TEST!(
    "Session",
    "World Xform Hierarchy",
    crate::session_test::run_session_world_xform_hierarchy
);
REGISTER_MINI_TEST!(
    "Session",
    "Xform Roundtrip",
    crate::session_test::run_session_xform_roundtrip
);
REGISTER_MINI_TEST!(
    "Session",
    "Tree Transformation Hierarchy",
    crate::session_test::run_session_tree_transformation_hierarchy
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Component",
    crate::session_test::run_session_add_component
);
REGISTER_MINI_TEST!(
    "Session",
    "Component Json Roundtrip",
    crate::session_test::run_session_component_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Session",
    "Document Workflow",
    crate::session_test::run_session_document_workflow
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Remove",
    crate::session_test::run_session_undo_remove
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Add",
    crate::session_test::run_session_undo_add
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Replace",
    crate::session_test::run_session_undo_replace
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Xform",
    crate::session_test::run_session_undo_xform
);
REGISTER_MINI_TEST!(
    "Session",
    "History Purged On Save",
    crate::session_test::run_session_history_purged_on_save
);
REGISTER_MINI_TEST!(
    "Session",
    "History Capacity",
    crate::session_test::run_session_history_capacity
);
REGISTER_MINI_TEST!(
    "Session",
    "Str Hierarchy",
    crate::session_test::run_session_str_hierarchy
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Definition",
    crate::session_test::run_session_add_definition
);
REGISTER_MINI_TEST!(
    "Session",
    "Add Instance",
    crate::session_test::run_session_add_instance
);
REGISTER_MINI_TEST!(
    "Session",
    "Definition Of",
    crate::session_test::run_session_definition_of
);
REGISTER_MINI_TEST!(
    "Session",
    "Instances Of",
    crate::session_test::run_session_instances_of
);
REGISTER_MINI_TEST!(
    "Session",
    "World Geometry",
    crate::session_test::run_session_world_geometry
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Geometry Resolves Instances",
    crate::session_test::run_session_get_geometry_resolves_instances
);
REGISTER_MINI_TEST!(
    "Session",
    "Replace Definition",
    crate::session_test::run_session_replace_definition
);
REGISTER_MINI_TEST!(
    "Session",
    "Remove Definition",
    crate::session_test::run_session_remove_definition
);
REGISTER_MINI_TEST!(
    "Session",
    "To Instance",
    crate::session_test::run_session_to_instance
);
REGISTER_MINI_TEST!(
    "Session",
    "Explode",
    crate::session_test::run_session_explode
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Instance",
    crate::session_test::run_session_undo_instance
);
REGISTER_MINI_TEST!(
    "Session",
    "Instance Json Roundtrip",
    crate::session_test::run_session_instance_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Session",
    "Instance Protobuf Roundtrip",
    crate::session_test::run_session_instance_protobuf_roundtrip
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Collisions Instances",
    crate::session_test::run_session_get_collisions_instances
);
REGISTER_MINI_TEST!(
    "Session",
    "Ray Cast Instance",
    crate::session_test::run_session_ray_cast_instance
);
REGISTER_MINI_TEST!(
    "Session",
    "Get Node",
    crate::session_test::run_session_get_node
);
REGISTER_MINI_TEST!(
    "Session",
    "Remove Keeps Slot",
    crate::session_test::run_session_remove_keeps_slot
);
REGISTER_MINI_TEST!(
    "Session",
    "Redo Add Keeps Node",
    crate::session_test::run_session_redo_add_keeps_node
);
REGISTER_MINI_TEST!(
    "Session",
    "Replace Shares Geometry",
    crate::session_test::run_session_replace_shares_geometry
);
REGISTER_MINI_TEST!(
    "Session",
    "Replace Across Types",
    crate::session_test::run_session_replace_across_types
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Restores Graph",
    crate::session_test::run_session_undo_restores_graph
);
REGISTER_MINI_TEST!(
    "Session",
    "Undo Restores Interactions",
    crate::session_test::run_session_undo_restores_interactions
);
REGISTER_MINI_TEST!(
    "Session",
    "Dead Guid Reused",
    crate::session_test::run_session_dead_guid_reused
);
REGISTER_MINI_TEST!(
    "Session",
    "Remove Keeps Lookup Edit",
    crate::session_test::run_session_remove_keeps_lookup_edit
);
REGISTER_MINI_TEST!(
    "Session",
    "Tree Ops",
    crate::session_test::run_session_tree_ops
);
REGISTER_MINI_TEST!(
    "Session",
    "Move Node",
    crate::session_test::run_session_move_node
);
REGISTER_MINI_TEST!(
    "Session",
    "Deleted Parent Orphans Children",
    crate::session_test::run_session_deleted_parent_orphans_children
);
REGISTER_MINI_TEST!(
    "Session",
    "Copy Drops Dead",
    crate::session_test::run_session_copy_drops_dead
);
REGISTER_MINI_TEST!(
    "Session",
    "Live Views",
    crate::session_test::run_session_live_views
);
REGISTER_MINI_TEST!(
    "Session",
    "Unrecorded Remove",
    crate::session_test::run_session_unrecorded_remove
);
