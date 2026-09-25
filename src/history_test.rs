use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_history_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::History;

        let history = History::new();
        let hstr = history.str();
        let hrepr = history.repr();

        MINI_CHECK!(!history.can_undo());
        MINI_CHECK!(!history.can_redo());
        MINI_CHECK!(history.depth() == 0);
        MINI_CHECK!(hstr == "History(0 undo, 0 redo)");
        MINI_CHECK!(hrepr == "History(0 undo, 0 redo)");
    })
}

pub fn run_history_begin_commit() -> TestResult {
    MINI_TEST!("Begin Commit", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();

        session.history.begin("empty");
        session.history.commit();
        session.add_point(Point::new(0.0, 0.0, 0.0), None);

        session.history.begin("add");
        session.add_point(Point::new(1.0, 0.0, 0.0), None);
        session.history.commit();

        MINI_CHECK!(session.history.depth() == 1);
        MINI_CHECK!(session.history.can_undo());
        MINI_CHECK!(session.history.undo_stack[0].ops.len() == 1);
        MINI_CHECK!(session.history.undo_stack[0].label == "add");
        MINI_CHECK!(session.history.undo_stack[0].ops[0].kind() == "add");
    })
}

pub fn run_history_undo_redo() -> TestResult {
    MINI_TEST!("Undo Redo", {
        use crate::session::FromGeometry;
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();

        session.history.begin("add");
        session.add_point(point, None);
        session.history.commit();

        let undone = session.undo();
        let absent = !session.lookup.contains_key(&guid);
        let redone = session.redo();

        MINI_CHECK!(undone);
        MINI_CHECK!(absent);
        MINI_CHECK!(redone);
        MINI_CHECK!(session.lookup.contains_key(&guid));
        MINI_CHECK!(TOLERANCE.is_close(
            Point::from_geometry(&session.lookup[&guid]).unwrap()[2],
            3.0
        ));
        MINI_CHECK!(!session.history.can_redo());
        MINI_CHECK!(!session.redo());
    })
}

pub fn run_history_clear() -> TestResult {
    MINI_TEST!("Clear", {
        use crate::Point;
        use crate::Session;

        let mut session = Session::default();

        session.history.begin("a");
        session.add_point(Point::new(0.0, 0.0, 0.0), None);
        session.history.commit();

        session.history.begin("b");
        session.add_point(Point::new(1.0, 0.0, 0.0), None);
        session.history.commit();

        session.undo();
        session.history.clear();

        MINI_CHECK!(!session.history.can_undo());
        MINI_CHECK!(!session.history.can_redo());
        MINI_CHECK!(session.history.depth() == 0);
        MINI_CHECK!(session.history.bytes == 0);
        MINI_CHECK!(session.history.dropped == 2);
        MINI_CHECK!(session.objects.points.len() == 1);
        MINI_CHECK!(session.objects.points.number_of_dead() == 1);
    })
}

pub fn run_history_undo_definition() -> TestResult {
    MINI_TEST!("Undo Definition", {
        use crate::session::FromGeometry;
        use crate::Geometry;
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let point = Point::new(1.0, 2.0, 3.0);
        let guid = point.guid().to_string();

        session.begin("define");
        session.add_definition(Geometry::Point(Rc::new(point)));
        session.commit();

        session.begin("replace");
        session.replace_definition(&guid, Geometry::Point(Rc::new(Point::new(9.0, 9.0, 9.0))));
        session.commit();

        session.begin("remove");
        session.remove_definition(&guid);
        session.commit();

        let defined = session.history.undo_stack[0].ops[0].repr();
        let swapped = session.history.undo_stack[1].ops[0].kind().to_string();
        let dropped = session.history.undo_stack[2].ops[0].repr();
        let removed = !session.definition_lookup.contains_key(&guid);
        session.undo();
        let replaced = Point::from_geometry(&session.definition_lookup[&guid]).unwrap()[0];
        session.undo();
        let restored = Point::from_geometry(&session.definition_lookup[&guid]).unwrap()[0];
        session.undo();
        let undefined =
            session.definition_lookup.is_empty() && session.definitions.points.is_empty();
        let redone = session.redo();

        MINI_CHECK!(defined == format!("add({guid}, definitions)"));
        MINI_CHECK!(swapped == "replace");
        MINI_CHECK!(dropped == format!("remove({guid}, definitions)"));
        MINI_CHECK!(removed);
        MINI_CHECK!(TOLERANCE.is_close(replaced, 9.0));
        MINI_CHECK!(TOLERANCE.is_close(restored, 1.0));
        MINI_CHECK!(undefined);
        MINI_CHECK!(redone);
        MINI_CHECK!(session.definitions.points.len() == 1);
        MINI_CHECK!(session.definitions.points.number_of_slots() == 1);
        MINI_CHECK!(session.definitions.points[0].guid() == guid);
    })
}

pub fn run_history_budget() -> TestResult {
    MINI_TEST!("Budget", {
        use crate::Session;

        let mut session = Session::default();
        session.history.budget = 1 << 20;
        let mut guids = Vec::new();

        for _ in 0..20 {
            let mesh = grid_mesh(100);
            guids.push(mesh.guid().to_string());
            session.add_mesh(mesh, None);
        }

        for guid in &guids {
            session.begin("remove");
            session.remove_object(guid);
            session.commit();
        }

        let newest = session.history.undo_stack[session.history.depth() - 1].bytes;
        let pinned: usize = session
            .history
            .undo_stack
            .iter()
            .chain(&session.history.redo_stack)
            .map(|transaction| transaction.bytes)
            .sum();

        MINI_CHECK!(session.history.depth() < 20);
        MINI_CHECK!(session.history.bytes <= session.history.budget + newest);
        MINI_CHECK!(session.history.dropped > 0);
        MINI_CHECK!(session.history.bytes == pinned);
        MINI_CHECK!(session.undo());
        MINI_CHECK!(session.lookup.contains_key(&guids[19]));
    })
}

pub fn run_history_weight() -> TestResult {
    MINI_TEST!("Weight", {
        use crate::history::weight;
        use crate::history::RECORD;
        use crate::Item;
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let point = Item::Geometry(crate::Geometry::Point(Rc::new(Point::new(0.0, 0.0, 0.0))));
        let small = Item::Geometry(crate::Geometry::Mesh(Rc::new(grid_mesh(32))));
        let mesh = grid_mesh(100);
        let large = Item::Geometry(crate::Geometry::Mesh(Rc::new(mesh.clone())));
        let mut session = Session::default();
        let guid = mesh.guid().to_string();
        let a = Point::new(0.0, 0.0, 0.0);
        let b = Point::new(1.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();
        let b_guid = b.guid().to_string();
        session.add_mesh(mesh, None);
        session.add_point(a, None);
        session.add_point(b, None);
        session.add_edge(&a_guid, &guid, "touch");
        session.add_edge(&b_guid, &guid, "touch");

        session.begin("remove");
        session.remove_object(&guid);
        let removed = session.history.current.as_ref().unwrap().bytes;
        session.add_point(Point::new(2.0, 0.0, 0.0), None);
        let added = session.history.current.as_ref().unwrap().bytes;
        session.commit();

        MINI_CHECK!(weight(&point) < weight(&small));
        MINI_CHECK!(weight(&small) < weight(&large));
        MINI_CHECK!(removed == RECORD + weight(&large) + 128 * 2);
        MINI_CHECK!(added == removed + RECORD);
    })
}

pub fn run_history_abort() -> TestResult {
    MINI_TEST!("Abort", {
        use crate::Point;
        use crate::Session;
        use std::rc::Rc;

        let mut session = Session::default();
        let group = session.add_group("g");
        let b = Point::new(1.0, 0.0, 0.0);
        let b_guid = b.guid().to_string();
        session.add_point(Point::new(0.0, 0.0, 0.0), Some(&group));
        let b_node = session.add_point(b, Some(&group));
        session.add_point(Point::new(2.0, 0.0, 0.0), Some(&group));
        session.begin("kept");
        session.set_xform(&b_guid, crate::Xform::translation(0.0, 1.0, 0.0));
        session.commit();
        session.undo();
        let a = Point::new(5.0, 0.0, 0.0);
        let a_guid = a.guid().to_string();

        session.begin("aborted");
        session.add_point(a, Some(&group));
        session.remove_object(&b_guid);
        let aborted = session.abort();

        MINI_CHECK!(aborted);
        MINI_CHECK!(!session.lookup.contains_key(&a_guid));
        MINI_CHECK!(!session.order().contains(&a_guid));
        MINI_CHECK!(session.get_node(&a_guid).is_none());
        MINI_CHECK!(session.lookup.contains_key(&b_guid));
        MINI_CHECK!(session.objects.points.get_slot(&b_guid) == Some(1));
        MINI_CHECK!(Rc::ptr_eq(&group.borrow().children()[1], &b_node));
        MINI_CHECK!(b_node.borrow().at() == 1);
        MINI_CHECK!(session.history.depth() == 0);
        MINI_CHECK!(session.history.can_redo());
        MINI_CHECK!(session.history.redo_stack.len() == 1);
        MINI_CHECK!(!session.abort());
    })
}

/// A mesh of n by n vertices in quads, one guid of its own.
fn grid_mesh(n: usize) -> crate::Mesh {
    use crate::Point;
    let mut vertices = Vec::with_capacity(n * n);
    let mut faces = Vec::with_capacity((n - 1) * (n - 1));

    for i in 0..n {
        for j in 0..n {
            vertices.push(Point::new(i as f64, j as f64, 0.0));
        }
    }

    for i in 0..n - 1 {
        for j in 0..n - 1 {
            let at = i * n + j;
            faces.push(vec![at, at + n, at + n + 1, at + 1]);
        }
    }

    crate::Mesh::from_vertices_and_faces(vertices, faces)
}

REGISTER_MINI_TEST!(
    "History",
    "Constructor",
    crate::history_test::run_history_constructor
);
REGISTER_MINI_TEST!(
    "History",
    "Begin Commit",
    crate::history_test::run_history_begin_commit
);
REGISTER_MINI_TEST!(
    "History",
    "Undo Redo",
    crate::history_test::run_history_undo_redo
);
REGISTER_MINI_TEST!("History", "Clear", crate::history_test::run_history_clear);
REGISTER_MINI_TEST!(
    "History",
    "Undo Definition",
    crate::history_test::run_history_undo_definition
);
REGISTER_MINI_TEST!("History", "Budget", crate::history_test::run_history_budget);
REGISTER_MINI_TEST!("History", "Weight", crate::history_test::run_history_weight);
REGISTER_MINI_TEST!("History", "Abort", crate::history_test::run_history_abort);
