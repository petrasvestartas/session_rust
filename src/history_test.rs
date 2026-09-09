use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_history_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::History;

        let history = History::new();
        let hstr = history.to_string();
        let hrepr = format!("{history}");

        MINI_CHECK!(!history.can_undo());
        MINI_CHECK!(!history.can_redo());
        MINI_CHECK!(history.depth() == 0);
        MINI_CHECK!(hstr == "History(0 undo, 0 redo)");
        MINI_CHECK!(hrepr == "History(0 undo, 0 redo)");
    })
}

pub fn run_history_begin_commit() -> TestResult {
    MINI_TEST!("Begin Commit", {
        use crate::{Point, Session};

        let mut session = Session::default();

        // An empty transaction is dropped, and nothing is recorded while none is open.
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
        use crate::{Point, Session};

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
        use crate::{Point, Session};

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
        MINI_CHECK!(session.objects.points.len() == 1);
    })
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
