use crate::mini_test::TestResult;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_collection_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Collection;
        use crate::Point;
        use std::rc::Rc;

        let a = Rc::new(Point::new(0.0, 0.0, 0.0));
        let b = Rc::new(Point::new(1.0, 0.0, 0.0));
        let c = Rc::new(Point::new(2.0, 0.0, 0.0));
        let mut points: Collection<Rc<Point>> = Collection::new();
        points.push(Rc::clone(&a));
        points.push(Rc::clone(&b));
        points.push(Rc::clone(&c));
        let iterated: Vec<Rc<Point>> = points.iter().cloned().collect();

        MINI_CHECK!(points.len() == 3);
        MINI_CHECK!(Rc::ptr_eq(&points[0], &a) && Rc::ptr_eq(&points[1], &b));
        MINI_CHECK!(Rc::ptr_eq(&points[2], &c));
        MINI_CHECK!(Rc::ptr_eq(&iterated[0], &a) && Rc::ptr_eq(&iterated[2], &c));
        MINI_CHECK!(points.get_slot(b.guid()) == Some(1));
        MINI_CHECK!(points.number_of_slots() == 3);
        MINI_CHECK!(points.number_of_dead() == 0);
        MINI_CHECK!(points.str() == "Collection(3 live, 0 dead)");
    })
}

pub fn run_collection_set_dead() -> TestResult {
    MINI_TEST!("Set Dead", {
        use crate::Collection;
        use crate::Point;
        use std::rc::Rc;

        let a = Rc::new(Point::new(0.0, 0.0, 0.0));
        let b = Rc::new(Point::new(1.0, 0.0, 0.0));
        let c = Rc::new(Point::new(2.0, 0.0, 0.0));
        let mut points: Collection<Rc<Point>> =
            Collection::from(vec![Rc::clone(&a), Rc::clone(&b), Rc::clone(&c)]);
        points.set_dead(1, true);
        let killed: Vec<Rc<Point>> = points.to_vec();
        let killed_len = points.len();
        let killed_slot = points.get_slot(b.guid());
        let killed_dead = points.number_of_dead();
        let killed_slots = points.number_of_slots();
        let kept = Rc::ptr_eq(points.get_item(1), &b) && points.is_dead(1);
        let second = Rc::ptr_eq(&points[1], &c);
        points.set_dead(1, false);

        MINI_CHECK!(killed_len == 2 && second);
        MINI_CHECK!(Rc::ptr_eq(&killed[0], &a) && Rc::ptr_eq(&killed[1], &c));
        MINI_CHECK!(killed_slot.is_none() && kept);
        MINI_CHECK!(killed_dead == 1 && killed_slots == 3);
        MINI_CHECK!(points.len() == 3);
        MINI_CHECK!(Rc::ptr_eq(&points[1], &b) && Rc::ptr_eq(&points[2], &c));
        MINI_CHECK!(points.get_slot(b.guid()) == Some(1));
    })
}

pub fn run_collection_index_skips_dead() -> TestResult {
    MINI_TEST!("Index Skips Dead", {
        use crate::Collection;
        use crate::Point;
        use std::rc::Rc;

        let mut e: Vec<Rc<Point>> = Vec::new();

        for i in 0..6 {
            e.push(Rc::new(Point::new(i as f64, 0.0, 0.0)));
        }

        let mut points: Collection<Rc<Point>> = Collection::from(e[0..5].to_vec());
        points.set_dead(0, true);
        points.set_dead(3, true);
        let missing = points.get(3).is_none();
        points.push(Rc::clone(&e[5]));

        MINI_CHECK!(Rc::ptr_eq(&points[0], &e[1]) && Rc::ptr_eq(&points[1], &e[2]));
        MINI_CHECK!(Rc::ptr_eq(&points[2], &e[4]));
        MINI_CHECK!(Rc::ptr_eq(points.first().unwrap(), &e[1]));
        MINI_CHECK!(missing);
        MINI_CHECK!(Rc::ptr_eq(&points[3], &e[5]));
        MINI_CHECK!(Rc::ptr_eq(points.last().unwrap(), &e[5]));
    })
}

pub fn run_collection_compact() -> TestResult {
    MINI_TEST!("Compact", {
        use crate::history::Tomb;
        use crate::Collection;
        use crate::Point;
        use std::rc::Rc;

        let mut e: Vec<Rc<Point>> = Vec::new();

        for i in 0..5 {
            e.push(Rc::new(Point::new(i as f64, 0.0, 0.0)));
        }

        let mut points: Collection<Rc<Point>> = Collection::from(e.clone());
        let tomb = Tomb::new("points", false, 3, None);
        points.set_dead(1, true);
        points.set_dead(3, true);
        points.set_tomb(3, &tomb);
        points.compact();
        let pinned_slots = points.number_of_slots();
        let pinned_dead = points.number_of_dead();
        let order = points.to_vec();
        let slots = [
            points.get_slot(e[0].guid()),
            points.get_slot(e[2].guid()),
            points.get_slot(e[4].guid()),
        ];
        let moved = tomb.slot.get();
        drop(tomb);
        points.compact();

        MINI_CHECK!(pinned_slots == 4 && pinned_dead == 1 && moved == 2);
        MINI_CHECK!(Rc::ptr_eq(&order[0], &e[0]) && Rc::ptr_eq(&order[1], &e[2]));
        MINI_CHECK!(Rc::ptr_eq(&order[2], &e[4]));
        MINI_CHECK!(slots == [Some(0), Some(1), Some(3)]);
        MINI_CHECK!(points.number_of_slots() == 3 && points.number_of_dead() == 0);
        MINI_CHECK!(points.get_slot(e[4].guid()) == Some(2));
    })
}

pub fn run_collection_compact_step() -> TestResult {
    MINI_TEST!("Compact Step", {
        use crate::history::Tomb;
        use crate::Collection;
        use crate::Point;
        use std::rc::Rc;

        let mut points: Collection<Rc<Point>> = Collection::new();
        let mut model: Vec<(Rc<Point>, bool)> = Vec::new();
        let mut tombs: Vec<Rc<Tomb>> = Vec::new();

        for i in 0..1000 {
            let point = Rc::new(Point::new(i as f64, 0.0, 0.0));
            points.push(Rc::clone(&point));
            model.push((point, i % 3 != 0));
        }

        for i in (0..1000).step_by(3) {
            points.set_dead(i, true);

            if tombs.len() < 10 {
                tombs.push(Tomb::new("points", false, i, None));
                points.set_tomb(i, &tombs[tombs.len() - 1]);
            }
        }

        let mut bounded = true;
        let mut exact = true;
        let mut revived = 0;

        loop {
            bounded &= points.compact_step(10) <= 10;
            let mut expected: Vec<&Rc<Point>> = Vec::new();

            for m in &model {
                if m.1 {
                    expected.push(&m.0);
                }
            }

            exact &= points.len() == expected.len();
            exact &= points.iter().zip(&expected).all(|(p, q)| Rc::ptr_eq(p, q));

            for p in &points {
                exact &= points
                    .get_slot(p.guid())
                    .is_some_and(|s| Rc::ptr_eq(points.get_item(s), p));
            }

            if !points.is_compacting() {
                break;
            }

            let point = Rc::new(Point::new(-1.0, 0.0, 0.0));
            points.push(Rc::clone(&point));
            model.push((point, true));

            if revived < 5 {
                points.set_dead(tombs[revived].slot.get(), false);
                model[revived * 3].1 = true;
                revived += 1;
            }
        }

        let settled = points.number_of_slots() == points.len() + 5;
        points.compact_step(10);
        points.set_dead(1, true);

        while points.is_compacting() {
            points.compact_step(10);
        }

        let waiting = points.is_dead(1) && points.number_of_dead() == 6;
        points.compact();

        MINI_CHECK!(bounded && exact && settled);
        MINI_CHECK!(waiting);
        MINI_CHECK!(points.number_of_dead() == 5);
        MINI_CHECK!(points.number_of_slots() == points.len() + 5);
    })
}

pub fn run_collection_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Collection;
        use crate::Point;
        use std::rc::Rc;

        let mut points: Collection<Rc<Point>> = Collection::new();
        points.push(Rc::new(Point::new(0.0, 0.0, 0.0)));
        points.push(Rc::new(Point::new(1.0, 0.0, 0.0)));
        points.push(Rc::new(Point::new(2.0, 0.0, 0.0)));
        points.set_dead(1, true);
        let json = serde_json::to_string(&points).unwrap();
        let loaded: Collection<Rc<Point>> = serde_json::from_str(&json).unwrap();

        MINI_CHECK!(json == serde_json::to_string(&points.to_vec()).unwrap());
        MINI_CHECK!(loaded.len() == 2 && loaded.number_of_dead() == 0);
        MINI_CHECK!(loaded[0].guid() == points[0].guid());
        MINI_CHECK!(loaded[1].guid() == points[1].guid());
    })
}

REGISTER_MINI_TEST!(
    "Collection",
    "Constructor",
    crate::collection_test::run_collection_constructor
);
REGISTER_MINI_TEST!(
    "Collection",
    "Set Dead",
    crate::collection_test::run_collection_set_dead
);
REGISTER_MINI_TEST!(
    "Collection",
    "Index Skips Dead",
    crate::collection_test::run_collection_index_skips_dead
);
REGISTER_MINI_TEST!(
    "Collection",
    "Compact",
    crate::collection_test::run_collection_compact
);
REGISTER_MINI_TEST!(
    "Collection",
    "Compact Step",
    crate::collection_test::run_collection_compact_step
);
REGISTER_MINI_TEST!(
    "Collection",
    "Json Roundtrip",
    crate::collection_test::run_collection_json_roundtrip
);
