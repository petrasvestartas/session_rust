use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_kdtree_nearest() -> TestResult {
    MINI_TEST!("Nearest", {
        use crate::KDTree;
        use crate::Point;

        // 5 known points on a line: 0, 1, 2, 3, 4
        // Query at 1.1 — nearest should be index 1 (point at x=1), distance 0.1
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];
        let tree = KDTree::new(pts);
        let query = Point::new(1.1, 0.0, 0.0);
        let (idx, dist) = tree.nearest(&query);

        MINI_CHECK!(idx == 1);
        MINI_CHECK!(TOLERANCE.is_close(dist, 0.1));
    })
}

pub fn run_kdtree_nearest_k() -> TestResult {
    MINI_TEST!("Nearest K", {
        use crate::KDTree;
        use crate::Point;

        // 5 points on X axis: 0, 1, 2, 3, 4
        // Query at 1.5 — 3 nearest are: x=1 (d=0.5), x=2 (d=0.5), x=3 (d=1.5)
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];
        let tree = KDTree::new(pts);
        let query = Point::new(1.5, 0.0, 0.0);
        let result = tree.nearest_k(&query, 3);

        MINI_CHECK!(result.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(result[0].1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(result[1].1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(result[2].1, 1.5));
    })
}

pub fn run_kdtree_radius_search() -> TestResult {
    MINI_TEST!("Radius Search", {
        use crate::KDTree;
        use crate::Point;

        // 4 points: 0, 1, 2, 5 on X axis
        // Query at 0.5, radius 1.1 — finds x=0 (d=0.5) and x=1 (d=0.5)
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(5.0, 0.0, 0.0),
        ];
        let tree = KDTree::new(pts);
        let query = Point::new(0.5, 0.0, 0.0);
        let result = tree.radius_search(&query, 1.1);

        MINI_CHECK!(result.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(result[0].1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(result[1].1, 0.5));
    })
}

REGISTER_MINI_TEST!("KDTree", "Nearest", crate::kdtree_test::run_kdtree_nearest);
REGISTER_MINI_TEST!("KDTree", "Nearest K", crate::kdtree_test::run_kdtree_nearest_k);
REGISTER_MINI_TEST!("KDTree", "Radius Search", crate::kdtree_test::run_kdtree_radius_search);
