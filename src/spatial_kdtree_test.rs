use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_kdtree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Point;
        use crate::SpatialKDTree;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(10.0, 0.0, 0.0),
        ];
        let tree = SpatialKDTree::new(pts);
        let (idx, dist) = tree.nearest(&Point::new(2.0, 0.0, 0.0));

        MINI_CHECK!(idx == 1);
        MINI_CHECK!(TOLERANCE.is_close(dist, 1.0));
    })
}

pub fn run_kdtree_nearest() -> TestResult {
    MINI_TEST!("Nearest", {
        use crate::Point;
        use crate::SpatialKDTree;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];
        let tree = SpatialKDTree::new(pts);
        let query = Point::new(1.1, 0.0, 0.0);
        let (idx, dist) = tree.nearest(&query);

        MINI_CHECK!(idx == 1);
        MINI_CHECK!(TOLERANCE.is_close(dist, 0.1));
    })
}

pub fn run_kdtree_nearest_k() -> TestResult {
    MINI_TEST!("Nearest K", {
        use crate::Point;
        use crate::SpatialKDTree;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];
        let tree = SpatialKDTree::new(pts);
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
        use crate::Point;
        use crate::SpatialKDTree;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(5.0, 0.0, 0.0),
        ];
        let tree = SpatialKDTree::new(pts);
        let query = Point::new(0.5, 0.0, 0.0);
        let result = tree.radius_search(&query, 1.1);

        MINI_CHECK!(result.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(result[0].1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(result[1].1, 0.5));
    })
}

REGISTER_MINI_TEST!(
    "SpatialKDTree",
    "Constructor",
    crate::spatial_kdtree_test::run_kdtree_constructor
);
REGISTER_MINI_TEST!(
    "SpatialKDTree",
    "Nearest",
    crate::spatial_kdtree_test::run_kdtree_nearest
);
REGISTER_MINI_TEST!(
    "SpatialKDTree",
    "Nearest K",
    crate::spatial_kdtree_test::run_kdtree_nearest_k
);
REGISTER_MINI_TEST!(
    "SpatialKDTree",
    "Radius Search",
    crate::spatial_kdtree_test::run_kdtree_radius_search
);
