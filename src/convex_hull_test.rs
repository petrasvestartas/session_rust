use crate::mini_test::TestResult;
use crate::tolerance::PI;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_convex_hull_hull_2d() -> TestResult {
    MINI_TEST!("Hull 2d", {
        use crate::ConvexHull;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.5, 0.5, 0.0),
            Point::new(0.3, 0.3, 0.0),
        ];
        let hull = ConvexHull::hull_2d(&pts);

        MINI_CHECK!(hull.len() == 4);
    })
}
REGISTER_MINI_TEST!(
    "ConvexHull",
    "Hull 2d",
    crate::convex_hull_test::run_convex_hull_hull_2d
);

pub fn run_convex_hull_hull_2d_collinear() -> TestResult {
    MINI_TEST!("Hull 2d Collinear", {
        use crate::ConvexHull;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(1.5, 1.0, 0.0),
        ];
        let hull = ConvexHull::hull_2d(&pts);

        MINI_CHECK!(hull.len() >= 3);
    })
}
REGISTER_MINI_TEST!(
    "ConvexHull",
    "Hull 2d Collinear",
    crate::convex_hull_test::run_convex_hull_hull_2d_collinear
);

pub fn run_convex_hull_hull_2d_circle() -> TestResult {
    MINI_TEST!("Hull 2d Circle", {
        use crate::ConvexHull;
        use crate::Point;
        let n = 12_usize;
        let mut pts = vec![];
        for i in 0..n {
            let angle = 2.0 * PI * i as f64 / n as f64;
            pts.push(Point::new(angle.cos(), angle.sin(), 0.0));
        }
        pts.push(Point::new(0.0, 0.0, 0.0));
        let hull = ConvexHull::hull_2d(&pts);

        MINI_CHECK!(hull.len() == n);
    })
}
REGISTER_MINI_TEST!(
    "ConvexHull",
    "Hull 2d Circle",
    crate::convex_hull_test::run_convex_hull_hull_2d_circle
);

pub fn run_convex_hull_hull_3d() -> TestResult {
    MINI_TEST!("Hull 3d", {
        use crate::ConvexHull;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 1.0),
            Point::new(0.25, 0.25, 0.25),
        ];
        let mesh = ConvexHull::hull_3d(&pts);

        MINI_CHECK!(mesh.number_of_vertices() == 4);
        MINI_CHECK!(mesh.number_of_faces() == 4);
    })
}
REGISTER_MINI_TEST!(
    "ConvexHull",
    "Hull 3d",
    crate::convex_hull_test::run_convex_hull_hull_3d
);

pub fn run_convex_hull_hull_3d_cube() -> TestResult {
    MINI_TEST!("Hull 3d Cube", {
        use crate::ConvexHull;
        use crate::Point;
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 1.0),
            Point::new(1.0, 0.0, 1.0),
            Point::new(1.0, 1.0, 1.0),
            Point::new(0.0, 1.0, 1.0),
            Point::new(0.5, 0.5, 0.5),
        ];
        let mesh = ConvexHull::hull_3d(&pts);

        MINI_CHECK!(mesh.number_of_vertices() == 8);
        MINI_CHECK!(mesh.number_of_faces() == 12);
    })
}
REGISTER_MINI_TEST!(
    "ConvexHull",
    "Hull 3d Cube",
    crate::convex_hull_test::run_convex_hull_hull_3d_cube
);
