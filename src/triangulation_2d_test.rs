use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::PI;


fn triangle_area_2d(pts: &[crate::Point], t: &(i32, i32, i32)) -> f64 {
    let (ax, ay) = (pts[t.0 as usize][0], pts[t.0 as usize][1]);
    let (bx, by) = (pts[t.1 as usize][0], pts[t.1 as usize][1]);
    let (cx, cy) = (pts[t.2 as usize][0], pts[t.2 as usize][1]);
    ((bx-ax)*(cy-ay) - (by-ay)*(cx-ax)).abs() * 0.5
}

fn total_area(pts: &[crate::Point], tris: &[(i32, i32, i32)]) -> f64 {
    tris.iter().map(|t| triangle_area_2d(pts, t)).sum()
}


pub fn run_triangulation2d_triangle() -> TestResult {
    MINI_TEST!("Triangle", {
        use crate::Point;
        use crate::triangulation_2d;
        let boundary = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0),
        ];
        let tris = triangulation_2d::triangulate(&boundary, None);
        MINI_CHECK!(tris.len() == 1);
        MINI_CHECK!(tris[0].0 >= 0);
        MINI_CHECK!(tris[0].1 >= 0);
        MINI_CHECK!(tris[0].2 >= 0);
    })
}

pub fn run_triangulation2d_square() -> TestResult {
    MINI_TEST!("Square", {
        use crate::Point;
        use crate::triangulation_2d;
        let boundary = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0),
        ];
        let tris = triangulation_2d::triangulate(&boundary, None);
        let pts = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ];
        let a = total_area(&pts, &tris);
        MINI_CHECK!(tris.len() == 2);
        MINI_CHECK!((a - 1.0).abs() < 1e-10);
    })
}

pub fn run_triangulation2d_convex_polygon() -> TestResult {
    MINI_TEST!("Convex Polygon", {
        use crate::Point;
        use crate::triangulation_2d;
        let r = 1.0_f64;
        let hex_pts: Vec<Point> = (0..6).map(|i| {
            let angle = i as f64 * PI / 3.0;
            Point::new(r * angle.cos(), r * angle.sin(), 0.0)
        }).chain(std::iter::once({
            let angle = 0.0_f64;
            Point::new(r * angle.cos(), r * angle.sin(), 0.0)
        })).collect();
        let tris = triangulation_2d::triangulate(&hex_pts, None);
        MINI_CHECK!(tris.len() == 4);
    })
}

pub fn run_triangulation2d_concave_polygon() -> TestResult {
    MINI_TEST!("Concave Polygon", {
        use crate::Point;
        use crate::triangulation_2d;
        let boundary = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0), Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 2.0, 0.0), Point::new(0.0, 0.0, 0.0),
        ];
        let tris = triangulation_2d::triangulate(&boundary, None);
        let pts = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 2.0, 0.0),
        ];
        let a = total_area(&pts, &tris);
        let expected = 3.0;
        MINI_CHECK!(tris.len() == 3);
        MINI_CHECK!((a - expected).abs() < 1e-10);
    })
}

pub fn run_triangulation2d_polygon_with_hole() -> TestResult {
    MINI_TEST!("Polygon With Hole", {
        use crate::Point;
        use crate::triangulation_2d;
        let boundary = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0), Point::new(0.0, 4.0, 0.0), Point::new(0.0, 0.0, 0.0),
        ];
        let hole = vec![
            Point::new(1.0, 1.0, 0.0), Point::new(3.0, 1.0, 0.0),
            Point::new(3.0, 3.0, 0.0), Point::new(1.0, 3.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let tris = triangulation_2d::triangulate(&boundary, Some(&[hole]));
        let all_pts = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0), Point::new(0.0, 4.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(3.0, 1.0, 0.0),
            Point::new(3.0, 3.0, 0.0), Point::new(1.0, 3.0, 0.0),
        ];
        let a = total_area(&all_pts, &tris);
        let expected = 4.0*4.0 - 2.0*2.0;
        MINI_CHECK!(tris.len() >= 8);
        MINI_CHECK!((a - expected).abs() < 1e-10);
    })
}

pub fn run_triangulation2d_polygon_with_multiple_holes() -> TestResult {
    MINI_TEST!("Polygon With Multiple Holes", {
        use crate::Point;
        use crate::triangulation_2d;
        let boundary = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 8.0, 0.0), Point::new(0.0, 8.0, 0.0), Point::new(0.0, 0.0, 0.0),
        ];
        let hole0 = vec![
            Point::new(1.0, 1.0, 0.0), Point::new(3.0, 1.0, 0.0),
            Point::new(3.0, 3.0, 0.0), Point::new(1.0, 3.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let hole1 = vec![
            Point::new(5.0, 4.0, 0.0), Point::new(9.0, 4.0, 0.0),
            Point::new(9.0, 7.0, 0.0), Point::new(5.0, 7.0, 0.0), Point::new(5.0, 4.0, 0.0),
        ];
        let tris = triangulation_2d::triangulate(&boundary, Some(&[hole0, hole1]));
        let all_pts = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(10.0, 0.0, 0.0),
            Point::new(10.0, 8.0, 0.0), Point::new(0.0, 8.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(3.0, 1.0, 0.0),
            Point::new(3.0, 3.0, 0.0), Point::new(1.0, 3.0, 0.0),
            Point::new(5.0, 4.0, 0.0), Point::new(9.0, 4.0, 0.0),
            Point::new(9.0, 7.0, 0.0), Point::new(5.0, 7.0, 0.0),
        ];
        let a = total_area(&all_pts, &tris);
        let expected = 10.0*8.0 - 2.0*2.0 - 4.0*3.0;
        MINI_CHECK!(tris.len() >= 10);
        MINI_CHECK!((a - expected).abs() < 1e-6);
    })
}

pub fn run_triangulation2d_winding_correction() -> TestResult {
    MINI_TEST!("Winding Correction", {
        use crate::Point;
        use crate::triangulation_2d;
        let boundary = vec![
            Point::new(0.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ];
        let tris = triangulation_2d::triangulate(&boundary, None);
        let pts = vec![
            Point::new(0.0, 1.0, 0.0), Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let a = total_area(&pts, &tris);
        MINI_CHECK!(tris.len() == 2);
        MINI_CHECK!((a - 1.0).abs() < 1e-10);
    })
}

REGISTER_MINI_TEST!("Triangulation2D", "Triangle", crate::triangulation_2d_test::run_triangulation2d_triangle);
REGISTER_MINI_TEST!("Triangulation2D", "Square", crate::triangulation_2d_test::run_triangulation2d_square);
REGISTER_MINI_TEST!("Triangulation2D", "Convex Polygon", crate::triangulation_2d_test::run_triangulation2d_convex_polygon);
REGISTER_MINI_TEST!("Triangulation2D", "Concave Polygon", crate::triangulation_2d_test::run_triangulation2d_concave_polygon);
REGISTER_MINI_TEST!("Triangulation2D", "Polygon With Hole", crate::triangulation_2d_test::run_triangulation2d_polygon_with_hole);
REGISTER_MINI_TEST!("Triangulation2D", "Polygon With Multiple Holes", crate::triangulation_2d_test::run_triangulation2d_polygon_with_multiple_holes);
REGISTER_MINI_TEST!("Triangulation2D", "Winding Correction", crate::triangulation_2d_test::run_triangulation2d_winding_correction);
