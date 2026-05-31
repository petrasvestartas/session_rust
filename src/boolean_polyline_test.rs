use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

const PI2: f64 = 6.283185307179586476;

#[test]
fn test_boolean_polyline_overlapping_squares() {
    let _ = run_boolean_polyline_overlapping_squares();
}

pub fn run_boolean_polyline_overlapping_squares() -> TestResult {
    MINI_TEST!("Overlapping Squares", {
        use crate::{Point, Polyline};
        let a = Polyline::new(vec![Point::new(-1.0,-1.0,0.0), Point::new(1.0,-1.0,0.0), Point::new(1.0,1.0,0.0), Point::new(-1.0,1.0,0.0), Point::new(-1.0,-1.0,0.0)]);
        let b = Polyline::new(vec![Point::new(0.0,0.0,0.0), Point::new(2.0,0.0,0.0), Point::new(2.0,2.0,0.0), Point::new(0.0,2.0,0.0), Point::new(0.0,0.0,0.0)]);
        let isect = Polyline::boolean_op(&a, &b, 0);
        let uni = Polyline::boolean_op(&a, &b, 1);
        let diff = Polyline::boolean_op(&a, &b, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Overlapping Squares", crate::boolean_polyline_test::run_boolean_polyline_overlapping_squares);

#[test]
fn test_boolean_polyline_circle_vs_rectangle() {
    let _ = run_boolean_polyline_circle_vs_rectangle();
}

pub fn run_boolean_polyline_circle_vs_rectangle() -> TestResult {
    MINI_TEST!("Circle Vs Rectangle", {
        use crate::{Point, Polyline};
        let mut pts: Vec<Point> = (0..64).map(|i| { let a = PI2 * i as f64 / 64.0; Point::new(5.0 + 1.5 * a.cos(), 1.5 * a.sin(), 0.0) }).collect();
        pts.push(pts[0].clone());
        let circle = Polyline::new(pts);
        let rect = Polyline::new(vec![Point::new(4.0,-0.5,0.0), Point::new(7.0,-0.5,0.0), Point::new(7.0,0.5,0.0), Point::new(4.0,0.5,0.0), Point::new(4.0,-0.5,0.0)]);
        let isect = Polyline::boolean_op(&circle, &rect, 0);
        let uni = Polyline::boolean_op(&circle, &rect, 1);
        let diff = Polyline::boolean_op(&circle, &rect, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Circle Vs Rectangle", crate::boolean_polyline_test::run_boolean_polyline_circle_vs_rectangle);

#[test]
fn test_boolean_polyline_star_vs_circle() {
    let _ = run_boolean_polyline_star_vs_circle();
}

pub fn run_boolean_polyline_star_vs_circle() -> TestResult {
    MINI_TEST!("Star Vs Circle", {
        use crate::{Point, Polyline};
        let mut star_pts: Vec<Point> = (0..10).map(|i| { let a = PI2 * i as f64 / 10.0; let r = if i % 2 == 0 { 2.0 } else { 0.8 }; Point::new(10.0 + r * a.cos(), r * a.sin(), 0.0) }).collect();
        star_pts.push(star_pts[0].clone());
        let star = Polyline::new(star_pts);
        let mut circ_pts: Vec<Point> = (0..32).map(|i| { let a = PI2 * i as f64 / 32.0; Point::new(10.5 + 1.2 * a.cos(), 0.5 + 1.2 * a.sin(), 0.0) }).collect();
        circ_pts.push(circ_pts[0].clone());
        let circle = Polyline::new(circ_pts);
        let isect = Polyline::boolean_op(&star, &circle, 0);
        let uni = Polyline::boolean_op(&star, &circle, 1);
        let diff = Polyline::boolean_op(&star, &circle, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Star Vs Circle", crate::boolean_polyline_test::run_boolean_polyline_star_vs_circle);

#[test]
fn test_boolean_polyline_l_shape_vs_rectangle() {
    let _ = run_boolean_polyline_l_shape_vs_rectangle();
}

pub fn run_boolean_polyline_l_shape_vs_rectangle() -> TestResult {
    MINI_TEST!("L Shape Vs Rectangle", {
        use crate::{Point, Polyline};
        let l_shape = Polyline::new(vec![Point::new(15.0,-1.0,0.0), Point::new(18.0,-1.0,0.0), Point::new(18.0,0.0,0.0), Point::new(16.0,0.0,0.0), Point::new(16.0,2.0,0.0), Point::new(15.0,2.0,0.0), Point::new(15.0,-1.0,0.0)]);
        let rect = Polyline::new(vec![Point::new(15.5,-0.5,0.0), Point::new(18.5,-0.5,0.0), Point::new(18.5,1.5,0.0), Point::new(15.5,1.5,0.0), Point::new(15.5,-0.5,0.0)]);
        let isect = Polyline::boolean_op(&l_shape, &rect, 0);
        let uni = Polyline::boolean_op(&l_shape, &rect, 1);
        let diff = Polyline::boolean_op(&l_shape, &rect, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "L Shape Vs Rectangle", crate::boolean_polyline_test::run_boolean_polyline_l_shape_vs_rectangle);

#[test]
fn test_boolean_polyline_two_large_circles() {
    let _ = run_boolean_polyline_two_large_circles();
}

pub fn run_boolean_polyline_two_large_circles() -> TestResult {
    MINI_TEST!("Two Large Circles", {
        use crate::{Point, Polyline};
        let mut pts_a: Vec<Point> = (0..256).map(|i| { let a = PI2 * i as f64 / 256.0; Point::new(22.0 + 2.0 * a.cos(), 2.0 * a.sin(), 0.0) }).collect();
        pts_a.push(pts_a[0].clone());
        let mut pts_b: Vec<Point> = (0..256).map(|i| { let a = PI2 * i as f64 / 256.0; Point::new(23.0 + 2.0 * a.cos(), 0.5 + 2.0 * a.sin(), 0.0) }).collect();
        pts_b.push(pts_b[0].clone());
        let ca = Polyline::new(pts_a);
        let cb = Polyline::new(pts_b);
        let isect = Polyline::boolean_op(&ca, &cb, 0);
        let uni = Polyline::boolean_op(&ca, &cb, 1);
        let diff = Polyline::boolean_op(&ca, &cb, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Two Large Circles", crate::boolean_polyline_test::run_boolean_polyline_two_large_circles);

#[test]
fn test_boolean_polyline_diamond_vs_triangle() {
    let _ = run_boolean_polyline_diamond_vs_triangle();
}

pub fn run_boolean_polyline_diamond_vs_triangle() -> TestResult {
    MINI_TEST!("Diamond Vs Triangle", {
        use crate::{Point, Polyline};
        let diamond = Polyline::new(vec![Point::new(28.0,0.0,0.0), Point::new(30.0,-2.0,0.0), Point::new(32.0,0.0,0.0), Point::new(30.0,2.0,0.0), Point::new(28.0,0.0,0.0)]);
        let tri = Polyline::new(vec![Point::new(29.0,-2.0,0.0), Point::new(33.0,0.0,0.0), Point::new(29.0,2.0,0.0), Point::new(29.0,-2.0,0.0)]);
        let isect = Polyline::boolean_op(&diamond, &tri, 0);
        let uni = Polyline::boolean_op(&diamond, &tri, 1);
        let diff = Polyline::boolean_op(&diamond, &tri, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Diamond Vs Triangle", crate::boolean_polyline_test::run_boolean_polyline_diamond_vs_triangle);

#[test]
fn test_boolean_polyline_star_vs_star() {
    let _ = run_boolean_polyline_star_vs_star();
}

pub fn run_boolean_polyline_star_vs_star() -> TestResult {
    MINI_TEST!("Star Vs Star", {
        use crate::{Point, Polyline};
        let mut pts_a: Vec<Point> = (0..12).map(|i| { let a = PI2 * i as f64 / 12.0; let r = if i % 2 == 0 { 2.5 } else { 1.0 }; Point::new(36.0 + r * a.cos(), r * a.sin(), 0.0) }).collect();
        pts_a.push(pts_a[0].clone());
        let mut pts_b: Vec<Point> = (0..10).map(|i| { let a = PI2 * i as f64 / 10.0; let r = if i % 2 == 0 { 2.0 } else { 0.8 }; Point::new(37.0 + r * a.cos(), 0.5 + r * a.sin(), 0.0) }).collect();
        pts_b.push(pts_b[0].clone());
        let sa = Polyline::new(pts_a);
        let sb = Polyline::new(pts_b);
        let isect = Polyline::boolean_op(&sa, &sb, 0);
        let uni = Polyline::boolean_op(&sa, &sb, 1);
        let diff = Polyline::boolean_op(&sa, &sb, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Star Vs Star", crate::boolean_polyline_test::run_boolean_polyline_star_vs_star);

#[test]
fn test_boolean_polyline_cross_shape() {
    let _ = run_boolean_polyline_cross_shape();
}

pub fn run_boolean_polyline_cross_shape() -> TestResult {
    MINI_TEST!("Cross Shape", {
        use crate::{Point, Polyline};
        let narrow = Polyline::new(vec![Point::new(42.0,-2.0,0.0), Point::new(44.0,-2.0,0.0), Point::new(44.0,2.0,0.0), Point::new(42.0,2.0,0.0), Point::new(42.0,-2.0,0.0)]);
        let wide = Polyline::new(vec![Point::new(40.0,-0.5,0.0), Point::new(46.0,-0.5,0.0), Point::new(46.0,0.5,0.0), Point::new(40.0,0.5,0.0), Point::new(40.0,-0.5,0.0)]);
        let isect = Polyline::boolean_op(&narrow, &wide, 0);
        let uni = Polyline::boolean_op(&narrow, &wide, 1);
        let diff = Polyline::boolean_op(&narrow, &wide, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Cross Shape", crate::boolean_polyline_test::run_boolean_polyline_cross_shape);

#[test]
fn test_boolean_polyline_concave_arrow_vs_circle() {
    let _ = run_boolean_polyline_concave_arrow_vs_circle();
}

pub fn run_boolean_polyline_concave_arrow_vs_circle() -> TestResult {
    MINI_TEST!("Concave Arrow Vs Circle", {
        use crate::{Point, Polyline};
        let arrow = Polyline::new(vec![Point::new(49.0,0.0,0.0), Point::new(52.0,2.0,0.0), Point::new(51.0,0.5,0.0), Point::new(53.0,0.5,0.0), Point::new(53.0,-0.5,0.0), Point::new(51.0,-0.5,0.0), Point::new(52.0,-2.0,0.0), Point::new(49.0,0.0,0.0)]);
        let mut pts: Vec<Point> = (0..48).map(|i| { let a = PI2 * i as f64 / 48.0; Point::new(51.5 + 1.5 * a.cos(), 1.5 * a.sin(), 0.0) }).collect();
        pts.push(pts[0].clone());
        let circle = Polyline::new(pts);
        let isect = Polyline::boolean_op(&arrow, &circle, 0);
        let uni = Polyline::boolean_op(&arrow, &circle, 1);
        let diff = Polyline::boolean_op(&arrow, &circle, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Concave Arrow Vs Circle", crate::boolean_polyline_test::run_boolean_polyline_concave_arrow_vs_circle);

#[test]
fn test_boolean_polyline_two_large_circles_1000() {
    let _ = run_boolean_polyline_two_large_circles_1000();
}

pub fn run_boolean_polyline_two_large_circles_1000() -> TestResult {
    MINI_TEST!("Two Large Circles 1000", {
        use crate::{Point, Polyline};
        let mut pts_a: Vec<Point> = (0..1000).map(|i| { let a = PI2 * i as f64 / 1000.0; Point::new(58.0 + 3.0 * a.cos(), 3.0 * a.sin(), 0.0) }).collect();
        pts_a.push(pts_a[0].clone());
        let mut pts_b: Vec<Point> = (0..1000).map(|i| { let a = PI2 * i as f64 / 1000.0; Point::new(59.5 + 3.0 * a.cos(), 3.0 * a.sin(), 0.0) }).collect();
        pts_b.push(pts_b[0].clone());
        let ca = Polyline::new(pts_a);
        let cb = Polyline::new(pts_b);
        let isect = Polyline::boolean_op(&ca, &cb, 0);
        let uni = Polyline::boolean_op(&ca, &cb, 1);
        let diff = Polyline::boolean_op(&ca, &cb, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Two Large Circles 1000", crate::boolean_polyline_test::run_boolean_polyline_two_large_circles_1000);

#[test]
fn test_boolean_polyline_large_coords_auto_scale() {
    let _ = run_boolean_polyline_large_coords_auto_scale();
}

pub fn run_boolean_polyline_large_coords_auto_scale() -> TestResult {
    MINI_TEST!("Large Coords Auto Scale", {
        use crate::{Point, Polyline};
        // Coordinates near 1e7 would overflow int64 cross products with the old
        // fixed 1e9 scale: (1e7*1e9)^2 = 1e32 >> int64::max (~9.2e18)
        let a = Polyline::new(vec![Point::new(64e6,1e6,0.0), Point::new(64e6+2e6,1e6,0.0), Point::new(64e6+2e6,1e6+2e6,0.0), Point::new(64e6,1e6+2e6,0.0), Point::new(64e6,1e6,0.0)]);
        let b = Polyline::new(vec![Point::new(64e6+1e6,1e6+1e6,0.0), Point::new(64e6+3e6,1e6+1e6,0.0), Point::new(64e6+3e6,1e6+3e6,0.0), Point::new(64e6+1e6,1e6+3e6,0.0), Point::new(64e6+1e6,1e6+1e6,0.0)]);
        let isect = Polyline::boolean_op(&a, &b, 0);
        let uni = Polyline::boolean_op(&a, &b, 1);
        let diff = Polyline::boolean_op(&a, &b, 2);
        MINI_CHECK!(isect.len() >= 1);
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(uni.len() >= 1);
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(diff.len() >= 1);
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}
REGISTER_MINI_TEST!("Boolean Polyline", "Large Coords Auto Scale", crate::boolean_polyline_test::run_boolean_polyline_large_coords_auto_scale);
