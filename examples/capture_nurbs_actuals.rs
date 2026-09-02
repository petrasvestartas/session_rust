//! Run actual NURBS computations under f32 and print the values that should
//! replace the f64-derived expectations in the test files. Used once to
//! rebaseline; safe to delete afterwards.

use session_rust::{NurbsCurve, Point};

fn main() {
    // Evaluation test setup
    let points = vec![
        Point::new(1.957614, 1.140253, -0.191281),
        Point::new(0.912252, 1.886721, 0.0),
        Point::new(3.089381, 2.701879, -0.696251),
        Point::new(5.015145, 1.189141, 0.35799),
        Point::new(1.854155, 0.514663, 0.347694),
        Point::new(3.309532, 1.328666, 0.0),
        Point::new(3.544072, 2.194233, 0.696217),
        Point::new(2.903513, 2.091287, 0.696217),
        Point::new(2.752484, 1.45432, 0.0),
        Point::new(2.406227, 1.288248, 0.0),
        Point::new(2.15032, 1.868606, 0.0),
    ];
    let curve = NurbsCurve::create(false, 2, &points);

    println!("=== Evaluation test (line 449-) ===");
    println!("length = {}", curve.length(None));
    let p = curve.point_at(0.5);
    println!("point_at(0.5) = ({}, {}, {})", p[0], p[1], p[2]);

    let derivatives = curve.evaluate(0.5, 2);
    for (i, d) in derivatives.iter().enumerate() {
        println!("derivatives[{}] = ({}, {}, {})", i, d[0], d[1], d[2]);
    }

    let tangent = curve.tangent_at(0.5);
    println!("tangent = ({}, {}, {})", tangent[0], tangent[1], tangent[2]);

    // Conversions test setup
    println!("\n=== Conversions test (line 401-) ===");
    let points = vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 2.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
        Point::new(3.0, 2.0, 0.0),
        Point::new(4.0, 0.0, 0.0),
    ];
    let curve = NurbsCurve::create(false, 2, &points);
    let (div_pts, _) = curve.divide_by_count(10, true);
    for (i, p) in div_pts.iter().enumerate() {
        println!("div_pts[{}] = ({}, {}, {})", i, p[0], p[1], p[2]);
    }
    let (len_pts, _) = curve.divide_by_length(0.5);
    for (i, p) in len_pts.iter().enumerate() {
        println!("len_pts[{}] = ({}, {}, {})", i, p[0], p[1], p[2]);
    }
}
