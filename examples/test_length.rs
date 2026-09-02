use session_rust::{NurbsCurve, Point};

fn main() {
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
    println!("Rust:   {:.10}", curve.length(Some(1e-6)));
}
