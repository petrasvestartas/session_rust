use session_rust::{Point, Polyline};

fn main() {
    let diamond = Polyline::new(vec![
        Point::new(28.0, 0.0, 0.0),
        Point::new(30.0, -2.0, 0.0),
        Point::new(32.0, 0.0, 0.0),
        Point::new(30.0, 2.0, 0.0),
        Point::new(28.0, 0.0, 0.0),
    ]);
    
    let tri = Polyline::new(vec![
        Point::new(29.0, -2.0, 0.0),
        Point::new(33.0, 0.0, 0.0),
        Point::new(29.0, 2.0, 0.0),
        Point::new(29.0, -2.0, 0.0),
    ]);
    
    // op=0: intersection
    let isect = Polyline::boolean_op(&diamond, &tri, 0);
    println!("Intersection: {} polylines", isect.len());
    for (i, p) in isect.iter().enumerate() {
        println!("  Polyline {}: {} points", i, p.point_count());
    }
    
    // op=1: union
    let uni = Polyline::boolean_op(&diamond, &tri, 1);
    println!("Union: {} polylines", uni.len());
    for (i, p) in uni.iter().enumerate() {
        println!("  Polyline {}: {} points", i, p.point_count());
    }
    
    // op=2: difference
    let diff = Polyline::boolean_op(&diamond, &tri, 2);
    println!("Difference: {} polylines", diff.len());
    for (i, p) in diff.iter().enumerate() {
        println!("  Polyline {}: {} points", i, p.point_count());
        // Print the points
        for j in 0..p.point_count() {
            let pt = p.get_point(j);
            println!("    Point {}: ({}, {}, {})", j, pt.x, pt.y, pt.z);
        }
    }
}
