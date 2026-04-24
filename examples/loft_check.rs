use session_rust::{Point, Polyline, Mesh};

fn main() {
    let bot = vec![Polyline::new(vec![
        Point::new(0.0,0.0,0.0), Point::new(3.0,0.0,0.0), Point::new(6.0,0.0,0.0),
        Point::new(10.0,0.0,0.0), Point::new(10.0,10.0,0.0), Point::new(0.0,10.0,0.0),
        Point::new(0.0,0.0,0.0),
    ])];
    let top = vec![Polyline::new(vec![
        Point::new(0.0,0.0,5.0), Point::new(3.0,0.0,5.0), Point::new(6.0,0.0,5.0),
        Point::new(10.0,0.0,5.0), Point::new(10.0,10.0,5.0), Point::new(0.0,10.0,5.0),
        Point::new(0.0,0.0,5.0),
    ])];
    let m = Mesh::loft(&bot, &top, true);
    println!("[RS COLLINEAR] V={} F={} closed={}", m.vertex.len(), m.face.len(), m.is_closed());

    let b2 = vec![
        Polyline::new(vec![Point::new(0.0,0.0,0.0),Point::new(10.0,0.0,0.0),Point::new(10.0,10.0,0.0),Point::new(0.0,10.0,0.0),Point::new(0.0,0.0,0.0)]),
        Polyline::new(vec![Point::new(2.0,2.0,0.0),Point::new(2.0,4.0,0.0),Point::new(4.0,4.0,0.0),Point::new(4.0,2.0,0.0),Point::new(2.0,2.0,0.0)]),
    ];
    let t2 = vec![
        Polyline::new(vec![Point::new(0.0,0.0,5.0),Point::new(10.0,0.0,5.0),Point::new(10.0,10.0,5.0),Point::new(0.0,10.0,5.0),Point::new(0.0,0.0,5.0)]),
        Polyline::new(vec![Point::new(2.0,2.0,5.0),Point::new(2.0,4.0,5.0),Point::new(4.0,4.0,5.0),Point::new(4.0,2.0,5.0),Point::new(2.0,2.0,5.0)]),
    ];
    let m2 = Mesh::loft(&b2, &t2, true);
    println!("[RS HOLE] V={} F={} closed={}", m2.vertex.len(), m2.face.len(), m2.is_closed());
}
