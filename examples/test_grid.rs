use session_rust::{NurbsSurface, Point};

fn main() {
    let mut srf = NurbsSurface::create_raw(3, false, 4, 4, 5, 5).unwrap();
    srf.make_clamped_uniform_nurbsknot_vector(0, 1.0);
    srf.make_clamped_uniform_nurbsknot_vector(1, 1.0);

    // Set control points
    srf.set_cv(0, 0, &Point::new(0.0, 0.0, -2.5));
    srf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
    srf.set_cv(0, 2, &Point::new(0.0, 2.0, 0.0));
    srf.set_cv(0, 3, &Point::new(0.0, 3.0, 0.0));
    srf.set_cv(0, 4, &Point::new(0.0, 4.0, -2.5));

    srf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
    srf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));
    srf.set_cv(1, 2, &Point::new(1.0, 2.0, 5.0));
    srf.set_cv(1, 3, &Point::new(1.0, 3.0, 0.0));
    srf.set_cv(1, 4, &Point::new(1.0, 4.0, 0.0));

    srf.set_cv(2, 0, &Point::new(2.0, 0.0, 0.0));
    srf.set_cv(2, 1, &Point::new(2.0, 1.0, 0.0));
    srf.set_cv(2, 2, &Point::new(2.0, 2.0, 0.0));
    srf.set_cv(2, 3, &Point::new(2.0, 3.0, 0.0));
    srf.set_cv(2, 4, &Point::new(2.0, 4.0, 0.0));

    srf.set_cv(3, 0, &Point::new(3.0, 0.0, 0.0));
    srf.set_cv(3, 1, &Point::new(3.0, 1.0, 0.0));
    srf.set_cv(3, 2, &Point::new(3.0, 2.0, 0.0));
    srf.set_cv(3, 3, &Point::new(3.0, 3.0, 0.0));
    srf.set_cv(3, 4, &Point::new(3.0, 4.0, 0.0));

    srf.set_cv(4, 0, &Point::new(4.0, 0.0, -2.5));
    srf.set_cv(4, 1, &Point::new(4.0, 1.0, 0.0));
    srf.set_cv(4, 2, &Point::new(4.0, 2.0, 0.0));
    srf.set_cv(4, 3, &Point::new(4.0, 3.0, 0.0));
    srf.set_cv(4, 4, &Point::new(4.0, 4.0, -2.5));

    let (u_min, u_max) = srf.domain(0).unwrap();
    let (v_min, v_max) = srf.domain(1).unwrap();

    println!("First 10 points (row 0):");
    for j in 0..10 {
        let u = u_min + (u_max - u_min) * 0.0 / 9.0;
        let v = v_min + (v_max - v_min) * (j as f32) / 9.0;
        let pt = srf.point_at(u, v).unwrap();
        println!("({:.3}, {:.3}, {:.3})", pt[0], pt[1], pt[2]);
    }

    println!("\nNext 10 points (row 1):");
    for j in 0..10 {
        let u = u_min + (u_max - u_min) * 1.0 / 9.0;
        let v = v_min + (v_max - v_min) * (j as f32) / 9.0;
        let pt = srf.point_at(u, v).unwrap();
        println!("({:.3}, {:.3}, {:.3})", pt[0], pt[1], pt[2]);
    }

    println!("\nRow 2 points:");
    for j in 0..10 {
        let u = u_min + (u_max - u_min) * 2.0 / 9.0;
        let v = v_min + (v_max - v_min) * (j as f32) / 9.0;
        let pt = srf.point_at(u, v).unwrap();
        println!("({:.3}, {:.3}, {:.3})", pt[0], pt[1], pt[2]);
    }
}
