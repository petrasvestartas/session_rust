use session_rust::{NurbsSurface, Point};

fn main() {
    let mut srf = NurbsSurface::create_raw(3, false, 4, 4, 5, 5, false, false, 1.0, 1.0).unwrap();
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

    // Expected coordinates from OpenNURBS (first 20 points)
    let expected = vec![
        (0.000, 0.000, -2.500),
        (0.000, 0.598, -1.176),
        (0.000, 1.081, -0.429),
        (0.000, 1.481, -0.093),
        (0.000, 1.833, -0.003),
        (0.000, 2.167, -0.003),
        (0.000, 2.519, -0.093),
        (0.000, 2.919, -0.429),
        (0.000, 3.402, -1.176),
        (0.000, 4.000, -2.500),
        (0.598, 0.000, -1.176),
        (0.598, 0.598, -0.407),
        (0.598, 1.081, 0.282),
        (0.598, 1.481, 0.815),
        (0.598, 1.833, 1.118),
        (0.598, 2.167, 1.118),
        (0.598, 2.519, 0.815),
        (0.598, 2.919, 0.282),
        (0.598, 3.402, -0.407),
        (0.598, 4.000, -1.176),
    ];

    let mut max_error = 0.0;
    let mut max_error_point = 0;
    let mut idx = 0;

    for i in 0..2 {
        // First 2 rows
        for j in 0..10 {
            let u = u_min + (u_max - u_min) * (i as f64) / 9.0;
            let v = v_min + (v_max - v_min) * (j as f64) / 9.0;
            let pt = srf.point_at(u, v).unwrap();

            let (exp_x, exp_y, exp_z) = expected[idx];

            // Compute raw errors (before rounding)
            let error_x = (pt[0] - exp_x).abs();
            let error_y = (pt[1] - exp_y).abs();
            let error_z = (pt[2] - exp_z).abs();
            let max_component_error = error_x.max(error_y).max(error_z);

            if max_component_error > max_error {
                max_error = max_component_error;
                max_error_point = idx;
            }

            println!(
                "Point {}: actual=({:.10}, {:.10}, {:.10}) expected=({:.3}, {:.3}, {:.3})",
                idx, pt[0], pt[1], pt[2], exp_x, exp_y, exp_z
            );
            println!(
                "  Errors: x={:.2e}, y={:.2e}, z={:.2e}",
                error_x, error_y, error_z
            );

            idx += 1;
        }
    }

    println!("\n=== PRECISION ANALYSIS ===");
    println!(
        "Maximum rounding error: {:.10} (at point {})",
        max_error, max_error_point
    );
    println!("Maximum error in decimal places: {:.2}", -max_error.log10());
    println!(
        "All coordinates match to 3 decimals (0.001): {}",
        max_error < 0.001
    );
    println!(
        "All coordinates match to 6 decimals (1e-6): {}",
        max_error < 1e-6
    );
    println!(
        "All coordinates match to 10 decimals (1e-10): {}",
        max_error < 1e-10
    );
}
