#[cfg(test)]
mod tests {
    use crate::nurbssurface::NurbsSurface;
    use crate::point::Point;

    #[test]
    fn test_nurbssurface_constructor() {
        let srf = NurbsSurface::create(3, false, 4, 4, 5, 5);
        assert!(srf.is_some());
        
        let srf = srf.unwrap();
        assert_eq!(srf.dimension(), 3);
        assert_eq!(srf.is_rational(), false);
        assert_eq!(srf.order(0), 4);
        assert_eq!(srf.order(1), 4);
        assert_eq!(srf.cv_count_dir(Some(0)), 5);
        assert_eq!(srf.cv_count_dir(Some(1)), 5);
        assert_eq!(srf.degree(0), 3);
        assert_eq!(srf.degree(1), 3);
    }

    #[test]
    fn test_nurbssurface_cv_access() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        
        // Set a CV
        let pt = Point::new(1.0, 2.0, 3.0);
        assert!(srf.set_cv(2, 2, &pt));
        
        // Get the CV back
        let retrieved = srf.get_cv(2, 2).unwrap();
        assert!((retrieved[0] - 1.0).abs() < 1e-10);
        assert!((retrieved[1] - 2.0).abs() < 1e-10);
        assert!((retrieved[2] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_nurbssurface_point_evaluation() {
        // Create a simple 5x5 surface
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        // Set control points in a grid
        for i in 0..5 {
            for j in 0..5 {
                let pt = Point::new(i as f64, j as f64, 0.0);
                srf.set_cv(i, j, &pt);
            }
        }
        
        // Evaluate at center
        let pt = srf.point_at(1.0, 1.0).unwrap();
        assert!((pt[0] - 2.0).abs() < 1e-10);
        assert!((pt[1] - 2.0).abs() < 1e-10);
        assert!((pt[2] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_nurbssurface_10x10_grid_evaluation() {
        // Create surface with 5x5 control points
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        // Set control points (corners inverted, middle lifted)
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
        
        // Expected coordinates from OpenNURBS implementation (100 points, 10x10 grid)
        // Format: (x, y, z) rounded to 3 decimals
        let expected = vec![
            // Row 0 (u=0.000)
            (0.000, 0.000, -2.500), (0.000, 0.598, -1.176), (0.000, 1.081, -0.429), (0.000, 1.481, -0.093), (0.000, 1.833, -0.003),
            (0.000, 2.167, -0.003), (0.000, 2.519, -0.093), (0.000, 2.919, -0.429), (0.000, 3.402, -1.176), (0.000, 4.000, -2.500),
            // Row 1 (u=0.598)
            (0.598, 0.000, -1.176), (0.598, 0.598, -0.407), (0.598, 1.081, 0.282), (0.598, 1.481, 0.815), (0.598, 1.833, 1.118),
            (0.598, 2.167, 1.118), (0.598, 2.519, 0.815), (0.598, 2.919, 0.282), (0.598, 3.402, -0.407), (0.598, 4.000, -1.176),
            // Row 2 (u=1.081)
            (1.081, 0.000, -0.429), (1.081, 0.598, -0.013), (1.081, 1.081, 0.550), (1.081, 1.481, 1.092), (1.081, 1.833, 1.443),
            (1.081, 2.167, 1.443), (1.081, 2.519, 1.092), (1.081, 2.919, 0.550), (1.081, 3.402, -0.013), (1.081, 4.000, -0.429),
            // Row 3 (u=1.481)
            (1.481, 0.000, -0.093), (1.481, 0.598, 0.120), (1.481, 1.081, 0.525), (1.481, 1.481, 0.957), (1.481, 1.833, 1.252),
            (1.481, 2.167, 1.252), (1.481, 2.519, 0.957), (1.481, 2.919, 0.525), (1.481, 3.402, 0.120), (1.481, 4.000, -0.093),
            // Row 4 (u=1.833)
            (1.833, 0.000, -0.003), (1.833, 0.598, 0.106), (1.833, 1.081, 0.354), (1.833, 1.481, 0.630), (1.833, 1.833, 0.821),
            (1.833, 2.167, 0.821), (1.833, 2.519, 0.630), (1.833, 2.919, 0.354), (1.833, 3.402, 0.106), (1.833, 4.000, -0.003),
            // Row 5 (u=2.167)
            (2.167, 0.000, -0.003), (2.167, 0.598, 0.054), (2.167, 1.081, 0.182), (2.167, 1.481, 0.325), (2.167, 1.833, 0.424),
            (2.167, 2.167, 0.424), (2.167, 2.519, 0.325), (2.167, 2.919, 0.182), (2.167, 3.402, 0.054), (2.167, 4.000, -0.003),
            // Row 6 (u=2.519)
            (2.519, 0.000, -0.093), (2.519, 0.598, -0.020), (2.519, 1.081, 0.061), (2.519, 1.481, 0.134), (2.519, 1.833, 0.179),
            (2.519, 2.167, 0.179), (2.519, 2.519, 0.134), (2.519, 2.919, 0.061), (2.519, 3.402, -0.020), (2.519, 4.000, -0.093),
            // Row 7 (u=2.919)
            (2.919, 0.000, -0.429), (2.919, 0.598, -0.195), (2.919, 1.081, -0.051), (2.919, 1.481, 0.025), (2.919, 1.833, 0.052),
            (2.919, 2.167, 0.052), (2.919, 2.519, 0.025), (2.919, 2.919, -0.051), (2.919, 3.402, -0.195), (2.919, 4.000, -0.429),
            // Row 8 (u=3.402)
            (3.402, 0.000, -1.176), (3.402, 0.598, -0.553), (3.402, 1.081, -0.199), (3.402, 1.481, -0.038), (3.402, 1.833, 0.005),
            (3.402, 2.167, 0.005), (3.402, 2.519, -0.038), (3.402, 2.919, -0.199), (3.402, 3.402, -0.553), (3.402, 4.000, -1.176),
            // Row 9 (u=4.000)
            (4.000, 0.000, -2.500), (4.000, 0.598, -1.176), (4.000, 1.081, -0.429), (4.000, 1.481, -0.093), (4.000, 1.833, -0.003),
            (4.000, 2.167, -0.003), (4.000, 2.519, -0.093), (4.000, 2.919, -0.429), (4.000, 3.402, -1.176), (4.000, 4.000, -2.500),
        ];
        
        // Evaluate 10x10 grid and verify all 100 points
        const GRID_SIZE: usize = 10;
        let mut idx = 0;
        
        for i in 0..GRID_SIZE {
            for j in 0..GRID_SIZE {
                let u = u_min + (u_max - u_min) * (i as f64) / (GRID_SIZE - 1) as f64;
                let v = v_min + (v_max - v_min) * (j as f64) / (GRID_SIZE - 1) as f64;
                let pt = srf.point_at(u, v).unwrap();
                
                let (exp_x, exp_y, exp_z) = expected[idx];
                
                // Round to 3 decimals for comparison
                let actual_x = (pt[0] * 1000.0).round() / 1000.0;
                let actual_y = (pt[1] * 1000.0).round() / 1000.0;
                let actual_z = (pt[2] * 1000.0).round() / 1000.0;
                
                assert!((actual_x - exp_x).abs() < 0.001, 
                    "Point {} x mismatch: {} != {}", idx, actual_x, exp_x);
                assert!((actual_y - exp_y).abs() < 0.001,
                    "Point {} y mismatch: {} != {}", idx, actual_y, exp_y);
                assert!((actual_z - exp_z).abs() < 0.001,
                    "Point {} z mismatch: {} != {}", idx, actual_z, exp_z);
                
                idx += 1;
            }
        }
        
        assert_eq!(idx, 100, "Should have verified all 100 grid points");
    }

    #[test]
    fn test_nurbssurface_validation() {
        let srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        assert!(srf.is_valid());
        
        // Empty surface is invalid
        let empty_srf = NurbsSurface::new();
        assert!(!empty_srf.is_valid());
    }

    #[test]
    fn test_nurbssurface_domain() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        let (u0, u1) = srf.domain(0).unwrap();
        let (v0, v1) = srf.domain(1).unwrap();
        
        // With 5 CVs and order 4, clamped uniform knot vector with delta=1.0
        // creates domain [0.0, 2.0]
        assert_eq!(u0, 0.0);
        assert_eq!(u1, 2.0);
        assert_eq!(v0, 0.0);
        assert_eq!(v1, 2.0);
    }

    #[test]
    fn test_nurbssurface_make_rational() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        assert_eq!(srf.is_rational(), false);
        assert!(srf.make_rational());
        assert_eq!(srf.is_rational(), true);
    }

    #[test]
    fn test_nurbssurface_reverse() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        for i in 0..5 {
            for j in 0..5 {
                srf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }
        
        assert!(srf.reverse(0));
        assert!(srf.is_valid());
    }

    #[test]
    fn test_nurbssurface_transpose() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        for i in 0..5 {
            for j in 0..5 {
                srf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }
        
        assert!(srf.transpose());
        assert!(srf.is_valid());
    }

    #[test]
    fn test_nurbssurface_geometric_queries() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        for i in 0..5 {
            for j in 0..5 {
                srf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }
        
        assert_eq!(srf.is_closed(0), false);
        assert_eq!(srf.is_periodic(0), false);
        // Planar surface (all z=0)
        assert!(srf.is_planar(0.01));
    }

    #[test]
    fn test_nurbssurface_string_representation() {
        let srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        let s = srf.to_string();
        assert!(s.contains("NurbsSurface"));
        assert!(s.contains("dim=3"));
        assert!(s.contains("order=[4,4]"));
        assert!(s.contains("cv_count=[5,5]"));
    }

    #[test]
    fn test_nurbssurface_make_non_rational() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        for i in 0..5 {
            for j in 0..5 {
                srf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.1));
            }
        }
        
        // Make rational first
        assert!(srf.make_rational());
        assert_eq!(srf.is_rational(), true);
        
        // Then make non-rational
        assert!(srf.make_non_rational());
        assert_eq!(srf.is_rational(), false);
    }

    #[test]
    fn test_nurbssurface_clamp_end() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
        // Clamp start of u direction
        assert!(srf.clamp_end(0, 0));
        
        // Clamp end of v direction
        assert!(srf.clamp_end(1, 1));
        
        // Clamp both ends of u direction
        assert!(srf.clamp_end(0, 2));
    }

    #[test]
    fn test_nurbssurface_isocurve_extraction() {
        // Create the same surface as in isocurve test
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);
        
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
        let u_param = 0.5 * (u_min + u_max);  // u = 1.0
        let v_param = 0.5 * (v_min + v_max);  // v = 1.0
        
        // Extract iso-v curve (u varies, v constant) using iso_curve method
        let iso_v_curve = srf.iso_curve(1, v_param).unwrap();
        assert_eq!(iso_v_curve.m_order, 4);
        assert_eq!(iso_v_curve.m_cv_count, 5);
        
        // Sample the extracted curve and verify coordinates
        let crv_domain = iso_v_curve.domain();
        let expected_iso_v = [
            (0.000, 2.000, 0.000),
            (0.120, 2.000, 0.288),
            (0.235, 2.000, 0.540),
        ];
        
        for (idx, (exp_x, exp_y, exp_z)) in expected_iso_v.iter().enumerate() {
            let t = crv_domain.0 + (crv_domain.1 - crv_domain.0) * (idx as f64) / 49.0;
            let pt = iso_v_curve.point_at(t);
            assert!((pt.x() - exp_x).abs() < 0.001, "iso-v curve point {} x mismatch", idx);
            assert!((pt.y() - exp_y).abs() < 0.001, "iso-v curve point {} y mismatch", idx);
            assert!((pt.z() - exp_z).abs() < 0.001, "iso-v curve point {} z mismatch", idx);
        }
        
        // Extract iso-u curve (v varies, u constant) using iso_curve method
        let iso_u_curve = srf.iso_curve(0, u_param).unwrap();
        assert_eq!(iso_u_curve.m_order, 4);
        assert_eq!(iso_u_curve.m_cv_count, 5);
        
        // Sample the extracted curve
        let crv_domain = iso_u_curve.domain();
        let expected_iso_u = [
            (2.000, 0.000, 0.000),
            (2.000, 0.120, 0.003),
            (2.000, 0.235, 0.012),
        ];
        
        for (idx, (exp_x, exp_y, exp_z)) in expected_iso_u.iter().enumerate() {
            let t = crv_domain.0 + (crv_domain.1 - crv_domain.0) * (idx as f64) / 49.0;
            let pt = iso_u_curve.point_at(t);
            assert!((pt.x() - exp_x).abs() < 0.001, "iso-u curve point {} x mismatch", idx);
            assert!((pt.y() - exp_y).abs() < 0.001, "iso-u curve point {} y mismatch", idx);
            assert!((pt.z() - exp_z).abs() < 0.001, "iso-u curve point {} z mismatch", idx);
        }
    }
}

#[cfg(test)]
mod short_surface_center_test {
    use crate::nurbssurface::NurbsSurface;
    use crate::point::Point;

    #[test]
    fn test_nurbssurface_center_point_numeric_3dec() {
        let mut srf = NurbsSurface::create(3, false, 4, 4, 5, 5).unwrap();
        srf.make_clamped_uniform_knot_vector(0, 1.0);
        srf.make_clamped_uniform_knot_vector(1, 1.0);

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
        let u_mid = 0.5 * (u_min + u_max);
        let v_mid = 0.5 * (v_min + v_max);

        let pt = srf.point_at(u_mid, v_mid).unwrap();
        let ax = (pt[0] * 1000.0).round() / 1000.0;
        let ay = (pt[1] * 1000.0).round() / 1000.0;
        let az = (pt[2] * 1000.0).round() / 1000.0;
        assert!((ax - 2.000).abs() < 0.001);
        assert!((ay - 2.000).abs() < 0.001);
        assert!((az - 0.625).abs() < 0.001);
    }
}
