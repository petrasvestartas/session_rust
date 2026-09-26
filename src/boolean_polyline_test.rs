use crate::mini_test::TestResult;
use crate::tolerance::PI;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

const PI2: f64 = 2.0 * PI;

pub fn run_boolean_polyline_overlapping_squares() -> TestResult {
    MINI_TEST!("Overlapping Squares", {
        use crate::BooleanPolyline;
        use crate::Point;
        use crate::Polyline;

        let a = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new(1.0, -1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(-1.0, 1.0, 0.0),
            Point::new(-1.0, -1.0, 0.0),
        ]);
        let b = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let isect = Polyline::boolean_op(&a, &b, 0, None);
        let uni = Polyline::boolean_op(&a, &b, 1, None);
        let diff = Polyline::boolean_op(&a, &b, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);

        let far_a = Polyline::new(vec![
            Point::new(10.0, -1.0, 0.0),
            Point::new(12.0, -1.0, 0.0),
            Point::new(12.0, 1.0, 0.0),
            Point::new(10.0, 1.0, 0.0),
            Point::new(10.0, -1.0, 0.0),
        ]);
        let far_b = Polyline::new(vec![
            Point::new(14.0, -1.0, 0.0),
            Point::new(16.0, -1.0, 0.0),
            Point::new(16.0, 1.0, 0.0),
            Point::new(14.0, 1.0, 0.0),
            Point::new(14.0, -1.0, 0.0),
        ]);

        MINI_CHECK!(BooleanPolyline::compute_count(&far_a, &far_b, 0) == 0);
        MINI_CHECK!(
            BooleanPolyline::compute_count(&far_a, &far_b, 1)
                == (far_a.point_count() + far_b.point_count()) as i32
        );
        MINI_CHECK!(
            BooleanPolyline::compute_count(&far_a, &far_b, 2) == far_a.point_count() as i32
        );
    })
}

pub fn run_boolean_polyline_circle_vs_rectangle() -> TestResult {
    MINI_TEST!("Circle Vs Rectangle", {
        use crate::Point;
        use crate::Polyline;

        let mut pts: Vec<Point> = Vec::new();

        for i in 0..64 {
            let a = PI2 * i as f64 / 64.0;
            pts.push(Point::new(5.0 + 1.5 * a.cos(), 1.5 * a.sin(), 0.0));
        }

        pts.push(pts[0].clone());
        let circle = Polyline::new(pts);
        let rect = Polyline::new(vec![
            Point::new(4.0, -0.5, 0.0),
            Point::new(7.0, -0.5, 0.0),
            Point::new(7.0, 0.5, 0.0),
            Point::new(4.0, 0.5, 0.0),
            Point::new(4.0, -0.5, 0.0),
        ]);
        let isect = Polyline::boolean_op(&circle, &rect, 0, None);
        let uni = Polyline::boolean_op(&circle, &rect, 1, None);
        let diff = Polyline::boolean_op(&circle, &rect, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_star_vs_circle() -> TestResult {
    MINI_TEST!("Star Vs Circle", {
        use crate::Point;
        use crate::Polyline;

        let mut star_pts: Vec<Point> = Vec::new();

        for i in 0..10 {
            let a = PI2 * i as f64 / 10.0;
            let r = if i % 2 == 0 { 2.0 } else { 0.8 };
            star_pts.push(Point::new(10.0 + r * a.cos(), r * a.sin(), 0.0));
        }

        star_pts.push(star_pts[0].clone());
        let star = Polyline::new(star_pts);
        let mut circ_pts: Vec<Point> = Vec::new();

        for i in 0..32 {
            let a = PI2 * i as f64 / 32.0;
            circ_pts.push(Point::new(10.5 + 1.2 * a.cos(), 0.5 + 1.2 * a.sin(), 0.0));
        }

        circ_pts.push(circ_pts[0].clone());
        let circle = Polyline::new(circ_pts);
        let isect = Polyline::boolean_op(&star, &circle, 0, None);
        let uni = Polyline::boolean_op(&star, &circle, 1, None);
        let diff = Polyline::boolean_op(&star, &circle, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_l_shape_vs_rectangle() -> TestResult {
    MINI_TEST!("L Shape Vs Rectangle", {
        use crate::Point;
        use crate::Polyline;

        let l_shape = Polyline::new(vec![
            Point::new(15.0, -1.0, 0.0),
            Point::new(18.0, -1.0, 0.0),
            Point::new(18.0, 0.0, 0.0),
            Point::new(16.0, 0.0, 0.0),
            Point::new(16.0, 2.0, 0.0),
            Point::new(15.0, 2.0, 0.0),
            Point::new(15.0, -1.0, 0.0),
        ]);
        let rect = Polyline::new(vec![
            Point::new(15.5, -0.5, 0.0),
            Point::new(18.5, -0.5, 0.0),
            Point::new(18.5, 1.5, 0.0),
            Point::new(15.5, 1.5, 0.0),
            Point::new(15.5, -0.5, 0.0),
        ]);
        let isect = Polyline::boolean_op(&l_shape, &rect, 0, None);
        let uni = Polyline::boolean_op(&l_shape, &rect, 1, None);
        let diff = Polyline::boolean_op(&l_shape, &rect, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_two_large_circles() -> TestResult {
    MINI_TEST!("Two Large Circles", {
        use crate::Point;
        use crate::Polyline;

        let mut pts_a: Vec<Point> = Vec::new();
        let mut pts_b: Vec<Point> = Vec::new();

        for i in 0..256 {
            let a = PI2 * i as f64 / 256.0;
            pts_a.push(Point::new(22.0 + 2.0 * a.cos(), 2.0 * a.sin(), 0.0));
            pts_b.push(Point::new(23.0 + 2.0 * a.cos(), 0.5 + 2.0 * a.sin(), 0.0));
        }

        pts_a.push(pts_a[0].clone());
        pts_b.push(pts_b[0].clone());
        let ca = Polyline::new(pts_a);
        let cb = Polyline::new(pts_b);
        let isect = Polyline::boolean_op(&ca, &cb, 0, None);
        let uni = Polyline::boolean_op(&ca, &cb, 1, None);
        let diff = Polyline::boolean_op(&ca, &cb, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_diamond_vs_triangle() -> TestResult {
    MINI_TEST!("Diamond Vs Triangle", {
        use crate::Point;
        use crate::Polyline;

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
        let isect = Polyline::boolean_op(&diamond, &tri, 0, None);
        let uni = Polyline::boolean_op(&diamond, &tri, 1, None);
        let diff = Polyline::boolean_op(&diamond, &tri, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_star_vs_star() -> TestResult {
    MINI_TEST!("Star Vs Star", {
        use crate::Point;
        use crate::Polyline;

        let mut pts_a: Vec<Point> = Vec::new();

        for i in 0..12 {
            let a = PI2 * i as f64 / 12.0;
            let r = if i % 2 == 0 { 2.5 } else { 1.0 };
            pts_a.push(Point::new(36.0 + r * a.cos(), r * a.sin(), 0.0));
        }

        pts_a.push(pts_a[0].clone());
        let mut pts_b: Vec<Point> = Vec::new();

        for i in 0..10 {
            let a = PI2 * i as f64 / 10.0;
            let r = if i % 2 == 0 { 2.0 } else { 0.8 };
            pts_b.push(Point::new(37.0 + r * a.cos(), 0.5 + r * a.sin(), 0.0));
        }

        pts_b.push(pts_b[0].clone());
        let sa = Polyline::new(pts_a);
        let sb = Polyline::new(pts_b);
        let isect = Polyline::boolean_op(&sa, &sb, 0, None);
        let uni = Polyline::boolean_op(&sa, &sb, 1, None);
        let diff = Polyline::boolean_op(&sa, &sb, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_cross_shape() -> TestResult {
    MINI_TEST!("Cross Shape", {
        use crate::Point;
        use crate::Polyline;

        let narrow = Polyline::new(vec![
            Point::new(42.0, -2.0, 0.0),
            Point::new(44.0, -2.0, 0.0),
            Point::new(44.0, 2.0, 0.0),
            Point::new(42.0, 2.0, 0.0),
            Point::new(42.0, -2.0, 0.0),
        ]);
        let wide = Polyline::new(vec![
            Point::new(40.0, -0.5, 0.0),
            Point::new(46.0, -0.5, 0.0),
            Point::new(46.0, 0.5, 0.0),
            Point::new(40.0, 0.5, 0.0),
            Point::new(40.0, -0.5, 0.0),
        ]);
        let isect = Polyline::boolean_op(&narrow, &wide, 0, None);
        let uni = Polyline::boolean_op(&narrow, &wide, 1, None);
        let diff = Polyline::boolean_op(&narrow, &wide, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_concave_arrow_vs_circle() -> TestResult {
    MINI_TEST!("Concave Arrow Vs Circle", {
        use crate::Point;
        use crate::Polyline;

        let arrow = Polyline::new(vec![
            Point::new(49.0, 0.0, 0.0),
            Point::new(52.0, 2.0, 0.0),
            Point::new(51.0, 0.5, 0.0),
            Point::new(53.0, 0.5, 0.0),
            Point::new(53.0, -0.5, 0.0),
            Point::new(51.0, -0.5, 0.0),
            Point::new(52.0, -2.0, 0.0),
            Point::new(49.0, 0.0, 0.0),
        ]);
        let mut pts: Vec<Point> = Vec::new();

        for i in 0..48 {
            let a = PI2 * i as f64 / 48.0;
            pts.push(Point::new(51.5 + 1.5 * a.cos(), 1.5 * a.sin(), 0.0));
        }

        pts.push(pts[0].clone());
        let circle = Polyline::new(pts);
        let isect = Polyline::boolean_op(&arrow, &circle, 0, None);
        let uni = Polyline::boolean_op(&arrow, &circle, 1, None);
        let diff = Polyline::boolean_op(&arrow, &circle, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_two_large_circles_1000() -> TestResult {
    MINI_TEST!("Two Large Circles 1000", {
        use crate::Point;
        use crate::Polyline;

        let mut pts_a: Vec<Point> = Vec::new();
        let mut pts_b: Vec<Point> = Vec::new();

        for i in 0..1000 {
            let a = PI2 * i as f64 / 1000.0;
            pts_a.push(Point::new(58.0 + 3.0 * a.cos(), 3.0 * a.sin(), 0.0));
            pts_b.push(Point::new(59.5 + 3.0 * a.cos(), 3.0 * a.sin(), 0.0));
        }

        pts_a.push(pts_a[0].clone());
        pts_b.push(pts_b[0].clone());
        let ca = Polyline::new(pts_a);
        let cb = Polyline::new(pts_b);
        let isect = Polyline::boolean_op(&ca, &cb, 0, None);
        let uni = Polyline::boolean_op(&ca, &cb, 1, None);
        let diff = Polyline::boolean_op(&ca, &cb, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_large_coords_auto_scale() -> TestResult {
    MINI_TEST!("Large Coords Auto Scale", {
        use crate::Point;
        use crate::Polyline;

        let a = Polyline::new(vec![
            Point::new(64e6, 1e6, 0.0),
            Point::new(64e6 + 2e6, 1e6, 0.0),
            Point::new(64e6 + 2e6, 1e6 + 2e6, 0.0),
            Point::new(64e6, 1e6 + 2e6, 0.0),
            Point::new(64e6, 1e6, 0.0),
        ]);
        let b = Polyline::new(vec![
            Point::new(64e6 + 1e6, 1e6 + 1e6, 0.0),
            Point::new(64e6 + 3e6, 1e6 + 1e6, 0.0),
            Point::new(64e6 + 3e6, 1e6 + 3e6, 0.0),
            Point::new(64e6 + 1e6, 1e6 + 3e6, 0.0),
            Point::new(64e6 + 1e6, 1e6 + 1e6, 0.0),
        ]);
        let isect = Polyline::boolean_op(&a, &b, 0, None);
        let uni = Polyline::boolean_op(&a, &b, 1, None);
        let diff = Polyline::boolean_op(&a, &b, 2, None);

        MINI_CHECK!(!isect.is_empty());
        MINI_CHECK!(isect[0].point_count() > 0);
        MINI_CHECK!(!uni.is_empty());
        MINI_CHECK!(uni[0].point_count() > 0);
        MINI_CHECK!(!diff.is_empty());
        MINI_CHECK!(diff[0].point_count() > 0);
    })
}

pub fn run_boolean_polyline_regions() -> TestResult {
    MINI_TEST!("Regions", {
        use crate::BooleanPolyline;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let plane = Plane::xy_plane();
        let outer = Polyline::rectangle(
            &Point::new(0.0, 0.0, 0.0),
            &Vector::new(1.0, 0.0, 0.0),
            &Vector::new(0.0, 1.0, 0.0),
            10.0,
            10.0,
            true,
        );
        let inner = Polyline::rectangle(
            &Point::new(3.0, 3.0, 0.0),
            &Vector::new(1.0, 0.0, 0.0),
            &Vector::new(0.0, 1.0, 0.0),
            4.0,
            4.0,
            true,
        );
        let left = Polyline::rectangle(
            &Point::new(0.0, -1.0, 0.0),
            &Vector::new(1.0, 0.0, 0.0),
            &Vector::new(0.0, 1.0, 0.0),
            5.0,
            12.0,
            true,
        );
        let apart = Polyline::rectangle(
            &Point::new(20.0, 0.0, 0.0),
            &Vector::new(1.0, 0.0, 0.0),
            &Vector::new(0.0, 1.0, 0.0),
            10.0,
            10.0,
            true,
        );
        let frame = BooleanPolyline::compute_regions(std::slice::from_ref(&outer), &[inner], 2);
        let half = BooleanPolyline::compute_regions(&frame, &[left], 0);
        let both = BooleanPolyline::compute_regions(&[outer], &[apart], 1);
        let mut clockwise = 0;

        for ring in &frame {
            clockwise += if ring.is_clockwise(&plane) { 1 } else { 0 };
        }

        MINI_CHECK!(frame.len() == 2);
        MINI_CHECK!(frame[0].is_closed());
        MINI_CHECK!(clockwise == 1);
        MINI_CHECK!(half.len() == 1);
        MINI_CHECK!(half[0].point_count() == 9);
        MINI_CHECK!(!half[0].is_clockwise(&plane));
        MINI_CHECK!(both.len() == 2);
    })
}

pub fn run_boolean_polyline_regions_orientation() -> TestResult {
    MINI_TEST!("Regions Orientation", {
        use crate::BooleanPolyline;
        use crate::Plane;
        use crate::Point;
        use crate::Polyline;
        use crate::Vector;

        let plane = Plane::xy_plane();
        let outer = Polyline::rectangle(
            &Point::new(0.0, 0.0, 0.0),
            &Vector::new(1.0, 0.0, 0.0),
            &Vector::new(0.0, 1.0, 0.0),
            10.0,
            10.0,
            true,
        );
        let inner = Polyline::rectangle(
            &Point::new(3.0, 3.0, 0.0),
            &Vector::new(1.0, 0.0, 0.0),
            &Vector::new(0.0, 1.0, 0.0),
            4.0,
            4.0,
            true,
        );
        let frame = BooleanPolyline::compute_regions(&[outer.clone(), inner.clone()], &[], 1);
        let turned = BooleanPolyline::compute_regions(&[outer.reversed(), inner], &[], 1);

        MINI_CHECK!(frame.len() == 2);
        MINI_CHECK!(turned.len() == 2);

        for ring in &frame {
            MINI_CHECK!(
                ring.is_clockwise(&plane)
                    == (ring.get_point(0).unwrap()[0] > 1.0 && ring.get_point(0).unwrap()[0] < 9.0)
            );
        }

        for ring in &turned {
            MINI_CHECK!(
                ring.is_clockwise(&plane)
                    == (ring.get_point(0).unwrap()[0] > 1.0 && ring.get_point(0).unwrap()[0] < 9.0)
            );
        }
    })
}

pub fn run_boolean_polyline_open_horizontal_line_vs_unit_square() -> TestResult {
    MINI_TEST!("Horizontal Line Vs Unit Square", {
        use crate::BooleanPolyline;
        use crate::Point;
        use crate::Polyline;

        let open_line = Polyline::new(vec![Point::new(-2.0, 0.0, 0.0), Point::new(2.0, 0.0, 0.0)]);
        let sq = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new(1.0, -1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(-1.0, 1.0, 0.0),
            Point::new(-1.0, -1.0, 0.0),
        ]);
        let out = BooleanPolyline::clip_open_against_closed(&open_line, &sq);

        MINI_CHECK!(out.len() == 1);
        MINI_CHECK!(out[0].point_count() == 2);

        let p0 = out[0].get_point(0).unwrap();
        let p1 = out[0].get_point(1).unwrap();

        MINI_CHECK!((p0[0].abs() - 1.0).abs() < 1e-6);
        MINI_CHECK!((p1[0].abs() - 1.0).abs() < 1e-6);
        MINI_CHECK!(p0[1].abs() < 1e-6);
        MINI_CHECK!(p1[1].abs() < 1e-6);
    })
}

pub fn run_boolean_polyline_open_diagonal_line_vs_unit_square() -> TestResult {
    MINI_TEST!("Diagonal Line Vs Unit Square", {
        use crate::BooleanPolyline;
        use crate::Point;
        use crate::Polyline;

        let open_line = Polyline::new(vec![Point::new(-2.0, -2.0, 0.0), Point::new(2.0, 2.0, 0.0)]);
        let sq = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new(1.0, -1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(-1.0, 1.0, 0.0),
            Point::new(-1.0, -1.0, 0.0),
        ]);
        let out = BooleanPolyline::clip_open_against_closed(&open_line, &sq);

        MINI_CHECK!(out.len() == 1);
        MINI_CHECK!(out[0].point_count() == 2);

        let p0 = out[0].get_point(0).unwrap();
        let p1 = out[0].get_point(1).unwrap();

        MINI_CHECK!((p0[0].abs() - 1.0).abs() < 1e-6);
        MINI_CHECK!((p1[0].abs() - 1.0).abs() < 1e-6);
    })
}

pub fn run_boolean_polyline_open_interior_open_path_passes_through() -> TestResult {
    MINI_TEST!("Interior Open Path Passes Through", {
        use crate::BooleanPolyline;
        use crate::Point;
        use crate::Polyline;

        let open_path = Polyline::new(vec![
            Point::new(-2.0, 0.0, 0.0),
            Point::new(0.0, 0.2, 0.0),
            Point::new(2.0, 0.0, 0.0),
        ]);
        let sq = Polyline::new(vec![
            Point::new(-1.0, -1.0, 0.0),
            Point::new(1.0, -1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(-1.0, 1.0, 0.0),
            Point::new(-1.0, -1.0, 0.0),
        ]);
        let out = BooleanPolyline::clip_open_against_closed(&open_path, &sq);

        MINI_CHECK!(out.len() == 1);
        MINI_CHECK!(out[0].point_count() >= 3);
    })
}

REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Overlapping Squares",
    crate::boolean_polyline_test::run_boolean_polyline_overlapping_squares
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Circle Vs Rectangle",
    crate::boolean_polyline_test::run_boolean_polyline_circle_vs_rectangle
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Star Vs Circle",
    crate::boolean_polyline_test::run_boolean_polyline_star_vs_circle
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "L Shape Vs Rectangle",
    crate::boolean_polyline_test::run_boolean_polyline_l_shape_vs_rectangle
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Two Large Circles",
    crate::boolean_polyline_test::run_boolean_polyline_two_large_circles
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Diamond Vs Triangle",
    crate::boolean_polyline_test::run_boolean_polyline_diamond_vs_triangle
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Star Vs Star",
    crate::boolean_polyline_test::run_boolean_polyline_star_vs_star
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Cross Shape",
    crate::boolean_polyline_test::run_boolean_polyline_cross_shape
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Concave Arrow Vs Circle",
    crate::boolean_polyline_test::run_boolean_polyline_concave_arrow_vs_circle
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Two Large Circles 1000",
    crate::boolean_polyline_test::run_boolean_polyline_two_large_circles_1000
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Large Coords Auto Scale",
    crate::boolean_polyline_test::run_boolean_polyline_large_coords_auto_scale
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Regions",
    crate::boolean_polyline_test::run_boolean_polyline_regions
);
REGISTER_MINI_TEST!(
    "Boolean Polyline",
    "Regions Orientation",
    crate::boolean_polyline_test::run_boolean_polyline_regions_orientation
);
REGISTER_MINI_TEST!(
    "Boolean Polyline Open",
    "Horizontal Line Vs Unit Square",
    crate::boolean_polyline_test::run_boolean_polyline_open_horizontal_line_vs_unit_square
);
REGISTER_MINI_TEST!(
    "Boolean Polyline Open",
    "Diagonal Line Vs Unit Square",
    crate::boolean_polyline_test::run_boolean_polyline_open_diagonal_line_vs_unit_square
);
REGISTER_MINI_TEST!(
    "Boolean Polyline Open",
    "Interior Open Path Passes Through",
    crate::boolean_polyline_test::run_boolean_polyline_open_interior_open_path_passes_through
);
