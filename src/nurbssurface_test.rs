use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_nurbssurface_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::nurbssurface::NurbsSurface;
        use crate::color::Color;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        // Create surface with parameters (4x4 quadratic surface, order 3)
        let mut s = NurbsSurface::create_raw(3, false, 3, 3, 4, 4, false, false, 2.5, 2.5).unwrap();

        // Set hardcoded control points
        let cvs = vec![
            // i=0
            Point::new(0.0, 0.0, 0.0),
            Point::new(-1.0, 0.75, 2.0),
            Point::new(-1.0, 4.25, 2.0),
            Point::new(0.0, 5.0, 0.0),
            // i=1
            Point::new(0.75, -1.0, 2.0),
            Point::new(1.25, 1.25, 4.0),
            Point::new(1.25, 3.75, 4.0),
            Point::new(0.75, 6.0, 2.0),
            // i=2
            Point::new(4.25, -1.0, 2.0),
            Point::new(3.75, 1.25, 4.0),
            Point::new(3.75, 3.75, 4.0),
            Point::new(4.25, 6.0, 2.0),
            // i=3
            Point::new(5.0, 0.0, 0.0),
            Point::new(6.0, 0.75, 2.0),
            Point::new(6.0, 4.25, 2.0),
            Point::new(5.0, 5.0, 0.0),
        ];

        // Setters
        let mut idx = 0;
        for i in 0..s.cv_count_dir(Some(0)) {
            for j in 0..s.cv_count_dir(Some(1)) {
                s.set_cv(i, j, &cvs[idx]);
                idx += 1;
            }
        }

        // Getters
        let control_point = s.get_cv(2, 1).unwrap();  // 3.75, 1.25, 4.0
        let point = s.point_at(2.5, 2.5).unwrap();     // 2.5, 2.5, 4.0

        // String representation
        let str_repr = s.to_string();

        // Duplicate for comparison
        let s_copy = s.duplicate();

        // Subdivision test
        let (v, uv) = s.divide_by_count(5, 5);

        MINI_CHECK!(s.name == "my_nurbssurface");
        MINI_CHECK!(s.width == 1.0);
        MINI_CHECK!(s.surfacecolor == Color::black());
        MINI_CHECK!(!s.guid.is_empty());
        MINI_CHECK!(s.m_dim == 3);
        MINI_CHECK!(!s.m_is_rat);
        MINI_CHECK!(s.dimension() == 3);
        MINI_CHECK!(!s.is_rational());
        MINI_CHECK!(s.order(0) == 3);
        MINI_CHECK!(s.order(1) == 3);
        MINI_CHECK!(s.degree(0) == 2);
        MINI_CHECK!(s.degree(1) == 2);
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 4);
        MINI_CHECK!(s.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(s.cv_count_dir(None) == 16);
        MINI_CHECK!(s.knot_count(0) == 5);
        MINI_CHECK!(s.knot_count(1) == 5);
        MINI_CHECK!(control_point[0] == 3.75 && control_point[1] == 1.25 && control_point[2] == 4.0);
        MINI_CHECK!(point[0] == 2.5 && point[1] == 2.5 && point[2] == 4.0);
        MINI_CHECK!(str_repr == "NurbsSurface(dim=3, order=(3,3), cv_count=(4,4))");
        MINI_CHECK!(s_copy == s);
        MINI_CHECK!(s_copy.name == s.name);
        MINI_CHECK!(s_copy.width == s.width);
        MINI_CHECK!(s_copy.surfacecolor == s.surfacecolor);
        MINI_CHECK!(s_copy.guid != s.guid);
        // Helper closure for tolerance-based point comparison
        let close_pt = |a: &Point, x: f64, y: f64, z: f64| -> bool {
            TOLERANCE.is_close(a[0], x) && TOLERANCE.is_close(a[1], y) && TOLERANCE.is_close(a[2], z)
        };
        MINI_CHECK!(close_pt(&v[0][0], 0.0, 0.0, 0.0));
        MINI_CHECK!(close_pt(&v[0][1], -0.64, 0.76, 1.28));
        MINI_CHECK!(close_pt(&v[0][2], -0.96, 1.84, 1.92));
        MINI_CHECK!(close_pt(&v[0][3], -0.96, 3.16, 1.92));
        MINI_CHECK!(close_pt(&v[0][4], -0.64, 4.24, 1.28));
        MINI_CHECK!(close_pt(&v[0][5], 0.0, 5.0, 0.0));
        MINI_CHECK!(close_pt(&v[1][0], 0.76, -0.64, 1.28));
        MINI_CHECK!(close_pt(&v[1][1], 0.6832, 0.6832, 2.56));
        MINI_CHECK!(close_pt(&v[1][2], 0.6448, 1.9168, 3.2));
        MINI_CHECK!(close_pt(&v[1][3], 0.6448, 3.0832, 3.2));
        MINI_CHECK!(close_pt(&v[1][4], 0.6832, 4.3168, 2.56));
        MINI_CHECK!(close_pt(&v[1][5], 0.76, 5.64, 1.28));
        MINI_CHECK!(close_pt(&v[2][0], 1.84, -0.96, 1.92));
        MINI_CHECK!(close_pt(&v[2][1], 1.9168, 0.6448, 3.2));
        MINI_CHECK!(close_pt(&v[2][2], 1.9552, 1.9552, 3.84));
        MINI_CHECK!(close_pt(&v[2][3], 1.9552, 3.0448, 3.84));
        MINI_CHECK!(close_pt(&v[2][4], 1.9168, 4.3552, 3.2));
        MINI_CHECK!(close_pt(&v[2][5], 1.84, 5.96, 1.92));
        MINI_CHECK!(close_pt(&v[3][0], 3.16, -0.96, 1.92));
        MINI_CHECK!(close_pt(&v[3][1], 3.0832, 0.6448, 3.2));
        MINI_CHECK!(close_pt(&v[3][2], 3.0448, 1.9552, 3.84));
        MINI_CHECK!(close_pt(&v[3][3], 3.0448, 3.0448, 3.84));
        MINI_CHECK!(close_pt(&v[3][4], 3.0832, 4.3552, 3.2));
        MINI_CHECK!(close_pt(&v[3][5], 3.16, 5.96, 1.92));
        MINI_CHECK!(close_pt(&v[4][0], 4.24, -0.64, 1.28));
        MINI_CHECK!(close_pt(&v[4][1], 4.3168, 0.6832, 2.56));
        MINI_CHECK!(close_pt(&v[4][2], 4.3552, 1.9168, 3.2));
        MINI_CHECK!(close_pt(&v[4][3], 4.3552, 3.0832, 3.2));
        MINI_CHECK!(close_pt(&v[4][4], 4.3168, 4.3168, 2.56));
        MINI_CHECK!(close_pt(&v[4][5], 4.24, 5.64, 1.28));
        MINI_CHECK!(close_pt(&v[5][0], 5.0, 0.0, 0.0));
        MINI_CHECK!(close_pt(&v[5][1], 5.64, 0.76, 1.28));
        MINI_CHECK!(close_pt(&v[5][2], 5.96, 1.84, 1.92));
        MINI_CHECK!(close_pt(&v[5][3], 5.96, 3.16, 1.92));
        MINI_CHECK!(close_pt(&v[5][4], 5.64, 4.24, 1.28));
        MINI_CHECK!(close_pt(&v[5][5], 5.0, 5.0, 0.0));
    })
}

pub fn run_nurbssurface_create_operations() -> TestResult {
    MINI_TEST!("create_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create a simple 2x2 bilinear surface
        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        // Set corner control points
        surf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        // Check knot vectors
        let (u0, _u1) = surf.domain(0).unwrap();
        let (v0, _v1) = surf.domain(1).unwrap();

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(u0 == 0.0);
        MINI_CHECK!(v0 == 0.0);
    })
}

pub fn run_nurbssurface_accessors() -> TestResult {
    MINI_TEST!("accessors", {
        use crate::nurbssurface::NurbsSurface;

        let mut surf = NurbsSurface::create_raw(3, false, 4, 3, 5, 4, false, false, 1.0, 1.0).unwrap();

        // Test knot access
        let _knot_val = surf.knot(0, 2);

        // Test set knot
        surf.set_knot(0, 2, 5.0);
        let new_val = surf.knot(0, 2);

        MINI_CHECK!(surf.dimension() == 3);
        MINI_CHECK!(!surf.is_rational());
        MINI_CHECK!(surf.order(0) == 4);
        MINI_CHECK!(surf.order(1) == 3);
        MINI_CHECK!(surf.degree(0) == 3);
        MINI_CHECK!(surf.degree(1) == 2);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 5);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(surf.cv_count_dir(None) == 20);
        MINI_CHECK!(surf.cv_size() == 3);
        MINI_CHECK!(surf.knot_count(0) == 7);
        MINI_CHECK!(surf.knot_count(1) == 5);
        MINI_CHECK!(surf.span_count(0) == 2);
        MINI_CHECK!(surf.span_count(1) == 2);
        MINI_CHECK!(new_val.is_some() && new_val.unwrap() == 5.0);
    })
}

pub fn run_nurbssurface_knot_operations() -> TestResult {
    MINI_TEST!("knot_operations", {
        use crate::nurbssurface::NurbsSurface;

        let surf = NurbsSurface::create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0).unwrap();

        // Verify domain
        let (u0, u1) = surf.domain(0).unwrap();
        let (v0, v1) = surf.domain(1).unwrap();

        MINI_CHECK!(u0 == 0.0);
        MINI_CHECK!(u1 > u0);
        MINI_CHECK!(v0 == 0.0);
        MINI_CHECK!(v1 > v0);
        MINI_CHECK!(surf.is_clamped(0, 0));
        MINI_CHECK!(surf.is_clamped(1, 0));
    })
}

pub fn run_nurbssurface_rational_operations() -> TestResult {
    MINI_TEST!("rational_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create non-rational surface
        let mut surf = NurbsSurface::create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0).unwrap();

        // Make it rational
        surf.make_rational();

        // Set a control point and weight
        surf.set_cv(1, 1, &Point::new(1.0, 2.0, 3.0));
        surf.set_weight(1, 1, 2.0);

        // Verify weight
        let w = surf.weight(1, 1);

        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.cv_size() == 4);
        MINI_CHECK!(w == 2.0);
    })
}

pub fn run_nurbssurface_evaluation() -> TestResult {
    MINI_TEST!("evaluation", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        // Create simple bilinear surface (2x2 control points, order 2x2)
        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        // Set corner control points to unit square in XY plane
        surf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        // Evaluate at domain bounds
        let (u0, u1) = surf.domain(0).unwrap();
        let (v0, v1) = surf.domain(1).unwrap();

        let pt_corner = surf.point_at(u0, v0).unwrap();
        let pt_mid = surf.point_at((u0 + u1) / 2.0, (v0 + v1) / 2.0).unwrap();
        let derivs = surf.evaluate((u0 + u1) / 2.0, (v0 + v1) / 2.0, 1);
        let normal = surf.normal_at((u0 + u1) / 2.0, (v0 + v1) / 2.0);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(pt_corner[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_corner[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_corner[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[0], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[1], 0.5));
        MINI_CHECK!(derivs.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(normal[2].abs(), 1.0));
    })
}

pub fn run_nurbssurface_geometric_queries() -> TestResult {
    MINI_TEST!("geometric_queries", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create and setup surface
        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        surf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(!surf.is_closed(0));
        MINI_CHECK!(!surf.is_closed(1));
        MINI_CHECK!(!surf.is_periodic(0));
        MINI_CHECK!(!surf.is_periodic(1));
        MINI_CHECK!(surf.is_clamped(0, 0));
        MINI_CHECK!(surf.is_clamped(1, 0));
        MINI_CHECK!(surf.is_planar(1e-6));
    })
}

pub fn run_nurbssurface_modification() -> TestResult {
    MINI_TEST!("modification", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 3, 2, false, false, 1.0, 1.0).unwrap();

        // Set some CVs
        surf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        surf.set_cv(2, 0, &Point::new(2.0, 0.0, 0.0));
        surf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        surf.set_cv(2, 1, &Point::new(2.0, 1.0, 0.0));

        let cv_before = surf.get_cv(0, 0).unwrap();

        // Test reverse in u direction
        surf.reverse(0);
        let cv_after = surf.get_cv(2, 0).unwrap();

        // Reverse back
        surf.reverse(0);

        // Test transpose
        let order_u_before = surf.order(0);
        let order_v_before = surf.order(1);
        surf.transpose();

        MINI_CHECK!(cv_after[0] == cv_before[0]);
        MINI_CHECK!(surf.order(0) == order_v_before);
        MINI_CHECK!(surf.order(1) == order_u_before);
    })
}

pub fn run_nurbssurface_isocurve() -> TestResult {
    MINI_TEST!("isocurve", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create surface
        let mut surf = NurbsSurface::create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0).unwrap();

        // Set up a grid of control points
        for i in 0..3 {
            for j in 0..3 {
                surf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }

        // Extract iso-u curve (v varies)
        let (u0, u1) = surf.domain(0).unwrap();
        let u_mid = (u0 + u1) / 2.0;
        let iso_u = surf.iso_curve(0, u_mid);

        // Extract iso-v curve (u varies)
        let (v0, v1) = surf.domain(1).unwrap();
        let v_mid = (v0 + v1) / 2.0;
        let iso_v = surf.iso_curve(1, v_mid);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(iso_u.is_some());
        MINI_CHECK!(iso_v.is_some());
    })
}

pub fn run_nurbssurface_transformation() -> TestResult {
    MINI_TEST!("transformation", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::xform::Xform;
        use crate::tolerance::TOLERANCE;

        // Create simple surface
        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        surf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        // Apply translation
        let xf = Xform::translation(1.0, 2.0, 3.0);
        surf.transform(&xf);

        // Check transformed CV
        let pt = surf.get_cv(0, 0).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 3.0));
    })
}

pub fn run_nurbssurface_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::color::Color;

        // Create and setup surface
        let mut surf = NurbsSurface::create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0).unwrap();
        surf.name = "test_nurbssurface".to_string();
        surf.width = 2.0;
        surf.surfacecolor = Color::new(255, 128, 64, 255);

        // Set some CVs
        for i in 0..3 {
            for j in 0..3 {
                surf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }

        //   json_dumps()    | String       | to JSON string
        //   json_loads(s)   | String       | from JSON string
        //   json_dump(path) | file         | write to file
        //   json_load(path) | file         | read from file

        // Serialize to JSON
        let json_str = serde_json::to_string_pretty(&surf).expect("Failed to serialize");

        // Deserialize from JSON
        let loaded: NurbsSurface = serde_json::from_str(&json_str).expect("Failed to deserialize");

        MINI_CHECK!(!json_str.is_empty());
        MINI_CHECK!(loaded.name == surf.name);
        MINI_CHECK!(loaded.width == surf.width);
        MINI_CHECK!(loaded.m_dim == surf.m_dim);
        MINI_CHECK!(loaded.m_is_rat == surf.m_is_rat);
        MINI_CHECK!(loaded.m_order[0] == surf.m_order[0]);
        MINI_CHECK!(loaded.m_order[1] == surf.m_order[1]);
        MINI_CHECK!(loaded.m_cv_count[0] == surf.m_cv_count[0]);
        MINI_CHECK!(loaded.m_cv_count[1] == surf.m_cv_count[1]);
        MINI_CHECK!(loaded.surfacecolor.r == 255);
        MINI_CHECK!(loaded.surfacecolor.g == 128);
        MINI_CHECK!(loaded.surfacecolor.b == 64);
    })
}

pub fn run_nurbssurface_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::color::Color;

        // Create and setup surface
        let mut surf = NurbsSurface::create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0).unwrap();
        surf.name = "test_nurbssurface".to_string();
        surf.width = 2.0;
        surf.surfacecolor = Color::new(255, 128, 64, 255);

        // Set some CVs
        for i in 0..3 {
            for j in 0..3 {
                surf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }

        // Serialize to protobuf
        let filename = "serialization/test_nurbssurface.bin";
        surf.pb_dump(filename);
        let loaded = NurbsSurface::pb_load(filename);

        MINI_CHECK!(loaded.name == surf.name);
        MINI_CHECK!(loaded.width == surf.width);
        MINI_CHECK!(loaded.m_dim == surf.m_dim);
        MINI_CHECK!(loaded.m_is_rat == surf.m_is_rat);
        MINI_CHECK!(loaded.m_order[0] == surf.m_order[0]);
        MINI_CHECK!(loaded.m_order[1] == surf.m_order[1]);
        MINI_CHECK!(loaded.m_cv_count[0] == surf.m_cv_count[0]);
        MINI_CHECK!(loaded.m_cv_count[1] == surf.m_cv_count[1]);
        MINI_CHECK!(loaded.surfacecolor.r == 255);
        MINI_CHECK!(loaded.surfacecolor.g == 128);
        MINI_CHECK!(loaded.surfacecolor.b == 64);
    })
}

pub fn run_nurbssurface_advanced_accessors() -> TestResult {
    MINI_TEST!("advanced_accessors", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create rational surface for testing get_cv_4d/set_cv_4d
        let mut surf = NurbsSurface::create_raw(3, true, 3, 3, 3, 3, false, false, 1.0, 1.0).unwrap();

        // Test set_cv_4d with homogeneous coordinates
        let x = 2.0;
        let y = 3.0;
        let z = 4.0;
        let w = 2.0;

        // Set CV using set_cv first, then change weight
        surf.set_cv(1, 1, &Point::new(x, y, z));
        surf.set_weight(1, 1, w);

        // get_cv returns Euclidean point
        let pt = surf.get_cv(1, 1).unwrap();
        let retrieved_w = surf.weight(1, 1);

        // Test knot_multiplicity
        let mult = surf.knot_count(0);
        let first_knot_mult = if mult > 0 {
            let first_val = surf.knot(0, 0).unwrap();
            let mut count = 1;
            for i in 1..mult {
                if let Some(val) = surf.knot(0, i) {
                    if (val - first_val).abs() < 1e-10 {
                        count += 1;
                    } else {
                        break;
                    }
                }
            }
            count
        } else {
            0
        };

        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(pt[0] == x && pt[1] == y && pt[2] == z);
        MINI_CHECK!(retrieved_w == w);
        MINI_CHECK!(first_knot_mult > 0);
    })
}

pub fn run_nurbssurface_clamp_operations() -> TestResult {
    MINI_TEST!("clamp_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut surf = NurbsSurface::create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0).unwrap();

        // Set up control points
        for i in 0..4 {
            for j in 0..4 {
                surf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }

        // Test clamp_end
        let _was_clamped_before = surf.is_clamped(0, 2);
        surf.clamp_end(0, 2);
        let is_clamped_after = surf.is_clamped(0, 2);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(is_clamped_after);
    })
}

pub fn run_nurbssurface_singularity() -> TestResult {
    MINI_TEST!("singularity", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create a simple surface
        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        // Set all CVs to different points (non-singular)
        surf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        surf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        surf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_clamped(0, 0));
    })
}

pub fn run_nurbssurface_bounding_box() -> TestResult {
    MINI_TEST!("bounding_box", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 3, 3, false, false, 1.0, 1.0).unwrap();

        // Set CVs in a known range
        for i in 0..3 {
            for j in 0..3 {
                surf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
            }
        }

        // Get bounding box
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;

        for i in 0..3 {
            for j in 0..3 {
                if let Some(pt) = surf.get_cv(i, j) {
                    min_x = min_x.min(pt[0]);
                    max_x = max_x.max(pt[0]);
                    min_y = min_y.min(pt[1]);
                    max_y = max_y.max(pt[1]);
                }
            }
        }

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(min_x == 0.0);
        MINI_CHECK!(max_x == 2.0);
        MINI_CHECK!(min_y == 0.0);
        MINI_CHECK!(max_y == 2.0);
    })
}

pub fn run_nurbssurface_domain_operations() -> TestResult {
    MINI_TEST!("domain_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::tolerance::TOLERANCE;

        let mut surf = NurbsSurface::create_raw(3, false, 3, 3, 3, 3, false, false, 1.0, 1.0).unwrap();

        // Get initial domain
        let dom_u = surf.domain(0).unwrap();
        let dom_v = surf.domain(1).unwrap();

        // Set new domain
        surf.set_domain(0, 0.0, 10.0);
        surf.set_domain(1, 5.0, 15.0);

        let new_dom_u = surf.domain(0).unwrap();
        let new_dom_v = surf.domain(1).unwrap();

        // Get span vectors
        let span_u = surf.get_span_vector(0);
        let span_v = surf.get_span_vector(1);

        MINI_CHECK!(dom_u.0 == 0.0 && dom_u.1 > 0.0);
        MINI_CHECK!(dom_v.0 == 0.0 && dom_v.1 > 0.0);
        MINI_CHECK!(TOLERANCE.is_close(new_dom_u.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(new_dom_u.1, 10.0));
        MINI_CHECK!(TOLERANCE.is_close(new_dom_v.0, 5.0));
        MINI_CHECK!(TOLERANCE.is_close(new_dom_v.1, 15.0));
        MINI_CHECK!(span_u.len() > 0);
        MINI_CHECK!(span_v.len() > 0);
    })
}

pub fn run_nurbssurface_corner_points() -> TestResult {
    MINI_TEST!("corner_points", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        // Set corner control points
        surf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        surf.set_cv(1, 0, &Point::new(10.0, 0.0, 0.0));
        surf.set_cv(0, 1, &Point::new(0.0, 10.0, 0.0));
        surf.set_cv(1, 1, &Point::new(10.0, 10.0, 0.0));

        // Get corner points
        let p00 = surf.point_at_corner(0, 0).unwrap();
        let p10 = surf.point_at_corner(1, 0).unwrap();
        let p01 = surf.point_at_corner(0, 1).unwrap();
        let p11 = surf.point_at_corner(1, 1).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(p00[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(p10[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(p01[1], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(p11[0], 10.0) && TOLERANCE.is_close(p11[1], 10.0));
    })
}

pub fn run_nurbssurface_swap_coordinates() -> TestResult {
    MINI_TEST!("swap_coordinates", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        // Set a control point with distinct coordinates
        surf.set_cv(0, 0, &Point::new(1.0, 2.0, 3.0));

        // Swap X and Y
        surf.swap_coordinates(0, 1);

        let pt = surf.get_cv(0, 0).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 3.0));
    })
}

pub fn run_nurbssurface_zero_cvs() -> TestResult {
    MINI_TEST!("zero_cvs", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        let mut surf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();

        // Set non-zero control points
        surf.set_cv(0, 0, &Point::new(1.0, 2.0, 3.0));
        surf.set_cv(1, 1, &Point::new(4.0, 5.0, 6.0));

        // Zero all CVs
        surf.zero_cvs();

        let pt0 = surf.get_cv(0, 0).unwrap();
        let pt1 = surf.get_cv(1, 1).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt0[0], 0.0) &&
                   TOLERANCE.is_close(pt0[1], 0.0) &&
                   TOLERANCE.is_close(pt0[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt1[0], 0.0) &&
                   TOLERANCE.is_close(pt1[1], 0.0) &&
                   TOLERANCE.is_close(pt1[2], 0.0));
    })
}

pub fn run_nurbssurface_get_knots() -> TestResult {
    MINI_TEST!("get_knots", {
        use crate::nurbssurface::NurbsSurface;

        let surf = NurbsSurface::create_raw(3, false, 4, 3, 4, 3, false, false, 1.0, 2.0).unwrap();

        let knots_u = surf.get_knots(0);
        let knots_v = surf.get_knots(1);

        MINI_CHECK!(knots_u.len() == surf.knot_count(0) as usize);
        MINI_CHECK!(knots_v.len() == surf.knot_count(1) as usize);
        MINI_CHECK!(knots_u.len() > 0);
        MINI_CHECK!(knots_v.len() > 0);
    })
}

pub fn run_nurbssurface_make_non_rational() -> TestResult {
    MINI_TEST!("make_non_rational", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create rational surface with all weights = 1
        let mut surf = NurbsSurface::create_raw(3, true, 3, 3, 3, 3, false, false, 1.0, 1.0).unwrap();

        // Set all weights to 1.0
        for i in 0..3 {
            for j in 0..3 {
                surf.set_cv(i, j, &Point::new(i as f64, j as f64, 0.0));
                surf.set_weight(i, j, 1.0);
            }
        }

        let was_rational = surf.is_rational();
        surf.make_non_rational();
        let is_rational_after = surf.is_rational();

        MINI_CHECK!(was_rational);
        MINI_CHECK!(!is_rational_after);
    })
}

pub fn run_nurbssurface_create_clamped_uniform() -> TestResult {
    MINI_TEST!("create_clamped_uniform", {
        use crate::nurbssurface::NurbsSurface;

        let mut surf = NurbsSurface::new();
        surf.create_clamped_uniform(3, 4, 3, 4, 4, 1.0, 2.0);

        let _dom_u = surf.domain(0);
        let _dom_v = surf.domain(1);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.dimension() == 3);
        MINI_CHECK!(surf.order(0) == 4);
        MINI_CHECK!(surf.order(1) == 3);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 4);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(surf.is_clamped(0, 0) && surf.is_clamped(0, 1));
        MINI_CHECK!(surf.is_clamped(1, 0) && surf.is_clamped(1, 1));
    })
}

pub fn run_nurbssurface_knot_multiplicity() -> TestResult {
    MINI_TEST!("knot_multiplicity", {
        use crate::nurbssurface::NurbsSurface;

        let surf = NurbsSurface::create_raw(3, false, 4, 4, 4, 4, false, false, 1.0, 1.0).unwrap();

        // Check first knot multiplicity (should be equal to degree for clamped)
        let mult_u_start = surf.knot_multiplicity(0, 0);
        let mult_v_start = surf.knot_multiplicity(1, 0);

        // Check last knot multiplicity
        let last_u = surf.knot_count(0) - 1;
        let last_v = surf.knot_count(1) - 1;
        let mult_u_end = surf.knot_multiplicity(0, last_u as usize);
        let mult_v_end = surf.knot_multiplicity(1, last_v as usize);

        MINI_CHECK!(mult_u_start >= surf.degree(0) as usize);
        MINI_CHECK!(mult_v_start >= surf.degree(1) as usize);
        MINI_CHECK!(mult_u_end >= surf.degree(0) as usize);
        MINI_CHECK!(mult_v_end >= surf.degree(1) as usize);
    })
}

pub fn run_nurbssurface_sphere() -> TestResult {
    MINI_TEST!("sphere", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        let radius = 2.0;
        let w = (2.0_f64).sqrt() / 2.0;
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 3, 9, 5, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_sphere".to_string();

        let u_knots = [0.0, 0.0, pi * 0.5, pi * 0.5, pi, pi, pi * 1.5, pi * 1.5, pi * 2.0, pi * 2.0];
        for i in 0..10 { surf.set_knot(0, i, u_knots[i]); }

        let v_knots = [-pi * 0.5, -pi * 0.5, 0.0, 0.0, pi * 0.5, pi * 0.5];
        for i in 0..6 { surf.set_knot(1, i, v_knots[i]); }

        let lat_weights = [w, 0.5, w, 0.5, w];
        let lat_z = [-radius, -radius * w, 0.0, radius * w, radius];
        let lat_r = [0.0, radius * w, radius, radius * w, 0.0];

        for j in 0..5 {
            let r = lat_r[j];
            let z = lat_z[j];
            let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75, pi, pi * 1.25, pi * 1.5, pi * 1.75, pi * 2.0];
            for i in 0..9 {
                let x = r * angles[i].cos();
                let y = r * angles[i].sin();
                surf.set_cv(i, j, &Point::new(x, y, z));
                let weight = if i % 2 == 0 { w } else { lat_weights[j] };
                let weight = if j == 0 || j == 4 { w } else { weight };
                surf.set_weight(i, j, weight);
            }
        }

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 2);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 5);

        let pt = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));

        let north = surf.point_at(0.0, pi * 0.5).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(north[2], radius));
    })
}

pub fn run_nurbssurface_cylinder() -> TestResult {
    MINI_TEST!("cylinder", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        let radius = 1.5;
        let height = 3.0;
        let w = (2.0_f64).sqrt() / 2.0;
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_cylinder".to_string();

        let u_knots = [0.0, 0.0, pi * 0.5, pi * 0.5, pi, pi, pi * 1.5, pi * 1.5, pi * 2.0, pi * 2.0];
        for i in 0..10 { surf.set_knot(0, i, u_knots[i]); }

        surf.set_knot(1, 0, 0.0);
        surf.set_knot(1, 1, height);

        let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75, pi, pi * 1.25, pi * 1.5, pi * 1.75, pi * 2.0];

        for j in 0..2 {
            let z = if j == 0 { 0.0 } else { height };
            for i in 0..9 {
                let (x, y) = if i % 2 == 1 {
                    (radius * (2.0_f64).sqrt() * angles[i].cos(), radius * (2.0_f64).sqrt() * angles[i].sin())
                } else {
                    (radius * angles[i].cos(), radius * angles[i].sin())
                };
                surf.set_cv(i, j, &Point::new(x, y, z));
                let weight = if i % 2 == 0 { 1.0 } else { w };
                surf.set_weight(i, j, weight);
            }
        }

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 1);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 2);

        let pt_bottom = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_bottom[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt_bottom[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_bottom[2], 0.0));

        let pt_top = surf.point_at(0.0, height).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_top[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt_top[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_top[2], height));

        let pt_mid = surf.point_at(pi * 0.5, height * 0.5).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[1], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[2], height * 0.5));
    })
}

pub fn run_nurbssurface_torus() -> TestResult {
    MINI_TEST!("torus", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        let major_radius = 3.0;
        let minor_radius = 1.0;
        let w = (2.0_f64).sqrt() / 2.0;
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 3, 9, 9, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_torus".to_string();

        let knots = [0.0, 0.0, pi * 0.5, pi * 0.5, pi, pi, pi * 1.5, pi * 1.5, pi * 2.0, pi * 2.0];
        for i in 0..10 {
            surf.set_knot(0, i, knots[i]);
            surf.set_knot(1, i, knots[i]);
        }

        let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75, pi, pi * 1.25, pi * 1.5, pi * 1.75, pi * 2.0];

        for i in 0..9 {
            let major_angle = angles[i];
            let cos_ma = major_angle.cos();
            let sin_ma = major_angle.sin();
            let major_scale = if i % 2 == 0 { 1.0 } else { (2.0_f64).sqrt() };

            for j in 0..9 {
                let minor_angle = angles[j];
                let cos_mi = minor_angle.cos();
                let sin_mi = minor_angle.sin();
                let minor_scale = if j % 2 == 0 { 1.0 } else { (2.0_f64).sqrt() };

                let r = major_radius + minor_radius * minor_scale * cos_mi;
                let x = r * major_scale * cos_ma;
                let y = r * major_scale * sin_ma;
                let z = minor_radius * minor_scale * sin_mi;

                surf.set_cv(i, j, &Point::new(x, y, z));

                let w_major = if i % 2 == 0 { 1.0 } else { w };
                let w_minor = if j % 2 == 0 { 1.0 } else { w };
                surf.set_weight(i, j, w_major * w_minor);
            }
        }

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 2);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 9);

        let pt = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt[0], major_radius + minor_radius));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));

        let pt_opp = surf.point_at(pi, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_opp[0], -(major_radius + minor_radius)));
        MINI_CHECK!(TOLERANCE.is_close(pt_opp[1], 0.0));
    })
}

pub fn run_nurbssurface_cone() -> TestResult {
    MINI_TEST!("cone", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        let radius = 2.0;
        let height = 4.0;
        let w = (2.0_f64).sqrt() / 2.0;
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_cone".to_string();

        let u_knots = [0.0, 0.0, pi * 0.5, pi * 0.5, pi, pi, pi * 1.5, pi * 1.5, pi * 2.0, pi * 2.0];
        for i in 0..10 { surf.set_knot(0, i, u_knots[i]); }

        surf.set_knot(1, 0, 0.0);
        surf.set_knot(1, 1, height);

        let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75, pi, pi * 1.25, pi * 1.5, pi * 1.75, pi * 2.0];

        for i in 0..9 {
            surf.set_cv(i, 0, &Point::new(0.0, 0.0, height));
            let weight = if i % 2 == 0 { 1.0 } else { w };
            surf.set_weight(i, 0, weight);
        }

        for i in 0..9 {
            let (x, y) = if i % 2 == 1 {
                (radius * (2.0_f64).sqrt() * angles[i].cos(), radius * (2.0_f64).sqrt() * angles[i].sin())
            } else {
                (radius * angles[i].cos(), radius * angles[i].sin())
            };
            surf.set_cv(i, 1, &Point::new(x, y, 0.0));
            let weight = if i % 2 == 0 { 1.0 } else { w };
            surf.set_weight(i, 1, weight);
        }

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 1);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 2);

        let apex = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(apex[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(apex[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(apex[2], height));

        let base = surf.point_at(0.0, height).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(base[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(base[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(base[2], 0.0));

        let mid = surf.point_at(0.0, height * 0.5).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(mid[0], radius * 0.5));
        MINI_CHECK!(TOLERANCE.is_close(mid[2], height * 0.5));
    })
}

// Register tests with the shared registry
REGISTER_MINI_TEST!("NurbsSurface", "constructor", crate::nurbssurface_test::run_nurbssurface_constructor);
REGISTER_MINI_TEST!("NurbsSurface", "create_operations", crate::nurbssurface_test::run_nurbssurface_create_operations);
REGISTER_MINI_TEST!("NurbsSurface", "accessors", crate::nurbssurface_test::run_nurbssurface_accessors);
REGISTER_MINI_TEST!("NurbsSurface", "knot_operations", crate::nurbssurface_test::run_nurbssurface_knot_operations);
REGISTER_MINI_TEST!("NurbsSurface", "rational_operations", crate::nurbssurface_test::run_nurbssurface_rational_operations);
REGISTER_MINI_TEST!("NurbsSurface", "evaluation", crate::nurbssurface_test::run_nurbssurface_evaluation);
REGISTER_MINI_TEST!("NurbsSurface", "geometric_queries", crate::nurbssurface_test::run_nurbssurface_geometric_queries);
REGISTER_MINI_TEST!("NurbsSurface", "modification", crate::nurbssurface_test::run_nurbssurface_modification);
REGISTER_MINI_TEST!("NurbsSurface", "isocurve", crate::nurbssurface_test::run_nurbssurface_isocurve);
REGISTER_MINI_TEST!("NurbsSurface", "transformation", crate::nurbssurface_test::run_nurbssurface_transformation);
REGISTER_MINI_TEST!("NurbsSurface", "json_roundtrip", crate::nurbssurface_test::run_nurbssurface_json_roundtrip);
REGISTER_MINI_TEST!("NurbsSurface", "protobuf_roundtrip", crate::nurbssurface_test::run_nurbssurface_protobuf_roundtrip);
REGISTER_MINI_TEST!("NurbsSurface", "advanced_accessors", crate::nurbssurface_test::run_nurbssurface_advanced_accessors);
REGISTER_MINI_TEST!("NurbsSurface", "clamp_operations", crate::nurbssurface_test::run_nurbssurface_clamp_operations);
REGISTER_MINI_TEST!("NurbsSurface", "singularity", crate::nurbssurface_test::run_nurbssurface_singularity);
REGISTER_MINI_TEST!("NurbsSurface", "bounding_box", crate::nurbssurface_test::run_nurbssurface_bounding_box);
REGISTER_MINI_TEST!("NurbsSurface", "domain_operations", crate::nurbssurface_test::run_nurbssurface_domain_operations);
REGISTER_MINI_TEST!("NurbsSurface", "corner_points", crate::nurbssurface_test::run_nurbssurface_corner_points);
REGISTER_MINI_TEST!("NurbsSurface", "swap_coordinates", crate::nurbssurface_test::run_nurbssurface_swap_coordinates);
REGISTER_MINI_TEST!("NurbsSurface", "zero_cvs", crate::nurbssurface_test::run_nurbssurface_zero_cvs);
REGISTER_MINI_TEST!("NurbsSurface", "get_knots", crate::nurbssurface_test::run_nurbssurface_get_knots);
REGISTER_MINI_TEST!("NurbsSurface", "make_non_rational", crate::nurbssurface_test::run_nurbssurface_make_non_rational);
REGISTER_MINI_TEST!("NurbsSurface", "create_clamped_uniform", crate::nurbssurface_test::run_nurbssurface_create_clamped_uniform);
REGISTER_MINI_TEST!("NurbsSurface", "knot_multiplicity", crate::nurbssurface_test::run_nurbssurface_knot_multiplicity);
REGISTER_MINI_TEST!("NurbsSurface", "sphere", crate::nurbssurface_test::run_nurbssurface_sphere);
REGISTER_MINI_TEST!("NurbsSurface", "cylinder", crate::nurbssurface_test::run_nurbssurface_cylinder);
REGISTER_MINI_TEST!("NurbsSurface", "torus", crate::nurbssurface_test::run_nurbssurface_torus);
REGISTER_MINI_TEST!("NurbsSurface", "cone", crate::nurbssurface_test::run_nurbssurface_cone);
