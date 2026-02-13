use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_nurbssurface_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::NurbsSurface;
        use crate::Point;

        let points = vec![
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

        let s = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Minimal and Full String Representation
        let sstr = s.to_string();
        let srepr = s.repr();

        // Copy (duplicates everything except guid)
        let scopy = s.duplicate();
        let _sother = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Point division matching Rhino's 4x6 grid
        let (v, _uv) = s.divide_by_count(4, 6);

        MINI_CHECK!(s.is_valid() == true);
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 4);
        MINI_CHECK!(s.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(s.cv_count_dir(None) == 16);
        MINI_CHECK!(s.degree(0) == 3);
        MINI_CHECK!(s.degree(1) == 3);
        MINI_CHECK!(s.order(0) == 4);
        MINI_CHECK!(s.order(1) == 4);
        MINI_CHECK!(s.dimension() == 3);
        MINI_CHECK!(!s.is_rational());
        MINI_CHECK!(s.knot_count(0) == 6);
        MINI_CHECK!(s.knot_count(1) == 6);
        MINI_CHECK!(s.name == "my_nurbssurface");
        MINI_CHECK!(!s.guid.is_empty());
        MINI_CHECK!(sstr == "NurbsSurface(name=my_nurbssurface, degree=(3,3), cvs=(4,4))");
        MINI_CHECK!(srepr.contains("name=my_nurbssurface"));
        MINI_CHECK!(scopy.cv_count_dir(None) == s.cv_count_dir(None));
        MINI_CHECK!(scopy.guid != s.guid);

        MINI_CHECK!(TOLERANCE.is_point_close(&v[0][0], &Point::new(0.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[0][1], &Point::new(-0.416666666666667, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[0][2], &Point::new(-0.666666666666667, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[0][3], &Point::new(-0.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[0][4], &Point::new(-0.666666666666667, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[0][5], &Point::new(-0.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[0][6], &Point::new(0.000000000000000, 5.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[1][0], &Point::new(0.992187500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[1][1], &Point::new(0.881510416666667, 0.333912037037037, 1.958333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[1][2], &Point::new(0.815104166666667, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[1][3], &Point::new(0.792968750000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[1][4], &Point::new(0.815104166666667, 3.620370370370370, 2.458333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[1][5], &Point::new(0.881510416666667, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[1][6], &Point::new(0.992187500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[2][0], &Point::new(2.500000000000000, -0.750000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[2][1], &Point::new(2.500000000000000, 0.252314814814815, 2.333333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[2][2], &Point::new(2.500000000000000, 1.351851851851852, 2.833333333333334)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[2][3], &Point::new(2.500000000000000, 2.500000000000000, 3.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[2][4], &Point::new(2.500000000000000, 3.648148148148148, 2.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[2][5], &Point::new(2.500000000000000, 4.747685185185186, 2.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[2][6], &Point::new(2.500000000000000, 5.750000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[3][0], &Point::new(4.007812500000000, -0.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[3][1], &Point::new(4.118489583333334, 0.333912037037037, 1.958333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[3][2], &Point::new(4.184895833333334, 1.379629629629630, 2.458333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[3][3], &Point::new(4.207031250000000, 2.500000000000000, 2.625000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[3][4], &Point::new(4.184895833333333, 3.620370370370370, 2.458333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[3][5], &Point::new(4.118489583333333, 4.666087962962964, 1.958333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[3][6], &Point::new(4.007812500000000, 5.562500000000000, 1.125000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[4][0], &Point::new(5.000000000000000, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[4][1], &Point::new(5.416666666666668, 0.578703703703704, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[4][2], &Point::new(5.666666666666668, 1.462962962962963, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[4][3], &Point::new(5.750000000000000, 2.500000000000000, 1.500000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[4][4], &Point::new(5.666666666666666, 3.537037037037037, 1.333333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[4][5], &Point::new(5.416666666666667, 4.421296296296297, 0.833333333333333)));
        MINI_CHECK!(TOLERANCE.is_point_close(&v[4][6], &Point::new(5.000000000000000, 5.000000000000000, 0.000000000000000)));
    })
}

pub fn run_nurbssurface_booleans_queries() -> TestResult {
    MINI_TEST!("Booleans Queries", {
        use crate::primitives::Primitives;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 5.0);

        // Validity surface and knots
        let is_valid = s.is_valid();
        let are_knots_valid = s.is_valid_knot_vector(0) && s.is_valid_knot_vector(1);

        // Are control points weights enabled?
        let is_rational = s.is_rational();

        // Sphere has one seam that is closed, but two poles
        let is_closed = s.is_closed(0) == true && s.is_closed(1) == false;

        // sphere cannot be truly periodic because it has poles
        let is_periodic = s.is_periodic(0) && s.is_periodic(1);

        // Planarity
        let is_planar = s.is_planar(1e-6);

        // Surface is collapsed to a point
        let is_point = s.is_singular(0) && s.is_singular(1) && s.is_singular(2) && s.is_singular(3);

        // Most surfaces are clamped except periodic surfaces
        let is_clamped = s.is_clamped(0, 2) && s.is_clamped(1, 2);

        MINI_CHECK!(is_valid);
        MINI_CHECK!(are_knots_valid);
        MINI_CHECK!(is_rational);
        MINI_CHECK!(is_closed);
        MINI_CHECK!(!is_periodic);
        MINI_CHECK!(!is_planar);
        MINI_CHECK!(!is_point);
        MINI_CHECK!(is_clamped);
    })
}

pub fn run_nurbssurface_accessors() -> TestResult {
    MINI_TEST!("Accessors", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..5 {
            for j in 0..4 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 3, 2, 5, 4, &points).unwrap();

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
    MINI_TEST!("Knot_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

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
    MINI_TEST!("Rational_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create non-rational surface, then make rational
        let points = vec![Point::new(0.0, 0.0, 0.0); 9];
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();

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
    MINI_TEST!("Evaluation", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        // Create simple bilinear surface
        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

        // Evaluate at corner
        let (u0, u1) = surf.domain(0).unwrap();
        let (v0, v1) = surf.domain(1).unwrap();

        let pt_corner = surf.point_at(u0, v0).unwrap();

        // Evaluate at center (should be center of unit square)
        let u_mid = (u0 + u1) / 2.0;
        let v_mid = (v0 + v1) / 2.0;
        let pt_mid = surf.point_at(u_mid, v_mid).unwrap();

        // Test derivatives
        let derivs = surf.evaluate(u_mid, v_mid, 1);

        // Test normal (for flat plane in XY, normal should point in +Z)
        let normal = surf.normal_at(u_mid, v_mid);

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
    MINI_TEST!("Geometric_queries", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create and setup surface
        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

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
    MINI_TEST!("Modification", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0), Point::new(2.0, 1.0, 0.0),
        ];
        let mut surf = NurbsSurface::create(false, false, 1, 1, 3, 2, &points).unwrap();

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
    MINI_TEST!("Isocurve", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create surface
        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();

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
        MINI_CHECK!(iso_u.unwrap().is_valid());
        MINI_CHECK!(iso_v.is_some());
        MINI_CHECK!(iso_v.unwrap().is_valid());
    })
}

pub fn run_nurbssurface_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::xform::Xform;
        use crate::tolerance::TOLERANCE;

        // Create simple surface
        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let mut surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

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
    MINI_TEST!("Json_roundtrip", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::color::Color;
        use std::path::PathBuf;

        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.name = "test_nurbssurface".to_string();
        surf.width = 2.0;
        surf.facecolors = vec![Color::new(255, 128, 64, 255)];
        surf.pointcolors = vec![Color::new(0, 255, 0, 255)];
        surf.linecolors = vec![Color::new(0, 0, 255, 255)];

        //   jsondump()      │ String       │ to JSON string (internal use)
        //   jsonload(s)     │ String       │ from JSON string (internal use)
        //   json_dumps()    │ String       │ to JSON string
        //   json_loads(s)   │ String       │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        // JSON object
        let json = surf.jsondump().unwrap();
        let loaded_json = NurbsSurface::jsonload(&json).unwrap();

        // String
        let json_string = surf.json_dumps();
        let loaded_json_string = NurbsSurface::json_loads(&json_string);

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_nurbssurface.json");
        surf.json_dump(filename.to_str().unwrap());
        let loaded_from_file = NurbsSurface::json_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_json == surf);
        MINI_CHECK!(loaded_json_string == surf);
        MINI_CHECK!(loaded_from_file == surf);
    })
}

pub fn run_nurbssurface_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf_roundtrip", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::color::Color;
        use std::path::PathBuf;

        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.name = "test_nurbssurface".to_string();
        surf.width = 2.0;
        surf.facecolors = vec![Color::new(255, 128, 64, 255)];
        surf.pointcolors = vec![Color::new(0, 255, 0, 255)];
        surf.linecolors = vec![Color::new(0, 0, 255, 255)];

        //   pb_dumps()      │ bytes        │ to protobuf bytes
        //   pb_loads(b)     │ bytes        │ from protobuf bytes
        //   pb_dump(path)   │ file         │ write to file
        //   pb_load(path)   │ file         │ read from file

        // String
        let proto_string = surf.pb_dumps();
        let loaded_proto_string = NurbsSurface::pb_loads(&proto_string).unwrap();

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_nurbssurface.bin");
        surf.pb_dump(filename.to_str().unwrap());
        let loaded = NurbsSurface::pb_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_proto_string == surf);
        MINI_CHECK!(loaded == surf);
    })
}

pub fn run_nurbssurface_advanced_accessors() -> TestResult {
    MINI_TEST!("Advanced_accessors", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        // Create rational surface for testing get_cv_4d/set_cv_4d
        let points = vec![Point::new(0.0, 0.0, 0.0); 9];
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.make_rational();

        // Test set_cv_4d with homogeneous coordinates
        let x = 2.0;
        let y = 3.0;
        let z = 4.0;
        let w = 2.0;

        // Set CV using set_cv_4d
        surf.set_cv_4d(1, 1, x, y, z, w);

        // Get CV and verify using get_cv_4d
        let (rx, ry, rz, rw) = surf.get_cv_4d(1, 1).unwrap();

        // Also test get_cv
        let pt = surf.get_cv(1, 1).unwrap();
        let retrieved_w = surf.weight(1, 1);

        // Test knot_multiplicity
        let mult = surf.knot_count(0);
        let first_knot_mult = if mult > 0 {
            let first_val = surf.knot(0, 0).unwrap();
            let mut count = 1;
            for i in 1..mult as usize {
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
        MINI_CHECK!(TOLERANCE.is_close(rx, x));
        MINI_CHECK!(TOLERANCE.is_close(ry, y));
        MINI_CHECK!(TOLERANCE.is_close(rz, z));
        MINI_CHECK!(TOLERANCE.is_close(rw, w));
        // get_cv returns Euclidean coordinates, so it divides homogeneous coords by w
        MINI_CHECK!(TOLERANCE.is_close(pt[0], x/w));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], y/w));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], z/w));
        MINI_CHECK!(TOLERANCE.is_close(retrieved_w, w));
        MINI_CHECK!(first_knot_mult > 0);
    })
}

pub fn run_nurbssurface_clamp_operations() -> TestResult {
    MINI_TEST!("Clamp_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

        // Test clamp_end
        let _was_clamped_before = surf.is_clamped(0, 2);
        surf.clamp_end(0, 2);
        let is_clamped_after = surf.is_clamped(0, 2);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(is_clamped_after);
    })
}

pub fn run_nurbssurface_singularity() -> TestResult {
    MINI_TEST!("Singularity", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create a simple surface with all CVs at different points (non-singular)
        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0),
        ];
        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

        // Test is_singular for each side
        let is_singular_south = surf.is_singular(0);
        let is_singular_east = surf.is_singular(1);
        let is_singular_north = surf.is_singular(2);
        let is_singular_west = surf.is_singular(3);

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(!is_singular_south);
        MINI_CHECK!(!is_singular_east);
        MINI_CHECK!(!is_singular_north);
        MINI_CHECK!(!is_singular_west);
    })
}

pub fn run_nurbssurface_bounding_box() -> TestResult {
    MINI_TEST!("Bounding_box", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 1, 1, 3, 3, &points).unwrap();

        // Get bounding box
        let _bbox = surf.get_bounding_box();

        MINI_CHECK!(surf.is_valid());
    })
}

pub fn run_nurbssurface_domain_operations() -> TestResult {
    MINI_TEST!("Domain_operations", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        let points = vec![Point::new(0.0, 0.0, 0.0); 9];
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();

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
    MINI_TEST!("Corner_points", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        let points = vec![
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 10.0, 0.0),
            Point::new(10.0, 0.0, 0.0), Point::new(10.0, 10.0, 0.0),
        ];
        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

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
    MINI_TEST!("Swap_coordinates", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        let points = vec![
            Point::new(1.0, 2.0, 3.0), Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 0.0, 0.0),
        ];
        let mut surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

        // Swap X and Y
        surf.swap_coordinates(0, 1);

        let pt = surf.get_cv(0, 0).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 3.0));
    })
}

pub fn run_nurbssurface_zero_cvs() -> TestResult {
    MINI_TEST!("Zero_cvs", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::TOLERANCE;

        let points = vec![
            Point::new(1.0, 2.0, 3.0), Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 0.0, 0.0), Point::new(4.0, 5.0, 6.0),
        ];
        let mut surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &points).unwrap();

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
    MINI_TEST!("Get_knots", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..4 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 3, 2, 4, 3, &points).unwrap();

        let knots_u = surf.get_knots(0);
        let knots_v = surf.get_knots(1);

        MINI_CHECK!(knots_u.len() == surf.knot_count(0) as usize);
        MINI_CHECK!(knots_v.len() == surf.knot_count(1) as usize);
        MINI_CHECK!(knots_u.len() > 0);
        MINI_CHECK!(knots_v.len() > 0);
    })
}

pub fn run_nurbssurface_make_non_rational() -> TestResult {
    MINI_TEST!("Make_non_rational", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        // Create surface, then make rational with all weights = 1
        let mut points = Vec::new();
        for i in 0..3 {
            for j in 0..3 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let mut surf = NurbsSurface::create(false, false, 2, 2, 3, 3, &points).unwrap();
        surf.make_rational();

        // Set all weights to 1.0
        for i in 0..3 {
            for j in 0..3 {
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
    MINI_TEST!("Create_clamped_uniform", {
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
    MINI_TEST!("Knot_multiplicity", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;

        let mut points = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                points.push(Point::new(i as f64, j as f64, 0.0));
            }
        }
        let surf = NurbsSurface::create(false, false, 3, 3, 4, 4, &points).unwrap();

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
    MINI_TEST!("Sphere", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        // Create a sphere as a rational NURBS surface
        // Sphere: 9x5 control points, degree 2 in both directions
        // Based on OpenNURBS sphere representation
        let radius = 2.0;
        let w = (2.0_f64).sqrt() / 2.0;  // 0.707107
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 3, 9, 5, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_sphere".to_string();

        // U-knots: periodic around equator with multiplicity 2
        let u_knots = [0.0, 0.0, pi * 0.5, pi * 0.5,
                        pi, pi, pi * 1.5, pi * 1.5,
                        pi * 2.0, pi * 2.0];
        for i in 0..10 { surf.set_knot(0, i, u_knots[i]); }

        // V-knots: from south pole to north pole
        let v_knots = [-pi * 0.5, -pi * 0.5, 0.0, 0.0,
                        pi * 0.5, pi * 0.5];
        for i in 0..6 { surf.set_knot(1, i, v_knots[i]); }

        // Set up control points for sphere (9 around, 5 latitude levels)
        // Latitude levels: south pole, -45deg, equator, +45deg, north pole
        let lat_weights = [w, 0.5, w, 0.5, w];  // alternating for cardinal/diagonal
        let lat_z = [-radius, -radius * w, 0.0, radius * w, radius];
        let lat_r = [0.0, radius * w, radius, radius * w, 0.0];  // radius at each latitude

        for j in 0..5 {
            let r = lat_r[j];
            let z = lat_z[j];
            // 9 points around (0, 45, 90, 135, 180, 225, 270, 315, 360=0)
            let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75,
                           pi, pi * 1.25, pi * 1.5, pi * 1.75,
                           pi * 2.0];
            for i in 0..9 {
                let x = r * angles[i].cos();
                let y = r * angles[i].sin();
                surf.set_cv(i, j, &Point::new(x, y, z));
                // Weight: w for cardinal directions (0, 90, 180, 270), 0.5 for diagonals at non-pole latitudes
                let weight = if i % 2 == 0 { w } else { lat_weights[j] };
                let weight = if j == 0 || j == 4 { w } else { weight };  // poles
                surf.set_weight(i, j, weight);
            }
        }

        // Verify sphere properties
        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 2);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 5);

        // Check point on equator at angle 0 (should be at (radius, 0, 0))
        let pt = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));

        // Check north pole
        let north = surf.point_at(0.0, pi * 0.5).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(north[2], radius));
    })
}

pub fn run_nurbssurface_cylinder() -> TestResult {
    MINI_TEST!("Cylinder", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        // Create an uncapped cylinder as a rational NURBS surface
        // Cylinder: 9x2 control points (circle at bottom, circle at top)
        // Degree 2 in U (angular), degree 1 in V (height)
        let radius = 1.5;
        let height = 3.0;
        let w = (2.0_f64).sqrt() / 2.0;
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_cylinder".to_string();

        // U-knots: periodic for circle
        let u_knots = [0.0, 0.0, pi * 0.5, pi * 0.5,
                        pi, pi, pi * 1.5, pi * 1.5,
                        pi * 2.0, pi * 2.0];
        for i in 0..10 { surf.set_knot(0, i, u_knots[i]); }

        // V-knots: linear from bottom to top
        surf.set_knot(1, 0, 0.0);
        surf.set_knot(1, 1, height);

        // Set up control points for cylinder (9 around, 2 heights)
        let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75,
                       pi, pi * 1.25, pi * 1.5, pi * 1.75,
                       pi * 2.0];

        for j in 0..2 {
            let z = if j == 0 { 0.0 } else { height };
            for i in 0..9 {
                let mut x = radius * angles[i].cos();
                let mut y = radius * angles[i].sin();
                // For diagonal points (45, 135, etc), radius needs to be scaled
                if i % 2 == 1 {
                    x = radius * (2.0_f64).sqrt() * angles[i].cos();
                    y = radius * (2.0_f64).sqrt() * angles[i].sin();
                }
                surf.set_cv(i, j, &Point::new(x, y, z));
                let weight = if i % 2 == 0 { 1.0 } else { w };
                surf.set_weight(i, j, weight);
            }
        }

        // Verify cylinder properties
        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 1);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 2);

        // Check point on bottom circle at angle 0
        let pt_bottom = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_bottom[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt_bottom[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_bottom[2], 0.0));

        // Check point on top circle at angle 0
        let pt_top = surf.point_at(0.0, height).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_top[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt_top[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_top[2], height));

        // Check midpoint at angle PI/2
        let pt_mid = surf.point_at(pi * 0.5, height * 0.5).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[1], radius));
        MINI_CHECK!(TOLERANCE.is_close(pt_mid[2], height * 0.5));
    })
}

pub fn run_nurbssurface_torus() -> TestResult {
    MINI_TEST!("Torus", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        // Create a torus as a rational NURBS surface
        // Torus: 9x9 control points (minor circle revolved around major axis)
        // Degree 2 in both directions
        let major_radius = 3.0;  // distance from center to tube center
        let minor_radius = 1.0;  // tube radius
        let w = (2.0_f64).sqrt() / 2.0;
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 3, 9, 9, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_torus".to_string();

        // Both U and V knots: periodic for circles
        let knots = [0.0, 0.0, pi * 0.5, pi * 0.5,
                      pi, pi, pi * 1.5, pi * 1.5,
                      pi * 2.0, pi * 2.0];
        for i in 0..10 {
            surf.set_knot(0, i, knots[i]);
            surf.set_knot(1, i, knots[i]);
        }

        // Set up control points for torus
        // U: major angle (around torus), V: minor angle (around tube)
        let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75,
                       pi, pi * 1.25, pi * 1.5, pi * 1.75,
                       pi * 2.0];

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

                // Weight is product of major and minor weights
                let w_major = if i % 2 == 0 { 1.0 } else { w };
                let w_minor = if j % 2 == 0 { 1.0 } else { w };
                surf.set_weight(i, j, w_major * w_minor);
            }
        }

        // Verify torus properties
        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 2);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 9);

        // Check point at (0, 0) - should be on outer edge of torus
        let pt = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt[0], major_radius + minor_radius));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));

        // Check point at (PI, 0) - should be on opposite side of torus
        let pt_opp = surf.point_at(pi, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(pt_opp[0], -(major_radius + minor_radius)));
        MINI_CHECK!(TOLERANCE.is_close(pt_opp[1], 0.0));
    })
}

pub fn run_nurbssurface_cone() -> TestResult {
    MINI_TEST!("Cone", {
        use crate::nurbssurface::NurbsSurface;
        use crate::point::Point;
        use crate::tolerance::{TOLERANCE, PI};

        // Create an uncapped cone as a rational NURBS surface
        // Cone: 9x2 control points (apex at top, circle at base)
        // Degree 2 in U (angular), degree 1 in V (height)
        let radius = 2.0;
        let height = 4.0;
        let w = (2.0_f64).sqrt() / 2.0;
        let pi = PI;

        let mut surf = NurbsSurface::create_raw(3, true, 3, 2, 9, 2, false, false, 1.0, 1.0).unwrap();
        surf.name = "unit_cone".to_string();

        // U-knots: periodic for circle
        let u_knots = [0.0, 0.0, pi * 0.5, pi * 0.5,
                        pi, pi, pi * 1.5, pi * 1.5,
                        pi * 2.0, pi * 2.0];
        for i in 0..10 { surf.set_knot(0, i, u_knots[i]); }

        // V-knots: linear from apex to base
        surf.set_knot(1, 0, 0.0);
        surf.set_knot(1, 1, height);

        // Set up control points for cone (9 around, 2 heights: apex and base)
        let angles = [0.0, pi * 0.25, pi * 0.5, pi * 0.75,
                       pi, pi * 1.25, pi * 1.5, pi * 1.75,
                       pi * 2.0];

        // Apex (j=0) - all points collapse to the apex
        for i in 0..9 {
            surf.set_cv(i, 0, &Point::new(0.0, 0.0, height));
            let weight = if i % 2 == 0 { 1.0 } else { w };
            surf.set_weight(i, 0, weight);
        }

        // Base (j=1) - circle at z=0
        for i in 0..9 {
            let mut x = radius * angles[i].cos();
            let mut y = radius * angles[i].sin();
            if i % 2 == 1 {
                x = radius * (2.0_f64).sqrt() * angles[i].cos();
                y = radius * (2.0_f64).sqrt() * angles[i].sin();
            }
            surf.set_cv(i, 1, &Point::new(x, y, 0.0));
            let weight = if i % 2 == 0 { 1.0 } else { w };
            surf.set_weight(i, 1, weight);
        }

        // Verify cone properties
        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(surf.is_rational());
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 1);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 2);

        // Check apex
        let apex = surf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(apex[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(apex[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(apex[2], height));

        // Check point on base circle at angle 0
        let base = surf.point_at(0.0, height).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(base[0], radius));
        MINI_CHECK!(TOLERANCE.is_close(base[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(base[2], 0.0));

        // Check midpoint - should be at half radius, half height
        let mid = surf.point_at(0.0, height * 0.5).unwrap();
        MINI_CHECK!(TOLERANCE.is_close(mid[0], radius * 0.5));
        MINI_CHECK!(TOLERANCE.is_close(mid[2], height * 0.5));
    })
}

// Register tests with the shared registry
REGISTER_MINI_TEST!("NurbsSurface", "Constructor", crate::nurbssurface_test::run_nurbssurface_constructor);
REGISTER_MINI_TEST!("NurbsSurface", "Booleans Queries", crate::nurbssurface_test::run_nurbssurface_booleans_queries);
REGISTER_MINI_TEST!("NurbsSurface", "Accessors", crate::nurbssurface_test::run_nurbssurface_accessors);
REGISTER_MINI_TEST!("NurbsSurface", "Knot_operations", crate::nurbssurface_test::run_nurbssurface_knot_operations);
REGISTER_MINI_TEST!("NurbsSurface", "Rational_operations", crate::nurbssurface_test::run_nurbssurface_rational_operations);
REGISTER_MINI_TEST!("NurbsSurface", "Evaluation", crate::nurbssurface_test::run_nurbssurface_evaluation);
REGISTER_MINI_TEST!("NurbsSurface", "Geometric_queries", crate::nurbssurface_test::run_nurbssurface_geometric_queries);
REGISTER_MINI_TEST!("NurbsSurface", "Modification", crate::nurbssurface_test::run_nurbssurface_modification);
REGISTER_MINI_TEST!("NurbsSurface", "Isocurve", crate::nurbssurface_test::run_nurbssurface_isocurve);
REGISTER_MINI_TEST!("NurbsSurface", "Transformation", crate::nurbssurface_test::run_nurbssurface_transformation);
REGISTER_MINI_TEST!("NurbsSurface", "Json_roundtrip", crate::nurbssurface_test::run_nurbssurface_json_roundtrip);
REGISTER_MINI_TEST!("NurbsSurface", "Protobuf_roundtrip", crate::nurbssurface_test::run_nurbssurface_protobuf_roundtrip);
REGISTER_MINI_TEST!("NurbsSurface", "Advanced_accessors", crate::nurbssurface_test::run_nurbssurface_advanced_accessors);
REGISTER_MINI_TEST!("NurbsSurface", "Clamp_operations", crate::nurbssurface_test::run_nurbssurface_clamp_operations);
REGISTER_MINI_TEST!("NurbsSurface", "Singularity", crate::nurbssurface_test::run_nurbssurface_singularity);
REGISTER_MINI_TEST!("NurbsSurface", "Bounding_box", crate::nurbssurface_test::run_nurbssurface_bounding_box);
REGISTER_MINI_TEST!("NurbsSurface", "Domain_operations", crate::nurbssurface_test::run_nurbssurface_domain_operations);
REGISTER_MINI_TEST!("NurbsSurface", "Corner_points", crate::nurbssurface_test::run_nurbssurface_corner_points);
REGISTER_MINI_TEST!("NurbsSurface", "Swap_coordinates", crate::nurbssurface_test::run_nurbssurface_swap_coordinates);
REGISTER_MINI_TEST!("NurbsSurface", "Zero_cvs", crate::nurbssurface_test::run_nurbssurface_zero_cvs);
REGISTER_MINI_TEST!("NurbsSurface", "Get_knots", crate::nurbssurface_test::run_nurbssurface_get_knots);
REGISTER_MINI_TEST!("NurbsSurface", "Make_non_rational", crate::nurbssurface_test::run_nurbssurface_make_non_rational);
REGISTER_MINI_TEST!("NurbsSurface", "Create_clamped_uniform", crate::nurbssurface_test::run_nurbssurface_create_clamped_uniform);
REGISTER_MINI_TEST!("NurbsSurface", "Knot_multiplicity", crate::nurbssurface_test::run_nurbssurface_knot_multiplicity);
REGISTER_MINI_TEST!("NurbsSurface", "Sphere", crate::nurbssurface_test::run_nurbssurface_sphere);
REGISTER_MINI_TEST!("NurbsSurface", "Cylinder", crate::nurbssurface_test::run_nurbssurface_cylinder);
REGISTER_MINI_TEST!("NurbsSurface", "Torus", crate::nurbssurface_test::run_nurbssurface_torus);
REGISTER_MINI_TEST!("NurbsSurface", "Cone", crate::nurbssurface_test::run_nurbssurface_cone);
