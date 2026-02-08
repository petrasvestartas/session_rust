use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_trimmedsurface_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;

        // Create a bilinear surface
        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(6.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 6.0, 0.0));
        srf.set_cv(1, 1, &Point::new(6.0, 6.0, 0.0));

        // Outer trim loop (rectangle in UV space)
        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.1, 0.1, 0.0), Point::new(0.9, 0.1, 0.0),
            Point::new(0.9, 0.9, 0.0), Point::new(0.1, 0.9, 0.0),
        ]);

        let ts = TrimmedSurface::create(&srf, &outer);

        // String representations
        let sstr = ts.to_string();
        let srepr = ts.repr();

        // Copy (new guid)
        let tscopy = ts.duplicate();

        MINI_CHECK!(ts.is_valid());
        MINI_CHECK!(ts.is_trimmed());
        MINI_CHECK!(ts.name == "my_trimmedsurface");
        MINI_CHECK!(!ts.guid.is_empty());
        MINI_CHECK!(sstr.contains("TrimmedSurface"));
        MINI_CHECK!(srepr.contains("name=my_trimmedsurface"));
        MINI_CHECK!(tscopy.is_valid());
        MINI_CHECK!(tscopy.guid != ts.guid);
        MINI_CHECK!(tscopy == ts);
    })
}

pub fn run_trimmedsurface_constructor_planar() -> TestResult {
    MINI_TEST!("constructor_planar", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::tolerance::PI;

        // Planar curve boundary
        let pts = vec![Point::new(0.0,0.0,0.0), Point::new(3.0,1.0,0.0), Point::new(5.0,0.5,0.0), Point::new(6.0,3.0,0.0), Point::new(4.0,5.0,0.0), Point::new(1.0,4.0,0.0)];
        let bnd = NurbsCurve::create(true, 3, &pts);
        let _ts = TrimmedSurface::create_planar(&bnd).unwrap();

        // Rotated planar
        let pts = vec![Point::new(0.0,0.0,0.0), Point::new(3.0,1.0,-2.0), Point::new(5.0,2.0,-3.0), Point::new(4.0,4.0,0.0), Point::new(1.0,3.0,2.0)];
        let bnd = NurbsCurve::create(true, 3, &pts);
        let _ts = TrimmedSurface::create_planar(&bnd).unwrap();

        // Triangle
        let bnd = NurbsCurve::create(true, 1, &[Point::new(0.0,0.0,0.0), Point::new(6.0,3.0,3.0), Point::new(2.0,5.0,1.0)]);
        let _ts = TrimmedSurface::create_planar(&bnd).unwrap();

        // Trapezoid
        let bnd = NurbsCurve::create(true, 1, &[Point::new(0.0,0.0,6.0), Point::new(5.0,0.0,6.0), Point::new(4.0,4.0,2.0), Point::new(1.0,4.0,2.0)]);
        let _ts = TrimmedSurface::create_planar(&bnd).unwrap();

        // Rectangle with a hole
        let bnd = NurbsCurve::create(true, 1, &[Point::new(0.0,0.0,0.0), Point::new(6.0,0.0,0.0), Point::new(6.0,6.0,0.0), Point::new(0.0,6.0,0.0)]);
        let mut ts = TrimmedSurface::create_planar(&bnd).unwrap();
        ts.add_hole(&NurbsCurve::create(true, 1, &[Point::new(2.0,2.0,0.0), Point::new(4.0,2.0,0.0), Point::new(4.0,4.0,0.0), Point::new(2.0,4.0,0.0)]));

        // Hexagon with 2 holes
        let r = 4.0f64;
        let mut pts = Vec::new();
        for k in 0..6 {
            let a = k as f64 * PI / 3.0;
            pts.push(Point::new(r * a.cos(), r * a.sin(), r * a.cos() * 0.5));
        }
        let bnd = NurbsCurve::create(true, 1, &pts);
        let mut ts = TrimmedSurface::create_planar(&bnd).unwrap();
        ts.add_holes(&[
            NurbsCurve::create(true, 1, &[Point::new(1.5,0.5,0.75), Point::new(2.5,0.5,1.25), Point::new(2.0,1.5,1.0)]),
            NurbsCurve::create(true, 1, &[Point::new(-2.0,-0.5,-1.0), Point::new(-1.0,-0.5,-0.5), Point::new(-1.0,-1.5,-0.5), Point::new(-2.0,-1.5,-1.0)]),
        ]);
    })
}

pub fn run_trimmedsurface_constructor_hole() -> TestResult {
    MINI_TEST!("constructor_hole", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;

        // Create surface with bump
        let n = 8usize;
        let mut pts = Vec::new();
        for i in 0..n {
            for j in 0..n {
                let x = i as f64;
                let y = j as f64;
                let r2 = (x - 1.5) * (x - 1.5) + (y - 1.5) * (y - 1.5);
                let z = 5.0 * (-r2 / 1.0).exp() + 0.3 * (crate::tolerance::PI * x / 7.0).sin() * (crate::tolerance::PI * y / 7.0).sin();
                pts.push(Point::new(x, y, z));
            }
        }

        let srf = NurbsSurface::create(false, false, 3, 3, n, n, &pts).unwrap();

        // Create outer loop (full boundary in UV)
        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);

        let mut ts = TrimmedSurface::create(&srf, &outer);

        // Add hole as UV curve directly
        let hole = NurbsCurve::create(true, 1, &[
            Point::new(0.4, 0.4, 0.0), Point::new(0.6, 0.4, 0.0),
            Point::new(0.6, 0.6, 0.0), Point::new(0.4, 0.6, 0.0),
        ]);
        ts.add_inner_loop(hole);

        MINI_CHECK!(ts.is_valid());
        MINI_CHECK!(ts.is_trimmed());
        MINI_CHECK!(ts.inner_loop_count() == 1);
    })
}

pub fn run_trimmedsurface_accessors() -> TestResult {
    MINI_TEST!("accessors", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;

        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(5.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 5.0, 0.0));
        srf.set_cv(1, 1, &Point::new(5.0, 5.0, 0.0));

        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.1, 0.1, 0.0), Point::new(0.9, 0.1, 0.0),
            Point::new(0.9, 0.9, 0.0), Point::new(0.1, 0.9, 0.0),
        ]);

        let mut ts = TrimmedSurface::create(&srf, &outer);
        ts.name = "test_accessors".to_string();
        ts.width = 2.5;

        let got_srf = ts.surface();
        let got_loop = ts.get_outer_loop();

        MINI_CHECK!(ts.is_valid());
        MINI_CHECK!(ts.is_trimmed());
        MINI_CHECK!(ts.name == "test_accessors");
        MINI_CHECK!(ts.width == 2.5);
        MINI_CHECK!(got_srf.is_valid());
        MINI_CHECK!(got_loop.is_some());
        MINI_CHECK!(ts.inner_loop_count() == 0);
    })
}

pub fn run_trimmedsurface_add_inner_loop() -> TestResult {
    MINI_TEST!("add_inner_loop", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;

        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(10.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 10.0, 0.0));
        srf.set_cv(1, 1, &Point::new(10.0, 10.0, 0.0));

        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);

        let mut ts = TrimmedSurface::create(&srf, &outer);

        // Add inner loops (holes in UV)
        let hole1 = NurbsCurve::create(true, 1, &[
            Point::new(0.2, 0.2, 0.0), Point::new(0.4, 0.2, 0.0),
            Point::new(0.4, 0.4, 0.0), Point::new(0.2, 0.4, 0.0),
        ]);
        let hole2 = NurbsCurve::create(true, 1, &[
            Point::new(0.6, 0.6, 0.0), Point::new(0.8, 0.6, 0.0),
            Point::new(0.8, 0.8, 0.0), Point::new(0.6, 0.8, 0.0),
        ]);

        ts.add_inner_loop(hole1);
        ts.add_inner_loop(hole2);

        let got = ts.get_inner_loop(0);

        MINI_CHECK!(ts.inner_loop_count() == 2);
        MINI_CHECK!(got.is_some());

        ts.clear_inner_loops();
        MINI_CHECK!(ts.inner_loop_count() == 0);
    })
}

pub fn run_trimmedsurface_point_at() -> TestResult {
    MINI_TEST!("point_at", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::tolerance::TOLERANCE;

        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(4.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 4.0, 0.0));
        srf.set_cv(1, 1, &Point::new(4.0, 4.0, 0.0));

        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);

        let ts = TrimmedSurface::create(&srf, &outer);

        let (u0, u1) = ts.surface().domain(0).unwrap();
        let (v0, v1) = ts.surface().domain(1).unwrap();
        let u_mid = (u0 + u1) / 2.0;
        let v_mid = (v0 + v1) / 2.0;

        let pt = ts.point_at(u_mid, v_mid).unwrap();
        let nm = ts.normal_at(u_mid, v_mid);

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(nm[2].abs(), 1.0));
    })
}

pub fn run_trimmedsurface_mesh() -> TestResult {
    MINI_TEST!("mesh", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;

        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(6.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 6.0, 0.0));
        srf.set_cv(1, 1, &Point::new(6.0, 6.0, 0.0));

        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.05, 0.05, 0.0), Point::new(0.95, 0.05, 0.0),
            Point::new(0.95, 0.95, 0.0), Point::new(0.05, 0.95, 0.0),
        ]);

        let ts = TrimmedSurface::create(&srf, &outer);
        let m = ts.mesh();

        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_trimmedsurface_transformation() -> TestResult {
    MINI_TEST!("transformation", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Xform;
        use crate::tolerance::TOLERANCE;

        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        srf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0), Point::new(0.0, 1.0, 0.0),
        ]);

        let mut ts = TrimmedSurface::create(&srf, &outer);
        ts.xform = Xform::translation(10.0, 20.0, 30.0);
        let ts2 = ts.transformed();

        let (u0, _u1) = ts2.surface().domain(0).unwrap();
        let (v0, _v1) = ts2.surface().domain(1).unwrap();
        let pt = ts2.point_at(u0, v0).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 20.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 30.0));
    })
}

pub fn run_trimmedsurface_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Color;
        use std::path::PathBuf;

        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(5.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 5.0, 0.0));
        srf.set_cv(1, 1, &Point::new(5.0, 5.0, 0.0));

        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.1, 0.1, 0.0), Point::new(0.9, 0.1, 0.0),
            Point::new(0.9, 0.9, 0.0), Point::new(0.1, 0.9, 0.0),
        ]);

        let mut ts = TrimmedSurface::create(&srf, &outer);
        ts.name = "test_trimmedsurface".to_string();
        ts.width = 2.0;
        ts.surfacecolor = Color::new(255, 128, 64, 255);

        // JSON object
        let json = ts.jsondump().unwrap();
        let loaded_json = TrimmedSurface::jsonload(&json).unwrap();

        // String
        let json_string = ts.json_dumps();
        let loaded_json_string = TrimmedSurface::json_loads(&json_string);

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_trimmedsurface.json");
        ts.json_dump(filename.to_str().unwrap());
        let loaded_from_file = TrimmedSurface::json_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_json == ts);
        MINI_CHECK!(loaded_json_string == ts);
        MINI_CHECK!(loaded_from_file == ts);
    })
}

pub fn run_trimmedsurface_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::trimmedsurface::TrimmedSurface;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;
        use crate::Color;
        use std::path::PathBuf;

        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(5.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 5.0, 0.0));
        srf.set_cv(1, 1, &Point::new(5.0, 5.0, 0.0));

        let outer = NurbsCurve::create(true, 1, &[
            Point::new(0.1, 0.1, 0.0), Point::new(0.9, 0.1, 0.0),
            Point::new(0.9, 0.9, 0.0), Point::new(0.1, 0.9, 0.0),
        ]);

        let mut ts = TrimmedSurface::create(&srf, &outer);
        ts.name = "test_trimmedsurface".to_string();
        ts.width = 2.0;
        ts.surfacecolor = Color::new(255, 128, 64, 255);

        // String
        let proto_string = ts.pb_dumps();
        let loaded_proto_string = TrimmedSurface::pb_loads(&proto_string).unwrap();

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_trimmedsurface.bin");
        ts.pb_dump(filename.to_str().unwrap());
        let loaded = TrimmedSurface::pb_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_proto_string == ts);
        MINI_CHECK!(loaded == ts);
    })
}

// Register tests with the shared registry
REGISTER_MINI_TEST!("TrimmedSurface", "constructor", crate::trimmedsurface_test::run_trimmedsurface_constructor);
REGISTER_MINI_TEST!("TrimmedSurface", "constructor_planar", crate::trimmedsurface_test::run_trimmedsurface_constructor_planar);
REGISTER_MINI_TEST!("TrimmedSurface", "constructor_hole", crate::trimmedsurface_test::run_trimmedsurface_constructor_hole);
REGISTER_MINI_TEST!("TrimmedSurface", "accessors", crate::trimmedsurface_test::run_trimmedsurface_accessors);
REGISTER_MINI_TEST!("TrimmedSurface", "add_inner_loop", crate::trimmedsurface_test::run_trimmedsurface_add_inner_loop);
REGISTER_MINI_TEST!("TrimmedSurface", "point_at", crate::trimmedsurface_test::run_trimmedsurface_point_at);
REGISTER_MINI_TEST!("TrimmedSurface", "mesh", crate::trimmedsurface_test::run_trimmedsurface_mesh);
REGISTER_MINI_TEST!("TrimmedSurface", "transformation", crate::trimmedsurface_test::run_trimmedsurface_transformation);
REGISTER_MINI_TEST!("TrimmedSurface", "json_roundtrip", crate::trimmedsurface_test::run_trimmedsurface_json_roundtrip);
REGISTER_MINI_TEST!("TrimmedSurface", "protobuf_roundtrip", crate::trimmedsurface_test::run_trimmedsurface_protobuf_roundtrip);
