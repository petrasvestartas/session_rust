use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_brep_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::brep::BRep;
        use crate::Point;

        let b = BRep::new();

        let sstr = b.str();
        let srepr = b.repr();

        let bcopy = b.duplicate();

        MINI_CHECK!(!b.is_valid());
        MINI_CHECK!(b.face_count() == 0);
        MINI_CHECK!(b.name == "my_brep");
        MINI_CHECK!(!b.guid.is_empty());
        MINI_CHECK!(sstr.contains("BRep"));
        MINI_CHECK!(srepr.contains("name=my_brep"));
        MINI_CHECK!(bcopy.guid != b.guid);
        MINI_CHECK!(bcopy == b);
        MINI_CHECK!(!(bcopy != b));
    })
}

pub fn run_brep_create_box() -> TestResult {
    MINI_TEST!("Create_box", {
        use crate::brep::BRep;
        use crate::Point;

        let b = BRep::create_box(2.0, 3.0, 4.0);

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 6);
        MINI_CHECK!(b.edge_count() == 12);
        MINI_CHECK!(b.vertex_count() == 8);
        MINI_CHECK!(b.is_solid());
        MINI_CHECK!(b.name == "box");
    })
}

pub fn run_brep_accessors() -> TestResult {
    MINI_TEST!("Accessors", {
        use crate::brep::BRep;
        use crate::Point;
        use crate::NurbsSurface;

        let b = BRep::create_box(2.0, 3.0, 4.0);

        let fc = b.face_count();
        let ec = b.edge_count();
        let vc = b.vertex_count();

        MINI_CHECK!(fc == 6);
        MINI_CHECK!(ec == 12);
        MINI_CHECK!(vc == 8);
        MINI_CHECK!(b.m_surfaces.len() == 6);
        MINI_CHECK!(b.m_loops.len() == 6);
        MINI_CHECK!(b.m_trims.len() == 24);
    })
}

pub fn run_brep_add_face() -> TestResult {
    MINI_TEST!("Add_face", {
        use crate::brep::{BRep, BRepLoopType, BRepTrimType};
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;

        let mut b = BRep::new();
        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0)); srf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0)); srf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        let si = b.add_surface(&srf);
        let fi = b.add_face(si as i32, false);
        let li = b.add_loop(fi as i32, BRepLoopType::Outer);

        let trim = NurbsCurve::create(false, 1, &[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)]);
        let ci = b.add_curve_2d(&trim);
        b.add_trim(ci as i32, -1, li as i32, false, BRepTrimType::Boundary);

        MINI_CHECK!(b.face_count() == 1);
        MINI_CHECK!(b.m_surfaces.len() == 1);
        MINI_CHECK!(b.m_loops.len() == 1);
        MINI_CHECK!(b.m_trims.len() == 1);
    })
}

pub fn run_brep_mesh() -> TestResult {
    MINI_TEST!("Mesh", {
        use crate::brep::BRep;
        use crate::Mesh;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let m = b.mesh();

        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_brep_point_at() -> TestResult {
    MINI_TEST!("Point_at", {
        use crate::brep::BRep;
        use crate::Point;

        let b = BRep::create_box(2.0, 3.0, 4.0);
        let pt = b.point_at(0, 0.5, 0.5);

        MINI_CHECK!((pt[2] + 2.0).abs() < 0.01 || (pt[2] - 2.0).abs() < 0.01
                 || (pt[1] + 1.5).abs() < 0.01 || (pt[1] - 1.5).abs() < 0.01
                 || (pt[0] + 1.0).abs() < 0.01 || (pt[0] - 1.0).abs() < 0.01);
    })
}

pub fn run_brep_is_solid() -> TestResult {
    MINI_TEST!("Is_solid", {
        use crate::brep::BRep;
        use crate::NurbsSurface;
        use crate::NurbsCurve;
        use crate::Point;

        let b = BRep::create_box(2.0, 3.0, 4.0);

        let mut single = BRep::new();
        let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0)); srf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0)); srf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));
        let si = single.add_surface(&srf);
        single.add_face(si as i32, false);
        single.add_vertex(&Point::new(0.0, 0.0, 0.0));

        MINI_CHECK!(b.is_solid());
        MINI_CHECK!(!single.is_solid());
    })
}

pub fn run_brep_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::brep::BRep;
        use crate::Point;
        use crate::Xform;

        let mut b = BRep::create_box(2.0, 3.0, 4.0);
        b.xform = Xform::translation(10.0, 20.0, 30.0);
        let moved = b.transformed();

        let pt = moved.point_at(0, 0.0, 0.0);
        let pt_orig = b.point_at(0, 0.0, 0.0);

        MINI_CHECK!((pt[0] - pt_orig[0] - 10.0).abs() < 0.01);
        MINI_CHECK!((pt[1] - pt_orig[1] - 20.0).abs() < 0.01);
        MINI_CHECK!((pt[2] - pt_orig[2] - 30.0).abs() < 0.01);
    })
}

pub fn run_brep_json_roundtrip() -> TestResult {
    MINI_TEST!("Json_roundtrip", {
        use crate::brep::BRep;
        use crate::Color;
        use std::path::PathBuf;

        let mut b = BRep::create_box(2.0, 3.0, 4.0);
        b.name = "test_brep".to_string();
        b.width = 2.0;
        b.surfacecolor = Color::new(255, 128, 64, 255);

        // String
        let json_string = b.json_dumps();
        let loaded_json_string = BRep::json_loads(&json_string);

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_brep.json");
        b.json_dump(filename.to_str().unwrap());
        let loaded_from_file = BRep::json_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_json_string == b);
        MINI_CHECK!(loaded_from_file == b);
    })
}

pub fn run_brep_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf_roundtrip", {
        use crate::brep::BRep;
        use crate::Color;
        use std::path::PathBuf;

        let mut b = BRep::create_box(2.0, 3.0, 4.0);
        b.name = "test_brep".to_string();
        b.width = 2.0;
        b.surfacecolor = Color::new(255, 128, 64, 255);

        // String
        let proto_data = b.pb_dumps();
        let loaded_proto = BRep::pb_loads(&proto_data).unwrap();

        // File
        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir.join("serialization").join("test_brep.bin");
        b.pb_dump(filename.to_str().unwrap());
        let loaded = BRep::pb_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_proto == b);
        MINI_CHECK!(loaded == b);
    })
}

pub fn run_brep_create_cylinder() -> TestResult {
    MINI_TEST!("Create_cylinder", {
        use crate::brep::BRep;
        use crate::Mesh;

        let cyl = BRep::create_cylinder(1.0, 2.0);
        let m = cyl.mesh();

        MINI_CHECK!(cyl.is_valid());
        MINI_CHECK!(cyl.face_count() == 3);
        MINI_CHECK!(cyl.is_solid());
        MINI_CHECK!(cyl.name == "cylinder");
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_brep_create_sphere() -> TestResult {
    MINI_TEST!("Create_sphere", {
        use crate::brep::BRep;
        use crate::Mesh;

        let sph = BRep::create_sphere(2.0);
        let m = sph.mesh();

        MINI_CHECK!(sph.is_valid());
        MINI_CHECK!(sph.face_count() == 1);
        MINI_CHECK!(sph.is_solid());
        MINI_CHECK!(sph.name == "sphere");
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_brep_from_polylines() -> TestResult {
    MINI_TEST!("From_polylines", {
        use crate::brep::BRep;
        use crate::polyline::Polyline;
        use crate::Mesh;
        use crate::Point;

        let hx = 1.0_f64; let hy = 1.5_f64; let hz = 2.0_f64;
        let c = [
            Point::new(-hx,-hy,-hz), Point::new( hx,-hy,-hz),
            Point::new( hx, hy,-hz), Point::new(-hx, hy,-hz),
            Point::new(-hx,-hy, hz), Point::new( hx,-hy, hz),
            Point::new( hx, hy, hz), Point::new(-hx, hy, hz),
        ];

        let bottom = Polyline::new(vec![c[0].clone(),c[3].clone(),c[2].clone(),c[1].clone(),c[0].clone()]);
        let top = Polyline::new(vec![c[4].clone(),c[5].clone(),c[6].clone(),c[7].clone(),c[4].clone()]);
        let front = Polyline::new(vec![c[0].clone(),c[1].clone(),c[5].clone(),c[4].clone(),c[0].clone()]);
        let right = Polyline::new(vec![c[1].clone(),c[2].clone(),c[6].clone(),c[5].clone(),c[1].clone()]);
        let back = Polyline::new(vec![c[2].clone(),c[3].clone(),c[7].clone(),c[6].clone(),c[2].clone()]);
        let left = Polyline::new(vec![c[3].clone(),c[0].clone(),c[4].clone(),c[7].clone(),c[3].clone()]);

        let b = BRep::from_polylines(&[bottom, top, front, right, back, left]);
        let m = b.mesh();

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 6);
        MINI_CHECK!(b.edge_count() == 12);
        MINI_CHECK!(b.vertex_count() == 8);
        MINI_CHECK!(b.is_solid());
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_brep_from_nurbscurves() -> TestResult {
    MINI_TEST!("From_nurbscurves", {
        use crate::brep::BRep;
        use crate::NurbsCurve;
        use crate::Mesh;
        use crate::Point;

        let hx = 1.0_f64; let hy = 1.5_f64; let hz = 2.0_f64;
        let c = [
            Point::new(-hx,-hy,-hz), Point::new( hx,-hy,-hz),
            Point::new( hx, hy,-hz), Point::new(-hx, hy,-hz),
            Point::new(-hx,-hy, hz), Point::new( hx,-hy, hz),
            Point::new( hx, hy, hz), Point::new(-hx, hy, hz),
        ];

        let bottom = NurbsCurve::create(false, 1, &[c[0].clone(),c[3].clone(),c[2].clone(),c[1].clone(),c[0].clone()]);
        let top = NurbsCurve::create(false, 1, &[c[4].clone(),c[5].clone(),c[6].clone(),c[7].clone(),c[4].clone()]);
        let front = NurbsCurve::create(false, 1, &[c[0].clone(),c[1].clone(),c[5].clone(),c[4].clone(),c[0].clone()]);
        let right = NurbsCurve::create(false, 1, &[c[1].clone(),c[2].clone(),c[6].clone(),c[5].clone(),c[1].clone()]);
        let back = NurbsCurve::create(false, 1, &[c[2].clone(),c[3].clone(),c[7].clone(),c[6].clone(),c[2].clone()]);
        let left = NurbsCurve::create(false, 1, &[c[3].clone(),c[0].clone(),c[4].clone(),c[7].clone(),c[3].clone()]);

        let b = BRep::from_nurbscurves(&[bottom, top, front, right, back, left], &[]);
        let m = b.mesh();

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 6);
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_brep_from_nurbscurves_holes() -> TestResult {
    MINI_TEST!("From_nurbscurves_holes", {
        use crate::brep::BRep;
        use crate::brep::BRepLoopType;
        use crate::NurbsCurve;
        use crate::Mesh;
        use crate::Point;
        use crate::primitives::Primitives;

        let outer = NurbsCurve::create(false, 1, &[Point::new(-5.0,-5.0,0.0), Point::new(5.0,-5.0,0.0), Point::new(5.0,5.0,0.0), Point::new(-5.0,5.0,0.0), Point::new(-5.0,-5.0,0.0)]);
        let hole = Primitives::circle(0.0, 0.0, 0.0, 2.0);

        let b = BRep::from_nurbscurves(&[outer], &[vec![hole]]);
        let m = b.mesh();

        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(b.face_count() == 1);
        MINI_CHECK!(b.m_loops.len() == 2);
        MINI_CHECK!(b.m_loops[0].loop_type == BRepLoopType::Outer);
        MINI_CHECK!(b.m_loops[1].loop_type == BRepLoopType::Inner);
        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

REGISTER_MINI_TEST!("BRep", "Constructor", crate::brep_test::run_brep_constructor);
REGISTER_MINI_TEST!("BRep", "Create_box", crate::brep_test::run_brep_create_box);
REGISTER_MINI_TEST!("BRep", "Accessors", crate::brep_test::run_brep_accessors);
REGISTER_MINI_TEST!("BRep", "Add_face", crate::brep_test::run_brep_add_face);
REGISTER_MINI_TEST!("BRep", "Mesh", crate::brep_test::run_brep_mesh);
REGISTER_MINI_TEST!("BRep", "Point_at", crate::brep_test::run_brep_point_at);
REGISTER_MINI_TEST!("BRep", "Is_solid", crate::brep_test::run_brep_is_solid);
REGISTER_MINI_TEST!("BRep", "Transformation", crate::brep_test::run_brep_transformation);
REGISTER_MINI_TEST!("BRep", "Json_roundtrip", crate::brep_test::run_brep_json_roundtrip);
REGISTER_MINI_TEST!("BRep", "Create_cylinder", crate::brep_test::run_brep_create_cylinder);
REGISTER_MINI_TEST!("BRep", "Create_sphere", crate::brep_test::run_brep_create_sphere);
REGISTER_MINI_TEST!("BRep", "From_polylines", crate::brep_test::run_brep_from_polylines);
REGISTER_MINI_TEST!("BRep", "From_nurbscurves", crate::brep_test::run_brep_from_nurbscurves);
REGISTER_MINI_TEST!("BRep", "From_nurbscurves_holes", crate::brep_test::run_brep_from_nurbscurves_holes);
REGISTER_MINI_TEST!("BRep", "Protobuf_roundtrip", crate::brep_test::run_brep_protobuf_roundtrip);
