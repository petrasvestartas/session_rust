#![allow(clippy::needless_range_loop)]
use crate::mini_test::TestResult;
use crate::tolerance::PI;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

/// Path of a file in the serialization folder, created when missing.
fn serialization_path(name: &str) -> String {
    let folder = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("serialization");
    let _ = std::fs::create_dir_all(&folder);

    folder.join(name).to_string_lossy().into_owned()
}

pub fn run_file_step_nurbscurve_round_trip() -> TestResult {
    MINI_TEST!("NurbsCurve Round Trip", {
        use crate::file_step;
        use crate::NurbsCurve;
        use crate::Point;

        let path = serialization_path("test_step_nurbscurve.step");
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ];
        let nc = NurbsCurve::create(false, 3, &pts);

        MINI_CHECK!(nc.is_valid());
        MINI_CHECK!(nc.degree() == 3);
        MINI_CHECK!(nc.cv_count() == 4);

        file_step::write_file_step_nurbscurves(std::slice::from_ref(&nc), &path);

        MINI_CHECK!(std::path::Path::new(&path).exists());

        let curves = file_step::read_file_step_nurbscurves(&path);

        MINI_CHECK!(!curves.is_empty());

        let back = &curves[0];

        MINI_CHECK!(back.is_valid());
        MINI_CHECK!(back.degree() == 3);
        MINI_CHECK!(back.cv_count() == 4);
        MINI_CHECK!(!back.m_is_rat);

        let kn_orig = nc.get_nurbsknots();
        let kn_back = back.get_nurbsknots();

        MINI_CHECK!(kn_orig.len() == kn_back.len());

        for i in 0..kn_orig.len().min(kn_back.len()) {
            MINI_CHECK!((kn_orig[i] - kn_back[i]).abs() < 1e-10);
        }

        for i in 0..4 {
            let p_orig = nc.get_cv(i).unwrap_or_default();
            let p_back = back.get_cv(i).unwrap_or_default();

            MINI_CHECK!((p_orig[0] - p_back[0]).abs() < 1e-10);
            MINI_CHECK!((p_orig[1] - p_back[1]).abs() < 1e-10);
            MINI_CHECK!((p_orig[2] - p_back[2]).abs() < 1e-10);
        }

        let _ = std::fs::remove_file(&path);
    })
}

pub fn run_file_step_nurbscurve_rational_round_trip() -> TestResult {
    MINI_TEST!("NurbsCurve Rational Round Trip", {
        use crate::file_step;
        use crate::NurbsCurve;

        let path = serialization_path("test_step_nurbscurve_rat.step");
        let mut nc = NurbsCurve::new(3, true, 3, 3);
        let w_mid = (PI / 4.0).cos();
        nc.m_nurbsknot = vec![0.0, 0.0, 1.0, 1.0];
        nc.m_cv[0] = 1.0;
        nc.m_cv[1] = 0.0;
        nc.m_cv[2] = 0.0;
        nc.m_cv[3] = 1.0;
        nc.m_cv[4] = w_mid * 1.0;
        nc.m_cv[5] = w_mid * 1.0;
        nc.m_cv[6] = 0.0;
        nc.m_cv[7] = w_mid;
        nc.m_cv[8] = 0.0;
        nc.m_cv[9] = 1.0;
        nc.m_cv[10] = 0.0;
        nc.m_cv[11] = 1.0;

        MINI_CHECK!(nc.is_valid());
        MINI_CHECK!(nc.degree() == 2);
        MINI_CHECK!(nc.cv_count() == 3);
        MINI_CHECK!(nc.m_is_rat);

        file_step::write_file_step_nurbscurves(std::slice::from_ref(&nc), &path);

        MINI_CHECK!(std::path::Path::new(&path).exists());

        let curves = file_step::read_file_step_nurbscurves(&path);

        MINI_CHECK!(!curves.is_empty());

        let back = &curves[0];

        MINI_CHECK!(back.is_valid());
        MINI_CHECK!(back.degree() == 2);
        MINI_CHECK!(back.cv_count() == 3);
        MINI_CHECK!(back.m_is_rat);

        let cv = &nc.m_cv;
        let cv_back = &back.m_cv;
        let s = back.m_cv_stride;

        for i in 0..3 {
            let w_orig = cv[i * 4 + 3];
            let w_back = cv_back[i * s + 3];

            MINI_CHECK!((w_orig - w_back).abs() < 1e-10);

            if w_orig.abs() > 1e-12 && w_back.abs() > 1e-12 {
                MINI_CHECK!((cv[i * 4] / w_orig - cv_back[i * s] / w_back).abs() < 1e-10);
                MINI_CHECK!((cv[i * 4 + 1] / w_orig - cv_back[i * s + 1] / w_back).abs() < 1e-10);
            }
        }

        let _ = std::fs::remove_file(&path);
    })
}

pub fn run_file_step_nurbssurface_round_trip() -> TestResult {
    MINI_TEST!("NurbsSurface Round Trip", {
        use crate::file_step;
        use crate::NurbsSurface;
        use crate::Point;

        let path = serialization_path("test_step_nurbssurface.step");
        let mut pts = Vec::new();

        for u in 0..4 {
            for v in 0..4 {
                pts.push(Point::new(u as f64, v as f64, ((u + v) as f64).sin() * 0.5));
            }
        }

        let srf = NurbsSurface::create(false, false, 3, 3, 4, 4, &pts).unwrap_or_default();

        MINI_CHECK!(srf.is_valid());
        MINI_CHECK!(srf.degree(0) == 3);
        MINI_CHECK!(srf.degree(1) == 3);
        MINI_CHECK!(srf.cv_count(0) == 4);
        MINI_CHECK!(srf.cv_count(1) == 4);

        file_step::write_file_step_nurbssurfaces(std::slice::from_ref(&srf), &path);

        MINI_CHECK!(std::path::Path::new(&path).exists());

        let surfaces = file_step::read_file_step_nurbssurfaces(&path);

        MINI_CHECK!(!surfaces.is_empty());

        let back = &surfaces[0];

        MINI_CHECK!(back.is_valid());
        MINI_CHECK!(back.degree(0) == 3);
        MINI_CHECK!(back.degree(1) == 3);
        MINI_CHECK!(back.cv_count(0) == 4);
        MINI_CHECK!(back.cv_count(1) == 4);
        MINI_CHECK!(!back.m_is_rat);

        let ku_orig = &srf.m_nurbsknot[0];
        let kv_orig = &srf.m_nurbsknot[1];
        let ku_back = &back.m_nurbsknot[0];
        let kv_back = &back.m_nurbsknot[1];

        MINI_CHECK!(ku_orig.len() == ku_back.len());
        MINI_CHECK!(kv_orig.len() == kv_back.len());

        for i in 0..ku_orig.len().min(ku_back.len()) {
            MINI_CHECK!((ku_orig[i] - ku_back[i]).abs() < 1e-10);
        }

        for u in 0..4 {
            for v in 0..4 {
                let p_orig = srf.get_cv(u, v).unwrap_or_default();
                let p_back = back.get_cv(u, v).unwrap_or_default();

                MINI_CHECK!((p_orig[0] - p_back[0]).abs() < 1e-10);
                MINI_CHECK!((p_orig[1] - p_back[1]).abs() < 1e-10);
                MINI_CHECK!((p_orig[2] - p_back[2]).abs() < 1e-10);
            }
        }

        let _ = std::fs::remove_file(&path);
    })
}

pub fn run_file_step_nurbssurface_rational_round_trip() -> TestResult {
    MINI_TEST!("NurbsSurface Rational Round Trip", {
        use crate::file_step;
        use crate::NurbsSurface;

        let path = serialization_path("test_step_nurbssurface_rat.step");
        let mut srf = NurbsSurface::new(3, true, 3, 3, 3, 3);
        srf.m_nurbsknot[0] = vec![0.0, 0.0, 1.0, 1.0];
        srf.m_nurbsknot[1] = vec![0.0, 0.0, 1.0, 1.0];

        let w = 0.8;

        for u in 0..3 {
            for v in 0..3 {
                let x = u as f64;
                let y = v as f64;
                let z = ((u + v) as f64).sin() * 0.3;
                srf.set_cv_4d(u, v, w * x, w * y, w * z, w);
            }
        }

        MINI_CHECK!(srf.is_valid());
        MINI_CHECK!(srf.m_is_rat);

        file_step::write_file_step_nurbssurfaces(std::slice::from_ref(&srf), &path);

        MINI_CHECK!(std::path::Path::new(&path).exists());

        let surfaces = file_step::read_file_step_nurbssurfaces(&path);

        MINI_CHECK!(!surfaces.is_empty());

        let back = &surfaces[0];

        MINI_CHECK!(back.is_valid());
        MINI_CHECK!(back.degree(0) == 2);
        MINI_CHECK!(back.degree(1) == 2);
        MINI_CHECK!(back.cv_count(0) == 3);
        MINI_CHECK!(back.cv_count(1) == 3);
        MINI_CHECK!(back.m_is_rat);

        for u in 0..3 {
            for v in 0..3 {
                let (x1, y1, _z1, w1) = srf.get_cv_4d(u, v).unwrap_or_default();
                let (x2, y2, _z2, w2) = back.get_cv_4d(u, v).unwrap_or_default();

                MINI_CHECK!((w1 - w2).abs() < 1e-10);

                if w1.abs() > 1e-12 && w2.abs() > 1e-12 {
                    MINI_CHECK!((x1 / w1 - x2 / w2).abs() < 1e-10);
                    MINI_CHECK!((y1 / w1 - y2 / w2).abs() < 1e-10);
                }
            }
        }

        let _ = std::fs::remove_file(&path);
    })
}

pub fn run_file_step_nurbssurface_trimmed_round_trip() -> TestResult {
    MINI_TEST!("NurbsSurfaceTrimmed Round Trip", {
        use crate::file_step;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;

        let path = serialization_path("test_step_nurbssurface_trimmed.step");
        let mut pts = Vec::new();

        for u in 0..4 {
            for v in 0..4 {
                pts.push(Point::new(u as f64, v as f64, 0.0));
            }
        }

        let srf = NurbsSurface::create(false, false, 3, 3, 4, 4, &pts).unwrap_or_default();
        let loop_pts = [
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ];
        let mut outer = NurbsCurve::new(2, false, 2, 5);
        outer.m_nurbsknot = vec![0.0, 1.0, 2.0, 3.0, 4.0];

        for i in 0..5 {
            outer.m_cv[i * 2] = loop_pts[i][0];
            outer.m_cv[i * 2 + 1] = loop_pts[i][1];
        }

        MINI_CHECK!(outer.is_valid());

        let trimmed = NurbsSurfaceTrimmed::create(&srf, &outer);

        MINI_CHECK!(trimmed.m_surface.is_valid());

        file_step::write_file_step_nurbssurfaces_trimmed(std::slice::from_ref(&trimmed), &path);

        MINI_CHECK!(std::path::Path::new(&path).exists());

        let surfaces = file_step::read_file_step_nurbssurfaces(&path);

        MINI_CHECK!(!surfaces.is_empty());

        let back_srf = &surfaces[0];

        MINI_CHECK!(back_srf.is_valid());
        MINI_CHECK!(back_srf.degree(0) == 3);
        MINI_CHECK!(back_srf.degree(1) == 3);
        MINI_CHECK!(back_srf.cv_count(0) == 4);
        MINI_CHECK!(back_srf.cv_count(1) == 4);

        let ncurves = file_step::read_file_step_nurbscurves(&path);

        MINI_CHECK!(!ncurves.is_empty());

        let _ = std::fs::remove_file(&path);
    })
}

pub fn run_file_step_brep_read_schoring() -> TestResult {
    MINI_TEST!("BRep Read Schoring", {
        use crate::file_step;
        use std::path::PathBuf;

        let step_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("session_data")
            .join("elements")
            .join("schoring_foot_0.step");

        if !step_path.exists() {
            return Ok(());
        }

        let breps = file_step::read_file_step_breps(&step_path.to_string_lossy());

        MINI_CHECK!(breps.len() == 3);

        let mut total_faces = 0;
        let mut total_edges = 0;
        let mut total_verts = 0;

        for b in &breps {
            total_faces += b.face_count();
            total_edges += b.edge_count();
            total_verts += b.vertex_count();
        }

        MINI_CHECK!(total_faces == 38);
        MINI_CHECK!(total_edges == 103);
        MINI_CHECK!(total_verts == 74);

        for b in &breps {
            MINI_CHECK!(b.is_valid());
            MINI_CHECK!(b.m_surfaces.len() == b.face_count());
            MINI_CHECK!(b.m_curves_3d.len() == b.edge_count());
            MINI_CHECK!(b.shell_count() == 1 && b.solid_count() == 1);

            for e in &b.m_edges {
                MINI_CHECK!(!e.pcurves.is_empty());
            }
        }

        let pts = file_step::read_file_step_points(&step_path.to_string_lossy());

        MINI_CHECK!(pts.len() == 350);
    })
}

pub fn run_file_step_brep_round_trip() -> TestResult {
    MINI_TEST!("BRep Round Trip", {
        use crate::file_step;
        use crate::BRep;

        let path = serialization_path("test_brep_roundtrip.step");
        let mut cyl = BRep::create_cylinder(1.0, 2.0);
        cyl.name = "cylinder".to_string();

        file_step::write_file_step_brep(&cyl, &path);

        let breps = file_step::read_file_step_breps(&path);

        MINI_CHECK!(breps.len() == 1);
        MINI_CHECK!(breps[0].is_valid());
        MINI_CHECK!(breps[0].face_count() == 3);
        MINI_CHECK!(breps[0].edge_count() == 3);
        MINI_CHECK!(breps[0].vertex_count() == 2);
        MINI_CHECK!(breps[0].is_solid());
        MINI_CHECK!((breps[0].volume() - cyl.volume()).abs() < 0.05 * cyl.volume());

        let _ = std::fs::remove_file(&path);
    })
}

REGISTER_MINI_TEST!(
    "FileStep",
    "NurbsCurve Round Trip",
    crate::file_step_test::run_file_step_nurbscurve_round_trip
);
REGISTER_MINI_TEST!(
    "FileStep",
    "NurbsCurve Rational Round Trip",
    crate::file_step_test::run_file_step_nurbscurve_rational_round_trip
);
REGISTER_MINI_TEST!(
    "FileStep",
    "NurbsSurface Round Trip",
    crate::file_step_test::run_file_step_nurbssurface_round_trip
);
REGISTER_MINI_TEST!(
    "FileStep",
    "NurbsSurface Rational Round Trip",
    crate::file_step_test::run_file_step_nurbssurface_rational_round_trip
);
REGISTER_MINI_TEST!(
    "FileStep",
    "NurbsSurfaceTrimmed Round Trip",
    crate::file_step_test::run_file_step_nurbssurface_trimmed_round_trip
);
REGISTER_MINI_TEST!(
    "FileStep",
    "BRep Read Schoring",
    crate::file_step_test::run_file_step_brep_read_schoring
);
REGISTER_MINI_TEST!(
    "FileStep",
    "BRep Round Trip",
    crate::file_step_test::run_file_step_brep_round_trip
);
