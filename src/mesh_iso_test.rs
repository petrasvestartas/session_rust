use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::tolerance::PI;

pub fn run_mesh_iso_eval_gyroid() -> TestResult {
    MINI_TEST!("Eval Gyroid", {
        use crate::mesh_iso::{MeshIso, TpmsType};

        MINI_CHECK!(TOLERANCE.is_close(MeshIso::eval(TpmsType::GYROID, PI / 2.0, 0.0, 0.0, 1.0), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(MeshIso::eval(TpmsType::SCHWARZ_P, 0.0, 0.0, 0.0, 1.0), 3.0));
    })
}

pub fn run_mesh_iso_eval_schwarz_p() -> TestResult {
    MINI_TEST!("Eval SchwarzP", {
        use crate::mesh_iso::{MeshIso, TpmsType};

        MINI_CHECK!(TOLERANCE.is_close(MeshIso::eval(TpmsType::SCHWARZ_P, PI, PI, PI, 1.0), -3.0));
    })
}

pub fn run_mesh_iso_eval_diamond() -> TestResult {
    MINI_TEST!("Eval Diamond", {
        use crate::mesh_iso::{MeshIso, TpmsType};

        MINI_CHECK!(TOLERANCE.is_close(MeshIso::eval(TpmsType::DIAMOND, 0.0, 0.0, 0.0, 1.0), 0.0));
    })
}

pub fn run_mesh_iso_from_tpms_gyroid_solid() -> TestResult {
    MINI_TEST!("From Tpms Gyroid Solid", {
        use crate::mesh_iso::{MeshIso, TpmsType, TpmsMode};
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)], 0.0);
        let m = MeshIso::from_tpms(TpmsType::GYROID, &box_, 10, 10, 10, 0.0, 1.0, TpmsMode::SOLID, 0.2);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_mesh_iso_from_tpms_diamond_sheet() -> TestResult {
    MINI_TEST!("From Tpms Diamond Sheet", {
        use crate::mesh_iso::{MeshIso, TpmsType, TpmsMode};
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)], 0.0);
        let m = MeshIso::from_tpms(TpmsType::DIAMOND, &box_, 10, 10, 10, 0.0, 1.0, TpmsMode::SHEET, 0.1);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_mesh_iso_from_tpms_neovius_shell() -> TestResult {
    MINI_TEST!("From Tpms Neovius Shell", {
        use crate::mesh_iso::{MeshIso, TpmsType, TpmsMode};
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)], 0.0);
        let m = MeshIso::from_tpms(TpmsType::NEOVIUS, &box_, 10, 10, 10, 0.0, 1.0, TpmsMode::SHELL, 0.1);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_mesh_iso_sdf_sphere() -> TestResult {
    MINI_TEST!("SDF Sphere", {
        use crate::mesh_iso::MeshIso;

        MINI_CHECK!(TOLERANCE.is_close(MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0), 0.0));
        MINI_CHECK!(TOLERANCE.is_close(MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0), -1.0));
        MINI_CHECK!(TOLERANCE.is_close(MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0), 1.0));
    })
}

pub fn run_mesh_iso_smooth_union() -> TestResult {
    MINI_TEST!("Smooth Union", {
        use crate::mesh_iso::MeshIso;

        MINI_CHECK!(MeshIso::smooth_union(-1.0, 1.0, 8.0) < 0.0);
        MINI_CHECK!(MeshIso::smooth_union(1.0, 1.0, 8.0) < 1.0);
    })
}

pub fn run_mesh_iso_from_function() -> TestResult {
    MINI_TEST!("From Function", {
        use crate::mesh_iso::MeshIso;
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(-2.0, -2.0, -2.0), Point::new(2.0, 2.0, 2.0)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, x, y, z);
        let m = MeshIso::from_function(fn_, &box_, 10, 10, 10, 0.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_mesh_iso_all_tpms_shells() -> TestResult {
    MINI_TEST!("All Tpms Shells", {
        use crate::mesh_iso::{MeshIso, TpmsType, TpmsMode};
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)], 0.0);
        let types = [
            TpmsType::GYROID, TpmsType::SCHWARZ_P, TpmsType::DIAMOND,
            TpmsType::NEOVIUS, TpmsType::IWP, TpmsType::LIDINOID,
            TpmsType::FISCHER_KOCH_S, TpmsType::FRD, TpmsType::PMY,
        ];

        for i in 0..9 {
            let m = MeshIso::from_tpms(types[i], &box_, 10, 10, 10, 0.0, 1.0, TpmsMode::SHELL, 0.1);
            MINI_CHECK!(m.is_valid());
            MINI_CHECK!(m.number_of_vertices() > 0);
        }
    })
}

pub fn run_mesh_iso_sdf_box() -> TestResult {
    MINI_TEST!("SDF Box", {
        use crate::mesh_iso::MeshIso;
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(-2.0, -2.0, -2.0), Point::new(2.0, 2.0, 2.0)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| MeshIso::sdf_box(0.0, 0.0, 0.0, 1.0, 0.7, 1.3, x, y, z);
        let m = MeshIso::from_function(fn_, &box_, 10, 10, 10, 0.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_mesh_iso_sdf_torus() -> TestResult {
    MINI_TEST!("SDF Torus", {
        use crate::mesh_iso::MeshIso;
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(-2.0, -2.0, -2.0), Point::new(2.0, 2.0, 2.0)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| MeshIso::sdf_torus(0.0, 0.0, 0.0, 1.1, 0.4, x, y, z);
        let m = MeshIso::from_function(fn_, &box_, 10, 10, 10, 0.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_mesh_iso_sdf_capsule() -> TestResult {
    MINI_TEST!("SDF Capsule", {
        use crate::mesh_iso::MeshIso;
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(-2.0, -2.0, -2.0), Point::new(2.0, 2.0, 2.0)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| {
            let p0 = Point::new(0.0, -1.0, 0.0);
            let p1 = Point::new(0.0, 1.0, 0.0);
            MeshIso::sdf_capsule(&p0, &p1, 0.5, x, y, z)
        };
        let m = MeshIso::from_function(fn_, &box_, 10, 10, 10, 0.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_mesh_iso_smooth_subtract() -> TestResult {
    MINI_TEST!("Smooth Subtract", {
        use crate::mesh_iso::MeshIso;
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(-2.0, -2.0, -2.0), Point::new(2.0, 2.0, 2.0)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| {
            let a = MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.2, x, y, z);
            let b = MeshIso::sdf_box(0.0, 0.0, 0.0, 0.8, 0.8, 0.8, x, y, z);
            MeshIso::smooth_subtract(a, b, 8.0)
        };
        let m = MeshIso::from_function(fn_, &box_, 15, 15, 15, 0.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_mesh_iso_smooth_intersect() -> TestResult {
    MINI_TEST!("Smooth Intersect", {
        use crate::mesh_iso::MeshIso;
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(-2.0, -2.0, -2.0), Point::new(2.0, 2.0, 2.0)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| {
            let a = MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.4, x, y, z);
            let b = MeshIso::sdf_box(0.0, 0.0, 0.0, 1.1, 1.1, 1.1, x, y, z);
            MeshIso::smooth_intersect(a, b, 8.0)
        };
        let m = MeshIso::from_function(fn_, &box_, 15, 15, 15, 0.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

pub fn run_mesh_iso_gyroid_sphere_shell() -> TestResult {
    MINI_TEST!("Gyroid Sphere Shell", {
        use crate::mesh_iso::{MeshIso, TpmsType};
        use crate::obb::OBB;
        use crate::point::Point;

        let box_ = OBB::from_points(&[Point::new(-1.6, -1.6, -1.6), Point::new(1.6, 1.6, 1.6)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| {
            let tpms = MeshIso::eval(TpmsType::GYROID, x, y, z, 1.0);
            let shell = MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.3, x, y, z).abs() - 0.08;
            MeshIso::smooth_union(tpms * 0.3, shell, 8.0)
        };
        let m = MeshIso::from_function(fn_, &box_, 10, 10, 10, 0.0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
    })
}

REGISTER_MINI_TEST!("MeshIso", "Eval Gyroid", crate::mesh_iso_test::run_mesh_iso_eval_gyroid);
REGISTER_MINI_TEST!("MeshIso", "Eval SchwarzP", crate::mesh_iso_test::run_mesh_iso_eval_schwarz_p);
REGISTER_MINI_TEST!("MeshIso", "Eval Diamond", crate::mesh_iso_test::run_mesh_iso_eval_diamond);
REGISTER_MINI_TEST!("MeshIso", "From Tpms Gyroid Solid", crate::mesh_iso_test::run_mesh_iso_from_tpms_gyroid_solid);
REGISTER_MINI_TEST!("MeshIso", "From Tpms Diamond Sheet", crate::mesh_iso_test::run_mesh_iso_from_tpms_diamond_sheet);
REGISTER_MINI_TEST!("MeshIso", "From Tpms Neovius Shell", crate::mesh_iso_test::run_mesh_iso_from_tpms_neovius_shell);
REGISTER_MINI_TEST!("MeshIso", "SDF Sphere", crate::mesh_iso_test::run_mesh_iso_sdf_sphere);
REGISTER_MINI_TEST!("MeshIso", "Smooth Union", crate::mesh_iso_test::run_mesh_iso_smooth_union);
REGISTER_MINI_TEST!("MeshIso", "From Function", crate::mesh_iso_test::run_mesh_iso_from_function);
REGISTER_MINI_TEST!("MeshIso", "All Tpms Shells", crate::mesh_iso_test::run_mesh_iso_all_tpms_shells);
REGISTER_MINI_TEST!("MeshIso", "SDF Box", crate::mesh_iso_test::run_mesh_iso_sdf_box);
REGISTER_MINI_TEST!("MeshIso", "SDF Torus", crate::mesh_iso_test::run_mesh_iso_sdf_torus);
REGISTER_MINI_TEST!("MeshIso", "SDF Capsule", crate::mesh_iso_test::run_mesh_iso_sdf_capsule);
REGISTER_MINI_TEST!("MeshIso", "Smooth Subtract", crate::mesh_iso_test::run_mesh_iso_smooth_subtract);
REGISTER_MINI_TEST!("MeshIso", "Smooth Intersect", crate::mesh_iso_test::run_mesh_iso_smooth_intersect);
REGISTER_MINI_TEST!("MeshIso", "Gyroid Sphere Shell", crate::mesh_iso_test::run_mesh_iso_gyroid_sphere_shell);
