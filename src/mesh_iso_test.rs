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
        use crate::boundingbox::BoundingBox;
        use crate::point::Point;

        let box_ = BoundingBox::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)], 0.0);
        let m = MeshIso::from_tpms(TpmsType::GYROID, &box_, 10, 10, 10, 0.0, 1.0, TpmsMode::SOLID, 0.2);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_mesh_iso_from_tpms_diamond_sheet() -> TestResult {
    MINI_TEST!("From Tpms Diamond Sheet", {
        use crate::mesh_iso::{MeshIso, TpmsType, TpmsMode};
        use crate::boundingbox::BoundingBox;
        use crate::point::Point;

        let box_ = BoundingBox::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)], 0.0);
        let m = MeshIso::from_tpms(TpmsType::DIAMOND, &box_, 10, 10, 10, 0.0, 1.0, TpmsMode::SHEET, 0.1);

        MINI_CHECK!(m.is_valid());
    })
}

pub fn run_mesh_iso_from_tpms_neovius_shell() -> TestResult {
    MINI_TEST!("From Tpms Neovius Shell", {
        use crate::mesh_iso::{MeshIso, TpmsType, TpmsMode};
        use crate::boundingbox::BoundingBox;
        use crate::point::Point;

        let box_ = BoundingBox::from_points(&[Point::new(0.0, 0.0, 0.0), Point::new(1.0, 1.0, 1.0)], 0.0);
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
        use crate::boundingbox::BoundingBox;
        use crate::point::Point;

        let box_ = BoundingBox::from_points(&[Point::new(-2.0, -2.0, -2.0), Point::new(2.0, 2.0, 2.0)], 0.0);
        let fn_ = |x: f64, y: f64, z: f64| MeshIso::sdf_sphere(0.0, 0.0, 0.0, 1.0, x, y, z);
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
