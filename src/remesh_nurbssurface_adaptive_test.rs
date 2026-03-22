use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_remesh_nurbssurface_adaptive_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let ta = RemeshNurbssurfaceAdaptive::new(s);

        MINI_CHECK!(ta.get_max_angle() == 20.0);
        MINI_CHECK!(ta.get_max_edge_length() == 0.0);
        MINI_CHECK!(ta.get_min_edge_length() == 0.0);
        MINI_CHECK!(ta.get_max_chord_height() == 0.0);
    })
}

pub fn run_remesh_nurbssurface_adaptive_parameters() -> TestResult {
    MINI_TEST!("Parameters", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let mut ta = RemeshNurbssurfaceAdaptive::new(s);
        ta.set_max_angle(15.0)
          .set_max_edge_length(2.0)
          .set_min_edge_length(0.1)
          .set_max_chord_height(0.05);

        MINI_CHECK!(ta.get_max_angle() == 15.0);
        MINI_CHECK!(ta.get_max_edge_length() == 2.0);
        MINI_CHECK!(ta.get_min_edge_length() == 0.1);
        MINI_CHECK!(ta.get_max_chord_height() == 0.05);
    })
}

pub fn run_remesh_nurbssurface_adaptive_mesh() -> TestResult {
    MINI_TEST!("Mesh", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let ta = RemeshNurbssurfaceAdaptive::new(s);
        let m = ta.mesh();

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_remesh_nurbssurface_adaptive_torus() -> TestResult {
    MINI_TEST!("Torus", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);
        let m = RemeshNurbssurfaceAdaptive::new(s).mesh();

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_remesh_nurbssurface_adaptive_cylinder() -> TestResult {
    MINI_TEST!("Cylinder", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = RemeshNurbssurfaceAdaptive::new(s).mesh();

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_remesh_nurbssurface_adaptive_cone() -> TestResult {
    MINI_TEST!("Cone", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = RemeshNurbssurfaceAdaptive::new(s).mesh();

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_remesh_nurbssurface_adaptive_doubly_curved() -> TestResult {
    MINI_TEST!("Doubly Curved", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::wave_surface(1.0, 0.5);
        let m = RemeshNurbssurfaceAdaptive::new(s).mesh();

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

pub fn run_remesh_nurbssurface_adaptive_flat() -> TestResult {
    MINI_TEST!("Flat", {
        use crate::primitives::Primitives;
        use crate::remesh_nurbssurface_adaptive::RemeshNurbssurfaceAdaptive;

        let s = Primitives::wave_surface(1.0, 0.0);
        let m = RemeshNurbssurfaceAdaptive::new(s).mesh();

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
    })
}

REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Constructor", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_constructor);
REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Parameters", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_parameters);
REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Mesh", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_mesh);
REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Torus", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_torus);
REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Cylinder", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_cylinder);
REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Cone", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_cone);
REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Doubly Curved", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_doubly_curved);
REGISTER_MINI_TEST!("RemeshNurbssurfaceAdaptive", "Flat", crate::remesh_nurbssurface_adaptive_test::run_remesh_nurbssurface_adaptive_flat);
