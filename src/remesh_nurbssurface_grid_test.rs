use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_remesh_nurbssurface_grid_sphere() -> TestResult {
    MINI_TEST!("Sphere", {
        use crate::Primitives;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 191);
        MINI_CHECK!(m.number_of_faces() == 378);
    })
}

pub fn run_remesh_nurbssurface_grid_torus() -> TestResult {
    MINI_TEST!("Torus", {
        use crate::Primitives;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 693);
        MINI_CHECK!(m.number_of_faces() == 1386);
    })
}

pub fn run_remesh_nurbssurface_grid_cylinder() -> TestResult {
    MINI_TEST!("Cylinder", {
        use crate::Primitives;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 42);
        MINI_CHECK!(m.number_of_faces() == 42);
    })
}

pub fn run_remesh_nurbssurface_grid_cone() -> TestResult {
    MINI_TEST!("Cone", {
        use crate::Primitives;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 22);
        MINI_CHECK!(m.number_of_faces() == 21);
    })
}

pub fn run_remesh_nurbssurface_grid_doubly_curved() -> TestResult {
    MINI_TEST!("Doubly Curved", {
        use crate::Primitives;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = Primitives::wave_surface(1.0, 0.5);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 961);
        MINI_CHECK!(m.number_of_faces() == 1800);
    })
}

pub fn run_remesh_nurbssurface_grid_grid_target() -> TestResult {
    MINI_TEST!("Grid Target", {
        use crate::Primitives;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = Primitives::wave_surface(1.0, 0.5);
        let m_lo = RemeshNurbsSurfaceGrid::from_u_v(s.clone(), 8, 8);
        let m_hi = RemeshNurbsSurfaceGrid::from_u_v(s, 32, 32);

        MINI_CHECK!(m_lo.is_valid());
        MINI_CHECK!(m_lo.number_of_vertices() == 64);
        MINI_CHECK!(m_hi.is_valid());
        MINI_CHECK!(m_hi.number_of_vertices() > m_lo.number_of_vertices());
    })
}

pub fn run_remesh_nurbssurface_grid_flat_quad() -> TestResult {
    MINI_TEST!("Flat Quad", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 0.0),
        ]).unwrap();
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 4);
        MINI_CHECK!(m.number_of_faces() == 2);
    })
}

pub fn run_remesh_nurbssurface_grid_flat_triangle() -> TestResult {
    MINI_TEST!("Flat Triangle", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let s = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 4.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(2.0, 4.0, 0.0),
        ]).unwrap();
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 3);
        MINI_CHECK!(m.number_of_faces() == 1);
    })
}

pub fn run_remesh_nurbssurface_grid_double_curved_triangle() -> TestResult {
    MINI_TEST!("Double-Curved Triangle", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 3.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(0.0, 2.0, 2.0),
            Point::new(2.0, 2.0, 5.0),
            Point::new(4.0, 2.0, 2.0),
            Point::new(2.0, 4.0, 0.0),
            Point::new(2.0, 4.0, 0.0),
            Point::new(2.0, 4.0, 0.0),
        ];
        let s = NurbsSurface::create(false, false, 2, 2, 3, 3, &pts).unwrap();
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 64);
        MINI_CHECK!(m.number_of_faces() == 98);
    })
}

REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Sphere", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_sphere);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Torus", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_torus);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Cylinder", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_cylinder);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Cone", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_cone);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Doubly Curved", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_doubly_curved);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Grid Target", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_grid_target);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Flat Quad", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_flat_quad);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Flat Triangle", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_flat_triangle);
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Double-Curved Triangle", crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_double_curved_triangle);
