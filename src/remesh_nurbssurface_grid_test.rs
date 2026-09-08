use crate::mini_test::TestResult;
use crate::tolerance::Tolerance;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_remesh_nurbssurface_grid_analytic_normals() -> TestResult {
    MINI_TEST!("Analytic Normals", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::Primitives;

        let surfaces = [
            Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0),
            Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0),
            Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0),
        ];
        for (index, s) in surfaces.into_iter().enumerate() {
            let m = RemeshNurbsSurfaceGrid::from_u_v_q(s, 0, 0, 30.0, 0.01);
            for vd in m.vertex.values() {
                let n = vd.normal().unwrap();
                let length = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
                MINI_CHECK!((length - 1.0).abs() < Tolerance::ZERO_TOLERANCE);
                if index < 2 {
                    let z = if index == 0 { vd.z } else { 0.0 };
                    let dot = vd.x * n[0] + vd.y * n[1] + z * n[2];
                    MINI_CHECK!((dot - 1.0).abs() < Tolerance::ZERO_TOLERANCE);
                }
            }
        }
    })
}

REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Analytic Normals",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_analytic_normals
);

pub fn run_remesh_nurbssurface_grid_sphere() -> TestResult {
    MINI_TEST!("Sphere", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::Primitives;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 191);
        MINI_CHECK!(m.number_of_faces() == 378);
    })
}

pub fn run_remesh_nurbssurface_grid_torus() -> TestResult {
    MINI_TEST!("Torus", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::Primitives;

        let s = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 693);
        MINI_CHECK!(m.number_of_faces() == 1386);
    })
}

pub fn run_remesh_nurbssurface_grid_cylinder() -> TestResult {
    MINI_TEST!("Cylinder", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::Primitives;

        let s = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 42);
        MINI_CHECK!(m.number_of_faces() == 42);
    })
}

pub fn run_remesh_nurbssurface_grid_cone() -> TestResult {
    MINI_TEST!("Cone", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::Primitives;

        let s = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 22);
        MINI_CHECK!(m.number_of_faces() == 21);
    })
}

pub fn run_remesh_nurbssurface_grid_doubly_curved() -> TestResult {
    MINI_TEST!("Doubly Curved", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::Primitives;

        let s = Primitives::wave_surface(1.0, 0.5);
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 961);
        MINI_CHECK!(m.number_of_faces() == 1800);
    })
}

pub fn run_remesh_nurbssurface_grid_grid_target() -> TestResult {
    MINI_TEST!("Grid Target", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::Primitives;

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
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::NurbsSurface;
        use crate::Point;

        let s = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(0.0, 4.0, 0.0),
                Point::new(4.0, 0.0, 0.0),
                Point::new(4.0, 4.0, 0.0),
            ],
        )
        .unwrap();
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 4);
        MINI_CHECK!(m.number_of_faces() == 2);
    })
}

pub fn run_remesh_nurbssurface_grid_flat_triangle() -> TestResult {
    MINI_TEST!("Flat Triangle", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::NurbsSurface;
        use crate::Point;

        let s = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(2.0, 4.0, 0.0),
                Point::new(4.0, 0.0, 0.0),
                Point::new(2.0, 4.0, 0.0),
            ],
        )
        .unwrap();
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);

        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 3);
        MINI_CHECK!(m.number_of_faces() == 1);
    })
}

pub fn run_remesh_nurbssurface_grid_double_curved_triangle() -> TestResult {
    MINI_TEST!("Double-Curved Triangle", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::NurbsSurface;
        use crate::Point;

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

REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Sphere",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_sphere
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Torus",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_torus
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Cylinder",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_cylinder
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Cone",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_cone
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Doubly Curved",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_doubly_curved
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Grid Target",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_grid_target
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Flat Quad",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_flat_quad
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Flat Triangle",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_flat_triangle
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Double-Curved Triangle",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_double_curved_triangle
);

pub fn run_remesh_nurbssurface_grid_crease_normals() -> TestResult {
    MINI_TEST!("Crease Normals", {
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        use crate::{NurbsSurface, Point};
        let s = NurbsSurface::create(
            false,
            false,
            1,
            1,
            3,
            2,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(2.0, 0.0, 1.0),
                Point::new(2.0, 1.0, 1.0),
            ],
        )
        .unwrap();
        let m = RemeshNurbsSurfaceGrid::from_u_v(s, 0, 0);
        MINI_CHECK!(m.vertex.len() == 8);
        MINI_CHECK!(m.face.len() == 4);
        let mut flat = 0;
        let mut tilted = 0;
        for vd in m.vertex.values() {
            if vd.x != 1.0 {
                continue;
            }
            let n = vd.normal().unwrap();
            if n[0].abs() < Tolerance::ZERO_TOLERANCE {
                flat += 1;
            }
            if (n[0] + 0.5f64.sqrt()).abs() < Tolerance::ZERO_TOLERANCE {
                tilted += 1;
            }
        }
        MINI_CHECK!(flat == 2 && tilted == 2);
    })
}
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Crease Normals",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_crease_normals
);

/// U-collapsed corners retain the planar fan normal on every nondegenerate triangle.
pub fn run_remesh_nurbssurface_grid_singular_planar_normal() -> TestResult {
    MINI_TEST!("Singular Planar Normal", {
        use crate::{NurbsSurface, Point};
        use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
        let surface = NurbsSurface::create(false,false,1,1,2,2,&[
            Point::new(0.0,0.0,1.0),Point::new(0.0,0.0,1.0),
            Point::new(-1.0,0.0,0.0),Point::new(1.0,0.0,0.0)]).unwrap();
        let mesh = RemeshNurbsSurfaceGrid::from_u_v_q(surface,0,0,5.0,0.001);
        let mut apex = false;
        for face in mesh.face.values() {
            let a = &mesh.vertex[&face[0]];
            let b = &mesh.vertex[&face[1]];
            let c = &mesh.vertex[&face[2]];
            if ((b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x)).abs() <= 1e-14 { continue; }
            for vertex in [a,b,c] {
                let normal = vertex.normal().unwrap();
                MINI_CHECK!(normal[0].abs() < 1e-12 && normal[2].abs() < 1e-12);
                MINI_CHECK!((normal[1].abs()-1.0).abs() < 1e-12);
                apex |= vertex.z == 1.0;
            }
        }
        MINI_CHECK!(apex);
    })
}
REGISTER_MINI_TEST!("RemeshNurbsSurfaceGrid", "Singular Planar Normal",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_singular_planar_normal);
