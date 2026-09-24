#![allow(clippy::needless_range_loop)]
use crate::mini_test::TestResult;
use crate::tolerance::Tolerance;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_remesh_nurbssurface_grid_singular_planar_normal() -> TestResult {
    MINI_TEST!("Singular Planar Normal", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(0.0, 0.0, 1.0),
                Point::new(0.0, 0.0, 1.0),
                Point::new(-1.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
            ],
        )
        .unwrap();
        let mesh = RemeshNurbsSurfaceGrid::from_u_v_q(&surface, 0, 0, 5.0, 0.001);
        let mut apex = false;

        for face in mesh.face.values() {
            let a = &mesh.vertex[&face[0]];
            let b = &mesh.vertex[&face[1]];
            let c = &mesh.vertex[&face[2]];

            if ((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)).abs() <= 1e-14 {
                continue;
            }

            for vertex_key in face {
                let vertex = &mesh.vertex[vertex_key];
                let normal = vertex.normal().unwrap();

                MINI_CHECK!(normal[0].abs() < 1e-12 && normal[2].abs() < 1e-12);
                MINI_CHECK!((normal[1].abs() - 1.0).abs() < 1e-12);

                apex = apex || vertex.z == 1.0;
            }
        }

        MINI_CHECK!(apex);
    })
}

pub fn run_remesh_nurbssurface_grid_crease_normals() -> TestResult {
    MINI_TEST!("Crease Normals", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = NurbsSurface::create(
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
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.vertex.len() == 8);
        MINI_CHECK!(mesh.face.len() == 4);

        let mut flat = 0;
        let mut tilted = 0;

        for vd in mesh.vertex.values() {
            if vd.x != 1.0 {
                continue;
            }

            let normal = vd.normal().unwrap();

            if normal[0].abs() < Tolerance::ZERO_TOLERANCE {
                flat += 1;
            }

            if (normal[0] + 0.5_f64.sqrt()).abs() < Tolerance::ZERO_TOLERANCE {
                tilted += 1;
            }
        }

        MINI_CHECK!(flat == 2 && tilted == 2);
    })
}

pub fn run_remesh_nurbssurface_grid_analytic_normals() -> TestResult {
    MINI_TEST!("Analytic Normals", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surfaces = [
            Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0),
            Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0),
            Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0),
        ];

        for index in 0..surfaces.len() {
            let surface = &surfaces[index];
            let mesh = RemeshNurbsSurfaceGrid::from_u_v_q(surface, 0, 0, 30.0, 0.01);

            for vd in mesh.vertex.values() {
                let normal = vd.normal().unwrap();
                let length = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];

                MINI_CHECK!((length - 1.0).abs() < Tolerance::ZERO_TOLERANCE);

                if index < 2 {
                    let z = if index == 0 { vd.z } else { 0.0 };
                    let dot = vd.x * normal[0] + vd.y * normal[1] + z * normal[2];

                    MINI_CHECK!((dot - 1.0).abs() < Tolerance::ZERO_TOLERANCE);
                }
            }
        }
    })
}

pub fn run_remesh_nurbssurface_grid_sphere() -> TestResult {
    MINI_TEST!("Sphere", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 191);
        MINI_CHECK!(mesh.number_of_faces() == 378);
    })
}

pub fn run_remesh_nurbssurface_grid_sphere_few_rows() -> TestResult {
    MINI_TEST!("Sphere Few Rows", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = Primitives::sphere_surface(0.0, 0.0, 0.0, 1.0);
        let one = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 1);
        let two = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 2);
        let three = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 3);

        MINI_CHECK!(one.number_of_vertices() == 0);
        MINI_CHECK!(two.number_of_vertices() == 0);
        MINI_CHECK!(three.is_valid());
        MINI_CHECK!(three.number_of_vertices() == 23);
        MINI_CHECK!(three.number_of_faces() == 42);
    })
}

pub fn run_remesh_nurbssurface_grid_torus() -> TestResult {
    MINI_TEST!("Torus", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 693);
        MINI_CHECK!(mesh.number_of_faces() == 1386);
    })
}

pub fn run_remesh_nurbssurface_grid_cylinder() -> TestResult {
    MINI_TEST!("Cylinder", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 42);
        MINI_CHECK!(mesh.number_of_faces() == 42);
    })
}

pub fn run_remesh_nurbssurface_grid_cone() -> TestResult {
    MINI_TEST!("Cone", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 22);
        MINI_CHECK!(mesh.number_of_faces() == 21);
    })
}

pub fn run_remesh_nurbssurface_grid_doubly_curved() -> TestResult {
    MINI_TEST!("Doubly Curved", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = Primitives::wave_surface(1.0, 0.5);
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 961);
        MINI_CHECK!(mesh.number_of_faces() == 1800);
    })
}

pub fn run_remesh_nurbssurface_grid_grid_target() -> TestResult {
    MINI_TEST!("Grid Target", {
        use crate::Primitives;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = Primitives::wave_surface(1.0, 0.5);
        let mesh_lo = RemeshNurbsSurfaceGrid::from_u_v(&surface, 8, 8);
        let mesh_hi = RemeshNurbsSurfaceGrid::from_u_v(&surface, 32, 32);

        MINI_CHECK!(mesh_lo.is_valid());
        MINI_CHECK!(mesh_lo.number_of_vertices() == 64);
        MINI_CHECK!(mesh_hi.is_valid());
        MINI_CHECK!(mesh_hi.number_of_vertices() > mesh_lo.number_of_vertices());
    })
}

pub fn run_remesh_nurbssurface_grid_flat_quad() -> TestResult {
    MINI_TEST!("Flat Quad", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = NurbsSurface::create(
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
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 4);
        MINI_CHECK!(mesh.number_of_faces() == 2);
    })
}

pub fn run_remesh_nurbssurface_grid_flat_triangle() -> TestResult {
    MINI_TEST!("Flat Triangle", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = NurbsSurface::create(
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
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 3);
        MINI_CHECK!(mesh.number_of_faces() == 1);
    })
}

pub fn run_remesh_nurbssurface_grid_double_curved_triangle() -> TestResult {
    MINI_TEST!("Double-Curved Triangle", {
        use crate::NurbsSurface;
        use crate::Point;
        use crate::RemeshNurbsSurfaceGrid;

        let surface = NurbsSurface::create(
            false,
            false,
            2,
            2,
            3,
            3,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 3.0),
                Point::new(4.0, 0.0, 0.0),
                Point::new(0.0, 2.0, 2.0),
                Point::new(2.0, 2.0, 5.0),
                Point::new(4.0, 2.0, 2.0),
                Point::new(2.0, 4.0, 0.0),
                Point::new(2.0, 4.0, 0.0),
                Point::new(2.0, 4.0, 0.0),
            ],
        )
        .unwrap();
        let mesh = RemeshNurbsSurfaceGrid::from_u_v(&surface, 0, 0);

        MINI_CHECK!(mesh.is_valid());
        MINI_CHECK!(mesh.number_of_vertices() == 64);
        MINI_CHECK!(mesh.number_of_faces() == 98);
    })
}

REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Singular Planar Normal",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_singular_planar_normal
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Crease Normals",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_crease_normals
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Analytic Normals",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_analytic_normals
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Sphere",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_sphere
);
REGISTER_MINI_TEST!(
    "RemeshNurbsSurfaceGrid",
    "Sphere Few Rows",
    crate::remesh_nurbssurface_grid_test::run_remesh_nurbssurface_grid_sphere_few_rows
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
