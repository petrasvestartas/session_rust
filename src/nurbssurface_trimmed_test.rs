#![allow(
    clippy::bool_comparison,
    clippy::excessive_precision,
    clippy::needless_range_loop
)]
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_nurbssurface_trimmed_singular_planar_normal() -> TestResult {
    MINI_TEST!("Singular Planar Normal", {
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use crate::TrimLoops;

        let mut trimmed = NurbsSurfaceTrimmed::new();
        trimmed.m_surface = NurbsSurface::create(
            false,
            false,
            1,
            1,
            2,
            2,
            &[
                Point::new(-1.0, 0.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
            ],
        )
        .unwrap();
        let mut loops = TrimLoops::default();
        loops.uv.push(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ]);
        let mesh = trimmed.mesh_loops(&loops, 5.0, 0.001);

        MINI_CHECK!(!mesh.face.is_empty());
        let mut apex = false;

        for vertex in mesh.vertex.values() {
            let normal = vertex.normal().unwrap();

            MINI_CHECK!(normal[0].abs() < 1e-12 && normal[2].abs() < 1e-12);
            MINI_CHECK!((normal[1].abs() - 1.0).abs() < 1e-12);
            apex = apex || vertex.z == 1.0;
        }

        MINI_CHECK!(apex);
    })
}

pub fn run_nurbssurface_trimmed_crease_loops() -> TestResult {
    MINI_TEST!("Crease Loops", {
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use crate::TrimLoops;

        let mut ts = NurbsSurfaceTrimmed::new();
        ts.m_surface = NurbsSurface::create(
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
        let mut loops = TrimLoops::default();
        loops.uv.push(vec![
            Point::new(0.1, 0.1, 0.0),
            Point::new(1.9, 0.1, 0.0),
            Point::new(1.9, 0.9, 0.0),
            Point::new(0.1, 0.9, 0.0),
        ]);
        loops.uv.push(vec![
            Point::new(0.8, 0.4, 0.0),
            Point::new(1.2, 0.4, 0.0),
            Point::new(1.2, 0.6, 0.0),
            Point::new(0.8, 0.6, 0.0),
        ]);
        let mesh = ts.mesh_loops(&loops, 20.0, 0.005);

        MINI_CHECK!(mesh.vertex.len() == 16 && mesh.face.len() == 12);
        let mut flat = 0;
        let mut tilted = 0;

        for vd in mesh.vertex.values() {
            if vd.attributes.get("u") != Some(&1.0) {
                continue;
            }

            let mut interval = false;

            for name in vd.attributes.keys() {
                if name.starts_with("boundary_interval/") {
                    interval = true;
                }
            }

            MINI_CHECK!(interval && vd.z == 0.0);
            let normal = vd.normal().unwrap();

            if normal[0].abs() < 1e-12 {
                flat += 1;
            }

            if (normal[0] + 0.5f64.sqrt()).abs() < 1e-12 {
                tilted += 1;
            }
        }

        MINI_CHECK!(flat == 4 && tilted == 4);

        for face in mesh.face.values() {
            let mut low = f64::INFINITY;
            let mut high = f64::NEG_INFINITY;
            let mut u = 0.0;
            let mut v = 0.0;

            for vkey in face {
                let x = mesh.vertex[vkey]
                    .attributes
                    .get("u")
                    .copied()
                    .unwrap_or(0.0);
                low = low.min(x);
                high = high.max(x);
                u += x;
                v += mesh.vertex[vkey]
                    .attributes
                    .get("v")
                    .copied()
                    .unwrap_or(0.0);
            }

            MINI_CHECK!(!(low < 1.0 && high > 1.0));
            u /= 3.0;
            v /= 3.0;

            MINI_CHECK!(!(u > 0.8 && u < 1.2 && v > 0.4 && v < 0.6));
        }
    })
}

pub fn run_nurbssurface_trimmed_mesh_loops() -> TestResult {
    MINI_TEST!("Mesh Loops", {
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use crate::Primitives;
        use crate::TrimLoops;

        let planar = NurbsSurface::create(
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

        for surface in [planar, Primitives::wave_surface(1.0, 0.5)] {
            let mut ts = NurbsSurfaceTrimmed::new();
            ts.m_surface = surface;
            let mut loops = TrimLoops::default();

            for (low, high) in [(0.0, 1.0), (0.25, 0.75)] {
                let mut uv = Vec::new();
                let mut xyz = Vec::new();
                let corners = [(low, low), (high, low), (high, high), (low, high)];

                for side in 0..4 {
                    let a = corners[side];
                    let b = corners[(side + 1) % 4];

                    for sample in 0..8 {
                        let t = sample as f64 / 8.0;
                        uv.push(Point::new(
                            a.0 + t * (b.0 - a.0),
                            a.1 + t * (b.1 - a.1),
                            0.0,
                        ));
                    }
                }

                for p in &uv {
                    xyz.push(ts.m_surface.point_at(p[0], p[1]).unwrap());
                }

                loops.uv.push(uv);
                loops.xyz.push(xyz);
            }

            let mesh = ts.mesh_loops(&loops, 20.0, 0.005);

            MINI_CHECK!(!mesh.face.is_empty());

            for li in 0..loops.xyz.len() {
                let points = &loops.xyz[li];

                for sample in 0..points.len() {
                    let p = &points[sample];
                    let key = format!("boundary/{li}/{sample}");
                    let mut found = false;

                    for vd in mesh.vertex.values() {
                        if vd.attributes.contains_key(&key) {
                            MINI_CHECK!(vd.x == p[0] && vd.y == p[1] && vd.z == p[2]);
                            found = true;
                            break;
                        }
                    }

                    MINI_CHECK!(found);
                }
            }

            for vertices in mesh.face.values() {
                let mut u = 0.0;
                let mut v = 0.0;

                for key in vertices {
                    u += mesh.vertex[key].attributes.get("u").copied().unwrap_or(0.0);
                    v += mesh.vertex[key].attributes.get("v").copied().unwrap_or(0.0);
                }

                u /= vertices.len() as f64;
                v /= vertices.len() as f64;

                MINI_CHECK!(!(u > 0.25 && u < 0.75 && v > 0.25 && v < 0.75));
            }

            loops.xyz[0].pop();

            MINI_CHECK!(ts.mesh_loops(&loops, 20.0, 0.005).face.is_empty());
        }
    })
}

pub fn run_nurbssurface_trimmed_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(6.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 6.0, 0.0));
        srf.set_cv(1, 1, &Point::new(6.0, 6.0, 0.0));

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.1, 0.1, 0.0),
                Point::new(0.9, 0.1, 0.0),
                Point::new(0.9, 0.9, 0.0),
                Point::new(0.1, 0.9, 0.0),
            ],
        );

        let ts = NurbsSurfaceTrimmed::create(&srf, &outer);

        let sstr = ts.str();
        let srepr = ts.repr();

        let tscopy = ts.duplicate();

        MINI_CHECK!(ts.is_valid());
        MINI_CHECK!(ts.is_trimmed());
        MINI_CHECK!(ts.name == "my_nurbssurface_trimmed");
        MINI_CHECK!(!ts.guid().is_empty());
        MINI_CHECK!(sstr.contains("NurbsSurfaceTrimmed"));
        MINI_CHECK!(srepr.contains("name=my_nurbssurface_trimmed"));
        MINI_CHECK!(tscopy.is_valid());
        MINI_CHECK!(tscopy.guid() != ts.guid());
        MINI_CHECK!(tscopy == ts);
    })
}

pub fn run_nurbssurface_trimmed_constructor_planar() -> TestResult {
    MINI_TEST!("Constructor Planar", {
        use crate::tolerance::PI;
        use crate::NurbsCurve;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
            Point::new(5.0, 0.5, 0.0),
            Point::new(6.0, 3.0, 0.0),
            Point::new(4.0, 5.0, 0.0),
            Point::new(1.0, 4.0, 0.0),
        ];
        let bnd = NurbsCurve::create(true, 3, &pts);
        let _ts = NurbsSurfaceTrimmed::create_planar(&bnd);

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 1.0, -2.0),
            Point::new(5.0, 2.0, -3.0),
            Point::new(4.0, 4.0, 0.0),
            Point::new(1.0, 3.0, 2.0),
        ];
        let bnd = NurbsCurve::create(true, 3, &pts);
        let _ts = NurbsSurfaceTrimmed::create_planar(&bnd);

        let bnd = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(6.0, 3.0, 3.0),
                Point::new(2.0, 5.0, 1.0),
            ],
        );
        let _ts = NurbsSurfaceTrimmed::create_planar(&bnd);

        let bnd = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.0, 0.0, 6.0),
                Point::new(5.0, 0.0, 6.0),
                Point::new(4.0, 4.0, 2.0),
                Point::new(1.0, 4.0, 2.0),
            ],
        );
        let _ts = NurbsSurfaceTrimmed::create_planar(&bnd);

        let bnd = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(6.0, 0.0, 0.0),
                Point::new(6.0, 6.0, 0.0),
                Point::new(0.0, 6.0, 0.0),
            ],
        );
        let mut ts = NurbsSurfaceTrimmed::create_planar(&bnd);
        ts.add_hole(&NurbsCurve::create(
            true,
            1,
            &[
                Point::new(2.0, 2.0, 0.0),
                Point::new(4.0, 2.0, 0.0),
                Point::new(4.0, 4.0, 0.0),
                Point::new(2.0, 4.0, 0.0),
            ],
        ));

        MINI_CHECK!(ts.inner_loop_count() == 1);

        let r = 4.0;
        let mut pts = Vec::new();

        for k in 0..6 {
            let a = k as f64 * PI / 3.0;
            pts.push(Point::new(r * a.cos(), r * a.sin(), r * a.cos() * 0.5));
        }

        let bnd = NurbsCurve::create(true, 1, &pts);
        let mut ts = NurbsSurfaceTrimmed::create_planar(&bnd);
        ts.add_holes(&[
            NurbsCurve::create(
                true,
                1,
                &[
                    Point::new(1.5, 0.5, 0.75),
                    Point::new(2.5, 0.5, 1.25),
                    Point::new(2.0, 1.5, 1.0),
                ],
            ),
            NurbsCurve::create(
                true,
                1,
                &[
                    Point::new(-2.0, -0.5, -1.0),
                    Point::new(-1.0, -0.5, -0.5),
                    Point::new(-1.0, -1.5, -0.5),
                    Point::new(-2.0, -1.5, -1.0),
                ],
            ),
        ]);

        MINI_CHECK!(ts.is_valid());
        MINI_CHECK!(ts.is_trimmed());
        MINI_CHECK!(ts.inner_loop_count() == 2);
    })
}

pub fn run_nurbssurface_trimmed_constructor_hole() -> TestResult {
    MINI_TEST!("Constructor Hole", {
        use crate::tolerance::PI;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use crate::Primitives;

        let n = 8;
        let mut pts = Vec::new();

        for i in 0..n {
            for j in 0..n {
                let x = i as f64;
                let y = j as f64;
                let r2 = (x - 1.5) * (x - 1.5) + (y - 1.5) * (y - 1.5);
                let z = 5.0 * (-r2 / 1.0).exp() + 0.3 * (PI * x / 7.0).sin() * (PI * y / 7.0).sin();
                pts.push(Point::new(x, y, z));
            }
        }

        let srf = NurbsSurface::create(false, false, 3, 3, n, n, &pts).unwrap();

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
        );

        let mut ts = NurbsSurfaceTrimmed::create(&srf, &outer);

        let hole = Primitives::circle(3.5, 3.5, 0.0, 1.0);
        ts.add_hole(&hole);

        MINI_CHECK!(ts.is_valid());
        MINI_CHECK!(ts.is_trimmed());
        MINI_CHECK!(ts.inner_loop_count() == 1);
    })
}

pub fn run_nurbssurface_trimmed_accessors() -> TestResult {
    MINI_TEST!("Accessors", {
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(5.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 5.0, 0.0));
        srf.set_cv(1, 1, &Point::new(5.0, 5.0, 0.0));

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.1, 0.1, 0.0),
                Point::new(0.9, 0.1, 0.0),
                Point::new(0.9, 0.9, 0.0),
                Point::new(0.1, 0.9, 0.0),
            ],
        );

        let mut ts = NurbsSurfaceTrimmed::create(&srf, &outer);
        ts.name = "test_accessors".to_string();
        ts.width = 2.5;

        let got_srf = ts.surface();
        let got_loop = ts.get_outer_loop().unwrap();

        MINI_CHECK!(ts.is_valid());
        MINI_CHECK!(ts.is_trimmed());
        MINI_CHECK!(ts.name == "test_accessors");
        MINI_CHECK!(ts.width == 2.5);
        MINI_CHECK!(got_srf.is_valid());
        MINI_CHECK!(got_loop.is_valid());
        MINI_CHECK!(ts.inner_loop_count() == 0);
    })
}

pub fn run_nurbssurface_trimmed_add_inner_loop() -> TestResult {
    MINI_TEST!("Add Inner Loop", {
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(10.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 10.0, 0.0));
        srf.set_cv(1, 1, &Point::new(10.0, 10.0, 0.0));

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
        );

        let mut ts = NurbsSurfaceTrimmed::create(&srf, &outer);

        let hole1 = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.2, 0.2, 0.0),
                Point::new(0.4, 0.2, 0.0),
                Point::new(0.4, 0.4, 0.0),
                Point::new(0.2, 0.4, 0.0),
            ],
        );
        let hole2 = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.6, 0.6, 0.0),
                Point::new(0.8, 0.6, 0.0),
                Point::new(0.8, 0.8, 0.0),
                Point::new(0.6, 0.8, 0.0),
            ],
        );

        ts.add_inner_loop(hole1);
        ts.add_inner_loop(hole2);

        let got = ts.get_inner_loop(0);

        MINI_CHECK!(ts.inner_loop_count() == 2);
        MINI_CHECK!(got.is_valid());

        ts.clear_inner_loops();

        MINI_CHECK!(ts.inner_loop_count() == 0);
    })
}

pub fn run_nurbssurface_trimmed_point_at() -> TestResult {
    MINI_TEST!("Point At", {
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(4.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 4.0, 0.0));
        srf.set_cv(1, 1, &Point::new(4.0, 4.0, 0.0));

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
        );

        let ts = NurbsSurfaceTrimmed::create(&srf, &outer);

        let dom_u = ts.surface().domain(0).unwrap();
        let dom_v = ts.surface().domain(1).unwrap();
        let u_mid = (dom_u.0 + dom_u.1) / 2.0;
        let v_mid = (dom_v.0 + dom_v.1) / 2.0;

        let pt = ts.point_at(u_mid, v_mid).unwrap();
        let nm = ts.normal_at(u_mid, v_mid);

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(nm[2].abs(), 1.0));
    })
}

pub fn run_nurbssurface_trimmed_mesh() -> TestResult {
    MINI_TEST!("Mesh", {
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(6.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 6.0, 0.0));
        srf.set_cv(1, 1, &Point::new(6.0, 6.0, 0.0));

        let m_full = srf.mesh();

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.1, 0.1, 0.0),
                Point::new(0.9, 0.1, 0.0),
                Point::new(0.9, 0.9, 0.0),
                Point::new(0.1, 0.9, 0.0),
            ],
        );
        let ts = NurbsSurfaceTrimmed::create(&srf, &outer);
        let m = ts.mesh();

        let hole = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.3, 0.3, 0.0),
                Point::new(0.7, 0.3, 0.0),
                Point::new(0.7, 0.7, 0.0),
                Point::new(0.3, 0.7, 0.0),
            ],
        );
        let mut ts_hole = NurbsSurfaceTrimmed::create(&srf, &outer);
        ts_hole.add_inner_loop(hole);
        let m_hole = ts_hole.mesh();

        MINI_CHECK!(!m.is_empty());
        MINI_CHECK!(m.number_of_vertices() > 0);
        MINI_CHECK!(m.number_of_faces() > 0);
        MINI_CHECK!(m_full.number_of_faces() > 0);
        MINI_CHECK!(m_hole.number_of_faces() > 0);

        let cw = 2.0f64.sqrt() / 2.0;
        let ccx = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let ccy = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let cwt = [1.0, cw, 1.0, cw, 1.0, cw, 1.0, cw, 1.0];
        let mut circle_loop = NurbsCurve::new(3, true, 3, 9);
        circle_loop.m_nurbsknot = vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];

        for i in 0..9 {
            circle_loop.set_cv_4d(
                i,
                (0.5 + 0.5 * ccx[i]) * cwt[i],
                (0.5 + 0.5 * ccy[i]) * cwt[i],
                0.0,
                cwt[i],
            );
        }

        let ts_circ = NurbsSurfaceTrimmed::create(&srf, &circle_loop);
        let mc = ts_circ.mesh();

        MINI_CHECK!(!mc.is_empty());
        MINI_CHECK!(mc.number_of_vertices() >= 30);
        MINI_CHECK!(mc.number_of_faces() >= 30);

        for vd in mc.vertex.values() {
            let nx = vd.attributes.get("nx").copied().unwrap_or(0.0);
            let ny = vd.attributes.get("ny").copied().unwrap_or(0.0);
            let nz = vd.attributes.get("nz").copied().unwrap_or(0.0);

            MINI_CHECK!((nx * nx + ny * ny + nz * nz).sqrt() > 0.5);
        }
    })
}

pub fn run_nurbssurface_trimmed_split_by_uv_curves() -> TestResult {
    MINI_TEST!("Split By UV Curves", {
        use crate::NurbsCurve;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use crate::Primitives;

        let srf = Primitives::wave_surface(10.0, 1.0);
        let (u0, u1) = srf.domain(0).unwrap();
        let (v0, v1) = srf.domain(1).unwrap();
        let pts = vec![
            Point::new(u0 + (u1 - u0) * 0.4, v0, 0.0),
            Point::new(u0 + (u1 - u0) * 0.6, v1, 0.0),
        ];
        let line = NurbsCurve::create(false, 1, &pts);

        let parts = NurbsSurfaceTrimmed::split_by_uv_curves(&srf, &[line], 0.0);

        MINI_CHECK!(parts.len() == 2);
        MINI_CHECK!(parts[0].is_trimmed());
        MINI_CHECK!(parts[1].is_trimmed());

        let circle = Primitives::circle((u0 + u1) * 0.5, (v0 + v1) * 0.5, 0.0, (u1 - u0) * 0.2);

        let ring = NurbsSurfaceTrimmed::split_by_uv_curves(&srf, &[circle], 0.0);

        MINI_CHECK!(ring.len() == 2);
        MINI_CHECK!(ring[0].inner_loop_count() + ring[1].inner_loop_count() == 1);

        let dangling = NurbsCurve::create(
            false,
            1,
            &[Point::new(3.0, 3.0, 0.0), Point::new(5.0, 5.0, 0.0)],
        );

        let whole = NurbsSurfaceTrimmed::split_by_uv_curves(&srf, &[dangling], 0.0);

        MINI_CHECK!(whole.len() == 1);
    })
}

pub fn run_nurbssurface_trimmed_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use crate::Xform;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(1.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 1.0, 0.0));
        srf.set_cv(1, 1, &Point::new(1.0, 1.0, 0.0));

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
        );

        let ts = NurbsSurfaceTrimmed::create(&srf, &outer);
        let ts_xf = Xform::translation(10.0, 20.0, 30.0);
        let ts2 = ts.transformed(&ts_xf);

        let dom_u = ts2.surface().domain(0).unwrap();
        let dom_v = ts2.surface().domain(1).unwrap();
        let pt = ts2.point_at(dom_u.0, dom_v.0).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 20.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 30.0));
    })
}

pub fn run_nurbssurface_trimmed_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Color;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use std::path::PathBuf;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(5.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 5.0, 0.0));
        srf.set_cv(1, 1, &Point::new(5.0, 5.0, 0.0));

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.1, 0.1, 0.0),
                Point::new(0.9, 0.1, 0.0),
                Point::new(0.9, 0.9, 0.0),
                Point::new(0.1, 0.9, 0.0),
            ],
        );

        let mut ts = NurbsSurfaceTrimmed::create(&srf, &outer);
        ts.name = "test_nurbssurface_trimmed".to_string();
        ts.width = 2.0;
        ts.surfacecolor = Color::new(1.0, 0.5, 0.25, 1.0);

        let json = ts.jsondump().unwrap();
        let loaded_json = NurbsSurfaceTrimmed::jsonload(&json).unwrap();

        let json_string = ts.file_json_dumps();
        let loaded_json_string = NurbsSurfaceTrimmed::file_json_loads(&json_string);

        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir
            .join("serialization")
            .join("test_nurbssurface_trimmed.json");
        ts.file_json_dump(filename.to_str().unwrap()).unwrap();
        let loaded_from_file =
            NurbsSurfaceTrimmed::file_json_load(filename.to_str().unwrap()).unwrap();

        MINI_CHECK!(loaded_json == ts);
        MINI_CHECK!(loaded_json_string == ts);
        MINI_CHECK!(loaded_from_file == ts);
    })
}

pub fn run_nurbssurface_trimmed_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Color;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::NurbsSurfaceTrimmed;
        use crate::Point;
        use std::path::PathBuf;

        let mut srf = NurbsSurface::new(3, false, 2, 2, 2, 2);
        srf.set_cv(0, 0, &Point::new(0.0, 0.0, 0.0));
        srf.set_cv(1, 0, &Point::new(5.0, 0.0, 0.0));
        srf.set_cv(0, 1, &Point::new(0.0, 5.0, 0.0));
        srf.set_cv(1, 1, &Point::new(5.0, 5.0, 0.0));

        let outer = NurbsCurve::create(
            true,
            1,
            &[
                Point::new(0.1, 0.1, 0.0),
                Point::new(0.9, 0.1, 0.0),
                Point::new(0.9, 0.9, 0.0),
                Point::new(0.1, 0.9, 0.0),
            ],
        );

        let mut ts = NurbsSurfaceTrimmed::create(&srf, &outer);
        ts.name = "test_nurbssurface_trimmed".to_string();
        ts.width = 2.0;
        ts.surfacecolor = Color::new(1.0, 0.5, 0.25, 1.0);

        let guid = ts.guid().to_string();
        let proto_string = ts.pb_dumps();
        let loaded_proto_string = NurbsSurfaceTrimmed::pb_loads(&proto_string).unwrap();
        let converted = NurbsSurfaceTrimmed::from_proto(ts.to_proto()).unwrap();

        let src_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let filename = src_dir
            .join("serialization")
            .join("test_nurbssurface_trimmed.bin");
        ts.pb_dump(filename.to_str().unwrap());
        let loaded = NurbsSurfaceTrimmed::pb_load(filename.to_str().unwrap());

        MINI_CHECK!(loaded_proto_string == ts);
        MINI_CHECK!(loaded == ts);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(converted == ts);
        MINI_CHECK!(converted.guid() == guid);
        MINI_CHECK!(converted.inner_loop_count() == ts.inner_loop_count());
        MINI_CHECK!(converted.is_trimmed());
    })
}

REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Singular Planar Normal",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_singular_planar_normal
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Crease Loops",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_crease_loops
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Mesh Loops",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_mesh_loops
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Constructor",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_constructor
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Constructor Planar",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_constructor_planar
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Constructor Hole",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_constructor_hole
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Accessors",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_accessors
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Add Inner Loop",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_add_inner_loop
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Point At",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_point_at
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Mesh",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_mesh
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Split By UV Curves",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_split_by_uv_curves
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Transformation",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_transformation
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Json Roundtrip",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_json_roundtrip
);
REGISTER_MINI_TEST!(
    "NurbsSurfaceTrimmed",
    "Protobuf Roundtrip",
    crate::nurbssurface_trimmed_test::run_nurbssurface_trimmed_protobuf_roundtrip
);
