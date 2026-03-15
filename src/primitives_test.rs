use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_primitives_mesh_arrow() -> TestResult {
    MINI_TEST!("Mesh Arrow", {
        use crate::primitives::Primitives;
        use crate::line::Line;

        let line = Line::new(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
        let m = Primitives::arrow_mesh(&line, 1.0);

        MINI_CHECK!(m.number_of_vertices() == 29);
        MINI_CHECK!(m.number_of_faces() == 28);
    })
}

pub fn run_primitives_mesh_cylinder() -> TestResult {
    MINI_TEST!("Mesh Cylinder", {
        use crate::primitives::Primitives;
        use crate::line::Line;

        let line = Line::new(0.0, 0.0, 0.0, 0.0, 0.0, 8.0);
        let m = Primitives::cylinder_mesh(&line, 1.0);

        MINI_CHECK!(m.number_of_vertices() == 20);
        MINI_CHECK!(m.number_of_faces() == 20);
    })
}

pub fn run_primitives_mesh_edge_pipes() -> TestResult {
    MINI_TEST!("Mesh Edge Pipes", {
        use crate::primitives::Primitives;
        use crate::mesh::Mesh;
        use crate::point::Point;
        use crate::color::Color;

        let mut mesh = Mesh::new();
        let v0 = mesh.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let v1 = mesh.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let v2 = mesh.add_vertex(Point::new(1.0, 1.0, 0.0), None);
        let v3 = mesh.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        mesh.add_face(vec![v0, v1, v2, v3], None);
        mesh.linecolors_mut()[0] = Color::red();

        let pipes = Primitives::edge_pipes(&mesh, 0.1);
        MINI_CHECK!(pipes.len() == 4);
        MINI_CHECK!(pipes[0].get_facecolors()[0][0] == Color::red()[0]);
    })
}

pub fn run_primitives_nurbscurve_polyline() -> TestResult {
    MINI_TEST!("Nurbscurve Polyline", {
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;

        let c = NurbsCurve::create(false, 1, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 2.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ]);

        MINI_CHECK!(c.cv_count() == 5);
        MINI_CHECK!(c.order() == 2);
        MINI_CHECK!(c.degree() == 1);
        MINI_CHECK!(c.is_rational() == false);
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(c.domain_start()), &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(c.domain_end()), &Point::new(4.0, 0.0, 0.0)));
    })
}

pub fn run_primitives_nurbscurve_circle() -> TestResult {
    MINI_TEST!("Nurbscurve Circle", {
        use crate::primitives::Primitives;

        let c = Primitives::circle(0.0, 0.0, 0.0, 1.0);

        MINI_CHECK!(c.cv_count() == 9);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == true);
    })
}

pub fn run_primitives_nurbscurve_ellipse() -> TestResult {
    MINI_TEST!("Nurbscurve Ellipse", {
        use crate::primitives::Primitives;

        let c = Primitives::ellipse(0.0, 0.0, 0.0, 2.0, 1.0);

        MINI_CHECK!(c.cv_count() == 9);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == true);
    })
}

pub fn run_primitives_nurbscurve_arc() -> TestResult {
    MINI_TEST!("Nurbscurve Arc", {
        use crate::primitives::Primitives;
        use crate::point::Point;

        let start = Point::new(0.0, 0.0, 0.0);
        let mid = Point::new(1.0, 1.0, 0.0);
        let end = Point::new(2.0, 0.0, 0.0);
        let c = Primitives::arc(&start, &mid, &end);

        MINI_CHECK!(c.cv_count() == 3);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == true);
    })
}

pub fn run_primitives_nurbscurve_parabola() -> TestResult {
    MINI_TEST!("Nurbscurve Parabola", {
        use crate::primitives::Primitives;
        use crate::point::Point;

        let p0 = Point::new(-1.0, 1.0, 0.0);
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 1.0, 0.0);
        let c = Primitives::parabola(&p0, &p1, &p2);

        MINI_CHECK!(c.cv_count() == 3);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == false);
    })
}

pub fn run_primitives_nurbscurve_hyperbola() -> TestResult {
    MINI_TEST!("Nurbscurve Hyperbola", {
        use crate::primitives::Primitives;
        use crate::point::Point;

        let center = Point::new(0.0, 0.0, 0.0);
        let c = Primitives::hyperbola(&center, 1.0, 1.0, 1.0);

        MINI_CHECK!(c.cv_count() >= 4);
        MINI_CHECK!(c.order() == 4);
        MINI_CHECK!(c.is_rational() == false);
    })
}

pub fn run_primitives_nurbscurve_spiral() -> TestResult {
    MINI_TEST!("Nurbscurve Spiral", {
        use crate::primitives::Primitives;

        let c = Primitives::spiral(1.0, 2.0, 1.0, 5.0);

        MINI_CHECK!(c.cv_count() >= 4);
        MINI_CHECK!(c.order() == 4);
        MINI_CHECK!(c.is_rational() == false);
    })
}

pub fn run_primitives_nurbssurface_cylinder() -> TestResult {
    MINI_TEST!("Nurbssurface Cylinder", {
        use crate::primitives::Primitives;

        let s = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);

        MINI_CHECK!(s.is_valid());
        MINI_CHECK!(s.is_rational());
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(s.order(0) == 3);
        MINI_CHECK!(s.order(1) == 2);

        let p00 = s.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!((p00[0] - 1.0).abs() < 1e-10);
        MINI_CHECK!((p00[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p00[2] - 0.0).abs() < 1e-10);

        let p01 = s.point_at(0.0, 1.0).unwrap();
        MINI_CHECK!((p01[0] - 1.0).abs() < 1e-10);
        MINI_CHECK!((p01[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p01[2] - 5.0).abs() < 1e-10);

        let pmid = s.point_at(1.0, 0.5).unwrap();
        MINI_CHECK!((pmid[0] - 0.0).abs() < 1e-10);
        MINI_CHECK!((pmid[1] - 1.0).abs() < 1e-10);
        MINI_CHECK!((pmid[2] - 2.5).abs() < 1e-10);
    })
}

pub fn run_primitives_nurbssurface_cone() -> TestResult {
    MINI_TEST!("Nurbssurface Cone", {
        use crate::primitives::Primitives;

        let s = Primitives::cone_surface(0.0, 0.0, 0.0, 1.0, 5.0);

        MINI_CHECK!(s.is_valid());
        MINI_CHECK!(s.is_rational());
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(s.order(0) == 3);
        MINI_CHECK!(s.order(1) == 2);

        let pbase = s.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!((pbase[0] - 1.0).abs() < 1e-10);
        MINI_CHECK!((pbase[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((pbase[2] - 0.0).abs() < 1e-10);

        let papex = s.point_at(0.0, 1.0).unwrap();
        MINI_CHECK!((papex[0] - 0.0).abs() < 1e-10);
        MINI_CHECK!((papex[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((papex[2] - 5.0).abs() < 1e-10);

        let pmid = s.point_at(0.0, 0.5).unwrap();
        MINI_CHECK!((pmid[0] - 0.5).abs() < 1e-10);
        MINI_CHECK!((pmid[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((pmid[2] - 2.5).abs() < 1e-10);
    })
}

pub fn run_primitives_nurbssurface_sphere() -> TestResult {
    MINI_TEST!("Nurbssurface Sphere", {
        use crate::primitives::Primitives;

        let s = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);

        MINI_CHECK!(s.is_valid());
        MINI_CHECK!(s.is_rational());
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s.cv_count_dir(Some(1)) == 5);
        MINI_CHECK!(s.order(0) == 3);
        MINI_CHECK!(s.order(1) == 3);

        let p00 = s.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!((p00[0] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p00[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p00[2] - (-2.0)).abs() < 1e-10);

        let p_top = s.point_at(0.0, 2.0).unwrap();
        MINI_CHECK!((p_top[0] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p_top[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p_top[2] - 2.0).abs() < 1e-10);

        let p_eq = s.point_at(0.0, 1.0).unwrap();
        MINI_CHECK!((p_eq[0] - 2.0).abs() < 1e-10);
        MINI_CHECK!((p_eq[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p_eq[2] - 0.0).abs() < 1e-10);

        let p_eq2 = s.point_at(1.0, 1.0).unwrap();
        MINI_CHECK!((p_eq2[0] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p_eq2[1] - 2.0).abs() < 1e-10);
        MINI_CHECK!((p_eq2[2] - 0.0).abs() < 1e-10);
    })
}

pub fn run_primitives_nurbssurface_quad_sphere() -> TestResult {
    MINI_TEST!("Nurbssurface Quad Sphere", {
        use crate::primitives::Primitives;

        let r = 5.0_f64;
        let faces = Primitives::quad_sphere(0.0, 0.0, 0.0, r);

        MINI_CHECK!(faces.len() == 6);
        for f in 0..6 {
            MINI_CHECK!(faces[f].is_valid());
            MINI_CHECK!(faces[f].is_rational());
            MINI_CHECK!(faces[f].order(0) == 3);
            MINI_CHECK!(faces[f].order(1) == 3);
            MINI_CHECK!(faces[f].cv_count_dir(Some(0)) == 3);
            MINI_CHECK!(faces[f].cv_count_dir(Some(1)) == 3);
        }

        let mut max_err = 0.0_f64;
        for f in 0..6 {
            for i in 0..=4 {
                let u = i as f64 / 4.0;
                for j in 0..=4 {
                    let v = j as f64 / 4.0;
                    let p = faces[f].point_at(u, v).unwrap();
                    let dist = (p[0]*p[0] + p[1]*p[1] + p[2]*p[2]).sqrt();
                    let err = (dist - r).abs();
                    if err > max_err { max_err = err; }
                }
            }
        }
        MINI_CHECK!(max_err < 0.02 * r);
    })
}

pub fn run_primitives_nurbssurface_torus() -> TestResult {
    MINI_TEST!("Nurbssurface Torus", {
        use crate::primitives::Primitives;

        let s = Primitives::torus_surface(0.0, 0.0, 0.0, 3.0, 1.0);

        MINI_CHECK!(s.is_valid());
        MINI_CHECK!(s.is_rational());
        MINI_CHECK!(s.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s.cv_count_dir(Some(1)) == 9);
        MINI_CHECK!(s.order(0) == 3);
        MINI_CHECK!(s.order(1) == 3);

        let p00 = s.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!((p00[0] - 4.0).abs() < 1e-10);
        MINI_CHECK!((p00[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p00[2] - 0.0).abs() < 1e-10);

        let p10 = s.point_at(1.0, 0.0).unwrap();
        MINI_CHECK!((p10[0] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p10[1] - 4.0).abs() < 1e-10);
        MINI_CHECK!((p10[2] - 0.0).abs() < 1e-10);

        let p_top = s.point_at(0.0, 1.0).unwrap();
        MINI_CHECK!((p_top[0] - 3.0).abs() < 1e-10);
        MINI_CHECK!((p_top[1] - 0.0).abs() < 1e-10);
        MINI_CHECK!((p_top[2] - 1.0).abs() < 1e-10);
    })
}

pub fn run_primitives_nurbssurface_ruled() -> TestResult {
    MINI_TEST!("Nurbssurface Ruled", {
        use crate::primitives::Primitives;
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;
        use crate::vector::Vector;

        let pts_a = vec![Point::new(3.0, 0.0, 0.0), Point::new(-2.0, 0.0, 5.0)];
        let pts_b = vec![Point::new(3.0, 5.0, 5.0), Point::new(-2.0, 5.0, 0.0)];
        let crv_a = NurbsCurve::create(false, 1, &pts_a);
        let crv_b = NurbsCurve::create(false, 1, &pts_b);
        let srf = Primitives::create_ruled(&crv_a, &crv_b);
        let _m = srf.mesh();

        MINI_CHECK!(srf.is_valid());
        MINI_CHECK!(srf.degree(0) == 1);
        MINI_CHECK!(srf.degree(1) == 1);
        MINI_CHECK!(srf.cv_count_dir(Some(0)) == 2);
        MINI_CHECK!(srf.cv_count_dir(Some(1)) == 2);

        let (rd, ruv) = srf.divide_by_count(4, 4);
        MINI_CHECK!(rd.len() == 5);
        MINI_CHECK!(rd[0].len() == 5);

        let mut pts: Vec<Point> = Vec::new();
        for i in 0..rd.len() {
            for j in 0..rd[i].len() {
                pts.push(rd[i][j].clone());
            }
        }

        let mut normals: Vec<Vector> = Vec::new();
        for i in 0..ruv.len() {
            for j in 0..ruv[i].len() {
                normals.push(srf.normal_at(ruv[i][j].0, ruv[i][j].1));
            }
        }

        let mut uvs: Vec<(f64, f64)> = Vec::new();
        for i in 0..ruv.len() {
            for j in 0..ruv[i].len() {
                uvs.push(ruv[i][j]);
            }
        }

        MINI_CHECK!(TOLERANCE.is_point_close(&pts[0],  &Point::new( 3.00, 0.00, 0.00)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[1],  &Point::new( 3.00, 1.25, 1.25)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[2],  &Point::new( 3.00, 2.50, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[3],  &Point::new( 3.00, 3.75, 3.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[4],  &Point::new( 3.00, 5.00, 5.00)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[5],  &Point::new( 1.75, 0.00, 1.25)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[6],  &Point::new( 1.75, 1.25, 1.875)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[7],  &Point::new( 1.75, 2.50, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[8],  &Point::new( 1.75, 3.75, 3.125)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[9],  &Point::new( 1.75, 5.00, 3.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[10], &Point::new( 0.50, 0.00, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[11], &Point::new( 0.50, 1.25, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[12], &Point::new( 0.50, 2.50, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[13], &Point::new( 0.50, 3.75, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[14], &Point::new( 0.50, 5.00, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[15], &Point::new(-0.75, 0.00, 3.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[16], &Point::new(-0.75, 1.25, 3.125)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[17], &Point::new(-0.75, 2.50, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[18], &Point::new(-0.75, 3.75, 1.875)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[19], &Point::new(-0.75, 5.00, 1.25)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[20], &Point::new(-2.00, 0.00, 5.00)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[21], &Point::new(-2.00, 1.25, 3.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[22], &Point::new(-2.00, 2.50, 2.50)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[23], &Point::new(-2.00, 3.75, 1.25)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[24], &Point::new(-2.00, 5.00, 0.00)));

        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[0],  &Vector::new(-0.577350269189626,  0.577350269189626, -0.577350269189626)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[1],  &Vector::new(-1.0/3.0,  2.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[2],  &Vector::new( 0.0,  0.707106781186547, -0.707106781186547)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[3],  &Vector::new( 1.0/3.0,  2.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[4],  &Vector::new( 0.577350269189626,  0.577350269189626, -0.577350269189626)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[5],  &Vector::new(-2.0/3.0,  1.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[6],  &Vector::new(-0.408248290463863,  0.408248290463863, -0.816496580927726)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[7],  &Vector::new( 0.0,  0.447213595499958, -0.894427190999916)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[8],  &Vector::new( 0.408248290463863,  0.408248290463863, -0.816496580927726)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[9],  &Vector::new( 2.0/3.0,  1.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[10], &Vector::new(-0.707106781186547,  0.0, -0.707106781186547)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[11], &Vector::new(-0.447213595499958,  0.0, -0.894427190999916)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[12], &Vector::new( 0.0,  0.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[13], &Vector::new( 0.447213595499958,  0.0, -0.894427190999916)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[14], &Vector::new( 0.707106781186547,  0.0, -0.707106781186547)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[15], &Vector::new(-2.0/3.0, -1.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[16], &Vector::new(-0.408248290463863, -0.408248290463863, -0.816496580927726)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[17], &Vector::new( 0.0, -0.447213595499958, -0.894427190999916)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[18], &Vector::new( 0.408248290463863, -0.408248290463863, -0.816496580927726)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[19], &Vector::new( 2.0/3.0, -1.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[20], &Vector::new(-0.577350269189626, -0.577350269189626, -0.577350269189626)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[21], &Vector::new(-1.0/3.0, -2.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[22], &Vector::new( 0.0, -0.707106781186547, -0.707106781186547)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[23], &Vector::new( 1.0/3.0, -2.0/3.0, -2.0/3.0)));
        MINI_CHECK!(TOLERANCE.is_vector_close(&normals[24], &Vector::new( 0.577350269189626, -0.577350269189626, -0.577350269189626)));

        MINI_CHECK!(TOLERANCE.is_close(uvs[0].0,  0.00) && TOLERANCE.is_close(uvs[0].1,  0.00));
        MINI_CHECK!(TOLERANCE.is_close(uvs[1].0,  0.00) && TOLERANCE.is_close(uvs[1].1,  0.25));
        MINI_CHECK!(TOLERANCE.is_close(uvs[4].0,  0.00) && TOLERANCE.is_close(uvs[4].1,  1.00));
        MINI_CHECK!(TOLERANCE.is_close(uvs[6].0,  0.25) && TOLERANCE.is_close(uvs[6].1,  0.25));
        MINI_CHECK!(TOLERANCE.is_close(uvs[12].0, 0.50) && TOLERANCE.is_close(uvs[12].1, 0.50));
        MINI_CHECK!(TOLERANCE.is_close(uvs[24].0, 1.00) && TOLERANCE.is_close(uvs[24].1, 1.00));
    })
}

pub fn run_primitives_nurbssurface_planar() -> TestResult {
    MINI_TEST!("Nurbssurface Planar", {
        use crate::primitives::Primitives;
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;

        let c1 = 0.7_f64.cos(); let s1 = 0.7_f64.sin();
        let c2 = 0.96_f64.cos(); let s2 = 0.96_f64.sin();
        let c3 = 0.52_f64.cos(); let s3 = 0.52_f64.sin();
        let c4 = 1.13_f64.cos(); let s4 = 1.13_f64.sin();

        let ca = NurbsCurve::create(false, 1, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 3.0*c1, 3.0*s1),
            Point::new(0.0, 3.0*c1, 3.0*s1),
            Point::new(0.0, 0.0, 0.0),
        ]);
        let s_quad = Primitives::create_planar(&ca);
        let m_quad = s_quad.mesh();

        let cb1 = NurbsCurve::create(false, 1, &[
            Point::new(8.0, 0.0, 0.0),
            Point::new(8.0+5.0*c2, 0.0, 5.0*s2),
            Point::new(8.0+2.0*c2, 3.0, 2.0*s2),
            Point::new(8.0, 0.0, 0.0),
        ]);
        let s_triangle = Primitives::create_planar(&cb1);
        let m_triangle = s_triangle.mesh();

        let ox = 18.0;
        let cb2 = NurbsCurve::create(false, 1, &[
            Point::new(ox+0.0*c3, 0.0*s3, 0.0),
            Point::new(ox+4.0*c3, 4.0*s3, 0.0),
            Point::new(ox+5.0*c3-2.0*s3, 5.0*s3+2.0*c3, 0.0),
            Point::new(ox+3.0*c3-4.0*s3, 3.0*s3+4.0*c3, 0.0),
            Point::new(ox-1.0*c3-3.0*s3, -1.0*s3+3.0*c3, 0.0),
            Point::new(ox+0.0*c3, 0.0*s3, 0.0),
        ]);
        let s_polygon = Primitives::create_planar(&cb2);
        let m_polygon = s_polygon.mesh();

        let cc = NurbsCurve::create(false, 3, &[
            Point::new(26.0, 0.0, 0.0),
            Point::new(29.0, 1.0*c4, 1.0*s4),
            Point::new(31.0, 0.5*c4, 0.5*s4),
            Point::new(32.0, 3.0*c4, 3.0*s4),
            Point::new(30.0, 5.0*c4, 5.0*s4),
            Point::new(27.0, 4.0*c4, 4.0*s4),
            Point::new(26.0, 0.0, 0.0),
        ]);
        let s_nurbs = Primitives::create_planar(&cc);
        let m_nurbs = s_nurbs.mesh();

        MINI_CHECK!(s_quad.is_valid());
        MINI_CHECK!(s_quad.is_planar(1e-6));
        MINI_CHECK!(s_quad.cv_count_dir(Some(0)) == 2);
        MINI_CHECK!(s_quad.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_quad.number_of_vertices() == 4);
        MINI_CHECK!(m_quad.number_of_faces() == 2);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_quad.get_cv(0,0).unwrap(), &Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_quad.get_cv(0,1).unwrap(), &Point::new(0.0, 2.294526561853465, 1.932653061713073)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_quad.get_cv(1,0).unwrap(), &Point::new(4.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_quad.get_cv(1,1).unwrap(), &Point::new(4.0, 2.294526561853465, 1.932653061713073)));

        MINI_CHECK!(s_triangle.is_valid());
        MINI_CHECK!(s_triangle.is_planar(1e-6));
        MINI_CHECK!(s_triangle.cv_count_dir(Some(0)) == 2);
        MINI_CHECK!(s_triangle.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_triangle.number_of_vertices() == 3);
        MINI_CHECK!(m_triangle.number_of_faces() == 1);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_triangle.get_cv(0,0).unwrap(), &Point::new(8.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_triangle.get_cv(0,1).unwrap(), &Point::new(8.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_triangle.get_cv(1,0).unwrap(), &Point::new(10.867599930362283, 0.0, 4.095957841504991)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_triangle.get_cv(1,1).unwrap(), &Point::new(9.147039972144913, 3.0, 1.638383136601997)));

        MINI_CHECK!(s_polygon.is_valid());
        MINI_CHECK!(s_polygon.is_planar(1e-6));
        MINI_CHECK!(s_polygon.cv_count_dir(Some(0)) == 2);
        MINI_CHECK!(s_polygon.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_polygon.number_of_vertices() == 4);
        MINI_CHECK!(m_polygon.number_of_faces() == 2);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_polygon.get_cv(0,0).unwrap(), &Point::new(19.673777861921977, 6.364048611360808, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_polygon.get_cv(0,1).unwrap(), &Point::new(22.915428262469927, 2.987233669553135, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_polygon.get_cv(1,0).unwrap(), &Point::new(15.247175891573059, 2.114631246911942, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_polygon.get_cv(1,1).unwrap(), &Point::new(18.488826292121008, -1.262183694895731, 0.0)));

        MINI_CHECK!(s_nurbs.is_valid());
        MINI_CHECK!(s_nurbs.is_planar(1e-6));
        MINI_CHECK!(s_nurbs.cv_count_dir(Some(0)) == 2);
        MINI_CHECK!(s_nurbs.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_nurbs.number_of_vertices() == 4);
        MINI_CHECK!(m_nurbs.number_of_faces() == 2);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_nurbs.get_cv(0,0).unwrap(), &Point::new(26.652846559932474, -0.727774577493594, -1.542700265577809)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_nurbs.get_cv(0,1).unwrap(), &Point::new(24.347485651711366, 0.916607409071279, 1.942978687541882)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_nurbs.get_cv(1,0).unwrap(), &Point::new(32.606791655643732, 0.791738725121784, 1.678288276735475)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_nurbs.get_cv(1,1).unwrap(), &Point::new(30.301430747422629, 2.436120711686657, 5.163967229855166)));
    })
}

pub fn run_primitives_nurbssurface_extrusion() -> TestResult {
    MINI_TEST!("Nurbssurface Extrusion", {
        use crate::primitives::Primitives;
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;
        use crate::vector::Vector;

        let dir = Vector::new(0.0, 1.0, 5.0);

        let c1 = NurbsCurve::create(false, 1, &[Point::new(13.0, 0.0, 0.0), Point::new(18.0, 0.0, 0.0)]);
        let s_line = Primitives::create_extrusion(&c1, &dir);
        let m_line = s_line.mesh();

        let c2 = Primitives::circle(24.0, 0.0, 0.0, 3.0);
        let s_circle = Primitives::create_extrusion(&c2, &dir);
        let m_circle = s_circle.mesh();

        let c3 = NurbsCurve::create(false, 2, &[
            Point::new(30.0, 0.0, 0.0), Point::new(33.0, 5.0, 0.0), Point::new(37.0, 0.0, 0.0),
        ]);
        let s_arc = Primitives::create_extrusion(&c3, &dir);
        let m_arc = s_arc.mesh();

        let c4 = NurbsCurve::create(false, 1, &[
            Point::new(40.0, 3.0, 0.0), Point::new(45.0, 0.0, 0.0),
            Point::new(50.0, 3.0, 0.0), Point::new(55.0, 0.0, 0.0),
        ]);
        let s_wavy = Primitives::create_extrusion(&c4, &dir);
        let m_wavy = s_wavy.mesh();

        MINI_CHECK!(s_line.is_valid());
        MINI_CHECK!(s_line.degree(0) == 1 && s_line.degree(1) == 1);
        MINI_CHECK!(s_line.cv_count_dir(Some(0)) == 2 && s_line.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_line.number_of_vertices() == 4);
        MINI_CHECK!(m_line.number_of_faces() == 2);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_line.get_cv(0,0).unwrap(), &Point::new(13.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_line.get_cv(0,1).unwrap(), &Point::new(13.0, 1.0, 5.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_line.get_cv(1,0).unwrap(), &Point::new(18.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_line.get_cv(1,1).unwrap(), &Point::new(18.0, 1.0, 5.0)));

        MINI_CHECK!(s_circle.is_valid());
        MINI_CHECK!(s_circle.degree(0) == 2 && s_circle.degree(1) == 1);
        MINI_CHECK!(s_circle.is_rational());
        MINI_CHECK!(s_circle.is_closed(0) == true && s_circle.is_closed(1) == false);
        MINI_CHECK!(s_circle.cv_count_dir(Some(0)) == 9 && s_circle.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_circle.number_of_vertices() == 40);
        MINI_CHECK!(m_circle.number_of_faces() == 40);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_circle.get_cv(0,0).unwrap(), &Point::new(27.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_circle.get_cv(0,1).unwrap(), &Point::new(27.0, 1.0, 5.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_circle.get_cv(4,0).unwrap(), &Point::new(21.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_circle.get_cv(4,1).unwrap(), &Point::new(21.0, 1.0, 5.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_circle.get_cv(8,0).unwrap(), &Point::new(27.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_circle.get_cv(8,1).unwrap(), &Point::new(27.0, 1.0, 5.0)));

        MINI_CHECK!(s_arc.is_valid());
        MINI_CHECK!(s_arc.degree(0) == 2 && s_arc.degree(1) == 1);
        MINI_CHECK!(s_arc.cv_count_dir(Some(0)) == 3 && s_arc.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_arc.number_of_vertices() == 16);
        MINI_CHECK!(m_arc.number_of_faces() == 14);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_arc.get_cv(0,0).unwrap(), &Point::new(30.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_arc.get_cv(0,1).unwrap(), &Point::new(30.0, 1.0, 5.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_arc.get_cv(1,0).unwrap(), &Point::new(33.0, 5.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_arc.get_cv(1,1).unwrap(), &Point::new(33.0, 6.0, 5.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_arc.get_cv(2,0).unwrap(), &Point::new(37.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_arc.get_cv(2,1).unwrap(), &Point::new(37.0, 1.0, 5.0)));

        MINI_CHECK!(s_wavy.is_valid());
        MINI_CHECK!(s_wavy.degree(0) == 1 && s_wavy.degree(1) == 1);
        MINI_CHECK!(s_wavy.cv_count_dir(Some(0)) == 4 && s_wavy.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_wavy.number_of_vertices() == 8);
        MINI_CHECK!(m_wavy.number_of_faces() == 6);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_wavy.get_cv(0,0).unwrap(), &Point::new(40.0, 3.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_wavy.get_cv(0,1).unwrap(), &Point::new(40.0, 4.0, 5.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_wavy.get_cv(1,0).unwrap(), &Point::new(45.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_wavy.get_cv(1,1).unwrap(), &Point::new(45.0, 1.0, 5.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_wavy.get_cv(3,0).unwrap(), &Point::new(55.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_wavy.get_cv(3,1).unwrap(), &Point::new(55.0, 1.0, 5.0)));
    })
}

pub fn run_primitives_nurbssurface_loft() -> TestResult {
    MINI_TEST!("Nurbssurface Loft", {
        use crate::primitives::Primitives;
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;

        let c1 = Primitives::circle(0.0, 0.0, 0.0, 2.0);
        let c2 = Primitives::circle(0.0, 0.0, 2.0, 1.0);
        let c3 = Primitives::circle(0.0, 0.0, 4.0, 1.5);
        let c4 = Primitives::circle(0.0, 0.0, 6.0, 0.8);

        let srf = Primitives::create_loft(&[c1, c2, c3, c4], 3);

        MINI_CHECK!(srf.is_valid());
        MINI_CHECK!(srf.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(srf.cv_count_dir(Some(1)) == 4);
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(0, 0).unwrap(), &Point::new(2.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(0, 1).unwrap(), &Point::new(-0.677194251158421, 0.0, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(0, 2).unwrap(), &Point::new(3.00619893067415, 0.0, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(0, 3).unwrap(), &Point::new(0.8, 0.0, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(1, 0).unwrap(), &Point::new(2.0, 2.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(1, 1).unwrap(), &Point::new(-0.677194251158421, -0.677194251158421, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(1, 2).unwrap(), &Point::new(3.00619893067414, 3.00619893067414, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(1, 3).unwrap(), &Point::new(0.8, 0.8, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(2, 0).unwrap(), &Point::new(0.0, 2.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(2, 1).unwrap(), &Point::new(0.0, -0.677194251158421, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(2, 2).unwrap(), &Point::new(0.0, 3.00619893067415, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(2, 3).unwrap(), &Point::new(0.0, 0.8, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(3, 0).unwrap(), &Point::new(-2.0, 2.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(3, 1).unwrap(), &Point::new(0.677194251158421, -0.677194251158421, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(3, 2).unwrap(), &Point::new(-3.00619893067414, 3.00619893067414, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(3, 3).unwrap(), &Point::new(-0.8, 0.8, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(4, 0).unwrap(), &Point::new(-2.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(4, 1).unwrap(), &Point::new(0.677194251158421, 0.0, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(4, 2).unwrap(), &Point::new(-3.00619893067415, 0.0, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(4, 3).unwrap(), &Point::new(-0.8, 0.0, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(5, 0).unwrap(), &Point::new(-2.0, -2.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(5, 1).unwrap(), &Point::new(0.677194251158421, 0.677194251158421, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(5, 2).unwrap(), &Point::new(-3.00619893067414, -3.00619893067414, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(5, 3).unwrap(), &Point::new(-0.8, -0.8, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(6, 0).unwrap(), &Point::new(0.0, -2.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(6, 1).unwrap(), &Point::new(0.0, 0.677194251158421, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(6, 2).unwrap(), &Point::new(0.0, -3.00619893067415, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(6, 3).unwrap(), &Point::new(0.0, -0.8, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(7, 0).unwrap(), &Point::new(2.0, -2.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(7, 1).unwrap(), &Point::new(-0.677194251158421, 0.677194251158421, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(7, 2).unwrap(), &Point::new(3.00619893067414, -3.00619893067414, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(7, 3).unwrap(), &Point::new(0.8, -0.8, 6.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(8, 0).unwrap(), &Point::new(2.0, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(8, 1).unwrap(), &Point::new(-0.677194251158421, 0.0, 1.75222035185728)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(8, 2).unwrap(), &Point::new(3.00619893067415, 0.0, 4.08030037218547)));
        MINI_CHECK!(TOLERANCE.is_point_close(&srf.get_cv(8, 3).unwrap(), &Point::new(0.8, 0.0, 6.0)));

        let open_pts = vec![
            vec![
                Point::new(10.0, -12.0, 0.0), Point::new(10.0, -10.0, 3.0),
                Point::new(10.0, -7.0, 3.0), Point::new(10.0, -5.0, 0.0),
            ],
            vec![
                Point::new(5.5, -12.0, 3.5), Point::new(5.5, -10.0, 1.5),
                Point::new(5.5, -7.0, 1.5), Point::new(5.5, -5.0, 3.5),
            ],
            vec![
                Point::new(1.0, -12.0, 0.0), Point::new(1.0, -10.0, 3.0),
                Point::new(1.0, -7.0, 3.0), Point::new(1.0, -5.0, 0.0),
            ],
        ];
        let open_curves = vec![
            NurbsCurve::create(false, 3, &open_pts[0]),
            NurbsCurve::create(false, 3, &open_pts[1]),
            NurbsCurve::create(false, 3, &open_pts[2]),
        ];
        let open_srf = Primitives::create_loft(&open_curves, 3);

        MINI_CHECK!(open_srf.is_valid());
        MINI_CHECK!(open_srf.cv_count_dir(Some(0)) == 4);
        MINI_CHECK!(open_srf.cv_count_dir(Some(1)) == 3);

        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(0, 0).unwrap(), &Point::new(10.0, -12.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(0, 1).unwrap(), &Point::new(5.5, -12.0, 7.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(0, 2).unwrap(), &Point::new(1.0, -12.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(1, 0).unwrap(), &Point::new(10.0, -10.0, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(1, 1).unwrap(), &Point::new(5.5, -10.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(1, 2).unwrap(), &Point::new(1.0, -10.0, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(2, 0).unwrap(), &Point::new(10.0, -7.0, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(2, 1).unwrap(), &Point::new(5.5, -7.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(2, 2).unwrap(), &Point::new(1.0, -7.0, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(3, 0).unwrap(), &Point::new(10.0, -5.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(3, 1).unwrap(), &Point::new(5.5, -5.0, 7.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&open_srf.get_cv(3, 2).unwrap(), &Point::new(1.0, -5.0, 0.0)));
    })
}

pub fn run_primitives_nurbssurface_revolve() -> TestResult {
    MINI_TEST!("Nurbssurface Revolve", {
        use crate::primitives::Primitives;
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;
        use crate::vector::Vector;

        let pa = NurbsCurve::create(false, 3, &[
            Point::new(1.5, 0.0, 0.0),
            Point::new(1.5, 0.0, 0.3),
            Point::new(0.3, 0.0, 0.5),
            Point::new(0.3, 0.0, 2.5),
            Point::new(0.2, 0.0, 3.0),
            Point::new(2.0, 0.0, 4.5),
            Point::new(1.8, 0.0, 5.0),
        ]);
        let s_vase = Primitives::create_revolve(&pa, &Point::new(0.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), 2.0 * crate::tolerance::PI);
        let m_vase = s_vase.mesh();

        let w = (2.0_f64).sqrt() / 2.0;
        let cw = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
        let ca = [1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
        let sa = [0.0, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
        let ck = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
        let rr = 5.0; let r = 1.5; let tcx = 14.0;
        let mut pb = NurbsCurve::new(3, true, 3, 9);
        for i in 0..10 { pb.set_knot(i, ck[i]); }
        for i in 0..9 { pb.set_cv_4d(i, (tcx + rr + r * ca[i]) * cw[i], 0.0, r * sa[i] * cw[i], cw[i]); }
        let s_torus = Primitives::create_revolve(&pb, &Point::new(tcx, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), 2.0 * crate::tolerance::PI);
        let m_torus = s_torus.mesh();

        let pc = NurbsCurve::create(false, 1, &[Point::new(29.0, 0.0, -0.5), Point::new(29.0, 0.0, 0.5)]);
        let s_elbow = Primitives::create_revolve(&pc, &Point::new(26.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), crate::tolerance::PI / 2.0);
        let m_elbow = s_elbow.mesh();

        let sr = 2.0; let scx = 36.0;
        let mut pd = NurbsCurve::new(3, true, 3, 5);
        let sk = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0];
        for i in 0..6 { pd.set_knot(i, sk[i]); }
        let spx = [0.0, sr, sr, sr, 0.0]; let spz = [-sr, -sr, 0.0, sr, sr]; let spw = [1.0, w, 1.0, w, 1.0];
        for i in 0..5 { pd.set_cv_4d(i, (scx + spx[i]) * spw[i], 0.0, spz[i] * spw[i], spw[i]); }
        let s_sphere = Primitives::create_revolve(&pd, &Point::new(scx, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), 2.0 * crate::tolerance::PI);
        let m_sphere = s_sphere.mesh();

        let pe = NurbsCurve::create(false, 1, &[Point::new(44.0, 0.0, 3.0), Point::new(46.0, 0.0, 0.0)]);
        let s_cone = Primitives::create_revolve(&pe, &Point::new(44.0, 0.0, 0.0), &Vector::new(0.0, 0.0, 1.0), 2.0 * crate::tolerance::PI);
        let m_cone = s_cone.mesh();

        MINI_CHECK!(s_vase.is_valid());
        MINI_CHECK!(s_vase.is_closed(0) == true);
        MINI_CHECK!(s_vase.is_closed(1) == false);
        MINI_CHECK!(s_vase.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s_vase.cv_count_dir(Some(1)) == 7);
        MINI_CHECK!(m_vase.number_of_vertices() == 660);
        MINI_CHECK!(m_vase.number_of_faces() == 1280);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_vase.get_cv(0,0).unwrap(), &Point::new(1.5, 0.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_vase.get_cv(0,6).unwrap(), &Point::new(1.8, 0.0, 5.0)));

        MINI_CHECK!(s_torus.is_valid());
        MINI_CHECK!(s_torus.is_closed(0) == true);
        MINI_CHECK!(s_torus.is_closed(1) == true);
        MINI_CHECK!(s_torus.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s_torus.cv_count_dir(Some(1)) == 9);
        MINI_CHECK!(m_torus.number_of_vertices() == 640);
        MINI_CHECK!(m_torus.number_of_faces() == 1280);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_torus.get_cv(0,0).unwrap(), &Point::new(20.5, 0.0, 0.0)));

        MINI_CHECK!(s_elbow.is_valid());
        MINI_CHECK!(s_elbow.is_closed(0) == false);
        MINI_CHECK!(s_elbow.is_closed(1) == false);
        MINI_CHECK!(s_elbow.cv_count_dir(Some(0)) == 3);
        MINI_CHECK!(s_elbow.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_elbow.number_of_vertices() == 16);
        MINI_CHECK!(m_elbow.number_of_faces() == 14);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_elbow.get_cv(0,0).unwrap(), &Point::new(29.0, 0.0, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_elbow.get_cv(0,1).unwrap(), &Point::new(29.0, 0.0, 0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_elbow.get_cv(2,0).unwrap(), &Point::new(26.0, 3.0, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_elbow.get_cv(2,1).unwrap(), &Point::new(26.0, 3.0, 0.5)));

        MINI_CHECK!(s_sphere.is_valid());
        MINI_CHECK!(s_sphere.is_closed(0) == true);
        MINI_CHECK!(s_sphere.is_closed(1) == false);
        MINI_CHECK!(s_sphere.is_singular(0) == true);
        MINI_CHECK!(s_sphere.is_singular(2) == true);
        MINI_CHECK!(s_sphere.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s_sphere.cv_count_dir(Some(1)) == 5);
        MINI_CHECK!(m_sphere.number_of_vertices() > 0);
        MINI_CHECK!(m_sphere.number_of_faces() > 0);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sphere.get_cv(0,0).unwrap(), &Point::new(36.0, 0.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sphere.get_cv(0,4).unwrap(), &Point::new(36.0, 0.0, 2.0)));

        MINI_CHECK!(s_cone.is_valid());
        MINI_CHECK!(s_cone.is_closed(0) == true);
        MINI_CHECK!(s_cone.is_closed(1) == false);
        MINI_CHECK!(s_cone.is_singular(0) == true);
        MINI_CHECK!(s_cone.is_singular(2) == false);
        MINI_CHECK!(s_cone.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s_cone.cv_count_dir(Some(1)) == 2);
        MINI_CHECK!(m_cone.number_of_vertices() > 0);
        MINI_CHECK!(m_cone.number_of_faces() > 0);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_cone.get_cv(0,0).unwrap(), &Point::new(44.0, 0.0, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_cone.get_cv(0,1).unwrap(), &Point::new(46.0, 0.0, 0.0)));
    })
}

pub fn run_primitives_nurbssurface_sweep() -> TestResult {
    MINI_TEST!("Nurbssurface Sweep", {
        use crate::primitives::Primitives;
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;

        let rail = NurbsCurve::create(false, 2, &[
            Point::new(0.0, 0.0, 0.0), Point::new(0.0, 5.0, 0.0), Point::new(2.0, 9.0, 0.0),
        ]);
        let profile = Primitives::circle(0.0, 0.0, 0.0, 1.0);
        let s_sweep1 = Primitives::create_sweep1(&rail, &profile);
        let m_sweep1 = s_sweep1.mesh();

        let rail1 = NurbsCurve::create(false, 2, &[
            Point::new(6.0, -1.0, 0.0), Point::new(7.0, 3.0, 0.0), Point::new(8.0, 4.0, 0.0),
        ]);
        let rail2 = NurbsCurve::create(false, 2, &[
            Point::new(10.0, -1.0, 0.0), Point::new(10.0, 3.0, 0.0), Point::new(9.0, 4.0, 0.0),
        ]);
        let shape1 = NurbsCurve::create(false, 2, &[
            Point::new(6.0, -1.0, 0.0), Point::new(8.0, -1.0, 2.0), Point::new(10.0, -1.0, 0.0),
        ]);
        let shape2 = NurbsCurve::create(false, 2, &[
            Point::new(8.0, 4.0, 0.0), Point::new(8.5, 4.0, 1.5), Point::new(9.0, 4.0, 0.0),
        ]);
        let s_sweep2 = Primitives::create_sweep2(&rail1, &rail2, &[shape1, shape2]);
        let m_sweep2 = s_sweep2.mesh();

        MINI_CHECK!(s_sweep1.is_valid());
        MINI_CHECK!(s_sweep1.is_rational());
        MINI_CHECK!(s_sweep1.cv_count_dir(Some(0)) == 9);
        MINI_CHECK!(s_sweep1.cv_count_dir(Some(1)) == 6);
        MINI_CHECK!(m_sweep1.number_of_vertices() > 0);
        MINI_CHECK!(m_sweep1.number_of_faces() > 0);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(0,0).unwrap(), &Point::new(0.888888888888889, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(0,1).unwrap(), &Point::new(0.888635792881381, 1.202714517481950, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(0,2).unwrap(), &Point::new(1.024939251342349, 2.995270183326687, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(0,3).unwrap(), &Point::new(1.646130147308625, 5.890456645823520, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(0,4).unwrap(), &Point::new(2.268126490080245, 7.550043751484516, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(0,5).unwrap(), &Point::new(2.795046402150731, 8.602476824301650, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(1,0).unwrap(), &Point::new(0.888888888888889, 0.000000000000000, -1.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(4,0).unwrap(), &Point::new(-1.111111111111111, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(8,0).unwrap(), &Point::new(0.888888888888889, 0.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep1.get_cv(8,5).unwrap(), &Point::new(2.795046402150731, 8.602476824301650, 0.000000000000000)));

        MINI_CHECK!(s_sweep2.is_valid());
        MINI_CHECK!(s_sweep2.cv_count_dir(Some(0)) == 3);
        MINI_CHECK!(s_sweep2.cv_count_dir(Some(1)) == 6);
        MINI_CHECK!(m_sweep2.number_of_vertices() > 0);
        MINI_CHECK!(m_sweep2.number_of_faces() > 0);
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep2.get_cv(0,0).unwrap(), &Point::new(6.000000000000000, -1.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep2.get_cv(0,5).unwrap(), &Point::new(8.000000000000000, 4.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep2.get_cv(2,0).unwrap(), &Point::new(9.999999999999998, -1.000000000000000, 0.000000000000000)));
        MINI_CHECK!(TOLERANCE.is_point_close(&s_sweep2.get_cv(2,5).unwrap(), &Point::new(8.999999999999998, 4.000000000000000, 0.000000000000000)));
    })
}

pub fn run_primitives_nurbssurface_edge() -> TestResult {
    MINI_TEST!("Nurbssurface Edge", {
        use crate::primitives::Primitives;
        use crate::nurbscurve::NurbsCurve;
        use crate::point::Point;

        let pts_south = vec![
            Point::new(1.0, 20.569076, 0.0), Point::new(1.0, 22.569076, 3.0),
            Point::new(1.0, 25.569076, 3.0), Point::new(1.0, 27.569076, 0.0),
        ];
        let pts_west  = vec![
            Point::new(10.0, 20.569076, 0.0), Point::new(5.5, 20.569076, 3.5),
            Point::new(1.0, 20.569076, 0.0),
        ];
        let pts_north = vec![
            Point::new(10.0, 20.569076, 0.0), Point::new(10.0, 22.569076, 3.0),
            Point::new(10.0, 25.569076, 3.0), Point::new(10.0, 27.569076, 0.0),
        ];
        let pts_east  = vec![
            Point::new(10.0, 27.569076, 0.0), Point::new(5.5, 27.569076, 3.5),
            Point::new(1.0, 27.569076, 0.0),
        ];

        let south = NurbsCurve::create(false, 3, &pts_south);
        let west  = NurbsCurve::create(false, 2, &pts_west);
        let north = NurbsCurve::create(false, 3, &pts_north);
        let east  = NurbsCurve::create(false, 2, &pts_east);

        let surf = Primitives::create_edge(&south, &west, &north, &east);
        let m = surf.mesh();

        MINI_CHECK!(surf.is_valid());
        MINI_CHECK!(m.number_of_faces() > 0);
        MINI_CHECK!(surf.degree(0) == 2);
        MINI_CHECK!(surf.degree(1) == 3);
        MINI_CHECK!(surf.cv_count_dir(Some(0)) == 3);
        MINI_CHECK!(surf.cv_count_dir(Some(1)) == 4);

        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(0, 0).unwrap(), &Point::new(1.0, 20.569076, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(0, 1).unwrap(), &Point::new(1.0, 22.569076, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(0, 2).unwrap(), &Point::new(1.0, 25.569076, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(0, 3).unwrap(), &Point::new(1.0, 27.569076, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(1, 0).unwrap(), &Point::new(5.5, 20.569076, 3.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(1, 1).unwrap(), &Point::new(5.5, 22.569076, 6.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(1, 2).unwrap(), &Point::new(5.5, 25.569076, 6.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(1, 3).unwrap(), &Point::new(5.5, 27.569076, 3.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(2, 0).unwrap(), &Point::new(10.0, 20.569076, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(2, 1).unwrap(), &Point::new(10.0, 22.569076, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(2, 2).unwrap(), &Point::new(10.0, 25.569076, 3.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&surf.get_cv(2, 3).unwrap(), &Point::new(10.0, 27.569076, 0.0)));
    })
}

///////////////////////////////////////////////////////////////////////////////////////////
// Surface-to-mesh subdivision
///////////////////////////////////////////////////////////////////////////////////////////

pub fn run_primitives_mesh_quad_mesh() -> TestResult {
    MINI_TEST!("Mesh Quad Mesh", {
        use crate::primitives::Primitives;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = Primitives::quad_mesh(&cyl, 8, 4);
        MINI_CHECK!(m.number_of_vertices() == 40);
        MINI_CHECK!(m.number_of_faces() == 32);
        MINI_CHECK!(m.is_valid());

        let sph = Primitives::sphere_surface(0.0, 0.0, 0.0, 3.0);
        let m2 = Primitives::quad_mesh(&sph, 8, 4);
        MINI_CHECK!(m2.number_of_vertices() == 26);
        MINI_CHECK!(m2.number_of_faces() == 32);
        MINI_CHECK!(m2.is_valid());
    })
}

pub fn run_primitives_mesh_diamond_mesh() -> TestResult {
    MINI_TEST!("Mesh Diamond Mesh", {
        use crate::primitives::Primitives;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = Primitives::diamond_mesh(&cyl, 8, 4);
        MINI_CHECK!(m.number_of_vertices() == 40);
        MINI_CHECK!(m.number_of_faces() == 20);
        MINI_CHECK!(m.is_valid());

        let sph = Primitives::sphere_surface(0.0, 0.0, 0.0, 3.0);
        let m2 = Primitives::diamond_mesh(&sph, 8, 4);
        MINI_CHECK!(m2.number_of_vertices() == 26);
        MINI_CHECK!(m2.number_of_faces() == 12);
        MINI_CHECK!(m2.is_valid());
    })
}

pub fn run_primitives_mesh_hex_mesh() -> TestResult {
    MINI_TEST!("Mesh Hex Mesh", {
        use crate::primitives::Primitives;

        let cyl = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 5.0);
        let m = Primitives::hex_mesh(&cyl, 6, 4, 1.0/3.0);
        MINI_CHECK!(m.number_of_vertices() == 78);
        MINI_CHECK!(m.number_of_faces() == 15);
        MINI_CHECK!(m.is_valid());

        let sph = Primitives::sphere_surface(0.0, 0.0, 0.0, 3.0);
        let m2 = Primitives::hex_mesh(&sph, 6, 4, 1.0/3.0);
        MINI_CHECK!(m2.number_of_vertices() == 68);
        MINI_CHECK!(m2.number_of_faces() == 15);
        MINI_CHECK!(m2.is_valid());
    })
}

pub fn run_primitives_mesh_cone_subdivisions() -> TestResult {
    MINI_TEST!("Mesh Cone Subdivisions", {
        use crate::primitives::Primitives;

        let cone = Primitives::cone_surface(0.0, 0.0, 0.0, 3.0, 5.0);

        let m1 = Primitives::quad_mesh(&cone, 8, 4);
        MINI_CHECK!(m1.number_of_vertices() == 33);
        MINI_CHECK!(m1.number_of_faces() == 32);
        MINI_CHECK!(m1.is_valid());

        let m2 = Primitives::diamond_mesh(&cone, 8, 4);
        MINI_CHECK!(m2.number_of_vertices() == 33);
        MINI_CHECK!(m2.number_of_faces() == 16);
        MINI_CHECK!(m2.is_valid());

        let m3 = Primitives::hex_mesh(&cone, 6, 4, 1.0/3.0);
        MINI_CHECK!(m3.number_of_vertices() == 73);
        MINI_CHECK!(m3.number_of_faces() == 15);
        MINI_CHECK!(m3.is_valid());
    })
}

pub fn run_primitives_nurbscurve_interpolated() -> TestResult {
    MINI_TEST!("Nurbscurve Interpolated", {
        use crate::primitives::Primitives;
        use crate::point::Point;
        use crate::knot::CurveKnotStyle;

        let points = vec![
            Point::new(14.0, 9.0, 0.0), Point::new(15.342777, 13.734889, 0.0),
            Point::new(21.897914, 32.239195, 0.0), Point::new(24.678472, 0.354555, 0.0),
            Point::new(33.813678, 24.76858, 0.0), Point::new(39.626394, 15.47249, 0.0),
            Point::new(41.0, 13.0, 0.0),
        ];

        let c = Primitives::create_interpolated(&points, CurveKnotStyle::Chord);

        MINI_CHECK!(c.is_valid());
        MINI_CHECK!(c.degree() == 3);
        MINI_CHECK!(c.order() == 4);
        MINI_CHECK!(c.cv_count() == 9);
        MINI_CHECK!(c.is_rational() == false);

        let (d0, d1) = c.domain();
        let knots = c.get_knots();
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(d0), &points[0]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(knots[3]), &points[1]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(knots[4]), &points[2]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(knots[5]), &points[3]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(knots[6]), &points[4]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(knots[7]), &points[5]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.point_at(d1), &points[6]));

        MINI_CHECK!(TOLERANCE.is_point_close(&c.get_cv(0).unwrap(), &points[0]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c.get_cv(8).unwrap(), &points[6]));

        let pts4 = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 2.0, 0.0),
            Point::new(3.0, 1.0, 0.0),
            Point::new(5.0, 3.0, 0.0),
        ];
        let c4 = Primitives::create_interpolated(&pts4, CurveKnotStyle::Chord);
        MINI_CHECK!(c4.is_valid());
        MINI_CHECK!(c4.degree() == 3);
        MINI_CHECK!(c4.cv_count() == 6);
        let (d4_0, d4_1) = c4.domain();
        MINI_CHECK!(TOLERANCE.is_point_close(&c4.point_at(d4_0), &pts4[0]));
        MINI_CHECK!(TOLERANCE.is_point_close(&c4.point_at(d4_1), &pts4[3]));
    })
}

pub fn run_primitives_mesh_tetrahedron() -> TestResult {
    MINI_TEST!("Mesh Tetrahedron", {
        use crate::primitives::Primitives;

        let m = Primitives::tetrahedron(2.0);
        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 4);
        MINI_CHECK!(m.number_of_faces() == 4);
    })
}

pub fn run_primitives_mesh_cube() -> TestResult {
    MINI_TEST!("Mesh Cube", {
        use crate::primitives::Primitives;

        let m = Primitives::cube(2.0);
        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 8);
        MINI_CHECK!(m.number_of_faces() == 6);
    })
}

pub fn run_primitives_mesh_octahedron() -> TestResult {
    MINI_TEST!("Mesh Octahedron", {
        use crate::primitives::Primitives;

        let m = Primitives::octahedron(2.0);
        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 6);
        MINI_CHECK!(m.number_of_faces() == 8);
    })
}

pub fn run_primitives_mesh_icosahedron() -> TestResult {
    MINI_TEST!("Mesh Icosahedron", {
        use crate::primitives::Primitives;

        let m = Primitives::icosahedron(2.0);
        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 12);
        MINI_CHECK!(m.number_of_faces() == 20);
    })
}

pub fn run_primitives_mesh_dodecahedron() -> TestResult {
    MINI_TEST!("Mesh Dodecahedron", {
        use crate::primitives::Primitives;

        let m = Primitives::dodecahedron(2.0);
        MINI_CHECK!(m.is_valid());
        MINI_CHECK!(m.number_of_vertices() == 20);
        MINI_CHECK!(m.number_of_faces() == 12);
    })
}

pub fn run_primitives_nurbssurface_wave() -> TestResult {
    MINI_TEST!("Nurbssurface Wave", {
        use crate::primitives::Primitives;

        let srf = Primitives::wave_surface(10.0, 2.0);
        MINI_CHECK!(srf.is_valid());
        MINI_CHECK!(srf.degree(0) == 3);
        MINI_CHECK!(srf.degree(1) == 3);
        MINI_CHECK!(srf.cv_count_dir(Some(0)) == 13);
        MINI_CHECK!(srf.cv_count_dir(Some(1)) == 13);
        let corner = srf.point_at(0.0, 0.0).unwrap();
        MINI_CHECK!(corner[2].abs() < 0.1);
    })
}

REGISTER_MINI_TEST!("Primitives", "Mesh Arrow", crate::primitives_test::run_primitives_mesh_arrow);
REGISTER_MINI_TEST!("Primitives", "Mesh Cylinder", crate::primitives_test::run_primitives_mesh_cylinder);
REGISTER_MINI_TEST!("Primitives", "Mesh Edge Pipes", crate::primitives_test::run_primitives_mesh_edge_pipes);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Polyline", crate::primitives_test::run_primitives_nurbscurve_polyline);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Circle", crate::primitives_test::run_primitives_nurbscurve_circle);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Ellipse", crate::primitives_test::run_primitives_nurbscurve_ellipse);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Arc", crate::primitives_test::run_primitives_nurbscurve_arc);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Parabola", crate::primitives_test::run_primitives_nurbscurve_parabola);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Hyperbola", crate::primitives_test::run_primitives_nurbscurve_hyperbola);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Spiral", crate::primitives_test::run_primitives_nurbscurve_spiral);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Cylinder", crate::primitives_test::run_primitives_nurbssurface_cylinder);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Cone", crate::primitives_test::run_primitives_nurbssurface_cone);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Sphere", crate::primitives_test::run_primitives_nurbssurface_sphere);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Quad Sphere", crate::primitives_test::run_primitives_nurbssurface_quad_sphere);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Torus", crate::primitives_test::run_primitives_nurbssurface_torus);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Ruled", crate::primitives_test::run_primitives_nurbssurface_ruled);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Planar", crate::primitives_test::run_primitives_nurbssurface_planar);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Extrusion", crate::primitives_test::run_primitives_nurbssurface_extrusion);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Loft", crate::primitives_test::run_primitives_nurbssurface_loft);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Revolve", crate::primitives_test::run_primitives_nurbssurface_revolve);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Sweep", crate::primitives_test::run_primitives_nurbssurface_sweep);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Edge", crate::primitives_test::run_primitives_nurbssurface_edge);
REGISTER_MINI_TEST!("Primitives", "Mesh Quad Mesh", crate::primitives_test::run_primitives_mesh_quad_mesh);
REGISTER_MINI_TEST!("Primitives", "Mesh Diamond Mesh", crate::primitives_test::run_primitives_mesh_diamond_mesh);
REGISTER_MINI_TEST!("Primitives", "Mesh Hex Mesh", crate::primitives_test::run_primitives_mesh_hex_mesh);
REGISTER_MINI_TEST!("Primitives", "Mesh Cone Subdivisions", crate::primitives_test::run_primitives_mesh_cone_subdivisions);
REGISTER_MINI_TEST!("Primitives", "Nurbscurve Interpolated", crate::primitives_test::run_primitives_nurbscurve_interpolated);
REGISTER_MINI_TEST!("Primitives", "Mesh Tetrahedron", crate::primitives_test::run_primitives_mesh_tetrahedron);
REGISTER_MINI_TEST!("Primitives", "Mesh Cube", crate::primitives_test::run_primitives_mesh_cube);
REGISTER_MINI_TEST!("Primitives", "Mesh Octahedron", crate::primitives_test::run_primitives_mesh_octahedron);
REGISTER_MINI_TEST!("Primitives", "Mesh Icosahedron", crate::primitives_test::run_primitives_mesh_icosahedron);
REGISTER_MINI_TEST!("Primitives", "Mesh Dodecahedron", crate::primitives_test::run_primitives_mesh_dodecahedron);
REGISTER_MINI_TEST!("Primitives", "Nurbssurface Wave", crate::primitives_test::run_primitives_nurbssurface_wave);
