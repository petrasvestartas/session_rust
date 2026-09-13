use crate::mini_test::TestResult;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_split_curve_by_curves() -> TestResult {
    MINI_TEST!("Split Curve By Curves", {
        use crate::simple_split::split_curve_by_curves;
        use crate::{NurbsCurve, Point, Primitives};
        let curve =
            NurbsCurve::create(false, 1, &[Point::new(-2., 0., 0.), Point::new(2., 0., 0.)]);
        let cutter =
            NurbsCurve::create(false, 1, &[Point::new(0., -2., 0.), Point::new(0., 2., 0.)]);
        let pieces = split_curve_by_curves(&curve, &[cutter.clone()], 1e-6).unwrap();
        MINI_CHECK!(pieces.len() == 2);
        MINI_CHECK!(
            pieces[0]
                .point_at_start()
                .distance(&Point::new(-2., 0., 0.), None)
                < 1e-6
        );
        MINI_CHECK!(
            pieces[0]
                .point_at_end()
                .distance(&Point::new(0., 0., 0.), None)
                < 1e-6
        );
        MINI_CHECK!(
            pieces[1]
                .point_at_end()
                .distance(&Point::new(2., 0., 0.), None)
                < 1e-6
        );
        MINI_CHECK!(curve.point_at_end().distance(&Point::new(2., 0., 0.), None) < 1e-6);
        let skew = NurbsCurve::create(false, 1, &[Point::new(0., -2., 1.), Point::new(0., 2., 1.)]);
        MINI_CHECK!(split_curve_by_curves(&curve, &[skew], 1e-6).unwrap().len() == 1);
        let crossing = NurbsCurve::create(
            false,
            1,
            &[
                Point::new(-2., -2., 0.),
                Point::new(2., 2., 0.),
                Point::new(-2., 2., 0.),
                Point::new(2., -2., 0.),
            ],
        );
        let short_cut = NurbsCurve::create(
            false,
            1,
            &[Point::new(0., -0.2, 0.), Point::new(0., 0.2, 0.)],
        );
        MINI_CHECK!(
            split_curve_by_curves(&crossing, &[short_cut], 1e-6)
                .unwrap()
                .len()
                == 3
        );
        let circle = Primitives::circle(0., 0., 0., 1.);
        let chord = NurbsCurve::create(
            false,
            1,
            &[Point::new(-2., 0.5, 0.), Point::new(2., 0.5, 0.)],
        );
        let arcs = split_curve_by_curves(&circle, &[chord], 1e-6).unwrap();
        MINI_CHECK!(arcs.len() == 2);
        MINI_CHECK!(arcs[0].is_rational() && arcs[1].is_rational());
        let tangent =
            NurbsCurve::create(false, 1, &[Point::new(-2., 1., 0.), Point::new(2., 1., 0.)]);
        MINI_CHECK!(
            split_curve_by_curves(&circle, &[tangent], 1e-6)
                .unwrap()
                .len()
                == 1
        );
        let rejected = split_curve_by_curves(&curve, &[curve.clone()], 1e-6).is_err();
        MINI_CHECK!(rejected);
        let rejected = split_curve_by_curves(&curve, &[cutter], f64::NAN).is_err();
        MINI_CHECK!(rejected);
    })
}

pub fn run_split_brep_face_by_curves() -> TestResult {
    MINI_TEST!("Split BRep Face By Curves", {
        use crate::simple_split::split_brep_face_by_curves;
        use crate::{BRep, NurbsCurve, Point, Primitives};
        let box_ = BRep::create_box(10., 10., 10.);
        let surface = box_.m_surfaces[0].clone();
        let a = surface.get_cv(0, 0).unwrap();
        let u = surface.get_cv(1, 0).unwrap();
        let v = surface.get_cv(0, 1).unwrap();
        let mapped = |x: f64, y: f64| {
            Point::new(
                a[0] + x * (u[0] - a[0]) + y * (v[0] - a[0]),
                a[1] + x * (u[1] - a[1]) + y * (v[1] - a[1]),
                a[2] + x * (u[2] - a[2]) + y * (v[2] - a[2]),
            )
        };
        let cutter = NurbsCurve::create(false, 1, &[mapped(0.5, -1.), mapped(0.5, 2.)]);
        let split = split_brep_face_by_curves(&box_, 0, &[cutter.clone()], 1e-6).unwrap();
        MINI_CHECK!(split.face_count() == 7);
        MINI_CHECK!(split.is_valid() && split.is_solid());
        MINI_CHECK!(box_.face_count() == 6);
        let meshes = split.face_meshes_q(Some((20., 0.005)));
        MINI_CHECK!((meshes[0].area() - 50.).abs() < 1e-6);
        MINI_CHECK!((meshes[6].area() - 50.).abs() < 1e-6);
        let neighbor_area: f64 = (1..6).map(|i| meshes[i].area()).sum();
        MINI_CHECK!((neighbor_area - 500.).abs() < 1e-6);
        let closed = NurbsCurve::create(
            false,
            3,
            &[
                mapped(0.2, 0.3),
                mapped(0.8, 0.3),
                mapped(0.5, 0.9),
                mapped(0.2, 0.3),
            ],
        );
        let island = split_brep_face_by_curves(&box_, 0, &[closed], 1e-6).unwrap();
        MINI_CHECK!(island.face_count() == 7 && island.is_solid());

        let crossing = NurbsCurve::create(false, 1, &[mapped(-1., 0.5), mapped(2., 0.5)]);
        let quarters =
            split_brep_face_by_curves(&box_, 0, &[cutter, crossing.clone()], 1e-6).unwrap();
        MINI_CHECK!(quarters.face_count() == 9 && quarters.is_solid());
        let repeated = split_brep_face_by_curves(&split, 0, &[crossing], 1e-6).unwrap();
        MINI_CHECK!(repeated.face_count() == 8 && repeated.is_solid());
        let loop_ = NurbsCurve::create(
            false,
            1,
            &[
                mapped(0.2, 0.2),
                mapped(0.8, 0.2),
                mapped(0.8, 0.8),
                mapped(0.2, 0.8),
                mapped(0.2, 0.2),
            ],
        );
        let regions = split_brep_face_by_curves(&box_, 0, &[loop_], 1e-6).unwrap();
        MINI_CHECK!(regions.face_count() == 7 && regions.is_solid());
        let restored = BRep::file_json_loads(&quarters.file_json_dumps());
        MINI_CHECK!(restored.face_count() == 9 && restored.is_solid());
        let protobuf = BRep::pb_loads(&quarters.pb_dumps()).unwrap();
        MINI_CHECK!(protobuf.face_count() == 9 && protobuf.is_solid());
        let outer = NurbsCurve::create(
            false,
            1,
            &[
                Point::new(0., 0., 0.),
                Point::new(10., 0., 0.),
                Point::new(10., 10., 0.),
                Point::new(0., 10., 0.),
                Point::new(0., 0., 0.),
            ],
        );
        let inner = NurbsCurve::create(
            false,
            1,
            &[
                Point::new(3., 3., 0.),
                Point::new(7., 3., 0.),
                Point::new(7., 7., 0.),
                Point::new(3., 7., 0.),
                Point::new(3., 3., 0.),
            ],
        );
        let ring = BRep::from_nurbscurves(&[outer], &[vec![inner]]);
        let through = NurbsCurve::create(
            false,
            1,
            &[Point::new(5., -1., 0.), Point::new(5., 11., 0.)],
        );
        let divided = split_brep_face_by_curves(&ring, 0, &[through], 1e-6).unwrap();
        MINI_CHECK!(divided.face_count() == 2);
        let outside = NurbsCurve::create(
            false,
            1,
            &[Point::new(1., -1., 0.), Point::new(1., 11., 0.)],
        );
        let preserved = split_brep_face_by_curves(&ring, 0, &[outside], 1e-6).unwrap();
        let mut holes = 0;
        for face in &preserved.m_faces {
            holes += face.wires.len() - 1;
        }
        MINI_CHECK!(preserved.face_count() == 2 && holes == 1);
        let disk = BRep::from_nurbscurves(&[Primitives::circle(0., 0., 0., 5.)], &[]);
        let chord = NurbsCurve::create(
            false,
            1,
            &[Point::new(-6., 1.2, 0.), Point::new(6., 1.2, 0.)],
        );
        let halves = split_brep_face_by_curves(&disk, 0, &[chord], 1e-6).unwrap();
        MINI_CHECK!(halves.face_count() == 2);
        MINI_CHECK!(disk.face_count() == 1);
        let cylinder = BRep::create_cylinder(5., 10.);
        let body = &cylinder.m_surfaces[cylinder.m_faces[0].surface_index as usize];
        let domain = body.domain(0).unwrap();
        let generator = body.iso_curve(1, (domain.0 + domain.1) * 0.5).unwrap();
        let seamed = split_brep_face_by_curves(&cylinder, 0, &[generator], 1e-6).unwrap();
        MINI_CHECK!(seamed.face_count() == 4 && seamed.is_solid());
        MINI_CHECK!(cylinder.face_count() == 3);
    })
}

pub fn run_split_surface_by_curves() -> TestResult {
    MINI_TEST!("Split Surface By Curves", {
        use crate::simple_split::split_surface_by_curves;
        use crate::{BRep, NurbsCurve, Point};
        let surface = BRep::create_box(10., 10., 10.).m_surfaces[0].clone();
        let a = surface.get_cv(0, 0).unwrap();
        let u = surface.get_cv(1, 0).unwrap();
        let v = surface.get_cv(0, 1).unwrap();
        let mapped = |x: f64, y: f64| {
            Point::new(
                a[0] + x * (u[0] - a[0]) + y * (v[0] - a[0]),
                a[1] + x * (u[1] - a[1]) + y * (v[1] - a[1]),
                a[2] + x * (u[2] - a[2]) + y * (v[2] - a[2]),
            )
        };
        let cutter = NurbsCurve::create(false, 1, &[mapped(0.5, -1.), mapped(0.5, 2.)]);
        let split = split_surface_by_curves(&surface, &[cutter], 1e-6).unwrap();
        MINI_CHECK!(split.face_count() == 2);
        MINI_CHECK!(split.is_valid() && !split.is_solid());
        let outside = NurbsCurve::create(false, 1, &[mapped(2., -1.), mapped(2., 2.)]);
        let untouched = split_surface_by_curves(&surface, &[outside.clone()], 1e-6).unwrap();
        MINI_CHECK!(untouched.face_count() == 1);
        MINI_CHECK!(surface.is_valid());
        let mut invalid = surface.clone();
        invalid.set_cv(0, 0, &Point::new(f64::NAN, 0., 0.));
        MINI_CHECK!(split_surface_by_curves(&invalid, &[outside], 1e-6).is_err());
    })
}
REGISTER_MINI_TEST!(
    "SimpleSplit",
    "Split Curve By Curves",
    crate::simple_split_test::run_split_curve_by_curves
);
REGISTER_MINI_TEST!(
    "SimpleSplit",
    "Split BRep Face By Curves",
    crate::simple_split_test::run_split_brep_face_by_curves
);
REGISTER_MINI_TEST!(
    "SimpleSplit",
    "Split Surface By Curves",
    crate::simple_split_test::run_split_surface_by_curves
);

pub fn run_split_line_by_curves() -> TestResult {
    MINI_TEST!("Split Line By Curves", {
        use crate::simple_split::split_line_by_curves;
        use crate::{Line, NurbsCurve, Point};
        let mut line = Line::from_points(&Point::new(-2., 0., 0.), &Point::new(2., 0., 0.));
        line.name = "retained".into();
        line.width = 3.;
        line.dash = vec![1., 2.];
        let cutter =
            NurbsCurve::create(false, 1, &[Point::new(0., -2., 0.), Point::new(0., 2., 0.)]);
        let pieces = split_line_by_curves(&line, &[cutter], 1e-6).unwrap();
        MINI_CHECK!(pieces.len() == 2);
        MINI_CHECK!(
            pieces[0]
                .point_at(1.)
                .distance(&Point::new(0., 0., 0.), None)
                < 1e-6
        );
        MINI_CHECK!(
            pieces[1]
                .point_at(0.)
                .distance(&Point::new(0., 0., 0.), None)
                < 1e-6
        );
        MINI_CHECK!(
            pieces[0].name == line.name
                && pieces[0].width == line.width
                && pieces[0].dash == line.dash
        );
        MINI_CHECK!(line.length() == 4.);
    })
}
pub fn run_split_polyline_by_curves() -> TestResult {
    MINI_TEST!("Split Polyline By Curves", {
        use crate::simple_split::split_polyline_by_curves;
        use crate::{NurbsCurve, Point, Polyline};
        let mut polyline = Polyline::new(vec![
            Point::new(-2., 0., 0.),
            Point::new(2., 0., 0.),
            Point::new(2., 3., 0.),
        ]);
        polyline.name = "retained".into();
        polyline.width = 3.;
        polyline.dash = vec![1., 2.];
        let cutter =
            NurbsCurve::create(false, 1, &[Point::new(0., -2., 0.), Point::new(0., 2., 0.)]);
        let pieces = split_polyline_by_curves(&polyline, &[cutter], 1e-6).unwrap();
        MINI_CHECK!(pieces.len() == 2);
        MINI_CHECK!(pieces[0].point_count() == 2 && pieces[1].point_count() == 3);
        MINI_CHECK!(
            pieces[1]
                .get_point(1)
                .unwrap()
                .distance(&Point::new(2., 0., 0.), None)
                < 1e-6
        );
        MINI_CHECK!(
            pieces[1]
                .get_point(2)
                .unwrap()
                .distance(&Point::new(2., 3., 0.), None)
                < 1e-6
        );
        MINI_CHECK!(
            pieces[0].name == polyline.name
                && pieces[0].width == polyline.width
                && pieces[0].dash == polyline.dash
        );
        MINI_CHECK!(polyline.point_count() == 3);
    })
}
REGISTER_MINI_TEST!(
    "SimpleSplit",
    "Split Line By Curves",
    crate::simple_split_test::run_split_line_by_curves
);
REGISTER_MINI_TEST!(
    "SimpleSplit",
    "Split Polyline By Curves",
    crate::simple_split_test::run_split_polyline_by_curves
);
