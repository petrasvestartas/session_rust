use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::tolerance::PI;


pub fn run_plane_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        // Default constructor - XY plane at origin
        let pl = Plane::default();

        // Origin and axes
        let origin = pl.origin();
        let x_axis = pl.x_axis();
        let y_axis = pl.y_axis();
        let z_axis = pl.z_axis();

        // Plane equation coefficients (ax + by + cz + d = 0)
        let a = pl.a();
        let b = pl.b();
        let c = pl.c();
        let d = pl.d();

        // Index access for axes
        let ax0 = &pl[0];
        let ax1 = &pl[1];
        let ax2 = &pl[2];

        // Minimal and Full String Representation
        let plstr = pl.str();
        let plrepr = pl.repr();

        // Copy (duplicates everything except guid)
        let plcopy = pl.duplicate();

        // From point and normal
        let p = Point::new(0.0, 0.0, 5.0);
        let n = Vector::new(0.0, 0.0, 1.0);
        let pl_pn = Plane::from_point_normal(p, n);

        // From three points
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        ];
        let pl_pts = Plane::from_points(pts);

        // From two points
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 0.0, 0.0);
        let pl_2pts = Plane::from_two_points(p1, p2);

        // Standard planes
        let xy = Plane::xy_plane();
        let yz = Plane::yz_plane();
        let xz = Plane::xz_plane();

        // Translation operators
        let offset = Vector::new(1.0, 2.0, 3.0);
        let mut pl_iadd = Plane::xy_plane();
        pl_iadd += offset.clone();
        let mut pl_isub = Plane::xy_plane();
        pl_isub -= offset.clone();
        let pl_base = Plane::xy_plane();
        let pl_add = pl_base.clone() + offset.clone();
        let pl_sub = pl_base.clone() - offset.clone();

        MINI_CHECK!(pl.name == "my_plane" && !pl.guid().is_empty());
        MINI_CHECK!(TOLERANCE.is_close(origin[0], 0.0) && TOLERANCE.is_close(origin[1], 0.0) && TOLERANCE.is_close(origin[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(x_axis[0], 1.0) && TOLERANCE.is_close(y_axis[1], 1.0) && TOLERANCE.is_close(z_axis[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(a, 0.0) && TOLERANCE.is_close(b, 0.0) && TOLERANCE.is_close(c, 1.0) && TOLERANCE.is_close(d, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(ax0[0], 1.0) && TOLERANCE.is_close(ax1[1], 1.0) && TOLERANCE.is_close(ax2[2], 1.0));
        MINI_CHECK!(plstr == "0.000000, 0.000000, 0.000000\n1.000000, 0.000000, 0.000000\n0.000000, 1.000000, 0.000000\n0.000000, 0.000000, 1.000000");
        MINI_CHECK!(plrepr == "Plane(my_plane, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, Color(blue, 0.0, 0.0, 1.0, 1.0))");
        MINI_CHECK!(plcopy == pl && plcopy.guid() != pl.guid());
        MINI_CHECK!(TOLERANCE.is_close(pl_pn.origin()[2], 5.0) && TOLERANCE.is_close(pl_pn.z_axis()[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_pts.c(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_2pts.x_axis()[0], 1.0));
        MINI_CHECK!(xy.name == "xy_plane" && yz.name == "yz_plane" && xz.name == "xz_plane");
        MINI_CHECK!(TOLERANCE.is_close(pl_iadd.origin()[0], 1.0) && TOLERANCE.is_close(pl_iadd.origin()[1], 2.0) && TOLERANCE.is_close(pl_iadd.origin()[2], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_isub.origin()[0], -1.0) && TOLERANCE.is_close(pl_isub.origin()[2], -3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_add.origin()[2], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_sub.origin()[2], -3.0));
    })
}

pub fn run_plane_is_valid() -> TestResult {
    MINI_TEST!("Is Valid", {
        use crate::Plane;

        let pl = Plane::xy_plane();
        let invalid = Plane::invalid();

        MINI_CHECK!(pl.is_valid());
        MINI_CHECK!(!invalid.is_valid());
    })
}

pub fn run_plane_reverse() -> TestResult {
    MINI_TEST!("Reverse", {
        use crate::Plane;

        let mut pl = Plane::xy_plane();
        pl.reverse();

        MINI_CHECK!(TOLERANCE.is_close(pl.x_axis()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.x_axis()[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.y_axis()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.y_axis()[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.c(), -1.0));
    })
}

pub fn run_plane_rotate() -> TestResult {
    MINI_TEST!("Rotate", {
        use crate::Plane;

        let mut pl = Plane::xy_plane();
        pl.rotate(PI / 2.0);

        MINI_CHECK!(TOLERANCE.is_close(pl.x_axis()[1], 1.0));
    })
}

pub fn run_plane_is_right_hand() -> TestResult {
    MINI_TEST!("Is Right Hand", {
        use crate::Plane;

        let xy = Plane::xy_plane();
        let yz = Plane::yz_plane();
        let xz = Plane::xz_plane();

        MINI_CHECK!(xy.is_right_hand());
        MINI_CHECK!(yz.is_right_hand());
        MINI_CHECK!(xz.is_right_hand());
    })
}

pub fn run_plane_is_same_direction() -> TestResult {
    MINI_TEST!("Is Same Direction", {
        use crate::Plane;

        let p1 = Plane::xy_plane();
        let p2 = Plane::xy_plane();
        let mut p3 = Plane::xy_plane();
        p3.reverse();

        MINI_CHECK!(Plane::is_same_direction(&p1, &p2, true));
        MINI_CHECK!(Plane::is_same_direction(&p1, &p3, true));
        MINI_CHECK!(Plane::is_same_direction(&p1, &p3, false));
    })
}

pub fn run_plane_is_same_position() -> TestResult {
    MINI_TEST!("Is Same Position", {
        use crate::Plane;
        use crate::Vector;

        let p1 = Plane::xy_plane();
        let mut p2 = Plane::xy_plane();
        p2 += Vector::new(0.0, 0.0, 1.0);

        MINI_CHECK!(Plane::is_same_position(&p1, &Plane::xy_plane()));
        MINI_CHECK!(!Plane::is_same_position(&p1, &p2));
    })
}

pub fn run_plane_is_coplanar() -> TestResult {
    MINI_TEST!("Is Coplanar", {
        use crate::Plane;
        use crate::Vector;

        let p1 = Plane::xy_plane();
        let p2 = Plane::xy_plane();
        let mut p3 = Plane::xy_plane();
        p3 += Vector::new(0.0, 0.0, 1.0);

        MINI_CHECK!(Plane::is_coplanar(&p1, &p2, true));
        MINI_CHECK!(!Plane::is_coplanar(&p1, &p3, true));
    })
}

pub fn run_plane_translate_by_normal() -> TestResult {
    MINI_TEST!("Translate By Normal", {
        use crate::Plane;

        let pl = Plane::xy_plane();
        let moved = pl.translate_by_normal(5.0);

        MINI_CHECK!(TOLERANCE.is_close(moved.origin()[2], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.origin()[2], 0.0));
    })
}

pub fn run_plane_base1_base2() -> TestResult {
    MINI_TEST!("Base1 Base2", {
        use crate::Plane;

        let xy = Plane::xy_plane();
        let b1 = xy.base1();
        let b2 = xy.base2();
        MINI_CHECK!(TOLERANCE.is_close(b1.dot(&xy.z_axis()).abs(), 0.0));
        MINI_CHECK!(TOLERANCE.is_close(b2.dot(&xy.z_axis()).abs(), 0.0));
        MINI_CHECK!(TOLERANCE.is_close(b1.dot(&b2), 0.0));
        MINI_CHECK!(TOLERANCE.is_close(b1.magnitude(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(b2.magnitude(), 1.0));
    })
}

pub fn run_plane_transform() -> TestResult {
    MINI_TEST!("Transform", {
        use crate::Plane;
        use crate::Xform;

        let mut pl = Plane::xy_plane();
        pl.xform = Xform::translation(1.0, 2.0, 3.0);
        pl.transform();

        MINI_CHECK!(TOLERANCE.is_close(pl.origin()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.origin()[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.origin()[2], 3.0));
    })
}

pub fn run_plane_transformed() -> TestResult {
    MINI_TEST!("Transformed", {
        use crate::Plane;
        use crate::Xform;

        let mut pl = Plane::xy_plane();
        pl.xform = Xform::translation(1.0, 2.0, 3.0);
        let pl2 = pl.transformed();

        MINI_CHECK!(TOLERANCE.is_close(pl2.origin()[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl2.origin()[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pl2.origin()[2], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.origin()[0], 0.0));
    })
}

pub fn run_plane_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Plane;

        let mut pl = Plane::xy_plane();
        pl.name = "test_plane".to_string();

        //   file_json_dumps()    │ String       │ to JSON string
        //   file_json_loads(s)   │ String       │ from JSON string
        //   file_json_dump(path) │ file         │ write to file
        //   file_json_load(path) │ file         │ read from file

        let fname = "serialization/test_plane.json";
        pl.file_json_dump(fname).unwrap();
        let loaded = Plane::file_json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == "test_plane");
        MINI_CHECK!(TOLERANCE.is_close(loaded.c(), 1.0));
    })
}

pub fn run_plane_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Plane;

        let mut pl = Plane::xy_plane();
        pl.name = "test_plane".to_string();

        let fname = "serialization/test_plane.bin";
        pl.pb_dump(fname);
        let loaded = Plane::pb_load(fname);

        MINI_CHECK!(loaded.name == "test_plane");
        MINI_CHECK!(TOLERANCE.is_close(loaded.c(), 1.0));
    })
}

REGISTER_MINI_TEST!("Plane", "Constructor", crate::plane_test::run_plane_constructor);
REGISTER_MINI_TEST!("Plane", "Is Valid", crate::plane_test::run_plane_is_valid);
REGISTER_MINI_TEST!("Plane", "Reverse", crate::plane_test::run_plane_reverse);
REGISTER_MINI_TEST!("Plane", "Rotate", crate::plane_test::run_plane_rotate);
REGISTER_MINI_TEST!("Plane", "Is Right Hand", crate::plane_test::run_plane_is_right_hand);
REGISTER_MINI_TEST!("Plane", "Is Same Direction", crate::plane_test::run_plane_is_same_direction);
REGISTER_MINI_TEST!("Plane", "Is Same Position", crate::plane_test::run_plane_is_same_position);
REGISTER_MINI_TEST!("Plane", "Is Coplanar", crate::plane_test::run_plane_is_coplanar);
REGISTER_MINI_TEST!("Plane", "Translate By Normal", crate::plane_test::run_plane_translate_by_normal);
REGISTER_MINI_TEST!("Plane", "Base1 Base2", crate::plane_test::run_plane_base1_base2);
REGISTER_MINI_TEST!("Plane", "Transform", crate::plane_test::run_plane_transform);
REGISTER_MINI_TEST!("Plane", "Transformed", crate::plane_test::run_plane_transformed);
REGISTER_MINI_TEST!("Plane", "Json Roundtrip", crate::plane_test::run_plane_json_roundtrip);
REGISTER_MINI_TEST!("Plane", "Protobuf Roundtrip", crate::plane_test::run_plane_protobuf_roundtrip);

pub fn run_plane_has_on_negative_side() -> TestResult {
    MINI_TEST!("Has On Negative Side", {
        use crate::{Plane, Point};
        let pl = Plane::xy_plane();
        let above = Point::new(0.0, 0.0, 1.0);
        let below = Point::new(0.0, 0.0, -1.0);
        MINI_CHECK!(pl.has_on_negative_side(&below));
        MINI_CHECK!(!pl.has_on_negative_side(&above));
    })
}
REGISTER_MINI_TEST!("Plane", "Has On Negative Side", crate::plane_test::run_plane_has_on_negative_side);
