use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::tolerance::PI;


pub fn run_plane_constructor() -> TestResult {
    MINI_TEST!("constructor", {
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
        let pts = vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0)];
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

        // In-place add
        let mut pl_iadd = Plane::xy_plane();
        pl_iadd += offset.clone();

        // In-place subtract
        let mut pl_isub = Plane::xy_plane();
        pl_isub -= offset.clone();

        // Copy add/subtract
        let pl_base = Plane::xy_plane();
        let pl_add = pl_base.clone() + offset.clone();
        let pl_sub = pl_base.clone() - offset.clone();

        MINI_CHECK!(pl.name == "my_plane" && !pl.guid.is_empty());
        MINI_CHECK!(TOLERANCE.is_close(origin[0], 0.0) && TOLERANCE.is_close(origin[1], 0.0) && TOLERANCE.is_close(origin[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(x_axis[0], 1.0) && TOLERANCE.is_close(x_axis[1], 0.0) && TOLERANCE.is_close(x_axis[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(y_axis[0], 0.0) && TOLERANCE.is_close(y_axis[1], 1.0) && TOLERANCE.is_close(y_axis[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(z_axis[0], 0.0) && TOLERANCE.is_close(z_axis[1], 0.0) && TOLERANCE.is_close(z_axis[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(a, 0.0) && TOLERANCE.is_close(b, 0.0) && TOLERANCE.is_close(c, 1.0) && TOLERANCE.is_close(d, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(ax0[0], 1.0) && TOLERANCE.is_close(ax1[1], 1.0) && TOLERANCE.is_close(ax2[2], 1.0));
        MINI_CHECK!(plstr == "0.000000, 0.000000, 0.000000");
        MINI_CHECK!(plrepr == "Plane(my_plane, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000)");
        MINI_CHECK!(plcopy == pl && plcopy.guid != pl.guid);
        MINI_CHECK!(TOLERANCE.is_close(pl_pn.origin()[2], 5.0) && TOLERANCE.is_close(pl_pn.z_axis()[2], 1.0) && TOLERANCE.is_close(pl_pn.d(), -5.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_pts.c(), 1.0) && TOLERANCE.is_close(pl_pts.d(), 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_2pts.origin()[0], 0.0) && TOLERANCE.is_close(pl_2pts.x_axis()[0], 1.0));
        MINI_CHECK!(xy.name == "xy_plane" && TOLERANCE.is_close(xy.c(), 1.0));
        MINI_CHECK!(yz.name == "yz_plane" && TOLERANCE.is_close(yz.a(), 1.0));
        MINI_CHECK!(xz.name == "xz_plane" && TOLERANCE.is_close(xz.b(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_iadd.origin()[0], 1.0) && TOLERANCE.is_close(pl_iadd.origin()[1], 2.0) && TOLERANCE.is_close(pl_iadd.origin()[2], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_isub.origin()[0], -1.0) && TOLERANCE.is_close(pl_isub.origin()[1], -2.0) && TOLERANCE.is_close(pl_isub.origin()[2], -3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_add.origin()[2], 3.0) && TOLERANCE.is_close(pl_base.origin()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_sub.origin()[2], -3.0));
    })
}

pub fn run_plane_reverse() -> TestResult {
    MINI_TEST!("reverse", {
        use crate::Plane;

        // Reverse flips normal and swaps x/y axes
        let mut pl = Plane::xy_plane();
        pl.reverse();

        MINI_CHECK!(TOLERANCE.is_close(pl.x_axis()[0], 0.0) && TOLERANCE.is_close(pl.x_axis()[1], 1.0) && TOLERANCE.is_close(pl.x_axis()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.y_axis()[0], 1.0) && TOLERANCE.is_close(pl.y_axis()[1], 0.0) && TOLERANCE.is_close(pl.y_axis()[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pl.c(), -1.0));
    })
}

pub fn run_plane_rotate() -> TestResult {
    MINI_TEST!("rotate", {
        use crate::Plane;

        // Rotate plane 90 degrees around its normal
        let mut pl = Plane::xy_plane();
        pl.rotate(PI / 2.0);

        MINI_CHECK!(TOLERANCE.is_close(pl.x_axis()[1], 1.0));
    })
}

pub fn run_plane_is_right_hand() -> TestResult {
    MINI_TEST!("is_right_hand", {
        use crate::Plane;

        // All standard planes should be right-handed
        let xy = Plane::xy_plane();
        let yz = Plane::yz_plane();
        let xz = Plane::xz_plane();
        let mut default_pl = Plane::default();

        let xy_rh = xy.is_right_hand();
        let yz_rh = yz.is_right_hand();
        let xz_rh = xz.is_right_hand();
        let default_rh = default_pl.is_right_hand();

        // After reverse, should still be right-handed
        default_pl.reverse();
        let reversed_rh = default_pl.is_right_hand();

        // After rotate, should still be right-handed
        default_pl.rotate(PI / 4.0);
        let rotated_rh = default_pl.is_right_hand();

        MINI_CHECK!(xy_rh == true);
        MINI_CHECK!(yz_rh == true);
        MINI_CHECK!(xz_rh == true);
        MINI_CHECK!(default_rh == true);
        MINI_CHECK!(reversed_rh == true);
        MINI_CHECK!(rotated_rh == true);
    })
}

pub fn run_plane_is_coplanar() -> TestResult {
    MINI_TEST!("is_coplanar", {
        use crate::Plane;
        use crate::Vector;

        // Same direction (parallel planes)
        let p1 = Plane::xy_plane();
        let p2 = Plane::xy_plane();
        let same_dir = Plane::is_same_direction(&p1, &p2, true);

        // Flipped direction
        let mut p3 = Plane::xy_plane();
        p3.reverse();
        let same_dir_flipped = Plane::is_same_direction(&p1, &p3, true);
        let same_dir_strict = Plane::is_same_direction(&p1, &p3, false);

        // Same position
        let mut p4 = Plane::xy_plane();
        let same_pos = Plane::is_same_position(&p1, &p4);
        p4 += Vector::new(0.0, 0.0, 1.0);
        let diff_pos = Plane::is_same_position(&p1, &p4);

        // Coplanar
        let p5 = Plane::xy_plane();
        let mut p6 = Plane::xy_plane();
        let coplanar = Plane::is_coplanar(&p5, &p6, true);
        p6.reverse();
        let coplanar_reversed = Plane::is_coplanar(&p5, &p6, true);
        p6 += Vector::new(0.0, 0.0, 1.0);
        let not_coplanar = Plane::is_coplanar(&p5, &p6, true);

        MINI_CHECK!(same_dir == true);
        MINI_CHECK!(same_dir_flipped == true);
        MINI_CHECK!(same_dir_strict == false);
        MINI_CHECK!(same_pos == true);
        MINI_CHECK!(diff_pos == false);
        MINI_CHECK!(coplanar == true);
        MINI_CHECK!(coplanar_reversed == true);
        MINI_CHECK!(not_coplanar == false);
    })
}

pub fn run_plane_transform() -> TestResult {
    MINI_TEST!("transform", {
        use crate::Plane;
        use crate::Xform;

        // Transform - in-place transformation
        let mut pl = Plane::xy_plane();
        pl.xform = Xform::translation(1.0, 2.0, 3.0);
        pl.transform();

        // Transformed - returns new plane
        let mut pl2 = Plane::xy_plane();
        pl2.xform = Xform::translation(1.0, 2.0, 3.0);
        let pl3 = pl2.transformed();

        MINI_CHECK!(TOLERANCE.is_close(pl.origin()[0], 1.0) && TOLERANCE.is_close(pl.origin()[1], 2.0) && TOLERANCE.is_close(pl.origin()[2], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl3.origin()[0], 1.0) && TOLERANCE.is_close(pl3.origin()[1], 2.0) && TOLERANCE.is_close(pl3.origin()[2], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(pl2.origin()[0], 0.0) && TOLERANCE.is_close(pl2.origin()[1], 0.0) && TOLERANCE.is_close(pl2.origin()[2], 0.0));
    })
}

pub fn run_plane_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::Plane;

        let mut pl = Plane::xy_plane();
        pl.name = "test_plane".to_string();

        let fname = "test_plane.json";
        pl.json_dump(fname).unwrap();
        let loaded = Plane::json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == "test_plane");
        MINI_CHECK!(TOLERANCE.is_close(loaded.c(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.d(), 0.0));
    })
}

#[cfg(feature = "protobuf")]
pub fn run_plane_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::Plane;

        let mut pl = Plane::xy_plane();
        pl.name = "test_plane".to_string();

        // protobuf_dump(fname) / protobuf_load(fname) - file-based serialization
        let fname = "test_plane.bin";
        pl.protobuf_dump(fname);
        let loaded = Plane::protobuf_load(fname);

        MINI_CHECK!(loaded.name == "test_plane");
        MINI_CHECK!(TOLERANCE.is_close(loaded.c(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.d(), 0.0));
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Plane", "constructor", crate::plane_test::run_plane_constructor);
REGISTER_MINI_TEST!("Plane", "reverse", crate::plane_test::run_plane_reverse);
REGISTER_MINI_TEST!("Plane", "rotate", crate::plane_test::run_plane_rotate);
REGISTER_MINI_TEST!("Plane", "is_right_hand", crate::plane_test::run_plane_is_right_hand);
REGISTER_MINI_TEST!("Plane", "is_coplanar", crate::plane_test::run_plane_is_coplanar);
REGISTER_MINI_TEST!("Plane", "transform", crate::plane_test::run_plane_transform);
REGISTER_MINI_TEST!("Plane", "json_roundtrip", crate::plane_test::run_plane_json_roundtrip);
#[cfg(feature = "protobuf")]
REGISTER_MINI_TEST!("Plane", "protobuf_roundtrip", crate::plane_test::run_plane_protobuf_roundtrip);
