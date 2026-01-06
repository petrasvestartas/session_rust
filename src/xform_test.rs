use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::tolerance::PI;

pub fn run_xform_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::Xform;
        use crate::Point;

        // Constructor (identity by default)
        let x = Xform::new();

        // Matrix access
        let m00 = x.m[0];
        let m11 = x.m[5];
        let m22 = x.m[10];
        let m33 = x.m[15];

        // Check identity
        let is_id = x.is_identity();

        // From matrix constructor
        let xfrom = Xform::from_matrix([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 10.0, 15.0, 1.0]);

        // Minimal and Full String Representation
        let xstr = x.str();
        let xrepr = x.repr();

        // Copy (duplicates everything except guid)
        let xcopy = x.duplicate();
        let xother = Xform::new();

        // Equality operators
        let x_eq = x == xother;
        let x_ne = x != xfrom;

        // Matrix multiplication (*)
        let t = Xform::translation(10.0, 0.0, 0.0);
        let s = Xform::scaling(2.0, 1.0, 1.0);
        let combined = &t * &s;
        let p = Point::new(1.0, 0.0, 0.0);
        let result = combined.transformed_point(&p);

        // In-place multiplication (*=)
        let mut t2 = Xform::translation(10.0, 0.0, 0.0);
        t2 *= s;
        let result2 = t2.transformed_point(&p);

        MINI_CHECK!(x.name == "my_xform" && !x.guid.is_empty());
        MINI_CHECK!(m00 == 1.0 && m11 == 1.0 && m22 == 1.0 && m33 == 1.0);
        MINI_CHECK!(is_id == true);
        MINI_CHECK!(xfrom.m[12] == 5.0 && xfrom.m[13] == 10.0 && xfrom.m[14] == 15.0);
        MINI_CHECK!(xstr.contains("1.000000"));
        MINI_CHECK!(xrepr.contains("Xform(") && xrepr.contains("my_xform"));
        MINI_CHECK!(xcopy == x && xcopy.guid != x.guid);
        MINI_CHECK!(x_eq == true && x_ne == true);
        // (1,0,0) * scale(2,1,1) = (2,0,0), then translate(10,0,0) = (12,0,0)
        MINI_CHECK!(TOLERANCE.is_close(result[0], 12.0) && TOLERANCE.is_close(result[1], 0.0) && TOLERANCE.is_close(result[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result2[0], 12.0) && TOLERANCE.is_close(result2[1], 0.0) && TOLERANCE.is_close(result2[2], 0.0));
    })
}

pub fn run_xform_translation() -> TestResult {
    MINI_TEST!("translation", {
        use crate::Xform;
        use crate::Point;

        // Translation matrix
        let t = Xform::translation(1.0, 2.0, 3.0);

        // Apply to point
        let p = Point::new(4.0, 5.0, 6.0);
        let tp = t.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(tp[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[1], 7.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[2], 9.0));
    })
}

pub fn run_xform_scaling() -> TestResult {
    MINI_TEST!("scaling", {
        use crate::Xform;
        use crate::Point;

        // Scaling matrix
        let s = Xform::scaling(2.0, 3.0, 4.0);

        // Apply to point
        let p = Point::new(1.0, 1.0, 1.0);
        let sp = s.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(sp[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(sp[1], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(sp[2], 4.0));
    })
}

pub fn run_xform_rotation() -> TestResult {
    MINI_TEST!("rotation", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;

        // Rotation around X axis by 90 degrees
        let rx = Xform::rotation_x(PI / 2.0);
        // Apply to point (0,1,0) -> (0,0,1)
        let px = Point::new(0.0, 1.0, 0.0);
        let rpx = rx.transformed_point(&px);

        // Rotation around Y axis by 90 degrees
        let ry = Xform::rotation_y(PI / 2.0);
        // Apply to point (0,0,1) -> (1,0,0)
        let py = Point::new(0.0, 0.0, 1.0);
        let rpy = ry.transformed_point(&py);

        // Rotation around Z axis by 90 degrees
        let rz = Xform::rotation_z(PI / 2.0);
        // Apply to point (1,0,0) -> (0,1,0)
        let pz = Point::new(1.0, 0.0, 0.0);
        let rpz = rz.transformed_point(&pz);

        // Rotation around arbitrary axis (1,1,1) by 120 degrees
        // This cycles x->y->z->x
        let axis = Vector::new(1.0, 1.0, 1.0);
        let r = Xform::rotation(&axis, 2.0 * PI / 3.0);
        // Apply to point (1,0,0) -> (0,1,0)
        let p = Point::new(1.0, 0.0, 0.0);
        let rp = r.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(rpx[0], 0.0) && TOLERANCE.is_close(rpx[1], 0.0) && TOLERANCE.is_close(rpx[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(rpy[0], 1.0) && TOLERANCE.is_close(rpy[1], 0.0) && TOLERANCE.is_close(rpy[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rpz[0], 0.0) && TOLERANCE.is_close(rpz[1], 1.0) && TOLERANCE.is_close(rpz[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[0], 0.0) && TOLERANCE.is_close(rp[1], 1.0) && TOLERANCE.is_close(rp[2], 0.0));
    })
}

pub fn run_xform_inverse() -> TestResult {
    MINI_TEST!("inverse", {
        use crate::Xform;

        // Create composite transformation
        let t = Xform::translation(1.0, 2.0, 3.0);
        let s = Xform::scaling(2.0, 2.0, 2.0);
        let composite = &t * &s;

        // Compute inverse
        let inv = composite.inverse().unwrap();

        // Multiply should give identity
        let result = &composite * &inv;

        MINI_CHECK!(result.is_identity());
    })
}

pub fn run_xform_transform_geometry() -> TestResult {
    MINI_TEST!("transform_geometry", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Line;
        use crate::Plane;
        use crate::Polyline;

        // Simple translation by (10, 20, 30)
        let t = Xform::translation(10.0, 20.0, 30.0);

        // Transform Point: (1,2,3) -> (11,22,33)
        let mut pt = Point::new(1.0, 2.0, 3.0);
        pt.xform = t.duplicate();
        let pt_transformed = pt.transformed();

        // Transform Vector: translation should NOT affect vectors
        let v = Vector::new(1.0, 0.0, 0.0);
        let v_transformed = t.transformed_vector(&v);

        // Transform Line: (0,0,0)-(1,0,0) -> (10,20,30)-(11,20,30)
        let mut ln = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        ln.xform = t.duplicate();
        let ln_transformed = ln.transformed();

        // Transform Plane: origin (0,0,0) -> (10,20,30)
        let mut pl = Plane::new(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        pl.xform = t.duplicate();
        let pl_transformed = pl.transformed();

        // Transform Polyline: 3 points translated
        let mut poly = Polyline::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(1.0, 1.0, 0.0)]);
        poly.xform = t.duplicate();
        let poly_transformed = poly.transformed();
        let pts = poly_transformed.get_points();

        MINI_CHECK!(TOLERANCE.is_close(pt_transformed[0], 11.0) && TOLERANCE.is_close(pt_transformed[1], 22.0) && TOLERANCE.is_close(pt_transformed[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(v_transformed[0], 1.0) && TOLERANCE.is_close(v_transformed[1], 0.0) && TOLERANCE.is_close(v_transformed[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(ln_transformed[0], 10.0) && TOLERANCE.is_close(ln_transformed[1], 20.0) && TOLERANCE.is_close(ln_transformed[2], 30.0));
        MINI_CHECK!(TOLERANCE.is_close(ln_transformed[3], 11.0) && TOLERANCE.is_close(ln_transformed[4], 20.0) && TOLERANCE.is_close(ln_transformed[5], 30.0));
        MINI_CHECK!(TOLERANCE.is_close(pl_transformed.origin()[0], 10.0) && TOLERANCE.is_close(pl_transformed.origin()[1], 20.0) && TOLERANCE.is_close(pl_transformed.origin()[2], 30.0));
        MINI_CHECK!(TOLERANCE.is_close(pts[0][0], 10.0) && TOLERANCE.is_close(pts[0][1], 20.0) && TOLERANCE.is_close(pts[0][2], 30.0));
        MINI_CHECK!(TOLERANCE.is_close(pts[1][0], 11.0) && TOLERANCE.is_close(pts[1][1], 20.0) && TOLERANCE.is_close(pts[1][2], 30.0));
        MINI_CHECK!(TOLERANCE.is_close(pts[2][0], 11.0) && TOLERANCE.is_close(pts[2][1], 21.0) && TOLERANCE.is_close(pts[2][2], 30.0));
    })
}

pub fn run_xform_change_basis() -> TestResult {
    MINI_TEST!("change_basis", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;

        // System 0: standard XY plane at origin
        let origin_0 = Point::new(0.0, 0.0, 0.0);
        let x_axis_0 = Vector::new(1.0, 0.0, 0.0);
        let y_axis_0 = Vector::new(0.0, 1.0, 0.0);
        let z_axis_0 = Vector::new(0.0, 0.0, 1.0);

        // System 1: translated and rotated 90 degrees around Z
        let origin_1 = Point::new(10.0, 20.0, 0.0);
        let x_axis_1 = Vector::new(0.0, 1.0, 0.0);
        let y_axis_1 = Vector::new(-1.0, 0.0, 0.0);
        let z_axis_1 = Vector::new(0.0, 0.0, 1.0);

        // Transform maps points FROM system 1 TO system 0
        let xform = Xform::change_basis(&origin_1, &x_axis_1, &y_axis_1, &z_axis_1, &origin_0, &x_axis_0, &y_axis_0, &z_axis_0);

        // Point at origin_1 should map to origin_0
        let p = Point::new(10.0, 20.0, 0.0);
        let tp = xform.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(tp[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[2], 0.0));
    })
}

pub fn run_xform_plane_to_plane() -> TestResult {
    MINI_TEST!("plane_to_plane", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Plane;

        // Source plane at origin, XY plane
        let plane_from = Plane::new(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));

        // Target plane translated and rotated
        let plane_to = Plane::new(Point::new(10.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0), Vector::new(-1.0, 0.0, 0.0));

        let xform = Xform::plane_to_plane(&plane_from, &plane_to);

        // Origin of source should map to origin of target
        let p = Point::new(0.0, 0.0, 0.0);
        let tp = xform.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(tp[0], 10.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[2], 0.0));
    })
}

pub fn run_xform_look_at_rh() -> TestResult {
    MINI_TEST!("look_at_rh", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;

        // Camera at (0,0,10) looking at origin
        let eye = Point::new(0.0, 0.0, 10.0);
        let target = Point::new(0.0, 0.0, 0.0);
        let up = Vector::new(0.0, 1.0, 0.0);

        let xform = Xform::look_at_rh(&eye, &target, &up);

        // The target point should be on the negative Z axis in view space
        let tp = xform.transformed_point(&target);

        MINI_CHECK!(TOLERANCE.is_close(tp[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(tp[2], -10.0));
    })
}

pub fn run_xform_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::Xform;

        // Create a non-identity xform
        let mut xform = Xform::translation(1.0, 2.0, 3.0);
        xform.name = "test_xform".to_string();

        // json_dump(filename) / json_load(filename) - file-based serialization
        let filename = "serialization/test_xform.json";
        xform.to_json(filename).unwrap();
        let loaded = Xform::from_json(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_xform");
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[12], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[14], 3.0));
    })
}

#[cfg(feature = "protobuf")]
pub fn run_xform_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::Xform;

        // Create a non-identity xform
        let mut xform = Xform::translation(1.0, 2.0, 3.0);
        xform.name = "test_xform_proto".to_string();

        // protobuf_dump(filename) / protobuf_load(filename) - file-based serialization
        let filename = "serialization/test_xform.bin";
        xform.protobuf_dump(filename);
        let loaded = Xform::protobuf_load(filename);

        MINI_CHECK!(loaded.name == "test_xform_proto");
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[12], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[14], 3.0));
    })
}

REGISTER_MINI_TEST!("Xform", "constructor", crate::xform_test::run_xform_constructor);
REGISTER_MINI_TEST!("Xform", "translation", crate::xform_test::run_xform_translation);
REGISTER_MINI_TEST!("Xform", "scaling", crate::xform_test::run_xform_scaling);
REGISTER_MINI_TEST!("Xform", "rotation", crate::xform_test::run_xform_rotation);
REGISTER_MINI_TEST!("Xform", "inverse", crate::xform_test::run_xform_inverse);
REGISTER_MINI_TEST!("Xform", "transform_geometry", crate::xform_test::run_xform_transform_geometry);
REGISTER_MINI_TEST!("Xform", "change_basis", crate::xform_test::run_xform_change_basis);
REGISTER_MINI_TEST!("Xform", "plane_to_plane", crate::xform_test::run_xform_plane_to_plane);
REGISTER_MINI_TEST!("Xform", "look_at_rh", crate::xform_test::run_xform_look_at_rh);
REGISTER_MINI_TEST!("Xform", "json_roundtrip", crate::xform_test::run_xform_json_roundtrip);
#[cfg(feature = "protobuf")]
REGISTER_MINI_TEST!("Xform", "protobuf_roundtrip", crate::xform_test::run_xform_protobuf_roundtrip);
