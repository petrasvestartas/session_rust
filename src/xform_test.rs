use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::tolerance::PI;

pub fn run_xform_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::Xform;

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

        MINI_CHECK!(x.name == "my_xform" && !x.guid.is_empty());
        MINI_CHECK!(m00 == 1.0 && m11 == 1.0 && m22 == 1.0 && m33 == 1.0);
        MINI_CHECK!(is_id == true);
        MINI_CHECK!(xfrom.m[12] == 5.0 && xfrom.m[13] == 10.0 && xfrom.m[14] == 15.0);
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

pub fn run_xform_rotation_z() -> TestResult {
    MINI_TEST!("rotation_z", {
        use crate::Xform;
        use crate::Point;

        // Rotation around Z axis by 90 degrees
        let r = Xform::rotation_z(PI / 2.0);

        // Apply to point (1,0,0) -> (0,1,0)
        let p = Point::new(1.0, 0.0, 0.0);
        let rp = r.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(rp[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[2], 0.0));
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

pub fn run_xform_mul_operator() -> TestResult {
    MINI_TEST!("mul_operator", {
        use crate::Xform;
        use crate::Point;

        // Matrix multiplication
        let t = Xform::translation(10.0, 0.0, 0.0);
        let s = Xform::scaling(2.0, 1.0, 1.0);

        // Combined: first scale, then translate
        let combined = &t * &s;

        // Apply to point
        let p = Point::new(1.0, 0.0, 0.0);
        let result = combined.transformed_point(&p);

        // (1,0,0) * scale(2,1,1) = (2,0,0), then translate(10,0,0) = (12,0,0)
        MINI_CHECK!(TOLERANCE.is_close(result[0], 12.0));
        MINI_CHECK!(TOLERANCE.is_close(result[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result[2], 0.0));
    })
}

pub fn run_xform_transform_vector() -> TestResult {
    MINI_TEST!("transform_vector", {
        use crate::Xform;
        use crate::Vector;

        // Translation should not affect vectors (only direction)
        let t = Xform::translation(100.0, 200.0, 300.0);
        let v = Vector::new(1.0, 0.0, 0.0);
        let tv = t.transformed_vector(&v);

        // Scaling should affect vectors
        let s = Xform::scaling(2.0, 3.0, 4.0);
        let v2 = Vector::new(1.0, 1.0, 1.0);
        let sv = s.transformed_vector(&v2);

        MINI_CHECK!(TOLERANCE.is_close(tv[0], 1.0) && TOLERANCE.is_close(tv[1], 0.0) && TOLERANCE.is_close(tv[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(sv[0], 2.0) && TOLERANCE.is_close(sv[1], 3.0) && TOLERANCE.is_close(sv[2], 4.0));
    })
}

pub fn run_xform_rotation_x() -> TestResult {
    MINI_TEST!("rotation_x", {
        use crate::Xform;
        use crate::Point;

        // Rotation around X axis by 90 degrees
        let r = Xform::rotation_x(PI / 2.0);

        // Apply to point (0,1,0) -> (0,0,1)
        let p = Point::new(0.0, 1.0, 0.0);
        let rp = r.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(rp[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[2], 1.0));
    })
}

pub fn run_xform_rotation_y() -> TestResult {
    MINI_TEST!("rotation_y", {
        use crate::Xform;
        use crate::Point;

        // Rotation around Y axis by 90 degrees
        let r = Xform::rotation_y(PI / 2.0);

        // Apply to point (0,0,1) -> (1,0,0)
        let p = Point::new(0.0, 0.0, 1.0);
        let rp = r.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(rp[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[2], 0.0));
    })
}

pub fn run_xform_rotation() -> TestResult {
    MINI_TEST!("rotation", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;

        // Rotation around arbitrary axis (1,1,1) by 120 degrees
        // This cycles x->y->z->x
        let axis = Vector::new(1.0, 1.0, 1.0);
        let r = Xform::rotation(&axis, 2.0 * PI / 3.0);

        // Apply to point (1,0,0) -> (0,1,0)
        let p = Point::new(1.0, 0.0, 0.0);
        let rp = r.transformed_point(&p);

        MINI_CHECK!(TOLERANCE.is_close(rp[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(rp[2], 0.0));
    })
}

pub fn run_xform_change_basis() -> TestResult {
    MINI_TEST!("change_basis", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;

        // Create a coordinate system at origin with rotated axes
        let origin = Point::new(10.0, 20.0, 30.0);
        let x_axis = Vector::new(1.0, 0.0, 0.0);
        let y_axis = Vector::new(0.0, 1.0, 0.0);
        let z_axis = Vector::new(0.0, 0.0, 1.0);

        // Change basis transform
        let xform = Xform::change_basis(&origin, &x_axis, &y_axis, &z_axis);

        // Point at local origin should map to world origin
        let p = Point::new(10.0, 20.0, 30.0);
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

        // Source plane at origin, XY plane
        let origin_0 = Point::new(0.0, 0.0, 0.0);
        let x_axis_0 = Vector::new(1.0, 0.0, 0.0);
        let y_axis_0 = Vector::new(0.0, 1.0, 0.0);
        let z_axis_0 = Vector::new(0.0, 0.0, 1.0);

        // Target plane translated and rotated
        let origin_1 = Point::new(10.0, 0.0, 0.0);
        let x_axis_1 = Vector::new(0.0, 1.0, 0.0);
        let y_axis_1 = Vector::new(-1.0, 0.0, 0.0);
        let z_axis_1 = Vector::new(0.0, 0.0, 1.0);

        let xform = Xform::plane_to_plane(&origin_0, &x_axis_0, &y_axis_0, &z_axis_0, &origin_1, &x_axis_1, &y_axis_1, &z_axis_1);

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
        let filename = "test_xform.json";
        xform.to_json(filename).unwrap();
        let loaded = Xform::from_json(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_xform");
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[12], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[14], 3.0));
    })
}

REGISTER_MINI_TEST!("Xform", "constructor", crate::xform_test::run_xform_constructor);
REGISTER_MINI_TEST!("Xform", "translation", crate::xform_test::run_xform_translation);
REGISTER_MINI_TEST!("Xform", "scaling", crate::xform_test::run_xform_scaling);
REGISTER_MINI_TEST!("Xform", "rotation_z", crate::xform_test::run_xform_rotation_z);
REGISTER_MINI_TEST!("Xform", "inverse", crate::xform_test::run_xform_inverse);
REGISTER_MINI_TEST!("Xform", "mul_operator", crate::xform_test::run_xform_mul_operator);
REGISTER_MINI_TEST!("Xform", "transform_vector", crate::xform_test::run_xform_transform_vector);
REGISTER_MINI_TEST!("Xform", "rotation_x", crate::xform_test::run_xform_rotation_x);
REGISTER_MINI_TEST!("Xform", "rotation_y", crate::xform_test::run_xform_rotation_y);
REGISTER_MINI_TEST!("Xform", "rotation", crate::xform_test::run_xform_rotation);
REGISTER_MINI_TEST!("Xform", "change_basis", crate::xform_test::run_xform_change_basis);
REGISTER_MINI_TEST!("Xform", "plane_to_plane", crate::xform_test::run_xform_plane_to_plane);
REGISTER_MINI_TEST!("Xform", "look_at_rh", crate::xform_test::run_xform_look_at_rh);
REGISTER_MINI_TEST!("Xform", "json_roundtrip", crate::xform_test::run_xform_json_roundtrip);
