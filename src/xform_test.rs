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

REGISTER_MINI_TEST!("Xform", "constructor", crate::xform_test::run_xform_constructor);
REGISTER_MINI_TEST!("Xform", "translation", crate::xform_test::run_xform_translation);
REGISTER_MINI_TEST!("Xform", "scaling", crate::xform_test::run_xform_scaling);
REGISTER_MINI_TEST!("Xform", "rotation_z", crate::xform_test::run_xform_rotation_z);
REGISTER_MINI_TEST!("Xform", "inverse", crate::xform_test::run_xform_inverse);
REGISTER_MINI_TEST!("Xform", "mul_operator", crate::xform_test::run_xform_mul_operator);
REGISTER_MINI_TEST!("Xform", "transform_vector", crate::xform_test::run_xform_transform_vector);
