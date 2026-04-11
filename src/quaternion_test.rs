use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::{TOLERANCE, PI};

pub fn run_quaternion_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Quaternion;
        use crate::Vector;

        // Default constructor (identity)
        let q0 = Quaternion::identity();

        // Constructor with arguments
        let mut q = Quaternion::from_components(2.0, Vector::new(1.0, 0.0, 0.0));

        // Setters
        q[0] = 5.0;
        q[1] = 0.0;
        q[2] = 1.0;
        q[3] = 0.0;

        // Getters
        let s_val = q[0];
        let x = q[1];
        let y = q[2];
        let z = q[3];

        // Minimal and Full String Representation
        let qstr = q.str();
        let qrepr = q.repr();

        // Copy (duplicates everything except guid)
        let qcopy = q.duplicate();
        let qother = Quaternion::from_components(2.0, Vector::new(1.0, 0.0, 0.0));

        // Copy operators
        let qrot = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let qmul = qrot.clone() * qrot.clone();
        let qscaled = Quaternion::identity() * 2.0;
        let a = Quaternion::from_components(1.0, Vector::new(0.0, 0.0, 0.0));
        let b = Quaternion::from_components(0.0, Vector::new(0.0, 0.0, 1.0));
        let qsum = a + b;
        let qdiff = qrot.clone() - qrot.clone();
        let qneg = -Quaternion::identity();

        MINI_CHECK!(q0.name == "my_quaternion");
        MINI_CHECK!(!q0.guid().is_empty());
        MINI_CHECK!(TOLERANCE.is_close(q0.scalar, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(q0.vector[0], 0.0) && TOLERANCE.is_close(q0.vector[1], 0.0) && TOLERANCE.is_close(q0.vector[2], 0.0));
        MINI_CHECK!(q[0] == 5.0 && q[1] == 0.0 && q[2] == 1.0 && q[3] == 0.0);
        MINI_CHECK!(s_val == 5.0 && x == 0.0 && y == 1.0 && z == 0.0);
        MINI_CHECK!(qstr == "5.000000, 0.000000, 1.000000, 0.000000");
        MINI_CHECK!(qrepr == "Quaternion(my_quaternion, 5.000000, 0.000000, 1.000000, 0.000000)");
        MINI_CHECK!(qcopy == q && qcopy.guid() != q.guid());
        MINI_CHECK!(qother != q);
        MINI_CHECK!(TOLERANCE.is_close(qmul.scalar, 0.0) && TOLERANCE.is_close(qmul.vector[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(qscaled.scalar, 2.0));
        MINI_CHECK!(TOLERANCE.is_close(qsum.scalar, 1.0) && TOLERANCE.is_close(qsum.vector[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(qdiff.scalar, 0.0) && TOLERANCE.is_close(qdiff.vector[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(qneg.scalar, -1.0));
    })
}

pub fn run_quaternion_identity() -> TestResult {
    MINI_TEST!("Identity", {
        use crate::Quaternion;
        let q = Quaternion::identity();

        MINI_CHECK!(TOLERANCE.is_close(q.scalar, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(q.vector[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(q.vector[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(q.vector[2], 0.0));
    })
}

pub fn run_quaternion_from_components() -> TestResult {
    MINI_TEST!("From Components", {
        use crate::Quaternion;
        use crate::Vector;

        // q = s + xi + yj + zk: first arg is scalar, second arg is (i,j,k) coefficients (NOT a rotation axis).
        let q = Quaternion::from_components(2.0, Vector::new(1.0, 2.0, 3.0));

        MINI_CHECK!(TOLERANCE.is_close(q.scalar, 2.0));
        MINI_CHECK!(TOLERANCE.is_close(q.vector[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(q.vector[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(q.vector[2], 3.0));

        // Geometric meaning of (s, v): a rotation by `angle` around `axis`.
        // to_axis_angle() extracts these — for q=(2,(1,2,3)):
        //   axis  = (1,2,3)/sqrt(14)
        //   angle = 2*acos(2/sqrt(18)) ≈ 2.1617 rad ≈ 123.85°
        let (axis, angle) = q.to_axis_angle();
        let sqrt14 = 14.0_f64.sqrt();
        MINI_CHECK!(TOLERANCE.is_close(axis[0], 1.0 / sqrt14));
        MINI_CHECK!(TOLERANCE.is_close(axis[1], 2.0 / sqrt14));
        MINI_CHECK!(TOLERANCE.is_close(axis[2], 3.0 / sqrt14));
        MINI_CHECK!(TOLERANCE.is_close(angle, 2.0 * (2.0_f64 / 18.0_f64.sqrt()).acos()));

        // Round-trip: from_axis_angle(to_axis_angle(q)) == q.normalized()
        let q_round = Quaternion::from_axis_angle(axis, angle);
        let qn = q.normalized();
        MINI_CHECK!(TOLERANCE.is_close(q_round.scalar, qn.scalar));
        MINI_CHECK!(TOLERANCE.is_close(q_round.vector[0], qn.vector[0]));
        MINI_CHECK!(TOLERANCE.is_close(q_round.vector[1], qn.vector[1]));
        MINI_CHECK!(TOLERANCE.is_close(q_round.vector[2], qn.vector[2]));
    })
}

pub fn run_quaternion_from_axis_angle() -> TestResult {
    MINI_TEST!("From Axis Angle", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);

        MINI_CHECK!(TOLERANCE.is_close(q.scalar, (PI / 4.0).cos()));
        MINI_CHECK!(TOLERANCE.is_close(q.vector[2], (PI / 4.0).sin()));
    })
}

pub fn run_quaternion_from_arc() -> TestResult {
    MINI_TEST!("From Arc", {
        use crate::Quaternion;
        use crate::Vector;
        let src = Vector::new(1.0, 0.0, 0.0);
        let dst = Vector::new(0.0, 1.0, 0.0);
        let q = Quaternion::from_arc(src.clone(), dst.clone());
        let rotated = q.rotate_vector(src);

        MINI_CHECK!(TOLERANCE.is_close(rotated[0], dst[0]));
        MINI_CHECK!(TOLERANCE.is_close(rotated[1], dst[1]));
        MINI_CHECK!(TOLERANCE.is_close(rotated[2], dst[2]));
        let src2 = Vector::new(1.0, 0.0, 0.0);
        let dst2 = Vector::new(-1.0, 0.0, 0.0);
        let q2 = Quaternion::from_arc(src2.clone(), dst2);
        let rot = q2.rotate_vector(src2);
        MINI_CHECK!(TOLERANCE.is_close(rot[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(rot[1], 0.0));
    })
}

pub fn run_quaternion_from_euler() -> TestResult {
    MINI_TEST!("From Euler", {
        use crate::Quaternion;
        use crate::Vector;
        let q_euler = Quaternion::from_euler(0.0, 0.0, PI / 2.0);
        let q_axis = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);

        MINI_CHECK!(TOLERANCE.is_close(q_euler.scalar, q_axis.scalar));
        MINI_CHECK!(TOLERANCE.is_close(q_euler.vector[2], q_axis.vector[2]));
    })
}

pub fn run_quaternion_from_rotation() -> TestResult {
    MINI_TEST!("From Rotation", {
        use crate::Quaternion;
        use crate::Vector;
        use crate::Plane;
        use crate::Point;
        let plane_a = Plane::new(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let plane_b = Plane::new(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0), Vector::new(-1.0, 0.0, 0.0));
        let q = Quaternion::from_rotation(&plane_a, &plane_b);
        let rotated_x = q.rotate_vector(plane_a.x_axis());

        MINI_CHECK!(TOLERANCE.is_close(rotated_x[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rotated_x[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(rotated_x[2], 0.0));
    })
}

pub fn run_quaternion_rotate_vector() -> TestResult {
    MINI_TEST!("Rotate Vector", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let rotated = q.rotate_vector(Vector::new(1.0, 0.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(rotated[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(rotated[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(rotated[2], 0.0));
    })
}

pub fn run_quaternion_get_rotation() -> TestResult {
    MINI_TEST!("Get Rotation", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let p = q.get_rotation();

        MINI_CHECK!(TOLERANCE.is_close(p.x_axis()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(p.x_axis()[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(p.y_axis()[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(p.y_axis()[1], 0.0));
    })
}

pub fn run_quaternion_magnitude() -> TestResult {
    MINI_TEST!("Magnitude", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);

        MINI_CHECK!(TOLERANCE.is_close(q.magnitude(), 1.0));
    })
}

pub fn run_quaternion_magnitude_squared() -> TestResult {
    MINI_TEST!("Magnitude Squared", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);

        MINI_CHECK!(TOLERANCE.is_close(q.magnitude_squared(), q.magnitude() * q.magnitude()));
    })
}

pub fn run_quaternion_normalized() -> TestResult {
    MINI_TEST!("Normalized", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_components(2.0, Vector::new(0.0, 0.0, 2.0));
        let n = q.normalized();

        MINI_CHECK!(TOLERANCE.is_close(n.magnitude(), 1.0));
    })
}

pub fn run_quaternion_conjugate() -> TestResult {
    MINI_TEST!("Conjugate", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);
        let r = q.conjugate();

        MINI_CHECK!(TOLERANCE.is_close(r.scalar, q.scalar));
        MINI_CHECK!(TOLERANCE.is_close(r.vector[0], -q.vector[0]));
        MINI_CHECK!(TOLERANCE.is_close(r.vector[2], -q.vector[2]));
    })
}

pub fn run_quaternion_invert() -> TestResult {
    MINI_TEST!("Invert", {
        use crate::Quaternion;
        use crate::Vector;
        let q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 3.0);
        let result = q.clone() * q.invert();

        MINI_CHECK!(TOLERANCE.is_close(result.scalar, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(result.vector[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result.vector[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(result.vector[2], 0.0));
    })
}

pub fn run_quaternion_dot() -> TestResult {
    MINI_TEST!("Dot", {
        use crate::Quaternion;
        let q = Quaternion::identity();

        MINI_CHECK!(TOLERANCE.is_close(q.dot(&q), 1.0));
    })
}

pub fn run_quaternion_slerp() -> TestResult {
    MINI_TEST!("Slerp", {
        use crate::Quaternion;
        use crate::Vector;
        let q1 = Quaternion::identity();
        let q2 = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let mid = q1.slerp(&q2, 0.5);
        let expected = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 4.0);

        MINI_CHECK!(TOLERANCE.is_close(mid.scalar, expected.scalar));
        MINI_CHECK!(TOLERANCE.is_close(mid.vector[2], expected.vector[2]));
        let q3 = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), 0.001);
        let mid2 = q1.slerp(&q3, 0.5);
        let half = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), 0.0005);
        MINI_CHECK!(TOLERANCE.is_close(mid2.scalar, half.scalar));
    })
}

pub fn run_quaternion_nlerp() -> TestResult {
    MINI_TEST!("Nlerp", {
        use crate::Quaternion;
        use crate::Vector;
        let q1 = Quaternion::identity();
        let q2 = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        let r0 = q1.nlerp(&q2, 0.0);
        let r1 = q1.nlerp(&q2, 1.0);

        MINI_CHECK!(TOLERANCE.is_close(r0.scalar, q1.scalar));
        MINI_CHECK!(TOLERANCE.is_close(r1.scalar, q2.scalar));
    })
}

pub fn run_quaternion_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Quaternion;
        use crate::Vector;

        let mut q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        q.name = "test_quaternion".to_string();

        //   jsondump()      | String       | to JSON string (internal use)
        //   jsonload(s)     | String       | from JSON string (internal use)
        //   json_dumps()    | String       | to JSON string
        //   json_loads(s)   | String       | from JSON string
        //   json_dump(path) | file         | write to file
        //   json_load(path) | file         | read from file

        let filename = "serialization/test_quaternion.json";
        q.json_dump(filename).unwrap();
        let loaded = Quaternion::json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_quaternion");
        MINI_CHECK!(TOLERANCE.is_close(loaded.scalar, q.scalar));
        MINI_CHECK!(TOLERANCE.is_close(loaded.vector[2], q.vector[2]));
    })
}

pub fn run_quaternion_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Quaternion;
        use crate::Vector;

        let mut q = Quaternion::from_axis_angle(Vector::new(0.0, 0.0, 1.0), PI / 2.0);
        q.name = "test_quaternion".to_string();

        let filename = "serialization/test_quaternion.bin";
        q.pb_dump(filename);
        let loaded = Quaternion::pb_load(filename);

        MINI_CHECK!(loaded.name == "test_quaternion");
        MINI_CHECK!(TOLERANCE.is_close(loaded.scalar, q.scalar));
        MINI_CHECK!(TOLERANCE.is_close(loaded.vector[2], q.vector[2]));
    })
}

REGISTER_MINI_TEST!("Quaternion", "Constructor", crate::quaternion_test::run_quaternion_constructor);
REGISTER_MINI_TEST!("Quaternion", "Identity", crate::quaternion_test::run_quaternion_identity);
REGISTER_MINI_TEST!("Quaternion", "From Components", crate::quaternion_test::run_quaternion_from_components);
REGISTER_MINI_TEST!("Quaternion", "From Axis Angle", crate::quaternion_test::run_quaternion_from_axis_angle);
REGISTER_MINI_TEST!("Quaternion", "From Arc", crate::quaternion_test::run_quaternion_from_arc);
REGISTER_MINI_TEST!("Quaternion", "From Euler", crate::quaternion_test::run_quaternion_from_euler);
REGISTER_MINI_TEST!("Quaternion", "From Rotation", crate::quaternion_test::run_quaternion_from_rotation);
REGISTER_MINI_TEST!("Quaternion", "Rotate Vector", crate::quaternion_test::run_quaternion_rotate_vector);
REGISTER_MINI_TEST!("Quaternion", "Get Rotation", crate::quaternion_test::run_quaternion_get_rotation);
REGISTER_MINI_TEST!("Quaternion", "Magnitude", crate::quaternion_test::run_quaternion_magnitude);
REGISTER_MINI_TEST!("Quaternion", "Magnitude Squared", crate::quaternion_test::run_quaternion_magnitude_squared);
REGISTER_MINI_TEST!("Quaternion", "Normalized", crate::quaternion_test::run_quaternion_normalized);
REGISTER_MINI_TEST!("Quaternion", "Conjugate", crate::quaternion_test::run_quaternion_conjugate);
REGISTER_MINI_TEST!("Quaternion", "Invert", crate::quaternion_test::run_quaternion_invert);
REGISTER_MINI_TEST!("Quaternion", "Dot", crate::quaternion_test::run_quaternion_dot);
REGISTER_MINI_TEST!("Quaternion", "Slerp", crate::quaternion_test::run_quaternion_slerp);
REGISTER_MINI_TEST!("Quaternion", "Nlerp", crate::quaternion_test::run_quaternion_nlerp);
REGISTER_MINI_TEST!("Quaternion", "Json Roundtrip", crate::quaternion_test::run_quaternion_json_roundtrip);
REGISTER_MINI_TEST!("Quaternion", "Protobuf Roundtrip", crate::quaternion_test::run_quaternion_protobuf_roundtrip);
