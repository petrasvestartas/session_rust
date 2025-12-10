use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_vector_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::Vector;
        use crate::Point;

        // Constructor
        let mut v = Vector::new(1.0, 2.0, 3.0);
        let p0 = Point::new(1.0, 2.0, 3.0);
        let p1 = Point::new(2.0, 4.0, 6.0);    
        let v_2p = Vector::from_points(&p0, &p1);

        // Setters
        v[0] = 10.0;
        v[1] = 20.0;
        v[2] = 30.0;

        // Getters
        let x = v[0];
        let y = v[1];
        let z = v[2];

        // Minimal and full string representation
        let vsrt = v.str();
        let vrepr = v.repr();

        // Copy (duplicate everything but guid)
        let vcopy = v.duplicate();
        let vother = Vector::new(1.0, 2.0, 3.0);

        // No-copy operators d
        let mut vmult = v.duplicate();
        vmult *= 2.0;
        let mut vdiv = v.duplicate();
        vdiv /= 2.0;
        let mut vadd = v.duplicate();
        vadd += Vector::new(1.0, 1.0, 1.0);
        let mut vsub = v.duplicate();
        vsub -= Vector::new(1.0, 1.0, 1.0);

        // Copy operators
        let result_mul = &v * 2.0;
        let result_div = &v / 2.0;
        let result_add = &v + Vector::new(1.0, 1.0, 1.0);
        let result_dif = &v - Vector::new(1.0, 1.0, 1.0);

        // Static axis constructors
        let vx = Vector::x_axis();
        let vy = Vector::y_axis();
        let vz = Vector::z_axis();
        let vzero = Vector::zero();

        MINI_CHECK!(
            v.name == "my_vector" &&
            v[0] == 10.0 &&
            v[1] == 20.0 &&
            v[2] == 30.0 &&
            !v.guid.is_empty()
        );
        MINI_CHECK!(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK!(v_2p[0] == 1.0 && v_2p[1] == 2.0 && v_2p[2] == 3.0);
        MINI_CHECK!(vsrt == "10.000000, 20.000000, 30.000000");
        MINI_CHECK!(vrepr == "Vector(my_vector, 10.000000, 20.000000, 30.000000, 37.416574)");
        MINI_CHECK!(vcopy == v && vcopy.guid != v.guid);
        MINI_CHECK!(vother != v);
        MINI_CHECK!(vmult[0] == 20.0 && vmult[1] == 40.0 && vmult[2] == 60.0);
        MINI_CHECK!(vdiv[0] == 5.0 && vdiv[1] == 10.0 && vdiv[2] == 15.0);
        MINI_CHECK!(vadd[0] == 11.0 && vadd[1] == 21.0 && vadd[2] == 31.0);
        MINI_CHECK!(vsub[0] == 9.0 && vsub[1] == 19.0 && vsub[2] == 29.0);
        MINI_CHECK!(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
        MINI_CHECK!(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
        MINI_CHECK!(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
        MINI_CHECK!(result_dif[0] == 9.0 && result_dif[1] == 19.0 && result_dif[2] == 29.0);
        MINI_CHECK!(vx[0] == 1.0 && vx[1] == 0.0 && vx[2] == 0.0);
        MINI_CHECK!(vy[0] == 0.0 && vy[1] == 1.0 && vy[2] == 0.0);
        MINI_CHECK!(vz[0] == 0.0 && vz[1] == 0.0 && vz[2] == 1.0);
        MINI_CHECK!(vzero[0] == 0.0 && vzero[1] == 0.0 && vzero[2] == 0.0);
    })
}

pub fn run_vector_magnitude() -> TestResult {
    MINI_TEST!("magnitude", {
        use crate::Vector;

        let mut v = Vector::new(3.0, 4.0, 0.0);
        let len = v.magnitude();
        let len_squared = v.magnitude_squared();

        MINI_CHECK!(len == 5.0);
        MINI_CHECK!(len_squared == 25.0);
    })
}

pub fn run_vector_normalize() -> TestResult {
    MINI_TEST!("normalize", {
        use crate::Vector;

        let mut v0 = Vector::new(3.0, 4.0, 0.0);
        v0.normalize();

        let v1 = Vector::new(3.0, 4.0, 0.0);
        let mut v2 = v1.normalized();

        MINI_CHECK!((v0.magnitude() - 1.0).abs() < 1e-10);
        MINI_CHECK!((v2.magnitude() - 1.0).abs() < 1e-10);
    })
}

pub fn run_vector_dot_product() -> TestResult {
    MINI_TEST!("dot_product", {
        use crate::Vector;

        let v1 = Vector::new(1.0, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0);
        let v3 = Vector::new(1.0, 0.0, 0.0);

        // Perpendicular vectors
        MINI_CHECK!(v1.dot(&v2) == 0.0);

        // Parallel vectors
        MINI_CHECK!(v1.dot(&v3) == 1.0);
    })
}

pub fn run_vector_cross_product() -> TestResult {
    MINI_TEST!("cross_product", {
        use crate::Vector;

        let v1 = Vector::new(1.0, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0);

        let cross = v1.cross(&v2);
        MINI_CHECK!(cross[0] == 0.0 && cross[1] == 0.0 && cross[2] == 1.0);
    })
}

pub fn run_vector_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::Vector;
        use crate::encoders::{json_dump, json_load};

        let mut v = Vector::new(42.1, 84.2, 126.3);
        v.name = "test_vector".to_string();

        let filename = "test_vector.json";
        json_dump(&v, filename, true).unwrap();
        let loaded: Vector = json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_vector");
        MINI_CHECK!((loaded[0] - 42.1).abs() < 1e-10);
        MINI_CHECK!((loaded[1] - 84.2).abs() < 1e-10);
        MINI_CHECK!((loaded[2] - 126.3).abs() < 1e-10);
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Vector", "constructor", crate::vector_test::run_vector_constructor);
REGISTER_MINI_TEST!("Vector", "magnitude", crate::vector_test::run_vector_magnitude);
REGISTER_MINI_TEST!("Vector", "normalize", crate::vector_test::run_vector_normalize);
REGISTER_MINI_TEST!("Vector", "dot_product", crate::vector_test::run_vector_dot_product);
REGISTER_MINI_TEST!("Vector", "cross_product", crate::vector_test::run_vector_cross_product);
REGISTER_MINI_TEST!("Vector", "json_roundtrip", crate::vector_test::run_vector_json_roundtrip);

#[cfg(test)]
mod vector_tests {
    use crate::encoders::{json_dump, json_load};
    use crate::{Point, Vector};

    #[test]
    fn test_vector_constructor() {
        let v = Vector::new(1.0, 2.0, 3.0);
        assert_eq!((v[0], v[1], v[2]), (1.0, 2.0, 3.0));
        assert!(!v.guid.is_empty());
    }

    #[test]
    fn test_vector_equality() {
        let v1 = Vector::new(1.0, 2.0, 3.0);
        let mut v2 = Vector::new(1.0, 2.0, 3.0);
        v2.guid = v1.guid.clone();
        assert_eq!(v1, v2);
        let v3 = Vector::new(1.1, 2.0, 3.0);
        assert_ne!(v1, v3);
    }

    #[test]
    fn test_vector_to_json_data() {
        let v = Vector::new(15.5, 25.7, 35.9);
        let data = v.jsondump().unwrap();
        assert!(data.contains("Vector") && data.contains("15.5"));
    }

    #[test]
    fn test_vector_from_json_data() {
        let orig = Vector::new(42.1, 84.2, 126.3);
        let rest = Vector::jsonload(&orig.jsondump().unwrap()).unwrap();
        assert_eq!((rest[0], rest[1], rest[2]), (42.1, 84.2, 126.3));
    }

    #[test]
    fn test_vector_to_json_from_json() {
        let orig = Vector::new(123.45, 678.90, 999.11);
        let filename = "test_vector.json";
        json_dump(&orig, filename, true).unwrap();
        let load = json_load::<Vector>(filename).unwrap();
        assert_eq!(
            (load[0], load[1], load[2]),
            (orig[0], orig[1], orig[2])
        );
    }

    #[test]
    fn test_vector_default_constructor() {
        let v = Vector::default();
        assert_eq!((v[0], v[1], v[2]), (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_vector_constructor_values() {
        let v = Vector::new(0.57, -158.63, 180.890);
        assert_eq!((v[0], v[1], v[2]), (0.57, -158.63, 180.890));
    }

    #[test]
    fn test_vector_static_methods() {
        assert_eq!(
            (
                Vector::x_axis()[0],
                Vector::x_axis()[1],
                Vector::x_axis()[2]
            ),
            (1.0, 0.0, 0.0)
        );
        assert_eq!(
            (
                Vector::y_axis()[0],
                Vector::y_axis()[1],
                Vector::y_axis()[2]
            ),
            (0.0, 1.0, 0.0)
        );
        assert_eq!(
            (
                Vector::z_axis()[0],
                Vector::z_axis()[1],
                Vector::z_axis()[2]
            ),
            (0.0, 0.0, 1.0)
        );
    }

    #[test]
    fn test_vector_from_points() {
        let v =
            Vector::from_points(&Point::new(8.7, 5.7, -1.87), &Point::new(1.0, 1.57, 2.0));
        assert!((v[0] + 7.7).abs() < 1e-5);
        assert!((v[1] + 4.13).abs() < 1e-5);
        assert!((v[2] - 3.87).abs() < 1e-5);
    }

    #[test]
    fn test_vector_operators() {
        let v1 = Vector::new(1.0, 2.0, 3.0);
        let v2 = Vector::new(4.0, 5.0, 6.0);
        let v3 = &v1 + &v2;
        assert_eq!((v3[0], v3[1], v3[2]), (5.0, 7.0, 9.0));
        let v3 = &v1 - &v2;
        assert_eq!((v3[0], v3[1], v3[2]), (-3.0, -3.0, -3.0));
        let v3 = &v1 * 2.0;
        assert_eq!((v3[0], v3[1], v3[2]), (2.0, 4.0, 6.0));
        let v3 = &v1 / 2.0;
        assert_eq!((v3[0], v3[1], v3[2]), (0.5, 1.0, 1.5));

        let mut v3 = Vector::new(1.0, 2.0, 3.0);
        v3 += &v2;
        assert_eq!((v3[0], v3[1], v3[2]), (5.0, 7.0, 9.0));
        v3 -= &v2;
        assert_eq!((v3[0], v3[1], v3[2]), (1.0, 2.0, 3.0));
        v3 *= 2.0;
        assert_eq!((v3[0], v3[1], v3[2]), (2.0, 4.0, 6.0));
        v3 /= 2.0;
        assert_eq!((v3[0], v3[1], v3[2]), (1.0, 2.0, 3.0));
    }

    #[test]
    fn test_vector_reverse() {
        let mut v = Vector::new(1.0, 2.0, 3.0);
        v.reverse();
        assert_eq!((v[0], v[1], v[2]), (-1.0, -2.0, -3.0));
    }

    #[test]
    fn test_vector_length() {
        let v = Vector::new(5.5697, -9.84, 1.587);
        let length = v.magnitude();
        assert_eq!(length, 11.4177811806848);
    }

    #[test]
    fn test_vector_unitize() {
        let v = Vector::new(5.5697, -9.84, 1.587);
        assert_eq!(v.normalized().magnitude(), 1.0);
        let mut v = Vector::new(5.5697, -9.84, 1.587);
        v.normalize();
        assert_eq!(v.magnitude(), 1.0);
    }

    #[test]
    fn test_vector_projection() {
        let v = Vector::new(1.0, 1.0, 1.0);
        let x = Vector::x_axis();
        let y = Vector::y_axis();
        let z = Vector::z_axis();
        let (proj_x, _lenx, _perp_x, _perp_lenx) = v.projection(&x);
        let (proj_y, _leny, _perp_y, _perp_leny) = v.projection(&y);
        let (proj_z, _lenz, _perp_z, _perp_lenz) = v.projection(&z);
        assert_eq!((proj_x[0], proj_x[1], proj_x[2]), (1.0, 0.0, 0.0));
        assert_eq!((proj_y[0], proj_y[1], proj_y[2]), (0.0, 1.0, 0.0));
        assert_eq!((proj_z[0], proj_z[1], proj_z[2]), (0.0, 0.0, 1.0));
    }

    #[test]
    fn test_vector_is_parallel_to() {
        let v1 = Vector::new(0.0, 0.0, 1.0);
        let v2 = Vector::new(0.0, 0.0, 2.0);
        let v3 = Vector::new(0.0, 0.0, -1.0);
        let v4 = Vector::new(0.0, 1.0, -1.0);
        assert_eq!(v1.is_parallel_to(&v2), 1);
        assert_eq!(v1.is_parallel_to(&v3), -1);
        assert_eq!(v1.is_parallel_to(&v4), 0);
    }

    #[test]
    fn test_vector_dot() {
        let v1 = Vector::new(1.0, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0);
        let v3 = Vector::new(-1.0, 0.0, 0.0);
        assert_eq!(v1.dot(&v2), 0.0);
        assert_eq!(v1.dot(&v3), -1.0);
        assert_eq!(v1.dot(&v1), 1.0);

        let dot = v1.dot(&v2);
        let mag = v1.magnitude() * v2.magnitude();
        if mag > 0.0 {
            let angle_deg = (dot / mag).acos() * crate::tolerance::TO_DEGREES;
            assert_eq!(angle_deg, 90.0);
        }
    }

    #[test]
    fn test_vector_cross() {
        let v1 = Vector::new(1.0, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0);
        let v3 = v1.cross(&v2);
        assert_eq!((v3[0], v3[1], v3[2]), (0.0, 0.0, 1.0));
    }

    #[test]
    fn test_vector_angle() {
        let v1 = Vector::new(1.0, 1.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0);
        assert!((v1.angle(&v2, false) - 45.0).abs() < crate::tolerance::Tolerance::ZERO_TOLERANCE);
        assert!(
            (Vector::new(-1.0, 1.0, 0.0).angle(&v2, true) + 45.0).abs()
                < crate::tolerance::Tolerance::ZERO_TOLERANCE
        );
    }

    #[test]
    fn test_vector_get_leveled_vector() {
        let lev = Vector::new(1.0, 1.0, 1.0).get_leveled_vector(1.0);
        assert!((lev.magnitude() - 4.168_432_5).abs() < 1e-4);
    }

    #[test]
    fn test_vector_cosine_law() {
        let c = Vector::cosine_law(100.0, 150.0, 115.0, true);
        assert_eq!((c * 100.0).round() / 100.0, 212.55);
    }

    #[test]
    fn test_vector_sine_law_angle() {
        let angle_b = Vector::sine_law_angle(212.55, 115.0, 150.0, true);
        assert_eq!((angle_b * 100.0).round() / 100.0, 39.76);
    }

    #[test]
    fn test_vector_sine_law_length() {
        let len_b = Vector::sine_law_length(212.55, 115.0, 39.761714, true);
        assert_eq!((len_b * 100.0).round() / 100.0, 150.0);
    }

    #[test]
    fn test_vector_angle_between_vector_xy_components() {
        let v1 = Vector::new(3.0_f64.sqrt(), 1.0, 0.0);
        let v2 = Vector::new(1.0, 3.0_f64.sqrt(), 0.0);
        assert_eq!(
            (Vector::angle_between_vector_xy_components(&v1) * 100.0).round() / 100.0,
            30.0
        );
        assert_eq!(
            (Vector::angle_between_vector_xy_components(&v2) * 100.0).round() / 100.0,
            60.0
        );
    }

    #[test]
    fn test_vector_sum_of_vectors() {
        let vecs = vec![
            Vector::new(1.0, 1.0, 1.0),
            Vector::new(2.0, 2.0, 2.0),
            Vector::new(3.0, 3.0, 3.0),
        ];
        let sum = Vector::sum_of_vectors(&vecs);
        assert_eq!((sum[0], sum[1], sum[2]), (6.0, 6.0, 6.0));
    }

    #[test]
    fn test_vector_coordinate_direction_angles() {
        let abg = Vector::new(35.4, 35.4, 86.6).coordinate_direction_3angles(true);
        assert!((abg[0] - 69.274_2).abs() < 1e-4);
        assert!((abg[1] - 69.274_2).abs() < 1e-4);
        assert!((abg[2] - 30.032058).abs() < 1e-4);

        let pt = Vector::new(1.0, 1.0, 2.0_f64.sqrt()).coordinate_direction_2angles(true);
        assert!((pt[0] - 45.0).abs() < 1e-6);
        assert!((pt[1] - 45.0).abs() < 1e-6);
    }
}
