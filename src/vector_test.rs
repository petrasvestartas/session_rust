use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_vector_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
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
            !v.guid().is_empty()
        );
        MINI_CHECK!(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK!(v_2p[0] == 1.0 && v_2p[1] == 2.0 && v_2p[2] == 3.0);
        MINI_CHECK!(vsrt == "10.000000, 20.000000, 30.000000");
        MINI_CHECK!(vrepr == "Vector(my_vector, 10.000000, 20.000000, 30.000000, 37.416574)");
        MINI_CHECK!(vcopy == v && vcopy.guid() != v.guid());
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

pub fn run_vector_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Vector;
        use crate::Xform;
        use crate::tolerance::PI;

        let mut v = Vector::new(1.0, 2.0, 3.0);
        v.xform = Xform::translation(10.0, 20.0, 30.0);
        let v_transformed = v.transformed();
        v.transform();

        MINI_CHECK!(v_transformed[0] == 1.0 && v_transformed[1] == 2.0 && v_transformed[2] == 3.0);
        MINI_CHECK!(v[0] == 1.0 && v[1] == 2.0 && v[2] == 3.0);
        MINI_CHECK!(v.xform == Xform::identity());

        let mut v2 = Vector::new(1.0, 0.0, 0.0);
        v2.xform = Xform::rotation_z(PI / 2.0, false);
        v2.transform();
        MINI_CHECK!(TOLERANCE.is_close(v2[0], 0.0) && TOLERANCE.is_close(v2[1], 1.0) && TOLERANCE.is_close(v2[2], 0.0));
    })
}

pub fn run_vector_magnitude() -> TestResult {
    MINI_TEST!("Magnitude", {
        use crate::Vector;

        let v = Vector::new(3.0, 4.0, 0.0);
        let len = v.magnitude();
        let len_squared = v.magnitude_squared();

        MINI_CHECK!(len == 5.0);
        MINI_CHECK!(len_squared == 25.0);
    })
}

pub fn run_vector_normalize() -> TestResult {
    MINI_TEST!("Normalize", {
        use crate::Vector;

        let mut v0 = Vector::new(3.0, 4.0, 0.0);
        v0.normalize_self();

        let v1 = Vector::new(3.0, 4.0, 0.0);
        let v2 = v1.normalized();

        MINI_CHECK!(TOLERANCE.is_close(v0.magnitude(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(v2.magnitude(), 1.0));
    })
}

pub fn run_vector_reverse() -> TestResult {
    MINI_TEST!("Reverse", {
        use crate::Vector;

        let mut v = Vector::new(1.0, -2.0, 3.0);
        v.reverse();

        MINI_CHECK!(v[0] == -1.0 && v[1] == 2.0 && v[2] == -3.0);
    })
}

pub fn run_vector_dot_product() -> TestResult {
    MINI_TEST!("Dot Product", {
        use crate::Vector;

        // Orthogonality and parallelism via dot product
        // Perpendicular vectors are close to 0.0
        // Parallel vectors are close to 1.0
        let v1 = Vector::new(1.0, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0);
        let v3 = Vector::new(1.0, 0.0, 0.0);
        let dot_perp = v1.dot(&v2);
        let dot_paral = v1.dot(&v3);

        // Projection of a onto b
        // Scalar projection:
        // (a . b) / ||b|| (here ||b||=1, so just a_x = 3.0)
        // Projection coefficient:
        // (a . b) / ||b||^2 = 6/4 = 1.5 (how many b2's fit in projection)
        let a = Vector::new(3.0, 4.0, 0.0);
        let b = Vector::new(1.0, 0.0, 0.0);
        let b2 = Vector::new(2.0, 0.0, 0.0);
        let proj_scalar = a.dot(&b) / (b[0].powi(2) + b[1].powi(2) + b[2].powi(2)).sqrt();
        let proj_coeff = a.dot(&b2) / (b2[0].powi(2) + b2[1].powi(2) + b2[2].powi(2));

        MINI_CHECK!(TOLERANCE.is_close(dot_perp, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(dot_paral, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(proj_scalar, 3.0));
        MINI_CHECK!(TOLERANCE.is_close(proj_coeff, 1.5));
    })
}

pub fn run_vector_cross_product() -> TestResult {
    MINI_TEST!("Cross Product", {
        use crate::Vector;

        // Get normal
        let v1 = Vector::new(1.0, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0);
        let vn = v1.cross(&v2);

        // Perpendicular, area = 3*4=12
        let a = Vector::new(3.0, 0.0, 0.0);
        let b = Vector::new(0.0, 4.0, 0.0); 
        let cross = a.cross(&b);
        let area = (cross[0].powi(2) + cross[1].powi(2) + cross[2].powi(2)).sqrt();
        
        MINI_CHECK!(vn[0] == 0.0 && vn[1] == 0.0 && vn[2] == 1.0);
        MINI_CHECK!(TOLERANCE.is_close(area, 12.0));

    })
}

pub fn run_vector_angle() -> TestResult {
    MINI_TEST!("Angle", {
        use crate::Vector;

        // angle(): Angle between two vectors (degrees)
        let v1 = Vector::new(1.0, 0.0, 0.0);  // x-axis
        let v2 = Vector::new(0.0, 1.0, 0.0);  // y-axis
        let v3 = Vector::new(1.0, 1.0, 0.0);  // 45° from x-axis

        let angle_90 = v1.angle(&v2, false);  // v1 to v2: 90°
        let angle_45 = v1.angle(&v3, false);  // v1 to v3: 45°

        // angle_between_vector_xy_components(): Angle of vector's XY projection from +X axis (atan2)
        let v_30 = Vector::new(3.0_f64.sqrt(), 1.0, 0.0);  // 30° from x-axis
        let v_60 = Vector::new(1.0, 3.0_f64.sqrt(), 0.0);  // 60° from x-axis
        let v_neg = Vector::new(-1.0, 1.0, 0.0);           // 135° from x-axis

        let xy_angle_30 = Vector::angle_between_vector_xy_components(&v_30);
        let xy_angle_60 = Vector::angle_between_vector_xy_components(&v_60);
        let xy_angle_135 = Vector::angle_between_vector_xy_components(&v_neg);

        // coordinate_direction_3angles(): Angles (α, β, γ) to x, y, z axes
        let v_dir = Vector::new(35.4, 35.4, 86.6);
        let abg = v_dir.coordinate_direction_3angles(true);

        // coordinate_direction_2angles(): Spherical angles (θ azimuth, φ elevation)
        let v_sph = Vector::new(1.0, 1.0, 2.0_f64.sqrt());
        let pt = v_sph.coordinate_direction_2angles(true);

        MINI_CHECK!(TOLERANCE.is_close(angle_90, 90.0));
        MINI_CHECK!(TOLERANCE.is_close(angle_45, 45.0));
        MINI_CHECK!(TOLERANCE.is_close(xy_angle_30, 30.0));
        MINI_CHECK!(TOLERANCE.is_close(xy_angle_60, 60.0));
        MINI_CHECK!(TOLERANCE.is_close(xy_angle_135, 135.0));
        MINI_CHECK!(TOLERANCE.is_close(abg[0], 69.274_2));
        MINI_CHECK!(TOLERANCE.is_close(abg[1], 69.274_2));
        MINI_CHECK!(TOLERANCE.is_close(abg[2], 30.032058));
        MINI_CHECK!(TOLERANCE.is_close(pt[0], 45.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 45.0));
    })
}

pub fn run_vector_projection() -> TestResult {
    MINI_TEST!("Projection", {
        use crate::Vector;

        // Project vector v=(1,1,1) onto each axis
        // Returns: (projection_vector, scalar_length, perpendicular_vector, perp_length)
        let v = Vector::new(1.0, 1.0, 1.0);
        let x = Vector::x_axis();
        let y = Vector::y_axis();
        let z = Vector::z_axis();
        let (proj_x, _lenx, _perp_x, _perp_lenx) = v.projection(&x);
        let (proj_y, _leny, _perp_y, _perp_leny) = v.projection(&y);
        let (proj_z, _lenz, _perp_z, _perp_lenz) = v.projection(&z);

        MINI_CHECK!(proj_x[0] == 1.0 && proj_x[1] == 0.0 && proj_x[2] == 0.0);
        MINI_CHECK!(proj_y[0] == 0.0 && proj_y[1] == 1.0 && proj_y[2] == 0.0);
        MINI_CHECK!(proj_z[0] == 0.0 && proj_z[1] == 0.0 && proj_z[2] == 1.0);
    })
}

pub fn run_vector_is_parallel_to() -> TestResult {
    MINI_TEST!("Is Parallel To", {
        use crate::Vector;

        // is_parallel_to returns: 1 (parallel), -1 (anti-parallel), 0 (not parallel)
        let v1 = Vector::new(2.0, 2.0, 2.0);
        let v2 = Vector::new(4.0, 4.0, 4.0);      // parallel (same direction)
        let v3 = Vector::new(-1.0, -1.0, -1.0);   // anti-parallel (opposite direction)
        let v4 = Vector::new(1.0, 0.0, 0.0);      // not parallel

        MINI_CHECK!(v1.is_parallel_to(&v2) == 1);
        MINI_CHECK!(v1.is_parallel_to(&v3) == -1);
        MINI_CHECK!(v1.is_parallel_to(&v4) == 0);
    })
}

pub fn run_vector_is_perpendicular_to() -> TestResult {
    MINI_TEST!("Is Perpendicular To", {
        use crate::Vector;

        // is_perpendicular_to: checks if two vectors are perpendicular (dot product ≈ 0)
        let v1 = Vector::new(1.0, 0.0, 0.0);
        let v2 = Vector::new(0.0, 1.0, 0.0); // perpendicular
        let v3 = Vector::new(0.0, 0.0, 1.0); // perpendicular
        let v4 = Vector::new(1.0, 1.0, 0.0); // not perpendicular

        // perpendicular_to: sets a vector to be perpendicular to another
        let z_axis = Vector::new(0.0, 0.0, 1.0);
        let mut x_axis = Vector::zero();
        // x_axis is now perpendicular to z_axis
        x_axis.perpendicular_to(&z_axis);  

        let arbitrary = Vector::new(1.0, 2.0, 3.0);
        let mut perp = Vector::zero();
        // perp is now perpendicular to arbitrary
        perp.perpendicular_to(&arbitrary);  

        MINI_CHECK!(v1.is_perpendicular_to(&v2));
        MINI_CHECK!(v1.is_perpendicular_to(&v3));
        MINI_CHECK!(!v1.is_perpendicular_to(&v4));
        MINI_CHECK!(x_axis.is_perpendicular_to(&z_axis));
        MINI_CHECK!(perp.is_perpendicular_to(&arbitrary));
    })
}

pub fn run_vector_get_leveled_vector() -> TestResult {
    MINI_TEST!("Get Leveled Vector", {
        use crate::Vector;

        // Scale vector along its direction so its Z-component equals vertical_height.
        let v = Vector::new(1.0, 1.0, 1.0);
        let vertical_height = 1.0;
        let v_leveled = v.get_leveled_vector(vertical_height);

        MINI_CHECK!(TOLERANCE.is_close(v_leveled.magnitude(), 3.0_f64.sqrt()));
    })
}

pub fn run_vector_cos_sin_laws() -> TestResult {
    MINI_TEST!("Cos Sin Laws", {
        use crate::Vector;

        // Given a 3-4-5 right triangle
        let a = 3.0; // side opposite to angle A
        let b = 4.0; // side opposite to angle B
        let c = 5.0; // hypotenuse, opposite to angle C (90°)

        // angle_from_cosine_law(adj1, adj2, opposite) -> angle opposite to 'opposite' side
        let angle_a = Vector::angle_from_cosine_law(b, c, a, true); 
        let angle_b = Vector::angle_from_cosine_law(a, c, b, true); 
        let angle_c = Vector::angle_from_cosine_law(a, b, c, true); 

        // given 2 angles + 1 side, find other side
        let side_a = Vector::side_from_sine_law(angle_a, angle_b, b, true);
        let side_b = Vector::side_from_sine_law(angle_b, angle_c, c, true);
        let side_c = Vector::side_from_sine_law(angle_c, angle_a, a, true);

        //  given 2 sides + included angle, find 3rd side
        let computed_c = Vector::cosine_law(a, b, angle_c, true); 
        let computed_a = Vector::cosine_law(b, c, angle_a, true);
        let computed_b = Vector::cosine_law(a, c, angle_b, true);

        // given 2 sides + 1 angle, find other angle
        let computed_angle_b = Vector::sine_law_angle(a, angle_a, b, true);
        let computed_angle_a = Vector::sine_law_angle(b, angle_b, a, true);

        //  given 1 side + 2 angles, find other side
        let computed_side_b = Vector::sine_law_length(a, angle_a, angle_b, true);
        let computed_side_a = Vector::sine_law_length(b, angle_b, angle_a, true);

        MINI_CHECK!(TOLERANCE.is_close(angle_a, 36.86989764584402));
        MINI_CHECK!(TOLERANCE.is_close(angle_b, 53.13010235415599));
        MINI_CHECK!(TOLERANCE.is_close(angle_c, 90.0));
        MINI_CHECK!(TOLERANCE.is_close(angle_a + angle_b + angle_c, 180.0)); // angles sum to 180°
        MINI_CHECK!(TOLERANCE.is_close(side_a, a));
        MINI_CHECK!(TOLERANCE.is_close(side_b, b));
        MINI_CHECK!(TOLERANCE.is_close(side_c, c));
        MINI_CHECK!(TOLERANCE.is_close(computed_c, c));
        MINI_CHECK!(TOLERANCE.is_close(computed_a, a));
        MINI_CHECK!(TOLERANCE.is_close(computed_b, b));
        MINI_CHECK!(TOLERANCE.is_close(computed_angle_b, angle_b));
        MINI_CHECK!(TOLERANCE.is_close(computed_angle_a, angle_a));
        MINI_CHECK!(TOLERANCE.is_close(computed_side_b, b));
        MINI_CHECK!(TOLERANCE.is_close(computed_side_a, a));
    })
}

pub fn run_vector_sum_of_vectors() -> TestResult {
    MINI_TEST!("Sum Of Vectors", {
        use crate::Vector;

        // Sum of multiple vectors
        let vecs = vec![
            Vector::new(1.0, 1.0, 1.0),
            Vector::new(2.0, 2.0, 2.0),
            Vector::new(3.0, 3.0, 3.0),
        ];
        let sum = Vector::sum_of_vectors(&vecs);

        MINI_CHECK!(sum[0] == 6.0);
        MINI_CHECK!(sum[1] == 6.0);
        MINI_CHECK!(sum[2] == 6.0);

        // Empty list returns zero vector
        let empty: Vec<Vector> = vec![];
        let zero = Vector::sum_of_vectors(&empty);
        MINI_CHECK!(zero[0] == 0.0);
        MINI_CHECK!(zero[1] == 0.0);
        MINI_CHECK!(zero[2] == 0.0);
    })
}

pub fn run_vector_average() -> TestResult {
    MINI_TEST!("Average", {
        use crate::Vector;

        // Average of multiple vectors
        let vecs = vec![
            Vector::new(1.0, 2.0, 3.0),
            Vector::new(3.0, 4.0, 5.0),
            Vector::new(5.0, 6.0, 7.0),
        ];
        let avg = Vector::average(&vecs);

        MINI_CHECK!(avg[0] == 3.0);
        MINI_CHECK!(avg[1] == 4.0);
        MINI_CHECK!(avg[2] == 5.0);
    })
}

pub fn run_vector_is_zero() -> TestResult {
    MINI_TEST!("Is Zero", {
        use crate::Vector;

        let zero = Vector::new(0.0, 0.0, 0.0);
        let nonzero = Vector::new(1.0, 0.0, 0.0);
        let tiny = Vector::new(1e-13, 1e-13, 1e-13); // Magnitude ~ 1.7e-13 < 1e-12

        MINI_CHECK!(zero.is_zero());
        MINI_CHECK!(!nonzero.is_zero());
        MINI_CHECK!(tiny.is_zero()); // Within tolerance
    })
}

pub fn run_vector_scale() -> TestResult {
    MINI_TEST!("Scale", {
        use crate::Vector;

        let mut v = Vector::new(2.0, 4.0, 6.0);
        v.scale(0.5);
        let mut v_up = Vector::new(1.0, 2.0, 3.0);
        v_up.scale_up();
        let mut v_rt = Vector::new(1.0, 2.0, 3.0);
        v_rt.scale_up();
        v_rt.scale_down();

        MINI_CHECK!(v[0] == 1.0 && v[1] == 2.0 && v[2] == 3.0);
        MINI_CHECK!(v_up[0] > 1.0);
        MINI_CHECK!(TOLERANCE.is_close(v_rt[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(v_rt[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(v_rt[2], 3.0));
    })
}

pub fn run_vector_reflect() -> TestResult {
    MINI_TEST!("Reflect", {
        use crate::Vector;

        let v = Vector::new(1.0, 2.0, 3.0);
        let n = Vector::x_axis();
        let r = v.reflect(&n);

        MINI_CHECK!(TOLERANCE.is_close(r[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(r[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(r[2], 3.0));
    })
}

pub fn run_vector_average_normal() -> TestResult {
    MINI_TEST!("Average Normal", {
        use crate::Point;
        use crate::Vector;

        let sq = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 0.0),
        ];
        let n = Vector::average_normal(&sq);

        MINI_CHECK!(TOLERANCE.is_close(n[2].abs(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(n[0], 0.0) && TOLERANCE.is_close(n[1], 0.0));
    })
}

pub fn run_vector_interpolate_points() -> TestResult {
    MINI_TEST!("Interpolate Points", {
        use crate::vector::interpolate_points;
        use crate::Point;

        let from_pt = Point::new(0.0, 0.0, 0.0);
        let to_pt = Point::new(1.0, 0.0, 0.0);
        let pts0 = interpolate_points(&from_pt, &to_pt, 2, 0);
        let pts1 = interpolate_points(&from_pt, &to_pt, 1, 1);

        MINI_CHECK!(pts0.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(pts0[0][0], 1.0 / 3.0));
        MINI_CHECK!(TOLERANCE.is_close(pts0[1][0], 2.0 / 3.0));
        MINI_CHECK!(pts1.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pts1[0][0], 0.0) && TOLERANCE.is_close(pts1[2][0], 1.0));
    })
}

pub fn run_vector_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Vector;

        let mut v = Vector::new(42.1, 84.2, 126.3);
        v.name = "test_vector".to_string();

        //   json_dumps()    │ String       │ to JSON string
        //   json_loads(s)   │ String       │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        let filename = "serialization/test_vector.json";
        v.json_dump(filename).unwrap();
        let loaded = Vector::json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_vector");
        MINI_CHECK!(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], 126.3));
    })
}

pub fn run_vector_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Vector;

        let mut v = Vector::new(42.1, 84.2, 126.3);
        v.name = "test_vector".to_string();

        // Test pb_dump / pb_load (file-based)
        let filename = "serialization/test_vector.bin";
        v.pb_dump(filename);
        let loaded = Vector::pb_load(filename);

        MINI_CHECK!(loaded.name == "test_vector");
        MINI_CHECK!(TOLERANCE.is_close(loaded[0], 42.1));
        MINI_CHECK!(TOLERANCE.is_close(loaded[1], 84.2));
        MINI_CHECK!(TOLERANCE.is_close(loaded[2], 126.3));
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Vector", "Constructor", crate::vector_test::run_vector_constructor);
REGISTER_MINI_TEST!("Vector", "Transformation", crate::vector_test::run_vector_transformation);
REGISTER_MINI_TEST!("Vector", "Magnitude", crate::vector_test::run_vector_magnitude);
REGISTER_MINI_TEST!("Vector", "Normalize", crate::vector_test::run_vector_normalize);
REGISTER_MINI_TEST!("Vector", "Reverse", crate::vector_test::run_vector_reverse);
REGISTER_MINI_TEST!("Vector", "Dot Product", crate::vector_test::run_vector_dot_product);
REGISTER_MINI_TEST!("Vector", "Cross Product", crate::vector_test::run_vector_cross_product);
REGISTER_MINI_TEST!("Vector", "Angle", crate::vector_test::run_vector_angle);
REGISTER_MINI_TEST!("Vector", "Projection", crate::vector_test::run_vector_projection);
REGISTER_MINI_TEST!("Vector", "Is Parallel To", crate::vector_test::run_vector_is_parallel_to);
REGISTER_MINI_TEST!("Vector", "Is Perpendicular To", crate::vector_test::run_vector_is_perpendicular_to);
REGISTER_MINI_TEST!("Vector", "Get Leveled Vector", crate::vector_test::run_vector_get_leveled_vector);
REGISTER_MINI_TEST!("Vector", "Cos Sin Laws", crate::vector_test::run_vector_cos_sin_laws);
REGISTER_MINI_TEST!("Vector", "Sum Of Vectors", crate::vector_test::run_vector_sum_of_vectors);
REGISTER_MINI_TEST!("Vector", "Average", crate::vector_test::run_vector_average);
REGISTER_MINI_TEST!("Vector", "Is Zero", crate::vector_test::run_vector_is_zero);
REGISTER_MINI_TEST!("Vector", "Scale", crate::vector_test::run_vector_scale);
REGISTER_MINI_TEST!("Vector", "Reflect", crate::vector_test::run_vector_reflect);
REGISTER_MINI_TEST!("Vector", "Average Normal", crate::vector_test::run_vector_average_normal);
REGISTER_MINI_TEST!("Vector", "Interpolate Points", crate::vector_test::run_vector_interpolate_points);
REGISTER_MINI_TEST!("Vector", "Json Roundtrip", crate::vector_test::run_vector_json_roundtrip);
REGISTER_MINI_TEST!("Vector", "Protobuf Roundtrip", crate::vector_test::run_vector_protobuf_roundtrip);
