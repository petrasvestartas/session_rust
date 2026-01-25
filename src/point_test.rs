use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_point_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        // Constructor
        let mut p = Point::new(1.0, 2.0, 3.0);

        // Setters
        p[0] = 10.0;
        p[1] = 20.0;
        p[2] = 30.0;

        // Getters
        let x = p[0];
        let y = p[1];
        let z = p[2];

        // Minimal and full string representation
        let pstr = p.str();
        let prepr = p.repr();

        // Copy (duplicate everything but guid)
        let pcopy = p.duplicate();
        let pother = Point::new(1.0, 2.0, 3.0);

        // No-copy operators
        let mut pmult = p.duplicate();
        pmult *= 2.0;
        let mut pdiv = p.duplicate();
        pdiv /= 2.0;
        let mut padd = p.duplicate();
        padd += Vector::new(1.0, 1.0, 1.0);
        let mut psub = p.duplicate();
        psub -= Vector::new(1.0, 1.0, 1.0);

        // Copy operators
        let result_mul = &p * 2.0;
        let result_div = &p / 2.0;
        let result_add = &p + Vector::new(1.0, 1.0, 1.0);
        let diff_point = &p - Vector::new(1.0, 1.0, 1.0);

        // Static sum and sub methods
        let p1 = Point::new(1.0, 2.0, 3.0);
        let p2 = Point::new(4.0, 5.0, 6.0);
        let psum = Point::sum(&p1, &p2);
        let pdif = Point::sub(&p2, &p1);

        MINI_CHECK!(
            p.name == "my_point" &&
            p[0] == 10.0 &&
            p[1] == 20.0 &&
            p[2] == 30.0 &&
            p.width == 1.0 &&
            p.pointcolor == Color::blue() &&
            !p.guid.is_empty()
        );
        MINI_CHECK!(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK!(pstr == "10.000000, 20.000000, 30.000000");
        MINI_CHECK!(prepr == "Point(my_point, 10.000000, 20.000000, 30.000000, Color(0, 0, 255, 255), 1.000000)");
        MINI_CHECK!(pcopy == p && pcopy.guid != p.guid);
        MINI_CHECK!(pother != p);
        MINI_CHECK!(pmult[0] == 20.0 && pmult[1] == 40.0 && pmult[2] == 60.0);
        MINI_CHECK!(pdiv[0] == 5.0 && pdiv[1] == 10.0 && pdiv[2] == 15.0);
        MINI_CHECK!(padd[0] == 11.0 && padd[1] == 21.0 && padd[2] == 31.0);
        MINI_CHECK!(psub[0] == 9.0 && psub[1] == 19.0 && psub[2] == 29.0);
        MINI_CHECK!(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
        MINI_CHECK!(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
        MINI_CHECK!(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
        MINI_CHECK!(diff_point[0] == 9.0 && diff_point[1] == 19.0 && diff_point[2] == 29.0);
        MINI_CHECK!(psum[0] == 5.0 && psum[1] == 7.0 && psum[2] == 9.0);
        MINI_CHECK!(pdif[0] == 3.0 && pdif[1] == 3.0 && pdif[2] == 3.0);
    })
}

pub fn run_point_transformation() -> TestResult {
    MINI_TEST!("transformation", {
        use crate::Point;
        use crate::Xform;

        let mut p = Point::new(1.0, 2.0, 3.0);
        p.xform = Xform::translation(1.0, 2.0, 3.0);
        let p_transformed = p.transformed(); // Make a copy
        p.transform(); // After the call, "xform" is reset

        MINI_CHECK!(p_transformed[0] == 2.0 && p_transformed[1] == 4.0 && p_transformed[2] == 6.0);
        MINI_CHECK!(p[0] == 2.0 && p[1] == 4.0 && p[2] == 6.0);
        MINI_CHECK!(p.xform == Xform::identity());
    })
}

pub fn run_point_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::Point;
        use crate::Color;

        let mut p = Point::with_name(1.5, 2.5, 3.5, "test_point");
        p.width = 2.0;
        p.pointcolor = Color::new(255, 128, 64, 255);

        let filename = "serialization/test_point.json";
        p.to_json(filename).unwrap();
        let loaded = Point::from_json(filename).unwrap();

        MINI_CHECK!(loaded.name == p.name);
        MINI_CHECK!(loaded[0] == p[0]);
        MINI_CHECK!(loaded[1] == p[1]);
        MINI_CHECK!(loaded[2] == p[2]);
        MINI_CHECK!(loaded.width == p.width);
        MINI_CHECK!(loaded.pointcolor.r == 255);
        MINI_CHECK!(loaded.pointcolor.g == 128);
        MINI_CHECK!(loaded.pointcolor.b == 64);
        MINI_CHECK!(loaded.pointcolor.a == 255);

    })
}

pub fn run_point_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::Point;
        use crate::Color;

        let mut p = Point::new(1.5, 2.5, 3.5);
        p.name = "test_point".to_string();
        p.width = 2.0;
        p.pointcolor = Color::new(255, 128, 64, 255);

        let filename = "serialization/test_point.bin";
        p.protobuf_dump(filename);
        let loaded = Point::protobuf_load(filename);

        MINI_CHECK!(loaded.name == p.name);
        MINI_CHECK!(loaded[0] == p[0]);
        MINI_CHECK!(loaded[1] == p[1]);
        MINI_CHECK!(loaded[2] == p[2]);
        MINI_CHECK!(loaded.width == p.width);
        MINI_CHECK!(loaded.pointcolor.r == 255);
        MINI_CHECK!(loaded.pointcolor.g == 128);
        MINI_CHECK!(loaded.pointcolor.b == 64);
        MINI_CHECK!(loaded.pointcolor.a == 255);
    })
}

pub fn run_point_is_ccw() -> TestResult {
    MINI_TEST!("is_ccw", {
        use crate::Point;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(1.0, 0.0, 0.0);
        let p2 = Point::new(0.05, 1.0, 0.0);

        // Points must be oriented to xy plane.
        let is_counter_clock_wise = Point::is_ccw(&p0, &p1, &p2);
        let is_clock_wise = Point::is_ccw(&p2, &p1, &p0);

        MINI_CHECK!(is_counter_clock_wise);
        MINI_CHECK!(!is_clock_wise);
    })
}

pub fn run_point_mid_point() -> TestResult {
    MINI_TEST!("mid_point", {
        use crate::Point;

        let p0 = Point::new(0.0, 2.0, 1.0);
        let p1 = Point::new(1.0, 5.0, 3.0);
        let mid = Point::mid_point(&p0, &p1);

        MINI_CHECK!(mid[0] == 0.5 && mid[1] == 3.5 && mid[2] == 2.0);
    })
}

pub fn run_point_distance() -> TestResult {
    MINI_TEST!("distance", {
        use crate::Point;
        use crate::tolerance::TOLERANCE;

        let p0 = Point::new(0.0, 2.0, 1.0);
        let p1 = Point::new(1.0, 5.0, 3.0);
        let d = p0.distance(&p1, None);

        MINI_CHECK!(TOLERANCE.is_close(d, 3.741657));
    })
}

pub fn run_point_squared_distance() -> TestResult {
    MINI_TEST!("squared_distance", {
        use crate::Point;
        use crate::tolerance::TOLERANCE;

        let p0 = Point::new(0.0, 2.0, 1.0);
        let p1 = Point::new(1.0, 5.0, 3.0);
        let d = p0.squared_distance(&p1, None);

        MINI_CHECK!(TOLERANCE.is_close(d, 14.0));
    })
}

pub fn run_point_area() -> TestResult {
    MINI_TEST!("area", {
        use crate::Point;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(2.0, 0.0, 0.0);
        let p2 = Point::new(2.0, 2.0, 0.0);
        let p3 = Point::new(0.0, 2.0, 0.0);
        let area = Point::area(&vec![p0, p1, p2, p3]);

        MINI_CHECK!(area == 4.0);
    })
}

pub fn run_point_centroid_quad() -> TestResult {
    MINI_TEST!("centroid_quad", {
        use crate::Point;
        use crate::tolerance::TOLERANCE;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(2.0, 0.0, 1.0);
        let p2 = Point::new(2.0, 2.0, 2.0);
        let p3 = Point::new(0.0, 2.0, 1.0);
        let centroid = Point::centroid_quad(&vec![p0, p1, p2, p3]).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(centroid[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(centroid[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(centroid[2], 1.0));
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Point", "constructor", crate::point_test::run_point_constructor);
REGISTER_MINI_TEST!("Point", "transformation", crate::point_test::run_point_transformation);
REGISTER_MINI_TEST!("Point", "json_roundtrip", crate::point_test::run_point_json_roundtrip);
REGISTER_MINI_TEST!("Point", "protobuf_roundtrip", crate::point_test::run_point_protobuf_roundtrip);
REGISTER_MINI_TEST!("Point", "is_ccw", crate::point_test::run_point_is_ccw);
REGISTER_MINI_TEST!("Point", "mid_point", crate::point_test::run_point_mid_point);
REGISTER_MINI_TEST!("Point", "distance", crate::point_test::run_point_distance);
REGISTER_MINI_TEST!("Point", "squared_distance", crate::point_test::run_point_squared_distance);
REGISTER_MINI_TEST!("Point", "area", crate::point_test::run_point_area);
REGISTER_MINI_TEST!("Point", "centroid_quad", crate::point_test::run_point_centroid_quad);