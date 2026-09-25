use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_point_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;
        use crate::Point;
        use crate::Vector;

        let mut p = Point::new(1.0, 2.0, 3.0);

        p[0] = 10.0;
        p[1] = 20.0;
        p[2] = 30.0;

        let x = p[0];
        let y = p[1];
        let z = p[2];

        let pstr = p.str();
        let prepr = p.repr();

        let pcopy = p.duplicate();
        let pother = Point::new(1.0, 2.0, 3.0);

        let mut pmult = p.duplicate();
        pmult *= 2.0;

        let mut pdiv = p.duplicate();
        pdiv /= 2.0;

        let mut padd = p.duplicate();
        padd += Vector::new(1.0, 1.0, 1.0);

        let mut psub = p.duplicate();
        psub -= Vector::new(1.0, 1.0, 1.0);

        let result_mul = &p * 2.0;
        let result_div = &p / 2.0;
        let result_add = &p + Vector::new(1.0, 1.0, 1.0);
        let result_sub = &p - Vector::new(1.0, 1.0, 1.0);
        let result_diff = &p - &pother;

        let p1 = Point::new(1.0, 2.0, 3.0);
        let p2 = Point::new(4.0, 5.0, 6.0);
        let psum = Point::sum(&p1, &p2);
        let pdif = &p2 - &p1;

        let mut pguid = Point::new(1.0, 2.0, 3.0);
        let minted = pguid.guid().to_string();
        pguid.set_guid("custom_guid".to_string());

        MINI_CHECK!(p.name == "my_point");
        MINI_CHECK!(p[0] == 10.0 && p[1] == 20.0 && p[2] == 30.0);
        MINI_CHECK!(p.width == 1.0);
        MINI_CHECK!(p.pointcolor == Color::black());
        MINI_CHECK!(!p.guid().is_empty());
        MINI_CHECK!(x == 10.0 && y == 20.0 && z == 30.0);
        MINI_CHECK!(pstr == "10.000000, 20.000000, 30.000000");
        MINI_CHECK!(prepr == "Point(my_point, 10.000000, 20.000000, 30.000000, Color(black, 0.0, 0.0, 0.0, 1.0), 1.000000)");
        MINI_CHECK!(pcopy == p && pcopy.guid() != p.guid());
        MINI_CHECK!(pother != p);
        MINI_CHECK!(pmult[0] == 20.0 && pmult[1] == 40.0 && pmult[2] == 60.0);
        MINI_CHECK!(pdiv[0] == 5.0 && pdiv[1] == 10.0 && pdiv[2] == 15.0);
        MINI_CHECK!(padd[0] == 11.0 && padd[1] == 21.0 && padd[2] == 31.0);
        MINI_CHECK!(psub[0] == 9.0 && psub[1] == 19.0 && psub[2] == 29.0);
        MINI_CHECK!(result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0);
        MINI_CHECK!(result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0);
        MINI_CHECK!(result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0);
        MINI_CHECK!(result_sub[0] == 9.0 && result_sub[1] == 19.0 && result_sub[2] == 29.0);
        MINI_CHECK!(result_diff[0] == 9.0 && result_diff[1] == 18.0 && result_diff[2] == 27.0);
        MINI_CHECK!(psum[0] == 5.0 && psum[1] == 7.0 && psum[2] == 9.0);
        MINI_CHECK!(pdif[0] == 3.0 && pdif[1] == 3.0 && pdif[2] == 3.0);
        MINI_CHECK!(pguid.guid() != minted && pguid.guid() == "custom_guid");
    })
}

pub fn run_point_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::Point;
        use crate::Xform;

        let mut p = Point::new(1.0, 2.0, 3.0);
        let xform = Xform::translation(1.0, 2.0, 3.0);
        let moved = p.transformed(&xform);
        p.transform(&xform);

        MINI_CHECK!(moved[0] == 2.0 && moved[1] == 4.0 && moved[2] == 6.0);
        MINI_CHECK!(p[0] == 2.0 && p[1] == 4.0 && p[2] == 6.0);
    })
}

pub fn run_point_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Color;
        use crate::Point;

        let mut p = Point::with_name(1.5, 2.5, 3.5, "test_point");
        p.width = 2.0;
        p.pointcolor = Color::new(1.0, 0.5, 0.25, 1.0);

        let guid = p.guid().to_string();
        let filename = "serialization/test_point.json";
        p.file_json_dump(filename).unwrap();

        let loaded = Point::file_json_load(filename).unwrap();
        let parsed = Point::file_json_loads(&p.file_json_dumps());

        MINI_CHECK!(loaded.name == "test_point");
        MINI_CHECK!(loaded[0] == 1.5 && loaded[1] == 2.5 && loaded[2] == 3.5);
        MINI_CHECK!(loaded.width == 2.0);
        MINI_CHECK!(loaded.pointcolor[0] == 1.0);
        MINI_CHECK!(loaded.pointcolor[1] == 0.5);
        MINI_CHECK!(loaded.pointcolor[2] == 0.25);
        MINI_CHECK!(loaded.pointcolor[3] == 1.0);
        MINI_CHECK!(parsed == p);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(parsed.guid() == guid);
    })
}

pub fn run_point_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Color;
        use crate::Point;

        let fresh = Point::default();
        let fresh_proto = fresh.to_proto();
        let mut p = Point::with_name(1.5, 2.5, 3.5, "test_point");
        p.width = 2.0;
        p.pointcolor = Color::new(1.0, 0.5, 0.25, 1.0);

        let guid = p.guid().to_string();
        let filename = "serialization/test_point.bin";
        p.pb_dump(filename).unwrap();

        let loaded = Point::pb_load(filename).unwrap();
        let parsed = Point::pb_loads(&p.pb_dumps()).unwrap();
        let converted = Point::from_proto(p.to_proto());

        MINI_CHECK!(!fresh.has_guid());
        MINI_CHECK!(fresh_proto.guid.is_empty());
        MINI_CHECK!(loaded.name == "test_point");
        MINI_CHECK!(loaded[0] == 1.5 && loaded[1] == 2.5 && loaded[2] == 3.5);
        MINI_CHECK!(loaded.width == 2.0);
        MINI_CHECK!(loaded.pointcolor[0] == 1.0);
        MINI_CHECK!(loaded.pointcolor[1] == 0.5);
        MINI_CHECK!(loaded.pointcolor[2] == 0.25);
        MINI_CHECK!(loaded.pointcolor[3] == 1.0);
        MINI_CHECK!(parsed == p);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(parsed.guid() == guid);
        MINI_CHECK!(converted == p);
        MINI_CHECK!(converted.guid() == guid);
    })
}

pub fn run_point_is_ccw() -> TestResult {
    MINI_TEST!("Is Ccw", {
        use crate::Point;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(1.0, 0.0, 0.0);
        let p2 = Point::new(0.05, 1.0, 0.0);
        let ccw = Point::is_ccw(&p0, &p1, &p2);
        let cw = Point::is_ccw(&p2, &p1, &p0);

        MINI_CHECK!(ccw);
        MINI_CHECK!(!cw);
    })
}

pub fn run_point_mid_point() -> TestResult {
    MINI_TEST!("Mid Point", {
        use crate::Point;

        let p0 = Point::new(0.0, 2.0, 1.0);
        let p1 = Point::new(1.0, 5.0, 3.0);
        let mid = Point::mid_point(&p0, &p1);

        MINI_CHECK!(mid[0] == 0.5 && mid[1] == 3.5 && mid[2] == 2.0);
    })
}

pub fn run_point_distance() -> TestResult {
    MINI_TEST!("Distance", {
        use crate::Point;

        let p0 = Point::new(0.0, 2.0, 1.0);
        let p1 = Point::new(1.0, 5.0, 3.0);
        let d = Point::distance(&p0, &p1, None);

        MINI_CHECK!(TOLERANCE.is_close(d, 3.741657));
    })
}

pub fn run_point_squared_distance() -> TestResult {
    MINI_TEST!("Squared Distance", {
        use crate::Point;

        let p0 = Point::new(0.0, 2.0, 1.0);
        let p1 = Point::new(1.0, 5.0, 3.0);
        let d = Point::squared_distance(&p0, &p1, None);

        MINI_CHECK!(TOLERANCE.is_close(d, 14.0));
    })
}

pub fn run_point_interpolate() -> TestResult {
    MINI_TEST!("Interpolate", {
        use crate::Point;

        let a = Point::new(0.0, 0.0, 0.0);
        let b = Point::new(4.0, 8.0, 12.0);
        let half = Point::lerp(&a, &b, 0.5);
        let inner = Point::interpolate(&a, &b, 3, 0);
        let both = Point::interpolate(&a, &b, 3, 1);
        let start = Point::interpolate(&a, &b, 3, 2);

        MINI_CHECK!(half[0] == 2.0 && half[1] == 4.0 && half[2] == 6.0);
        MINI_CHECK!(inner.len() == 3);
        MINI_CHECK!(inner[0][0] == 1.0 && inner[1][0] == 2.0 && inner[2][0] == 3.0);
        MINI_CHECK!(both.len() == 5);
        MINI_CHECK!(both[0][0] == 0.0 && both[4][0] == 4.0);
        MINI_CHECK!(start.len() == 4);
        MINI_CHECK!(start[0][0] == 0.0 && start[3][0] == 3.0);
    })
}

pub fn run_point_area() -> TestResult {
    MINI_TEST!("Area", {
        use crate::Point;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(2.0, 0.0, 0.0);
        let p2 = Point::new(2.0, 2.0, 0.0);
        let p3 = Point::new(0.0, 2.0, 0.0);
        let area = Point::area(&[p0, p1, p2, p3]);

        MINI_CHECK!(area == 4.0);
    })
}

pub fn run_point_centroid_quad() -> TestResult {
    MINI_TEST!("Centroid Quad", {
        use crate::Point;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(2.0, 0.0, 1.0);
        let p2 = Point::new(2.0, 2.0, 2.0);
        let p3 = Point::new(0.0, 2.0, 1.0);
        let centroid = Point::centroid_quad(&[p0, p1, p2, p3]).unwrap();

        MINI_CHECK!(TOLERANCE.is_close(centroid[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(centroid[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(centroid[2], 1.0));
    })
}

pub fn run_point_centroid() -> TestResult {
    MINI_TEST!("Centroid", {
        use crate::Point;

        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(2.0, 0.0, 0.0);
        let p2 = Point::new(2.0, 2.0, 0.0);
        let p3 = Point::new(0.0, 2.0, 0.0);
        let centroid = Point::centroid(&[p0, p1, p2, p3]);

        MINI_CHECK!(TOLERANCE.is_close(centroid[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(centroid[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(centroid[2], 0.0));
    })
}

pub fn run_point_dihedral_angle_deg() -> TestResult {
    MINI_TEST!("Dihedral Angle Deg", {
        use crate::Point;

        let p = Point::new(0.0, 0.0, 0.0);
        let q = Point::new(1.0, 0.0, 0.0);
        let r = Point::new(0.0, 1.0, 0.0);
        let s = Point::new(0.0, 0.0, 1.0);
        let angle = Point::dihedral_angle_deg(&p, &q, &r, &s);

        MINI_CHECK!(TOLERANCE.is_close(angle, 90.0));
    })
}

REGISTER_MINI_TEST!(
    "Point",
    "Constructor",
    crate::point_test::run_point_constructor
);
REGISTER_MINI_TEST!(
    "Point",
    "Transformation",
    crate::point_test::run_point_transformation
);
REGISTER_MINI_TEST!(
    "Point",
    "Json Roundtrip",
    crate::point_test::run_point_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Point",
    "Protobuf Roundtrip",
    crate::point_test::run_point_protobuf_roundtrip
);
REGISTER_MINI_TEST!("Point", "Is Ccw", crate::point_test::run_point_is_ccw);
REGISTER_MINI_TEST!("Point", "Mid Point", crate::point_test::run_point_mid_point);
REGISTER_MINI_TEST!("Point", "Distance", crate::point_test::run_point_distance);
REGISTER_MINI_TEST!(
    "Point",
    "Squared Distance",
    crate::point_test::run_point_squared_distance
);
REGISTER_MINI_TEST!(
    "Point",
    "Interpolate",
    crate::point_test::run_point_interpolate
);
REGISTER_MINI_TEST!("Point", "Area", crate::point_test::run_point_area);
REGISTER_MINI_TEST!(
    "Point",
    "Centroid Quad",
    crate::point_test::run_point_centroid_quad
);
REGISTER_MINI_TEST!("Point", "Centroid", crate::point_test::run_point_centroid);
REGISTER_MINI_TEST!(
    "Point",
    "Dihedral Angle Deg",
    crate::point_test::run_point_dihedral_angle_deg
);
