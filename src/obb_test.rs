use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_obb_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::OBB;
        use crate::Point;
        use crate::Vector;

        // from_point
        let mut bb1 = OBB::from_point(Point::new(5.0, 5.0, 5.0), 2.0);

        MINI_CHECK!(TOLERANCE.is_close(bb1.center[0], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(bb1.half_size[0], 2.0));

        // from_points (AABB)
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 3.0, 4.0),
        ];
        let bb2 = OBB::from_points(&pts, 0.0);
        let mn = bb2.min_point();
        let mx = bb2.max_point();

        MINI_CHECK!(TOLERANCE.is_close(mn[0], 0.0) && TOLERANCE.is_close(mn[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(mx[0], 2.0) && TOLERANCE.is_close(mx[2], 4.0));

        // OBB constructor
        let obb = OBB::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(1.0, 2.0, 3.0),
        );

        MINI_CHECK!(TOLERANCE.is_close(obb.half_size[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(obb.half_size[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(obb.half_size[2], 3.0));

        // aabb
        let bb_aabb = bb2.aabb();

        MINI_CHECK!(TOLERANCE.is_close(bb_aabb.min_point()[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(bb_aabb.max_point()[2], 4.0));

        // corners
        let corners = bb2.corners();

        MINI_CHECK!(corners.len() == 8);

        // point_at: center + x*x_axis + y*y_axis + z*z_axis (raw OBB offsets)
        let p_center = bb2.point_at(0.0, 0.0, 0.0);
        let hx = bb2.half_size[0];
        let hy = bb2.half_size[1];
        let hz = bb2.half_size[2];
        let p_max_pt = bb2.point_at(hx, hy, hz);

        MINI_CHECK!(TOLERANCE.is_close(p_center[0], 1.0) && TOLERANCE.is_close(p_center[2], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(p_max_pt[0], 2.0) && TOLERANCE.is_close(p_max_pt[2], 4.0));

        // inflate
        let mut bb3 = OBB::from_points(&[
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 2.0),
        ], 0.0);
        bb3.inflate(1.0);

        MINI_CHECK!(TOLERANCE.is_close(bb3.min_point()[0], -1.0));
        MINI_CHECK!(TOLERANCE.is_close(bb3.max_point()[0], 3.0));

        // guid and name
        MINI_CHECK!(!bb1.guid().is_empty());
        bb1.name = "test_bbox".to_string();

        MINI_CHECK!(bb1.name == "test_bbox");
    })
}

pub fn run_obb_collision() -> TestResult {
    MINI_TEST!("Collision", {
        use crate::OBB;
        use crate::Point;

        let bb1 = OBB::from_point(Point::new(0.0, 0.0, 0.0), 1.0);
        let bb2 = OBB::from_point(Point::new(1.5, 0.0, 0.0), 1.0);
        let bb3 = OBB::from_point(Point::new(5.0, 5.0, 5.0), 0.5);

        MINI_CHECK!(bb1.collides_with(&bb2));
        MINI_CHECK!(!bb1.collides_with(&bb3));
        MINI_CHECK!(bb1.collides_with_broad(&bb2));
        MINI_CHECK!(!bb1.collides_with_broad(&bb3));
    })
}

pub fn run_obb_transformation() -> TestResult {
    MINI_TEST!("Transformation", {
        use crate::OBB;
        use crate::Point;
        use crate::Xform;

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ];
        let mut bb = OBB::from_points(&pts, 0.0);
        bb.xform = Xform::translation(0.0, 0.0, 5.0);

        let bbt = bb.transformed();

        MINI_CHECK!(TOLERANCE.is_close(bbt.center[2], 5.0));

        bb.transform();

        MINI_CHECK!(bb.xform == Xform::identity());
        MINI_CHECK!(TOLERANCE.is_close(bb.center[2], 5.0));
    })
}

pub fn run_obb_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::OBB;
        use crate::Point;

        let mut bb = OBB::from_point(Point::new(1.0, 2.0, 3.0), 5.0);
        bb.name = "test_bbox".to_string();

        // JSON object (string)
        let js = bb.jsondump().unwrap();
        let loaded_j = OBB::jsonload(&js).unwrap();

        MINI_CHECK!(loaded_j.name == "test_bbox");
        MINI_CHECK!(TOLERANCE.is_close(loaded_j.center[0], 1.0));

        // String
        let s = bb.json_dumps();
        let loaded_s = OBB::json_loads(&s);

        MINI_CHECK!(loaded_s.name == "test_bbox");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s.half_size[0], 5.0));

        // File
        let src_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let fname = src_dir.join("serialization").join("test_obb.json");
        let fname = fname.to_str().unwrap();
        bb.json_dump(fname);
        let loaded = OBB::json_load(fname);

        MINI_CHECK!(loaded.name == "test_bbox");
        MINI_CHECK!(TOLERANCE.is_close(loaded.center[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.half_size[0], 5.0));
    })
}

pub fn run_obb_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::OBB;
        use crate::Point;

        let mut bb = OBB::from_point(Point::new(1.0, 2.0, 3.0), 5.0);
        bb.name = "test_bbox_proto".to_string();

        // Bytes
        let b = bb.pb_dumps();
        let loaded_s = OBB::pb_loads(&b).unwrap();

        MINI_CHECK!(loaded_s.name == "test_bbox_proto");
        MINI_CHECK!(loaded_s.guid() == bb.guid());
        MINI_CHECK!(TOLERANCE.is_close(loaded_s.center[0], 1.0));

        // File
        let src_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let fname = src_dir.join("serialization").join("test_obb.bin");
        let fname = fname.to_str().unwrap();
        bb.pb_dump(fname);
        let loaded = OBB::pb_load(fname);

        MINI_CHECK!(loaded.name == "test_bbox_proto");
        MINI_CHECK!(loaded.guid() == bb.guid());
        MINI_CHECK!(TOLERANCE.is_close(loaded.center[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.half_size[0], 5.0));
    })
}

pub fn run_obb_accessors() -> crate::mini_test::TestResult {
    use crate::tolerance::TOLERANCE;
    use crate::{OBB, Point};
    MINI_TEST!("Accessors", {
        // axis-aligned OBB: center=(1,2,3), half_size=(1,2,3), dims 2×4×6
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 4.0, 0.0),
            Point::new(0.0, 4.0, 0.0),
            Point::new(0.0, 0.0, 6.0),
            Point::new(2.0, 4.0, 6.0),
        ];
        let mut b = OBB::from_points(&pts, 0.0);

        MINI_CHECK!(TOLERANCE.is_close(b.area(), 88.0));
        MINI_CHECK!(TOLERANCE.is_close(b.diagonal(), 2.0 * 14.0_f64.sqrt()));
        MINI_CHECK!(b.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(b.volume(), 48.0));
        MINI_CHECK!(b.closest_point(&Point::new(1.0, 2.0, 3.0)) == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(b.closest_point(&Point::new(10.0, 2.0, 3.0)) == Point::new(2.0, 2.0, 3.0));
        MINI_CHECK!(b.contains(&Point::new(1.0, 2.0, 3.0)));
        MINI_CHECK!(!b.contains(&Point::new(10.0, 2.0, 3.0)));
        MINI_CHECK!(b.corner(false, false, false) == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(b.corner(true, true, true) == Point::new(2.0, 4.0, 6.0));
        MINI_CHECK!(b.get_corners().len() == 8);
        MINI_CHECK!(b.get_edges().len() == 12);
        let c = OBB::from_point(Point::new(5.0, 2.0, 3.0), 1.0);
        b.union_with(&c);

        MINI_CHECK!(TOLERANCE.is_close(b.half_size[0], 3.0));
    })
}

REGISTER_MINI_TEST!("OBB", "Constructor", crate::obb_test::run_obb_constructor);
REGISTER_MINI_TEST!("OBB", "Collision", crate::obb_test::run_obb_collision);
REGISTER_MINI_TEST!("OBB", "Transformation", crate::obb_test::run_obb_transformation);
REGISTER_MINI_TEST!("OBB", "Json Roundtrip", crate::obb_test::run_obb_json_roundtrip);
REGISTER_MINI_TEST!("OBB", "Protobuf Roundtrip", crate::obb_test::run_obb_protobuf_roundtrip);
REGISTER_MINI_TEST!("OBB", "Accessors", crate::obb_test::run_obb_accessors);

