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
        MINI_CHECK!(bb1.collides_with_rtcd(&bb2));
        MINI_CHECK!(!bb1.collides_with_rtcd(&bb3));
        MINI_CHECK!(bb1.collides_with_naive(&bb2));
        MINI_CHECK!(!bb1.collides_with_naive(&bb3));
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
        let bb_xf = Xform::translation(0.0, 0.0, 5.0);

        let bbt = bb.transformed(&bb_xf);

        MINI_CHECK!(TOLERANCE.is_close(bbt.center[2], 5.0));

        bb.transform(&bb_xf);

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
        let s = bb.file_json_dumps();
        let loaded_s = OBB::file_json_loads(&s);

        MINI_CHECK!(loaded_s.name == "test_bbox");
        MINI_CHECK!(TOLERANCE.is_close(loaded_s.half_size[0], 5.0));

        // File
        let src_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let fname = src_dir.join("serialization").join("test_obb.json");
        let fname = fname.to_str().unwrap();
        bb.file_json_dump(fname);
        let loaded = OBB::file_json_load(fname);

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
    use crate::{OBB, Point, Vector};
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

        let c45 = 2.0_f64.sqrt() * 0.5;
        let rotated = OBB::new(Point::new(0.0, 0.0, 0.0), Vector::new(c45, c45, 0.0), Vector::new(-c45, c45, 0.0), Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 1.0, 1.0));

        MINI_CHECK!(TOLERANCE.is_close(rotated.min_point()[0], -2.0_f64.sqrt()));
        MINI_CHECK!(TOLERANCE.is_close(rotated.max_point()[1], 2.0_f64.sqrt()));

        let c = OBB::from_point(Point::new(5.0, 2.0, 3.0), 1.0);
        b.union_with(&c);

        MINI_CHECK!(TOLERANCE.is_close(b.half_size[0], 3.0));
    })
}

pub fn run_obb_from_geometry() -> TestResult {
    MINI_TEST!("From Geometry", {
        use crate::Color;
        use crate::Line;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::OBB;
        use crate::Plane;
        use crate::Point;
        use crate::PointCloud;
        use crate::Polyline;
        use crate::Primitives;
        use crate::Vector;

        let bb_line = OBB::from_line(&Line::new(0.0, 0.0, 0.0, 4.0, 0.0, 0.0), 0.1);

        MINI_CHECK!(bb_line.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(bb_line.center[0], 2.0));

        let bb_pl = OBB::from_polyline(&Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 4.0),
        ]), 0.0);

        MINI_CHECK!(bb_pl.is_valid());
        MINI_CHECK!(bb_pl.volume() > 0.0);

        let bb_mesh = OBB::from_mesh(&Primitives::cube(2.0), 0.0);

        MINI_CHECK!(bb_mesh.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(bb_mesh.center[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(bb_mesh.volume(), 8.0));

        let bb_pc = OBB::from_pointcloud(&PointCloud::new(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(0.0, 2.0, 0.0),
                Point::new(0.0, 0.0, 2.0),
            ],
            vec![
                Vector::new(0.0, 0.0, 1.0),
                Vector::new(0.0, 0.0, 1.0),
                Vector::new(0.0, 0.0, 1.0),
                Vector::new(0.0, 0.0, 1.0),
            ],
            vec![
                Color::new(1.0, 0.0, 0.0, 1.0),
                Color::new(0.0, 1.0, 0.0, 1.0),
                Color::new(0.0, 0.0, 1.0, 1.0),
                Color::new(1.0, 1.0, 0.0, 1.0),
            ],
        ), 0.0);

        MINI_CHECK!(bb_pc.is_valid());
        MINI_CHECK!(bb_pc.volume() > 0.0);

        let bb_nc = OBB::from_nurbscurve(&NurbsCurve::create(false, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]), 0.5, false);

        MINI_CHECK!(bb_nc.is_valid());

        let surf_ns = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(2.0, 2.0, 2.0),
        ]).unwrap();
        let bb_ns = OBB::from_nurbssurface(&surf_ns, 0.0);

        MINI_CHECK!(bb_ns.is_valid());

        let plane = Plane::xy_plane();
        let bb_pl_plane = OBB::from_polyline_with_plane(&Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
            Point::new(4.0, 4.0, 4.0),
        ]), &plane, 0.0);
        let bb_mesh_plane = OBB::from_mesh_with_plane(&Primitives::cube(2.0), &plane, 0.0);

        MINI_CHECK!(TOLERANCE.is_close(bb_pl_plane.half_size[0], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(bb_mesh_plane.volume(), 8.0));
    })
}

pub fn run_obb_from_plane() -> TestResult {
    MINI_TEST!("From Plane", {
        use crate::OBB;
        use crate::Plane;
        use crate::Point;
        use crate::Vector;

        let plane = Plane::xy_plane();
        let box_ = OBB::from_plane(&plane, 2.0, 3.0, 4.0);

        MINI_CHECK!(TOLERANCE.is_close(box_.half_size[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(box_.half_size[1], 1.5));
        MINI_CHECK!(TOLERANCE.is_close(box_.half_size[2], 2.0));
        MINI_CHECK!(box_.center == Point::new(0.0, 0.0, 0.0));

        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 3.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
        ];
        let bb = OBB::from_points_with_plane(&pts, &plane, 0.0);

        MINI_CHECK!(TOLERANCE.is_close(bb.half_size[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(bb.half_size[1], 1.5));
        MINI_CHECK!(TOLERANCE.is_close(bb.x_axis[0], 1.0));

        let c45 = 2.0_f64.sqrt() * 0.5;
        let rotated = Plane::new(Point::new(10.0, 0.0, 0.0), Vector::new(c45, c45, 0.0), Vector::new(-c45, c45, 0.0));
        let mut box_pts: Vec<Point> = Vec::new();
        for u in [0.0, 2.0] {
            for v in [0.0, 4.0] {
                for w in [0.0, 6.0] {
                    box_pts.push(Point::new(10.0 + (u - v) * c45, (u + v) * c45, w));
                }
            }
        }
        let rb = OBB::from_points_with_plane(&box_pts, &rotated, 0.0);

        MINI_CHECK!(TOLERANCE.is_close(rb.half_size[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(rb.half_size[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(rb.center[0], 10.0 - c45));
        MINI_CHECK!(TOLERANCE.is_close(rb.center[1], 3.0 * c45));
    })
}

pub fn run_obb_two_rectangles() -> TestResult {
    MINI_TEST!("Two Rectangles", {
        use crate::OBB;
        use crate::Point;
        use crate::Vector;

        let bb = OBB::new(
            Point::new(1.0, 2.0, 3.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(2.0, 3.0, 4.0),
        );
        let rects = bb.two_rectangles();

        // bottom rect (z=-4 offset): corners at z=-1; top rect (z=+4 offset): corners at z=7
        MINI_CHECK!(rects.len() == 10);
        MINI_CHECK!(rects[0] == Point::new(3.0, 5.0, -1.0));
        MINI_CHECK!(rects[2] == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(rects[4] == rects[0]);
        MINI_CHECK!(rects[5] == Point::new(3.0, 5.0, 7.0));
        MINI_CHECK!(rects[7] == Point::new(-1.0, -1.0, 7.0));
        MINI_CHECK!(rects[9] == rects[5]);
    })
}

REGISTER_MINI_TEST!("OBB", "Constructor", crate::obb_test::run_obb_constructor);
REGISTER_MINI_TEST!("OBB", "Collision", crate::obb_test::run_obb_collision);
REGISTER_MINI_TEST!("OBB", "Transformation", crate::obb_test::run_obb_transformation);
REGISTER_MINI_TEST!("OBB", "Json Roundtrip", crate::obb_test::run_obb_json_roundtrip);
REGISTER_MINI_TEST!("OBB", "Protobuf Roundtrip", crate::obb_test::run_obb_protobuf_roundtrip);
REGISTER_MINI_TEST!("OBB", "Accessors", crate::obb_test::run_obb_accessors);
REGISTER_MINI_TEST!("OBB", "From Geometry", crate::obb_test::run_obb_from_geometry);
REGISTER_MINI_TEST!("OBB", "From Plane", crate::obb_test::run_obb_from_plane);
REGISTER_MINI_TEST!("OBB", "Two Rectangles", crate::obb_test::run_obb_two_rectangles);

