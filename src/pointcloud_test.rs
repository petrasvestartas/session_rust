use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_pointcloud_constructor() -> TestResult {
    MINI_TEST!("constructor", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        // Constructor with points, normals, colors
        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(1.0, 0.0, 0.0);
        let p2 = Point::new(0.0, 1.0, 0.0);
        let n0 = Vector::new(0.0, 0.0, 1.0);
        let n1 = Vector::new(0.0, 0.0, 1.0);
        let n2 = Vector::new(0.0, 0.0, 1.0);
        let c0 = Color::new(255, 0, 0, 255);
        let c1 = Color::new(0, 255, 0, 255);
        let c2 = Color::new(0, 0, 255, 255);
        let pc = PointCloud::new(vec![p0, p1, p2], vec![n0, n1, n2], vec![c0, c1, c2]);

        // Basic properties
        let point_count = pc.len();
        let color_count = pc.color_count();
        let normal_count = pc.normal_count();
        let is_empty = pc.is_empty();

        // Minimal and Full String Representation
        let pcstr = pc.str();
        let pcrepr = pc.repr();

        // Copy (duplicates everything except guid)
        let pccopy = pc.duplicate();

        // Get point/color/normal at index
        let pt0 = pc.get_point(0);
        let col0 = pc.get_color(0);
        let norm0 = pc.get_normal(0);

        MINI_CHECK!(pc.name == "my_pointcloud" && !pc.guid.is_empty() && point_count == 3);
        MINI_CHECK!(color_count == 3 && normal_count == 3 && is_empty == false);
        MINI_CHECK!(pcstr.contains("3 points"));
        MINI_CHECK!(pcrepr.contains("PointCloud(my_pointcloud"));
        MINI_CHECK!(pccopy == pc && pccopy.guid != pc.guid);
        MINI_CHECK!(TOLERANCE.is_close(pt0[0], 0.0) && TOLERANCE.is_close(pt0[1], 0.0) && TOLERANCE.is_close(pt0[2], 0.0));
        MINI_CHECK!(col0.r == 255 && col0.g == 0 && col0.b == 0 && col0.a == 255);
        MINI_CHECK!(TOLERANCE.is_close(norm0[0], 0.0) && TOLERANCE.is_close(norm0[1], 0.0) && TOLERANCE.is_close(norm0[2], 1.0));
    })
}

pub fn run_pointcloud_from_coords() -> TestResult {
    MINI_TEST!("from_coords", {
        use crate::PointCloud;

        // Create from flat arrays
        let coords = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let colors = vec![255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255];
        let normals = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let pc = PointCloud::from_coords(coords, colors, normals);

        let point_count = pc.len();
        let color_count = pc.color_count();
        let normal_count = pc.normal_count();

        let pt1 = pc.get_point(1);
        let col1 = pc.get_color(1);
        let norm1 = pc.get_normal(1);

        MINI_CHECK!(point_count == 3 && color_count == 3 && normal_count == 3);
        MINI_CHECK!(TOLERANCE.is_close(pt1[0], 1.0) && TOLERANCE.is_close(pt1[1], 0.0) && TOLERANCE.is_close(pt1[2], 0.0));
        MINI_CHECK!(col1.r == 0 && col1.g == 255 && col1.b == 0 && col1.a == 255);
        MINI_CHECK!(TOLERANCE.is_close(norm1[0], 0.0) && TOLERANCE.is_close(norm1[1], 0.0) && TOLERANCE.is_close(norm1[2], 1.0));
    })
}

pub fn run_pointcloud_add_and_set() -> TestResult {
    MINI_TEST!("add_and_set", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        // Empty cloud
        let mut pc = PointCloud::default();

        // Add points, colors, normals
        pc.add_point(&Point::new(1.0, 2.0, 3.0));
        pc.add_color(&Color::new(128, 64, 32, 255));
        pc.add_normal(&Vector::new(1.0, 0.0, 0.0));

        let point_count = pc.len();
        let color_count = pc.color_count();
        let normal_count = pc.normal_count();

        // Set point/color/normal at index
        pc.set_point(0, &Point::new(4.0, 5.0, 6.0));
        pc.set_color(0, &Color::new(200, 100, 50, 255));
        pc.set_normal(0, &Vector::new(0.0, 1.0, 0.0));

        let pt0 = pc.get_point(0);
        let col0 = pc.get_color(0);
        let norm0 = pc.get_normal(0);

        MINI_CHECK!(point_count == 1 && color_count == 1 && normal_count == 1);
        MINI_CHECK!(TOLERANCE.is_close(pt0[0], 4.0) && TOLERANCE.is_close(pt0[1], 5.0) && TOLERANCE.is_close(pt0[2], 6.0));
        MINI_CHECK!(col0.r == 200 && col0.g == 100 && col0.b == 50 && col0.a == 255);
        MINI_CHECK!(TOLERANCE.is_close(norm0[0], 0.0) && TOLERANCE.is_close(norm0[1], 1.0) && TOLERANCE.is_close(norm0[2], 0.0));
    })
}

pub fn run_pointcloud_translate() -> TestResult {
    MINI_TEST!("translate", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;

        // Create cloud with one point
        let pc = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        let offset = Vector::new(10.0, 20.0, 30.0);

        // In-place add
        let mut pc_iadd = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc_iadd += offset.clone();

        // In-place subtract
        let mut pc_isub = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc_isub -= offset.clone();

        // Copy add/subtract
        let pc_add = &pc + offset.clone();
        let pc_sub = &pc - offset.clone();

        let pt_iadd = pc_iadd.get_point(0);
        let pt_isub = pc_isub.get_point(0);
        let pt_add = pc_add.get_point(0);
        let pt_sub = pc_sub.get_point(0);
        let pt_orig = pc.get_point(0);

        MINI_CHECK!(TOLERANCE.is_close(pt_iadd[0], 11.0) && TOLERANCE.is_close(pt_iadd[1], 22.0) && TOLERANCE.is_close(pt_iadd[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_isub[0], -9.0) && TOLERANCE.is_close(pt_isub[1], -18.0) && TOLERANCE.is_close(pt_isub[2], -27.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_add[0], 11.0) && TOLERANCE.is_close(pt_add[1], 22.0) && TOLERANCE.is_close(pt_add[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_sub[0], -9.0) && TOLERANCE.is_close(pt_sub[1], -18.0) && TOLERANCE.is_close(pt_sub[2], -27.0));
        MINI_CHECK!(TOLERANCE.is_close(pt_orig[0], 1.0) && TOLERANCE.is_close(pt_orig[1], 2.0) && TOLERANCE.is_close(pt_orig[2], 3.0));
    })
}

pub fn run_pointcloud_transform() -> TestResult {
    MINI_TEST!("transform", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Xform;

        // Transform - in-place transformation
        let mut pc = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc.xform = Xform::translation(10.0, 20.0, 30.0);
        pc.transform();

        // Transformed - returns new cloud
        let mut pc2 = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc2.xform = Xform::translation(10.0, 20.0, 30.0);
        let pc3 = pc2.transformed();

        let pt = pc.get_point(0);
        let pt3 = pc3.get_point(0);
        let pt2 = pc2.get_point(0);

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 11.0) && TOLERANCE.is_close(pt[1], 22.0) && TOLERANCE.is_close(pt[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pt3[0], 11.0) && TOLERANCE.is_close(pt3[1], 22.0) && TOLERANCE.is_close(pt3[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pt2[0], 1.0) && TOLERANCE.is_close(pt2[1], 2.0) && TOLERANCE.is_close(pt2[2], 3.0));
    })
}

pub fn run_pointcloud_json_roundtrip() -> TestResult {
    MINI_TEST!("json_roundtrip", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        let mut pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![Color::new(255, 0, 0, 255), Color::new(0, 255, 0, 255)],
        );
        pc.name = "test_pointcloud".to_string();

        let fname = "test_pointcloud.json";
        pc.json_dump(fname).unwrap();
        let loaded = PointCloud::json_load(fname).unwrap();

        let pt0 = loaded.get_point(0);
        let col0 = loaded.get_color(0);
        let norm0 = loaded.get_normal(0);

        MINI_CHECK!(loaded.name == "test_pointcloud");
        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(pt0[0], 1.0) && TOLERANCE.is_close(pt0[1], 2.0) && TOLERANCE.is_close(pt0[2], 3.0));
        MINI_CHECK!(col0.r == 255 && col0.g == 0 && col0.b == 0);
        MINI_CHECK!(TOLERANCE.is_close(norm0[2], 1.0));
    })
}

#[cfg(feature = "protobuf")]
pub fn run_pointcloud_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("protobuf_roundtrip", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        let mut pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![Color::new(255, 0, 0, 255), Color::new(0, 255, 0, 255)],
        );
        pc.name = "test_pointcloud".to_string();

        let fname = "test_pointcloud.bin";
        pc.protobuf_dump(fname);
        let loaded = PointCloud::protobuf_load(fname);

        let pt0 = loaded.get_point(0);
        let col0 = loaded.get_color(0);
        let norm0 = loaded.get_normal(0);

        MINI_CHECK!(loaded.name == "test_pointcloud");
        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(pt0[0], 1.0) && TOLERANCE.is_close(pt0[1], 2.0) && TOLERANCE.is_close(pt0[2], 3.0));
        MINI_CHECK!(col0.r == 255 && col0.g == 0 && col0.b == 0);
        MINI_CHECK!(TOLERANCE.is_close(norm0[2], 1.0));
    })
}

REGISTER_MINI_TEST!("PointCloud", "constructor", crate::pointcloud_test::run_pointcloud_constructor);
REGISTER_MINI_TEST!("PointCloud", "from_coords", crate::pointcloud_test::run_pointcloud_from_coords);
REGISTER_MINI_TEST!("PointCloud", "add_and_set", crate::pointcloud_test::run_pointcloud_add_and_set);
REGISTER_MINI_TEST!("PointCloud", "translate", crate::pointcloud_test::run_pointcloud_translate);
REGISTER_MINI_TEST!("PointCloud", "transform", crate::pointcloud_test::run_pointcloud_transform);
REGISTER_MINI_TEST!("PointCloud", "json_roundtrip", crate::pointcloud_test::run_pointcloud_json_roundtrip);
#[cfg(feature = "protobuf")]
REGISTER_MINI_TEST!("PointCloud", "protobuf_roundtrip", crate::pointcloud_test::run_pointcloud_protobuf_roundtrip);
