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

        // Add points, colors, normals to empty cloud
        let mut pc2 = PointCloud::default();
        pc2.add_point(&Point::new(1.0, 2.0, 3.0));
        pc2.add_color(&Color::new(128, 64, 32, 255));
        pc2.add_normal(&Vector::new(1.0, 0.0, 0.0));

        // Set point/color/normal at index
        pc2.set_point(0, &Point::new(4.0, 5.0, 6.0));
        pc2.set_color(0, &Color::new(200, 100, 50, 255));
        pc2.set_normal(0, &Vector::new(0.0, 1.0, 0.0));

        // Translate with Vector offset
        let pc3 = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        let offset = Vector::new(10.0, 20.0, 30.0);
        let mut pc_iadd = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc_iadd += offset.clone();
        let mut pc_isub = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc_isub -= offset.clone();
        let pc_add = &pc3 + offset.clone();
        let pc_sub = &pc3 - offset.clone();

        // Create from flat arrays
        let coords = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let colors_arr = vec![255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255];
        let normals_arr = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let pc4 = PointCloud::from_coords(coords, colors_arr, normals_arr);

        MINI_CHECK!(pc.name == "my_pointcloud" && !pc.guid.is_empty() && point_count == 3);
        MINI_CHECK!(color_count == 3 && normal_count == 3 && is_empty == false);
        MINI_CHECK!(pcstr.contains("3 points"));
        MINI_CHECK!(pcrepr.contains("PointCloud(my_pointcloud"));
        MINI_CHECK!(pccopy == pc && pccopy.guid != pc.guid);
        MINI_CHECK!(TOLERANCE.is_close(pt0[0], 0.0) && TOLERANCE.is_close(pt0[1], 0.0) && TOLERANCE.is_close(pt0[2], 0.0));
        MINI_CHECK!(col0.r == 255 && col0.g == 0 && col0.b == 0 && col0.a == 255);
        MINI_CHECK!(TOLERANCE.is_close(norm0[0], 0.0) && TOLERANCE.is_close(norm0[1], 0.0) && TOLERANCE.is_close(norm0[2], 1.0));
        MINI_CHECK!(pc2.len() == 1 && pc2.color_count() == 1 && pc2.normal_count() == 1);
        MINI_CHECK!(TOLERANCE.is_close(pc2.get_point(0)[0], 4.0) && TOLERANCE.is_close(pc2.get_point(0)[1], 5.0) && TOLERANCE.is_close(pc2.get_point(0)[2], 6.0));
        MINI_CHECK!(pc2.get_color(0).r == 200 && pc2.get_color(0).g == 100 && pc2.get_color(0).b == 50 && pc2.get_color(0).a == 255);
        MINI_CHECK!(TOLERANCE.is_close(pc2.get_normal(0)[0], 0.0) && TOLERANCE.is_close(pc2.get_normal(0)[1], 1.0) && TOLERANCE.is_close(pc2.get_normal(0)[2], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pc_iadd.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pc_isub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_isub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_isub.get_point(0)[2], -27.0));
        MINI_CHECK!(TOLERANCE.is_close(pc_add.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_add.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_add.get_point(0)[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pc_sub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_sub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_sub.get_point(0)[2], -27.0));
        MINI_CHECK!(TOLERANCE.is_close(pc3.get_point(0)[0], 1.0) && TOLERANCE.is_close(pc3.get_point(0)[1], 2.0) && TOLERANCE.is_close(pc3.get_point(0)[2], 3.0));
        MINI_CHECK!(pc4.len() == 3 && pc4.color_count() == 3 && pc4.normal_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pc4.get_point(1)[0], 1.0) && TOLERANCE.is_close(pc4.get_point(1)[1], 0.0) && TOLERANCE.is_close(pc4.get_point(1)[2], 0.0));
        MINI_CHECK!(pc4.get_color(1).r == 0 && pc4.get_color(1).g == 255 && pc4.get_color(1).b == 0 && pc4.get_color(1).a == 255);
        MINI_CHECK!(TOLERANCE.is_close(pc4.get_normal(1)[0], 0.0) && TOLERANCE.is_close(pc4.get_normal(1)[1], 0.0) && TOLERANCE.is_close(pc4.get_normal(1)[2], 1.0));
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

        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc.get_point(0)[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pc3.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc3.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc3.get_point(0)[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pc2.get_point(0)[0], 1.0) && TOLERANCE.is_close(pc2.get_point(0)[1], 2.0) && TOLERANCE.is_close(pc2.get_point(0)[2], 3.0));
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

        let fname = "serialization/test_pointcloud.json";
        pc.json_dump(fname).unwrap();
        let loaded = PointCloud::json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == "test_pointcloud");
        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0) && TOLERANCE.is_close(loaded.get_point(0)[1], 2.0) && TOLERANCE.is_close(loaded.get_point(0)[2], 3.0));
        MINI_CHECK!(loaded.get_color(0).r == 255 && loaded.get_color(0).g == 0 && loaded.get_color(0).b == 0);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
    })
}

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

        let fname = "serialization/test_pointcloud.bin";
        pc.protobuf_dump(fname);
        let loaded = PointCloud::protobuf_load(fname);

        MINI_CHECK!(loaded.name == "test_pointcloud");
        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0) && TOLERANCE.is_close(loaded.get_point(0)[1], 2.0) && TOLERANCE.is_close(loaded.get_point(0)[2], 3.0));
        MINI_CHECK!(loaded.get_color(0).r == 255 && loaded.get_color(0).g == 0 && loaded.get_color(0).b == 0);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
    })
}

REGISTER_MINI_TEST!("PointCloud", "constructor", crate::pointcloud_test::run_pointcloud_constructor);
REGISTER_MINI_TEST!("PointCloud", "transform", crate::pointcloud_test::run_pointcloud_transform);
REGISTER_MINI_TEST!("PointCloud", "json_roundtrip", crate::pointcloud_test::run_pointcloud_json_roundtrip);
REGISTER_MINI_TEST!("PointCloud", "protobuf_roundtrip", crate::pointcloud_test::run_pointcloud_protobuf_roundtrip);
