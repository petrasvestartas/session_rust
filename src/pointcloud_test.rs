use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_pointcloud_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        // Default constructor (empty cloud)
        let pc0 = PointCloud::default();

        // Constructor with points, normals, colors
        let p0 = Point::new(0.0, 0.0, 0.0);
        let p1 = Point::new(1.0, 0.0, 0.0);
        let p2 = Point::new(0.0, 1.0, 0.0);
        let n0 = Vector::new(0.0, 0.0, 1.0);
        let n1 = Vector::new(0.0, 0.0, 1.0);
        let n2 = Vector::new(0.0, 0.0, 1.0);
        let c0 = Color::new(1.0, 0.0, 0.0, 1.0);
        let c1 = Color::new(0.0, 1.0, 0.0, 1.0);
        let c2 = Color::new(0.0, 0.0, 1.0, 1.0);
        let pc = PointCloud::new(vec![p0, p1, p2], vec![n0, n1, n2], vec![c0, c1, c2]);

        // Minimal and Full String Representation
        let pcstr = pc.str();
        let pcrepr = pc.repr();

        // Copy (duplicates everything except guid)
        let pccopy = pc.duplicate();
        let pcother = PointCloud::default();

        // Copy operators
        let offset = Vector::new(10.0, 20.0, 30.0);
        let mut pc_iadd = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc_iadd += offset.clone();
        let mut pc_isub = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc_isub -= offset.clone();
        let pc3 = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        let pc_add = &pc3 + offset.clone();
        let pc_sub = &pc3 - offset.clone();

        MINI_CHECK!(pc0.name == "my_pointcloud");
        MINI_CHECK!(!pc0.guid().is_empty());
        MINI_CHECK!(pc0.is_empty());
        MINI_CHECK!(pc.len() == 3);
        MINI_CHECK!(pcstr == "3 points");
        MINI_CHECK!(pcrepr == "PointCloud(my_pointcloud, 3 points, 3 colors, 3 normals)");
        MINI_CHECK!(pccopy == pc && pccopy.guid() != pc.guid());
        MINI_CHECK!(pcother != pc);
        MINI_CHECK!(TOLERANCE.is_close(pc_iadd.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_iadd.get_point(0)[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pc_isub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_isub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_isub.get_point(0)[2], -27.0));
        MINI_CHECK!(TOLERANCE.is_close(pc_add.get_point(0)[0], 11.0) && TOLERANCE.is_close(pc_add.get_point(0)[1], 22.0) && TOLERANCE.is_close(pc_add.get_point(0)[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pc_sub.get_point(0)[0], -9.0) && TOLERANCE.is_close(pc_sub.get_point(0)[1], -18.0) && TOLERANCE.is_close(pc_sub.get_point(0)[2], -27.0));
    })
}

pub fn run_pointcloud_from_coords() -> TestResult {
    MINI_TEST!("From Coords", {
        use crate::PointCloud;

        let coords = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let colors = vec![255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255];
        let normals = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let pc = PointCloud::from_coords(coords, colors, normals);

        MINI_CHECK!(pc.len() == 3 && pc.color_count() == 3 && pc.normal_count() == 3);
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(1)[0], 1.0));
        MINI_CHECK!(pc.get_color(1).g == 1.0);
        MINI_CHECK!(TOLERANCE.is_close(pc.get_normal(1)[2], 1.0));
    })
}

pub fn run_pointcloud_point_count() -> TestResult {
    MINI_TEST!("Point Count", {
        use crate::PointCloud;
        use crate::Point;

        let pc = PointCloud::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0), Point::new(0.0, 1.0, 0.0)], vec![], vec![]);

        MINI_CHECK!(pc.point_count() == 3);
    })
}

pub fn run_pointcloud_len() -> TestResult {
    MINI_TEST!("Len", {
        use crate::PointCloud;
        use crate::Point;

        let pc = PointCloud::new(vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)], vec![], vec![]);

        MINI_CHECK!(pc.len() == 2);
    })
}

pub fn run_pointcloud_is_empty() -> TestResult {
    MINI_TEST!("Is Empty", {
        use crate::PointCloud;
        use crate::Point;

        let pc0 = PointCloud::default();
        let pc1 = PointCloud::new(vec![Point::new(0.0, 0.0, 0.0)], vec![], vec![]);

        MINI_CHECK!(pc0.is_empty());
        MINI_CHECK!(!pc1.is_empty());
    })
}

pub fn run_pointcloud_get_point() -> TestResult {
    MINI_TEST!("Get Point", {
        use crate::PointCloud;
        use crate::Point;

        let pc = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)], vec![], vec![]);
        let pt = pc.get_point(1);

        MINI_CHECK!(TOLERANCE.is_close(pt[0], 4.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(pt[2], 6.0));
    })
}

pub fn run_pointcloud_set_point() -> TestResult {
    MINI_TEST!("Set Point", {
        use crate::PointCloud;
        use crate::Point;

        let mut pc = PointCloud::new(vec![Point::new(0.0, 0.0, 0.0)], vec![], vec![]);
        pc.set_point(0, &Point::new(4.0, 5.0, 6.0));

        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[0], 4.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[1], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[2], 6.0));
    })
}

pub fn run_pointcloud_add_point() -> TestResult {
    MINI_TEST!("Add Point", {
        use crate::PointCloud;
        use crate::Point;

        let mut pc = PointCloud::default();
        pc.add_point(&Point::new(1.0, 2.0, 3.0));

        MINI_CHECK!(pc.len() == 1);
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[2], 3.0));
    })
}

pub fn run_pointcloud_get_points() -> TestResult {
    MINI_TEST!("Get Points", {
        use crate::PointCloud;
        use crate::Point;

        let pc = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)], vec![], vec![]);
        let points = pc.get_points();

        MINI_CHECK!(points.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(points[0][0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(points[1][2], 6.0));
    })
}

pub fn run_pointcloud_color_count() -> TestResult {
    MINI_TEST!("Color Count", {
        use crate::PointCloud;
        use crate::Color;

        let pc = PointCloud::new(vec![], vec![], vec![Color::new(1.0, 0.0, 0.0, 1.0), Color::new(0.0, 1.0, 0.0, 1.0)]);

        MINI_CHECK!(pc.color_count() == 2);
    })
}

pub fn run_pointcloud_get_color() -> TestResult {
    MINI_TEST!("Get Color", {
        use crate::PointCloud;
        use crate::Color;

        let pc = PointCloud::new(vec![], vec![], vec![Color::new(1.0, 0.0, 0.0, 1.0), Color::new(0.0, 1.0, 0.0, 1.0)]);
        let c = pc.get_color(1);

        MINI_CHECK!(c.r == 0.0 && c.g == 1.0 && c.b == 0.0 && c.a == 1.0);
    })
}

pub fn run_pointcloud_set_color() -> TestResult {
    MINI_TEST!("Set Color", {
        use crate::PointCloud;
        use crate::Color;

        let mut pc = PointCloud::new(vec![], vec![], vec![Color::new(0.0, 0.0, 0.0, 0.0)]);
        pc.set_color(0, &Color::new(1.0, 0.0, 0.0, 1.0));

        MINI_CHECK!(TOLERANCE.is_close(pc.get_color(0).r, 1.0) && pc.get_color(0).g == 0.0 && pc.get_color(0).b == 0.0 && pc.get_color(0).a == 1.0);
    })
}

pub fn run_pointcloud_add_color() -> TestResult {
    MINI_TEST!("Add Color", {
        use crate::PointCloud;
        use crate::Color;

        let mut pc = PointCloud::default();
        pc.add_color(&Color::new(1.0, 0.0, 0.0, 1.0));

        MINI_CHECK!(pc.color_count() == 1);
        MINI_CHECK!(TOLERANCE.is_close(pc.get_color(0).r, 1.0) && pc.get_color(0).g == 0.0 && pc.get_color(0).b == 0.0);
    })
}

pub fn run_pointcloud_get_colors() -> TestResult {
    MINI_TEST!("Get Colors", {
        use crate::PointCloud;
        use crate::Color;

        let pc = PointCloud::new(vec![], vec![], vec![Color::new(1.0, 0.0, 0.0, 1.0), Color::new(0.0, 1.0, 0.0, 1.0)]);
        let colors = pc.get_colors();

        MINI_CHECK!(colors.len() == 2);
        MINI_CHECK!(colors[0].r == 1.0);
        MINI_CHECK!(colors[1].g == 1.0);
    })
}

pub fn run_pointcloud_normal_count() -> TestResult {
    MINI_TEST!("Normal Count", {
        use crate::PointCloud;
        use crate::Vector;

        let pc = PointCloud::new(vec![], vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)], vec![]);

        MINI_CHECK!(pc.normal_count() == 2);
    })
}

pub fn run_pointcloud_get_normal() -> TestResult {
    MINI_TEST!("Get Normal", {
        use crate::PointCloud;
        use crate::Vector;

        let pc = PointCloud::new(vec![], vec![Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 0.0, 0.0)], vec![]);
        let n = pc.get_normal(1);

        MINI_CHECK!(TOLERANCE.is_close(n[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(n[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(n[2], 0.0));
    })
}

pub fn run_pointcloud_set_normal() -> TestResult {
    MINI_TEST!("Set Normal", {
        use crate::PointCloud;
        use crate::Vector;

        let mut pc = PointCloud::new(vec![], vec![Vector::new(0.0, 0.0, 1.0)], vec![]);
        pc.set_normal(0, &Vector::new(0.0, 1.0, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(pc.get_normal(0)[0], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_normal(0)[1], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_normal(0)[2], 0.0));
    })
}

pub fn run_pointcloud_add_normal() -> TestResult {
    MINI_TEST!("Add Normal", {
        use crate::PointCloud;
        use crate::Vector;

        let mut pc = PointCloud::default();
        pc.add_normal(&Vector::new(1.0, 0.0, 0.0));

        MINI_CHECK!(pc.normal_count() == 1);
        MINI_CHECK!(TOLERANCE.is_close(pc.get_normal(0)[0], 1.0));
    })
}

pub fn run_pointcloud_get_normals() -> TestResult {
    MINI_TEST!("Get Normals", {
        use crate::PointCloud;
        use crate::Vector;

        let pc = PointCloud::new(vec![], vec![Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 0.0, 0.0)], vec![]);
        let normals = pc.get_normals();

        MINI_CHECK!(normals.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(normals[0][2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(normals[1][0], 1.0));
    })
}

pub fn run_pointcloud_transform() -> TestResult {
    MINI_TEST!("Transform", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Xform;

        let mut pc = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc.xform = Xform::translation(10.0, 20.0, 30.0);
        pc.transform();

        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[0], 11.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[1], 22.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[2], 33.0));
    })
}

pub fn run_pointcloud_transformed() -> TestResult {
    MINI_TEST!("Transformed", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Xform;

        let mut pc = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        pc.xform = Xform::translation(10.0, 20.0, 30.0);
        let pc2 = pc.transformed();

        MINI_CHECK!(TOLERANCE.is_close(pc2.get_point(0)[0], 11.0));
        MINI_CHECK!(TOLERANCE.is_close(pc2.get_point(0)[1], 22.0));
        MINI_CHECK!(TOLERANCE.is_close(pc2.get_point(0)[2], 33.0));
        MINI_CHECK!(TOLERANCE.is_close(pc.get_point(0)[0], 1.0));
    })
}

pub fn run_pointcloud_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        let mut pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![Color::new(1.0, 0.0, 0.0, 1.0), Color::new(0.0, 1.0, 0.0, 1.0)],
        );
        pc.name = "test_pointcloud".to_string();

        //   jsondump()      │ String       │ to JSON string (internal use)
        //   jsonload(s)     │ String       │ from JSON string (internal use)
        //   file_json_dumps()    │ String       │ to JSON string
        //   file_json_loads(s)   │ String       │ from JSON string
        //   file_json_dump(path) │ file         │ write to file
        //   file_json_load(path) │ file         │ read from file

        let fname = "serialization/test_pointcloud.json";
        pc.file_json_dump(fname).unwrap();
        let loaded = PointCloud::file_json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == "test_pointcloud");
        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
        MINI_CHECK!(loaded.get_color(0).r == 1.0);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
    })
}

pub fn run_pointcloud_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::PointCloud;
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        let mut pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![Color::new(1.0, 0.0, 0.0, 1.0), Color::new(0.0, 1.0, 0.0, 1.0)],
        );
        pc.name = "test_pointcloud".to_string();

        let fname = "serialization/test_pointcloud.bin";
        pc.pb_dump(fname);
        let loaded = PointCloud::pb_load(fname);

        MINI_CHECK!(loaded.name == "test_pointcloud");
        MINI_CHECK!(loaded.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_point(0)[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_color(0).r, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_normal(0)[2], 1.0));
    })
}

REGISTER_MINI_TEST!("PointCloud", "Constructor", crate::pointcloud_test::run_pointcloud_constructor);
REGISTER_MINI_TEST!("PointCloud", "From Coords", crate::pointcloud_test::run_pointcloud_from_coords);
REGISTER_MINI_TEST!("PointCloud", "Point Count", crate::pointcloud_test::run_pointcloud_point_count);
REGISTER_MINI_TEST!("PointCloud", "Len", crate::pointcloud_test::run_pointcloud_len);
REGISTER_MINI_TEST!("PointCloud", "Is Empty", crate::pointcloud_test::run_pointcloud_is_empty);
REGISTER_MINI_TEST!("PointCloud", "Get Point", crate::pointcloud_test::run_pointcloud_get_point);
REGISTER_MINI_TEST!("PointCloud", "Set Point", crate::pointcloud_test::run_pointcloud_set_point);
REGISTER_MINI_TEST!("PointCloud", "Add Point", crate::pointcloud_test::run_pointcloud_add_point);
REGISTER_MINI_TEST!("PointCloud", "Get Points", crate::pointcloud_test::run_pointcloud_get_points);
REGISTER_MINI_TEST!("PointCloud", "Color Count", crate::pointcloud_test::run_pointcloud_color_count);
REGISTER_MINI_TEST!("PointCloud", "Get Color", crate::pointcloud_test::run_pointcloud_get_color);
REGISTER_MINI_TEST!("PointCloud", "Set Color", crate::pointcloud_test::run_pointcloud_set_color);
REGISTER_MINI_TEST!("PointCloud", "Add Color", crate::pointcloud_test::run_pointcloud_add_color);
REGISTER_MINI_TEST!("PointCloud", "Get Colors", crate::pointcloud_test::run_pointcloud_get_colors);
REGISTER_MINI_TEST!("PointCloud", "Normal Count", crate::pointcloud_test::run_pointcloud_normal_count);
REGISTER_MINI_TEST!("PointCloud", "Get Normal", crate::pointcloud_test::run_pointcloud_get_normal);
REGISTER_MINI_TEST!("PointCloud", "Set Normal", crate::pointcloud_test::run_pointcloud_set_normal);
REGISTER_MINI_TEST!("PointCloud", "Add Normal", crate::pointcloud_test::run_pointcloud_add_normal);
REGISTER_MINI_TEST!("PointCloud", "Get Normals", crate::pointcloud_test::run_pointcloud_get_normals);
REGISTER_MINI_TEST!("PointCloud", "Transform", crate::pointcloud_test::run_pointcloud_transform);
REGISTER_MINI_TEST!("PointCloud", "Transformed", crate::pointcloud_test::run_pointcloud_transformed);
REGISTER_MINI_TEST!("PointCloud", "Json Roundtrip", crate::pointcloud_test::run_pointcloud_json_roundtrip);
REGISTER_MINI_TEST!("PointCloud", "Protobuf Roundtrip", crate::pointcloud_test::run_pointcloud_protobuf_roundtrip);
