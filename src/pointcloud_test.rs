use crate::mini_test::TestResult;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_pointcloud_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;
        use crate::Point;
        use crate::PointCloud;
        use crate::Vector;

        let pc0 = PointCloud::default();

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

        let pcstr = pc.str();
        let pcrepr = pc.repr();

        let pccopy = pc.duplicate();
        let pcother = PointCloud::default();

        let offset = Vector::new(10.0, 20.0, 30.0);
        let pc3 = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);

        let mut pc_iadd = pc3.duplicate();
        pc_iadd += &offset;

        let mut pc_isub = pc3.duplicate();
        pc_isub -= &offset;

        let pc_add = &pc3 + &offset;
        let pc_sub = &pc3 - &offset;

        MINI_CHECK!(pc0.name == "my_pointcloud");
        MINI_CHECK!(!pc0.guid().is_empty());
        MINI_CHECK!(pc0.is_empty());
        MINI_CHECK!(pc.len() == 3);
        MINI_CHECK!(pcstr == "3 points");
        MINI_CHECK!(pcrepr == "PointCloud(my_pointcloud, 3 points, 3 colors, 3 normals)");
        MINI_CHECK!(pccopy == pc && pccopy.guid() != pc.guid());
        MINI_CHECK!(pcother != pc);
        MINI_CHECK!(pc_iadd.get_point(0) == Point::new(11.0, 22.0, 33.0));
        MINI_CHECK!(pc_isub.get_point(0) == Point::new(-9.0, -18.0, -27.0));
        MINI_CHECK!(pc_add.get_point(0) == Point::new(11.0, 22.0, 33.0));
        MINI_CHECK!(pc_sub.get_point(0) == Point::new(-9.0, -18.0, -27.0));
    })
}

pub fn run_pointcloud_from_coords() -> TestResult {
    MINI_TEST!("From Coords", {
        use crate::Color;
        use crate::Point;
        use crate::PointCloud;
        use crate::Vector;

        let coords = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
        let colors = vec![255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255];
        let normals = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
        let pc = PointCloud::from_coords(coords, colors, normals);

        MINI_CHECK!(pc.len() == 3 && pc.color_count() == 3 && pc.normal_count() == 3);
        MINI_CHECK!(pc.get_point(1) == Point::new(1.0, 0.0, 0.0));
        MINI_CHECK!(pc.get_color(1) == Color::new(0.0, 1.0, 0.0, 1.0));
        MINI_CHECK!(pc.get_normal(1) == Vector::new(0.0, 0.0, 1.0));
    })
}

pub fn run_pointcloud_transform() -> TestResult {
    MINI_TEST!("Transform", {
        use crate::Point;
        use crate::PointCloud;
        use crate::Vector;
        use crate::Xform;

        let mut pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0)],
            vec![Vector::new(1.0, 0.0, 0.0)],
            vec![],
        );
        let xform = Xform::translation(10.0, 20.0, 30.0);
        pc.transform(&xform);

        MINI_CHECK!(pc.get_point(0) == Point::new(11.0, 22.0, 33.0));
        MINI_CHECK!(pc.get_normal(0) == Vector::new(1.0, 0.0, 0.0));
    })
}

pub fn run_pointcloud_transformed() -> TestResult {
    MINI_TEST!("Transformed", {
        use crate::Point;
        use crate::PointCloud;
        use crate::Xform;

        let pc = PointCloud::new(vec![Point::new(1.0, 2.0, 3.0)], vec![], vec![]);
        let xform = Xform::translation(10.0, 20.0, 30.0);
        let moved = pc.transformed(&xform);

        MINI_CHECK!(moved.get_point(0) == Point::new(11.0, 22.0, 33.0));
        MINI_CHECK!(pc.get_point(0) == Point::new(1.0, 2.0, 3.0));
    })
}

pub fn run_pointcloud_point_count() -> TestResult {
    MINI_TEST!("Point Count", {
        use crate::Point;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![],
            vec![],
        );

        MINI_CHECK!(pc.point_count() == 3);
    })
}

pub fn run_pointcloud_len() -> TestResult {
    MINI_TEST!("Len", {
        use crate::Point;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)],
            vec![],
            vec![],
        );

        MINI_CHECK!(pc.len() == 2);
    })
}

pub fn run_pointcloud_is_empty() -> TestResult {
    MINI_TEST!("Is Empty", {
        use crate::Point;
        use crate::PointCloud;

        let pc0 = PointCloud::default();
        let pc1 = PointCloud::new(vec![Point::new(0.0, 0.0, 0.0)], vec![], vec![]);

        MINI_CHECK!(pc0.is_empty());
        MINI_CHECK!(!pc1.is_empty());
    })
}

pub fn run_pointcloud_get_point() -> TestResult {
    MINI_TEST!("Get Point", {
        use crate::Point;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![],
            vec![],
        );
        let point = pc.get_point(1);

        MINI_CHECK!(point == Point::new(4.0, 5.0, 6.0));
    })
}

pub fn run_pointcloud_set_point() -> TestResult {
    MINI_TEST!("Set Point", {
        use crate::Point;
        use crate::PointCloud;

        let mut pc = PointCloud::new(vec![Point::new(0.0, 0.0, 0.0)], vec![], vec![]);
        pc.set_point(0, &Point::new(4.0, 5.0, 6.0));

        MINI_CHECK!(pc.get_point(0) == Point::new(4.0, 5.0, 6.0));
    })
}

pub fn run_pointcloud_add_point() -> TestResult {
    MINI_TEST!("Add Point", {
        use crate::Point;
        use crate::PointCloud;

        let mut pc = PointCloud::default();
        pc.add_point(&Point::new(1.0, 2.0, 3.0));

        MINI_CHECK!(pc.len() == 1);
        MINI_CHECK!(pc.get_point(0) == Point::new(1.0, 2.0, 3.0));
    })
}

pub fn run_pointcloud_get_points() -> TestResult {
    MINI_TEST!("Get Points", {
        use crate::Point;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![],
            vec![],
        );
        let points = pc.get_points();

        MINI_CHECK!(points.len() == 2);
        MINI_CHECK!(points[0] == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(points[1] == Point::new(4.0, 5.0, 6.0));
    })
}

pub fn run_pointcloud_coords() -> TestResult {
    MINI_TEST!("Coords", {
        use crate::Point;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![],
            vec![],
        );
        let coords = pc.coords();

        MINI_CHECK!(coords.len() == 6);
        MINI_CHECK!(coords[0] == 1.0 && coords[5] == 6.0);
    })
}

pub fn run_pointcloud_color_count() -> TestResult {
    MINI_TEST!("Color Count", {
        use crate::Color;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![],
            vec![],
            vec![
                Color::new(1.0, 0.0, 0.0, 1.0),
                Color::new(0.0, 1.0, 0.0, 1.0),
            ],
        );

        MINI_CHECK!(pc.color_count() == 2);
    })
}

pub fn run_pointcloud_get_color() -> TestResult {
    MINI_TEST!("Get Color", {
        use crate::Color;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![],
            vec![],
            vec![
                Color::new(1.0, 0.0, 0.0, 1.0),
                Color::new(0.0, 1.0, 0.0, 1.0),
            ],
        );
        let color = pc.get_color(1);

        MINI_CHECK!(color == Color::new(0.0, 1.0, 0.0, 1.0));
    })
}

pub fn run_pointcloud_set_color() -> TestResult {
    MINI_TEST!("Set Color", {
        use crate::Color;
        use crate::PointCloud;

        let mut pc = PointCloud::new(vec![], vec![], vec![Color::new(0.0, 0.0, 0.0, 0.0)]);
        pc.set_color(0, &Color::new(1.0, 0.0, 0.0, 1.0));

        MINI_CHECK!(pc.get_color(0) == Color::new(1.0, 0.0, 0.0, 1.0));
    })
}

pub fn run_pointcloud_add_color() -> TestResult {
    MINI_TEST!("Add Color", {
        use crate::Color;
        use crate::PointCloud;

        let mut pc = PointCloud::default();
        pc.add_color(&Color::new(1.0, 0.0, 1.0, 1.0));

        MINI_CHECK!(pc.color_count() == 1);
        MINI_CHECK!(pc.get_color(0) == Color::new(1.0, 0.0, 1.0, 1.0));
    })
}

pub fn run_pointcloud_get_colors() -> TestResult {
    MINI_TEST!("Get Colors", {
        use crate::Color;
        use crate::PointCloud;

        let pc = PointCloud::new(
            vec![],
            vec![],
            vec![
                Color::new(1.0, 0.0, 0.0, 1.0),
                Color::new(0.0, 1.0, 0.0, 1.0),
            ],
        );
        let colors = pc.get_colors();

        MINI_CHECK!(colors.len() == 2);
        MINI_CHECK!(colors[0] == Color::new(1.0, 0.0, 0.0, 1.0));
        MINI_CHECK!(colors[1] == Color::new(0.0, 1.0, 0.0, 1.0));
    })
}

pub fn run_pointcloud_colors() -> TestResult {
    MINI_TEST!("Colors", {
        use crate::Color;
        use crate::PointCloud;

        let pc = PointCloud::new(vec![], vec![], vec![Color::new(1.0, 0.0, 0.0, 1.0)]);
        let colors = pc.colors();

        MINI_CHECK!(colors.len() == 4);
        MINI_CHECK!(colors[0] == 255 && colors[1] == 0 && colors[2] == 0 && colors[3] == 255);
    })
}

pub fn run_pointcloud_normal_count() -> TestResult {
    MINI_TEST!("Normal Count", {
        use crate::PointCloud;
        use crate::Vector;

        let pc = PointCloud::new(
            vec![],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![],
        );

        MINI_CHECK!(pc.normal_count() == 2);
    })
}

pub fn run_pointcloud_get_normal() -> TestResult {
    MINI_TEST!("Get Normal", {
        use crate::PointCloud;
        use crate::Vector;

        let pc = PointCloud::new(
            vec![],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 0.0, 0.0)],
            vec![],
        );
        let normal = pc.get_normal(1);

        MINI_CHECK!(normal == Vector::new(1.0, 0.0, 0.0));
    })
}

pub fn run_pointcloud_set_normal() -> TestResult {
    MINI_TEST!("Set Normal", {
        use crate::PointCloud;
        use crate::Vector;

        let mut pc = PointCloud::new(vec![], vec![Vector::new(0.0, 0.0, 1.0)], vec![]);
        pc.set_normal(0, &Vector::new(0.0, 1.0, 0.0));

        MINI_CHECK!(pc.get_normal(0) == Vector::new(0.0, 1.0, 0.0));
    })
}

pub fn run_pointcloud_add_normal() -> TestResult {
    MINI_TEST!("Add Normal", {
        use crate::PointCloud;
        use crate::Vector;

        let mut pc = PointCloud::default();
        pc.add_normal(&Vector::new(1.0, 0.0, 0.0));

        MINI_CHECK!(pc.normal_count() == 1);
        MINI_CHECK!(pc.get_normal(0) == Vector::new(1.0, 0.0, 0.0));
    })
}

pub fn run_pointcloud_get_normals() -> TestResult {
    MINI_TEST!("Get Normals", {
        use crate::PointCloud;
        use crate::Vector;

        let pc = PointCloud::new(
            vec![],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 0.0, 0.0)],
            vec![],
        );
        let normals = pc.get_normals();

        MINI_CHECK!(normals.len() == 2);
        MINI_CHECK!(normals[0] == Vector::new(0.0, 0.0, 1.0));
        MINI_CHECK!(normals[1] == Vector::new(1.0, 0.0, 0.0));
    })
}

pub fn run_pointcloud_normals() -> TestResult {
    MINI_TEST!("Normals", {
        use crate::PointCloud;
        use crate::Vector;

        let pc = PointCloud::new(
            vec![],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(1.0, 0.0, 0.0)],
            vec![],
        );
        let normals = pc.normals();

        MINI_CHECK!(normals.len() == 6);
        MINI_CHECK!(normals[2] == 1.0 && normals[3] == 1.0);
    })
}

pub fn run_pointcloud_build_lod() -> TestResult {
    MINI_TEST!("Build Lod", {
        use crate::Point;
        use crate::PointCloud;

        let coords = vec![
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
            1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        ];
        let mut pc = PointCloud::from_coords(coords, vec![], vec![]);
        pc.build_lod(1.0, 2);

        let cube = pc.lod_cube(0);
        let span = pc.lod_range(0);
        let children = pc.lod_children(0);

        MINI_CHECK!(pc.has_lod());
        MINI_CHECK!(pc.lod_node_count() == 8);
        MINI_CHECK!(cube.0 == Point::new(0.5, 0.5, 0.5) && cube.1 == 1.0);
        MINI_CHECK!(pc.lod_spacing(0) == 1.0 && pc.lod_level(1) == 1);
        MINI_CHECK!(span.0 == 0 && span.1 == 1);
        MINI_CHECK!(children[0] == 1 && children[7] == -1);
        MINI_CHECK!(pc.coords().len() == 24);
    })
}

pub fn run_pointcloud_point_ids() -> TestResult {
    MINI_TEST!("Point Ids", {
        use crate::PointCloud;

        let coords = vec![
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
            1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        ];
        let mut pc = PointCloud::from_coords(coords, vec![], vec![]);
        let before = pc.get_point(5);
        pc.build_lod(1.0, 2);

        let index = pc.index_of_id(5);

        MINI_CHECK!(pc.point_ids().len() == 8);
        MINI_CHECK!(index.is_some());
        MINI_CHECK!(pc.point_id(index.unwrap()) == 5);
        MINI_CHECK!(pc.get_point(index.unwrap()) == before);
    })
}

pub fn run_pointcloud_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Color;
        use crate::Point;
        use crate::PointCloud;
        use crate::Vector;

        let mut pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![
                Color::new(1.0, 0.0, 0.0, 1.0),
                Color::new(0.0, 1.0, 0.0, 1.0),
            ],
        );
        pc.name = "test_pointcloud".to_string();

        let guid = pc.guid().to_string();
        let filename = "serialization/test_pointcloud.json";
        pc.file_json_dump(filename).unwrap();

        let loaded = PointCloud::file_json_load(filename).unwrap();
        let parsed = PointCloud::file_json_loads(&pc.file_json_dumps());

        MINI_CHECK!(loaded == pc);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(parsed == pc);
        MINI_CHECK!(parsed.guid() == guid);
    })
}

pub fn run_pointcloud_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Color;
        use crate::Point;
        use crate::PointCloud;
        use crate::Vector;

        let fresh = PointCloud::default();
        let fresh_proto = fresh.to_proto();
        let mut pc = PointCloud::new(
            vec![Point::new(1.0, 2.0, 3.0), Point::new(4.0, 5.0, 6.0)],
            vec![Vector::new(0.0, 0.0, 1.0), Vector::new(0.0, 0.0, 1.0)],
            vec![
                Color::new(1.0, 0.0, 0.0, 1.0),
                Color::new(0.0, 1.0, 0.0, 1.0),
            ],
        );
        pc.name = "test_pointcloud".to_string();

        let guid = pc.guid().to_string();
        let filename = "serialization/test_pointcloud.bin";
        pc.pb_dump(filename);

        let loaded = PointCloud::pb_load(filename);
        let parsed = PointCloud::pb_loads(&pc.pb_dumps()).unwrap();
        let converted = PointCloud::from_proto(pc.to_proto());

        MINI_CHECK!(!fresh.has_guid());
        MINI_CHECK!(fresh_proto.guid.is_empty());
        MINI_CHECK!(loaded == pc);
        MINI_CHECK!(loaded.guid() == guid);
        MINI_CHECK!(parsed == pc);
        MINI_CHECK!(parsed.guid() == guid);
        MINI_CHECK!(converted == pc);
        MINI_CHECK!(converted.guid() == guid);
    })
}

REGISTER_MINI_TEST!("PointCloud", "Constructor", run_pointcloud_constructor);
REGISTER_MINI_TEST!("PointCloud", "From Coords", run_pointcloud_from_coords);
REGISTER_MINI_TEST!("PointCloud", "Transform", run_pointcloud_transform);
REGISTER_MINI_TEST!("PointCloud", "Transformed", run_pointcloud_transformed);
REGISTER_MINI_TEST!("PointCloud", "Point Count", run_pointcloud_point_count);
REGISTER_MINI_TEST!("PointCloud", "Len", run_pointcloud_len);
REGISTER_MINI_TEST!("PointCloud", "Is Empty", run_pointcloud_is_empty);
REGISTER_MINI_TEST!("PointCloud", "Get Point", run_pointcloud_get_point);
REGISTER_MINI_TEST!("PointCloud", "Set Point", run_pointcloud_set_point);
REGISTER_MINI_TEST!("PointCloud", "Add Point", run_pointcloud_add_point);
REGISTER_MINI_TEST!("PointCloud", "Get Points", run_pointcloud_get_points);
REGISTER_MINI_TEST!("PointCloud", "Coords", run_pointcloud_coords);
REGISTER_MINI_TEST!("PointCloud", "Color Count", run_pointcloud_color_count);
REGISTER_MINI_TEST!("PointCloud", "Get Color", run_pointcloud_get_color);
REGISTER_MINI_TEST!("PointCloud", "Set Color", run_pointcloud_set_color);
REGISTER_MINI_TEST!("PointCloud", "Add Color", run_pointcloud_add_color);
REGISTER_MINI_TEST!("PointCloud", "Get Colors", run_pointcloud_get_colors);
REGISTER_MINI_TEST!("PointCloud", "Colors", run_pointcloud_colors);
REGISTER_MINI_TEST!("PointCloud", "Normal Count", run_pointcloud_normal_count);
REGISTER_MINI_TEST!("PointCloud", "Get Normal", run_pointcloud_get_normal);
REGISTER_MINI_TEST!("PointCloud", "Set Normal", run_pointcloud_set_normal);
REGISTER_MINI_TEST!("PointCloud", "Add Normal", run_pointcloud_add_normal);
REGISTER_MINI_TEST!("PointCloud", "Get Normals", run_pointcloud_get_normals);
REGISTER_MINI_TEST!("PointCloud", "Normals", run_pointcloud_normals);
REGISTER_MINI_TEST!("PointCloud", "Build Lod", run_pointcloud_build_lod);
REGISTER_MINI_TEST!("PointCloud", "Point Ids", run_pointcloud_point_ids);
REGISTER_MINI_TEST!(
    "PointCloud",
    "Json Roundtrip",
    run_pointcloud_json_roundtrip
);
REGISTER_MINI_TEST!(
    "PointCloud",
    "Protobuf Roundtrip",
    run_pointcloud_protobuf_roundtrip
);
