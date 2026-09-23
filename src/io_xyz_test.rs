use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_io_xyz_read_bunny() -> TestResult {
    MINI_TEST!("Read Bunny", {
        use crate::read_xyz;
        use std::path::PathBuf;

        let bunny_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .unwrap()
            .join("session_data")
            .join("bunny.xyz");

        if !bunny_path.exists() {
            return Ok(());
        }

        let cloud = read_xyz(bunny_path.to_str().unwrap()).unwrap();
        let points = cloud.get_points();
        let mut has_non_zero = false;

        for p in points.iter() {
            if p[0] != 0.0 || p[1] != 0.0 || p[2] != 0.0 {
                has_non_zero = true;
            }
        }

        MINI_CHECK!(cloud.point_count() == 397);
        MINI_CHECK!(points.len() == 397);
        MINI_CHECK!(has_non_zero);
    })
}

pub fn run_io_xyz_write_read_roundtrip() -> TestResult {
    MINI_TEST!("Write Read Roundtrip", {
        use crate::read_xyz;
        use crate::write_xyz;
        use crate::Point;
        use crate::PointCloud;
        use std::path::PathBuf;

        std::fs::create_dir_all("./serialization").unwrap();

        let mut original = PointCloud::default();
        original.add_point(&Point::new(0.0, 0.0, 0.0));
        original.add_point(&Point::new(1.0, 0.0, 0.0));
        original.add_point(&Point::new(0.0, 1.0, 0.0));
        original.add_point(&Point::new(0.0, 0.0, 1.0));

        let filepath = "./serialization/test_temp_roundtrip.xyz";
        write_xyz(&original, filepath).unwrap();
        let exists = PathBuf::from(filepath).exists();
        let loaded = read_xyz(filepath).unwrap();

        MINI_CHECK!(original.point_count() == 4);
        MINI_CHECK!(exists);
        MINI_CHECK!(loaded.point_count() == original.point_count());

        std::fs::remove_file(filepath).unwrap();
    })
}

pub fn run_io_xyz_string_roundtrip() -> TestResult {
    MINI_TEST!("String Roundtrip", {
        use crate::read_xyz_from_str;
        use crate::write_xyz_to_string;
        use crate::Point;
        use crate::PointCloud;

        let mut original = PointCloud::default();
        original.add_point(&Point::new(0.0, 0.0, 0.0));
        original.add_point(&Point::new(1.0, 0.0, 0.0));
        original.add_point(&Point::new(0.0, 1.0, 0.0));
        original.add_point(&Point::new(0.0, 0.0, 1.0));

        let content = write_xyz_to_string(&original);
        let loaded = read_xyz_from_str(&content);

        MINI_CHECK!(loaded.point_count() == original.point_count());
        MINI_CHECK!(TOLERANCE.is_close(loaded.get_points()[1][0], 1.0));
    })
}

REGISTER_MINI_TEST!(
    "IoXyz",
    "Read Bunny",
    crate::io_xyz_test::run_io_xyz_read_bunny
);
REGISTER_MINI_TEST!(
    "IoXyz",
    "Write Read Roundtrip",
    crate::io_xyz_test::run_io_xyz_write_read_roundtrip
);
REGISTER_MINI_TEST!(
    "IoXyz",
    "String Roundtrip",
    crate::io_xyz_test::run_io_xyz_string_roundtrip
);
