use crate::{Point, PointCloud};
use std::io;

pub fn write_xyz_to_string(cloud: &PointCloud) -> String {
    let mut s = String::new();
    for p in cloud.get_points().iter() {
        s.push_str(&format!("{} {} {}\n", p[0], p[1], p[2]));
    }
    s
}

pub fn write_xyz(cloud: &PointCloud, filepath: &str) -> io::Result<()> {
    std::fs::write(filepath, write_xyz_to_string(cloud))
}

pub fn read_xyz_from_str(content: &str) -> PointCloud {
    let mut cloud = PointCloud::default();
    for line in content.lines() {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() < 3 {
            continue;
        }
        let x = parts[0].parse::<f64>();
        let y = parts[1].parse::<f64>();
        let z = parts[2].parse::<f64>();
        if x.is_err() || y.is_err() || z.is_err() {
            continue;
        }
        cloud.add_point(&Point::new(x.unwrap(), y.unwrap(), z.unwrap()));
    }
    cloud
}

pub fn read_xyz(filepath: &str) -> io::Result<PointCloud> {
    let content = std::fs::read_to_string(filepath)?;
    Ok(read_xyz_from_str(&content))
}
