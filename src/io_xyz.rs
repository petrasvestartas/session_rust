use crate::Point;
use crate::PointCloud;
use std::io;

// ═══════════════════════════════════════════════════════════════════════════
// Write
// ═══════════════════════════════════════════════════════════════════════════
/// Return the shortest round-trip text of value like C++ fmt "{}": closest digits, exponent outside [1e-4, 1e16).
fn format_number(value: f64) -> String {
    if !value.is_finite() {
        return value.to_string().to_lowercase();
    }

    let shortest = format!("{:e}", value);
    let digits = shortest
        .split('e')
        .next()
        .unwrap()
        .trim_start_matches('-')
        .replace('.', "")
        .len();
    let rounded = format!("{:.*e}", digits - 1, value);
    let scientific = if rounded.parse::<f64>() == Ok(value) {
        rounded
    } else {
        shortest
    };
    let (mantissa, exponent) = scientific.split_once('e').unwrap();
    let exponent: i32 = exponent.parse().unwrap();

    if (-4..16).contains(&exponent) {
        return format!(
            "{:.*}",
            (digits as i32 - 1 - exponent).max(0) as usize,
            value
        );
    }

    let sign = if exponent < 0 { '-' } else { '+' };

    format!("{}e{}{:02}", mantissa, sign, exponent.abs())
}

/// Return the cloud points as "x y z" lines, each number the shortest round-trip text.
pub fn write_xyz_to_string(cloud: &PointCloud) -> String {
    let mut out = String::new();

    for p in cloud.get_points().iter() {
        out.push_str(&format!(
            "{} {} {}\n",
            format_number(p[0]),
            format_number(p[1]),
            format_number(p[2])
        ));
    }

    out
}

/// Write the cloud points as "x y z" lines to filepath; errors if it cannot be opened.
pub fn write_xyz(cloud: &PointCloud, filepath: &str) -> io::Result<()> {
    std::fs::write(filepath, write_xyz_to_string(cloud))
}

// ═══════════════════════════════════════════════════════════════════════════
// Read
// ═══════════════════════════════════════════════════════════════════════════
/// Return the cloud read from "x y z" lines; blank and # lines skipped.
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

/// Return the cloud read from an .xyz file; errors if it cannot be opened.
pub fn read_xyz(filepath: &str) -> io::Result<PointCloud> {
    let content = std::fs::read_to_string(filepath)?;

    Ok(read_xyz_from_str(&content))
}
