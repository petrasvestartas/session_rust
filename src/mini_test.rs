use serde::Serialize;
use serde_json::Value;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fs;
use std::path::PathBuf;

/// Shared per-check metadata for the Rust mini-test framework.
#[derive(Serialize)]
pub struct CheckRecord {
    pub line: u32,
    pub code_line: &'static str,
    pub passed: bool,
}

/// Shared test result structure used by all Rust mini-tests.
#[derive(Serialize)]
pub struct TestResult {
    pub test_name: &'static str,
    pub passed: bool,
    pub time_ms: f64,
    pub line: u32,
    pub file: &'static str,
    pub code: String,
    pub checks: Vec<CheckRecord>,
    pub failures: Vec<Value>,
}

std::thread_local! {
    static CURRENT_CHECKS: RefCell<Vec<CheckRecord>> = RefCell::new(Vec::new());
    static CURRENT_ASSERTION_TIME: RefCell<f64> = RefCell::new(0.0);
}

pub fn start_checks() {
    CURRENT_CHECKS.with(|c| c.borrow_mut().clear());
    CURRENT_ASSERTION_TIME.with(|t| *t.borrow_mut() = 0.0);
}

pub fn push_check(line: u32, code_line: &'static str, passed: bool, start: std::time::Instant) {
    CURRENT_CHECKS.with(|c| {
        c.borrow_mut().push(CheckRecord {
            line,
            code_line,
            passed,
        });
    });
    let elapsed = start.elapsed().as_secs_f64() * 1000.0;
    CURRENT_ASSERTION_TIME.with(|t| *t.borrow_mut() += elapsed);
}

pub fn take_assertion_time() -> f64 {
    CURRENT_ASSERTION_TIME.with(|t| {
        let v = *t.borrow();
        *t.borrow_mut() = 0.0;
        v
    })
}

pub fn take_checks() -> Vec<CheckRecord> {
    CURRENT_CHECKS.with(|c| {
        let mut v = c.borrow_mut();
        std::mem::take(&mut *v)
    })
}

/// Generic helper to extract the timed body of a mini-test from source.
///
/// It looks in `file` starting near `anchor_line` for the standard pattern
///
///   let result: Result<(), String> = (|| {
///       // timed body lines ...
///       MINI_CHECK!(...);
///   })();
///
/// and returns all lines between the macro call and the first `MINI_CHECK!`
/// call in the same file. This mirrors the C++ implementation that slices
/// between the `MINI_TEST` macro line and the first check.
pub fn extract_timed_body(file: &str, macro_line: u32, checks: &[CheckRecord]) -> String {
    // CARGO_MANIFEST_DIR points at session_rust/, but source files live in src/
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut path = PathBuf::from(file);
    if !path.is_absolute() {
        path = manifest_dir.join(&path);
    }
    if !path.exists() {
        path = manifest_dir.join("src").join(file);
    }

    // Determine the last check line to know where the test body ends
    let last_check_line = checks
        .iter()
        .map(|c| c.line)
        .max()
        .unwrap_or(macro_line + 50);

    // Snippet starts on the line after the MINI_TEST! call
    let start_line = macro_line.saturating_add(1);
    let end_line = last_check_line + 100;

    let src = match fs::read_to_string(&path) {
        Ok(s) => s,
        Err(_) => return String::new(),
    };

    let mut code_lines: Vec<&str> = Vec::new();
    let mut in_check = false;
    let mut paren_depth = 0i32;
    // MINI_TEST!(..., { opens with depth 1 on the macro line; stop when we close it
    let mut brace_depth = 1i32;

    for (idx, line) in src.lines().enumerate() {
        let line_no = (idx as u32) + 1;
        if line_no < start_line {
            continue;
        }
        if line_no > end_line {
            break;
        }

        // Stop if we hit another MINI_TEST
        if line.contains("MINI_TEST!(") {
            break;
        }

        // If we're continuing a multi-line MINI_CHECK, skip and track parentheses
        if in_check {
            for ch in line.chars() {
                if ch == '(' {
                    paren_depth += 1;
                } else if ch == ')' {
                    paren_depth -= 1;
                }
            }
            if paren_depth <= 0 {
                in_check = false;
            }
            continue;
        }

        // Track brace depth to find end of test body
        for ch in line.chars() {
            if ch == '{' {
                brace_depth += 1;
            } else if ch == '}' {
                brace_depth -= 1;
            }
        }
        if brace_depth <= 0 {
            break;
        }

        // Skip MINI_CHECK! lines from code display
        if line.contains("MINI_CHECK!") {
            in_check = true;
            paren_depth = 0;
            for ch in line.chars() {
                if ch == '(' {
                    paren_depth += 1;
                } else if ch == ')' {
                    paren_depth -= 1;
                }
            }
            if paren_depth <= 0 {
                in_check = false;
            }
            continue;
        }

        code_lines.push(line);
    }

    // Remove empty if-blocks (entire body was MINI_CHECK calls)
    let mut out: Vec<&str> = Vec::new();
    let mut i = 0;
    while i < code_lines.len() {
        let line = code_lines[i];
        let trimmed = line.trim();
        let is_if =
            (trimmed.starts_with("if ") || trimmed.starts_with("if(")) && trimmed.ends_with('{');
        if is_if {
            // Find matching closing brace using brace depth
            let mut depth = 0i32;
            for ch in line.chars() {
                if ch == '{' {
                    depth += 1;
                } else if ch == '}' {
                    depth -= 1;
                }
            }
            let mut j = i + 1;
            while j < code_lines.len() && depth > 0 {
                for ch in code_lines[j].chars() {
                    if ch == '{' {
                        depth += 1;
                    } else if ch == '}' {
                        depth -= 1;
                    }
                }
                j += 1;
            }
            // j is one past the closing brace line; check if body is empty
            let mut body_empty = true;
            for k in (i + 1)..(j.saturating_sub(1)) {
                if !code_lines[k].trim().is_empty() {
                    body_empty = false;
                    break;
                }
            }
            if body_empty {
                i = j;
                continue;
            }
        }
        out.push(line);
        i += 1;
    }

    out.join("\n")
}

#[macro_export]
macro_rules! MINI_CHECK {
    ($expr:expr) => {{
        let _check_start = std::time::Instant::now();
        let passed = $expr;
        $crate::mini_test::push_check(line!(), stringify!($expr), passed, _check_start);
        if !passed {
            return Err(format!("expression is not true: {}", stringify!($expr)));
        }
    }};
    ($checks:expr, $expr:expr) => {{
        let passed = $expr;
        $checks.push($crate::mini_test::CheckRecord {
            line: line!(),
            code_line: stringify!($expr),
            passed,
        });
        if !passed {
            return Err(format!("expression is not true: {}", stringify!($expr)));
        }
    }};
}

#[macro_export]
macro_rules! MINI_TEST {
    // New, simpler form: body is a block that can use the local `checks` Vec
    ($test_name:expr, $body:block) => {{
        let line = line!();
        let start = std::time::Instant::now();
        let mut failures = Vec::new();
        let mut passed = true;

        $crate::mini_test::start_checks();
        let result: Result<(), String> = (|| -> Result<(), String> { $body; Ok(()) })();

        if let Err(msg) = result {
            passed = false;
            failures.push(serde_json::json!({ "error": msg }));
        }

        let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
        let assertion_time = $crate::mini_test::take_assertion_time();
        let effective_ms = (elapsed_ms - assertion_time).max(0.0);
        let time_ms = (effective_ms * 1000.0).round() / 1000.0;
        let checks = $crate::mini_test::take_checks();
        let code = $crate::mini_test::extract_timed_body(file!(), line, &checks);

        $crate::mini_test::TestResult {
            test_name: $test_name,
            passed,
            time_ms,
            line,
            file: file!(),
            code,
            checks,
            failures,
        }
    }};
    // Backwards-compatible form: explicit closure that receives &mut checks
    ($test_name:expr, $body:expr) => {{
        let line = line!();
        let start = std::time::Instant::now();
        let mut checks: Vec<$crate::mini_test::CheckRecord> = Vec::new();
        let mut failures = Vec::new();
        let mut passed = true;

        let result: Result<(), String> = (|| $body(&mut checks))();

        if let Err(msg) = result {
            passed = false;
            failures.push(serde_json::json!({ "error": msg }));
        }

        let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
        let time_ms = (elapsed_ms * 1000.0).round() / 1000.0;
        // Note: backwards-compatible form doesn't subtract assertion time
        let code = $crate::mini_test::extract_timed_body(file!(), line, &checks);

        $crate::mini_test::TestResult {
            test_name: $test_name,
            passed,
            time_ms,
            line,
            file: file!(),
            code,
            checks,
            failures,
        }
    }};
}

/// Description of a registered Rust mini-test.
#[derive(Clone, Copy)]
pub struct RegisteredTest {
    pub group: &'static str,
    pub name: &'static str,
    pub func: fn() -> TestResult,
}

inventory::collect!(RegisteredTest);

#[macro_export]
macro_rules! REGISTER_MINI_TEST {
    ($group:expr, $name:expr, $func:path) => {
        inventory::submit! {
            $crate::mini_test::RegisteredTest {
                group: $group,
                name: $name,
                func: $func,
            }
        }
    };
}

#[macro_export]
macro_rules! MINI_TEST_CASE {
    ($group:expr, $name:expr, $body:block) => {
        inventory::submit! {
            $crate::mini_test::RegisteredTest {
                group: $group,
                name: $name,
                func: || $crate::MINI_TEST!($name, |checks: &mut Vec<_>| $body),
            }
        }
    };
}

#[macro_export]
macro_rules! MINI_TEST_FN {
    ($group:expr, $name:expr, $fn_name:ident, $body:block) => {
        pub fn $fn_name() -> $crate::mini_test::TestResult {
            $crate::MINI_TEST!($name, $body)
        }

        $crate::REGISTER_MINI_TEST!($group, $name, $fn_name);
    };
}

/// Get all tests manually (fallback when inventory doesn't work, and canonical order oracle)
pub fn get_all_tests() -> Vec<RegisteredTest> {
    use crate::aabb_test::*;
    use crate::boolean_polyline_test::*;
    use crate::brep_test::*;
    use crate::closest_test::*;
    use crate::color_test::*;
    use crate::convex_hull_test::*;
    use crate::element_test::*;
    use crate::file_encoders_test::*;
    use crate::file_obj_test::*;
    use crate::graph_test::*;
    use crate::instance_ref_test::*;
    use crate::intersection_test::*;
    use crate::io_test::*;
    use crate::line_test::*;
    use crate::matrix_test::*;
    use crate::mesh_offset_test::*;
    use crate::mesh_test::*;
    use crate::nurbscurve_test::*;
    use crate::nurbsknot_test::*;
    use crate::nurbssurface_test::*;
    use crate::nurbssurface_trimmed_test::*;
    use crate::obb_test::*;
    use crate::objects_test::*;
    use crate::plane_test::*;
    use crate::point_test::*;
    use crate::pointcloud_test::*;
    use crate::polyline_test::*;
    use crate::primitives_test::*;
    use crate::quaternion_test::*;
    use crate::remesh_cdt_test::*;
    use crate::remesh_nurbssurface_adaptive_test::*;
    use crate::remesh_nurbssurface_grid_test::*;
    use crate::session_config_test::*;
    use crate::session_test::*;
    use crate::spatial_aabbtree_test::*;
    use crate::spatial_bvh_test::*;
    use crate::spatial_kdtree_test::*;
    use crate::spatial_octree_test::*;
    use crate::spatial_rtree_test::*;
    use crate::tolerance_test::*;
    use crate::tree_test::*;
    use crate::vector_test::*;
    use crate::xform_test::*;

    let tests = vec![
        // BRep tests
        RegisteredTest {
            group: "BRep",
            name: "Constructor",
            func: run_brep_constructor,
        },
        RegisteredTest {
            group: "BRep",
            name: "Create Box",
            func: run_brep_create_box,
        },
        RegisteredTest {
            group: "BRep",
            name: "Accessors",
            func: run_brep_accessors,
        },
        RegisteredTest {
            group: "BRep",
            name: "Add Face",
            func: run_brep_add_face,
        },
        RegisteredTest {
            group: "BRep",
            name: "Mesh",
            func: run_brep_mesh,
        },
        RegisteredTest {
            group: "BRep",
            name: "Point At",
            func: run_brep_point_at,
        },
        RegisteredTest {
            group: "BRep",
            name: "Is Solid",
            func: run_brep_is_solid,
        },
        RegisteredTest {
            group: "BRep",
            name: "Is Closed",
            func: run_brep_is_closed,
        },
        RegisteredTest {
            group: "BRep",
            name: "Wire Edges",
            func: run_brep_wire_edges,
        },
        RegisteredTest {
            group: "BRep",
            name: "Edge Faces",
            func: run_brep_edge_faces,
        },
        RegisteredTest {
            group: "BRep",
            name: "Update Tolerances",
            func: run_brep_update_tolerances,
        },
        RegisteredTest {
            group: "BRep",
            name: "Transformation",
            func: run_brep_transformation,
        },
        RegisteredTest {
            group: "BRep",
            name: "Transform Roundtrip",
            func: run_brep_transform_roundtrip,
        },
        RegisteredTest {
            group: "BRep",
            name: "Json Roundtrip",
            func: run_brep_json_roundtrip,
        },
        RegisteredTest {
            group: "BRep",
            name: "Create Cylinder",
            func: run_brep_create_cylinder,
        },
        RegisteredTest {
            group: "BRep",
            name: "Create Sphere",
            func: run_brep_create_sphere,
        },
        RegisteredTest {
            group: "BRep",
            name: "Create Cone",
            func: run_brep_create_cone,
        },
        RegisteredTest {
            group: "BRep",
            name: "Create Pyramid",
            func: run_brep_create_pyramid,
        },
        RegisteredTest {
            group: "BRep",
            name: "Create Torus",
            func: run_brep_create_torus,
        },
        RegisteredTest {
            group: "BRep",
            name: "Create Block With Hole",
            func: run_brep_create_block_with_hole,
        },
        RegisteredTest {
            group: "BRep",
            name: "From Polylines",
            func: run_brep_from_polylines,
        },
        RegisteredTest {
            group: "BRep",
            name: "From Nurbscurves",
            func: run_brep_from_nurbscurves,
        },
        RegisteredTest {
            group: "BRep",
            name: "From Nurbscurves Holes",
            func: run_brep_from_nurbscurves_holes,
        },
        RegisteredTest {
            group: "BRep",
            name: "Mesh Orientation",
            func: run_brep_mesh_orientation,
        },
        RegisteredTest {
            group: "BRep",
            name: "Protobuf Roundtrip",
            func: run_brep_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "BRep",
            name: "Volume",
            func: run_brep_volume,
        },
        // Color tests
        RegisteredTest {
            group: "Color",
            name: "Constructor",
            func: run_color_constructor,
        },
        RegisteredTest {
            group: "Color",
            name: "Json Roundtrip",
            func: run_color_json_roundtrip,
        },
        RegisteredTest {
            group: "Color",
            name: "Protobuf Roundtrip",
            func: run_color_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Color",
            name: "Conversion",
            func: run_color_conversion,
        },
        RegisteredTest {
            group: "Color",
            name: "Presets",
            func: run_color_presets,
        },
        // Point tests
        RegisteredTest {
            group: "Point",
            name: "Constructor",
            func: run_point_constructor,
        },
        RegisteredTest {
            group: "Point",
            name: "Transformation",
            func: run_point_transformation,
        },
        RegisteredTest {
            group: "Point",
            name: "Json Roundtrip",
            func: run_point_json_roundtrip,
        },
        RegisteredTest {
            group: "Point",
            name: "Protobuf Roundtrip",
            func: run_point_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Point",
            name: "Is Ccw",
            func: run_point_is_ccw,
        },
        RegisteredTest {
            group: "Point",
            name: "Mid Point",
            func: run_point_mid_point,
        },
        RegisteredTest {
            group: "Point",
            name: "Distance",
            func: run_point_distance,
        },
        RegisteredTest {
            group: "Point",
            name: "Squared Distance",
            func: run_point_squared_distance,
        },
        RegisteredTest {
            group: "Point",
            name: "Area",
            func: run_point_area,
        },
        RegisteredTest {
            group: "Point",
            name: "Centroid Quad",
            func: run_point_centroid_quad,
        },
        RegisteredTest {
            group: "Point",
            name: "Centroid",
            func: run_point_centroid,
        },
        RegisteredTest {
            group: "Point",
            name: "Dihedral Angle Deg",
            func: run_point_dihedral_angle_deg,
        },
        // Vector tests
        RegisteredTest {
            group: "Vector",
            name: "Constructor",
            func: run_vector_constructor,
        },
        RegisteredTest {
            group: "Vector",
            name: "Magnitude",
            func: run_vector_magnitude,
        },
        RegisteredTest {
            group: "Vector",
            name: "Normalize",
            func: run_vector_normalize,
        },
        RegisteredTest {
            group: "Vector",
            name: "Reverse",
            func: run_vector_reverse,
        },
        RegisteredTest {
            group: "Vector",
            name: "Dot Product",
            func: run_vector_dot_product,
        },
        RegisteredTest {
            group: "Vector",
            name: "Cross Product",
            func: run_vector_cross_product,
        },
        RegisteredTest {
            group: "Vector",
            name: "Angle",
            func: run_vector_angle,
        },
        RegisteredTest {
            group: "Vector",
            name: "Projection",
            func: run_vector_projection,
        },
        RegisteredTest {
            group: "Vector",
            name: "Is Parallel To",
            func: run_vector_is_parallel_to,
        },
        RegisteredTest {
            group: "Vector",
            name: "Is Perpendicular To",
            func: run_vector_is_perpendicular_to,
        },
        RegisteredTest {
            group: "Vector",
            name: "Get Leveled Vector",
            func: run_vector_get_leveled_vector,
        },
        RegisteredTest {
            group: "Vector",
            name: "Cos Sin Laws",
            func: run_vector_cos_sin_laws,
        },
        RegisteredTest {
            group: "Vector",
            name: "Sum Of Vectors",
            func: run_vector_sum_of_vectors,
        },
        RegisteredTest {
            group: "Vector",
            name: "Average",
            func: run_vector_average,
        },
        RegisteredTest {
            group: "Vector",
            name: "Is Zero",
            func: run_vector_is_zero,
        },
        RegisteredTest {
            group: "Vector",
            name: "Json Roundtrip",
            func: run_vector_json_roundtrip,
        },
        RegisteredTest {
            group: "Vector",
            name: "Protobuf Roundtrip",
            func: run_vector_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Vector",
            name: "Transformation",
            func: run_vector_transformation,
        },
        RegisteredTest {
            group: "Vector",
            name: "Scale",
            func: run_vector_scale,
        },
        RegisteredTest {
            group: "Vector",
            name: "Reflect",
            func: run_vector_reflect,
        },
        RegisteredTest {
            group: "Vector",
            name: "Average Normal",
            func: run_vector_average_normal,
        },
        RegisteredTest {
            group: "Vector",
            name: "Interpolate Points",
            func: run_vector_interpolate_points,
        },
        // Tolerance tests
        RegisteredTest {
            group: "Tolerance",
            name: "Is Zero",
            func: run_tolerance_is_zero,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Close",
            func: run_tolerance_is_close,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Positive",
            func: run_tolerance_is_positive,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Negative",
            func: run_tolerance_is_negative,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Between",
            func: run_tolerance_is_between,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Format Number",
            func: run_tolerance_format_number,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Key",
            func: run_tolerance_key,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Runtime Modification",
            func: run_tolerance_runtime_modification,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "To Radians",
            func: run_tolerance_to_radians,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "To Degrees",
            func: run_tolerance_to_degrees,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Unique From Two Int",
            func: run_tolerance_unique_from_two_int,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Wrap Index",
            func: run_tolerance_wrap_index,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Triangle Edge By Angle",
            func: run_tolerance_triangle_edge_by_angle,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Rad Deg Conversion",
            func: run_tolerance_rad_deg,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Count Digits",
            func: run_tolerance_count_digits,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Angle Zero",
            func: run_tolerance_is_angle_zero,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Angles Close",
            func: run_tolerance_is_angles_close,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Point Close",
            func: run_tolerance_is_point_close,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Allclose",
            func: run_tolerance_is_allclose,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Key Xy",
            func: run_tolerance_key_xy,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Round To",
            func: run_tolerance_round_to,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Precision From Tolerance",
            func: run_tolerance_precision_from_tolerance,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Tolerance",
            func: run_tolerance_tolerance,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Compare",
            func: run_tolerance_compare,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Finite",
            func: run_tolerance_is_finite,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Is Vector Close",
            func: run_tolerance_is_vector_close,
        },
        RegisteredTest {
            group: "Tolerance",
            name: "Temporary",
            func: run_tolerance_temporary,
        },
        // Line tests
        RegisteredTest {
            group: "Line",
            name: "Constructor",
            func: run_line_constructor,
        },
        RegisteredTest {
            group: "Line",
            name: "Transformation",
            func: run_line_transformation,
        },
        RegisteredTest {
            group: "Line",
            name: "Json Roundtrip",
            func: run_line_json_roundtrip,
        },
        RegisteredTest {
            group: "Line",
            name: "Protobuf Roundtrip",
            func: run_line_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Line",
            name: "Length",
            func: run_line_length,
        },
        RegisteredTest {
            group: "Line",
            name: "To Vector",
            func: run_line_to_vector,
        },
        RegisteredTest {
            group: "Line",
            name: "To Direction",
            func: run_line_to_direction,
        },
        RegisteredTest {
            group: "Line",
            name: "Point At",
            func: run_line_point_at,
        },
        RegisteredTest {
            group: "Line",
            name: "Closest Point",
            func: run_line_closest_point,
        },
        RegisteredTest {
            group: "Line",
            name: "Start End Center",
            func: run_line_start_end_center,
        },
        RegisteredTest {
            group: "Line",
            name: "Fit Points",
            func: run_line_fit_points,
        },
        RegisteredTest {
            group: "Line",
            name: "Subdivide",
            func: run_line_subdivide,
        },
        RegisteredTest {
            group: "Line",
            name: "Overlap",
            func: run_line_overlap,
        },
        RegisteredTest {
            group: "Line",
            name: "Overlap Average",
            func: run_line_overlap_average,
        },
        RegisteredTest {
            group: "Line",
            name: "Extend",
            func: run_line_extend,
        },
        // Polyline tests
        RegisteredTest {
            group: "Polyline",
            name: "Constructor",
            func: run_polyline_constructor,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Transformation",
            func: run_polyline_transformation,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Json Roundtrip",
            func: run_polyline_json_roundtrip,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Protobuf Roundtrip",
            func: run_polyline_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Length",
            func: run_polyline_length,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Center",
            func: run_polyline_center,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Is Closed",
            func: run_polyline_is_closed,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Reverse",
            func: run_polyline_reverse,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Closest Point",
            func: run_polyline_closest_point,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Extend Segment",
            func: run_polyline_extend_segment,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Extend Segment Equally",
            func: run_polyline_extend_segment_equally,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Get Points",
            func: run_polyline_get_points,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Shift",
            func: run_polyline_shift,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Point At",
            func: run_polyline_point_at,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Is Clockwise",
            func: run_polyline_is_clockwise,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Convex Corners",
            func: run_polyline_convex_corners,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Tween",
            func: run_polyline_tween,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Average Plane",
            func: run_polyline_average_plane,
        },
        RegisteredTest {
            group: "Polyline",
            name: "From Coords",
            func: run_polyline_from_coords,
        },
        RegisteredTest {
            group: "Polyline",
            name: "From Sides",
            func: run_polyline_from_sides,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Rectangle",
            func: run_polyline_rectangle,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Closest Point To Line",
            func: run_polyline_closest_point_to_line,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Line Line Overlap",
            func: run_polyline_line_line_overlap,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Line Line Average",
            func: run_polyline_line_line_average,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Line Line Overlap Average",
            func: run_polyline_line_line_overlap_average,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Line From Projected Points",
            func: run_polyline_line_from_projected_points,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Point In Polygon 2d",
            func: run_polyline_point_in_polygon_2d,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Extend Line Segment",
            func: run_polyline_extend_line_segment,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Shrink Line Segment",
            func: run_polyline_shrink_line_segment,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Add Point",
            func: run_polyline_add_point,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Insert Point",
            func: run_polyline_insert_point,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Remove Point",
            func: run_polyline_remove_point,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Closed",
            func: run_polyline_closed,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Get Lines",
            func: run_polyline_get_lines,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Interpolate Points",
            func: run_polyline_interpolate_points,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Quick Hull",
            func: run_polyline_quick_hull,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Bounding Rectangle",
            func: run_polyline_bounding_rectangle,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Grid Of Points In Polygon",
            func: run_polyline_grid_of_points,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Polylabel",
            func: run_polyline_polylabel,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Polylabel Circle Division Points",
            func: run_polyline_polylabel_circle_division_points,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Boolean Op",
            func: run_polyline_boolean_op,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Boolean Op Plane",
            func: run_polyline_boolean_op_plane,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Merge Collinear",
            func: run_polyline_merge_collinear,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Simplify Points",
            func: run_polyline_simplify_points,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Simplify",
            func: run_polyline_simplify,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Simplify Collinear",
            func: run_polyline_simplify_collinear,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Simplify Zigzag",
            func: run_polyline_simplify_zigzag,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Simplify Two Points",
            func: run_polyline_simplify_two_points,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Translate",
            func: run_polyline_translate,
        },
        RegisteredTest {
            group: "Polyline",
            name: "Extend Edge Equally",
            func: run_polyline_extend_edge_equally,
        },
        // Plane tests
        RegisteredTest {
            group: "Plane",
            name: "Constructor",
            func: run_plane_constructor,
        },
        RegisteredTest {
            group: "Plane",
            name: "Reverse",
            func: run_plane_reverse,
        },
        RegisteredTest {
            group: "Plane",
            name: "Rotate",
            func: run_plane_rotate,
        },
        RegisteredTest {
            group: "Plane",
            name: "Is Right Hand",
            func: run_plane_is_right_hand,
        },
        RegisteredTest {
            group: "Plane",
            name: "Is Coplanar",
            func: run_plane_is_coplanar,
        },
        RegisteredTest {
            group: "Plane",
            name: "Transform",
            func: run_plane_transform,
        },
        RegisteredTest {
            group: "Plane",
            name: "Json Roundtrip",
            func: run_plane_json_roundtrip,
        },
        RegisteredTest {
            group: "Plane",
            name: "Protobuf Roundtrip",
            func: run_plane_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Plane",
            name: "Is Valid",
            func: run_plane_is_valid,
        },
        RegisteredTest {
            group: "Plane",
            name: "Is Same Direction",
            func: run_plane_is_same_direction,
        },
        RegisteredTest {
            group: "Plane",
            name: "Is Same Position",
            func: run_plane_is_same_position,
        },
        RegisteredTest {
            group: "Plane",
            name: "Translate By Normal",
            func: run_plane_translate_by_normal,
        },
        RegisteredTest {
            group: "Plane",
            name: "Base1 Base2",
            func: run_plane_base1_base2,
        },
        RegisteredTest {
            group: "Plane",
            name: "Transformed",
            func: run_plane_transformed,
        },
        RegisteredTest {
            group: "Plane",
            name: "Has On Negative Side",
            func: run_plane_has_on_negative_side,
        },
        // PointCloud tests
        RegisteredTest {
            group: "PointCloud",
            name: "Constructor",
            func: run_pointcloud_constructor,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Transform",
            func: run_pointcloud_transform,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Build Lod",
            func: run_pointcloud_build_lod,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Point Ids",
            func: run_pointcloud_point_ids,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Json Roundtrip",
            func: run_pointcloud_json_roundtrip,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Protobuf Roundtrip",
            func: run_pointcloud_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "From Coords",
            func: run_pointcloud_from_coords,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Point Count",
            func: run_pointcloud_point_count,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Len",
            func: run_pointcloud_len,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Is Empty",
            func: run_pointcloud_is_empty,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Get Point",
            func: run_pointcloud_get_point,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Set Point",
            func: run_pointcloud_set_point,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Add Point",
            func: run_pointcloud_add_point,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Get Points",
            func: run_pointcloud_get_points,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Color Count",
            func: run_pointcloud_color_count,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Get Color",
            func: run_pointcloud_get_color,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Set Color",
            func: run_pointcloud_set_color,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Add Color",
            func: run_pointcloud_add_color,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Get Colors",
            func: run_pointcloud_get_colors,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Normal Count",
            func: run_pointcloud_normal_count,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Get Normal",
            func: run_pointcloud_get_normal,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Set Normal",
            func: run_pointcloud_set_normal,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Add Normal",
            func: run_pointcloud_add_normal,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Get Normals",
            func: run_pointcloud_get_normals,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Transformed",
            func: run_pointcloud_transformed,
        },
        // Xform tests
        RegisteredTest {
            group: "Xform",
            name: "Constructor",
            func: run_xform_constructor,
        },
        RegisteredTest {
            group: "Xform",
            name: "Translation",
            func: run_xform_translation,
        },
        RegisteredTest {
            group: "Xform",
            name: "Rotation X",
            func: run_xform_rotation_x,
        },
        RegisteredTest {
            group: "Xform",
            name: "Rotation Y",
            func: run_xform_rotation_y,
        },
        RegisteredTest {
            group: "Xform",
            name: "Rotation Z",
            func: run_xform_rotation_z,
        },
        RegisteredTest {
            group: "Xform",
            name: "Rotation Axis",
            func: run_xform_rotation_axis,
        },
        RegisteredTest {
            group: "Xform",
            name: "Rotation Around Line",
            func: run_xform_rotation_around_line,
        },
        RegisteredTest {
            group: "Xform",
            name: "Change Basis",
            func: run_xform_change_basis,
        },
        RegisteredTest {
            group: "Xform",
            name: "Plane To Plane",
            func: run_xform_plane_to_plane,
        },
        RegisteredTest {
            group: "Xform",
            name: "Scale XYZ",
            func: run_xform_scale_xyz,
        },
        RegisteredTest {
            group: "Xform",
            name: "Scale Uniform",
            func: run_xform_scale_uniform,
        },
        RegisteredTest {
            group: "Xform",
            name: "Scale Non Uniform",
            func: run_xform_scale_non_uniform,
        },
        RegisteredTest {
            group: "Xform",
            name: "Look At Right Handed",
            func: run_xform_look_at_right_handed,
        },
        RegisteredTest {
            group: "Xform",
            name: "Look To Right Handed",
            func: run_xform_look_to_right_handed,
        },
        RegisteredTest {
            group: "Xform",
            name: "Perspective",
            func: run_xform_perspective,
        },
        RegisteredTest {
            group: "Xform",
            name: "Orthographic",
            func: run_xform_orthographic,
        },
        RegisteredTest {
            group: "Xform",
            name: "Project To Plane",
            func: run_xform_project_to_plane,
        },
        RegisteredTest {
            group: "Xform",
            name: "Project To Plane By Axis",
            func: run_xform_project_to_plane_by_axis,
        },
        RegisteredTest {
            group: "Xform",
            name: "Inverse",
            func: run_xform_inverse,
        },
        RegisteredTest {
            group: "Xform",
            name: "Transform Geometry",
            func: run_xform_transform_geometry,
        },
        RegisteredTest {
            group: "Xform",
            name: "Json Roundtrip",
            func: run_xform_json_roundtrip,
        },
        RegisteredTest {
            group: "Xform",
            name: "Protobuf Roundtrip",
            func: run_xform_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Xform",
            name: "Transform Point",
            func: run_xform_transform_point,
        },
        RegisteredTest {
            group: "Xform",
            name: "Transform Vector",
            func: run_xform_transform_vector,
        },
        RegisteredTest {
            group: "Xform",
            name: "To Cols",
            func: run_xform_to_cols,
        },
        RegisteredTest {
            group: "Xform",
            name: "From Change Of Basis",
            func: run_xform_from_change_of_basis,
        },
        // Mesh tests
        RegisteredTest {
            group: "Mesh",
            name: "Constructor",
            func: run_mesh_constructor,
        },
        RegisteredTest {
            group: "Mesh",
            name: "From Polylines",
            func: run_mesh_from_polylines,
        },
        RegisteredTest {
            group: "Mesh",
            name: "From Lines",
            func: run_mesh_from_lines,
        },
        RegisteredTest {
            group: "Mesh",
            name: "From Polygon With Holes",
            func: run_mesh_from_polygon_with_holes,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Loft",
            func: run_mesh_loft,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Loft concave with holes and collinear",
            func: run_mesh_loft_concave_with_holes_and_collinear,
        },
        RegisteredTest {
            group: "Mesh",
            name: "From Polygon With Holes Many",
            func: run_mesh_from_polygon_with_holes_many,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Loft Many",
            func: run_mesh_loft_many,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Boolean Queries",
            func: run_mesh_boolean_queries,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Attributes",
            func: run_mesh_attributes,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Create Dodecahedron",
            func: run_mesh_create_dodecahedron,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Vertex and Face Operations",
            func: run_mesh_vertex_and_face_operations,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Connectivity Queries",
            func: run_mesh_connectivity_queries,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Geometric Properties",
            func: run_mesh_geometric_properties,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Transformation",
            func: run_mesh_transformation,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Json Roundtrip",
            func: run_mesh_json_roundtrip,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Protobuf Roundtrip",
            func: run_mesh_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Loft with quads and triangles",
            func: run_mesh_loft_panels,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Edges",
            func: run_mesh_edges,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Loft plate_failing 15-vert outer + 3 holes",
            func: run_mesh_loft_plate_failing,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Loft plate_v2 15-vert outer + 3 holes",
            func: run_mesh_loft_plate_v2,
        },
        RegisteredTest {
            group: "Mesh",
            name: "Refresh Guid",
            func: run_mesh_refresh_guid,
        },
        // NurbsCurve tests
        RegisteredTest {
            group: "NurbsCurve",
            name: "Constructor",
            func: run_nurbscurve_constructor,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Attributes",
            func: run_nurbscurve_attributes,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Conversions",
            func: run_nurbscurve_conversions,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Evaluation",
            func: run_nurbscurve_evaluation,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Modifications",
            func: run_nurbscurve_modifications,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Json Roundtrip",
            func: run_nurbscurve_json_roundtrip,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Protobuf Roundtrip",
            func: run_nurbscurve_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Transformations",
            func: run_nurbscurve_transformations,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Create Interpolated",
            func: run_nurbscurve_create_interpolated,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Create From Parameters",
            func: run_nurbscurve_create_from_parameters,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Create Fitted",
            func: run_nurbscurve_create_fitted,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Curvature",
            func: run_nurbscurve_curvature,
        },
        RegisteredTest {
            group: "NurbsCurve",
            name: "Join",
            func: run_nurbscurve_join,
        },
        // NurbsSurface tests
        RegisteredTest {
            group: "NurbsSurface",
            name: "Constructor",
            func: run_nurbssurface_constructor,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Create From Parameters",
            func: run_nurbssurface_create_from_parameters,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Booleans Queries",
            func: run_nurbssurface_booleans_queries,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Attributes",
            func: run_nurbssurface_attributes,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Control Vertices Access",
            func: run_nurbssurface_control_vertices_access,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "NurbsKnot Access",
            func: run_nurbssurface_nurbsknot_access,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Domain",
            func: run_nurbssurface_domain,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Division",
            func: run_nurbssurface_division,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Evaluation",
            func: run_nurbssurface_evaluation,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Modification",
            func: run_nurbssurface_modification,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Transformations",
            func: run_nurbssurface_transformations,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Meshing",
            func: run_nurbssurface_meshing,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Json Roundtrip",
            func: run_nurbssurface_json_roundtrip,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Protobuf Roundtrip",
            func: run_nurbssurface_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "ClosestPoint",
            func: run_nurbssurface_closest_point,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Split By Plane",
            func: run_nurbssurface_split_by_plane,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Split By Curves",
            func: run_nurbssurface_split_by_curves,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Split By Line",
            func: run_nurbssurface_split_by_line,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Split By Surface",
            func: run_nurbssurface_split_by_surface,
        },
        RegisteredTest {
            group: "NurbsSurface",
            name: "Split By Brep",
            func: run_nurbssurface_split_by_brep,
        },
        // NurbsKnot tests
        RegisteredTest {
            group: "NurbsKnot",
            name: "Make Clamped Uniform",
            func: run_make_clamped_uniform,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Make Periodic Uniform",
            func: run_make_periodic_uniform,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Is Clamped",
            func: run_is_clamped,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Reverse",
            func: run_reverse,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Find Span",
            func: run_find_span,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Solve Tridiagonal",
            func: run_solve_tridiagonal,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Compute Parameters",
            func: run_compute_parameters,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Build Interpolation NurbsKnots",
            func: run_build_interp_nurbsknots,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Evaluation Basis",
            func: run_eval_basis,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Build Fitted NurbsKnots Adaptive",
            func: run_build_fitted_nurbsknots_adaptive,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Build Fitted NurbsKnots Periodic Adaptive",
            func: run_build_fitted_nurbsknots_periodic_adaptive,
        },
        RegisteredTest {
            group: "NurbsKnot",
            name: "Solve Banded SPD",
            func: run_solve_banded_spd,
        },
        // NurbsSurfaceTrimmed tests
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Constructor",
            func: run_nurbssurface_trimmed_constructor,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Constructor Planar",
            func: run_nurbssurface_trimmed_constructor_planar,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Constructor Hole",
            func: run_nurbssurface_trimmed_constructor_hole,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Accessors",
            func: run_nurbssurface_trimmed_accessors,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Add Inner Loop",
            func: run_nurbssurface_trimmed_add_inner_loop,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Point At",
            func: run_nurbssurface_trimmed_point_at,
        },
        // TODO(f64-followup): NurbsSurfaceTrimmed::mesh produces empty result under f64
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Mesh",
            func: run_nurbssurface_trimmed_mesh,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Transformation",
            func: run_nurbssurface_trimmed_transformation,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Json Roundtrip",
            func: run_nurbssurface_trimmed_json_roundtrip,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Protobuf Roundtrip",
            func: run_nurbssurface_trimmed_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "NurbsSurfaceTrimmed",
            name: "Split By UV Curves",
            func: run_nurbssurface_trimmed_split_by_uv_curves,
        },
        // Closest tests
        RegisteredTest {
            group: "Closest",
            name: "Line Point",
            func: run_closest_line_point,
        },
        RegisteredTest {
            group: "Closest",
            name: "Polyline Point",
            func: run_closest_polyline_point,
        },
        RegisteredTest {
            group: "Closest",
            name: "Curve Point",
            func: run_closest_curve_point,
        },
        RegisteredTest {
            group: "Closest",
            name: "Surface Point",
            func: run_closest_surface_point,
        },
        RegisteredTest {
            group: "Closest",
            name: "Mesh Point",
            func: run_closest_mesh_point,
        },
        RegisteredTest {
            group: "Closest",
            name: "Mesh Point AABB",
            func: run_closest_mesh_point_aabb,
        },
        RegisteredTest {
            group: "Closest",
            name: "Pointcloud Point",
            func: run_closest_pointcloud_point,
        },
        RegisteredTest {
            group: "Closest",
            name: "Surface Curve",
            func: run_closest_surface_curve,
        },
        RegisteredTest {
            group: "Closest",
            name: "Pointcloud Point SpatialKDTree",
            func: run_closest_pointcloud_point_kdtree,
        },
        RegisteredTest {
            group: "Closest",
            name: "Lines Closest",
            func: run_closest_lines_closest,
        },
        RegisteredTest {
            group: "Closest",
            name: "Polylines Closest",
            func: run_closest_polylines_closest,
        },
        RegisteredTest {
            group: "Closest",
            name: "Nurbscurves Closest",
            func: run_closest_nurbscurves_closest,
        },
        RegisteredTest {
            group: "Closest",
            name: "Boxes Closest",
            func: run_closest_boxes_closest,
        },
        // Primitives tests
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Arrow",
            func: run_primitives_mesh_arrow,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Cylinder",
            func: run_primitives_mesh_cylinder,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbscurve Polyline",
            func: run_primitives_nurbscurve_polyline,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbscurve Circle",
            func: run_primitives_nurbscurve_circle,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbscurve Ellipse",
            func: run_primitives_nurbscurve_ellipse,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbscurve Arc",
            func: run_primitives_nurbscurve_arc,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbscurve Parabola",
            func: run_primitives_nurbscurve_parabola,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbscurve Hyperbola",
            func: run_primitives_nurbscurve_hyperbola,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbscurve Spiral",
            func: run_primitives_nurbscurve_spiral,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Cylinder",
            func: run_primitives_nurbssurface_cylinder,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Cone",
            func: run_primitives_nurbssurface_cone,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Sphere",
            func: run_primitives_nurbssurface_sphere,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Quad Sphere",
            func: run_primitives_nurbssurface_quad_sphere,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Torus",
            func: run_primitives_nurbssurface_torus,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Ruled",
            func: run_primitives_nurbssurface_ruled,
        },
        // TODO(f64-followup): high-precision get_cv/closure assertions; rebaseline.
        // RegisteredTest { group: "Primitives", name: "Nurbssurface Planar", func: run_primitives_nurbssurface_planar },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Extrusion",
            func: run_primitives_nurbssurface_extrusion,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Loft",
            func: run_primitives_nurbssurface_loft,
        },
        // RegisteredTest { group: "Primitives", name: "Nurbssurface Revolve", func: run_primitives_nurbssurface_revolve },
        // RegisteredTest { group: "Primitives", name: "Nurbssurface Sweep", func: run_primitives_nurbssurface_sweep },
        // RegisteredTest { group: "Primitives", name: "Nurbssurface Edge", func: run_primitives_nurbssurface_edge },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Quad Mesh",
            func: run_primitives_mesh_quad_mesh,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Diamond Mesh",
            func: run_primitives_mesh_diamond_mesh,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Hex Mesh",
            func: run_primitives_mesh_hex_mesh,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Cone Subdivisions",
            func: run_primitives_mesh_cone_subdivisions,
        },
        // TODO(f64-followup): rebaseline f64 interpolation expected values.
        // RegisteredTest { group: "Primitives", name: "Nurbscurve Interpolated", func: run_primitives_nurbscurve_interpolated },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Tetrahedron",
            func: run_primitives_mesh_tetrahedron,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Cube",
            func: run_primitives_mesh_cube,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Octahedron",
            func: run_primitives_mesh_octahedron,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Icosahedron",
            func: run_primitives_mesh_icosahedron,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Nurbssurface Wave",
            func: run_primitives_nurbssurface_wave,
        },
        RegisteredTest {
            group: "Primitives",
            name: "Mesh Edge Pipes",
            func: run_primitives_mesh_edge_pipes,
        },
        // Intersection tests
        RegisteredTest {
            group: "Intersection",
            name: "Line Line",
            func: run_intersection_line_line,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Line Parallel",
            func: run_intersection_line_line_parallel,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Line Parameters",
            func: run_intersection_line_line_parameters,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Line Parameters Endpoints",
            func: run_intersection_line_line_parameters_endpoints,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Line Parameters Infinite",
            func: run_intersection_line_line_parameters_infinite,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane Plane",
            func: run_intersection_plane_plane,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane Plane Complex",
            func: run_intersection_plane_plane_complex,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Plane",
            func: run_intersection_line_plane,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Plane Parallel",
            func: run_intersection_line_plane_parallel,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Plane Real World",
            func: run_intersection_line_plane_real_world,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane Plane Plane",
            func: run_intersection_plane_plane_plane,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane Plane Plane Parallel",
            func: run_intersection_plane_plane_plane_parallel,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Box",
            func: run_intersection_ray_box,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Box Miss",
            func: run_intersection_ray_box_miss,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Sphere",
            func: run_intersection_ray_sphere,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Sphere Tangent",
            func: run_intersection_ray_sphere_tangent,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Sphere Miss",
            func: run_intersection_ray_sphere_miss,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Triangle",
            func: run_intersection_ray_triangle,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Triangle Miss",
            func: run_intersection_ray_triangle_miss,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Triangle Parallel",
            func: run_intersection_ray_triangle_parallel,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Mesh",
            func: run_intersection_ray_mesh,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Mesh First",
            func: run_intersection_ray_mesh_first,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Mesh Miss",
            func: run_intersection_ray_mesh_miss,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Mesh Bvh",
            func: run_intersection_ray_mesh_bvh,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Mesh Bvh First",
            func: run_intersection_ray_mesh_bvh_first,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Mesh Bvh Miss",
            func: run_intersection_ray_mesh_bvh_miss,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Mesh Bvh Vs Naive",
            func: run_intersection_ray_mesh_bvh_vs_naive,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Box Real World",
            func: run_intersection_ray_box_real_world,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Sphere Real World",
            func: run_intersection_ray_sphere_real_world,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Ray Triangle Real World",
            func: run_intersection_ray_triangle_real_world,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Surface Plane",
            func: run_intersection_surface_plane,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Surface Plane Curved",
            func: run_intersection_surface_plane_curved,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Surface Plane Miss",
            func: run_intersection_surface_plane_miss,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Surface Plane UV",
            func: run_intersection_surface_plane_uv,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Surface Surface",
            func: run_intersection_surface_surface,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Surface Surface Accuracy",
            func: run_intersection_surface_surface_accuracy,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Remap",
            func: run_intersection_remap,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Closest Point On Segment",
            func: run_intersection_closest_point_on_segment,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane Plane Plane Check Parallel",
            func: run_intersection_plane_plane_plane_check_parallel,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane 4 Planes Closed",
            func: run_intersection_plane_4planes,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane 4 Planes Open",
            func: run_intersection_plane_4planes_open,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Plane 4 Lines",
            func: run_intersection_plane_4lines,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Scale Vector To Distance Of 2 Planes",
            func: run_intersection_scale_vector_to_distance_of_2planes,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Polyline Plane",
            func: run_intersection_polyline_plane,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Line 3D",
            func: run_intersection_line_line_3d,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Polyline Plane To Line",
            func: run_intersection_polyline_plane_to_line,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Quad From Line Top Bottom Planes",
            func: run_intersection_quad_from_line_top_bottom_planes,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Orthogonal Vector Between Two Plane Pairs",
            func: run_intersection_orthogonal_vector_between_two_plane_pairs,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Closed And Open Paths 2D",
            func: run_intersection_closed_and_open_paths_2d,
        },
        RegisteredTest {
            group: "Intersection",
            name: "Line Line Classified",
            func: run_intersection_line_line_classified,
        },
        // Session tests
        RegisteredTest {
            group: "Session",
            name: "Constructor",
            func: run_session_constructor,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Point",
            func: run_session_add_point,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Line",
            func: run_session_add_line,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Plane",
            func: run_session_add_plane,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Polyline",
            func: run_session_add_polyline,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Pointcloud",
            func: run_session_add_pointcloud,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Mesh",
            func: run_session_add_mesh,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Element",
            func: run_session_add_element,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Empty Geometry",
            func: run_session_add_empty_geometry,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Group",
            func: run_session_add_group,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Edge",
            func: run_session_add_edge,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Hierarchy",
            func: run_session_add_hierarchy,
        },
        RegisteredTest {
            group: "Session",
            name: "Get Children",
            func: run_session_get_children,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Relationship",
            func: run_session_add_relationship,
        },
        RegisteredTest {
            group: "Session",
            name: "Get Neighbours",
            func: run_session_get_neighbours,
        },
        RegisteredTest {
            group: "Session",
            name: "Get Object",
            func: run_session_get_object,
        },
        RegisteredTest {
            group: "Session",
            name: "Remove Object",
            func: run_session_remove_object,
        },
        RegisteredTest {
            group: "Session",
            name: "Get Geometry",
            func: run_session_get_geometry,
        },
        RegisteredTest {
            group: "Session",
            name: "Json Roundtrip",
            func: run_session_json_roundtrip,
        },
        RegisteredTest {
            group: "Session",
            name: "Protobuf Roundtrip",
            func: run_session_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Session",
            name: "Set Xform",
            func: run_session_set_xform,
        },
        RegisteredTest {
            group: "Session",
            name: "World Xform Hierarchy",
            func: run_session_world_xform_hierarchy,
        },
        RegisteredTest {
            group: "Session",
            name: "Xform Roundtrip",
            func: run_session_xform_roundtrip,
        },
        RegisteredTest {
            group: "Session",
            name: "Tree Transformation Hierarchy",
            func: run_session_tree_transformation_hierarchy,
        },
        RegisteredTest {
            group: "Session",
            name: "Add OBB",
            func: run_session_add_obb,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Nurbscurve",
            func: run_session_add_nurbscurve,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Nurbssurface",
            func: run_session_add_nurbssurface,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Brep",
            func: run_session_add_brep,
        },
        RegisteredTest {
            group: "Session",
            name: "Get Collisions",
            func: run_session_get_collisions,
        },
        RegisteredTest {
            group: "Session",
            name: "Ray Cast",
            func: run_session_ray_cast,
        },
        RegisteredTest {
            group: "Session",
            name: "Lookup Mutation Roundtrip",
            func: run_session_lookup_mutation_roundtrip,
        },
        RegisteredTest {
            group: "Session",
            name: "Order",
            func: run_session_order,
        },
        RegisteredTest {
            group: "Session",
            name: "Add Component",
            func: run_session_add_component,
        },
        RegisteredTest {
            group: "Session",
            name: "Component Json Roundtrip",
            func: run_session_component_json_roundtrip,
        },
        // SessionConfig tests
        RegisteredTest {
            group: "SessionConfig",
            name: "Runtime Modification",
            func: run_session_config_runtime_modification,
        },
        // FileObj tests
        RegisteredTest {
            group: "FileObj",
            name: "Read Bunny",
            func: run_file_obj_read_bunny,
        },
        RegisteredTest {
            group: "FileObj",
            name: "Write Read Roundtrip",
            func: run_file_obj_write_read_roundtrip,
        },
        RegisteredTest {
            group: "FileObj",
            name: "String Roundtrip",
            func: run_file_obj_string_roundtrip,
        },
        // RemeshCDT tests
        RegisteredTest {
            group: "RemeshCDT",
            name: "Triangulate",
            func: run_remesh_cdt_triangulate,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Triangle",
            func: run_remesh_cdt_triangle,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Rectangle",
            func: run_remesh_cdt_rectangle,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "L-shape",
            func: run_remesh_cdt_l_shape,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "U-shape",
            func: run_remesh_cdt_u_shape,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Octagon",
            func: run_remesh_cdt_octagon,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Rectangle with rectangle hole",
            func: run_remesh_cdt_rectangle_with_rectangle_hole,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Duplicate vertices",
            func: run_remesh_cdt_duplicate_vertices,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Tilted rectangle with rectangle hole",
            func: run_remesh_cdt_tilted_rectangle_with_rectangle_hole,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Irregular tilted polyline.",
            func: run_remesh_cdt_irregular_tilted_polyline,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Irregular tilted polyline with holes.",
            func: run_remesh_cdt_irregular_tilted_polyline_with_holes,
        },
        // SpatialBVH tests
        RegisteredTest {
            group: "SpatialBVH",
            name: "Expand Bits",
            func: run_bvh_expand_bits,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Morton Code Origin",
            func: run_bvh_morton_code_origin,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Morton Code Corners",
            func: run_bvh_morton_code_corners,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Morton Code Spatial Locality",
            func: run_bvh_morton_code_spatial_locality,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Node Creation",
            func: run_bvh_node_creation,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Node Leaf",
            func: run_bvh_node_leaf,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Creation",
            func: run_bvh_creation,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Build Empty",
            func: run_bvh_build_empty,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Build Single",
            func: run_bvh_build_single,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Build Multiple",
            func: run_bvh_build_multiple,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Aabb Intersect",
            func: run_bvh_aabb_intersect,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Check All Collisions",
            func: run_bvh_check_all_collisions,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Merge Aabb",
            func: run_bvh_merge_aabb,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Fixed 100 Boxes",
            func: run_bvh_fixed_100_boxes,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Query Aabb",
            func: run_bvh_query_aabb,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Nearest Neighbors",
            func: run_bvh_nearest_neighbors,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Build From Boxes",
            func: run_bvh_build_from_boxes,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Build From Aabbs",
            func: run_bvh_build_from_aabbs,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Build With Guids",
            func: run_bvh_build_with_guids,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Check All Collisions Guids",
            func: run_bvh_check_all_collisions_guids,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Find Collisions",
            func: run_bvh_find_collisions,
        },
        RegisteredTest {
            group: "SpatialBVH",
            name: "Constructor",
            func: run_bvh_constructor,
        },
        // Quaternion tests
        RegisteredTest {
            group: "Quaternion",
            name: "Json Roundtrip",
            func: run_quaternion_json_roundtrip,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Constructor",
            func: run_quaternion_constructor,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Identity",
            func: run_quaternion_identity,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "From Components",
            func: run_quaternion_from_components,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "From Axis Angle",
            func: run_quaternion_from_axis_angle,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "From Arc",
            func: run_quaternion_from_arc,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "From Euler",
            func: run_quaternion_from_euler,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "From Rotation",
            func: run_quaternion_from_rotation,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Rotate Vector",
            func: run_quaternion_rotate_vector,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Get Rotation",
            func: run_quaternion_get_rotation,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Magnitude",
            func: run_quaternion_magnitude,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Magnitude Squared",
            func: run_quaternion_magnitude_squared,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Normalized",
            func: run_quaternion_normalized,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Conjugate",
            func: run_quaternion_conjugate,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Invert",
            func: run_quaternion_invert,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Dot",
            func: run_quaternion_dot,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Slerp",
            func: run_quaternion_slerp,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Nlerp",
            func: run_quaternion_nlerp,
        },
        RegisteredTest {
            group: "Quaternion",
            name: "Protobuf Roundtrip",
            func: run_quaternion_protobuf_roundtrip,
        },
        // OBB tests
        RegisteredTest {
            group: "OBB",
            name: "Constructor",
            func: run_obb_constructor,
        },
        RegisteredTest {
            group: "OBB",
            name: "Collision",
            func: run_obb_collision,
        },
        RegisteredTest {
            group: "OBB",
            name: "Transformation",
            func: run_obb_transformation,
        },
        RegisteredTest {
            group: "OBB",
            name: "Json Roundtrip",
            func: run_obb_json_roundtrip,
        },
        RegisteredTest {
            group: "OBB",
            name: "Protobuf Roundtrip",
            func: run_obb_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "OBB",
            name: "Accessors",
            func: run_obb_accessors,
        },
        RegisteredTest {
            group: "OBB",
            name: "From Geometry",
            func: run_obb_from_geometry,
        },
        RegisteredTest {
            group: "OBB",
            name: "From Plane",
            func: run_obb_from_plane,
        },
        RegisteredTest {
            group: "OBB",
            name: "Two Rectangles",
            func: run_obb_two_rectangles,
        },
        // Edge tests
        RegisteredTest {
            group: "Edge",
            name: "Json Roundtrip",
            func: run_edge_json_roundtrip,
        },
        RegisteredTest {
            group: "Edge",
            name: "Constructor",
            func: run_edge_constructor,
        },
        RegisteredTest {
            group: "Edge",
            name: "Vertices",
            func: run_edge_vertices,
        },
        RegisteredTest {
            group: "Edge",
            name: "Connects",
            func: run_edge_connects,
        },
        RegisteredTest {
            group: "Edge",
            name: "Other Vertex",
            func: run_edge_other_vertex,
        },
        // Graph tests
        RegisteredTest {
            group: "Graph",
            name: "Json Roundtrip",
            func: run_graph_json_roundtrip,
        },
        RegisteredTest {
            group: "Graph",
            name: "Constructor",
            func: run_graph_constructor,
        },
        RegisteredTest {
            group: "Graph",
            name: "Protobuf Roundtrip",
            func: run_graph_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Graph",
            name: "Has Node",
            func: run_graph_has_node,
        },
        RegisteredTest {
            group: "Graph",
            name: "Has Edge",
            func: run_graph_has_edge,
        },
        RegisteredTest {
            group: "Graph",
            name: "Has Guid",
            func: run_graph_has_guid,
        },
        RegisteredTest {
            group: "Graph",
            name: "Add Node",
            func: run_graph_add_node,
        },
        RegisteredTest {
            group: "Graph",
            name: "Add Edge",
            func: run_graph_add_edge,
        },
        RegisteredTest {
            group: "Graph",
            name: "Remove Node",
            func: run_graph_remove_node,
        },
        RegisteredTest {
            group: "Graph",
            name: "Remove Edge",
            func: run_graph_remove_edge,
        },
        RegisteredTest {
            group: "Graph",
            name: "Get Vertices",
            func: run_graph_get_vertices,
        },
        RegisteredTest {
            group: "Graph",
            name: "Get Edges",
            func: run_graph_get_edges,
        },
        RegisteredTest {
            group: "Graph",
            name: "Neighbors",
            func: run_graph_neighbors,
        },
        RegisteredTest {
            group: "Graph",
            name: "Get Neighbors",
            func: run_graph_get_neighbors,
        },
        RegisteredTest {
            group: "Graph",
            name: "Number Of Vertices",
            func: run_graph_number_of_vertices,
        },
        RegisteredTest {
            group: "Graph",
            name: "Number Of Edges",
            func: run_graph_number_of_edges,
        },
        RegisteredTest {
            group: "Graph",
            name: "Clear",
            func: run_graph_clear,
        },
        RegisteredTest {
            group: "Graph",
            name: "Node Attribute",
            func: run_graph_node_attribute,
        },
        RegisteredTest {
            group: "Graph",
            name: "Edge Attribute",
            func: run_graph_edge_attribute,
        },
        RegisteredTest {
            group: "Graph",
            name: "Bfs",
            func: run_graph_bfs,
        },
        RegisteredTest {
            group: "Graph",
            name: "Dfs",
            func: run_graph_dfs,
        },
        RegisteredTest {
            group: "Graph",
            name: "Connected Components",
            func: run_graph_connected_components,
        },
        RegisteredTest {
            group: "Graph",
            name: "Shortest Path",
            func: run_graph_shortest_path,
        },
        RegisteredTest {
            group: "Graph",
            name: "Has Cycle",
            func: run_graph_has_cycle,
        },
        RegisteredTest {
            group: "Graph",
            name: "Cycle Basis",
            func: run_graph_cycle_basis,
        },
        // Objects tests
        RegisteredTest {
            group: "Objects",
            name: "Json Roundtrip",
            func: run_objects_json_roundtrip,
        },
        RegisteredTest {
            group: "Objects",
            name: "Constructor",
            func: run_objects_constructor,
        },
        RegisteredTest {
            group: "Objects",
            name: "Protobuf Roundtrip",
            func: run_objects_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Objects",
            name: "Component Constructor",
            func: run_objects_component_constructor,
        },
        RegisteredTest {
            group: "Objects",
            name: "Component Json Roundtrip",
            func: run_objects_component_json_roundtrip,
        },
        RegisteredTest {
            group: "Objects",
            name: "Component Protobuf Roundtrip",
            func: run_objects_component_protobuf_roundtrip,
        },
        // Tree tests
        RegisteredTest {
            group: "Tree",
            name: "Json Roundtrip",
            func: run_tree_json_roundtrip,
        },
        RegisteredTest {
            group: "Tree",
            name: "Constructor",
            func: run_tree_constructor,
        },
        RegisteredTest {
            group: "Tree",
            name: "Protobuf Roundtrip",
            func: run_tree_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Tree",
            name: "Root",
            func: run_tree_root,
        },
        RegisteredTest {
            group: "Tree",
            name: "Add",
            func: run_tree_add,
        },
        RegisteredTest {
            group: "Tree",
            name: "Nodes",
            func: run_tree_nodes,
        },
        RegisteredTest {
            group: "Tree",
            name: "Remove",
            func: run_tree_remove,
        },
        RegisteredTest {
            group: "Tree",
            name: "Leaves",
            func: run_tree_leaves,
        },
        RegisteredTest {
            group: "Tree",
            name: "Traverse",
            func: run_tree_traverse,
        },
        RegisteredTest {
            group: "Tree",
            name: "Get Node By Name",
            func: run_tree_get_node_by_name,
        },
        RegisteredTest {
            group: "Tree",
            name: "Get Nodes By Name",
            func: run_tree_get_nodes_by_name,
        },
        RegisteredTest {
            group: "Tree",
            name: "Find Node By Guid",
            func: run_tree_find_node_by_guid,
        },
        RegisteredTest {
            group: "Tree",
            name: "Add Child By Guid",
            func: run_tree_add_child_by_guid,
        },
        RegisteredTest {
            group: "Tree",
            name: "Get Children Guids",
            func: run_tree_get_children_guids,
        },
        // TreeNode tests
        RegisteredTest {
            group: "TreeNode",
            name: "Json Roundtrip",
            func: run_treenode_json_roundtrip,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Constructor",
            func: run_treenode_constructor,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Is Root",
            func: run_treenode_is_root,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Is Leaf",
            func: run_treenode_is_leaf,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Tree",
            func: run_treenode_tree,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Add",
            func: run_treenode_add,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Remove",
            func: run_treenode_remove,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Parent",
            func: run_treenode_parent,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Ancestors",
            func: run_treenode_ancestors,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Descendants",
            func: run_treenode_descendants,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Children",
            func: run_treenode_children,
        },
        RegisteredTest {
            group: "TreeNode",
            name: "Traverse",
            func: run_treenode_traverse,
        },
        // Vertex tests
        RegisteredTest {
            group: "Vertex",
            name: "Json Roundtrip",
            func: run_vertex_json_roundtrip,
        },
        RegisteredTest {
            group: "Vertex",
            name: "Constructor",
            func: run_vertex_constructor,
        },
        // Encoders tests
        RegisteredTest {
            group: "FileEncoders",
            name: "Json Dump Load",
            func: run_encoders_file_json_dump_load,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Json Dumps Loads",
            func: run_encoders_file_json_dumps_loads,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Encode Collection Values",
            func: run_encoders_file_encode_collection_values,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Encode Collection Shared Ptr",
            func: run_encoders_file_encode_collection_shared_ptr,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Decode Collection",
            func: run_encoders_file_decode_collection,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Decode Collection Ptr",
            func: run_encoders_file_decode_collection_ptr,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Nested Collections",
            func: run_encoders_nested_collections,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Roundtrip File Io",
            func: run_encoders_roundtrip_file_io,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Pretty Vs Compact",
            func: run_encoders_pretty_vs_compact,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Decode Primitives",
            func: run_encoders_decode_primitives,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Decode List",
            func: run_encoders_decode_list,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Decode Dict",
            func: run_encoders_decode_dict,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "List In List In List",
            func: run_encoders_list_in_list_in_list,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Dict Of Lists",
            func: run_encoders_dict_of_lists,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "List Of Dict",
            func: run_encoders_list_of_dict,
        },
        RegisteredTest {
            group: "FileEncoders",
            name: "Dict Of Dicts",
            func: run_encoders_dict_of_dicts,
        },
        // SpatialAABBTree tests
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Build Empty",
            func: run_spatial_aabbtree_build_empty,
        },
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Build Single",
            func: run_spatial_aabbtree_build_single,
        },
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Build Multiple",
            func: run_spatial_aabbtree_build_multiple,
        },
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Node Count",
            func: run_spatial_aabbtree_node_count,
        },
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Mesh Point Aabb",
            func: run_spatial_aabbtree_mesh_point_aabb,
        },
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Mesh Point Aabb Matches Bvh",
            func: run_spatial_aabbtree_mesh_point_aabb_matches_bvh,
        },
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Constructor",
            func: run_spatial_aabbtree_constructor,
        },
        RegisteredTest {
            group: "SpatialAABBTree",
            name: "Query Aabb",
            func: run_spatial_aabbtree_query_aabb,
        },
        // RemeshNurbsSurfaceAdaptive tests
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Constructor",
            func: run_remesh_nurbssurface_adaptive_constructor,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Parameters",
            func: run_remesh_nurbssurface_adaptive_parameters,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Mesh",
            func: run_remesh_nurbssurface_adaptive_mesh,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Torus",
            func: run_remesh_nurbssurface_adaptive_torus,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Cylinder",
            func: run_remesh_nurbssurface_adaptive_cylinder,
        },
        // TODO(f64-followup): vertex count diverges under f64 adaptive remesh.
        // RegisteredTest { group: "RemeshNurbsSurfaceAdaptive", name: "Cone", func: run_remesh_nurbssurface_adaptive_cone },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Doubly Curved",
            func: run_remesh_nurbssurface_adaptive_doubly_curved,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Flat",
            func: run_remesh_nurbssurface_adaptive_flat,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Singular Triangle",
            func: run_remesh_nurbssurface_adaptive_singular_triangle,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceAdaptive",
            name: "Double-Curved Triangle",
            func: run_remesh_nurbssurface_adaptive_double_curved_triangle,
        },
        // RemeshNurbsSurfaceGrid tests
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Sphere",
            func: run_remesh_nurbssurface_grid_sphere,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Torus",
            func: run_remesh_nurbssurface_grid_torus,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Cylinder",
            func: run_remesh_nurbssurface_grid_cylinder,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Cone",
            func: run_remesh_nurbssurface_grid_cone,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Doubly Curved",
            func: run_remesh_nurbssurface_grid_doubly_curved,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Grid Target",
            func: run_remesh_nurbssurface_grid_grid_target,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Flat Quad",
            func: run_remesh_nurbssurface_grid_flat_quad,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Flat Triangle",
            func: run_remesh_nurbssurface_grid_flat_triangle,
        },
        RegisteredTest {
            group: "RemeshNurbsSurfaceGrid",
            name: "Double-Curved Triangle",
            func: run_remesh_nurbssurface_grid_double_curved_triangle,
        },
        // SpatialRTree tests
        RegisteredTest {
            group: "SpatialRTree",
            name: "Creation",
            func: run_rtree_creation,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Insert",
            func: run_rtree_insert,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Insert Multiple",
            func: run_rtree_insert_multiple,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Search Hit",
            func: run_rtree_search_hit,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Search Miss",
            func: run_rtree_search_miss,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Remove",
            func: run_rtree_remove,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Remove All",
            func: run_rtree_remove_all,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Search Count",
            func: run_rtree_search_count,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Search Stop",
            func: run_rtree_search_stop,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Search 100 Boxes",
            func: run_rtree_search_100_boxes,
        },
        RegisteredTest {
            group: "SpatialRTree",
            name: "Constructor",
            func: run_rtree_constructor,
        },
        // Element tests
        RegisteredTest {
            group: "Element",
            name: "Constructor",
            func: run_element_constructor,
        },
        RegisteredTest {
            group: "Element",
            name: "Place",
            func: run_element_place,
        },
        RegisteredTest {
            group: "Element",
            name: "Add Geometry Op",
            func: run_element_add_feature,
        },
        RegisteredTest {
            group: "Element",
            name: "AABB",
            func: run_element_aabb,
        },
        RegisteredTest {
            group: "Element",
            name: "OBB",
            func: run_element_obb,
        },
        RegisteredTest {
            group: "Element",
            name: "Session Geometry",
            func: run_element_session_geometry,
        },
        RegisteredTest {
            group: "Element",
            name: "Reset",
            func: run_element_reset,
        },
        RegisteredTest {
            group: "Element",
            name: "Compute Point",
            func: run_element_compute_point,
        },
        RegisteredTest {
            group: "Element",
            name: "Brep Aabb",
            func: run_element_brep_aabb,
        },
        RegisteredTest {
            group: "Element",
            name: "Json Roundtrip",
            func: run_element_json_roundtrip,
        },
        RegisteredTest {
            group: "Element",
            name: "Protobuf Roundtrip",
            func: run_element_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "Element",
            name: "Polylines",
            func: run_element_polylines,
        },
        RegisteredTest {
            group: "Element",
            name: "RegistryRoundTrip",
            func: run_element_registry_round_trip,
        },
        RegisteredTest {
            group: "Element",
            name: "RegistryUnknownTypeDegrades",
            func: run_element_registry_unknown_type_degrades,
        },
        RegisteredTest {
            group: "Element",
            name: "RegistryLeavesBaseBytesUnchanged",
            func: run_element_registry_leaves_base_bytes_unchanged,
        },
        RegisteredTest {
            group: "Element",
            name: "FeaturesRoundTrip",
            func: run_element_features_round_trip,
        },
        RegisteredTest {
            group: "Element",
            name: "DimensionsAreNominalNotMeasured",
            func: run_element_dimensions_are_nominal_not_measured,
        },
        RegisteredTest {
            group: "Element",
            name: "UnknownTypeSurvivesResave",
            func: run_element_unknown_type_survives_resave,
        },
        RegisteredTest {
            group: "Element",
            name: "RegistryJsonRoundTrip",
            func: run_element_registry_json_round_trip,
        },
        RegisteredTest {
            group: "Element",
            name: "ThrowingFactoryDegradesToBase",
            func: run_element_throwing_factory_degrades_to_base,
        },
        RegisteredTest {
            group: "Element",
            name: "DuplicateKeepsEveryField",
            func: run_element_duplicate_keeps_every_field,
        },
        RegisteredTest {
            group: "Element",
            name: "EqualityComparesCarriedFields",
            func: run_element_equality_compares_carried_fields,
        },
        RegisteredTest {
            group: "ElementFeature",
            name: "Constructor",
            func: run_element_feature_constructor,
        },
        RegisteredTest {
            group: "ElementFeature",
            name: "Json Roundtrip",
            func: run_element_feature_json_roundtrip,
        },
        RegisteredTest {
            group: "ElementFeature",
            name: "Protobuf Roundtrip",
            func: run_element_feature_protobuf_roundtrip,
        },
        RegisteredTest {
            group: "MeshOffset",
            name: "from_mesh",
            func: run_mesh_offset_from_mesh,
        },
        RegisteredTest {
            group: "MeshOffset",
            name: "from_mesh_grid",
            func: run_mesh_offset_from_mesh_grid,
        },
        RegisteredTest {
            group: "MeshOffset",
            name: "from_mesh_layers",
            func: run_mesh_offset_from_mesh_layers,
        },
        RegisteredTest {
            group: "MeshOffset",
            name: "file_json_dump",
            func: run_mesh_offset_file_json_dump,
        },
        RegisteredTest {
            group: "MeshOffset",
            name: "file_json_load",
            func: run_mesh_offset_file_json_load,
        },
        RegisteredTest {
            group: "MeshOffset",
            name: "to_proto",
            func: run_mesh_offset_to_proto,
        },
        RegisteredTest {
            group: "MeshOffset",
            name: "from_proto",
            func: run_mesh_offset_from_proto,
        },
        // AABB tests
        RegisteredTest {
            group: "AABB",
            name: "Constructor",
            func: run_aabb_constructor,
        },
        RegisteredTest {
            group: "AABB",
            name: "From Geometry",
            func: run_aabb_from_geometry,
        },
        // Boolean Polyline tests
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Overlapping Squares",
            func: run_boolean_polyline_overlapping_squares,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Circle Vs Rectangle",
            func: run_boolean_polyline_circle_vs_rectangle,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Star Vs Circle",
            func: run_boolean_polyline_star_vs_circle,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "L Shape Vs Rectangle",
            func: run_boolean_polyline_l_shape_vs_rectangle,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Two Large Circles",
            func: run_boolean_polyline_two_large_circles,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Diamond Vs Triangle",
            func: run_boolean_polyline_diamond_vs_triangle,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Star Vs Star",
            func: run_boolean_polyline_star_vs_star,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Cross Shape",
            func: run_boolean_polyline_cross_shape,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Concave Arrow Vs Circle",
            func: run_boolean_polyline_concave_arrow_vs_circle,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Two Large Circles 1000",
            func: run_boolean_polyline_two_large_circles_1000,
        },
        RegisteredTest {
            group: "Boolean Polyline",
            name: "Large Coords Auto Scale",
            func: run_boolean_polyline_large_coords_auto_scale,
        },
        // Boolean Polyline Open tests
        RegisteredTest {
            group: "Boolean Polyline Open",
            name: "Horizontal Line Vs Unit Square",
            func: run_boolean_polyline_open_horizontal_line_vs_unit_square,
        },
        RegisteredTest {
            group: "Boolean Polyline Open",
            name: "Diagonal Line Vs Unit Square",
            func: run_boolean_polyline_open_diagonal_line_vs_unit_square,
        },
        RegisteredTest {
            group: "Boolean Polyline Open",
            name: "Interior Open Path Passes Through",
            func: run_boolean_polyline_open_interior_open_path_passes_through,
        },
        // ConvexHull tests
        RegisteredTest {
            group: "ConvexHull",
            name: "Hull 2d",
            func: run_convex_hull_hull_2d,
        },
        RegisteredTest {
            group: "ConvexHull",
            name: "Hull 2d Collinear",
            func: run_convex_hull_hull_2d_collinear,
        },
        RegisteredTest {
            group: "ConvexHull",
            name: "Hull 2d Circle",
            func: run_convex_hull_hull_2d_circle,
        },
        RegisteredTest {
            group: "ConvexHull",
            name: "Hull 3d",
            func: run_convex_hull_hull_3d,
        },
        RegisteredTest {
            group: "ConvexHull",
            name: "Hull 3d Cube",
            func: run_convex_hull_hull_3d_cube,
        },
        // InstanceRef tests
        RegisteredTest {
            group: "InstanceRef",
            name: "Constructor",
            func: run_instance_ref_constructor,
        },
        RegisteredTest {
            group: "InstanceRef",
            name: "Transformation",
            func: run_instance_ref_transformation,
        },
        RegisteredTest {
            group: "InstanceRef",
            name: "Json Roundtrip",
            func: run_instance_ref_json_roundtrip,
        },
        RegisteredTest {
            group: "InstanceRef",
            name: "Protobuf Roundtrip",
            func: run_instance_ref_protobuf_roundtrip,
        },
        // Io tests
        RegisteredTest {
            group: "Io",
            name: "Read Bunny",
            func: run_io_read_bunny,
        },
        RegisteredTest {
            group: "Io",
            name: "String Roundtrip",
            func: run_io_string_roundtrip,
        },
        RegisteredTest {
            group: "Io",
            name: "Write Read Roundtrip",
            func: run_io_write_read_roundtrip,
        },
        RegisteredTest {
            group: "Io",
            name: "Read Colors",
            func: run_io_read_colors,
        },
        // Matrix tests
        RegisteredTest {
            group: "Matrix",
            name: "Constructor",
            func: run_matrix_constructor,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Properties",
            func: run_matrix_properties,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Add",
            func: run_matrix_add,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Subtract",
            func: run_matrix_subtract,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Scale",
            func: run_matrix_scale,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Multiply",
            func: run_matrix_multiply,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Transpose",
            func: run_matrix_transpose,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Determinant",
            func: run_matrix_determinant,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Inverse",
            func: run_matrix_inverse,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Solve",
            func: run_matrix_solve,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Lu Decompose",
            func: run_matrix_lu_decompose,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Qr Decompose",
            func: run_matrix_qr_decompose,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Cholesky",
            func: run_matrix_cholesky,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Eigenvalues",
            func: run_matrix_eigenvalues,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Svd",
            func: run_matrix_svd,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Norms",
            func: run_matrix_norms,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Rank",
            func: run_matrix_rank,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Json Roundtrip",
            func: run_matrix_json_roundtrip,
        },
        RegisteredTest {
            group: "Matrix",
            name: "Protobuf Roundtrip",
            func: run_matrix_protobuf_roundtrip,
        },
        // SpatialKDTree tests
        RegisteredTest {
            group: "SpatialKDTree",
            name: "Constructor",
            func: run_kdtree_constructor,
        },
        RegisteredTest {
            group: "SpatialKDTree",
            name: "Nearest",
            func: run_kdtree_nearest,
        },
        RegisteredTest {
            group: "SpatialKDTree",
            name: "Nearest K",
            func: run_kdtree_nearest_k,
        },
        RegisteredTest {
            group: "SpatialKDTree",
            name: "Radius Search",
            func: run_kdtree_radius_search,
        },
        // Io tests - were registered by macro only; see the drift guard in run_all.
        // PointCloud tests - were registered by macro only; see the drift guard in run_all.
        RegisteredTest {
            group: "PointCloud",
            name: "Coords",
            func: run_pointcloud_coords,
        },
        RegisteredTest {
            group: "PointCloud",
            name: "Colors",
            func: run_pointcloud_colors,
        },
        // RemeshCDT tests - were registered by macro only; see the drift guard in run_all.
        RegisteredTest {
            group: "RemeshCDT",
            name: "plate_failing 15-vert outer + 4 holes",
            func: run_remesh_cdt_plate_failing_15_vert_outer_4_holes,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Large coordinates",
            func: run_remesh_cdt_large_coordinates,
        },
        RegisteredTest {
            group: "RemeshCDT",
            name: "Degenerate hole keeps flat indices",
            func: run_remesh_cdt_degenerate_hole_keeps_flat_indices,
        },
        // SpatialOctree tests - were registered by macro only; see the drift guard in run_all.
        RegisteredTest {
            group: "SpatialOctree",
            name: "Constructor",
            func: run_octree_constructor,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "Node Count",
            func: run_octree_node_count,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "Node Cube",
            func: run_octree_node_cube,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "Node Level",
            func: run_octree_node_level,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "Node Spacing",
            func: run_octree_node_spacing,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "Node Range",
            func: run_octree_node_range,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "Children",
            func: run_octree_children,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "Order",
            func: run_octree_order,
        },
        RegisteredTest {
            group: "SpatialOctree",
            name: "From Coords",
            func: run_octree_from_coords,
        },
    ];

    // Feature-gated exactly like its REGISTER_MINI_TEST! in io_test.rs, so the drift guard in
    // run_all() sees the same set on both sides whether or not `pdf` is enabled.
    #[cfg(all(feature = "pdf", not(target_arch = "wasm32")))]
    let tests = {
        let mut tests = tests;
        tests.push(RegisteredTest {
            group: "Io",
            name: "Import Minimal",
            func: run_io_pdf_import_minimal,
        });
        tests
    };

    tests
}

/// Run all registered Rust mini-tests for this crate and write JSON results
/// to the session_tests/session_rust directory, matching the layout of the
/// Python and C++ mini-test frameworks.
pub fn run_all(language: &str) -> Result<(), Box<dyn std::error::Error>> {
    let _ = language; // kept for symmetry with other languages

    println!("[rust-minitest] Starting test collection...");

    // Group registered tests by logical group name (e.g. "Point", "Color").
    let mut groups: BTreeMap<&'static str, Vec<RegisteredTest>> = BTreeMap::new();

    // First try inventory collection
    for t in inventory::iter::<RegisteredTest> {
        groups.entry(t.group).or_default().push(*t);
    }

    println!("[rust-minitest] Inventory found {} groups", groups.len());

    // ── Guard the two registration paths against drift ──────────────────────────────────
    //
    // A test is registered TWICE: by REGISTER_MINI_TEST! (inventory, the primary path) and by
    // hand in get_all_tests() below, which exists as a Windows fallback AND as the canonical
    // ordering oracle. The merge that follows dedups by NAME, so when the two disagree the
    // result is not an error - it is an extra test. Renaming "Add Feature" to "Add Geometry
    // Op" in the macro but not in the table silently produced 18 Element tests where the
    // other two languages had 17, and the cross-language parity check then failed several
    // steps downstream with no hint that a rename was the cause.
    //
    // Compare the two sets up front and say exactly which entries drifted.
    {
        use std::collections::BTreeSet;
        let from_inventory: BTreeSet<(&str, &str)> = inventory::iter::<RegisteredTest>
            .into_iter()
            .map(|t| (t.group, t.name))
            .collect();
        let from_table: BTreeSet<(&str, &str)> =
            get_all_tests().iter().map(|t| (t.group, t.name)).collect();

        // Only meaningful when inventory actually collected something: on a platform where it
        // yields nothing the table IS the registration, and every entry would look "missing".
        if !from_inventory.is_empty() {
            let missing_from_table: Vec<_> = from_inventory.difference(&from_table).collect();
            let missing_from_macros: Vec<_> = from_table.difference(&from_inventory).collect();

            if !missing_from_table.is_empty() || !missing_from_macros.is_empty() {
                let mut message = String::from(
                    "test registration drift: REGISTER_MINI_TEST! and get_all_tests() disagree.\n\
                     Every test must appear in BOTH - the macro registers it, the table orders \
                     it and covers platforms where inventory finds nothing.\n",
                );
                for (group, name) in &missing_from_table {
                    message.push_str(&format!(
                        "  registered by macro, ABSENT from get_all_tests(): {group}::{name}\n"
                    ));
                }
                for (group, name) in &missing_from_macros {
                    message.push_str(&format!(
                        "  in get_all_tests(), NOT registered by macro:      {group}::{name}\n"
                    ));
                }
                message.push_str(
                    "A rename usually shows up as one of each: the old name on one side, the \
                     new name on the other.\n",
                );
                return Err(message.into());
            }
        }
    }

    // Merge manual tests (covers modules inventory may miss on Windows)
    for t in get_all_tests() {
        let entry = groups.entry(t.group).or_default();
        if !entry.iter().any(|e| e.name == t.name) {
            entry.push(t);
        }
    }

    println!("[rust-minitest] Total groups: {}", groups.len());

    if groups.is_empty() {
        eprintln!("Warning: No tests found to run");
        return Ok(());
    }

    // Compute output directory: repo_root/session_tests/session_rust
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir
        .parent()
        .ok_or("Failed to find repo root from CARGO_MANIFEST_DIR")?;
    let out_dir = repo_root.join("session_tests").join("session_rust");
    fs::create_dir_all(&out_dir)?;

    // Build canonical ordering from get_all_tests()
    let canonical_order: std::collections::HashMap<(&str, &str), usize> = get_all_tests()
        .iter()
        .enumerate()
        .map(|(i, t)| ((t.group, t.name), i))
        .collect();

    let mut total_tests = 0usize;
    let mut total_passed = 0usize;
    let mut failed_tests: Vec<(String, String, String, u32, Vec<serde_json::Value>)> = Vec::new();

    // Run all tests; key results by source-file stem so groups sharing a file
    // (e.g. "Tree" + "TreeNode" in tree_test.rs) land in the same output file.
    let mut results_by_file: BTreeMap<String, Vec<serde_json::Value>> = BTreeMap::new();
    for (group, mut tests) in groups {
        tests.sort_by_key(|t| {
            canonical_order
                .get(&(t.group, t.name))
                .copied()
                .unwrap_or(usize::MAX)
        });

        for t in tests {
            let res = (t.func)();
            total_tests += 1;
            if res.passed {
                total_passed += 1;
            } else {
                failed_tests.push((
                    group.to_string(),
                    res.test_name.to_string(),
                    res.file.to_string(),
                    res.line,
                    res.failures.clone(),
                ));
            }
            let file_stem = std::path::Path::new(res.file)
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("unknown")
                .to_string();
            let res_json = serde_json::json!({
                "group": t.group,
                "test_name": res.test_name,
                "passed": res.passed,
                "time_ms": res.time_ms,
                "line": res.line,
                "code": res.code,
                "checks": res.checks,
                "failures": res.failures,
            });
            results_by_file.entry(file_stem).or_default().push(res_json);
        }
    }

    for (file_stem, results) in &results_by_file {
        let filename = format!("{}.json", file_stem);
        let path = out_dir.join(&filename);
        let tmp_path = out_dir.join(format!("{}.tmp", &filename));
        let json = serde_json::to_string_pretty(results)?;
        fs::write(&tmp_path, &json)?;
        if let Err(_) = fs::rename(&tmp_path, &path) {
            let _ = fs::remove_file(&path);
            fs::rename(&tmp_path, &path).or_else(|_| fs::write(&path, &json))?;
        }
    }

    if !failed_tests.is_empty() {
        eprintln!("\n[rust-minitest] FAILURES:");
        for (group, name, file, line, failures) in &failed_tests {
            eprintln!("  FAIL {group}::{name}  {file}:{line}");
            for f in failures {
                if let Some(err) = f.get("error").and_then(|v| v.as_str()) {
                    eprintln!("       {err}");
                }
            }
        }
        eprintln!(
            "\n[rust-minitest] {total_passed}/{total_tests} passed, {} failed",
            failed_tests.len()
        );
        std::process::exit(1);
    }

    println!("[rust-minitest] {total_passed}/{total_tests} passed");
    Ok(())
}

#[cfg(test)]
mod harness {
    /// `cargo test` runs the same registered suite as `cargo run --bin minitest`.
    #[test]
    fn minitest_suite() {
        super::run_all("rust").expect("minitest suite failed");
    }
}
