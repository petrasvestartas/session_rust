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
        c.borrow_mut().push(CheckRecord { line, code_line, passed });
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
    // End a couple lines after the last check (for closing braces)
    let end_line = last_check_line + 2;

    let src = match fs::read_to_string(&path) {
        Ok(s) => s,
        Err(_) => return String::new(),
    };

    let mut code_lines = Vec::new();
    let mut in_check = false;
    let mut paren_depth = 0i32;

    for (idx, line) in src.lines().enumerate() {
        let line_no = (idx as u32) + 1;
        if line_no < start_line {
            continue;
        }
        if line_no > end_line {
            break;
        }

        // Check if this line starts a MINI_CHECK!
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

        // If we're in a multi-line MINI_CHECK, skip and track parentheses
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

        // Skip closing brace only lines
        let trimmed = line.trim();
        if trimmed == "}" || trimmed == "})" || trimmed == "});" {
            continue;
        }
        code_lines.push(line);
    }

    code_lines.join("\n")
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
    use crate::color_test::*;
    use crate::point_test::*;
    use crate::vector_test::*;
    use crate::tolerance_test::*;
    use crate::line_test::*;
    use crate::polyline_test::*;
    use crate::plane_test::*;
    use crate::pointcloud_test::*;
    use crate::xform_test::*;
    use crate::mesh_test::*;
    use crate::nurbscurve_test::*;
    use crate::nurbssurface_test::*;
    use crate::brep_test::*;
    use crate::closest_test::*;
    use crate::intersection_test::*;
    use crate::obj_test::*;
    use crate::session_test::*;
    use crate::triangulation_2d_test::*;
    use crate::bvh_test::*;
    use crate::quaternion_test::*;
    use crate::boundingbox_test::*;
    use crate::edge_test::*;
    use crate::graph_test::*;
    use crate::objects_test::*;
    use crate::tree_test::*;
    use crate::treenode_test::*;
    use crate::vertex_test::*;
    use crate::encoders_test::*;
    use crate::aabb_test::*;
    use crate::primitives_test::*;
    use crate::knot_test::*;
    use crate::trimmedsurface_test::*;
    use crate::mesh_iso_test::*;
    use crate::rtree_test::*;
    use crate::session_config_test::*;

    vec![
        // BRep tests
        RegisteredTest { group: "BRep", name: "Constructor", func: run_brep_constructor },
        RegisteredTest { group: "BRep", name: "Create Box", func: run_brep_create_box },
        RegisteredTest { group: "BRep", name: "Accessors", func: run_brep_accessors },
        RegisteredTest { group: "BRep", name: "Add Face", func: run_brep_add_face },
        RegisteredTest { group: "BRep", name: "Mesh", func: run_brep_mesh },
        RegisteredTest { group: "BRep", name: "Point At", func: run_brep_point_at },
        RegisteredTest { group: "BRep", name: "Is Solid", func: run_brep_is_solid },
        RegisteredTest { group: "BRep", name: "Transformation", func: run_brep_transformation },
        RegisteredTest { group: "BRep", name: "Json Roundtrip", func: run_brep_json_roundtrip },
        RegisteredTest { group: "BRep", name: "Create Cylinder", func: run_brep_create_cylinder },
        RegisteredTest { group: "BRep", name: "Create Sphere", func: run_brep_create_sphere },
        RegisteredTest { group: "BRep", name: "From Polylines", func: run_brep_from_polylines },
        RegisteredTest { group: "BRep", name: "From Nurbscurves", func: run_brep_from_nurbscurves },
        RegisteredTest { group: "BRep", name: "From Nurbscurves Holes", func: run_brep_from_nurbscurves_holes },
        RegisteredTest { group: "BRep", name: "Protobuf Roundtrip", func: run_brep_protobuf_roundtrip },
        // Color tests
        RegisteredTest { group: "Color", name: "Constructor", func: run_color_constructor },
        RegisteredTest { group: "Color", name: "Json Roundtrip", func: run_color_json_roundtrip },
        RegisteredTest { group: "Color", name: "Protobuf Roundtrip", func: run_color_protobuf_roundtrip },
        RegisteredTest { group: "Color", name: "Conversion", func: run_color_conversion },
        RegisteredTest { group: "Color", name: "Presets", func: run_color_presets },
        // Point tests
        RegisteredTest { group: "Point", name: "Constructor", func: run_point_constructor },
        RegisteredTest { group: "Point", name: "Transformation", func: run_point_transformation },
        RegisteredTest { group: "Point", name: "Json Roundtrip", func: run_point_json_roundtrip },
        RegisteredTest { group: "Point", name: "Protobuf Roundtrip", func: run_point_protobuf_roundtrip },
        RegisteredTest { group: "Point", name: "Is Ccw", func: run_point_is_ccw },
        RegisteredTest { group: "Point", name: "Mid Point", func: run_point_mid_point },
        RegisteredTest { group: "Point", name: "Distance", func: run_point_distance },
        RegisteredTest { group: "Point", name: "Squared Distance", func: run_point_squared_distance },
        RegisteredTest { group: "Point", name: "Area", func: run_point_area },
        RegisteredTest { group: "Point", name: "Centroid Quad", func: run_point_centroid_quad },
        // Vector tests
        RegisteredTest { group: "Vector", name: "Constructor", func: run_vector_constructor },
        RegisteredTest { group: "Vector", name: "Magnitude", func: run_vector_magnitude },
        RegisteredTest { group: "Vector", name: "Normalize", func: run_vector_normalize },
        RegisteredTest { group: "Vector", name: "Reverse", func: run_vector_reverse },
        RegisteredTest { group: "Vector", name: "Dot Product", func: run_vector_dot_product },
        RegisteredTest { group: "Vector", name: "Cross Product", func: run_vector_cross_product },
        RegisteredTest { group: "Vector", name: "Angle", func: run_vector_angle },
        RegisteredTest { group: "Vector", name: "Projection", func: run_vector_projection },
        RegisteredTest { group: "Vector", name: "Is Parallel To", func: run_vector_is_parallel_to },
        RegisteredTest { group: "Vector", name: "Is Perpendicular To", func: run_vector_is_perpendicular_to },
        RegisteredTest { group: "Vector", name: "Get Leveled Vector", func: run_vector_get_leveled_vector },
        RegisteredTest { group: "Vector", name: "Cos Sin Laws", func: run_vector_cos_sin_laws },
        RegisteredTest { group: "Vector", name: "Sum Of Vectors", func: run_vector_sum_of_vectors },
        RegisteredTest { group: "Vector", name: "Average", func: run_vector_average },
        RegisteredTest { group: "Vector", name: "Is Zero", func: run_vector_is_zero },
        RegisteredTest { group: "Vector", name: "Json Roundtrip", func: run_vector_json_roundtrip },
        RegisteredTest { group: "Vector", name: "Protobuf Roundtrip", func: run_vector_protobuf_roundtrip },
        // Tolerance tests
        RegisteredTest { group: "Tolerance", name: "Is Zero", func: run_tolerance_is_zero },
        RegisteredTest { group: "Tolerance", name: "Is Close", func: run_tolerance_is_close },
        RegisteredTest { group: "Tolerance", name: "Is Positive", func: run_tolerance_is_positive },
        RegisteredTest { group: "Tolerance", name: "Is Negative", func: run_tolerance_is_negative },
        RegisteredTest { group: "Tolerance", name: "Is Between", func: run_tolerance_is_between },
        RegisteredTest { group: "Tolerance", name: "Format Number", func: run_tolerance_format_number },
        RegisteredTest { group: "Tolerance", name: "Key", func: run_tolerance_key },
        RegisteredTest { group: "Tolerance", name: "Runtime Modification", func: run_tolerance_runtime_modification },
        // Line tests
        RegisteredTest { group: "Line", name: "Constructor", func: run_line_constructor },
        RegisteredTest { group: "Line", name: "Transformation", func: run_line_transformation },
        RegisteredTest { group: "Line", name: "Json Roundtrip", func: run_line_json_roundtrip },
        RegisteredTest { group: "Line", name: "Protobuf Roundtrip", func: run_line_protobuf_roundtrip },
        RegisteredTest { group: "Line", name: "Length", func: run_line_length },
        RegisteredTest { group: "Line", name: "To Vector", func: run_line_to_vector },
        RegisteredTest { group: "Line", name: "To Direction", func: run_line_to_direction },
        RegisteredTest { group: "Line", name: "Point At", func: run_line_point_at },
        RegisteredTest { group: "Line", name: "Closest Point", func: run_line_closest_point },
        RegisteredTest { group: "Line", name: "Start End Center", func: run_line_start_end_center },
        RegisteredTest { group: "Line", name: "Fit Points", func: run_line_fit_points },
        RegisteredTest { group: "Line", name: "Subdivide", func: run_line_subdivide },
        // Polyline tests
        RegisteredTest { group: "Polyline", name: "Constructor", func: run_polyline_constructor },
        RegisteredTest { group: "Polyline", name: "Transformation", func: run_polyline_transformation },
        RegisteredTest { group: "Polyline", name: "Json Roundtrip", func: run_polyline_json_roundtrip },
        RegisteredTest { group: "Polyline", name: "Protobuf Roundtrip", func: run_polyline_protobuf_roundtrip },
        RegisteredTest { group: "Polyline", name: "Length", func: run_polyline_length },
        RegisteredTest { group: "Polyline", name: "Center", func: run_polyline_center },
        RegisteredTest { group: "Polyline", name: "Is Closed", func: run_polyline_is_closed },
        RegisteredTest { group: "Polyline", name: "Reverse", func: run_polyline_reverse },
        RegisteredTest { group: "Polyline", name: "Closest Point", func: run_polyline_closest_point },
        RegisteredTest { group: "Polyline", name: "Extend Segment", func: run_polyline_extend_segment },
        RegisteredTest { group: "Polyline", name: "Extend Segment Equally", func: run_polyline_extend_segment_equally },
        RegisteredTest { group: "Polyline", name: "Get Points", func: run_polyline_get_points },
        RegisteredTest { group: "Polyline", name: "Shift", func: run_polyline_shift },
        RegisteredTest { group: "Polyline", name: "Point At", func: run_polyline_point_at },
        RegisteredTest { group: "Polyline", name: "Is Clockwise", func: run_polyline_is_clockwise },
        RegisteredTest { group: "Polyline", name: "Convex Corners", func: run_polyline_convex_corners },
        RegisteredTest { group: "Polyline", name: "Tween", func: run_polyline_tween },
        RegisteredTest { group: "Polyline", name: "Average Plane", func: run_polyline_average_plane },
        // Plane tests
        RegisteredTest { group: "Plane", name: "Constructor", func: run_plane_constructor },
        RegisteredTest { group: "Plane", name: "Reverse", func: run_plane_reverse },
        RegisteredTest { group: "Plane", name: "Rotate", func: run_plane_rotate },
        RegisteredTest { group: "Plane", name: "Is Right Hand", func: run_plane_is_right_hand },
        RegisteredTest { group: "Plane", name: "Is Coplanar", func: run_plane_is_coplanar },
        RegisteredTest { group: "Plane", name: "Transform", func: run_plane_transform },
        RegisteredTest { group: "Plane", name: "Json Roundtrip", func: run_plane_json_roundtrip },
        RegisteredTest { group: "Plane", name: "Protobuf Roundtrip", func: run_plane_protobuf_roundtrip },
        // PointCloud tests
        RegisteredTest { group: "PointCloud", name: "Constructor", func: run_pointcloud_constructor },
        RegisteredTest { group: "PointCloud", name: "Transform", func: run_pointcloud_transform },
        RegisteredTest { group: "PointCloud", name: "Json Roundtrip", func: run_pointcloud_json_roundtrip },
        RegisteredTest { group: "PointCloud", name: "Protobuf Roundtrip", func: run_pointcloud_protobuf_roundtrip },
        // Xform tests
        RegisteredTest { group: "Xform", name: "Constructor", func: run_xform_constructor },
        RegisteredTest { group: "Xform", name: "Translation", func: run_xform_translation },
        RegisteredTest { group: "Xform", name: "Scaling", func: run_xform_scaling },
        RegisteredTest { group: "Xform", name: "Rotation", func: run_xform_rotation },
        RegisteredTest { group: "Xform", name: "Inverse", func: run_xform_inverse },
        RegisteredTest { group: "Xform", name: "Transform Geometry", func: run_xform_transform_geometry },
        RegisteredTest { group: "Xform", name: "Change Basis", func: run_xform_change_basis },
        RegisteredTest { group: "Xform", name: "Plane To Plane", func: run_xform_plane_to_plane },
        RegisteredTest { group: "Xform", name: "Look At Rh", func: run_xform_look_at_rh },
        RegisteredTest { group: "Xform", name: "Json Roundtrip", func: run_xform_json_roundtrip },
        RegisteredTest { group: "Xform", name: "Protobuf Roundtrip", func: run_xform_protobuf_roundtrip },
        // Mesh tests
        RegisteredTest { group: "Mesh", name: "Constructor", func: run_mesh_constructor },
        RegisteredTest { group: "Mesh", name: "From Polylines", func: run_mesh_from_polylines },
        RegisteredTest { group: "Mesh", name: "From Lines", func: run_mesh_from_lines },
        RegisteredTest { group: "Mesh", name: "From Polygon With Holes", func: run_mesh_from_polygon_with_holes },
        RegisteredTest { group: "Mesh", name: "Loft", func: run_mesh_loft },
        RegisteredTest { group: "Mesh", name: "From Polygon With Holes Many", func: run_mesh_from_polygon_with_holes_many },
        RegisteredTest { group: "Mesh", name: "Loft Many", func: run_mesh_loft_many },
        RegisteredTest { group: "Mesh", name: "Boolean Queries", func: run_mesh_boolean_queries },
        RegisteredTest { group: "Mesh", name: "Attributes", func: run_mesh_attributes },
        RegisteredTest { group: "Mesh", name: "Vertex and Face Operations", func: run_mesh_vertex_and_face_operations },
        RegisteredTest { group: "Mesh", name: "Connectivity Queries", func: run_mesh_connectivity_queries },
        RegisteredTest { group: "Mesh", name: "Geometric Properties", func: run_mesh_geometric_properties },
        RegisteredTest { group: "Mesh", name: "Transformation", func: run_mesh_transformation },
        RegisteredTest { group: "Mesh", name: "Json Roundtrip", func: run_mesh_json_roundtrip },
        RegisteredTest { group: "Mesh", name: "Protobuf Roundtrip", func: run_mesh_protobuf_roundtrip },
        // NurbsCurve tests
        RegisteredTest { group: "NurbsCurve", name: "Constructor", func: run_nurbscurve_constructor },
        RegisteredTest { group: "NurbsCurve", name: "Attributes", func: run_nurbscurve_attributes },
        RegisteredTest { group: "NurbsCurve", name: "Conversions", func: run_nurbscurve_conversions },
        RegisteredTest { group: "NurbsCurve", name: "Evaluation", func: run_nurbscurve_evaluation },
        RegisteredTest { group: "NurbsCurve", name: "Modifications", func: run_nurbscurve_modifications },
        RegisteredTest { group: "NurbsCurve", name: "Json Roundtrip", func: run_nurbscurve_json_roundtrip },
        RegisteredTest { group: "NurbsCurve", name: "Protobuf Roundtrip", func: run_nurbscurve_protobuf_roundtrip },
        RegisteredTest { group: "NurbsCurve", name: "Transformations", func: run_nurbscurve_transformations },
        RegisteredTest { group: "NurbsCurve", name: "Create Interpolated", func: run_nurbscurve_create_interpolated },
        RegisteredTest { group: "NurbsCurve", name: "Create Fitted", func: run_nurbscurve_create_fitted },
        // NurbsSurface tests
        RegisteredTest { group: "NurbsSurface", name: "Constructor", func: run_nurbssurface_constructor },
        RegisteredTest { group: "NurbsSurface", name: "Booleans Queries", func: run_nurbssurface_booleans_queries },
        RegisteredTest { group: "NurbsSurface", name: "Attributes", func: run_nurbssurface_attributes },
        RegisteredTest { group: "NurbsSurface", name: "Control Vertices Access", func: run_nurbssurface_control_vertices_access },
        RegisteredTest { group: "NurbsSurface", name: "Knot Access", func: run_nurbssurface_knot_access },
        RegisteredTest { group: "NurbsSurface", name: "Domain", func: run_nurbssurface_domain },
        RegisteredTest { group: "NurbsSurface", name: "Division", func: run_nurbssurface_division },
        RegisteredTest { group: "NurbsSurface", name: "Evaluation", func: run_nurbssurface_evaluation },
        RegisteredTest { group: "NurbsSurface", name: "Modification", func: run_nurbssurface_modification },
        RegisteredTest { group: "NurbsSurface", name: "Transformations", func: run_nurbssurface_transformations },
        RegisteredTest { group: "NurbsSurface", name: "Meshing", func: run_nurbssurface_meshing },
        RegisteredTest { group: "NurbsSurface", name: "Json Roundtrip", func: run_nurbssurface_json_roundtrip },
        RegisteredTest { group: "NurbsSurface", name: "Protobuf Roundtrip", func: run_nurbssurface_protobuf_roundtrip },
        // Knot tests
        RegisteredTest { group: "Knot", name: "Knot Count", func: run_knot_count },
        RegisteredTest { group: "Knot", name: "Make Clamped Uniform", func: run_make_clamped_uniform },
        RegisteredTest { group: "Knot", name: "Make Periodic Uniform", func: run_make_periodic_uniform },
        RegisteredTest { group: "Knot", name: "Clamp", func: run_clamp },
        RegisteredTest { group: "Knot", name: "Is Valid", func: run_is_valid },
        RegisteredTest { group: "Knot", name: "Is Clamped", func: run_is_clamped },
        RegisteredTest { group: "Knot", name: "Is Periodic", func: run_is_periodic },
        RegisteredTest { group: "Knot", name: "Get Domain", func: run_get_domain },
        RegisteredTest { group: "Knot", name: "Set Domain", func: run_set_domain },
        RegisteredTest { group: "Knot", name: "Reverse", func: run_reverse },
        RegisteredTest { group: "Knot", name: "Multiplicity", func: run_multiplicity },
        RegisteredTest { group: "Knot", name: "Span Count", func: run_span_count },
        RegisteredTest { group: "Knot", name: "Find Span", func: run_find_span },
        RegisteredTest { group: "Knot", name: "Greville Abcissae", func: run_greville_abcissae },
        RegisteredTest { group: "Knot", name: "Domain Tolerance", func: run_domain_tolerance },
        // TrimmedSurface tests
        RegisteredTest { group: "TrimmedSurface", name: "Constructor", func: run_trimmedsurface_constructor },
        RegisteredTest { group: "TrimmedSurface", name: "Constructor Planar", func: run_trimmedsurface_constructor_planar },
        RegisteredTest { group: "TrimmedSurface", name: "Constructor Hole", func: run_trimmedsurface_constructor_hole },
        RegisteredTest { group: "TrimmedSurface", name: "Accessors", func: run_trimmedsurface_accessors },
        RegisteredTest { group: "TrimmedSurface", name: "Add Inner Loop", func: run_trimmedsurface_add_inner_loop },
        RegisteredTest { group: "TrimmedSurface", name: "Point At", func: run_trimmedsurface_point_at },
        RegisteredTest { group: "TrimmedSurface", name: "Mesh", func: run_trimmedsurface_mesh },
        RegisteredTest { group: "TrimmedSurface", name: "Transformation", func: run_trimmedsurface_transformation },
        RegisteredTest { group: "TrimmedSurface", name: "Json Roundtrip", func: run_trimmedsurface_json_roundtrip },
        RegisteredTest { group: "TrimmedSurface", name: "Protobuf Roundtrip", func: run_trimmedsurface_protobuf_roundtrip },
        // Closest tests
        RegisteredTest { group: "Closest", name: "Line Point", func: run_closest_line_point },
        RegisteredTest { group: "Closest", name: "Polyline Point", func: run_closest_polyline_point },
        RegisteredTest { group: "Closest", name: "Curve Point", func: run_closest_curve_point },
        RegisteredTest { group: "Closest", name: "Surface Point", func: run_closest_surface_point },
        RegisteredTest { group: "Closest", name: "Mesh Point", func: run_closest_mesh_point },
        RegisteredTest { group: "Closest", name: "Mesh Point AABB", func: run_closest_mesh_point_aabb },
        RegisteredTest { group: "Closest", name: "Pointcloud Point", func: run_closest_pointcloud_point },
        // Primitives tests
        RegisteredTest { group: "Primitives", name: "Mesh Arrow", func: run_primitives_mesh_arrow },
        RegisteredTest { group: "Primitives", name: "Mesh Cylinder", func: run_primitives_mesh_cylinder },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Polyline", func: run_primitives_nurbscurve_polyline },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Circle", func: run_primitives_nurbscurve_circle },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Ellipse", func: run_primitives_nurbscurve_ellipse },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Arc", func: run_primitives_nurbscurve_arc },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Parabola", func: run_primitives_nurbscurve_parabola },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Hyperbola", func: run_primitives_nurbscurve_hyperbola },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Spiral", func: run_primitives_nurbscurve_spiral },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Cylinder", func: run_primitives_nurbssurface_cylinder },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Cone", func: run_primitives_nurbssurface_cone },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Sphere", func: run_primitives_nurbssurface_sphere },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Quad Sphere", func: run_primitives_nurbssurface_quad_sphere },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Torus", func: run_primitives_nurbssurface_torus },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Ruled", func: run_primitives_nurbssurface_ruled },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Planar", func: run_primitives_nurbssurface_planar },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Extrusion", func: run_primitives_nurbssurface_extrusion },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Loft", func: run_primitives_nurbssurface_loft },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Revolve", func: run_primitives_nurbssurface_revolve },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Sweep", func: run_primitives_nurbssurface_sweep },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Edge", func: run_primitives_nurbssurface_edge },
        RegisteredTest { group: "Primitives", name: "Mesh Quad Mesh", func: run_primitives_mesh_quad_mesh },
        RegisteredTest { group: "Primitives", name: "Mesh Diamond Mesh", func: run_primitives_mesh_diamond_mesh },
        RegisteredTest { group: "Primitives", name: "Mesh Hex Mesh", func: run_primitives_mesh_hex_mesh },
        RegisteredTest { group: "Primitives", name: "Mesh Cone Subdivisions", func: run_primitives_mesh_cone_subdivisions },
        RegisteredTest { group: "Primitives", name: "Nurbscurve Interpolated", func: run_primitives_nurbscurve_interpolated },
        RegisteredTest { group: "Primitives", name: "Mesh Tetrahedron", func: run_primitives_mesh_tetrahedron },
        RegisteredTest { group: "Primitives", name: "Mesh Cube", func: run_primitives_mesh_cube },
        RegisteredTest { group: "Primitives", name: "Mesh Octahedron", func: run_primitives_mesh_octahedron },
        RegisteredTest { group: "Primitives", name: "Mesh Icosahedron", func: run_primitives_mesh_icosahedron },
        RegisteredTest { group: "Primitives", name: "Mesh Dodecahedron", func: run_primitives_mesh_dodecahedron },
        RegisteredTest { group: "Primitives", name: "Nurbssurface Wave", func: run_primitives_nurbssurface_wave },
        // Intersection tests
        RegisteredTest { group: "Intersection", name: "Line Line", func: run_intersection_line_line },
        RegisteredTest { group: "Intersection", name: "Line Line Parallel", func: run_intersection_line_line_parallel },
        RegisteredTest { group: "Intersection", name: "Line Line Parameters", func: run_intersection_line_line_parameters },
        RegisteredTest { group: "Intersection", name: "Line Line Parameters Endpoints", func: run_intersection_line_line_parameters_endpoints },
        RegisteredTest { group: "Intersection", name: "Line Line Parameters Infinite", func: run_intersection_line_line_parameters_infinite },
        RegisteredTest { group: "Intersection", name: "Plane Plane", func: run_intersection_plane_plane },
        RegisteredTest { group: "Intersection", name: "Plane Plane Complex", func: run_intersection_plane_plane_complex },
        RegisteredTest { group: "Intersection", name: "Line Plane", func: run_intersection_line_plane },
        RegisteredTest { group: "Intersection", name: "Line Plane Parallel", func: run_intersection_line_plane_parallel },
        RegisteredTest { group: "Intersection", name: "Line Plane Real World", func: run_intersection_line_plane_real_world },
        RegisteredTest { group: "Intersection", name: "Plane Plane Plane", func: run_intersection_plane_plane_plane },
        RegisteredTest { group: "Intersection", name: "Plane Plane Plane Parallel", func: run_intersection_plane_plane_plane_parallel },
        RegisteredTest { group: "Intersection", name: "Ray Box", func: run_intersection_ray_box },
        RegisteredTest { group: "Intersection", name: "Ray Box Miss", func: run_intersection_ray_box_miss },
        RegisteredTest { group: "Intersection", name: "Ray Sphere", func: run_intersection_ray_sphere },
        RegisteredTest { group: "Intersection", name: "Ray Sphere Tangent", func: run_intersection_ray_sphere_tangent },
        RegisteredTest { group: "Intersection", name: "Ray Sphere Miss", func: run_intersection_ray_sphere_miss },
        RegisteredTest { group: "Intersection", name: "Ray Triangle", func: run_intersection_ray_triangle },
        RegisteredTest { group: "Intersection", name: "Ray Triangle Miss", func: run_intersection_ray_triangle_miss },
        RegisteredTest { group: "Intersection", name: "Ray Triangle Parallel", func: run_intersection_ray_triangle_parallel },
        RegisteredTest { group: "Intersection", name: "Ray Mesh", func: run_intersection_ray_mesh },
        RegisteredTest { group: "Intersection", name: "Ray Mesh First", func: run_intersection_ray_mesh_first },
        RegisteredTest { group: "Intersection", name: "Ray Mesh Miss", func: run_intersection_ray_mesh_miss },
        RegisteredTest { group: "Intersection", name: "Ray Mesh Bvh", func: run_intersection_ray_mesh_bvh },
        RegisteredTest { group: "Intersection", name: "Ray Mesh Bvh First", func: run_intersection_ray_mesh_bvh_first },
        RegisteredTest { group: "Intersection", name: "Ray Mesh Bvh Miss", func: run_intersection_ray_mesh_bvh_miss },
        RegisteredTest { group: "Intersection", name: "Ray Mesh Bvh Vs Naive", func: run_intersection_ray_mesh_bvh_vs_naive },
        RegisteredTest { group: "Intersection", name: "Ray Box Real World", func: run_intersection_ray_box_real_world },
        RegisteredTest { group: "Intersection", name: "Ray Sphere Real World", func: run_intersection_ray_sphere_real_world },
        RegisteredTest { group: "Intersection", name: "Ray Triangle Real World", func: run_intersection_ray_triangle_real_world },
        RegisteredTest { group: "Intersection", name: "Surface Plane", func: run_intersection_surface_plane },
        RegisteredTest { group: "Intersection", name: "Surface Plane Curved", func: run_intersection_surface_plane_curved },
        RegisteredTest { group: "Intersection", name: "Surface Plane Miss", func: run_intersection_surface_plane_miss },
        // Session tests
        RegisteredTest { group: "Session", name: "Constructor", func: run_session_constructor },
        RegisteredTest { group: "Session", name: "Jsondump", func: run_session_jsondump },
        RegisteredTest { group: "Session", name: "Jsonload", func: run_session_jsonload },
        RegisteredTest { group: "Session", name: "File Io", func: run_session_file_io },
        RegisteredTest { group: "Session", name: "Add Point", func: run_session_add_point },
        RegisteredTest { group: "Session", name: "Add Edge", func: run_session_add_edge },
        RegisteredTest { group: "Session", name: "Get Object", func: run_session_get_object },
        RegisteredTest { group: "Session", name: "File Io Comprehensive", func: run_session_file_io_comprehensive },
        RegisteredTest { group: "Session", name: "Tree Transformation Hierarchy", func: run_session_tree_transformation_hierarchy },
        // SessionConfig tests
        RegisteredTest { group: "SessionConfig", name: "Default Values", func: run_session_config_default_values },
        RegisteredTest { group: "SessionConfig", name: "Runtime Modification", func: run_session_config_runtime_modification },
        // OBJ tests
        RegisteredTest { group: "OBJ", name: "Read Bunny", func: run_obj_read_bunny },
        RegisteredTest { group: "OBJ", name: "Write Read Roundtrip", func: run_obj_write_read_roundtrip },
        // Triangulation2D tests
        RegisteredTest { group: "Triangulation2D", name: "Triangle", func: run_triangulation2d_triangle },
        RegisteredTest { group: "Triangulation2D", name: "Square", func: run_triangulation2d_square },
        RegisteredTest { group: "Triangulation2D", name: "Convex Polygon", func: run_triangulation2d_convex_polygon },
        RegisteredTest { group: "Triangulation2D", name: "Concave Polygon", func: run_triangulation2d_concave_polygon },
        RegisteredTest { group: "Triangulation2D", name: "Polygon With Hole", func: run_triangulation2d_polygon_with_hole },
        RegisteredTest { group: "Triangulation2D", name: "Polygon With Multiple Holes", func: run_triangulation2d_polygon_with_multiple_holes },
        RegisteredTest { group: "Triangulation2D", name: "Winding Correction", func: run_triangulation2d_winding_correction },
        // BVH tests
        RegisteredTest { group: "BVH", name: "Expand Bits", func: run_bvh_expand_bits },
        RegisteredTest { group: "BVH", name: "Morton Code Origin", func: run_bvh_morton_code_origin },
        RegisteredTest { group: "BVH", name: "Morton Code Corners", func: run_bvh_morton_code_corners },
        RegisteredTest { group: "BVH", name: "Morton Code Spatial Locality", func: run_bvh_morton_code_spatial_locality },
        RegisteredTest { group: "BVH", name: "Node Creation", func: run_bvh_node_creation },
        RegisteredTest { group: "BVH", name: "Node Leaf", func: run_bvh_node_leaf },
        RegisteredTest { group: "BVH", name: "Creation", func: run_bvh_creation },
        RegisteredTest { group: "BVH", name: "Build Empty", func: run_bvh_build_empty },
        RegisteredTest { group: "BVH", name: "Build Single", func: run_bvh_build_single },
        RegisteredTest { group: "BVH", name: "Build Multiple", func: run_bvh_build_multiple },
        RegisteredTest { group: "BVH", name: "Aabb Intersect", func: run_bvh_aabb_intersect },
        RegisteredTest { group: "BVH", name: "Check All Collisions", func: run_bvh_check_all_collisions },
        RegisteredTest { group: "BVH", name: "Merge Aabb", func: run_bvh_merge_aabb },
        RegisteredTest { group: "BVH", name: "Fixed 100 Boxes", func: run_bvh_fixed_100_boxes },
        // Quaternion tests
        RegisteredTest { group: "Quaternion", name: "Json Roundtrip", func: run_quaternion_json_roundtrip },
        // BoundingBox tests
        RegisteredTest { group: "BoundingBox", name: "Constructor", func: run_boundingbox_constructor },
        RegisteredTest { group: "BoundingBox", name: "Collision", func: run_boundingbox_collision },
        RegisteredTest { group: "BoundingBox", name: "Transformation", func: run_boundingbox_transformation },
        RegisteredTest { group: "BoundingBox", name: "Json Roundtrip", func: run_boundingbox_json_roundtrip },
        RegisteredTest { group: "BoundingBox", name: "Protobuf Roundtrip", func: run_boundingbox_protobuf_roundtrip },
        // Edge tests
        RegisteredTest { group: "Edge", name: "Json Roundtrip", func: run_edge_json_roundtrip },
        // Graph tests
        RegisteredTest { group: "Graph", name: "Json Roundtrip", func: run_graph_json_roundtrip },
        // Objects tests
        RegisteredTest { group: "Objects", name: "Json Roundtrip", func: run_objects_json_roundtrip },
        // Tree tests
        RegisteredTest { group: "Tree", name: "Json Roundtrip", func: run_tree_json_roundtrip },
        // TreeNode tests
        RegisteredTest { group: "TreeNode", name: "Json Roundtrip", func: run_treenode_json_roundtrip },
        // Vertex tests
        RegisteredTest { group: "Vertex", name: "Json Roundtrip", func: run_vertex_json_roundtrip },
        // Encoders tests
        RegisteredTest { group: "Encoders", name: "Json Dump Load", func: run_encoders_json_dump_load },
        RegisteredTest { group: "Encoders", name: "Json Dumps Loads", func: run_encoders_json_dumps_loads },
        RegisteredTest { group: "Encoders", name: "Encode Collection Values", func: run_encoders_encode_collection_values },
        RegisteredTest { group: "Encoders", name: "Encode Collection Shared Ptr", func: run_encoders_encode_collection_shared_ptr },
        RegisteredTest { group: "Encoders", name: "Decode Collection", func: run_encoders_decode_collection },
        RegisteredTest { group: "Encoders", name: "Decode Collection Ptr", func: run_encoders_decode_collection_ptr },
        RegisteredTest { group: "Encoders", name: "Nested Collections", func: run_encoders_nested_collections },
        RegisteredTest { group: "Encoders", name: "Roundtrip File Io", func: run_encoders_roundtrip_file_io },
        RegisteredTest { group: "Encoders", name: "Pretty Vs Compact", func: run_encoders_pretty_vs_compact },
        RegisteredTest { group: "Encoders", name: "Decode Primitives", func: run_encoders_decode_primitives },
        RegisteredTest { group: "Encoders", name: "Decode List", func: run_encoders_decode_list },
        RegisteredTest { group: "Encoders", name: "Decode Dict", func: run_encoders_decode_dict },
        RegisteredTest { group: "Encoders", name: "List In List In List", func: run_encoders_list_in_list_in_list },
        RegisteredTest { group: "Encoders", name: "Dict Of Lists", func: run_encoders_dict_of_lists },
        RegisteredTest { group: "Encoders", name: "List Of Dict", func: run_encoders_list_of_dict },
        RegisteredTest { group: "Encoders", name: "Dict Of Dicts", func: run_encoders_dict_of_dicts },
        // MeshIso tests
        RegisteredTest { group: "MeshIso", name: "Eval Gyroid", func: run_mesh_iso_eval_gyroid },
        RegisteredTest { group: "MeshIso", name: "Eval SchwarzP", func: run_mesh_iso_eval_schwarz_p },
        RegisteredTest { group: "MeshIso", name: "Eval Diamond", func: run_mesh_iso_eval_diamond },
        RegisteredTest { group: "MeshIso", name: "From Tpms Gyroid Solid", func: run_mesh_iso_from_tpms_gyroid_solid },
        RegisteredTest { group: "MeshIso", name: "From Tpms Diamond Sheet", func: run_mesh_iso_from_tpms_diamond_sheet },
        RegisteredTest { group: "MeshIso", name: "From Tpms Neovius Shell", func: run_mesh_iso_from_tpms_neovius_shell },
        RegisteredTest { group: "MeshIso", name: "SDF Sphere", func: run_mesh_iso_sdf_sphere },
        RegisteredTest { group: "MeshIso", name: "Smooth Union", func: run_mesh_iso_smooth_union },
        RegisteredTest { group: "MeshIso", name: "From Function", func: run_mesh_iso_from_function },
        // AABBTree tests
        RegisteredTest { group: "AABBTree", name: "Build Empty", func: run_aabbtree_build_empty },
        RegisteredTest { group: "AABBTree", name: "Build Single", func: run_aabbtree_build_single },
        RegisteredTest { group: "AABBTree", name: "Build Multiple", func: run_aabbtree_build_multiple },
        RegisteredTest { group: "AABBTree", name: "Node Count", func: run_aabbtree_node_count },
        RegisteredTest { group: "AABBTree", name: "Mesh Point Aabb", func: run_aabbtree_mesh_point_aabb },
        RegisteredTest { group: "AABBTree", name: "Mesh Point Aabb Matches Bvh", func: run_aabbtree_mesh_point_aabb_matches_bvh },
        // RTree tests
        RegisteredTest { group: "RTree", name: "Creation", func: run_rtree_creation },
        RegisteredTest { group: "RTree", name: "Insert", func: run_rtree_insert },
        RegisteredTest { group: "RTree", name: "Insert Multiple", func: run_rtree_insert_multiple },
        RegisteredTest { group: "RTree", name: "Search Hit", func: run_rtree_search_hit },
        RegisteredTest { group: "RTree", name: "Search Miss", func: run_rtree_search_miss },
        RegisteredTest { group: "RTree", name: "Remove", func: run_rtree_remove },
        RegisteredTest { group: "RTree", name: "Remove All", func: run_rtree_remove_all },
        RegisteredTest { group: "RTree", name: "Search Count", func: run_rtree_search_count },
        RegisteredTest { group: "RTree", name: "Search Stop", func: run_rtree_search_stop },
        RegisteredTest { group: "RTree", name: "Search 100 Boxes", func: run_rtree_search_100_boxes },
    ]
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

    // For each group, run its tests and emit <group>_test.json (lowercased).
    for (group, mut tests) in groups {
        tests.sort_by_key(|t| canonical_order.get(&(t.group, t.name)).copied().unwrap_or(usize::MAX));

        let mut results = Vec::new();
        for t in tests {
            let res = (t.func)();
            results.push(res);
        }

        let filename = format!("{}_test.json", group.to_lowercase());
        let path = out_dir.join(&filename);
        let tmp_path = out_dir.join(format!("{}.tmp", &filename));
        let json = serde_json::to_string_pretty(&results)?;
        fs::write(&tmp_path, &json)?;
        if let Err(_) = fs::rename(&tmp_path, &path) {
            // rename failed (Windows lock), fall back to remove + rename
            let _ = fs::remove_file(&path);
            fs::rename(&tmp_path, &path).or_else(|_| fs::write(&path, &json))?;
        }
    }

    Ok(())
}
