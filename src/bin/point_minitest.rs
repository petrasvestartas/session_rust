use session_rust::{Color, Point};
use serde::Serialize;
use serde_json::json;
use std::fs;
use std::path::PathBuf;
use std::time::Instant;

#[derive(Serialize)]
struct CheckRecord {
    line: u32,
    code_line: &'static str,
    passed: bool,
}

#[derive(Serialize)]
struct TestResult {
    test_name: &'static str,
    passed: bool,
    time_ms: f64,
    line: u32,
    code: &'static str,
    checks: Vec<CheckRecord>,
    failures: Vec<serde_json::Value>,
}

macro_rules! MINI_CHECK {
    ($checks:expr, $expr:expr) => {{
        let passed = $expr;
        $checks.push(CheckRecord {
            line: line!(),
            code_line: stringify!($expr),
            passed,
        });
        if !passed {
            return Err(format!("expression is not true: {}", stringify!($expr)));
        }
    }};
}

fn run_point_constructor() -> TestResult {
    let line = line!();
    let start = Instant::now();
    let mut checks = Vec::new();
    let mut failures = Vec::new();
    let mut passed = true;

    let result: Result<(), String> = (|| {
        let mut point = Point::new(1.0, 2.0, 3.0);
        point[0] = 10.0;

        MINI_CHECK!(checks, point.name == "my_point");
        MINI_CHECK!(checks, !point.guid.is_empty());
        MINI_CHECK!(checks, point[0] == 10.0);
        MINI_CHECK!(checks, point[1] == 2.0);
        MINI_CHECK!(checks, point[2] == 3.0);
        MINI_CHECK!(checks, point.width == 1.0);
        MINI_CHECK!(checks, point.pointcolor == Color::white());
        Ok(())
    })();

    if let Err(msg) = result {
        passed = false;
        failures.push(json!({ "error": msg }));
    }

    let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
    let time_ms = (elapsed_ms * 1000.0).round() / 1000.0;

    let code = "    let mut point = Point::new(1.0, 2.0, 3.0);\n    point[0] = 10.0;";

    TestResult {
        test_name: "constructor",
        passed,
        time_ms,
        line,
        code,
        checks,
        failures,
    }
}

fn run_point_equality_equal() -> TestResult {
    let line = line!();
    let start = Instant::now();
    let mut checks = Vec::new();
    let mut failures = Vec::new();
    let mut passed = true;

    let result: Result<(), String> = (|| {
        let p1 = Point::new(1.0, 2.0, 3.0);
        let p2 = Point::new(1.0, 2.0, 3.0);

        let eq_result = p1 == p2;
        let neq_result = p1 != p2;

        MINI_CHECK!(checks, eq_result == true);
        MINI_CHECK!(checks, neq_result == false);
        Ok(())
    })();

    if let Err(msg) = result {
        passed = false;
        failures.push(json!({ "error": msg }));
    }

    let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
    let time_ms = (elapsed_ms * 1000.0).round() / 1000.0;

    let code = "    let p1 = Point::new(1.0, 2.0, 3.0);\n    let p2 = Point::new(1.0, 2.0, 3.0);\n\n    let eq_result = p1 == p2;\n    let neq_result = p1 != p2;";

    TestResult {
        test_name: "equality_equal",
        passed,
        time_ms,
        line,
        code,
        checks,
        failures,
    }
}

fn run_point_equality_not_equal() -> TestResult {
    let line = line!();
    let start = Instant::now();
    let mut checks = Vec::new();
    let mut failures = Vec::new();
    let mut passed = true;

    let result: Result<(), String> = (|| {
        let p3 = Point::new(1.0, 2.0, 3.0);
        let p4 = Point::new(1.1, 2.0, 3.0);

        let eq_result = p3 == p4;
        let neq_result = p3 != p4;

        MINI_CHECK!(checks, eq_result == false);
        MINI_CHECK!(checks, neq_result == true);
        Ok(())
    })();

    if let Err(msg) = result {
        passed = false;
        failures.push(json!({ "error": msg }));
    }

    let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
    let time_ms = (elapsed_ms * 1000.0).round() / 1000.0;

    let code = "    let p3 = Point::new(1.0, 2.0, 3.0);\n    let p4 = Point::new(1.1, 2.0, 3.0);\n\n    let eq_result = p3 == p4;\n    let neq_result = p3 != p4;";

    TestResult {
        test_name: "equality_not_equal",
        passed,
        time_ms,
        line,
        code,
        checks,
        failures,
    }
}

fn run_color_constructor() -> TestResult {
    let line = line!();
    let start = Instant::now();
    let mut checks = Vec::new();
    let mut failures = Vec::new();
    let mut passed = true;

    let result: Result<(), String> = (|| {
        let mut red = Color::new(255, 0, 0, 255);
        red.name = "red".to_string();

        MINI_CHECK!(checks, red.name == "red");
        MINI_CHECK!(checks, !red.guid.to_string().is_empty());
        MINI_CHECK!(checks, red.r == 255);
        MINI_CHECK!(checks, red.g == 0);
        MINI_CHECK!(checks, red.b == 0);
        MINI_CHECK!(checks, red.a == 255);
        Ok(())
    })();

    if let Err(msg) = result {
        passed = false;
        failures.push(json!({ "error": msg }));
    }

    let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
    let time_ms = (elapsed_ms * 1000.0).round() / 1000.0;

    let code = "    let mut red = Color::new(255, 0, 0, 255);\n    red.name = \"red\".to_string();";

    TestResult {
        test_name: "constructor",
        passed,
        time_ms,
        line,
        code,
        checks,
        failures,
    }
}

fn run_color_json_roundtrip() -> TestResult {
    let line = line!();
    let start = Instant::now();
    let mut checks = Vec::new();
    let mut failures = Vec::new();
    let mut passed = true;

    let result: Result<(), String> = (|| {
        let mut original = Color::new(128, 64, 192, 255);
        original.name = "purple".to_string();

        let json_string = original.jsondump().map_err(|e| e.to_string())?;
        let restored = Color::jsonload(&json_string).map_err(|e| e.to_string())?;

        MINI_CHECK!(checks, restored.r == original.r);
        MINI_CHECK!(checks, restored.g == original.g);
        MINI_CHECK!(checks, restored.b == original.b);
        MINI_CHECK!(checks, restored.a == original.a);
        MINI_CHECK!(checks, restored.name == original.name);
        Ok(())
    })();

    if let Err(msg) = result {
        passed = false;
        failures.push(json!({ "error": msg }));
    }

    let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;
    let time_ms = (elapsed_ms * 1000.0).round() / 1000.0;

    let code = "    let mut original = Color::new(128, 64, 192, 255);\n    original.name = \"purple\".to_string();\n    let json_string = original.jsondump()?;\n    let restored = Color::jsonload(&json_string)?;";

    TestResult {
        test_name: "json_roundtrip",
        passed,
        time_ms,
        line,
        code,
        checks,
        failures,
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut point_results = Vec::new();
    point_results.push(run_point_constructor());
    point_results.push(run_point_equality_equal());
    point_results.push(run_point_equality_not_equal());

    let mut color_results = Vec::new();
    color_results.push(run_color_constructor());
    color_results.push(run_color_json_roundtrip());

    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir
        .parent()
        .ok_or("Failed to find repo root from CARGO_MANIFEST_DIR")?;
    let out_dir = repo_root.join("session_tests").join("session_rust");
    fs::create_dir_all(&out_dir)?;

    let point_path = out_dir.join("point_test.json");
    let point_json = serde_json::to_string_pretty(&point_results)?;
    fs::write(point_path, point_json)?;

    let color_path = out_dir.join("color_test.json");
    let color_json = serde_json::to_string_pretty(&color_results)?;
    fs::write(color_path, color_json)?;

    Ok(())
}
