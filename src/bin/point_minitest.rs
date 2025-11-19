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
        MINI_CHECK!(checks, point.x() == 10.0);
        MINI_CHECK!(checks, point.y() == 2.0);
        MINI_CHECK!(checks, point.z() == 3.0);
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut results = Vec::new();
    results.push(run_point_constructor());
    results.push(run_point_equality_equal());
    results.push(run_point_equality_not_equal());

    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir
        .parent()
        .ok_or("Failed to find repo root from CARGO_MANIFEST_DIR")?;
    let out_dir = repo_root.join("session_tests").join("session_rust");
    fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("point_test.json");

    let json = serde_json::to_string_pretty(&results)?;
    fs::write(out_path, json)?;

    Ok(())
}
