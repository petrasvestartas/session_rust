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

    // Determine the first check line (if any)
    let first_check_line = checks
        .iter()
        .map(|c| c.line)
        .min();

    // If we have no checks, we don't know where to stop; return empty snippet
    let first_check_line = match first_check_line {
        Some(l) => l,
        None => return String::new(),
    };

    // Snippet starts on the line after the MINI_TEST! call
    let start_line = macro_line.saturating_add(1);
    if first_check_line <= start_line {
        return String::new();
    }
    let end_line = first_check_line - 1;

    let src = match fs::read_to_string(&path) {
        Ok(s) => s,
        Err(_) => return String::new(),
    };

    let mut code_lines = Vec::new();
    for (idx, line) in src.lines().enumerate() {
        let line_no = (idx as u32) + 1;
        if line_no < start_line {
            continue;
        }
        if line_no > end_line {
            break;
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

/// Run all registered Rust mini-tests for this crate and write JSON results
/// to the session_tests/session_rust directory, matching the layout of the
/// Python and C++ mini-test frameworks.
pub fn run_all(language: &str) -> Result<(), Box<dyn std::error::Error>> {
    let _ = language; // kept for symmetry with other languages

    // Group registered tests by logical group name (e.g. "Point", "Color").
    let mut groups: BTreeMap<&'static str, Vec<RegisteredTest>> = BTreeMap::new();
    for t in inventory::iter::<RegisteredTest> {
        groups.entry(t.group).or_default().push(*t);
    }

    if groups.is_empty() {
        return Ok(());
    }

    // Compute output directory: repo_root/session_tests/session_rust
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let repo_root = manifest_dir
        .parent()
        .ok_or("Failed to find repo root from CARGO_MANIFEST_DIR")?;
    let out_dir = repo_root.join("session_tests").join("session_rust");
    fs::create_dir_all(&out_dir)?;

    // For each group, run its tests and emit <group>_test.json (lowercased).
    for (group, mut tests) in groups {
        tests.sort_by_key(|t| {
            let pri = match t.name {
                "constructor" => 0,
                "transformation" => 1,
                "json_roundtrip" => 2,
                "protobuf_roundtrip" => 3,
                _ => 100,
            };
            (pri, t.name)
        });

        let mut results = Vec::new();
        for t in tests {
            let res = (t.func)();
            results.push(res);
        }

        let filename = format!("{}_test.json", group.to_lowercase());
        let path = out_dir.join(filename);
        let json = serde_json::to_string_pretty(&results)?;
        fs::write(path, json)?;
    }

    Ok(())
}
