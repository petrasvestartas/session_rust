# session_rust Agent Guide

Rust geometry kernel. One of three parallel implementations (`session_cpp`, `session_py`, `session_rust`) sharing protobuf schemas in `session_proto` and a Vue test viewer in `session_tests`.

## Goal

- Keep the Rust API identical to C++ and Python (names, signatures, test logic, line counts).
- Keep `cargo clippy --all-targets --all-features --release -- -D warnings` clean.
- Keep public items documented with `///` doc comments.
- Keep every class covered by minitests.

## Scope

These instructions apply to the whole `session_rust` repository. Preserve unrelated working-tree changes and keep patches focused on the requested task.

## Cross-Language Parity

- C++ is ground truth. Port to Rust without renaming methods, parameters, or test variables.
- A change to a public method here is incomplete until the same change exists in `session_cpp` and `session_py`. If the task is Rust-only, say so explicitly in the summary.
- Serialization names are fixed across languages: `file_json_dump` / `file_json_load` / `file_json_dumps` / `file_json_loads`, `pb_dump` / `pb_load` / `pb_dumps` / `pb_loads`, `to_proto` / `from_proto`.
- JSON fields are written in alphabetical order in all three languages. `Plane` and friends use hand-written `Serialize`/`Deserialize` impls to hold that order and the flat `frame` array — do not replace them with `#[derive]` for tidiness.
- Method order in a type's `impl`: constructors → accessors → mutators (`*_self`) → operators → utilities → serialization → `str` / `repr`.
- Rust naming stays snake_case, but the *identifier stem* must match C++: `from_point_normal`, `has_on_negative_side`, `duplicate`. Do not rename to something more idiomatic.
- `str()` and `repr()` are inherent methods matching the other languages, separate from `Display`/`Debug`.

## Types and Idioms

- `f64` for every coordinate, tolerance, and parameter value — the kernel is double-precision throughout.
- Accessors return `&T` where the C++ side returns `const&` (`origin()`, `x_axis()`); return owned values only for computed results.
- `guid` is a `OnceLock<String>` minted lazily on first read, so `duplicate()` produces a fresh GUID while `clone()` keeps identity semantics aligned with C++ and Python.
- Derive `Debug, Clone` on geometry types; add `Serialize, Deserialize` only where the derived field order matches the shared JSON contract.
- File I/O returns `Result<_, Box<dyn std::error::Error>>`; the `pb_load` / `file_json_loads` family that C++ and Python expose infallibly may unwrap internally, but do not add new panicking paths to methods that have a `Result` counterpart.
- Avoid `unwrap()`/`expect()` in library code except where an invariant is already established a line or two above; when you do, keep the panic message specific.
- Prefer slices (`&[Point]`) over `&Vec<Point>` in parameters.
- Import geometry types with a flat `use crate::{Point, Vector};` — the crate re-exports them; do not path through the defining module.
- `use crate::tolerance::{TOLERANCE, PI};` at the top of the file. Never hardcode epsilon literals.

## Runtime-Behavior Preservation

- Refactors must not change valid-input behavior, return types, ordering, orientation, side effects, or error behavior.
- Before changing an implementation, compare old and new control flow and identify any equivalence that depends on data-structure invariants.
- When a change requires a small, non-obvious change to a function body, add one short comment explaining which contract or invariant it preserves.
- Keep diffs narrow. Do not fold speculative architecture changes into documentation or lint-fixing patches.
- Silencing clippy with `#[allow(...)]` needs a one-line reason; fixing the code is the default.
- Use names that reflect cardinality: a returned collection is `points`, not `point`.
- No debug printing in library code. Comment only what is non-obvious.

## Crate Layout

- Every new module needs `pub mod <name>;` in `src/lib.rs`, and every new test module needs `pub mod <name>_test;` alongside it — a test module missing from `lib.rs` silently never runs.
- New public geometry types are re-exported so `use crate::Thing;` works, and added to `prelude` if they belong to the common working set.
- Optional heavy dependencies stay behind features (`pdf`) and off the default path; the wasm target must never be able to pull them. The existing `Cargo.toml` comments explain why — preserve that structure and its reasoning.

## Tests and Validation

- Run the smallest relevant selection first, then broaden:
  - `../bash/quicktest.sh <class> --rust` — one class
  - `../bash/minitest.sh --rust --no-web` — all Rust minitests
  - `../bash/minitest.sh` — all three languages plus the viewer on `localhost:8769`
- `./test.sh` runs `cargo fmt --all`, then `cargo clippy --all-targets --all-features --release -- -D warnings` (fails the run on any warning), then `cargo test --release`.
- CI builds `cargo build --lib --bin minitest` and runs `cargo run --bin minitest` on Linux, macOS, and Windows. Keep tests platform-agnostic — build paths with `Path`/`PathBuf`, never hardcode separators.
- Always run `git diff --check` on changed patches.
- Minitest conventions (identical test names and logic across languages, one test per API method, operators tested inside the constructor test, `file_json_*` and `to_proto`/`from_proto` tests for every class, one object per line in collections) are documented in the parent repo's `/test-rules` command — follow them exactly.
- Test file shape: `use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};`, `use crate::mini_test::TestResult;`, and the `tolerance` imports at the top; geometry imports go *inside* each `MINI_TEST!` block. Each test is a `pub fn run_<class>_<name>() -> TestResult` wrapping one `MINI_TEST!` block, followed by its `REGISTER_MINI_TEST!("Class", "Name", crate::<class>_test::run_<class>_<name>);` line.
- The constructor test covers: default and parameterized construction, indexing, `==`, `!=`, `str()`, `repr()`, in-place and copy operators, and `duplicate()` with a fresh GUID.
- Test artifacts write to `serialization/` and `session_tests/session_rust/`; regenerated artifacts are untracked — do not commit them.
- Prefer behavioral assertions over assertions about implementation details.

## Documentation

- Public types, methods, and free functions get `///` doc comments: one summary line, then `# Returns` / `# Panics` sections only where the behavior is not obvious from the signature.
- Document why an accessor returns a reference when that is the point of the method ("avoids clone").
- Document non-obvious setter side effects — in particular when setting one axis of a `Plane` or `Xform` normalizes it or recomputes another axis to preserve orthonormality.
- Keep doc examples compiling; a `no_run` example is better than a stale one.
- Explanatory comments about a dependency or build decision belong next to that decision (see `Cargo.toml`) and should survive edits to the lines around them.

## Public API

- Do not expose implementation-only payload, encoder, or helper types without a clear public use case; keep them `pub(crate)`.
- Preserve existing convenience APIs during refactors unless the task explicitly includes a deprecation or breaking-change plan.
- Removing a parameter or method is part of a task only when explicitly requested; then update implementation, docs, tests, and the C++/Python counterparts together rather than leaving hidden aliases.

## Git

- Never add Claude or any AI as git author, contributor, or co-author.
- Push all submodules with `../bash/git_push.sh "message"`.
- Check CI with `gh run list --limit 5`; inspect failures with `gh run view <id> --log-failed`.
