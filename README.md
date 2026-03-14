# session_rust

Rust geometry kernel — mirrors the C++ and Python implementations with identical APIs.

## Build

```bash
cd session_rust
cargo build --release
```

## Test

```bash
cargo test --lib
.\target\release\minitest.exe   # Windows
./target/release/minitest       # macOS/Linux
```

## Format & Lint

```bash
cargo fmt
cargo clippy --fix --allow-dirty --allow-staged
```
