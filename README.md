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

## GPU bridge — `Mesh` functions for the viewer

The kernel is f64; these prepare the f32 snapshot the wgpu viewer draws:

| function | what it does |
|---|---|
| `mesh.gpu_mesh(&device) -> &GpuMesh` | **viewer entry point** — flattens + uploads **once**, caches, and returns `GpuMesh { vbo, ibo, index_count }` to bind and `draw_indexed`. Repeat calls are free. |
| `mesh.to_render() -> RenderMesh` | CPU-only flatten: f64 half-edge → `RenderMesh { vertices: Vec<RenderVertex>, indices: Vec<u32> }` — position + normal + RGBA, color chosen by `color_mode`. |
| `mesh.invalidate_gpu()` | drops the cache so the next `gpu_mesh` rebuilds (the color setters / edits already call it). |
| `mesh.strip_render_data()` | clears the half-edge map + color arrays to shrink a serialized mesh (after marching-cubes / merges). |
| `RenderVertex::layout()` | the `wgpu::VertexBufferLayout` the pipeline declares — pos @0, normal @1, color @2, stride 40. |

The whole GPU bridge lives in `src/render_mesh.rs` (`to_render`, `gpu_mesh`, `invalidate_gpu` + the
`RenderVertex`/`RenderMesh`/`GpuMesh`/`GpuCache` types); `src/mesh.rs` keeps only the `gpu_cache`
field and `strip_render_data`, so the core mesh code stays free of `wgpu`.
