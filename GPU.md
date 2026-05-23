# GPU layer (session_rust)

GPU mirror of `Session` for use in a wgpu-based viewer. Lives entirely in `session_rust` — no extra crates required. Branch of origin: `feat/gpu-session`, merged to `main` 2026-05-23.

## What it is

A 1:1 GPU mirror of a CPU `Session`. Three topology arenas (triangles / lines / points) hold tessellated vertex data; one instance buffer holds per-object transform/color/flag state; a pick table maps GPU `instance_id` ⇄ session `guid`.

```
Session (CPU)                GpuSession (GPU)
  Geometry by guid    ─►       tri  : GpuArena<MeshVertex>      (Triangles)
                                line : GpuArena<LineVertex>      (Lines)
                                point: GpuArena<PointVertex>     (Points)
                                instance_buffer: [InstanceData]  (per-object)
                                pick: PickTable                  (id ⇄ guid)
```

## Files

| File | What |
|---|---|
| `src/gpu_session.rs` | `GpuSession`, vertex types, `InstanceData`, `PickTable`. Public draw API. |
| `src/gpu_arena.rs` | `GpuArena<V>` — game-engine-style free-list allocator over a wgpu vertex+index buffer pair. Amortized 2× growth, no per-update buffer recreate. |
| `src/gpu_adapters.rs` | `to_*_vertices()` for each geometry type (Point, Line, Polyline, PointCloud, Mesh, Plane, OBB) + `kind_for_geometry()`. Flat normals for meshes. |
| `src/gpu_shaders.rs` | `MESH_WGSL`, `LINE_WGSL`, `POINT_WGSL` source strings + `build_*_pipeline()` factories + `build_bind_group_layout/build_bind_group/create_camera_buffer`. |
| `src/gpu_demo.rs` | `make_demo_session()` — point/line/polyline/pointcloud/mesh/plane/obb/nurbscurve sample. |
| `src/session_pick.rs` | `Ray`, `screen_to_world_ray`, `Session::pick_by_ray`, `Session::pick_by_screen`. CPU-side, delegates to existing `Session::ray_cast`. |
| `tests/gpu_integration.rs` | End-to-end: build session → rebuild GPU → draw → update → pick. |

`src/lib.rs::prelude` re-exports the entire wiring surface; `use session_rust::prelude::*;` is the one-stop import.

## Wiring recipe

The canonical recipe lives in the `//!` doc-comment at the top of `src/gpu_session.rs`. Summary:

```rust
use session_rust::prelude::*;

// One-time setup
let session = make_demo_session();           // or your own
let mut gpu = GpuSession::new(&device);
gpu.rebuild_from(&session, &device, &queue);

let bgl        = build_bind_group_layout(&device);
let camera_buf = create_camera_buffer(&device);
let bind_group = build_bind_group(&device, &bgl, &camera_buf, &gpu.instance_buffer);

let mesh_pipe  = build_mesh_pipeline (&device, surface_format, depth_format, &bgl);
let line_pipe  = build_line_pipeline (&device, surface_format, depth_format, &bgl);
let point_pipe = build_point_pipeline(&device, surface_format, depth_format, &bgl);

// Per frame
queue.write_buffer(&camera_buf, 0, bytemuck::bytes_of(&CameraUniform { view_proj }));
pass.set_bind_group(0, &bind_group, &[]);
pass.set_pipeline(&mesh_pipe);  gpu.draw_meshes(&mut pass);
pass.set_pipeline(&line_pipe);  gpu.draw_lines (&mut pass);
pass.set_pipeline(&point_pipe); gpu.draw_points(&mut pass);
```

All three pipelines share the same bind group: `group(0) binding(0)=camera uniform`, `group(0) binding(1)=instances storage`.

## Picking

```rust
let hits = session.pick_by_screen(&view, &proj,
    (width as f32, height as f32),
    (cursor_x, cursor_y),
    0.1, // pick_radius_world
);
if let Some(hit) = hits.first() {
    gpu.set_flag(hit.guid(), InstanceData::FLAG_SELECTED, true, &queue);
}
```

CPU-side only — unprojects to a world-space ray, then delegates to the existing `Session::ray_cast` (Point / Line / Polyline / Plane / OBB / Mesh narrow-phase via BVH).

## Updating geometry at runtime

```rust
// Full rebuild (simple, O(n)):
gpu.rebuild_from(&session, &device, &queue);

// Incremental (warm path):
gpu.add_geometry(&guid, geom_ref, &device, &queue);
gpu.remove(&guid);
gpu.update_transform(&guid, model_mat4, &queue);
gpu.update_color    (&guid, [r,g,b,a], &queue);
gpu.set_flag        (&guid, InstanceData::FLAG_SELECTED, true, &queue);
```

Arenas grow 2× on overflow; removal frees the vertex/index range to a best-fit free list, no shifting.

## Vertex / instance layouts

```
MeshVertex  : position[3] + normal[3] + color[u8;4]   (28 B) — Float32x3, Float32x3, Unorm8x4
LineVertex  : position[3] + color[u8;4]               (16 B) — Float32x3, Unorm8x4
PointVertex : position[3] + color[u8;4]               (16 B)
InstanceData: model[4x4] + tint[4] + object_id + flags + _pad[2]   (96 B, align 16)
CameraUniform: view_proj[4x4]                         (64 B, align 16)
```

`InstanceData::FLAG_SELECTED / FLAG_HOVERED / FLAG_HIDDEN` are bit 0/1/2.

Positions are `f32` (the wider library is f32 after the f32 migration on origin/main).

## Phased delivery (commit history)

1. `phase 2: GpuSession + arena + per-type vertex adapters` (707699f)
2. `phase 2: GpuSession composer + Session picking convenience` (f414661)
3. `phase 3: per-vertex colors, flat normals, BRep tessellation, growth, integration tests` (fb44871)
4. `phase 4: NurbsCurve/NurbsSurface tessellation + draw helpers` (b198f17)
5. `phase 5: WGSL shader templates + pipeline factories` (70ad677)
6. `phase 6: viewer wiring infrastructure (prelude + bind-group + demo + docs)` (6c1a7b2)

## Tests

```bash
cargo test --lib gpu_           # unit tests
cargo test --test gpu_integration   # end-to-end (headless wgpu)
```

Integration test asserts: rebuild produces non-empty tri/line/point arenas for the demo session, updates write to the right instance slot, pick returns the expected guid.

## Out of scope (intentional)

- No window/event loop — that's the viewer's job (session_viewer uses winit).
- No depth/MSAA opinion — pipeline factories take `depth_format: Option<wgpu::TextureFormat>`; pass `None` for no depth.
- No GPU-side picking — CPU `ray_cast` handles it. `InstanceData::object_id` is mirrored in case shader-side ID readback is added later.
