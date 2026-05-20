//! End-to-end GpuSession integration tests. Spawns a wgpu device (any backend,
//! any adapter — CPU fallback OK), builds a Session containing every
//! renderable geometry type, calls `rebuild_from`, and asserts:
//!
//! - Every guid that lands in an arena gets an instance_id via PickTable
//! - guid → instance_id → guid round-trips
//! - Arena slot counts match what we added
//! - Removing then re-adding the same guid leaves arena counts consistent
//! - Instance buffer grows past DEFAULT_INSTANCE_CAP without losing data
//!
//! Skipped (with a printed reason) if no wgpu adapter is available — e.g.
//! macOS-Intel CI runner without a usable GPU.

use session_rust::gpu_session::{
    GpuSession, InstanceData, DEFAULT_INSTANCE_CAP,
};
use session_rust::{
    Color, Line, Mesh, NurbsCurve, Plane, Point, PointCloud, Polyline, Session, Vector, OBB,
};

/// Try to build a wgpu device. Returns None on systems with no adapter
/// (typical for CI without GPU) so tests can skip gracefully.
fn try_make_device() -> Option<(wgpu::Device, wgpu::Queue)> {
    let instance = wgpu::Instance::default();
    let adapter_opt = pollster::block_on(instance.request_adapter(
        &wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::default(),
            compatible_surface: None,
            force_fallback_adapter: false,
        },
    ))
    .ok();
    let adapter = adapter_opt?;
    pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("gpu_session_test_device"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits::downlevel_defaults(),
        memory_hints: wgpu::MemoryHints::default(),
        trace: wgpu::Trace::Off,
        experimental_features: wgpu::ExperimentalFeatures::default(),
    }))
    .ok()
}

fn build_session_with_every_type() -> Session {
    let mut s = Session::new("integration");
    let _ = s.add_point(Point::new(1.0, 2.0, 3.0), None);
    let _ = s.add_line(Line::new(0.0, 0.0, 0.0, 1.0, 1.0, 0.0), None);
    let _ = s.add_polyline(
        Polyline::from_coords(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0]),
        None,
    );
    let pc = PointCloud::new(
        vec![Point::new(0.0, 0.0, 0.0), Point::new(1.0, 0.0, 0.0)],
        Vec::new(),
        vec![Color::new(255, 0, 0, 255), Color::new(0, 255, 0, 255)],
    );
    let _ = s.add_pointcloud(pc, None);

    // Mesh: a single triangle so we exercise the tri arena
    let mut m = Mesh::new();
    let a = m.add_vertex(Point::new(0.0, 0.0, 0.0), None);
    let b = m.add_vertex(Point::new(1.0, 0.0, 0.0), None);
    let c = m.add_vertex(Point::new(0.0, 1.0, 0.0), None);
    m.add_face(vec![a, b, c], None);
    let _ = s.add_mesh(m, None);

    let _ = s.add_plane(
        Plane::from_point_normal(Point::new(0.0, 0.0, 0.0), Vector::new(0.0, 0.0, 1.0)),
        None,
    );

    let bbox = OBB::new(
        Point::new(0.0, 0.0, 0.0),
        Vector::new(1.0, 0.0, 0.0),
        Vector::new(0.0, 1.0, 0.0),
        Vector::new(0.0, 0.0, 1.0),
        Vector::new(0.5, 0.5, 0.5),
    );
    let _ = s.add_obb(bbox);

    s
}

#[test]
fn rebuild_from_session_populates_arenas_and_pick_table() {
    let (device, queue) = match try_make_device() {
        Some(dq) => dq,
        None => {
            eprintln!("no wgpu adapter; skipping gpu integration test");
            return;
        }
    };

    let session = build_session_with_every_type();
    let mut gpu = GpuSession::new(&device);
    gpu.rebuild_from(&session, &device, &queue);

    // Every object should have a pick-table entry that round-trips
    for (guid, _) in &session.lookup {
        let id = gpu.pick.instance_id(guid).expect("guid not in pick table");
        let back = gpu.guid_for_instance(id).expect("instance_id not in pick table");
        assert_eq!(back, guid.as_str(), "round-trip mismatch");
    }

    // Each object should land in exactly one arena
    for guid in session.lookup.keys() {
        let in_tri = gpu.tri.slot(guid).is_some();
        let in_line = gpu.line.slot(guid).is_some();
        let in_point = gpu.point.slot(guid).is_some();
        let count = [in_tri, in_line, in_point].iter().filter(|b| **b).count();
        assert_eq!(count, 1, "guid {} present in {} arenas", guid, count);
    }
}

#[test]
fn remove_then_readd_keeps_arena_consistent() {
    let (device, queue) = match try_make_device() {
        Some(dq) => dq,
        None => return,
    };

    let mut session = Session::new("readd");
    let _ = session.add_point(Point::new(1.0, 2.0, 3.0), None);
    let point_guid = session
        .lookup
        .keys()
        .next()
        .expect("session should have one object")
        .clone();

    let mut gpu = GpuSession::new(&device);
    gpu.rebuild_from(&session, &device, &queue);

    let before = gpu.point.len();
    gpu.remove(&point_guid);
    assert_eq!(gpu.point.len(), before - 1);
    assert!(gpu.pick.instance_id(&point_guid).is_none());

    // Re-add via the same geometry
    if let Some(geom) = session.lookup.get(&point_guid).cloned() {
        gpu.add_geometry(&point_guid, &geom, &device, &queue);
    }
    assert_eq!(gpu.point.len(), before);
    assert!(gpu.pick.instance_id(&point_guid).is_some());
}

#[test]
fn instance_buffer_grows_past_default_cap() {
    let (device, queue) = match try_make_device() {
        Some(dq) => dq,
        None => return,
    };

    let mut gpu = GpuSession::new(&device);
    let initial_cap = gpu.instance_capacity;
    assert_eq!(initial_cap, DEFAULT_INSTANCE_CAP);

    // Force allocation beyond initial capacity by adding many small points.
    let mut session = Session::new("grow");
    for i in 0..(DEFAULT_INSTANCE_CAP as usize + 16) {
        let p = Point::new(i as f32 * 0.01, 0.0, 0.0);
        let _ = session.add_point(p, None);
    }
    gpu.rebuild_from(&session, &device, &queue);

    assert!(
        gpu.instance_capacity > initial_cap,
        "instance_capacity should have grown (was {}, now {})",
        initial_cap,
        gpu.instance_capacity
    );
    // CPU mirror should match the actual number of added points
    assert_eq!(
        gpu.instances_cpu.len() as u32,
        DEFAULT_INSTANCE_CAP + 16
    );
}

#[test]
fn nurbscurve_tessellates_into_line_arena() {
    let (device, queue) = match try_make_device() {
        Some(dq) => dq,
        None => return,
    };

    let mut session = Session::new("nurbs");
    let pts = vec![
        Point::new(0.0, 0.0, 0.0),
        Point::new(1.0, 2.0, 0.0),
        Point::new(2.0, 0.0, 0.0),
        Point::new(3.0, 2.0, 0.0),
        Point::new(4.0, 0.0, 0.0),
    ];
    let nc = NurbsCurve::create(false, 3, &pts);
    let guid = nc.guid().to_string();
    session.objects.nurbscurves.push(nc);

    let mut gpu = GpuSession::new(&device);
    gpu.rebuild_from(&session, &device, &queue);

    assert!(
        gpu.line.slot(&guid).is_some(),
        "NurbsCurve should land in the line arena"
    );
    let inst_id = gpu.pick.instance_id(&guid);
    assert!(inst_id.is_some(), "NurbsCurve should have a pick entry");
}

#[test]
fn instance_data_is_pod_and_correct_size() {
    // Compile-time-ish sanity: this would panic if alignment/size assumptions
    // shifted (e.g. someone added a field without updating _pad).
    assert_eq!(std::mem::size_of::<InstanceData>(), 96);
    assert_eq!(std::mem::align_of::<InstanceData>(), 16);
}
