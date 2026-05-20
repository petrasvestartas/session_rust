//! Headless tests for GpuSession vertex types, PickTable, and InstanceData
//! layout. The arena tests live alongside (require a wgpu device).

#[cfg(test)]
mod tests {
    use crate::gpu_session::*;

    #[test]
    fn vertex_sizes_are_packed() {
        assert_eq!(std::mem::size_of::<MeshVertex>(), 24);
        assert_eq!(std::mem::size_of::<LineVertex>(), 12);
        assert_eq!(std::mem::size_of::<PointVertex>(), 12);
    }

    #[test]
    fn instance_data_is_96_bytes_aligned_to_16() {
        assert_eq!(std::mem::size_of::<InstanceData>(), 96);
        assert_eq!(std::mem::align_of::<InstanceData>(), 16);
    }

    #[test]
    fn pick_table_roundtrips_guid() {
        let mut pt = PickTable::new();
        let id_a = pt.allocate("guid-a");
        let id_b = pt.allocate("guid-b");
        assert_eq!(pt.resolve(id_a), Some("guid-a"));
        assert_eq!(pt.resolve(id_b), Some("guid-b"));
        assert_eq!(pt.instance_id("guid-a"), Some(id_a));
    }

    #[test]
    fn pick_table_reuses_freed_ids() {
        let mut pt = PickTable::new();
        let id_a = pt.allocate("a");
        let _ = pt.allocate("b");
        pt.release("a");
        let id_c = pt.allocate("c");
        // c reuses a's slot
        assert_eq!(id_a, id_c);
        assert_eq!(pt.resolve(id_a), Some("c"));
    }

    #[test]
    fn pick_table_allocate_same_guid_twice_is_idempotent() {
        let mut pt = PickTable::new();
        let id1 = pt.allocate("x");
        let id2 = pt.allocate("x");
        assert_eq!(id1, id2);
    }

    #[test]
    fn topology_dispatch_is_correct() {
        use wgpu::PrimitiveTopology::*;
        assert_eq!(GeometryKind::Mesh.topology(), TriangleList);
        assert_eq!(GeometryKind::Polyline.topology(), LineList);
        assert_eq!(GeometryKind::Point.topology(), PointList);
        assert_eq!(GeometryKind::PointCloud.topology(), PointList);
        assert_eq!(GeometryKind::Plane.topology(), TriangleList);
    }
}
