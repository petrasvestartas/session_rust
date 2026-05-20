#[cfg(test)]
mod tests {
    use crate::gpu_shaders::*;

    #[test]
    fn camera_uniform_size_and_align() {
        assert_eq!(std::mem::size_of::<CameraUniform>(), 64);
        assert_eq!(std::mem::align_of::<CameraUniform>(), 16);
    }

    #[test]
    fn wgsl_sources_are_non_empty() {
        assert!(!MESH_WGSL.is_empty());
        assert!(!LINE_WGSL.is_empty());
        assert!(!POINT_WGSL.is_empty());
        // The Instance struct binding layout must match InstanceData in
        // gpu_session.rs — sanity check field names appear:
        assert!(MESH_WGSL.contains("model: mat4x4<f32>"));
        assert!(MESH_WGSL.contains("tint: vec4<f32>"));
        assert!(MESH_WGSL.contains("@builtin(instance_index)"));
    }
}
