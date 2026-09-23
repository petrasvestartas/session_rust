use parking_lot::RwLock;

/// Runtime settings used by session operations.
pub struct SessionConfig {
    pub explode_mesh_faces: bool, // Whether meshing emits one face per triangle.
    pub scale_factor: f64,        // Scale applied by external session adapters.
}

impl SessionConfig {
    /// Creates settings with their default values.
    pub const fn new() -> Self {
        Self {
            explode_mesh_faces: false,
            scale_factor: 1.0,
        }
    }

    /// Restores every setting to its default value.
    pub fn reset(&mut self) {
        self.explode_mesh_faces = false;
        self.scale_factor = 1.0;
    }
}

impl Default for SessionConfig {
    fn default() -> Self {
        Self::new()
    }
}

pub static SESSION_CONFIG: RwLock<SessionConfig> = RwLock::new(SessionConfig::new()); // Process-wide settings used by session operations.
