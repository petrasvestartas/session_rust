use once_cell::sync::Lazy;
use parking_lot::RwLock;

/// Runtime settings used by session operations.
pub struct SessionConfig {
    /// Whether meshing emits one face per triangle.
    pub explode_mesh_faces: bool,
    /// Scale applied by external session adapters.
    pub scale_factor: f64,
}

impl SessionConfig {
    /// Creates settings with their default values.
    pub fn new() -> Self {
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

/// Thread-safe owner for shared session settings.
pub struct GlobalSessionConfig {
    inner: RwLock<SessionConfig>,
}

impl GlobalSessionConfig {
    /// Creates shared settings with their default values.
    pub fn new() -> Self {
        Self {
            inner: RwLock::new(SessionConfig::default()),
        }
    }

    /// Returns whether meshing emits one face per triangle.
    pub fn explode_mesh_faces(&self) -> bool {
        self.inner.read().explode_mesh_faces
    }

    /// Sets whether meshing emits one face per triangle.
    pub fn set_explode_mesh_faces(&self, value: bool) {
        self.inner.write().explode_mesh_faces = value;
    }

    /// Returns the scale applied by external session adapters.
    pub fn scale_factor(&self) -> f64 {
        self.inner.read().scale_factor
    }

    /// Sets the scale applied by external session adapters.
    pub fn set_scale_factor(&self, value: f64) {
        self.inner.write().scale_factor = value;
    }

    /// Restores every setting to its default value.
    pub fn reset(&self) {
        self.inner.write().reset();
    }
}

impl Default for GlobalSessionConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Process-wide settings used by session operations.
pub static SESSION_CONFIG: Lazy<GlobalSessionConfig> = Lazy::new(GlobalSessionConfig::new);
