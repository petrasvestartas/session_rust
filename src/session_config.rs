use once_cell::sync::Lazy;
use parking_lot::RwLock;

pub struct SessionConfig {
    pub explode_mesh_faces: bool,
    pub scale_factor: f64,
}

impl SessionConfig {
    pub fn new() -> Self {
        Self {
            explode_mesh_faces: false,
            scale_factor: 1.0,
        }
    }

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

pub struct GlobalSessionConfig {
    inner: RwLock<SessionConfig>,
}

impl GlobalSessionConfig {
    pub fn new() -> Self {
        Self {
            inner: RwLock::new(SessionConfig::default()),
        }
    }

    pub fn explode_mesh_faces(&self) -> bool {
        self.inner.read().explode_mesh_faces
    }
    pub fn set_explode_mesh_faces(&self, value: bool) {
        self.inner.write().explode_mesh_faces = value;
    }
    pub fn scale_factor(&self) -> f64 {
        self.inner.read().scale_factor
    }
    pub fn set_scale_factor(&self, value: f64) {
        self.inner.write().scale_factor = value;
    }
    pub fn reset(&self) {
        self.inner.write().reset();
    }
}

pub static SESSION_CONFIG: Lazy<GlobalSessionConfig> = Lazy::new(GlobalSessionConfig::new);
