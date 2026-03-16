use once_cell::sync::Lazy;
use parking_lot::RwLock;

pub struct SessionConfig {
    pub explode_mesh_faces: bool,
}

impl SessionConfig {
    pub fn new() -> Self {
        Self { explode_mesh_faces: false }
    }

    pub fn reset(&mut self) {
        self.explode_mesh_faces = false;
    }
}

impl Default for SessionConfig {
    fn default() -> Self { Self::new() }
}

pub struct GlobalSessionConfig {
    inner: RwLock<SessionConfig>,
}

impl GlobalSessionConfig {
    pub fn new() -> Self {
        Self { inner: RwLock::new(SessionConfig::default()) }
    }

    pub fn explode_mesh_faces(&self) -> bool { self.inner.read().explode_mesh_faces }
    pub fn set_explode_mesh_faces(&self, value: bool) { self.inner.write().explode_mesh_faces = value; }
    pub fn reset(&self) { self.inner.write().reset(); }
}

pub static SESSION_CONFIG: Lazy<GlobalSessionConfig> = Lazy::new(GlobalSessionConfig::new);
