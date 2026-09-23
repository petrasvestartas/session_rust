use crate::mini_test::TestResult;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_session_config_runtime_modification() -> TestResult {
    MINI_TEST!("Runtime Modification", {
        use crate::session_config::SessionConfig;
        use crate::session_config::SESSION_CONFIG;

        SESSION_CONFIG.write().reset();

        let mut config = SessionConfig::new();
        let other = SessionConfig::new();

        MINI_CHECK!(!config.explode_mesh_faces);
        MINI_CHECK!(config.scale_factor == 1.0);

        config.explode_mesh_faces = true;
        config.scale_factor = 0.001;

        MINI_CHECK!(config.explode_mesh_faces);
        MINI_CHECK!(config.scale_factor == 0.001);
        MINI_CHECK!(!other.explode_mesh_faces);
        MINI_CHECK!(other.scale_factor == 1.0);
        MINI_CHECK!(!SESSION_CONFIG.read().explode_mesh_faces);
        MINI_CHECK!(SESSION_CONFIG.read().scale_factor == 1.0);

        SESSION_CONFIG.write().explode_mesh_faces = true;
        SESSION_CONFIG.write().scale_factor = 0.001;

        MINI_CHECK!(SESSION_CONFIG.read().explode_mesh_faces);
        MINI_CHECK!(SESSION_CONFIG.read().scale_factor == 0.001);

        SESSION_CONFIG.write().reset();

        MINI_CHECK!(!SESSION_CONFIG.read().explode_mesh_faces);
        MINI_CHECK!(SESSION_CONFIG.read().scale_factor == 1.0);

        config.reset();

        MINI_CHECK!(!config.explode_mesh_faces);
        MINI_CHECK!(config.scale_factor == 1.0);
    })
}

REGISTER_MINI_TEST!(
    "SessionConfig",
    "Runtime Modification",
    crate::session_config_test::run_session_config_runtime_modification
);
