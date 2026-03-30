use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::session_config::SESSION_CONFIG;


pub fn run_session_config_default_values() -> TestResult {
    MINI_TEST!("Default Values", {
        MINI_CHECK!(SESSION_CONFIG.explode_mesh_faces() == false);
        MINI_CHECK!((SESSION_CONFIG.scale_factor() - 1.0).abs() < 1e-10);
    })
}

pub fn run_session_config_runtime_modification() -> TestResult {
    MINI_TEST!("Runtime Modification", {
        MINI_CHECK!(SESSION_CONFIG.explode_mesh_faces() == false);
        SESSION_CONFIG.set_explode_mesh_faces(true);
        MINI_CHECK!(SESSION_CONFIG.explode_mesh_faces() == true);
        MINI_CHECK!((SESSION_CONFIG.scale_factor() - 1.0).abs() < 1e-10);
        SESSION_CONFIG.set_scale_factor(0.001);
        MINI_CHECK!((SESSION_CONFIG.scale_factor() - 0.001).abs() < 1e-10);
        SESSION_CONFIG.reset();
        MINI_CHECK!(SESSION_CONFIG.explode_mesh_faces() == false);
        MINI_CHECK!((SESSION_CONFIG.scale_factor() - 1.0).abs() < 1e-10);
    })
}

REGISTER_MINI_TEST!("SessionConfig", "Default Values", crate::session_config_test::run_session_config_default_values);
REGISTER_MINI_TEST!("SessionConfig", "Runtime Modification", crate::session_config_test::run_session_config_runtime_modification);
