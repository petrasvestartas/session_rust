#[cfg(feature = "viewer-display")]
mod enabled {
    use std::cell::Cell;

    #[derive(Clone, Copy)]
    struct State {
        remaining: usize,
        evaluations: usize,
        operations: usize,
        failed: bool,
    }

    thread_local! {
        static CURRENT: Cell<Option<State>> = const { Cell::new(None) };
    }

    pub struct Budget(Option<State>);

    impl Budget {
        pub fn new(limit: usize) -> Self {
            Self(CURRENT.replace(Some(State {
                remaining: limit,
                evaluations: limit.saturating_mul(16),
                operations: limit.saturating_mul(256),
                failed: limit == 0,
            })))
        }
    }

    impl Drop for Budget {
        fn drop(&mut self) {
            CURRENT.set(self.0);
        }
    }

    pub fn charge(count: usize) -> bool {
        CURRENT.with(|current| {
            let Some(mut state) = current.get() else {
                return true;
            };
            if count > state.remaining {
                state.failed = true;
            }
            if !state.failed {
                state.remaining -= count;
            }
            current.set(Some(state));
            !state.failed
        })
    }

    pub fn failed() -> bool {
        CURRENT.get().is_some_and(|state| state.failed)
    }

    pub fn charge_evaluations(count: usize) -> bool {
        CURRENT.with(|current| {
            let Some(mut state) = current.get() else {
                return true;
            };
            if count > state.evaluations {
                state.failed = true;
            }
            if !state.failed {
                state.evaluations -= count;
            }
            current.set(Some(state));
            !state.failed
        })
    }

    pub fn charge_operations(count: usize) -> bool {
        CURRENT.with(|current| {
            let Some(mut state) = current.get() else {
                return true;
            };
            if count > state.operations {
                state.failed = true;
            }
            if !state.failed {
                state.operations -= count;
            }
            current.set(Some(state));
            !state.failed
        })
    }

    pub fn subdivisions(ordinary: usize) -> usize {
        if CURRENT.get().is_some() {
            ordinary.max(128)
        } else {
            ordinary
        }
    }

    pub fn relative_chord(factor: f64, whole: f64, face: f64) -> f64 {
        if CURRENT.get().is_some() && whole > 0.0 && face > 0.0 {
            factor * whole / face
        } else {
            factor
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn failure_is_scoped_and_does_not_change_normal_meshing() {
            assert!(charge(usize::MAX));
            {
                let _budget = Budget::new(4);
                assert!(charge(3));
                assert!(!charge(2));
                assert!(failed());
            }
            assert!(!failed());
            assert!(charge(usize::MAX));
        }

        #[test]
        fn nested_budgets_restore_the_outer_remaining_work() {
            let _outer = Budget::new(4);
            assert!(charge(2));
            {
                let _inner = Budget::new(1);
                assert!(!charge(2));
            }
            assert!(charge(2));
            assert!(!charge(1));
        }

        #[test]
        fn scalar_buffers_are_charged_in_kibibytes_before_allocation() {
            let _budget = Budget::new(1);
            assert!(crate::display_budget::charge_scalars(128));
            assert!(!crate::display_budget::charge_scalars(1));
        }

        #[test]
        fn evaluation_work_has_an_independent_scoped_limit() {
            let _budget = Budget::new(2);
            assert!(charge_evaluations(32));
            assert!(charge(2));
            assert!(!charge_evaluations(1));
        }

        #[test]
        fn triangle_visits_have_a_separate_scoped_limit() {
            let _budget = Budget::new(1);
            assert!(charge_operations(256));
            assert!(charge_evaluations(16));
            assert!(charge(1));
            assert!(!charge_operations(1));
        }

        #[test]
        fn display_tolerance_is_absolute_across_patches_and_defaults_stay_unchanged() {
            assert_eq!(subdivisions(24), 24);
            assert_eq!(relative_chord(0.001, 100.0, 10.0), 0.001);
            {
                let _budget = Budget::new(100);
                assert_eq!(subdivisions(24), 128);
                assert_eq!(relative_chord(0.001, 100.0, 10.0), 0.01);
            }
            assert_eq!(subdivisions(24), 24);
            assert_eq!(relative_chord(0.001, 100.0, 10.0), 0.001);
        }
    }
}

pub(crate) fn charge_scalars(count: usize) -> bool {
    // One work unit reserves 1024 bytes; 128 f64 values occupy that space.
    charge(count.saturating_add(127) / 128)
}

pub(crate) fn charge_bytes(count: usize) -> bool {
    charge(count.saturating_add(1023) / 1024)
}

#[cfg(feature = "viewer-display")]
pub(crate) use enabled::{
    Budget, charge, charge_evaluations, charge_operations, failed, relative_chord, subdivisions,
};

#[cfg(not(feature = "viewer-display"))]
pub(crate) fn charge(_: usize) -> bool {
    true
}

#[cfg(not(feature = "viewer-display"))]
pub(crate) fn failed() -> bool {
    false
}

#[cfg(not(feature = "viewer-display"))]
pub(crate) fn charge_evaluations(_: usize) -> bool {
    true
}

#[cfg(not(feature = "viewer-display"))]
pub(crate) fn subdivisions(ordinary: usize) -> usize {
    ordinary
}

#[cfg(not(feature = "viewer-display"))]
pub(crate) fn relative_chord(factor: f64, _: f64, _: f64) -> f64 {
    factor
}

#[cfg(not(feature = "viewer-display"))]
pub(crate) fn charge_operations(_: usize) -> bool {
    true
}
