use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_marching_squares_extract() -> TestResult {
    MINI_TEST!("Extract", {
        use crate::MarchingSquares;
        let grid = vec![
            vec![0.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 0.0],
        ];
        let result = MarchingSquares::extract(&grid, 0.5, 1.0);

        MINI_CHECK!(result.len() == 1);
        MINI_CHECK!(result[0].point_count() == 5);
        MINI_CHECK!(result[0].is_closed());
    })
}
REGISTER_MINI_TEST!("MarchingSquares", "Extract", crate::marching_squares_test::run_marching_squares_extract);

pub fn run_marching_squares_extract_from_func() -> TestResult {
    MINI_TEST!("Extract From Func", {
        use crate::MarchingSquares;
        let result = MarchingSquares::extract_from_func(
            |x, y| 1.0 - (x * x + y * y),
            (-2.0, 2.0), (-2.0, 2.0), 20, 20, 0.0
        );

        MINI_CHECK!(result.len() > 0);
    })
}
REGISTER_MINI_TEST!("MarchingSquares", "Extract From Func", crate::marching_squares_test::run_marching_squares_extract_from_func);

pub fn run_marching_squares_empty_grid() -> TestResult {
    MINI_TEST!("Empty Grid", {
        use crate::MarchingSquares;
        let grid: Vec<Vec<f64>> = vec![];
        let result = MarchingSquares::extract(&grid, 0.5, 1.0);

        MINI_CHECK!(result.len() == 0);
    })
}
REGISTER_MINI_TEST!("MarchingSquares", "Empty Grid", crate::marching_squares_test::run_marching_squares_empty_grid);

pub fn run_marching_squares_all_above() -> TestResult {
    MINI_TEST!("All Above", {
        use crate::MarchingSquares;
        let grid = vec![
            vec![2.0, 2.0, 2.0],
            vec![2.0, 2.0, 2.0],
            vec![2.0, 2.0, 2.0],
        ];
        let result = MarchingSquares::extract(&grid, 1.0, 1.0);

        MINI_CHECK!(result.len() == 0);
    })
}
REGISTER_MINI_TEST!("MarchingSquares", "All Above", crate::marching_squares_test::run_marching_squares_all_above);

pub fn run_marching_squares_all_below() -> TestResult {
    MINI_TEST!("All Below", {
        use crate::MarchingSquares;
        let grid = vec![
            vec![0.0, 0.0, 0.0],
            vec![0.0, 0.0, 0.0],
            vec![0.0, 0.0, 0.0],
        ];
        let result = MarchingSquares::extract(&grid, 1.0, 1.0);

        MINI_CHECK!(result.len() == 0);
    })
}
REGISTER_MINI_TEST!("MarchingSquares", "All Below", crate::marching_squares_test::run_marching_squares_all_below);

pub fn run_marching_squares_interpolation() -> TestResult {
    MINI_TEST!("Interpolation", {
        use crate::MarchingSquares;
        let grid = vec![
            vec![0.0, 2.0],
            vec![0.0, 2.0],
        ];
        let result = MarchingSquares::extract(&grid, 1.0, 1.0);

        MINI_CHECK!(result.len() == 1);
        let seg = &result[0];
        MINI_CHECK!(TOLERANCE.is_close(seg.get_point(0).unwrap()[0], 0.5));
        MINI_CHECK!(TOLERANCE.is_close(seg.get_point(1).unwrap()[0], 0.5));
    })
}
REGISTER_MINI_TEST!("MarchingSquares", "Interpolation", crate::marching_squares_test::run_marching_squares_interpolation);
