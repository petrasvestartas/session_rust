use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_remesh_cdt_triangle() -> TestResult {
    MINI_TEST!("Triangle", {
        use crate::remesh_cdt::cdt_triangulate;

        let border = vec![(0.0_f64, 0.0_f64), (1.0, 0.0), (0.0, 1.0)];
        let tris = cdt_triangulate(&border, &[]);

        MINI_CHECK!(tris.len() == 1);
        MINI_CHECK!(tris[0].0 != tris[0].1);
        MINI_CHECK!(tris[0].1 != tris[0].2);
        MINI_CHECK!(tris[0].0 != tris[0].2);
    })
}

pub fn run_remesh_cdt_square() -> TestResult {
    MINI_TEST!("Square", {
        use crate::remesh_cdt::cdt_triangulate;

        let border = vec![(0.0_f64, 0.0_f64), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let tris = cdt_triangulate(&border, &[]);

        MINI_CHECK!(tris.len() == 2);
    })
}

pub fn run_remesh_cdt_convex_polygon() -> TestResult {
    MINI_TEST!("Convex Polygon", {
        use crate::remesh_cdt::cdt_triangulate;
        use crate::tolerance::PI;

        let hex: Vec<(f64, f64)> = (0..6).map(|i| {
            let a = i as f64 * PI / 3.0;
            (a.cos(), a.sin())
        }).collect();
        let tris = cdt_triangulate(&hex, &[]);

        MINI_CHECK!(tris.len() == 4);
    })
}

pub fn run_remesh_cdt_polygon_with_hole() -> TestResult {
    MINI_TEST!("Polygon With Hole", {
        use crate::remesh_cdt::cdt_triangulate;
        use crate::tolerance::TOLERANCE;

        let border = vec![(0.0_f64,0.0),(4.0,0.0),(4.0,4.0),(0.0,4.0)];
        let hole = vec![(1.0_f64,1.0),(1.0,3.0),(3.0,3.0),(3.0,1.0)];
        let tris = cdt_triangulate(&border, &[hole]);

        let flat: Vec<(f64,f64)> = vec![(0.0,0.0),(4.0,0.0),(4.0,4.0),(0.0,4.0),(1.0,1.0),(1.0,3.0),(3.0,3.0),(3.0,1.0)];
        let area: f64 = tris.iter().map(|&(a,b,c)| {
            let (ax,ay) = flat[a]; let (bx,by) = flat[b]; let (cx,cy) = flat[c];
            ((bx-ax)*(cy-ay) - (cx-ax)*(by-ay)).abs() * 0.5
        }).sum();

        MINI_CHECK!(!tris.is_empty());
        MINI_CHECK!(TOLERANCE.is_close(area, 4.0*4.0 - 2.0*2.0));
    })
}

REGISTER_MINI_TEST!("RemeshCDT", "Triangle", crate::remesh_cdt_test::run_remesh_cdt_triangle);
REGISTER_MINI_TEST!("RemeshCDT", "Square", crate::remesh_cdt_test::run_remesh_cdt_square);
REGISTER_MINI_TEST!("RemeshCDT", "Convex Polygon", crate::remesh_cdt_test::run_remesh_cdt_convex_polygon);
REGISTER_MINI_TEST!("RemeshCDT", "Polygon With Hole", crate::remesh_cdt_test::run_remesh_cdt_polygon_with_hole);
