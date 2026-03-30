use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_kdtree_nearest() -> TestResult {
    MINI_TEST!("Nearest", {
        use crate::{KDTree, Point};
        // Simple deterministic random using LCG
        let mut seed: u64 = 42;
        let mut rng = || -> f64 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 33) as f64 / (u32::MAX as f64) * 20.0 - 10.0
        };
        let mut pts = vec![];
        for _ in 0..100 {
            pts.push(Point::new(rng(), rng(), rng()));
        }
        let tree = KDTree::new(pts.clone());
        let query = Point::new(0.0, 0.0, 0.0);
        let (idx, dist) = tree.nearest(&query);
        let brute_idx = (0..pts.len()).min_by(|&a, &b| {
            let da = pts[a][0]*pts[a][0]+pts[a][1]*pts[a][1]+pts[a][2]*pts[a][2];
            let db = pts[b][0]*pts[b][0]+pts[b][1]*pts[b][1]+pts[b][2]*pts[b][2];
            da.partial_cmp(&db).unwrap()
        }).unwrap();
        let brute_dist = (pts[brute_idx][0].powi(2)+pts[brute_idx][1].powi(2)+pts[brute_idx][2].powi(2)).sqrt();

        MINI_CHECK!(idx == brute_idx);
        MINI_CHECK!(TOLERANCE.is_close(dist, brute_dist));
    })
}
REGISTER_MINI_TEST!("KDTree", "Nearest", crate::kdtree_test::run_kdtree_nearest);

pub fn run_kdtree_nearest_k() -> TestResult {
    MINI_TEST!("NearestK", {
        use crate::{KDTree, Point};
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
            Point::new(4.0, 0.0, 0.0),
        ];
        let tree = KDTree::new(pts);
        let query = Point::new(1.5, 0.0, 0.0);
        let result = tree.nearest_k(&query, 3);

        MINI_CHECK!(result.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(result[0].1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(result[1].1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(result[2].1, 1.5));
    })
}
REGISTER_MINI_TEST!("KDTree", "NearestK", crate::kdtree_test::run_kdtree_nearest_k);

pub fn run_kdtree_radius_search() -> TestResult {
    MINI_TEST!("RadiusSearch", {
        use crate::{KDTree, Point};
        let pts = vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(5.0, 0.0, 0.0),
        ];
        let tree = KDTree::new(pts);
        let query = Point::new(0.5, 0.0, 0.0);
        let result = tree.radius_search(&query, 1.1);

        MINI_CHECK!(result.len() == 2);
        MINI_CHECK!(TOLERANCE.is_close(result[0].1, 0.5));
        MINI_CHECK!(TOLERANCE.is_close(result[1].1, 0.5));
    })
}
REGISTER_MINI_TEST!("KDTree", "RadiusSearch", crate::kdtree_test::run_kdtree_radius_search);

pub fn run_kdtree_single_point() -> TestResult {
    MINI_TEST!("SinglePoint", {
        use crate::{KDTree, Point};
        let pts = vec![Point::new(3.0, 4.0, 0.0)];
        let tree = KDTree::new(pts);
        let query = Point::new(0.0, 0.0, 0.0);
        let (idx, dist) = tree.nearest(&query);

        MINI_CHECK!(idx == 0);
        MINI_CHECK!(TOLERANCE.is_close(dist, 5.0));
    })
}
REGISTER_MINI_TEST!("KDTree", "SinglePoint", crate::kdtree_test::run_kdtree_single_point);

pub fn run_kdtree_nearest_brute_force() -> TestResult {
    MINI_TEST!("NearestBruteForce", {
        use crate::{KDTree, Point};
        let mut seed: u64 = 7;
        let mut rng = || -> f64 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (seed >> 33) as f64 / (u32::MAX as f64) * 100.0
        };
        let mut pts = vec![];
        for _ in 0..50 {
            pts.push(Point::new(rng(), rng(), rng()));
        }
        let mut queries = vec![];
        for _ in 0..10 {
            queries.push(Point::new(rng(), rng(), rng()));
        }
        let tree = KDTree::new(pts.clone());
        let mut all_match = true;
        for q in &queries {
            let (_idx, dist) = tree.nearest(q);
            let brute = (0..pts.len()).min_by(|&a, &b| {
                let da = (pts[a][0]-q[0]).powi(2)+(pts[a][1]-q[1]).powi(2)+(pts[a][2]-q[2]).powi(2);
                let db = (pts[b][0]-q[0]).powi(2)+(pts[b][1]-q[1]).powi(2)+(pts[b][2]-q[2]).powi(2);
                da.partial_cmp(&db).unwrap()
            }).unwrap();
            let brute_d = ((pts[brute][0]-q[0]).powi(2)+(pts[brute][1]-q[1]).powi(2)+(pts[brute][2]-q[2]).powi(2)).sqrt();
            if !TOLERANCE.is_close(dist, brute_d) {
                all_match = false;
            }
        }

        MINI_CHECK!(all_match);
    })
}
REGISTER_MINI_TEST!("KDTree", "NearestBruteForce", crate::kdtree_test::run_kdtree_nearest_brute_force);
