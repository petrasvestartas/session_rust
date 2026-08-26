use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_octree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::SpatialOctree;
        use crate::Point;

        // SpatialOctree: per-node spacing-limited subsamples for LOD point rendering
        // leaf_capacity 16 > 9 points: the root absorbs everything, one node
        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 16);

        MINI_CHECK!(tree.node_count() == 1);
        MINI_CHECK!(tree.node_range(0) == (0, 9));
        MINI_CHECK!(tree.order() == [0, 1, 2, 3, 4, 5, 6, 7, 8]);
    })
}

pub fn run_octree_node_count() -> TestResult {
    MINI_TEST!("Node Count", {
        use crate::SpatialOctree;
        use crate::Point;

        // 9 points on X: root cube size 8, spacing 4 -> 2 cells, first-wins accepts x=0
        // and x=4; the 7 leftovers split into two octants -> two leaf children
        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 4);

        MINI_CHECK!(tree.node_count() == 3);
    })
}

pub fn run_octree_node_cube() -> TestResult {
    MINI_TEST!("Node Cube", {
        use crate::SpatialOctree;
        use crate::Point;

        // Root cube: aabb (0..8, 0, 0) grown to a cube -> center (4,0,0), size 8.
        // Child in octant 6 (x<cx, y>=cy, z>=cz): min (0,0,0), size 4 -> center (2,2,2)
        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 4);
        let (center, size) = tree.node_cube(0);
        let (child_center, child_size) = tree.node_cube(1);

        MINI_CHECK!(TOLERANCE.is_close(center[0], 4.0) && TOLERANCE.is_close(center[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(size, 8.0));
        MINI_CHECK!(TOLERANCE.is_close(child_center[0], 2.0) && TOLERANCE.is_close(child_center[2], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(child_size, 4.0));
    })
}

pub fn run_octree_node_level() -> TestResult {
    MINI_TEST!("Node Level", {
        use crate::SpatialOctree;
        use crate::Point;

        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 4);

        MINI_CHECK!(tree.node_level(0) == 0);
        MINI_CHECK!(tree.node_level(1) == 1);
        MINI_CHECK!(tree.node_level(2) == 1);
    })
}

pub fn run_octree_node_spacing() -> TestResult {
    MINI_TEST!("Node Spacing", {
        use crate::SpatialOctree;
        use crate::Point;

        // Spacing halves per level, like Potree
        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 4);

        MINI_CHECK!(TOLERANCE.is_close(tree.node_spacing(0), 4.0));
        MINI_CHECK!(TOLERANCE.is_close(tree.node_spacing(1), 2.0));
        MINI_CHECK!(TOLERANCE.is_close(tree.node_spacing(2), 2.0));
    })
}

pub fn run_octree_node_range() -> TestResult {
    MINI_TEST!("Node Range", {
        use crate::SpatialOctree;
        use crate::Point;

        // Every node's points are contiguous in order(): root [0..2), children after
        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 4);

        MINI_CHECK!(tree.node_range(0) == (0, 2));
        MINI_CHECK!(tree.node_range(1) == (2, 3));
        MINI_CHECK!(tree.node_range(2) == (5, 4));
    })
}

pub fn run_octree_children() -> TestResult {
    MINI_TEST!("Children", {
        use crate::SpatialOctree;
        use crate::Point;

        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 4);

        MINI_CHECK!(tree.children(0) == [1, 2]);
        MINI_CHECK!(tree.children(1).is_empty());
    })
}

pub fn run_octree_order() -> TestResult {
    MINI_TEST!("Order", {
        use crate::SpatialOctree;
        use crate::Point;

        // Root's grid accepts x=0 and x=4 (first point wins its cell); the octant
        // leaves absorb the rest in input order
        let pts: Vec<Point> = (0..9).map(|x| Point::new(x as f64, 0.0, 0.0)).collect();
        let tree = SpatialOctree::new(pts, 4.0, 4);

        MINI_CHECK!(tree.order() == [0, 4, 1, 2, 3, 5, 6, 7, 8]);
    })
}

pub fn run_octree_from_coords() -> TestResult {
    MINI_TEST!("From Coords", {
        use crate::SpatialOctree;

        // The flat-array constructor is the renderer's path (no per-point Point allocs)
        let mut coords: Vec<f64> = Vec::new();
        for x in 0..9 {
            coords.extend_from_slice(&[x as f64, 0.0, 0.0]);
        }
        let tree = SpatialOctree::from_coords(&coords, 4.0, 4);

        MINI_CHECK!(tree.node_count() == 3);
        MINI_CHECK!(tree.order() == [0, 4, 1, 2, 3, 5, 6, 7, 8]);
    })
}

REGISTER_MINI_TEST!("SpatialOctree", "Constructor", crate::spatial_octree_test::run_octree_constructor);
REGISTER_MINI_TEST!("SpatialOctree", "Node Count", crate::spatial_octree_test::run_octree_node_count);
REGISTER_MINI_TEST!("SpatialOctree", "Node Cube", crate::spatial_octree_test::run_octree_node_cube);
REGISTER_MINI_TEST!("SpatialOctree", "Node Level", crate::spatial_octree_test::run_octree_node_level);
REGISTER_MINI_TEST!("SpatialOctree", "Node Spacing", crate::spatial_octree_test::run_octree_node_spacing);
REGISTER_MINI_TEST!("SpatialOctree", "Node Range", crate::spatial_octree_test::run_octree_node_range);
REGISTER_MINI_TEST!("SpatialOctree", "Children", crate::spatial_octree_test::run_octree_children);
REGISTER_MINI_TEST!("SpatialOctree", "Order", crate::spatial_octree_test::run_octree_order);
REGISTER_MINI_TEST!("SpatialOctree", "From Coords", crate::spatial_octree_test::run_octree_from_coords);
