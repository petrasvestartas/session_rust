use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

pub fn run_octree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 16);

        MINI_CHECK!(tree.node_count() == 1);
        MINI_CHECK!(tree.node_range(0) == (0, 9));
        MINI_CHECK!(tree.order() == [0, 1, 2, 3, 4, 5, 6, 7, 8]);
    })
}

pub fn run_octree_node_count() -> TestResult {
    MINI_TEST!("Node Count", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 4);

        MINI_CHECK!(tree.node_count() == 3);
    })
}

pub fn run_octree_node_cube() -> TestResult {
    MINI_TEST!("Node Cube", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 4);
        let cube = tree.node_cube(0);
        let child = tree.node_cube(1);

        MINI_CHECK!(TOLERANCE.is_close(cube.0[0], 4.0) && TOLERANCE.is_close(cube.0[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(cube.1, 8.0));
        MINI_CHECK!(TOLERANCE.is_close(child.0[0], 2.0) && TOLERANCE.is_close(child.0[2], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(child.1, 4.0));
    })
}

pub fn run_octree_node_level() -> TestResult {
    MINI_TEST!("Node Level", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 4);

        MINI_CHECK!(tree.node_level(0) == 0);
        MINI_CHECK!(tree.node_level(1) == 1);
        MINI_CHECK!(tree.node_level(2) == 1);
    })
}

pub fn run_octree_node_spacing() -> TestResult {
    MINI_TEST!("Node Spacing", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 4);

        MINI_CHECK!(TOLERANCE.is_close(tree.node_spacing(0), 4.0));
        MINI_CHECK!(TOLERANCE.is_close(tree.node_spacing(1), 2.0));
        MINI_CHECK!(TOLERANCE.is_close(tree.node_spacing(2), 2.0));
    })
}

pub fn run_octree_node_range() -> TestResult {
    MINI_TEST!("Node Range", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 4);

        MINI_CHECK!(tree.node_range(0) == (0, 2));
        MINI_CHECK!(tree.node_range(1) == (2, 3));
        MINI_CHECK!(tree.node_range(2) == (5, 4));
    })
}

pub fn run_octree_children() -> TestResult {
    MINI_TEST!("Children", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 4);

        MINI_CHECK!(tree.children(0) == [1, 2]);
        MINI_CHECK!(tree.children(1).is_empty());
    })
}

pub fn run_octree_order() -> TestResult {
    MINI_TEST!("Order", {
        use crate::Point;
        use crate::SpatialOctree;

        let mut pts: Vec<Point> = Vec::new();

        for x in 0..9 {
            pts.push(Point::new(x as f64, 0.0, 0.0));
        }

        let tree = SpatialOctree::new(&pts, 4.0, 4);

        MINI_CHECK!(tree.order() == [0, 4, 1, 2, 3, 5, 6, 7, 8]);
    })
}

pub fn run_octree_from_coords() -> TestResult {
    MINI_TEST!("From Coords", {
        use crate::SpatialOctree;

        let mut coords: Vec<f64> = Vec::new();

        for x in 0..9 {
            coords.push(x as f64);
            coords.push(0.0);
            coords.push(0.0);
        }

        let tree = SpatialOctree::from_coords(&coords, 4.0, 4);

        MINI_CHECK!(tree.node_count() == 3);
        MINI_CHECK!(tree.order() == [0, 4, 1, 2, 3, 5, 6, 7, 8]);
    })
}

REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Constructor",
    crate::spatial_octree_test::run_octree_constructor
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Node Count",
    crate::spatial_octree_test::run_octree_node_count
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Node Cube",
    crate::spatial_octree_test::run_octree_node_cube
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Node Level",
    crate::spatial_octree_test::run_octree_node_level
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Node Spacing",
    crate::spatial_octree_test::run_octree_node_spacing
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Node Range",
    crate::spatial_octree_test::run_octree_node_range
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Children",
    crate::spatial_octree_test::run_octree_children
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "Order",
    crate::spatial_octree_test::run_octree_order
);
REGISTER_MINI_TEST!(
    "SpatialOctree",
    "From Coords",
    crate::spatial_octree_test::run_octree_from_coords
);
