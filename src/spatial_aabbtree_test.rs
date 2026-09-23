use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};

pub fn run_spatial_aabbtree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Closest;
        use crate::AABB;

        let boxes = vec![
            AABB::new(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
            AABB::new(5.0, 0.0, 0.0, 0.5, 0.5, 0.5),
            AABB::new(10.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        ];
        let pairs = Closest::boxes_closest(&boxes, 0.0);

        MINI_CHECK!(pairs.is_empty());

        let boxes_near = vec![
            AABB::new(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
            AABB::new(1.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        ];
        let pairs_near = Closest::boxes_closest(&boxes_near, 0.0);

        MINI_CHECK!(pairs_near.len() == 1);
        MINI_CHECK!(pairs_near[0].0 == 0);
        MINI_CHECK!(pairs_near[0].1 == 1);
    })
}

pub fn run_spatial_aabbtree_build_empty() -> TestResult {
    MINI_TEST!("Build Empty", {
        use crate::SpatialAABBTree;

        let mut tree = SpatialAABBTree::new();
        tree.build(&[]);

        MINI_CHECK!(tree.empty());
    })
}

pub fn run_spatial_aabbtree_build_single() -> TestResult {
    MINI_TEST!("Build Single", {
        use crate::SpatialAABBTree;
        use crate::AABB;

        let aabb = AABB::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        let mut tree = SpatialAABBTree::new();
        tree.build(&[aabb]);

        MINI_CHECK!(tree.size() == 1);
        MINI_CHECK!(tree.nodes[0].object_id == 0);
    })
}

pub fn run_spatial_aabbtree_build_multiple() -> TestResult {
    MINI_TEST!("Build Multiple", {
        use crate::SpatialAABBTree;
        use crate::AABB;

        let aabbs = vec![
            AABB::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0),
            AABB::new(5.0, 0.0, 0.0, 1.0, 1.0, 1.0),
            AABB::new(10.0, 0.0, 0.0, 1.0, 1.0, 1.0),
        ];

        let mut tree = SpatialAABBTree::new();
        tree.build(&aabbs);

        MINI_CHECK!(tree.size() == 5);
        MINI_CHECK!(tree.nodes[0].object_id == -1);
    })
}

pub fn run_spatial_aabbtree_node_count() -> TestResult {
    MINI_TEST!("Node Count", {
        use crate::SpatialAABBTree;
        use crate::AABB;

        let mut aabbs: Vec<AABB> = Vec::new();

        for i in 0..100 {
            aabbs.push(AABB::new(i as f64, 0.0, 0.0, 0.5, 0.5, 0.5));
        }

        let mut tree = SpatialAABBTree::new();
        tree.build(&aabbs);

        MINI_CHECK!(tree.size() == 199);
    })
}

pub fn run_spatial_aabbtree_mesh_point_aabb() -> TestResult {
    MINI_TEST!("Mesh Point Aabb", {
        use crate::Closest;
        use crate::Point;
        use crate::Primitives;

        let mut m = Primitives::cube(2.0);

        let (cp1, _fk1, d1) = Closest::mesh_point_aabb(&mut m, &Point::new(0.0, 0.0, 2.0));

        MINI_CHECK!(TOLERANCE.is_close(cp1[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d1, 1.0));

        let d2 = Closest::mesh_point_aabb(&mut m, &Point::new(1.0, 1.0, 1.0)).2;

        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
    })
}

pub fn run_spatial_aabbtree_mesh_point_aabb_matches_bvh() -> TestResult {
    MINI_TEST!("Mesh Point Aabb Matches Bvh", {
        use crate::Closest;
        use crate::Point;
        use crate::Primitives;

        let mut m = Primitives::cube(2.0);
        let tp = Point::new(0.3, 0.7, 1.5);

        let (cp_bvh, _fk_bvh, d_bvh) = Closest::mesh_point(&mut m, &tp);

        let (cp_aabb, _fk_aabb, d_aabb) = Closest::mesh_point_aabb(&mut m, &tp);

        MINI_CHECK!(TOLERANCE.is_close(d_bvh, d_aabb));
        MINI_CHECK!(TOLERANCE.is_close(cp_bvh[0], cp_aabb[0]));
        MINI_CHECK!(TOLERANCE.is_close(cp_bvh[1], cp_aabb[1]));
        MINI_CHECK!(TOLERANCE.is_close(cp_bvh[2], cp_aabb[2]));
    })
}

pub fn run_spatial_aabbtree_query_aabb() -> TestResult {
    MINI_TEST!("Query Aabb", {
        use crate::SpatialAABBTree;
        use crate::AABB;

        let aabbs = vec![
            AABB::new(0.0, 0.0, 0.0, 0.5, 0.5, 0.5),
            AABB::new(5.0, 0.0, 0.0, 0.5, 0.5, 0.5),
            AABB::new(10.0, 0.0, 0.0, 0.5, 0.5, 0.5),
        ];

        let mut tree = SpatialAABBTree::new();
        tree.build(&aabbs);

        let hits = tree.query_aabb(&AABB::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0));

        MINI_CHECK!(hits.len() == 1);
        MINI_CHECK!(hits[0] == 0);

        let none = tree.query_aabb(&AABB::new(20.0, 0.0, 0.0, 0.5, 0.5, 0.5));

        MINI_CHECK!(none.is_empty());

        let all = tree.query_aabb(&AABB::new(5.0, 0.0, 0.0, 10.0, 1.0, 1.0));

        MINI_CHECK!(all.len() == 3);
    })
}

REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Constructor",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_constructor
);
REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Build Empty",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_build_empty
);
REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Build Single",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_build_single
);
REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Build Multiple",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_build_multiple
);
REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Node Count",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_node_count
);
REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Mesh Point Aabb",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_mesh_point_aabb
);
REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Mesh Point Aabb Matches Bvh",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_mesh_point_aabb_matches_bvh
);
REGISTER_MINI_TEST!(
    "SpatialAABBTree",
    "Query Aabb",
    crate::spatial_aabbtree_test::run_spatial_aabbtree_query_aabb
);
