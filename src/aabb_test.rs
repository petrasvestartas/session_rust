use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_aabbtree_build_empty() -> TestResult {
    MINI_TEST!("Build Empty", {
        use crate::Closest;
        use crate::Mesh;
        use crate::Point;

        let m = Mesh::new();
        let (_cp, _fk, d) = Closest::mesh_point_aabb(&m, &Point::new(0.0, 0.0, 0.0));

        MINI_CHECK!(d == f64::INFINITY);
    })
}

pub fn run_aabbtree_build_single() -> TestResult {
    MINI_TEST!("Build Single", {
        use crate::Closest;
        use crate::Mesh;
        use crate::Point;

        let mut m = Mesh::new();
        let vk0 = m.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let vk1 = m.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let vk2 = m.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        m.add_face(vec![vk0, vk1, vk2], None);
        let (_cp, _fk, d) = Closest::mesh_point_aabb(&m, &Point::new(0.0, 0.0, 1.0));

        MINI_CHECK!(d > 0.0);
        MINI_CHECK!(TOLERANCE.is_close(d, 1.0));
    })
}

pub fn run_aabbtree_build_multiple() -> TestResult {
    MINI_TEST!("Build Multiple", {
        use crate::Closest;
        use crate::Mesh;
        use crate::Point;

        let mut m = Mesh::new();
        let vk0 = m.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let vk1 = m.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let vk2 = m.add_vertex(Point::new(0.0, 1.0, 0.0), None);
        let vk3 = m.add_vertex(Point::new(5.0, 0.0, 0.0), None);
        let vk4 = m.add_vertex(Point::new(6.0, 0.0, 0.0), None);
        let vk5 = m.add_vertex(Point::new(5.0, 1.0, 0.0), None);
        let vk6 = m.add_vertex(Point::new(10.0, 0.0, 0.0), None);
        let vk7 = m.add_vertex(Point::new(11.0, 0.0, 0.0), None);
        let vk8 = m.add_vertex(Point::new(10.0, 1.0, 0.0), None);
        m.add_face(vec![vk0, vk1, vk2], None);
        m.add_face(vec![vk3, vk4, vk5], None);
        m.add_face(vec![vk6, vk7, vk8], None);
        let (_cp, _fk, d) = Closest::mesh_point_aabb(&m, &Point::new(0.5, 0.0, 0.0));

        MINI_CHECK!(d < 0.5);
    })
}

pub fn run_aabbtree_node_count() -> TestResult {
    MINI_TEST!("Node Count", {
        use crate::Closest;
        use crate::Mesh;
        use crate::Point;

        let mut m = Mesh::new();
        let mut vkeys: Vec<usize> = Vec::new();
        for i in 0..100 {
            vkeys.push(m.add_vertex(Point::new(i as f64, 0.0, 0.0), None));
            vkeys.push(m.add_vertex(Point::new(i as f64 + 0.5, 0.5, 0.0), None));
            vkeys.push(m.add_vertex(Point::new(i as f64, 0.5, 0.0), None));
        }
        for i in 0..100 {
            m.add_face(vec![vkeys[i*3], vkeys[i*3+1], vkeys[i*3+2]], None);
        }
        let (_cp, _fk, d) = Closest::mesh_point_aabb(&m, &Point::new(50.0, 0.0, 0.0));

        MINI_CHECK!(d < 0.5);
    })
}

pub fn run_aabbtree_mesh_point_aabb() -> TestResult {
    MINI_TEST!("Mesh Point Aabb", {
        use crate::Closest;
        use crate::Primitives;
        use crate::Point;

        let m = Primitives::cube(2.0);
        let (cp1, _fk1, d1) = Closest::mesh_point_aabb(&m, &Point::new(0.0, 0.0, 2.0));

        MINI_CHECK!(TOLERANCE.is_close(cp1[2], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d1, 1.0));
        let (_cp2, _fk2, d2) = Closest::mesh_point_aabb(&m, &Point::new(1.0, 1.0, 1.0));
        MINI_CHECK!(TOLERANCE.is_close(d2, 0.0));
    })
}

pub fn run_aabbtree_mesh_point_aabb_matches_bvh() -> TestResult {
    MINI_TEST!("Mesh Point Aabb Matches Bvh", {
        use crate::Closest;
        use crate::Primitives;
        use crate::Point;

        let m = Primitives::cube(2.0);
        let tp = Point::new(0.3, 0.7, 1.5);
        let (cp_bvh, _fk_bvh, d_bvh) = Closest::mesh_point(&m, &tp);
        let (cp_aabb, _fk_aabb, d_aabb) = Closest::mesh_point_aabb(&m, &tp);

        MINI_CHECK!(TOLERANCE.is_close(d_bvh, d_aabb));
        MINI_CHECK!(TOLERANCE.is_close(cp_bvh[0], cp_aabb[0]));
        MINI_CHECK!(TOLERANCE.is_close(cp_bvh[1], cp_aabb[1]));
        MINI_CHECK!(TOLERANCE.is_close(cp_bvh[2], cp_aabb[2]));
    })
}

pub fn run_aabb_constructor() -> crate::mini_test::TestResult {
    use crate::tolerance::TOLERANCE;
    use crate::{AABB, Point};
    MINI_TEST!("Constructor", {
        // AABB(0,0,0, 1,2,3) — dims 2×4×6
        let a = AABB::new(0.0, 0.0, 0.0, 1.0, 2.0, 3.0);

        MINI_CHECK!(TOLERANCE.is_close(a.area(), 88.0));
        MINI_CHECK!(a.center() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(a.diagonal(), 2.0 * 14.0_f64.sqrt()));
        MINI_CHECK!(a.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(a.volume(), 48.0));
        MINI_CHECK!(a.closest_point(&Point::new(0.0, 0.0, 0.0)) == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a.closest_point(&Point::new(10.0, 0.0, 0.0)) == Point::new(1.0, 0.0, 0.0));
        MINI_CHECK!(a.contains(&Point::new(0.0, 0.0, 0.0)));
        MINI_CHECK!(!a.contains(&Point::new(10.0, 0.0, 0.0)));
        MINI_CHECK!(a.corner(false, false, false) == Point::new(-1.0, -2.0, -3.0));
        MINI_CHECK!(a.corner(true, true, true) == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(a.get_corners().len() == 8);
        MINI_CHECK!(a.get_edges().len() == 12);
        MINI_CHECK!(a.point_at(1.0, 0.0, 0.0) == Point::new(1.0, 0.0, 0.0));
        MINI_CHECK!(a.point_at(0.0, 0.0, 0.0) == Point::new(0.0, 0.0, 0.0));
        let mut a = a;
        let b = AABB::new(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        a.union_with(&b);
        MINI_CHECK!(a.min_point() == Point::new(-1.0, -2.0, -3.0));
        MINI_CHECK!(a.max_point() == Point::new(6.0, 2.0, 3.0));
    })
}

REGISTER_MINI_TEST!("AABBTree", "Build Empty", crate::aabb_test::run_aabbtree_build_empty);
REGISTER_MINI_TEST!("AABBTree", "Build Single", crate::aabb_test::run_aabbtree_build_single);
REGISTER_MINI_TEST!("AABBTree", "Build Multiple", crate::aabb_test::run_aabbtree_build_multiple);
REGISTER_MINI_TEST!("AABBTree", "Node Count", crate::aabb_test::run_aabbtree_node_count);
REGISTER_MINI_TEST!("AABBTree", "Mesh Point Aabb", crate::aabb_test::run_aabbtree_mesh_point_aabb);
REGISTER_MINI_TEST!("AABBTree", "Mesh Point Aabb Matches Bvh", crate::aabb_test::run_aabbtree_mesh_point_aabb_matches_bvh);
REGISTER_MINI_TEST!("Aabb", "Constructor", crate::aabb_test::run_aabb_constructor);
