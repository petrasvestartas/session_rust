use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;


pub fn run_aabbtree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::AABB;
        use crate::Closest;

        // AABBTree: O(n log n) build, O(log n) cull — prune candidates before exact test
        let box0 = AABB::new(0.0, 0.0, 0.0, 0.5, 0.5, 0.5);
        let box1 = AABB::new(5.0, 0.0, 0.0, 0.5, 0.5, 0.5);
        let box2 = AABB::new(10.0, 0.0, 0.0, 0.5, 0.5, 0.5);
        let pairs = Closest::boxes_closest(&[box0, box1, box2], 0.0);

        MINI_CHECK!(pairs.is_empty());

        let box_a = AABB::new(0.0, 0.0, 0.0, 0.5, 0.5, 0.5);
        let box_b = AABB::new(1.0, 0.0, 0.0, 0.5, 0.5, 0.5);
        let pairs_near = Closest::boxes_closest(&[box_a, box_b], 0.0);

        MINI_CHECK!(pairs_near.len() == 1);
        MINI_CHECK!(pairs_near[0].0 == 0);
        MINI_CHECK!(pairs_near[0].1 == 1);
    })
}

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
        MINI_CHECK!(a.intersects(&AABB::new(0.5, 0.0, 0.0, 0.5, 0.5, 0.5)));
        MINI_CHECK!(!a.intersects(&AABB::new(10.0, 0.0, 0.0, 0.5, 0.5, 0.5)));
        let mut a = a;
        let b = AABB::new(5.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        a.union_with(&b);
        MINI_CHECK!(a.min_point() == Point::new(-1.0, -2.0, -3.0));
        MINI_CHECK!(a.max_point() == Point::new(6.0, 2.0, 3.0));
        let c = AABB::merge(AABB::new(0.0, 0.0, 0.0, 1.0, 1.0, 1.0), AABB::new(4.0, 0.0, 0.0, 1.0, 1.0, 1.0));
        MINI_CHECK!(c.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(c.max_point() == Point::new(5.0, 1.0, 1.0));
    })
}

pub fn run_aabb_from_geometry() -> TestResult {
    MINI_TEST!("From Geometry", {
        use crate::AABB;
        use crate::Color;
        use crate::Line;
        use crate::NurbsCurve;
        use crate::NurbsSurface;
        use crate::Point;
        use crate::PointCloud;
        use crate::Polyline;
        use crate::Primitives;
        use crate::Vector;

        let a_pt = AABB::from_point(&Point::new(1.0, 2.0, 3.0), 0.5);

        MINI_CHECK!(a_pt.center() == Point::new(1.0, 2.0, 3.0));
        MINI_CHECK!(TOLERANCE.is_close(a_pt.hx, 0.5));

        let a_pts = AABB::from_points(&[
            Point::new(0.0, 0.0, 0.0),
            Point::new(3.0, 4.0, 5.0),
        ], 0.0);

        MINI_CHECK!(a_pts.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pts.max_point() == Point::new(3.0, 4.0, 5.0));

        let ln = Line::new(0.0, 0.0, 0.0, 4.0, 0.0, 0.0);
        let a_line = AABB::from_line(&ln, 1.0);

        MINI_CHECK!(a_line.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(a_line.max_point() == Point::new(5.0, 1.0, 1.0));

        let pl = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 2.0, 0.0),
        ]);
        let a_pl = AABB::from_polyline(&pl, 0.0);

        MINI_CHECK!(a_pl.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pl.max_point() == Point::new(2.0, 2.0, 0.0));

        let cube = Primitives::cube(2.0);
        let a_mesh = AABB::from_mesh(&cube, 0.0);

        MINI_CHECK!(a_mesh.min_point() == Point::new(-1.0, -1.0, -1.0));
        MINI_CHECK!(a_mesh.max_point() == Point::new(1.0, 1.0, 1.0));

        let pc = PointCloud::new(
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(4.0, 2.0, 6.0),
            ],
            vec![
                Vector::new(0.0, 0.0, 1.0),
                Vector::new(0.0, 0.0, 1.0),
            ],
            vec![
                Color::new(255, 0, 0, 255),
                Color::new(0, 255, 0, 255),
            ],
        );
        let a_pc = AABB::from_pointcloud(&pc, 0.0);

        MINI_CHECK!(a_pc.min_point() == Point::new(0.0, 0.0, 0.0));
        MINI_CHECK!(a_pc.max_point() == Point::new(4.0, 2.0, 6.0));

        let curve = NurbsCurve::create(false, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(3.0, 0.0, 0.0),
        ]);
        let a_nc = AABB::from_nurbscurve(&curve, 0.5, false);

        MINI_CHECK!(a_nc.is_valid());
        MINI_CHECK!(a_nc.contains(&Point::new(1.5, 0.0, 0.0)));

        let surf = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(0.0, 2.0, 0.0),
            Point::new(2.0, 2.0, 2.0),
        ]).unwrap();
        let a_ns = AABB::from_nurbssurface(&surf, 0.0);

        MINI_CHECK!(a_ns.is_valid());
        MINI_CHECK!(TOLERANCE.is_close(a_ns.volume(), 8.0));
    })
}

pub fn run_aabbtree_query_aabb() -> TestResult {
    MINI_TEST!("Query Aabb", {
        use crate::Closest;
        use crate::Mesh;
        use crate::Point;

        let mut m = Mesh::new();
        let vk0 = m.add_vertex(Point::new(0.0, 0.0, 0.0), None);
        let vk1 = m.add_vertex(Point::new(1.0, 0.0, 0.0), None);
        let vk2 = m.add_vertex(Point::new(0.5, 1.0, 0.0), None);
        let vk3 = m.add_vertex(Point::new(20.0, 0.0, 0.0), None);
        let vk4 = m.add_vertex(Point::new(21.0, 0.0, 0.0), None);
        let vk5 = m.add_vertex(Point::new(20.5, 1.0, 0.0), None);
        m.add_face(vec![vk0, vk1, vk2], None);
        m.add_face(vec![vk3, vk4, vk5], None);
        let (cp_near, fk_near, d_near) = Closest::mesh_point_aabb(&m, &Point::new(0.5, 0.25, 0.0));
        let (cp_far, fk_far, d_far) = Closest::mesh_point_aabb(&m, &Point::new(20.5, 0.25, 0.0));

        MINI_CHECK!(TOLERANCE.is_close(d_near, 0.0));
        MINI_CHECK!(TOLERANCE.is_close(d_far, 0.0));
        MINI_CHECK!(fk_near != fk_far);
        MINI_CHECK!(cp_near[0] < 2.0);
        MINI_CHECK!(cp_far[0] > 15.0);
    })
}

REGISTER_MINI_TEST!("AABBTree", "Constructor", crate::aabb_test::run_aabbtree_constructor);
REGISTER_MINI_TEST!("AABBTree", "Build Empty", crate::aabb_test::run_aabbtree_build_empty);
REGISTER_MINI_TEST!("AABBTree", "Build Single", crate::aabb_test::run_aabbtree_build_single);
REGISTER_MINI_TEST!("AABBTree", "Build Multiple", crate::aabb_test::run_aabbtree_build_multiple);
REGISTER_MINI_TEST!("AABBTree", "Node Count", crate::aabb_test::run_aabbtree_node_count);
REGISTER_MINI_TEST!("AABBTree", "Mesh Point Aabb", crate::aabb_test::run_aabbtree_mesh_point_aabb);
REGISTER_MINI_TEST!("AABBTree", "Mesh Point Aabb Matches Bvh", crate::aabb_test::run_aabbtree_mesh_point_aabb_matches_bvh);
REGISTER_MINI_TEST!("AABBTree", "Query Aabb", crate::aabb_test::run_aabbtree_query_aabb);
REGISTER_MINI_TEST!("Aabb", "Constructor", crate::aabb_test::run_aabb_constructor);
REGISTER_MINI_TEST!("Aabb", "From Geometry", crate::aabb_test::run_aabb_from_geometry);
