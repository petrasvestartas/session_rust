use crate::mesh::Mesh;
use crate::point::Point;
use crate::vector::Vector;
use std::collections::BTreeSet;

/// Twice the signed area of o-a-b in XY, positive for a left turn
fn cross_2d(o: &Point, a: &Point, b: &Point) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

/// Appends point i to the chain after popping every tail that no longer turns left towards it
fn extend_chain(points: &[Point], chain: &mut Vec<usize>, i: usize) {
    while chain.len() >= 2
        && cross_2d(
            &points[chain[chain.len() - 2]],
            &points[chain[chain.len() - 1]],
            &points[i],
        ) <= 0.0
    {
        chain.pop();
    }
    chain.push(i);
}

/// Six times the signed volume of a-b-c-d, positive when d is on the normal side of a-b-c
fn signed_volume(a: &Point, b: &Point, c: &Point, d: &Point) -> f64 {
    (b - a).cross(&(c - a)).dot(&(d - a))
}

/// Indices of the points above the face a-b-c
fn visible_from(
    indices: &[usize],
    points: &[Point],
    a: &Point,
    b: &Point,
    c: &Point,
) -> Vec<usize> {
    let mut result = Vec::new();
    for &i in indices {
        if signed_volume(a, b, c, &points[i]) > 1e-10 {
            result.push(i);
        }
    }
    result
}

/// Index of the point highest above the face a-b-c, None when none is above
fn farthest_point(
    indices: &[usize],
    points: &[Point],
    a: &Point,
    b: &Point,
    c: &Point,
) -> Option<usize> {
    let mut best = None;
    let mut best_volume = 0.0;
    for &i in indices {
        let volume = signed_volume(a, b, c, &points[i]);
        if volume > best_volume {
            best_volume = volume;
            best = Some(i);
        }
    }
    best
}

/// Hull faces over a-b-c: the face itself when no candidate is above it, else the three faces to the farthest candidate, recursively
fn quickhull_faces(
    points: &[Point],
    indices: &[usize],
    a: usize,
    b: usize,
    c: usize,
    faces: &mut Vec<[usize; 3]>,
) {
    let visible = visible_from(indices, points, &points[a], &points[b], &points[c]);
    let Some(apex) = farthest_point(&visible, points, &points[a], &points[b], &points[c]) else {
        faces.push([a, b, c]);
        return;
    };
    quickhull_faces(
        points,
        &visible_from(&visible, points, &points[a], &points[b], &points[apex]),
        a,
        b,
        apex,
        faces,
    );
    quickhull_faces(
        points,
        &visible_from(&visible, points, &points[b], &points[c], &points[apex]),
        b,
        c,
        apex,
        faces,
    );
    quickhull_faces(
        points,
        &visible_from(&visible, points, &points[c], &points[a], &points[apex]),
        c,
        a,
        apex,
        faces,
    );
}

/// Convex hull: monotone chain in XY for 2D, quickhull for 3D
pub struct ConvexHull;

impl ConvexHull {
    /// Counter-clockwise hull of the points projected to XY, collinear points dropped; fewer than three points come back as given
    pub fn hull_2d(points: &[Point]) -> Vec<Point> {
        let n = points.len();
        if n < 3 {
            return points.to_vec();
        }
        let mut order: Vec<usize> = (0..n).collect();
        order.sort_by(|&a, &b| {
            points[a][0]
                .total_cmp(&points[b][0])
                .then(points[a][1].total_cmp(&points[b][1]))
        });
        let mut lower = Vec::new();
        for &i in &order {
            extend_chain(points, &mut lower, i);
        }
        let mut upper = Vec::new();
        for &i in order.iter().rev() {
            extend_chain(points, &mut upper, i);
        }
        lower.pop();
        upper.pop();
        let mut hull = Vec::new();
        for &i in &lower {
            hull.push(points[i].clone());
        }
        for &i in &upper {
            hull.push(points[i].clone());
        }
        hull
    }

    /// Triangle mesh of the hull with outward faces; fewer than four points give the points and, for three, one face
    pub fn hull_3d(points: &[Point]) -> Mesh {
        let n = points.len();
        let mut mesh = Mesh::new();
        if n < 4 {
            let mut vkeys = Vec::new();
            for point in points {
                vkeys.push(mesh.add_vertex(point.clone(), None));
            }
            if n == 3 {
                mesh.add_face(vkeys, None);
            }
            return mesh;
        }
        let mut p0 = 0;
        for i in 1..n {
            if points[i][0] < points[p0][0] {
                p0 = i;
            }
        }
        let mut p1 = 0;
        for i in 1..n {
            if (&points[i] - &points[p0]).magnitude_squared()
                > (&points[p1] - &points[p0]).magnitude_squared()
            {
                p1 = i;
            }
        }
        let axis: Vector = &points[p1] - &points[p0];
        let mut p2 = 0;
        let mut best_distance = -1.0;
        for i in 0..n {
            if i == p0 || i == p1 {
                continue;
            }
            let distance = axis.cross(&(&points[i] - &points[p0])).magnitude_squared();
            if distance > best_distance {
                best_distance = distance;
                p2 = i;
            }
        }
        let mut p3 = 0;
        let mut best_volume = -1.0;
        for i in 0..n {
            if i == p0 || i == p1 || i == p2 {
                continue;
            }
            let volume = signed_volume(&points[p0], &points[p1], &points[p2], &points[i]).abs();
            if volume > best_volume {
                best_volume = volume;
                p3 = i;
            }
        }
        if best_distance <= 1e-20 || best_volume <= 1e-20 {
            for point in points {
                mesh.add_vertex(point.clone(), None);
            }
            return mesh;
        }
        if signed_volume(&points[p0], &points[p1], &points[p2], &points[p3]) > 0.0 {
            std::mem::swap(&mut p1, &mut p2);
        }
        let mut rest = Vec::new();
        for i in 0..n {
            if i != p0 && i != p1 && i != p2 && i != p3 {
                rest.push(i);
            }
        }
        let mut faces = Vec::new();
        quickhull_faces(points, &rest, p0, p1, p2, &mut faces);
        quickhull_faces(points, &rest, p0, p3, p1, &mut faces);
        quickhull_faces(points, &rest, p1, p3, p2, &mut faces);
        quickhull_faces(points, &rest, p2, p3, p0, &mut faces);
        let mut used: BTreeSet<usize> = BTreeSet::new();
        for face in &faces {
            used.extend(*face);
        }
        let mut vkeys = vec![0; n];
        for &i in &used {
            vkeys[i] = mesh.add_vertex(points[i].clone(), None);
        }
        for face in &faces {
            mesh.add_face(vec![vkeys[face[0]], vkeys[face[1]], vkeys[face[2]]], None);
        }
        mesh
    }
}
