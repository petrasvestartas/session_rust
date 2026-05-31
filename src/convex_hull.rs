use crate::mesh::Mesh;
use crate::point::Point;
use std::collections::HashSet;

pub struct ConvexHull;

impl ConvexHull {
    // -------------------------------------------------------------------------
    // 2D Graham scan (projects to XY)
    // -------------------------------------------------------------------------

    fn cross_2d(o: &Point, a: &Point, b: &Point) -> f64 {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    }

    pub fn hull_2d(points: &[Point]) -> Vec<Point> {
        let n = points.len();
        if n < 3 {
            return points.to_vec();
        }
        let mut pts: Vec<&Point> = points.iter().collect();
        pts.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap().then(a[1].partial_cmp(&b[1]).unwrap()));
        let mut lower: Vec<&Point> = Vec::new();
        for p in &pts {
            while lower.len() >= 2 && Self::cross_2d(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
                lower.pop();
            }
            lower.push(p);
        }
        let mut upper: Vec<&Point> = Vec::new();
        for p in pts.iter().rev() {
            while upper.len() >= 2 && Self::cross_2d(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
                upper.pop();
            }
            upper.push(p);
        }
        lower.pop();
        upper.pop();
        lower.iter().chain(upper.iter()).map(|p| (*p).clone()).collect()
    }

    // -------------------------------------------------------------------------
    // 3D Quickhull
    // -------------------------------------------------------------------------

    fn normal(a: &Point, b: &Point, c: &Point) -> (f64, f64, f64) {
        let ax = b[0] - a[0];
        let ay = b[1] - a[1];
        let az = b[2] - a[2];
        let bx = c[0] - a[0];
        let by = c[1] - a[1];
        let bz = c[2] - a[2];
        (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx)
    }

    fn signed_volume(a: &Point, b: &Point, c: &Point, d: &Point) -> f64 {
        let (nx, ny, nz) = Self::normal(a, b, c);
        nx * (d[0] - a[0]) + ny * (d[1] - a[1]) + nz * (d[2] - a[2])
    }

    fn farthest_point(pts_idx: &[usize], points: &[Point], a: &Point, b: &Point, c: &Point) -> Option<usize> {
        let mut best_idx = None;
        let mut best_vol = 0.0_f64;
        for &i in pts_idx {
            let v = Self::signed_volume(a, b, c, &points[i]);
            if v > best_vol {
                best_vol = v;
                best_idx = Some(i);
            }
        }
        best_idx
    }

    fn visible_from(pts_idx: &[usize], points: &[Point], a: &Point, b: &Point, c: &Point) -> Vec<usize> {
        pts_idx.iter().filter(|&&i| Self::signed_volume(a, b, c, &points[i]) > 1e-10).cloned().collect()
    }

    fn quickhull_3d_faces(points: &[Point], pts_idx: &[usize], a: usize, b: usize, c: usize, faces: &mut Vec<(usize, usize, usize)>) {
        let visible = Self::visible_from(pts_idx, points, &points[a], &points[b], &points[c]);
        if visible.is_empty() {
            faces.push((a, b, c));
            return;
        }
        match Self::farthest_point(&visible, points, &points[a], &points[b], &points[c]) {
            None => {
                faces.push((a, b, c));
            }
            Some(apex) => {
                let v_ab = Self::visible_from(&visible, points, &points[a], &points[b], &points[apex]);
                let v_bc = Self::visible_from(&visible, points, &points[b], &points[c], &points[apex]);
                let v_ca = Self::visible_from(&visible, points, &points[c], &points[a], &points[apex]);
                Self::quickhull_3d_faces(points, &v_ab, a, b, apex, faces);
                Self::quickhull_3d_faces(points, &v_bc, b, c, apex, faces);
                Self::quickhull_3d_faces(points, &v_ca, c, a, apex, faces);
            }
        }
    }

    pub fn hull_3d(points: &[Point]) -> Mesh {
        let n = points.len();
        let mut mesh = Mesh::new();
        if n < 4 {
            let vkeys: Vec<usize> = points.iter().map(|p| mesh.add_vertex(p.clone(), None)).collect();
            if n == 3 {
                mesh.add_face(vkeys, None);
            }
            return mesh;
        }
        // Use first-maximum semantics (like Python's max()) for consistent tie-breaking
        let first_max = |iter: &mut dyn Iterator<Item = usize>, key: &dyn Fn(usize) -> f64| -> usize {
            iter.fold(None::<(usize, f64)>, |acc, i| {
                let d = key(i);
                match acc {
                    None => Some((i, d)),
                    Some((bi, bd)) => if d > bd { Some((i, d)) } else { Some((bi, bd)) },
                }
            }).map(|(i, _)| i).unwrap()
        };
        let p0 = first_max(&mut (0..n), &|i| -points[i][0]);
        let p1 = first_max(&mut (0..n), &|i| {
            (points[i][0] - points[p0][0]).powi(2) + (points[i][1] - points[p0][1]).powi(2) + (points[i][2] - points[p0][2]).powi(2)
        });
        let ax = points[p1][0] - points[p0][0];
        let ay = points[p1][1] - points[p0][1];
        let az = points[p1][2] - points[p0][2];
        let dist_to_line = |i: usize| -> f64 {
            let bx = points[i][0] - points[p0][0];
            let by = points[i][1] - points[p0][1];
            let bz = points[i][2] - points[p0][2];
            let cx = ay * bz - az * by;
            let cy = az * bx - ax * bz;
            let cz = ax * by - ay * bx;
            cx * cx + cy * cy + cz * cz
        };
        let p2 = first_max(&mut (0..n).filter(|&i| i != p0 && i != p1), &dist_to_line);
        let p3 = first_max(&mut (0..n).filter(|&i| i != p0 && i != p1 && i != p2), &|i| {
            Self::signed_volume(&points[p0], &points[p1], &points[p2], &points[i]).abs()
        });

        let (mut p1, mut p2) = (p1, p2);
        if Self::signed_volume(&points[p0], &points[p1], &points[p2], &points[p3]) > 0.0 {
            std::mem::swap(&mut p1, &mut p2);
        }

        let rest: Vec<usize> = (0..n).filter(|&i| i != p0 && i != p1 && i != p2 && i != p3).collect();
        let mut faces: Vec<(usize, usize, usize)> = Vec::new();
        Self::quickhull_3d_faces(points, &rest, p0, p1, p2, &mut faces);
        Self::quickhull_3d_faces(points, &rest, p0, p3, p1, &mut faces);
        Self::quickhull_3d_faces(points, &rest, p1, p3, p2, &mut faces);
        Self::quickhull_3d_faces(points, &rest, p2, p3, p0, &mut faces);

        let mut used: HashSet<usize> = HashSet::new();
        for &(a, b, c) in &faces {
            used.insert(a);
            used.insert(b);
            used.insert(c);
        }
        let mut sorted_used: Vec<usize> = used.into_iter().collect();
        sorted_used.sort();
        let mut idx_to_vkey = vec![0usize; n];
        for &idx in &sorted_used {
            let vk = mesh.add_vertex(points[idx].clone(), None);
            idx_to_vkey[idx] = vk;
        }
        for &(a, b, c) in &faces {
            mesh.add_face(vec![idx_to_vkey[a], idx_to_vkey[b], idx_to_vkey[c]], None);
        }
        mesh
    }
}
