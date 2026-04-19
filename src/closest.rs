//! Closest point operations for geometry classes.

use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbssurface::NurbsSurface;
use crate::point::Point;
use crate::pointcloud::PointCloud;
use crate::polyline::Polyline;
use crate::vector::Vector;

pub struct Closest;

impl Closest {
    /// Find closest point on NURBS curve to test point.
    ///
    /// # Arguments
    /// * `curve` - The NURBS curve
    /// * `test_point` - Point to find closest curve point to
    /// * `t0` - Start of search interval (0.0 means curve start)
    /// * `t1` - End of search interval (0.0 means curve end)
    ///
    /// # Returns
    /// (parameter, distance) of closest point
    pub fn curve_point(curve: &NurbsCurve, test_point: &Point, t0: f64, t1: f64) -> (f64, f64) {
        if !curve.is_valid() {
            return (0.0, f64::INFINITY);
        }

        let (domain_start, domain_end) = curve.domain();
        let mut t0 = if t0 <= 0.0 { domain_start } else { t0 };
        let mut t1 = if t1 <= 0.0 { domain_end } else { t1 };

        t0 = t0.max(domain_start);
        t1 = t1.min(domain_end);

        let num_samples = (curve.degree() * 2).max(10);
        let dt = (t1 - t0) / num_samples as f64;

        let mut best_t = t0;
        let mut best_dist = curve.point_at(t0).distance(test_point, None);

        for i in 0..=num_samples {
            let t = t0 + i as f64 * dt;
            let dist = curve.point_at(t).distance(test_point, None);
            if dist < best_dist {
                best_dist = dist;
                best_t = t;
            }
        }

        let max_iterations = 20;
        let step_tolerance = (t1 - t0) * 1e-10;

        let mut t = best_t;

        for _ in 0..max_iterations {
            let pt = curve.point_at(t);
            let tangent = curve.tangent_at(t);

            let delta = Vector::new(
                test_point[0] - pt[0],
                test_point[1] - pt[1],
                test_point[2] - pt[2],
            );

            let f = -delta.dot(&tangent);

            if f.abs() < step_tolerance {
                break;
            }

            let derivs = curve.evaluate(t, 2);
            if derivs.len() < 3 {
                break;
            }

            let d2 = Vector::new(derivs[2][0], derivs[2][1], derivs[2][2]);
            let tangent_mag = tangent.magnitude();
            let df = delta.dot(&d2) - tangent_mag * tangent_mag;

            if df.abs() < 1e-12 {
                break;
            }

            let mut dt_step = f / df;

            if dt_step.abs() > (t1 - t0) * 0.5 {
                dt_step = dt_step.signum() * (t1 - t0) * 0.5;
            }

            t += dt_step;

            if t < t0 {
                t = t0;
            }
            if t > t1 {
                t = t1;
            }

            if dt_step.abs() < step_tolerance {
                break;
            }
        }

        let mut final_dist = curve.point_at(t).distance(test_point, None);

        let dist_start = curve.point_at(t0).distance(test_point, None);
        let dist_end = curve.point_at(t1).distance(test_point, None);

        if dist_start < final_dist {
            t = t0;
            final_dist = dist_start;
        }
        if dist_end < final_dist {
            t = t1;
            final_dist = dist_end;
        }

        (t, final_dist)
    }

    /// Find closest point on line to test point.
    ///
    /// # Arguments
    /// * `line` - The line segment
    /// * `test_point` - Point to find closest line point to
    ///
    /// # Returns
    /// (closest_point, parameter, distance)
    pub fn line_point(line: &Line, test_point: &Point) -> (Point, f64, f64) {
        let start = line.start();
        let end = line.end();

        let dx = end[0] - start[0];
        let dy = end[1] - start[1];
        let dz = end[2] - start[2];

        let len_sq = dx * dx + dy * dy + dz * dz;

        if len_sq < 1e-20 {
            let dist = start.distance(test_point, None);
            return (start, 0.0, dist);
        }

        let mut t = ((test_point[0] - start[0]) * dx
            + (test_point[1] - start[1]) * dy
            + (test_point[2] - start[2]) * dz)
            / len_sq;

        t = t.max(0.0).min(1.0);

        let closest = Point::new(start[0] + t * dx, start[1] + t * dy, start[2] + t * dz);

        let dist = closest.distance(test_point, None);

        (closest, t, dist)
    }

    /// Find closest point on polyline to test point.
    ///
    /// # Arguments
    /// * `polyline` - The polyline
    /// * `test_point` - Point to find closest polyline point to
    ///
    /// # Returns
    /// (closest_point, parameter, distance)
    pub fn polyline_point(polyline: &Polyline, test_point: &Point) -> (Point, f64, f64) {
        let points = polyline.get_points();

        if points.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), 0.0, f64::INFINITY);
        }

        if points.len() == 1 {
            let dist = points[0].distance(test_point, None);
            return (points[0].clone(), 0.0, dist);
        }

        let mut best_point = points[0].clone();
        let mut best_param = 0.0;
        let mut best_dist = f64::INFINITY;

        let mut cumulative_length = 0.0;
        let total_length = polyline.length();

        for i in 0..points.len() - 1 {
            let segment = Line::from_points(&points[i], &points[i + 1]);
            let (closest, t, dist) = Self::line_point(&segment, test_point);

            if dist < best_dist {
                best_dist = dist;
                best_point = closest;
                let segment_length = segment.length();
                if total_length > 1e-20 {
                    best_param = (cumulative_length + t * segment_length) / total_length;
                } else {
                    best_param = i as f64 / (points.len() - 1) as f64;
                }
            }

            cumulative_length += segment.length();
        }

        (best_point, best_param, best_dist)
    }

    /// Find closest point on NURBS surface to test point.
    ///
    /// # Arguments
    /// * `surface` - The NURBS surface
    /// * `test_point` - Point to find closest surface point to
    /// * `u0`, `u1` - U parameter search interval (0.0 means use surface domain)
    /// * `v0`, `v1` - V parameter search interval (0.0 means use surface domain)
    ///
    /// # Returns
    /// (u_param, v_param, distance)
    pub fn surface_point(
        surface: &NurbsSurface,
        test_point: &Point,
        u0: f64,
        u1: f64,
        v0: f64,
        v1: f64,
    ) -> (f64, f64, f64) {
        if !surface.is_valid() {
            return (0.0, 0.0, f64::INFINITY);
        }

        let (domain_u0, domain_u1) = surface.domain(0).unwrap_or((0.0, 1.0));
        let (domain_v0, domain_v1) = surface.domain(1).unwrap_or((0.0, 1.0));

        let mut u0 = if u0 <= 0.0 { domain_u0 } else { u0 };
        let mut u1 = if u1 <= 0.0 { domain_u1 } else { u1 };
        let mut v0 = if v0 <= 0.0 { domain_v0 } else { v0 };
        let mut v1 = if v1 <= 0.0 { domain_v1 } else { v1 };

        u0 = u0.max(domain_u0);
        u1 = u1.min(domain_u1);
        v0 = v0.max(domain_v0);
        v1 = v1.min(domain_v1);

        let u_samples = surface.order(0).max(10) as usize;
        let v_samples = surface.order(1).max(10) as usize;

        let du_param = (u1 - u0) / u_samples as f64;
        let dv_param = (v1 - v0) / v_samples as f64;

        let mut best_u = u0;
        let mut best_v = v0;
        let mut best_dist = f64::INFINITY;

        for i in 0..=u_samples {
            for j in 0..=v_samples {
                let uu = u0 + i as f64 * du_param;
                let vv = v0 + j as f64 * dv_param;
                if let Some(pt) = surface.point_at(uu, vv) {
                    let dist = pt.distance(test_point, None);
                    if dist < best_dist {
                        best_dist = dist;
                        best_u = uu;
                        best_v = vv;
                    }
                }
            }
        }

        let max_iterations = 20;
        let step_tolerance = (u1 - u0).min(v1 - v0) * 1e-10;

        let mut u = best_u;
        let mut v = best_v;

        for _ in 0..max_iterations {
            let derivs = surface.evaluate(u, v, 1);
            if derivs.len() < 3 { break; }

            if let Some(pt) = surface.point_at(u, v) {
                let du_vec = &derivs[2];  // evaluate returns [S, Sv, Su, ...]
                let dv_vec = &derivs[1];

                let delta = Vector::new(
                    test_point[0] - pt[0],
                    test_point[1] - pt[1],
                    test_point[2] - pt[2],
                );

                let fu = -delta.dot(du_vec);
                let fv = -delta.dot(dv_vec);

                if fu.abs() < step_tolerance && fv.abs() < step_tolerance { break; }

                let duu = du_vec.dot(du_vec);
                let dvv = dv_vec.dot(dv_vec);
                let duv = du_vec.dot(dv_vec);

                let det = duu * dvv - duv * duv;
                if det.abs() < 1e-12 { break; }

                let mut du_step = (dvv * fu - duv * fv) / det;
                let mut dv_step = (duu * fv - duv * fu) / det;

                let max_step = (u1 - u0).min(v1 - v0) * 0.5;
                if du_step.abs() > max_step { du_step = du_step.signum() * max_step; }
                if dv_step.abs() > max_step { dv_step = dv_step.signum() * max_step; }

                u -= du_step;
                v -= dv_step;

                u = u.clamp(u0, u1);
                v = v.clamp(v0, v1);

                if du_step.abs() < step_tolerance && dv_step.abs() < step_tolerance { break; }
            } else {
                break;
            }
        }

        let final_dist = surface.point_at(u, v)
            .map(|pt| pt.distance(test_point, None))
            .unwrap_or(f64::INFINITY);

        (u, v, final_dist)
    }

    fn closest_point_on_triangle(p: &Point, a: &Point, b: &Point, c: &Point) -> Point {
        let (abx, aby, abz) = (b[0]-a[0], b[1]-a[1], b[2]-a[2]);
        let (acx, acy, acz) = (c[0]-a[0], c[1]-a[1], c[2]-a[2]);
        let (apx, apy, apz) = (p[0]-a[0], p[1]-a[1], p[2]-a[2]);

        let d1 = abx*apx + aby*apy + abz*apz;
        let d2 = acx*apx + acy*apy + acz*apz;
        if d1 <= 0.0 && d2 <= 0.0 { return a.clone(); }

        let (bpx, bpy, bpz) = (p[0]-b[0], p[1]-b[1], p[2]-b[2]);
        let d3 = abx*bpx + aby*bpy + abz*bpz;
        let d4 = acx*bpx + acy*bpy + acz*bpz;
        if d3 >= 0.0 && d4 <= d3 { return b.clone(); }

        let vc = d1*d4 - d3*d2;
        if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
            let v = d1 / (d1 - d3);
            return Point::new(a[0] + v*abx, a[1] + v*aby, a[2] + v*abz);
        }

        let (cpx, cpy, cpz) = (p[0]-c[0], p[1]-c[1], p[2]-c[2]);
        let d5 = abx*cpx + aby*cpy + abz*cpz;
        let d6 = acx*cpx + acy*cpy + acz*cpz;
        if d6 >= 0.0 && d5 <= d6 { return c.clone(); }

        let vb = d5*d2 - d1*d6;
        if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
            let w = d2 / (d2 - d6);
            return Point::new(a[0] + w*acx, a[1] + w*acy, a[2] + w*acz);
        }

        let va = d3*d6 - d5*d4;
        if va <= 0.0 && (d4-d3) >= 0.0 && (d5-d6) >= 0.0 {
            let w = (d4-d3) / ((d4-d3) + (d5-d6));
            return Point::new(b[0] + w*(c[0]-b[0]), b[1] + w*(c[1]-b[1]), b[2] + w*(c[2]-b[2]));
        }

        let denom = 1.0 / (va + vb + vc);
        let v = vb * denom;
        let w = vc * denom;
        Point::new(a[0] + abx*v + acx*w, a[1] + aby*v + acy*w, a[2] + abz*v + acz*w)
    }

    pub fn mesh_point(mesh: &Mesh, test_point: &Point) -> (Point, usize, f64) {
        if mesh.number_of_faces() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        let (vertices, faces) = mesh.to_vertices_and_faces();
        let mut sorted_face_keys: Vec<usize> = mesh.face.keys().copied().collect();
        sorted_face_keys.sort();

        let mut best_point = Point::new(0.0, 0.0, 0.0);
        let mut best_face_key: usize = 0;
        let mut best_dist = f64::INFINITY;

        for (fi, fv) in faces.iter().enumerate() {
            if fv.len() < 3 { continue; }
            let v0 = &vertices[fv[0]];
            for j in 1..fv.len() - 1 {
                let v1 = &vertices[fv[j]];
                let v2 = &vertices[fv[j + 1]];
                let cp = Self::closest_point_on_triangle(test_point, v0, v1, v2);
                let dist = cp.distance(test_point, None);
                if dist < best_dist {
                    best_dist = dist;
                    best_point = cp;
                    best_face_key = sorted_face_keys[fi];
                }
            }
        }

        (best_point, best_face_key, best_dist)
    }

    pub fn mesh_point_aabb(mesh: &Mesh, test_point: &Point) -> (Point, usize, f64) {
        if mesh.number_of_faces() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        let (vertices, faces) = mesh.to_vertices_and_faces();
        let mut sorted_face_keys: Vec<usize> = mesh.face.keys().copied().collect();
        sorted_face_keys.sort();

        let mut tris: Vec<(Point, Point, Point)> = Vec::new();
        let mut tri_face_idx: Vec<usize> = Vec::new();
        for (fi, fv) in faces.iter().enumerate() {
            if fv.len() < 3 { continue; }
            let v0 = &vertices[fv[0]];
            for j in 1..fv.len() - 1 {
                tris.push((v0.clone(), vertices[fv[j]].clone(), vertices[fv[j + 1]].clone()));
                tri_face_idx.push(fi);
            }
        }

        if tris.is_empty() {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        // Build AABB boxes for each triangle: (cx, cy, cz, hx, hy, hz)
        let boxes: Vec<[f64; 6]> = tris.iter().map(|(v0, v1, v2)| {
            let lx = v0[0].min(v1[0]).min(v2[0]);
            let ly = v0[1].min(v1[1]).min(v2[1]);
            let lz = v0[2].min(v1[2]).min(v2[2]);
            let hx = v0[0].max(v1[0]).max(v2[0]);
            let hy = v0[1].max(v1[1]).max(v2[1]);
            let hz = v0[2].max(v1[2]).max(v2[2]);
            [(lx+hx)*0.5, (ly+hy)*0.5, (lz+hz)*0.5,
             (hx-lx)*0.5, (hy-ly)*0.5, (hz-lz)*0.5]
        }).collect();

        // AABBTree nodes: (aabb, right_child, object_id)
        let mut nodes: Vec<([f64; 6], i32, i32)> = Vec::new();

        fn build_node(ids: &mut [usize], boxes: &[[f64; 6]], nodes: &mut Vec<([f64; 6], i32, i32)>) {
            let ni = nodes.len();
            nodes.push(([0.0; 6], -1, -1));
            let mut lo = [f64::MAX; 3];
            let mut hi = [f64::MIN; 3];
            for &i in ids.iter() {
                let b = &boxes[i];
                for a in 0..3 {
                    lo[a] = lo[a].min(b[a] - b[a + 3]);
                    hi[a] = hi[a].max(b[a] + b[a + 3]);
                }
            }
            nodes[ni].0 = [
                (lo[0]+hi[0])*0.5, (lo[1]+hi[1])*0.5, (lo[2]+hi[2])*0.5,
                (hi[0]-lo[0])*0.5, (hi[1]-lo[1])*0.5, (hi[2]-lo[2])*0.5,
            ];
            if ids.len() == 1 {
                nodes[ni].2 = ids[0] as i32;
                return;
            }
            let dx = hi[0]-lo[0]; let dy = hi[1]-lo[1]; let dz = hi[2]-lo[2];
            let axis = if dx >= dy && dx >= dz { 0 } else if dy >= dz { 1 } else { 2 };
            let mid = ids.len() / 2;
            ids.select_nth_unstable_by(mid, |&a, &b| {
                boxes[a][axis].partial_cmp(&boxes[b][axis]).unwrap()
            });
            let (left_ids, right_ids) = ids.split_at_mut(mid);
            build_node(left_ids, boxes, nodes);
            nodes[ni].1 = nodes.len() as i32;
            build_node(right_ids, boxes, nodes);
        }

        let mut ids: Vec<usize> = (0..tris.len()).collect();
        build_node(&mut ids, &boxes, &mut nodes);

        fn aabb_min_dist(aabb: &[f64; 6], pt: &Point) -> f64 {
            let dx = (pt[0] - aabb[0]).abs() - aabb[3];
            let dy = (pt[1] - aabb[1]).abs() - aabb[4];
            let dz = (pt[2] - aabb[2]).abs() - aabb[5];
            let dx = dx.max(0.0); let dy = dy.max(0.0); let dz = dz.max(0.0);
            (dx*dx + dy*dy + dz*dz).sqrt()
        }

        let mut best_point = Point::new(0.0, 0.0, 0.0);
        let mut best_face_key: usize = 0;
        let mut best_dist = f64::INFINITY;

        fn dfs(
            ni: usize, nodes: &[([f64; 6], i32, i32)],
            tris: &[(Point, Point, Point)], tri_face_idx: &[usize],
            sorted_face_keys: &[usize], test_point: &Point,
            best_point: &mut Point, best_face_key: &mut usize, best_dist: &mut f64,
        ) {
            let (ref aabb, right, obj) = nodes[ni];
            if aabb_min_dist(aabb, test_point) >= *best_dist { return; }
            if obj >= 0 {
                let (ref v0, ref v1, ref v2) = tris[obj as usize];
                let cp = Closest::closest_point_on_triangle(test_point, v0, v1, v2);
                let d = cp.distance(test_point, None);
                if d < *best_dist {
                    *best_dist = d;
                    *best_point = cp;
                    *best_face_key = sorted_face_keys[tri_face_idx[obj as usize]];
                }
                return;
            }
            let left = ni + 1;
            let right = right as usize;
            let ld = aabb_min_dist(&nodes[left].0, test_point);
            let rd = aabb_min_dist(&nodes[right].0, test_point);
            if ld <= rd {
                if ld < *best_dist { dfs(left, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
                if rd < *best_dist { dfs(right, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
            } else {
                if rd < *best_dist { dfs(right, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
                if ld < *best_dist { dfs(left, nodes, tris, tri_face_idx, sorted_face_keys, test_point, best_point, best_face_key, best_dist); }
            }
        }

        dfs(0, &nodes, &tris, &tri_face_idx, &sorted_face_keys, test_point,
            &mut best_point, &mut best_face_key, &mut best_dist);

        (best_point, best_face_key, best_dist)
    }

    pub fn pointcloud_point(cloud: &PointCloud, test_point: &Point) -> (Point, usize, f64) {
        if cloud.point_count() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }

        let mut best_point = cloud.get_point(0);
        let mut best_index: usize = 0;
        let mut best_dist = best_point.distance(test_point, None);

        for i in 1..cloud.point_count() {
            let p = cloud.get_point(i);
            let dist = p.distance(test_point, None);
            if dist < best_dist {
                best_dist = dist;
                best_point = p;
                best_index = i;
            }
        }

        (best_point, best_index, best_dist)
    }

    pub fn pointcloud_point_kdtree(cloud: &PointCloud, test_point: &Point) -> (Point, usize, f64) {
        if cloud.point_count() == 0 {
            return (Point::new(0.0, 0.0, 0.0), 0, f64::INFINITY);
        }
        use crate::kdtree::KDTree;
        let pts: Vec<Point> = (0..cloud.point_count()).map(|i| cloud.get_point(i)).collect();
        let kd = KDTree::new(pts);
        let (idx, dist) = kd.nearest(test_point);
        (cloud.get_point(idx), idx, dist)
    }

    fn build_raw_boxes(aabbs: &[[f64; 6]]) -> Vec<([f64; 6], i32, i32)> {
        let mut nodes: Vec<([f64; 6], i32, i32)> = Vec::new();
        fn build(ids: &mut [usize], boxes: &[[f64; 6]], nodes: &mut Vec<([f64; 6], i32, i32)>) {
            let ni = nodes.len();
            nodes.push(([0.0; 6], -1, -1));
            let mut lo = [f64::MAX; 3];
            let mut hi = [f64::MIN; 3];
            for &i in ids.iter() {
                let b = &boxes[i];
                for a in 0..3 { lo[a] = lo[a].min(b[a] - b[a + 3]); hi[a] = hi[a].max(b[a] + b[a + 3]); }
            }
            nodes[ni].0 = [(lo[0]+hi[0])*0.5, (lo[1]+hi[1])*0.5, (lo[2]+hi[2])*0.5,
                           (hi[0]-lo[0])*0.5, (hi[1]-lo[1])*0.5, (hi[2]-lo[2])*0.5];
            if ids.len() == 1 { nodes[ni].2 = ids[0] as i32; return; }
            let dx = hi[0]-lo[0]; let dy = hi[1]-lo[1]; let dz = hi[2]-lo[2];
            let axis = if dx >= dy && dx >= dz { 0 } else if dy >= dz { 1 } else { 2 };
            let mid = ids.len() / 2;
            ids.select_nth_unstable_by(mid, |&a, &b| boxes[a][axis].partial_cmp(&boxes[b][axis]).unwrap());
            let (left_ids, right_ids) = ids.split_at_mut(mid);
            build(left_ids, boxes, nodes);
            nodes[ni].1 = nodes.len() as i32;
            build(right_ids, boxes, nodes);
        }
        let mut ids: Vec<usize> = (0..aabbs.len()).collect();
        build(&mut ids, aabbs, &mut nodes);
        nodes
    }

    fn query_raw_nodes(ni: usize, nodes: &[([f64; 6], i32, i32)], query: &[f64; 6], result: &mut Vec<usize>) {
        let (ref aabb, right, obj) = nodes[ni];
        let overlaps = (0..3).all(|a| (aabb[a] - query[a]).abs() <= aabb[a+3] + query[a+3]);
        if !overlaps { return; }
        if obj >= 0 { result.push(obj as usize); return; }
        Self::query_raw_nodes(ni + 1, nodes, query, result);
        Self::query_raw_nodes(right as usize, nodes, query, result);
    }

    fn aabb_to_aabb_min_dist(a: &[f64; 6], b: &[f64; 6]) -> f64 {
        let dx = ((a[0] - b[0]).abs() - a[3] - b[3]).max(0.0);
        let dy = ((a[1] - b[1]).abs() - a[4] - b[4]).max(0.0);
        let dz = ((a[2] - b[2]).abs() - a[5] - b[5]).max(0.0);
        (dx*dx + dy*dy + dz*dz).sqrt()
    }

    pub fn lines_closest(lines: &[Line], threshold: f64) -> Vec<(usize, usize)> {
        use crate::aabb::AABB;
        if lines.len() < 2 { return Vec::new(); }
        let raw: Vec<[f64; 6]> = lines.iter().map(|ln| {
            let b = AABB::from_line(ln, threshold);
            [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]
        }).collect();
        let nodes = Self::build_raw_boxes(&raw);
        let mut pairs = Vec::new();
        for i in 0..lines.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let (_, _, d_a) = Self::line_point(&lines[j], &lines[i].start());
                let (_, _, d_b) = Self::line_point(&lines[j], &lines[i].end());
                let (_, _, d_c) = Self::line_point(&lines[i], &lines[j].start());
                let (_, _, d_d) = Self::line_point(&lines[i], &lines[j].end());
                if d_a.min(d_b).min(d_c).min(d_d) <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }

    pub fn polylines_closest(polylines: &[Polyline], threshold: f64) -> Vec<(usize, usize)> {
        use crate::aabb::AABB;
        if polylines.len() < 2 { return Vec::new(); }
        let raw: Vec<[f64; 6]> = polylines.iter().map(|pl| {
            let b = AABB::from_polyline(pl, threshold);
            [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]
        }).collect();
        let nodes = Self::build_raw_boxes(&raw);
        let mut pairs = Vec::new();
        for i in 0..polylines.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let pts_a = polylines[i].get_points();
                let dist = pts_a.iter().map(|pt| Self::polyline_point(&polylines[j], pt).2)
                    .fold(f64::INFINITY, f64::min);
                if dist <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }

    pub fn nurbscurves_closest(curves: &[NurbsCurve], threshold: f64) -> Vec<(usize, usize)> {
        use crate::aabb::AABB;
        if curves.len() < 2 { return Vec::new(); }
        let raw: Vec<[f64; 6]> = curves.iter().map(|crv| {
            let b = AABB::from_nurbscurve(crv, threshold, false);
            [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]
        }).collect();
        let nodes = Self::build_raw_boxes(&raw);
        let mut pairs = Vec::new();
        for i in 0..curves.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let (t0, t1) = curves[i].domain();
                let p_start = curves[i].point_at(t0);
                let p_end = curves[i].point_at(t1);
                let (_, d_a) = Self::curve_point(&curves[j], &p_start, 0.0, 0.0);
                let (_, d_b) = Self::curve_point(&curves[j], &p_end, 0.0, 0.0);
                if d_a.min(d_b) <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }

    pub fn boxes_closest(boxes: &[crate::aabb::AABB], threshold: f64) -> Vec<(usize, usize)> {
        if boxes.len() < 2 { return Vec::new(); }
        let raw_orig: Vec<[f64; 6]> = boxes.iter().map(|b| [b.cx, b.cy, b.cz, b.hx, b.hy, b.hz]).collect();
        let raw_inf: Vec<[f64; 6]> = boxes.iter().map(|b| [b.cx, b.cy, b.cz, b.hx + threshold, b.hy + threshold, b.hz + threshold]).collect();
        let nodes = Self::build_raw_boxes(&raw_inf);
        let mut pairs = Vec::new();
        for i in 0..boxes.len() {
            let mut candidates = Vec::new();
            Self::query_raw_nodes(0, &nodes, &raw_inf[i], &mut candidates);
            for j in candidates {
                if j <= i { continue; }
                let dist = Self::aabb_to_aabb_min_dist(&raw_orig[i], &raw_orig[j]);
                if dist <= threshold { pairs.push((i, j)); }
            }
        }
        pairs
    }
}
