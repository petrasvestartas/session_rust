use crate::point::Point;

fn cross_2d(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> f64 {
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
}

fn signed_area_2d(coords: &[f64]) -> f64 {
    let n = coords.len() / 2;
    let mut area = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        area += coords[i*2] * coords[j*2+1] - coords[j*2] * coords[i*2+1];
    }
    area * 0.5
}

fn point_in_triangle_2d(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64) -> bool {
    let d1 = cross_2d(ax, ay, bx, by, px, py);
    let d2 = cross_2d(bx, by, cx, cy, px, py);
    let d3 = cross_2d(cx, cy, ax, ay, px, py);
    !((d1 < 0.0 || d2 < 0.0 || d3 < 0.0) && (d1 > 0.0 || d2 > 0.0 || d3 > 0.0))
}

fn segments_intersect(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64, dx: f64, dy: f64) -> bool {
    let d1 = cross_2d(cx, cy, dx, dy, ax, ay);
    let d2 = cross_2d(cx, cy, dx, dy, bx, by);
    let d3 = cross_2d(ax, ay, bx, by, cx, cy);
    let d4 = cross_2d(ax, ay, bx, by, dx, dy);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
    ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

fn is_visible(coords: &[f64], polygon: &[i32], from_v: i32, to_v: i32) -> bool {
    let ax = coords[from_v as usize * 2];
    let ay = coords[from_v as usize * 2 + 1];
    let bx = coords[to_v as usize * 2];
    let by = coords[to_v as usize * 2 + 1];
    for i in 0..polygon.len() {
        let j = (i + 1) % polygon.len();
        let vi = polygon[i];
        let vj = polygon[j];
        if vi == from_v || vi == to_v || vj == from_v || vj == to_v { continue; }
        let cx = coords[vi as usize * 2];
        let cy = coords[vi as usize * 2 + 1];
        let dx = coords[vj as usize * 2];
        let dy = coords[vj as usize * 2 + 1];
        if segments_intersect(ax, ay, bx, by, cx, cy, dx, dy) { return false; }
    }
    true
}

fn ear_clip(coords: &[f64], indices_in: &[i32]) -> Vec<(i32, i32, i32)> {
    let mut triangles = Vec::new();
    let n = indices_in.len();
    if n < 3 { return triangles; }
    let mut poly_coords = vec![0.0; n * 2];
    for i in 0..n {
        poly_coords[i*2] = coords[indices_in[i] as usize * 2];
        poly_coords[i*2+1] = coords[indices_in[i] as usize * 2 + 1];
    }
    let mut indices: Vec<i32> = if signed_area_2d(&poly_coords) < 0.0 {
        indices_in.iter().rev().copied().collect()
    } else {
        indices_in.to_vec()
    };
    let build_reflex = |idx: &[i32]| -> Vec<bool> {
        let mut r = vec![false; idx.len()];
        for i in 0..idx.len() {
            let p = (i + idx.len() - 1) % idx.len();
            let nx = (i + 1) % idx.len();
            if cross_2d(coords[idx[p] as usize*2], coords[idx[p] as usize*2+1],
                        coords[idx[i] as usize*2], coords[idx[i] as usize*2+1],
                        coords[idx[nx] as usize*2], coords[idx[nx] as usize*2+1]) <= 0.0 {
                r[i] = true;
            }
        }
        r
    };
    let mut reflex = build_reflex(&indices);
    let max_iter = n * n * 2;
    let mut it = 0;
    while indices.len() > 3 && it < max_iter {
        let mut found = false;
        for i in 0..indices.len() {
            if reflex[i] { continue; }
            let p = (i + indices.len() - 1) % indices.len();
            let nx = (i + 1) % indices.len();
            let ax = coords[indices[p] as usize*2]; let ay = coords[indices[p] as usize*2+1];
            let bx = coords[indices[i] as usize*2]; let by = coords[indices[i] as usize*2+1];
            let cx = coords[indices[nx] as usize*2]; let cy = coords[indices[nx] as usize*2+1];
            let mut is_ear = true;
            for k in 0..indices.len() {
                if k == p || k == i || k == nx { continue; }
                if !reflex[k] { continue; }
                let px = coords[indices[k] as usize*2];
                let py = coords[indices[k] as usize*2+1];
                if (px-ax).abs() < 1e-12 && (py-ay).abs() < 1e-12 { continue; }
                if (px-bx).abs() < 1e-12 && (py-by).abs() < 1e-12 { continue; }
                if (px-cx).abs() < 1e-12 && (py-cy).abs() < 1e-12 { continue; }
                if point_in_triangle_2d(px, py, ax, ay, bx, by, cx, cy) {
                    is_ear = false;
                    break;
                }
            }
            if is_ear {
                let (ip, ii, inx) = (indices[p], indices[i], indices[nx]);
                triangles.push((ip, ii, inx));
                indices.remove(i);
                reflex = build_reflex(&indices);
                found = true;
                break;
            }
        }
        if !found {
            let mut removed = false;
            for i in 0..indices.len() {
                let p = (i + indices.len() - 1) % indices.len();
                let nx = (i + 1) % indices.len();
                let c = cross_2d(coords[indices[p] as usize*2], coords[indices[p] as usize*2+1],
                                 coords[indices[i] as usize*2], coords[indices[i] as usize*2+1],
                                 coords[indices[nx] as usize*2], coords[indices[nx] as usize*2+1]);
                if c.abs() < 1e-12 {
                    let (ip, ii, inx) = (indices[p], indices[i], indices[nx]);
                    triangles.push((ip, ii, inx));
                    indices.remove(i);
                    reflex = build_reflex(&indices);
                    removed = true;
                    break;
                }
            }
            if !removed {
                let mut coord_map: std::collections::HashMap<(i64, i64), usize> = std::collections::HashMap::new();
                let mut did_split = false;
                for pos in 0..indices.len() {
                    let kx = (coords[indices[pos] as usize * 2] * 1e8).round() as i64;
                    let ky = (coords[indices[pos] as usize * 2 + 1] * 1e8).round() as i64;
                    if let Some(&sf) = coord_map.get(&(kx, ky)) {
                        let poly_a: Vec<i32> = indices[sf..pos].to_vec();
                        let mut poly_b: Vec<i32> = indices[pos..].to_vec();
                        poly_b.extend_from_slice(&indices[..sf]);
                        triangles.extend(ear_clip(coords, &poly_a));
                        triangles.extend(ear_clip(coords, &poly_b));
                        did_split = true;
                        break;
                    }
                    coord_map.insert((kx, ky), pos);
                }
                if did_split { indices.clear(); }
                break;
            }
        }
        it += 1;
    }
    if indices.len() == 3 {
        triangles.push((indices[0], indices[1], indices[2]));
    }
    triangles
}

fn merge_holes(coords: &[f64], boundary_indices: &[i32], hole_indices_list: &[Vec<i32>]) -> Vec<i32> {
    if hole_indices_list.is_empty() { return boundary_indices.to_vec(); }
    let mut sorted_holes: Vec<(usize, f64, usize)> = hole_indices_list.iter().enumerate().map(|(h, hole)| {
        let mut mx = f64::NEG_INFINITY;
        let mut mx_local = 0;
        for (i, &vi) in hole.iter().enumerate() {
            let x = coords[vi as usize * 2];
            if x > mx { mx = x; mx_local = i; }
        }
        (h, mx, mx_local)
    }).collect();
    sorted_holes.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    let mut merged = boundary_indices.to_vec();
    for &(h_idx, _, hole_vi) in &sorted_holes {
        let hole = &hole_indices_list[h_idx];
        let hx = coords[hole[hole_vi] as usize * 2];
        let hy = coords[hole[hole_vi] as usize * 2 + 1];
        let mut best_ix = f64::INFINITY;
        let mut best_edge: i32 = -1;
        for i in 0..merged.len() {
            let j = (i + 1) % merged.len();
            let y0 = coords[merged[i] as usize * 2 + 1];
            let y1 = coords[merged[j] as usize * 2 + 1];
            let x0 = coords[merged[i] as usize * 2];
            let x1 = coords[merged[j] as usize * 2];
            if (y0 <= hy && y1 > hy) || (y1 <= hy && y0 > hy) {
                let t = (hy - y0) / (y1 - y0);
                let ix = x0 + t * (x1 - x0);
                if ix >= hx && ix < best_ix {
                    best_ix = ix;
                    best_edge = i as i32;
                }
            }
        }
        if best_edge < 0 { continue; }
        let be0 = best_edge as usize;
        let be1 = (best_edge as usize + 1) % merged.len();
        let vx0 = coords[merged[be0] as usize * 2];
        let vx1 = coords[merged[be1] as usize * 2];
        let candidate = if vx0 >= vx1 { be0 } else { be1 };
        let vis_x = coords[merged[candidate] as usize * 2];
        let vis_y = coords[merged[candidate] as usize * 2 + 1];
        let mut min_dist = f64::INFINITY;
        let mut visible = candidate;
        for i in 0..merged.len() {
            let px = coords[merged[i] as usize * 2];
            let py = coords[merged[i] as usize * 2 + 1];
            if px < hx { continue; }
            if point_in_triangle_2d(px, py, hx, hy, best_ix, hy, vis_x, vis_y) ||
               ((px - vis_x).abs() < 1e-12 && (py - vis_y).abs() < 1e-12) {
                let dx = px - hx;
                let dy = py - hy;
                let dist = dx*dx + dy*dy;
                if dist < min_dist && is_visible(coords, &merged, hole[hole_vi], merged[i]) {
                    min_dist = dist;
                    visible = i;
                }
            }
        }
        if visible == candidate && !is_visible(coords, &merged, hole[hole_vi], merged[candidate]) {
            min_dist = f64::INFINITY;
            for i in 0..merged.len() {
                let px = coords[merged[i] as usize * 2];
                let py = coords[merged[i] as usize * 2 + 1];
                let dx = px - hx;
                let dy = py - hy;
                let dist = dx*dx + dy*dy;
                if dist < min_dist && is_visible(coords, &merged, hole[hole_vi], merged[i]) {
                    min_dist = dist;
                    visible = i;
                }
            }
        }
        let mut new_merged = merged[..=visible].to_vec();
        let hole_n = hole.len();
        for i in 0..=hole_n {
            new_merged.push(hole[(hole_vi + i) % hole_n]);
        }
        new_merged.push(merged[visible]);
        new_merged.extend_from_slice(&merged[visible+1..]);
        merged = new_merged;
    }
    merged
}

pub fn point_in_polygon_2d(px: f64, py: f64, coords: &[f64]) -> bool {
    let mut winding = 0i32;
    let n = coords.len() / 2;
    for i in 0..n {
        let j = (i + 1) % n;
        let y0 = coords[i*2+1];
        let y1 = coords[j*2+1];
        if y0 <= py {
            if y1 > py {
                if cross_2d(coords[i*2], y0, coords[j*2], y1, px, py) > 0.0 { winding += 1; }
            }
        } else if y1 <= py {
            if cross_2d(coords[i*2], y0, coords[j*2], y1, px, py) < 0.0 { winding -= 1; }
        }
    }
    winding != 0
}

pub fn triangulate(outer_pts: &[Point], hole_pts_list: Option<&[Vec<Point>]>) -> Vec<(i32, i32, i32)> {
    if outer_pts.len() < 3 { return vec![]; }
    let mut coords = Vec::new();
    let mut bn = outer_pts.len();
    if bn > 1 && (outer_pts[0][0] - outer_pts[bn-1][0]).abs() < 1e-12 &&
       (outer_pts[0][1] - outer_pts[bn-1][1]).abs() < 1e-12 { bn -= 1; }
    let no_holes = hole_pts_list.map_or(true, |h| h.is_empty());
    if no_holes {
        if bn == 3 { return vec![(0, 1, 2)]; }
        if bn == 4 {
            let d02 = (outer_pts[0][0]-outer_pts[2][0]).powi(2) + (outer_pts[0][1]-outer_pts[2][1]).powi(2);
            let d13 = (outer_pts[1][0]-outer_pts[3][0]).powi(2) + (outer_pts[1][1]-outer_pts[3][1]).powi(2);
            return if d02 <= d13 { vec![(0, 1, 2), (0, 2, 3)] } else { vec![(0, 1, 3), (1, 2, 3)] };
        }
        let mut convex = true;
        let mut first_cross = 0.0_f64;
        for i in 0..bn {
            let j = (i + 1) % bn;
            let k = (i + 2) % bn;
            let c = cross_2d(outer_pts[i][0], outer_pts[i][1], outer_pts[j][0], outer_pts[j][1], outer_pts[k][0], outer_pts[k][1]);
            if c.abs() < 1e-12 { continue; }
            if first_cross == 0.0 { first_cross = c; }
            else if (c > 0.0) != (first_cross > 0.0) { convex = false; break; }
        }
        if convex && bn >= 5 {
            return (1..bn as i32 - 1).map(|i| (0, i, i + 1)).collect();
        }
    }
    let mut boundary_indices = Vec::new();
    for i in 0..bn {
        let idx = (coords.len() / 2) as i32;
        coords.push(outer_pts[i][0]);
        coords.push(outer_pts[i][1]);
        boundary_indices.push(idx);
    }
    let bcoords: Vec<f64> = boundary_indices.iter().flat_map(|&i| {
        vec![coords[i as usize * 2], coords[i as usize * 2 + 1]]
    }).collect();
    if signed_area_2d(&bcoords) < 0.0 { boundary_indices.reverse(); }
    let mut hole_indices_list = Vec::new();
    if let Some(holes) = hole_pts_list {
        for hole in holes {
            let mut hn = hole.len();
            if hn < 3 { continue; }
            if hn > 1 && (hole[0][0] - hole[hn-1][0]).abs() < 1e-12 &&
               (hole[0][1] - hole[hn-1][1]).abs() < 1e-12 { hn -= 1; }
            let mut hole_indices = Vec::new();
            for i in 0..hn {
                let idx = (coords.len() / 2) as i32;
                coords.push(hole[i][0]);
                coords.push(hole[i][1]);
                hole_indices.push(idx);
            }
            let hcoords: Vec<f64> = hole_indices.iter().flat_map(|&i| {
                vec![coords[i as usize * 2], coords[i as usize * 2 + 1]]
            }).collect();
            if signed_area_2d(&hcoords) > 0.0 { hole_indices.reverse(); }
            hole_indices_list.push(hole_indices);
        }
    }
    let merged = merge_holes(&coords, &boundary_indices, &hole_indices_list);
    ear_clip(&coords, &merged)
}
