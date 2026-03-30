use crate::point::Point;
use crate::polyline::Polyline;
use std::collections::BTreeMap;

pub struct MarchingSquares;

// Edge table: for each of 16 cases, list of (edge_a, edge_b) pairs
// Edges: 0=bottom, 1=right, 2=top, 3=left (CCW)
const EDGE_TABLE: [&[(usize, usize)]; 16] = [
    &[],                        // 0000
    &[(3, 0)],                  // 0001
    &[(0, 1)],                  // 0010
    &[(3, 1)],                  // 0011
    &[(1, 2)],                  // 0100
    &[(3, 2), (0, 1)],          // 0101 ambiguous
    &[(0, 2)],                  // 0110
    &[(3, 2)],                  // 0111
    &[(2, 3)],                  // 1000
    &[(2, 0)],                  // 1001
    &[(2, 1), (3, 0)],          // 1010 ambiguous
    &[(2, 1)],                  // 1011
    &[(1, 3)],                  // 1100
    &[(1, 0)],                  // 1101
    &[(0, 3)],                  // 1110
    &[],                        // 1111
];

impl MarchingSquares {
    fn interp(a: f64, b: f64, va: f64, vb: f64, iso: f64) -> f64 {
        if (vb - va).abs() < 1e-12 { return (a + b) * 0.5; }
        let t = (iso - va) / (vb - va);
        a + t * (b - a)
    }

    fn edge_pt(e: usize, x0: f64, y0: f64, x1: f64, y1: f64, v0: f64, v1: f64, v2: f64, v3: f64, iso: f64) -> Point {
        match e {
            0 => Point::new(Self::interp(x0, x1, v0, v1, iso), y0, 0.0),
            1 => Point::new(x1, Self::interp(y0, y1, v1, v2, iso), 0.0),
            2 => Point::new(Self::interp(x0, x1, v3, v2, iso), y1, 0.0),
            _ => Point::new(x0, Self::interp(y0, y1, v0, v3, iso), 0.0),
        }
    }

    fn connect_segments(segs: Vec<(Point, Point)>) -> Vec<Polyline> {
        let snap = |v: f64| (v * 1e8).round() as i64;
        let key  = |p: &Point| (snap(p[0]), snap(p[1]));
        let mut ep: BTreeMap<(i64, i64), Vec<(usize, usize)>> = BTreeMap::new();
        for (i, (a, b)) in segs.iter().enumerate() {
            ep.entry(key(a)).or_default().push((i, 0));
            ep.entry(key(b)).or_default().push((i, 1));
        }
        let mut used = vec![false; segs.len()];
        let pop_next = |pt: &Point, used: &mut Vec<bool>| -> Option<Point> {
            let k = key(pt);
            if let Some(entries) = ep.get(&k) {
                for &(i, end) in entries {
                    if !used[i] {
                        used[i] = true;
                        return Some(if end == 0 { segs[i].1.clone() } else { segs[i].0.clone() });
                    }
                }
            }
            None
        };
        let mut result = Vec::new();
        for start in 0..segs.len() {
            if used[start] { continue; }
            used[start] = true;
            let (a, b) = (segs[start].0.clone(), segs[start].1.clone());
            let mut forward = vec![b];
            while let Some(nxt) = pop_next(forward.last().unwrap(), &mut used) { forward.push(nxt); }
            let mut backward = vec![a];
            while let Some(nxt) = pop_next(backward.last().unwrap(), &mut used) { backward.push(nxt); }
            let mut pts: Vec<Point> = backward.into_iter().rev().collect();
            pts.extend(forward);
            result.push(Polyline::new(pts));
        }
        result
    }

    pub fn extract(grid: &[Vec<f64>], iso_value: f64, cell_size: f64) -> Vec<Polyline> {
        let rows = grid.len();
        if rows == 0 { return vec![]; }
        let cols = grid[0].len();
        if cols == 0 { return vec![]; }
        let mut segs = Vec::new();
        for r in 0..rows - 1 {
            for c in 0..cols - 1 {
                let v0 = grid[r][c];
                let v1 = grid[r][c + 1];
                let v2 = grid[r + 1][c + 1];
                let v3 = grid[r + 1][c];
                let x0 = c as f64 * cell_size;
                let y0 = r as f64 * cell_size;
                let x1 = (c + 1) as f64 * cell_size;
                let y1 = (r + 1) as f64 * cell_size;
                let case =
                    ((v3 >= iso_value) as usize) << 3 |
                    ((v2 >= iso_value) as usize) << 2 |
                    ((v1 >= iso_value) as usize) << 1 |
                    ((v0 >= iso_value) as usize);
                for &(ea, eb) in EDGE_TABLE[case] {
                    segs.push((Self::edge_pt(ea, x0, y0, x1, y1, v0, v1, v2, v3, iso_value),
                                Self::edge_pt(eb, x0, y0, x1, y1, v0, v1, v2, v3, iso_value)));
                }
            }
        }
        Self::connect_segments(segs)
    }

    pub fn extract_from_func<F>(func: F, x_range: (f64, f64), y_range: (f64, f64), nx: usize, ny: usize, iso_value: f64) -> Vec<Polyline>
    where F: Fn(f64, f64) -> f64 {
        let (x0, x1) = x_range;
        let (y0, y1) = y_range;
        let dx = if nx > 1 { (x1 - x0) / (nx - 1) as f64 } else { 0.0 };
        let dy = if ny > 1 { (y1 - y0) / (ny - 1) as f64 } else { 0.0 };
        let mut grid: Vec<Vec<f64>> = Vec::new();
        for r in 0..ny {
            let mut row = Vec::new();
            for c in 0..nx { row.push(func(x0 + c as f64 * dx, y0 + r as f64 * dy)); }
            grid.push(row);
        }
        let mut segs = Vec::new();
        for r in 0..ny - 1 {
            for c in 0..nx - 1 {
                let v0 = grid[r][c];
                let v1 = grid[r][c + 1];
                let v2 = grid[r + 1][c + 1];
                let v3 = grid[r + 1][c];
                let px0 = x0 + c as f64 * dx;
                let py0 = y0 + r as f64 * dy;
                let px1 = x0 + (c + 1) as f64 * dx;
                let py1 = y0 + (r + 1) as f64 * dy;
                let case =
                    ((v3 >= iso_value) as usize) << 3 |
                    ((v2 >= iso_value) as usize) << 2 |
                    ((v1 >= iso_value) as usize) << 1 |
                    ((v0 >= iso_value) as usize);
                for &(ea, eb) in EDGE_TABLE[case] {
                    segs.push((Self::edge_pt(ea, px0, py0, px1, py1, v0, v1, v2, v3, iso_value),
                                Self::edge_pt(eb, px0, py0, px1, py1, v0, v1, v2, v3, iso_value)));
                }
            }
        }
        Self::connect_segments(segs)
    }
}
