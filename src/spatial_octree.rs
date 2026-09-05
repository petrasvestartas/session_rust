// SpatialOctree — Potree-style multi-resolution LOD octree over bare 3D points.
// Use for: level-of-detail point-cloud rendering. Each node owns a spacing-limited
//   SUBSAMPLE of the points (grid accept, first point wins, leftovers descend into
//   octants at half the spacing), so drawing shallow nodes far away and deep nodes up
//   close gives Potree's uniform on-screen density. `order()` is the permutation that
//   makes every node's points CONTIGUOUS - upload points in that order and a node is
//   one (first, count) range.
// Prefer over SpatialKDTree when the question is "which points at what density",
//   not "which point is nearest".
// Note: static structure; rebuild required after point insertion.
use std::collections::HashSet;

use crate::point::Point;

// Duplicate points can never be separated by subdivision: below this level the node
// absorbs everything instead of recursing forever (spacing has shrunk by 2^21 anyway).
const MAX_LEVEL: usize = 21;

struct Node {
    min: [f64; 3],
    size: f64,
    level: usize,
    spacing: f64,
    first: usize,
    count: usize,
    children: [i32; 8],
}

pub struct SpatialOctree {
    nodes: Vec<Node>,
    order: Vec<usize>,
}

impl SpatialOctree {
    pub fn new(points: Vec<Point>, root_spacing: f64, leaf_capacity: usize) -> Self {
        let mut coords: Vec<f64> = Vec::with_capacity(points.len() * 3);
        for p in &points {
            coords.push(p[0]);
            coords.push(p[1]);
            coords.push(p[2]);
        }
        Self::from_coords(&coords, root_spacing, leaf_capacity)
    }

    /// Coords are only read during construction - nothing is stored, so a renderer can
    /// hand its flat table over without a copy.
    pub fn from_coords(coords: &[f64], root_spacing: f64, leaf_capacity: usize) -> Self {
        let mut tree = SpatialOctree {
            nodes: Vec::new(),
            order: Vec::new(),
        };
        let n = coords.len() / 3;
        if n == 0 {
            return tree;
        }
        let mut lo = [coords[0], coords[1], coords[2]];
        let mut hi = lo;
        for i in 1..n {
            for k in 0..3 {
                lo[k] = lo[k].min(coords[i * 3 + k]);
                hi[k] = hi[k].max(coords[i * 3 + k]);
            }
        }
        let mut size = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(hi[2] - lo[2]);
        if size <= 0.0 {
            size = 1.0;
        }
        let root_min = [0, 1, 2].map(|k| (lo[k] + hi[k]) * 0.5 - size * 0.5);
        let idxs: Vec<usize> = (0..n).collect();
        tree.build(coords, root_min, size, 0, root_spacing, idxs, leaf_capacity);
        tree
    }

    fn build(
        &mut self,
        coords: &[f64],
        min: [f64; 3],
        size: f64,
        level: usize,
        spacing: f64,
        idxs: Vec<usize>,
        leaf_capacity: usize,
    ) -> usize {
        let node_id = self.nodes.len();
        self.nodes.push(Node {
            min,
            size,
            level,
            spacing,
            first: self.order.len(),
            count: 0,
            children: [-1; 8],
        });
        if idxs.len() <= leaf_capacity || level >= MAX_LEVEL {
            self.nodes[node_id].count = idxs.len();
            self.order.extend(idxs);
            return node_id;
        }
        let cells = ((size / spacing).ceil() as i64).max(1);
        let center = [0, 1, 2].map(|k| min[k] + size * 0.5);
        let mut seen: HashSet<(i64, i64, i64)> = HashSet::new();
        let mut accepted: Vec<usize> = Vec::new();
        let mut buckets: [Vec<usize>; 8] = Default::default();
        for i in idxs {
            let key = [0, 1, 2].map(|k| {
                let c = ((coords[i * 3 + k] - min[k]) / spacing).floor() as i64;
                c.clamp(0, cells - 1)
            });
            if seen.insert((key[0], key[1], key[2])) {
                accepted.push(i);
            } else {
                let mut b = 0usize;
                if coords[i * 3] >= center[0] {
                    b |= 1;
                }
                if coords[i * 3 + 1] >= center[1] {
                    b |= 2;
                }
                if coords[i * 3 + 2] >= center[2] {
                    b |= 4;
                }
                buckets[b].push(i);
            }
        }
        self.nodes[node_id].count = accepted.len();
        self.order.extend(accepted);
        let half = size * 0.5;
        for (b, bucket) in std::mem::take(&mut buckets).into_iter().enumerate() {
            if !bucket.is_empty() {
                let child_min = [
                    min[0] + (b & 1) as f64 * half,
                    min[1] + ((b >> 1) & 1) as f64 * half,
                    min[2] + ((b >> 2) & 1) as f64 * half,
                ];
                let child_id = self.build(
                    coords,
                    child_min,
                    half,
                    level + 1,
                    spacing * 0.5,
                    bucket,
                    leaf_capacity,
                );
                self.nodes[node_id].children[b] = child_id as i32;
            }
        }
        node_id
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    pub fn node_cube(&self, i: usize) -> (Point, f64) {
        let node = &self.nodes[i];
        let half = node.size * 0.5;
        (
            Point::new(node.min[0] + half, node.min[1] + half, node.min[2] + half),
            node.size,
        )
    }

    pub fn node_level(&self, i: usize) -> usize {
        self.nodes[i].level
    }

    pub fn node_spacing(&self, i: usize) -> f64 {
        self.nodes[i].spacing
    }

    pub fn node_range(&self, i: usize) -> (usize, usize) {
        let node = &self.nodes[i];
        (node.first, node.count)
    }

    pub fn children(&self, i: usize) -> Vec<usize> {
        self.nodes[i]
            .children
            .iter()
            .filter(|&&c| c >= 0)
            .map(|&c| c as usize)
            .collect()
    }

    pub fn order(&self) -> &[usize] {
        &self.order
    }
}
