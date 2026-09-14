use crate::point::Point;

const STACK_SIZE: usize = 64;
const NULL_IDX: usize = usize::MAX;

struct Node {
    idx: usize,
    axis: usize,
    left: usize,
    right: usize,
}

#[derive(Clone, Copy)]
struct Range {
    lo: usize,
    hi: usize,
    depth: usize,
    parent: usize,
    is_left: bool,
}

impl Range {
    fn new(lo: usize, hi: usize, depth: usize, parent: usize, is_left: bool) -> Self {
        Range {
            lo,
            hi,
            depth,
            parent,
            is_left,
        }
    }
}

#[derive(Clone, Copy)]
struct Visit {
    node: usize,
    bound: f64,
}

const EMPTY_RANGE: Range = Range {
    lo: 0,
    hi: 0,
    depth: 0,
    parent: NULL_IDX,
    is_left: false,
};

const EMPTY_VISIT: Visit = Visit {
    node: NULL_IDX,
    bound: 0.0,
};

/// KD-tree with alternating-axis median split over points for nearest, k-nearest and radius queries.
pub struct SpatialKDTree {
    points: Vec<Point>,
    nodes: Vec<Node>,
}

impl SpatialKDTree {
    pub fn new(points: Vec<Point>) -> Self {
        let mut tree = SpatialKDTree {
            points,
            nodes: Vec::new(),
        };
        tree.build();
        tree
    }

    fn build(&mut self) {
        let n = self.points.len();
        let mut indices: Vec<usize> = (0..n).collect();
        self.nodes.reserve(n);
        let mut stack = [EMPTY_RANGE; STACK_SIZE];
        let mut top = 0;
        if n > 0 {
            stack[top] = Range::new(0, n, 0, NULL_IDX, false);
            top += 1;
        }
        while top > 0 {
            top -= 1;
            let range = stack[top];
            let axis = range.depth % 3;
            let mid = range.lo + (range.hi - range.lo) / 2;
            let points = &self.points;
            indices[range.lo..range.hi].select_nth_unstable_by(mid - range.lo, |&a, &b| {
                points[a][axis].total_cmp(&points[b][axis])
            });
            let node = self.nodes.len();
            self.nodes.push(Node {
                idx: indices[mid],
                axis,
                left: NULL_IDX,
                right: NULL_IDX,
            });
            if range.parent != NULL_IDX && range.is_left {
                self.nodes[range.parent].left = node;
            }
            if range.parent != NULL_IDX && !range.is_left {
                self.nodes[range.parent].right = node;
            }
            if range.lo < mid {
                assert!(top < STACK_SIZE);
                stack[top] = Range::new(range.lo, mid, range.depth + 1, node, true);
                top += 1;
            }
            if mid + 1 < range.hi {
                assert!(top < STACK_SIZE);
                stack[top] = Range::new(mid + 1, range.hi, range.depth + 1, node, false);
                top += 1;
            }
        }
    }

    fn push(&self, stack: &mut [Visit; STACK_SIZE], top: &mut usize, node: usize, bound: f64) {
        if node == NULL_IDX {
            return;
        }
        assert!(*top < STACK_SIZE);
        stack[*top] = Visit { node, bound };
        *top += 1;
    }

    fn dist_sq(&self, a: &Point, b: &Point) -> f64 {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let dz = a[2] - b[2];
        dx * dx + dy * dy + dz * dz
    }

    fn insert_sorted(&self, best: &mut Vec<(usize, f64)>, idx: usize, d2: f64, k: usize) {
        let mut pos = best.len();
        while pos > 0 && best[pos - 1].1 > d2 {
            pos -= 1;
        }
        best.insert(pos, (idx, d2));
        if best.len() > k {
            best.pop();
        }
    }

    pub fn nearest(&self, query: &Point) -> (usize, f64) {
        let mut best = 0;
        let mut best_d2 = f64::INFINITY;
        let mut stack = [EMPTY_VISIT; STACK_SIZE];
        let mut top = 0;
        if !self.nodes.is_empty() {
            self.push(&mut stack, &mut top, 0, 0.0);
        }
        while top > 0 {
            top -= 1;
            let visit = stack[top];
            if visit.bound >= best_d2 {
                continue;
            }
            let node = &self.nodes[visit.node];
            let d2 = self.dist_sq(query, &self.points[node.idx]);
            if d2 < best_d2 {
                best_d2 = d2;
                best = node.idx;
            }
            let diff = query[node.axis] - self.points[node.idx][node.axis];
            let near = if diff <= 0.0 { node.left } else { node.right };
            let far = if diff <= 0.0 { node.right } else { node.left };
            self.push(&mut stack, &mut top, far, diff * diff);
            self.push(&mut stack, &mut top, near, 0.0);
        }
        (best, best_d2.sqrt())
    }

    pub fn nearest_k(&self, query: &Point, k: usize) -> Vec<(usize, f64)> {
        let mut best: Vec<(usize, f64)> = Vec::new();
        if k == 0 {
            return best;
        }
        let mut stack = [EMPTY_VISIT; STACK_SIZE];
        let mut top = 0;
        if !self.nodes.is_empty() {
            self.push(&mut stack, &mut top, 0, 0.0);
        }
        while top > 0 {
            top -= 1;
            let visit = stack[top];
            let full = best.len() == k;
            if full && visit.bound >= best[best.len() - 1].1 {
                continue;
            }
            let node = &self.nodes[visit.node];
            let d2 = self.dist_sq(query, &self.points[node.idx]);
            if !full || d2 < best[best.len() - 1].1 {
                self.insert_sorted(&mut best, node.idx, d2, k);
            }
            let diff = query[node.axis] - self.points[node.idx][node.axis];
            let near = if diff <= 0.0 { node.left } else { node.right };
            let far = if diff <= 0.0 { node.right } else { node.left };
            self.push(&mut stack, &mut top, far, diff * diff);
            self.push(&mut stack, &mut top, near, 0.0);
        }
        for hit in best.iter_mut() {
            hit.1 = hit.1.sqrt();
        }
        best
    }

    pub fn radius_search(&self, query: &Point, radius: f64) -> Vec<(usize, f64)> {
        let mut result: Vec<(usize, f64)> = Vec::new();
        let r2 = radius * radius;
        let mut stack = [EMPTY_VISIT; STACK_SIZE];
        let mut top = 0;
        if !self.nodes.is_empty() {
            self.push(&mut stack, &mut top, 0, 0.0);
        }
        while top > 0 {
            top -= 1;
            let visit = stack[top];
            if visit.bound > r2 {
                continue;
            }
            let node = &self.nodes[visit.node];
            let d2 = self.dist_sq(query, &self.points[node.idx]);
            if d2 <= r2 {
                result.push((node.idx, d2.sqrt()));
            }
            let diff = query[node.axis] - self.points[node.idx][node.axis];
            let near = if diff <= 0.0 { node.left } else { node.right };
            let far = if diff <= 0.0 { node.right } else { node.left };
            self.push(&mut stack, &mut top, far, diff * diff);
            self.push(&mut stack, &mut top, near, 0.0);
        }
        result.sort_by(|a, b| a.1.total_cmp(&b.1));
        result
    }
}
