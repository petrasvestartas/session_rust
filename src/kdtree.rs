// KDTree — alternating-axis median split over bare 3D points.
// Use for: k-nearest-neighbor queries on point clouds (fastest option).
//   Points only — no volumes, no boxes, no rotation.
// Prefer over AABBTree/BVH when data is a point cloud, not triangle faces.
// Prefer over RTree   when queries are k-NN, not region overlap.
// Note: static structure; rebuild required after point insertion.
use crate::point::Point;

struct Node {
    idx: usize,
    axis: usize,
    left: Option<Box<Node>>,
    right: Option<Box<Node>>,
}

pub struct KDTree {
    points: Vec<Point>,
    root: Option<Box<Node>>,
}

impl KDTree {
    pub fn new(points: Vec<Point>) -> Self {
        let mut indices: Vec<usize> = (0..points.len()).collect();
        let root = if points.is_empty() { None } else { Some(Self::build(&points, &mut indices, 0)) };
        KDTree { points, root }
    }

    fn build(points: &[Point], indices: &mut [usize], depth: usize) -> Box<Node> {
        let axis = depth % 3;
        indices.sort_by(|&a, &b| points[a][axis].partial_cmp(&points[b][axis]).unwrap());
        let mid = indices.len() / 2;
        let left = if mid > 0 { Some(Self::build(points, &mut indices[..mid], depth + 1)) } else { None };
        let right = if mid + 1 < indices.len() { Some(Self::build(points, &mut indices[mid + 1..], depth + 1)) } else { None };
        Box::new(Node { idx: indices[mid], axis, left, right })
    }

    fn dist_sq(a: &Point, b: &Point) -> f64 {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let dz = a[2] - b[2];
        dx * dx + dy * dy + dz * dz
    }

    fn nearest_1(node: &Option<Box<Node>>, points: &[Point], query: &Point, best_idx: &mut usize, best_d2: &mut f64) {
        let node = match node { Some(n) => n, None => return };
        let d = Self::dist_sq(query, &points[node.idx]);
        if d < *best_d2 {
            *best_d2 = d;
            *best_idx = node.idx;
        }
        let diff = query[node.axis] - points[node.idx][node.axis];
        let (near, far) = if diff <= 0.0 { (&node.left, &node.right) } else { (&node.right, &node.left) };
        Self::nearest_1(near, points, query, best_idx, best_d2);
        if diff * diff < *best_d2 {
            Self::nearest_1(far, points, query, best_idx, best_d2);
        }
    }

    pub fn nearest(&self, query: &Point) -> (usize, f64) {
        let mut best_idx = 0;
        let mut best_d2 = f64::INFINITY;
        Self::nearest_1(&self.root, &self.points, query, &mut best_idx, &mut best_d2);
        (best_idx, best_d2.sqrt())
    }

    fn nearest_k_rec(node: &Option<Box<Node>>, points: &[Point], query: &Point, k: usize, heap: &mut Vec<(f64, usize)>) {
        let node = match node { Some(n) => n, None => return };
        let d = Self::dist_sq(query, &points[node.idx]);
        if heap.len() < k {
            heap.push((d, node.idx));
            heap.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        } else if d < heap[0].0 {
            heap[0] = (d, node.idx);
            heap.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        }
        let diff = query[node.axis] - points[node.idx][node.axis];
        let (near, far) = if diff <= 0.0 { (&node.left, &node.right) } else { (&node.right, &node.left) };
        Self::nearest_k_rec(near, points, query, k, heap);
        if heap.len() < k || diff * diff < heap[0].0 {
            Self::nearest_k_rec(far, points, query, k, heap);
        }
    }

    pub fn nearest_k(&self, query: &Point, k: usize) -> Vec<(usize, f64)> {
        let mut heap: Vec<(f64, usize)> = Vec::new();
        Self::nearest_k_rec(&self.root, &self.points, query, k, &mut heap);
        let mut result: Vec<(usize, f64)> = heap.iter().map(|&(d2, i)| (i, d2.sqrt())).collect();
        result.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        result
    }

    fn radius_rec(node: &Option<Box<Node>>, points: &[Point], query: &Point, radius_sq: f64, result: &mut Vec<(usize, f64)>) {
        let node = match node { Some(n) => n, None => return };
        let d = Self::dist_sq(query, &points[node.idx]);
        if d <= radius_sq {
            result.push((node.idx, d.sqrt()));
        }
        let diff = query[node.axis] - points[node.idx][node.axis];
        let (near, far) = if diff <= 0.0 { (&node.left, &node.right) } else { (&node.right, &node.left) };
        Self::radius_rec(near, points, query, radius_sq, result);
        if diff * diff <= radius_sq {
            Self::radius_rec(far, points, query, radius_sq, result);
        }
    }

    pub fn radius_search(&self, query: &Point, radius: f64) -> Vec<(usize, f64)> {
        let mut result = Vec::new();
        Self::radius_rec(&self.root, &self.points, query, radius * radius, &mut result);
        result.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        result
    }
}
