use std::collections::HashSet;

use crate::point::Point;

const MAX_LEVEL: usize = 21; // Deepest subdivision level.
const STACK_SIZE: usize = 8 * MAX_LEVEL; // Explicit stack depth, 8 children per level.
const NULL_IDX: usize = usize::MAX; // Missing child marker.

/// A cube node with its subsample range and children.
struct Node {
    min: [f64; 3],        // Cube min corner.
    size: f64,            // Cube edge length.
    level: usize,         // Depth from the root.
    spacing: f64,         // Grid-accept spacing.
    first: usize,         // First point index into order.
    count: usize,         // Point count in order.
    children: [usize; 8], // Child node per octant or NULL_IDX.
}

/// A pending cube on the build stack with its index range.
#[derive(Clone, Copy)]
struct Task {
    min: [f64; 3], // Cube min corner.
    size: f64,     // Cube edge length.
    level: usize,  // Depth from the root.
    spacing: f64,  // Grid-accept spacing.
    lo: usize,     // Index range start.
    hi: usize,     // Index range end, exclusive.
    parent: usize, // Parent node or NULL_IDX.
    octant: usize, // Octant of the parent this task fills.
}

/// Filler for the build stack array.
const EMPTY_TASK: Task = Task {
    min: [0.0; 3],
    size: 0.0,
    level: 0,
    spacing: 0.0,
    lo: 0,
    hi: 0,
    parent: NULL_IDX,
    octant: 0,
};

/// Potree-style LOD octree: every node keeps a spacing-limited subsample and order() makes each node's points contiguous.
pub struct SpatialOctree {
    nodes: Vec<Node>,  // Nodes in build order, root first.
    order: Vec<usize>, // Point indices permuted so each node's points are contiguous.
}

impl SpatialOctree {
    /// Constructs the tree over points.
    pub fn new(points: Vec<Point>, root_spacing: f64, leaf_capacity: usize) -> Self {
        let mut coords: Vec<f64> = Vec::with_capacity(points.len() * 3);

        for p in &points {
            coords.push(p[0]);
            coords.push(p[1]);
            coords.push(p[2]);
        }

        let mut tree = SpatialOctree {
            nodes: Vec::new(),
            order: Vec::new(),
        };
        tree.build(&coords, root_spacing, leaf_capacity);

        tree
    }

    /// Constructs the tree over flat [x, y, z, ...] coordinates.
    pub fn from_coords(coords: &[f64], root_spacing: f64, leaf_capacity: usize) -> Self {
        let mut tree = SpatialOctree {
            nodes: Vec::new(),
            order: Vec::new(),
        };
        tree.build(coords, root_spacing, leaf_capacity);

        tree
    }

    /// Returns the min corner and edge length of the cube bounding coords.
    fn root_cube(&self, coords: &[f64]) -> ([f64; 3], f64) {
        let n = coords.len() / 3;
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

        let mut min = [0.0; 3];

        for k in 0..3 {
            min[k] = (lo[k] + hi[k]) * 0.5 - size * 0.5;
        }

        (min, size)
    }

    /// Builds the nodes by iterative subdivision over an explicit stack.
    fn build(&mut self, coords: &[f64], root_spacing: f64, leaf_capacity: usize) {
        let n = coords.len() / 3;

        if n == 0 {
            return;
        }

        let (root_min, root_size) = self.root_cube(coords);
        let mut indices: Vec<usize> = (0..n).collect();
        let mut stack = [EMPTY_TASK; STACK_SIZE];
        let mut top = 0;
        self.push(
            &mut stack,
            &mut top,
            Task {
                min: root_min,
                size: root_size,
                level: 0,
                spacing: root_spacing,
                lo: 0,
                hi: n,
                parent: NULL_IDX,
                octant: 0,
            },
        );

        while top > 0 {
            top -= 1;
            let task = stack[top];
            let node = self.nodes.len();
            self.nodes.push(Node {
                min: task.min,
                size: task.size,
                level: task.level,
                spacing: task.spacing,
                first: self.order.len(),
                count: 0,
                children: [NULL_IDX; 8],
            });

            if task.parent != NULL_IDX {
                self.nodes[task.parent].children[task.octant] = node;
            }

            if task.hi - task.lo <= leaf_capacity || task.level >= MAX_LEVEL {
                self.order.extend_from_slice(&indices[task.lo..task.hi]);
                self.nodes[node].count = task.hi - task.lo;
                continue;
            }

            let bounds = self.accept(coords, &task, &mut indices);
            self.nodes[node].count = self.order.len() - self.nodes[node].first;
            let half = task.size * 0.5;

            for b in (0..8).rev() {
                if bounds[b] == bounds[b + 1] {
                    continue;
                }

                let min = [
                    task.min[0] + (b & 1) as f64 * half,
                    task.min[1] + ((b >> 1) & 1) as f64 * half,
                    task.min[2] + ((b >> 2) & 1) as f64 * half,
                ];
                self.push(
                    &mut stack,
                    &mut top,
                    Task {
                        min,
                        size: half,
                        level: task.level + 1,
                        spacing: task.spacing * 0.5,
                        lo: bounds[b],
                        hi: bounds[b + 1],
                        parent: node,
                        octant: b,
                    },
                );
            }
        }
    }

    /// Keeps one point per spacing cell in the node, buckets the rest by octant and returns the 9 octant bounds.
    fn accept(&mut self, coords: &[f64], task: &Task, indices: &mut [usize]) -> [usize; 9] {
        let cells = ((task.size / task.spacing).ceil() as i64).max(1);
        let half = task.size * 0.5;
        let center = [task.min[0] + half, task.min[1] + half, task.min[2] + half];
        let mut seen: HashSet<[i64; 3]> = HashSet::new();
        let mut buckets: [Vec<usize>; 8] = Default::default();

        for &idx in &indices[task.lo..task.hi] {
            let mut key = [0i64; 3];

            for k in 0..3 {
                key[k] = (((coords[idx * 3 + k] - task.min[k]) / task.spacing).floor() as i64)
                    .clamp(0, cells - 1);
            }

            if seen.insert(key) {
                self.order.push(idx);
                continue;
            }

            let mut octant = 0;

            for k in 0..3 {
                if coords[idx * 3 + k] >= center[k] {
                    octant |= 1 << k;
                }
            }

            buckets[octant].push(idx);
        }

        let mut bounds = [0usize; 9];
        bounds[0] = task.lo;

        for b in 0..8 {
            bounds[b + 1] = bounds[b] + buckets[b].len();
            indices[bounds[b]..bounds[b + 1]].copy_from_slice(&buckets[b]);
        }

        bounds
    }

    /// Pushes a task onto the build stack.
    fn push(&self, stack: &mut [Task; STACK_SIZE], top: &mut usize, task: Task) {
        assert!(*top < STACK_SIZE);
        stack[*top] = task;
        *top += 1;
    }

    /// Returns the number of nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the node cube center and edge length.
    pub fn node_cube(&self, i: usize) -> (Point, f64) {
        let node = &self.nodes[i];
        let half = node.size * 0.5;
        (
            Point::new(node.min[0] + half, node.min[1] + half, node.min[2] + half),
            node.size,
        )
    }

    /// Returns the node depth from the root.
    pub fn node_level(&self, i: usize) -> usize {
        self.nodes[i].level
    }

    /// Returns the grid-accept spacing of a node.
    pub fn node_spacing(&self, i: usize) -> f64 {
        self.nodes[i].spacing
    }

    /// Returns the node point range as (first, count) into order.
    pub fn node_range(&self, i: usize) -> (usize, usize) {
        let node = &self.nodes[i];

        (node.first, node.count)
    }

    /// Returns the present child node indices.
    pub fn children(&self, i: usize) -> Vec<usize> {
        let mut result: Vec<usize> = Vec::new();

        for c in self.nodes[i].children {
            if c != NULL_IDX {
                result.push(c);
            }
        }

        result
    }

    /// Returns the point indices permuted so each node's points are contiguous.
    pub fn order(&self) -> &[usize] {
        &self.order
    }
}
