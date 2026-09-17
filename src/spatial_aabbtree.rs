use crate::aabb::AABB;

const STACK_SIZE: usize = 64; // Depth bound of the explicit traversal stack.
const NULL_IDX: i32 = -1; // Index of a missing child or object.

/// Tree node.
#[derive(Clone, Debug, Default)]
pub struct Node {
    pub aabb: AABB,     // Bounds of the subtree.
    pub right: i32,     // Right child index, NULL_IDX on a leaf.
    pub object_id: i32, // Primitive id on a leaf, NULL_IDX on an internal node.
}

/// Pending id range of the build stack.
#[derive(Clone, Copy)]
struct Range {
    lo: usize,     // First id of the range.
    hi: usize,     // One past the last id of the range.
    parent: i32,   // Parent node index, NULL_IDX at the root.
    is_left: bool, // Whether the range is the left child of parent.
}

impl Range {
    /// Constructs from id range, parent index and side.
    fn new(lo: usize, hi: usize, parent: i32, is_left: bool) -> Self {
        Range {
            lo,
            hi,
            parent,
            is_left,
        }
    }
}

/// Unused stack slot.
const EMPTY_RANGE: Range = Range {
    lo: 0,
    hi: 0,
    parent: NULL_IDX,
    is_left: false,
};

/// Flat AABB tree with longest-axis median split; the left child of node i is i + 1, the right child is stored.
#[derive(Clone, Debug, Default)]
pub struct SpatialAABBTree {
    pub nodes: Vec<Node>, // Nodes in depth-first order.
}

impl SpatialAABBTree {
    /// Constructs an empty tree.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns whether the tree has no nodes.
    pub fn empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Returns the node count.
    pub fn size(&self) -> usize {
        self.nodes.len()
    }

    /// Builds the tree over the boxes, one leaf per box.
    pub fn build(&mut self, aabbs: &[AABB]) {
        self.nodes.clear();
        let n = aabbs.len();
        let mut ids: Vec<usize> = (0..n).collect();
        self.nodes.reserve(2 * n);
        let mut stack = [EMPTY_RANGE; STACK_SIZE];
        let mut top = 0;

        if n > 0 {
            stack[top] = Range::new(0, n, NULL_IDX, false);
            top += 1;
        }

        while top > 0 {
            top -= 1;
            let range = stack[top];
            let node = self.nodes.len();
            let aabb = self.bounds(&ids, range.lo, range.hi, aabbs);
            self.nodes.push(Node {
                aabb,
                right: NULL_IDX,
                object_id: NULL_IDX,
            });

            if range.parent != NULL_IDX && !range.is_left {
                self.nodes[range.parent as usize].right = node as i32;
            }

            if range.hi - range.lo == 1 {
                self.nodes[node].object_id = ids[range.lo] as i32;
                continue;
            }

            let axis = self.longest_axis(&self.nodes[node].aabb);
            let mid = range.lo + (range.hi - range.lo) / 2;
            ids[range.lo..range.hi].select_nth_unstable_by(mid - range.lo, |&a, &b| {
                self.center(&aabbs[a], axis)
                    .total_cmp(&self.center(&aabbs[b], axis))
            });
            assert!(top + 2 <= STACK_SIZE);
            stack[top] = Range::new(mid, range.hi, node as i32, false);
            top += 1;
            stack[top] = Range::new(range.lo, mid, node as i32, true);
            top += 1;
        }
    }

    /// Returns the ids of every leaf box that intersects query.
    pub fn query_aabb(&self, query: &AABB) -> Vec<i32> {
        let mut hits: Vec<i32> = Vec::new();
        let mut stack = [0usize; STACK_SIZE];
        let mut top = 0;

        if !self.nodes.is_empty() {
            stack[top] = 0;
            top += 1;
        }

        while top > 0 {
            top -= 1;
            let idx = stack[top];
            let node = &self.nodes[idx];

            if !node.aabb.intersects(query) {
                continue;
            }

            if node.object_id != NULL_IDX {
                hits.push(node.object_id);
                continue;
            }

            assert!(top + 2 <= STACK_SIZE);
            stack[top] = idx + 1;
            top += 1;
            stack[top] = node.right as usize;
            top += 1;
        }

        hits
    }

    /// Returns the box enclosing ids[lo, hi).
    fn bounds(&self, ids: &[usize], lo: usize, hi: usize, aabbs: &[AABB]) -> AABB {
        let mut aabb = aabbs[ids[lo]];

        for i in lo + 1..hi {
            aabb = AABB::merge(&aabb, &aabbs[ids[i]]);
        }

        aabb
    }

    /// Returns the axis of the largest half-size.
    fn longest_axis(&self, aabb: &AABB) -> usize {
        if aabb.hx >= aabb.hy && aabb.hx >= aabb.hz {
            return 0;
        }

        if aabb.hy >= aabb.hz {
            return 1;
        }

        2
    }

    /// Returns the center coordinate of aabb along axis.
    fn center(&self, aabb: &AABB, axis: usize) -> f64 {
        if axis == 0 {
            return aabb.cx;
        }

        if axis == 1 {
            return aabb.cy;
        }

        aabb.cz
    }
}
