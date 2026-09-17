const MAXNODES: usize = 8; // Fan-out ceiling.
const MINNODES: usize = 4; // Fan-out floor.
const NOT_TAKEN: i32 = -1; // Partition slot not yet assigned.
const NULL_IDX: usize = usize::MAX; // Absent node index.
const STACK_SIZE: usize = 64; // Explicit traversal stack depth.

/// Axis-aligned box.
#[derive(Clone, Copy)]
struct Rect {
    m_min: [f64; 3],
    m_max: [f64; 3],
}

const EMPTY_RECT: Rect = Rect {
    m_min: [0.0; 3],
    m_max: [0.0; 3],
};

/// Child pointer or leaf datum with its cover.
#[derive(Clone, Copy)]
struct Branch {
    m_rect: Rect,
    m_child: usize,
    m_data: i32,
}

const EMPTY_BRANCH: Branch = Branch {
    m_rect: EMPTY_RECT,
    m_child: NULL_IDX,
    m_data: 0,
};

/// Inner or leaf node with up to MAXNODES + 1 branches during a split.
struct Node {
    m_count: i32,
    m_level: i32,
    m_branch: [Branch; MAXNODES + 1],
}

impl Node {
    fn new() -> Self {
        Node {
            m_count: 0,
            m_level: 0,
            m_branch: [EMPTY_BRANCH; MAXNODES + 1],
        }
    }

    /// Whether the node is a leaf.
    fn is_leaf(&self) -> bool {
        self.m_level == 0
    }
}

/// Traversal stack entry.
#[derive(Clone, Copy)]
struct Visit {
    node: usize,
    index: usize,
}

const EMPTY_VISIT: Visit = Visit {
    node: NULL_IDX,
    index: 0,
};

/// Scratch state for a quadratic split.
struct PartitionVars {
    m_partition: [i32; MAXNODES + 1],
    m_total: i32,
    m_min_fill: i32,
    m_count: [i32; 2],
    m_cover: [Rect; 2],
    m_area: [f64; 2],
    m_branch_buf: [Branch; MAXNODES + 1],
    m_branch_count: i32,
    m_cover_split: Rect,
    m_cover_split_area: f64,
}

impl PartitionVars {
    fn new() -> Self {
        PartitionVars {
            m_partition: [NOT_TAKEN; MAXNODES + 1],
            m_total: 0,
            m_min_fill: 0,
            m_count: [0; 2],
            m_cover: [EMPTY_RECT; 2],
            m_area: [0.0; 2],
            m_branch_buf: [EMPTY_BRANCH; MAXNODES + 1],
            m_branch_count: 0,
            m_cover_split: EMPTY_RECT,
            m_cover_split_area: 0.0,
        }
    }
}

/// R-tree with dynamic insert and remove (Guttman quadratic split, fan-out 4 to 8) for box overlap queries.
pub struct SpatialRTree {
    nodes: Vec<Node>,      // Node arena.
    free_list: Vec<usize>, // Recycled node indices.
    m_root: usize,         // Tree root.
    m_size: i32,           // Stored item count.
}

impl SpatialRTree {
    /// Construct an empty tree with a single leaf root.
    pub fn new() -> Self {
        let mut tree = SpatialRTree {
            nodes: Vec::new(),
            free_list: Vec::new(),
            m_root: 0,
            m_size: 0,
        };
        tree.m_root = tree.alloc_node();
        tree
    }

    /// Number of stored items.
    pub fn count(&self) -> i32 {
        self.m_size
    }

    /// Insert an item with its bounding box.
    pub fn insert(&mut self, a_min: [f64; 3], a_max: [f64; 3], a_data: i32) {
        let branch = Branch {
            m_rect: self.make_rect(a_min, a_max),
            m_child: NULL_IDX,
            m_data: a_data,
        };
        self.insert_branch_internal(branch, 0);
        self.m_size += 1;
    }

    /// Remove an item by its bounding box and data; false when not found.
    pub fn remove(&mut self, a_min: [f64; 3], a_max: [f64; 3], a_data: i32) -> bool {
        let rect = self.make_rect(a_min, a_max);
        let mut reinsert_list: Vec<usize> = Vec::new();

        if !self.remove_rect_internal(&rect, a_data, &mut reinsert_list) {
            return false;
        }

        for node in reinsert_list {
            let count = self.nodes[node].m_count as usize;
            let level = self.nodes[node].m_level;

            for i in 0..count {
                let branch = self.nodes[node].m_branch[i];
                self.insert_branch_internal(branch, level);
            }

            self.free_node(node);
        }

        while !self.nodes[self.m_root].is_leaf() && self.nodes[self.m_root].m_count == 1 {
            let old_root = self.m_root;
            self.m_root = self.nodes[old_root].m_branch[0].m_child;
            self.free_node(old_root);
        }

        self.m_size -= 1;
        true
    }

    /// Remove every item.
    pub fn remove_all(&mut self) {
        self.nodes.clear();
        self.free_list.clear();
        self.m_root = self.alloc_node();
        self.m_size = 0;
    }

    /// Visit every item overlapping the box until the callback returns false; returns the visit count.
    pub fn search(
        &self,
        a_min: [f64; 3],
        a_max: [f64; 3],
        mut a_callback: impl FnMut(i32) -> bool,
    ) -> i32 {
        let rect = self.make_rect(a_min, a_max);
        let mut stack = [EMPTY_VISIT; STACK_SIZE];
        let mut top = 0;
        stack[top] = Visit {
            node: self.m_root,
            index: 0,
        };
        top += 1;
        let mut count: i32 = 0;

        while top > 0 {
            let visit = stack[top - 1];

            if visit.index == self.nodes[visit.node].m_count as usize {
                top -= 1;
                continue;
            }

            let branch = self.nodes[visit.node].m_branch[visit.index];
            stack[top - 1].index += 1;

            if !self.overlaps(&rect, &branch.m_rect) {
                continue;
            }

            if !self.nodes[visit.node].is_leaf() {
                assert!(top < STACK_SIZE);
                stack[top] = Visit {
                    node: branch.m_child,
                    index: 0,
                };
                top += 1;
                continue;
            }

            count += 1;

            if !a_callback(branch.m_data) {
                return count;
            }
        }

        count
    }

    /// Allocate an empty leaf node.
    fn alloc_node(&mut self) -> usize {
        if let Some(idx) = self.free_list.pop() {
            self.nodes[idx] = Node::new();

            return idx;
        }

        self.nodes.push(Node::new());
        self.nodes.len() - 1
    }

    /// Free one node.
    fn free_node(&mut self, node: usize) {
        self.free_list.push(node);
    }

    /// Build a rect from min and max corners.
    fn make_rect(&self, a_min: [f64; 3], a_max: [f64; 3]) -> Rect {
        Rect {
            m_min: [
                a_min[0].min(a_max[0]),
                a_min[1].min(a_max[1]),
                a_min[2].min(a_max[2]),
            ],
            m_max: [
                a_min[0].max(a_max[0]),
                a_min[1].max(a_max[1]),
                a_min[2].max(a_max[2]),
            ],
        }
    }

    /// Volume of a rect.
    fn calc_rect_volume(&self, rect: &Rect) -> f64 {
        let mut volume = 1.0;

        for i in 0..3 {
            volume *= rect.m_max[i] - rect.m_min[i];
        }

        volume
    }

    /// Smallest rect covering both.
    fn combine_rect(&self, a: &Rect, b: &Rect) -> Rect {
        let mut rect = EMPTY_RECT;

        for i in 0..3 {
            rect.m_min[i] = a.m_min[i].min(b.m_min[i]);
            rect.m_max[i] = a.m_max[i].max(b.m_max[i]);
        }

        rect
    }

    /// Whether two rects overlap.
    fn overlaps(&self, a: &Rect, b: &Rect) -> bool {
        for i in 0..3 {
            if a.m_max[i] < b.m_min[i] || b.m_max[i] < a.m_min[i] {
                return false;
            }
        }

        true
    }

    /// Rect covering every branch of a node.
    fn node_cover(&self, node: usize) -> Rect {
        let mut rect = self.nodes[node].m_branch[0].m_rect;

        for i in 1..self.nodes[node].m_count as usize {
            rect = self.combine_rect(&rect, &self.nodes[node].m_branch[i].m_rect);
        }

        rect
    }

    /// Add a branch, splitting the node when full; returns the new sibling or none.
    fn add_branch(&mut self, branch: Branch, node: usize) -> Option<usize> {
        let count = self.nodes[node].m_count as usize;

        if count == MAXNODES {
            return Some(self.split_node(node, branch));
        }

        self.nodes[node].m_branch[count] = branch;
        self.nodes[node].m_count += 1;
        None
    }

    /// Remove a branch by swapping in the last one.
    fn disconnect_branch(&mut self, node: usize, index: usize) {
        let last = self.nodes[node].m_count as usize - 1;
        assert!(index <= last);
        self.nodes[node].m_branch[index] = self.nodes[node].m_branch[last];
        self.nodes[node].m_count -= 1;
    }

    /// Branch whose rect grows least when covering the rect.
    fn pick_branch(&self, rect: &Rect, node: usize) -> usize {
        let mut best_incr = -1.0;
        let mut best_area = -1.0;
        let mut best = 0;

        for i in 0..self.nodes[node].m_count as usize {
            let cur = self.nodes[node].m_branch[i].m_rect;
            let area = self.calc_rect_volume(&cur);
            let combined = self.combine_rect(rect, &cur);
            let incr = self.calc_rect_volume(&combined) - area;

            if i == 0 || incr < best_incr || (incr == best_incr && area < best_area) {
                best = i;
                best_incr = incr;
                best_area = area;
            }
        }

        best
    }

    /// Collect the node's branches plus one extra into the partition buffer.
    fn get_branches(&mut self, node: usize, branch: Branch, part_vars: &mut PartitionVars) {
        assert!(self.nodes[node].m_count as usize == MAXNODES);

        for i in 0..MAXNODES {
            part_vars.m_branch_buf[i] = self.nodes[node].m_branch[i];
        }

        part_vars.m_branch_buf[MAXNODES] = branch;
        part_vars.m_branch_count = MAXNODES as i32 + 1;
        part_vars.m_cover_split = part_vars.m_branch_buf[0].m_rect;

        for i in 1..MAXNODES + 1 {
            part_vars.m_cover_split =
                self.combine_rect(&part_vars.m_cover_split, &part_vars.m_branch_buf[i].m_rect);
        }

        part_vars.m_cover_split_area = self.calc_rect_volume(&part_vars.m_cover_split);
        self.nodes[node].m_count = 0;
    }

    /// Reset the partition buffer.
    fn init_part_vars(&self, part_vars: &mut PartitionVars, max_rects: i32, min_fill: i32) {
        part_vars.m_count[0] = 0;
        part_vars.m_count[1] = 0;
        part_vars.m_area[0] = 0.0;
        part_vars.m_area[1] = 0.0;
        part_vars.m_total = max_rects;
        part_vars.m_min_fill = min_fill;

        for i in 0..max_rects as usize {
            part_vars.m_partition[i] = NOT_TAKEN;
        }
    }

    /// Assign a branch to a group and grow the group cover.
    fn classify_branch(&self, index: usize, group: usize, part_vars: &mut PartitionVars) {
        assert!(part_vars.m_partition[index] == NOT_TAKEN);
        part_vars.m_partition[index] = group as i32;

        if part_vars.m_count[group] == 0 {
            part_vars.m_cover[group] = part_vars.m_branch_buf[index].m_rect;
        } else {
            part_vars.m_cover[group] = self.combine_rect(
                &part_vars.m_branch_buf[index].m_rect,
                &part_vars.m_cover[group],
            );
        }

        part_vars.m_area[group] = self.calc_rect_volume(&part_vars.m_cover[group]);
        part_vars.m_count[group] += 1;
    }

    /// Seed the two groups with the most wasteful pair.
    fn pick_seeds(&self, part_vars: &mut PartitionVars) {
        let mut seed0 = 0;
        let mut seed1 = 1;
        let mut worst = -part_vars.m_cover_split_area - 1.0;
        let mut area = [0.0; MAXNODES + 1];
        let total = part_vars.m_total as usize;

        for (i, slot) in area.iter_mut().enumerate().take(total) {
            *slot = self.calc_rect_volume(&part_vars.m_branch_buf[i].m_rect);
        }

        for i in 0..total - 1 {
            for j in i + 1..total {
                let combined = self.combine_rect(
                    &part_vars.m_branch_buf[i].m_rect,
                    &part_vars.m_branch_buf[j].m_rect,
                );
                let waste = self.calc_rect_volume(&combined) - area[i] - area[j];

                if waste > worst {
                    worst = waste;
                    seed0 = i;
                    seed1 = j;
                }
            }
        }

        self.classify_branch(seed0, 0, part_vars);
        self.classify_branch(seed1, 1, part_vars);
    }

    /// Quadratic split of the partition buffer into two groups.
    fn choose_partition(&self, part_vars: &mut PartitionVars, min_fill: i32) {
        let branch_count = part_vars.m_branch_count;
        self.init_part_vars(part_vars, branch_count, min_fill);
        self.pick_seeds(part_vars);

        while (part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total
            && part_vars.m_count[0] < (part_vars.m_total - part_vars.m_min_fill)
            && part_vars.m_count[1] < (part_vars.m_total - part_vars.m_min_fill)
        {
            let mut biggest_diff = -1.0;
            let mut chosen = 0;
            let mut better_group = 0;

            for i in 0..part_vars.m_total as usize {
                if part_vars.m_partition[i] != NOT_TAKEN {
                    continue;
                }

                let r0 =
                    self.combine_rect(&part_vars.m_branch_buf[i].m_rect, &part_vars.m_cover[0]);

                let r1 =
                    self.combine_rect(&part_vars.m_branch_buf[i].m_rect, &part_vars.m_cover[1]);

                let growth0 = self.calc_rect_volume(&r0) - part_vars.m_area[0];
                let growth1 = self.calc_rect_volume(&r1) - part_vars.m_area[1];
                let mut diff = growth1 - growth0;
                let mut group = 0;

                if diff < 0.0 {
                    group = 1;
                    diff = -diff;
                }

                if diff > biggest_diff {
                    biggest_diff = diff;
                    chosen = i;
                    better_group = group;
                } else if diff == biggest_diff
                    && part_vars.m_count[group] < part_vars.m_count[better_group]
                {
                    chosen = i;
                    better_group = group;
                }
            }

            self.classify_branch(chosen, better_group, part_vars);
        }

        if (part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total {
            let group = if part_vars.m_count[0] >= part_vars.m_total - part_vars.m_min_fill {
                1
            } else {
                0
            };

            for i in 0..part_vars.m_total as usize {
                if part_vars.m_partition[i] == NOT_TAKEN {
                    self.classify_branch(i, group, part_vars);
                }
            }
        }
    }

    /// Move partitioned branches into the two nodes.
    fn load_nodes(&mut self, node_a: usize, node_b: usize, part_vars: &mut PartitionVars) {
        for i in 0..part_vars.m_total as usize {
            let target = if part_vars.m_partition[i] == 0 {
                node_a
            } else {
                node_b
            };
            self.add_branch(part_vars.m_branch_buf[i], target);
        }
    }

    /// Split a full node with the extra branch; returns the new sibling.
    fn split_node(&mut self, node: usize, branch: Branch) -> usize {
        let mut part_vars = PartitionVars::new();
        self.get_branches(node, branch, &mut part_vars);
        self.choose_partition(&mut part_vars, MINNODES as i32);
        let new_node = self.alloc_node();
        self.nodes[new_node].m_level = self.nodes[node].m_level;
        self.load_nodes(node, new_node, &mut part_vars);
        new_node
    }

    /// Insert a branch at a level; returns the root's new sibling or none.
    fn insert_rect_internal(&mut self, branch: Branch, level: i32) -> Option<usize> {
        let mut stack = [EMPTY_VISIT; STACK_SIZE];
        let mut top = 0;
        let mut node = self.m_root;

        while self.nodes[node].m_level != level {
            assert!(self.nodes[node].m_level > level);
            assert!(top < STACK_SIZE);
            let idx = self.pick_branch(&branch.m_rect, node);
            stack[top] = Visit { node, index: idx };
            top += 1;
            node = self.nodes[node].m_branch[idx].m_child;
        }

        let mut other = self.add_branch(branch, node);

        for d in (0..top).rev() {
            let parent = stack[d].node;
            let idx = stack[d].index;
            let child = self.nodes[parent].m_branch[idx].m_child;

            if other.is_none() {
                self.nodes[parent].m_branch[idx].m_rect =
                    self.combine_rect(&self.nodes[parent].m_branch[idx].m_rect, &branch.m_rect);

                continue;
            }

            self.nodes[parent].m_branch[idx].m_rect = self.node_cover(child);
            let new_b = Branch {
                m_rect: self.node_cover(other.unwrap()),
                m_child: other.unwrap(),
                m_data: 0,
            };
            other = self.add_branch(new_b, parent);
        }

        other
    }

    /// Insert a branch and grow the root when it splits.
    fn insert_branch_internal(&mut self, branch: Branch, level: i32) {
        let new_node = self.insert_rect_internal(branch, level);

        if new_node.is_none() {
            return;
        }

        let old_root = self.m_root;
        self.m_root = self.alloc_node();
        self.nodes[self.m_root].m_level = self.nodes[old_root].m_level + 1;
        let b1 = Branch {
            m_rect: self.node_cover(old_root),
            m_child: old_root,
            m_data: 0,
        };
        let b2 = Branch {
            m_rect: self.node_cover(new_node.unwrap()),
            m_child: new_node.unwrap(),
            m_data: 0,
        };
        let root = self.m_root;
        self.add_branch(b1, root);
        self.add_branch(b2, root);
    }

    /// Remove the matching leaf branch; underfull nodes go to the reinsert list.
    fn remove_rect_internal(
        &mut self,
        rect: &Rect,
        data: i32,
        reinsert_list: &mut Vec<usize>,
    ) -> bool {
        let mut stack = [EMPTY_VISIT; STACK_SIZE];
        let mut top = 0;
        stack[top] = Visit {
            node: self.m_root,
            index: 0,
        };
        top += 1;

        while top > 0 {
            let visit = stack[top - 1];

            if visit.index == self.nodes[visit.node].m_count as usize {
                top -= 1;
                continue;
            }

            let branch = self.nodes[visit.node].m_branch[visit.index];
            stack[top - 1].index += 1;

            if !self.overlaps(rect, &branch.m_rect) {
                continue;
            }

            if !self.nodes[visit.node].is_leaf() {
                assert!(top < STACK_SIZE);
                stack[top] = Visit {
                    node: branch.m_child,
                    index: 0,
                };
                top += 1;
                continue;
            }

            if branch.m_data != data {
                continue;
            }

            self.disconnect_branch(visit.node, visit.index);

            for d in (0..top - 1).rev() {
                self.shrink_branch(stack[d].node, stack[d].index - 1, reinsert_list);
            }

            return true;
        }

        false
    }

    /// Recompute a child's cover or queue it for reinsertion when underfull.
    fn shrink_branch(&mut self, node: usize, index: usize, reinsert_list: &mut Vec<usize>) {
        let child = self.nodes[node].m_branch[index].m_child;

        if self.nodes[child].m_count as usize >= MINNODES {
            self.nodes[node].m_branch[index].m_rect = self.node_cover(child);

            return;
        }

        reinsert_list.push(child);
        self.disconnect_branch(node, index);
    }
}

impl Default for SpatialRTree {
    fn default() -> Self {
        Self::new()
    }
}
