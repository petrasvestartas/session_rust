const MAXNODES: usize = 8;
const MINNODES: usize = 4;
const NOT_TAKEN: i32 = -1;
const NULL_IDX: usize = usize::MAX;

#[derive(Clone, Copy)]
struct Rect {
    m_min: [f64; 3],
    m_max: [f64; 3],
}

const EMPTY_RECT: Rect = Rect {
    m_min: [0.0; 3],
    m_max: [0.0; 3],
};

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

    fn is_leaf(&self) -> bool {
        self.m_level == 0
    }
}

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
    nodes: Vec<Node>,
    free_list: Vec<usize>,
    m_root: usize,
    m_size: i32,
}

impl SpatialRTree {
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

    pub fn count(&self) -> i32 {
        self.m_size
    }

    pub fn insert(&mut self, a_min: [f64; 3], a_max: [f64; 3], a_data: i32) {
        let branch = Branch {
            m_rect: self.make_rect(a_min, a_max),
            m_child: NULL_IDX,
            m_data: a_data,
        };
        self.insert_branch_internal(branch, 0);
        self.m_size += 1;
    }

    pub fn remove(&mut self, a_min: [f64; 3], a_max: [f64; 3], a_data: i32) -> bool {
        let rect = self.make_rect(a_min, a_max);
        let mut reinsert_list: Vec<usize> = Vec::new();
        let root = self.m_root;
        if !self.remove_rect_internal(&rect, a_data, root, &mut reinsert_list) {
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

    pub fn remove_all(&mut self) {
        self.nodes.clear();
        self.free_list.clear();
        self.m_root = self.alloc_node();
        self.m_size = 0;
    }

    pub fn search(
        &self,
        a_min: [f64; 3],
        a_max: [f64; 3],
        mut a_callback: impl FnMut(i32) -> bool,
    ) -> i32 {
        let rect = self.make_rect(a_min, a_max);
        let mut count: i32 = 0;
        self.search_internal(&rect, self.m_root, &mut count, &mut a_callback);
        count
    }

    fn alloc_node(&mut self) -> usize {
        if let Some(idx) = self.free_list.pop() {
            self.nodes[idx] = Node::new();
            return idx;
        }
        self.nodes.push(Node::new());
        self.nodes.len() - 1
    }

    fn free_node(&mut self, node: usize) {
        self.free_list.push(node);
    }

    fn make_rect(&self, a_min: [f64; 3], a_max: [f64; 3]) -> Rect {
        Rect {
            m_min: a_min,
            m_max: a_max,
        }
    }

    fn calc_rect_volume(&self, rect: &Rect) -> f64 {
        let mut volume = 1.0;
        for i in 0..3 {
            volume *= rect.m_max[i] - rect.m_min[i];
        }
        volume
    }

    fn combine_rect(&self, a: &Rect, b: &Rect) -> Rect {
        let mut rect = EMPTY_RECT;
        for i in 0..3 {
            rect.m_min[i] = a.m_min[i].min(b.m_min[i]);
            rect.m_max[i] = a.m_max[i].max(b.m_max[i]);
        }
        rect
    }

    fn overlaps(&self, a: &Rect, b: &Rect) -> bool {
        for i in 0..3 {
            if a.m_max[i] < b.m_min[i] || b.m_max[i] < a.m_min[i] {
                return false;
            }
        }
        true
    }

    fn node_cover(&self, node: usize) -> Rect {
        let mut rect = self.nodes[node].m_branch[0].m_rect;
        for i in 1..self.nodes[node].m_count as usize {
            rect = self.combine_rect(&rect, &self.nodes[node].m_branch[i].m_rect);
        }
        rect
    }

    fn add_branch(&mut self, branch: Branch, node: usize) -> Option<usize> {
        let count = self.nodes[node].m_count as usize;
        if count == MAXNODES {
            return Some(self.split_node(node, branch));
        }
        self.nodes[node].m_branch[count] = branch;
        self.nodes[node].m_count += 1;
        None
    }

    fn disconnect_branch(&mut self, node: usize, index: usize) {
        let last = self.nodes[node].m_count as usize - 1;
        assert!(index <= last);
        self.nodes[node].m_branch[index] = self.nodes[node].m_branch[last];
        self.nodes[node].m_count -= 1;
    }

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

    fn pick_seeds(&self, part_vars: &mut PartitionVars) {
        let mut seed0 = 0;
        let mut seed1 = 1;
        let mut worst = -part_vars.m_cover_split_area - 1.0;
        let mut area = [0.0; MAXNODES + 1];
        let total = part_vars.m_total as usize;
        for i in 0..total {
            area[i] = self.calc_rect_volume(&part_vars.m_branch_buf[i].m_rect);
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

    fn split_node(&mut self, node: usize, branch: Branch) -> usize {
        let mut part_vars = PartitionVars::new();
        self.get_branches(node, branch, &mut part_vars);
        self.choose_partition(&mut part_vars, MINNODES as i32);
        let new_node = self.alloc_node();
        self.nodes[new_node].m_level = self.nodes[node].m_level;
        self.load_nodes(node, new_node, &mut part_vars);
        new_node
    }

    fn insert_rect_rec(&mut self, branch: Branch, node: usize, level: i32) -> Option<usize> {
        if self.nodes[node].m_level == level {
            return self.add_branch(branch, node);
        }
        assert!(self.nodes[node].m_level > level);
        let idx = self.pick_branch(&branch.m_rect, node);
        let child = self.nodes[node].m_branch[idx].m_child;
        let other = self.insert_rect_rec(branch, child, level);
        if other.is_none() {
            self.nodes[node].m_branch[idx].m_rect =
                self.combine_rect(&self.nodes[node].m_branch[idx].m_rect, &branch.m_rect);
            return None;
        }
        self.nodes[node].m_branch[idx].m_rect = self.node_cover(child);
        let new_b = Branch {
            m_rect: self.node_cover(other.unwrap()),
            m_child: other.unwrap(),
            m_data: 0,
        };
        self.add_branch(new_b, node)
    }

    fn insert_branch_internal(&mut self, branch: Branch, level: i32) {
        let root = self.m_root;
        let new_node = self.insert_rect_rec(branch, root, level);
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

    fn search_internal(
        &self,
        rect: &Rect,
        node: usize,
        count: &mut i32,
        callback: &mut impl FnMut(i32) -> bool,
    ) -> bool {
        for i in 0..self.nodes[node].m_count as usize {
            if !self.overlaps(rect, &self.nodes[node].m_branch[i].m_rect) {
                continue;
            }
            if self.nodes[node].is_leaf() {
                *count += 1;
                if !callback(self.nodes[node].m_branch[i].m_data) {
                    return false;
                }
            } else if !self.search_internal(
                rect,
                self.nodes[node].m_branch[i].m_child,
                count,
                callback,
            ) {
                return false;
            }
        }
        true
    }

    fn remove_rect_internal(
        &mut self,
        rect: &Rect,
        data: i32,
        node: usize,
        reinsert_list: &mut Vec<usize>,
    ) -> bool {
        for i in 0..self.nodes[node].m_count as usize {
            if !self.overlaps(rect, &self.nodes[node].m_branch[i].m_rect) {
                continue;
            }
            if self.nodes[node].is_leaf() {
                if self.nodes[node].m_branch[i].m_data != data {
                    continue;
                }
                self.disconnect_branch(node, i);
                return true;
            }
            let child = self.nodes[node].m_branch[i].m_child;
            if !self.remove_rect_internal(rect, data, child, reinsert_list) {
                continue;
            }
            if self.nodes[child].m_count as usize >= MINNODES {
                self.nodes[node].m_branch[i].m_rect = self.node_cover(child);
            } else {
                reinsert_list.push(child);
                self.disconnect_branch(node, i);
            }
            return true;
        }
        false
    }
}
