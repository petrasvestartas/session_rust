// RTree — R-tree with dynamic insert/delete (Guttman split, fan-out 4–8).
// Use for: "find all objects overlapping this region" (spatial range queries).
//   Supports live insertion and deletion; good for mutable object sets.
// Prefer over AABBTree/BVH when data changes frequently.
// Prefer over KDTree  when querying volumes/boxes, not bare point clouds.
// Note: k-NN is possible but slower than KDTree for pure point queries.
const MAXNODES: usize = 8;
const MINNODES: usize = 4;
const NOT_TAKEN: i32 = -1;
const NULL_IDX: usize = usize::MAX;

#[derive(Clone, Copy)]
struct Rect {
    m_min: [f64; 3],
    m_max: [f64; 3],
}

const EMPTY_RECT: Rect = Rect { m_min: [0.0; 3], m_max: [0.0; 3] };

#[derive(Clone, Copy)]
struct Branch {
    m_rect: Rect,
    m_child: usize,
    m_data: i32,
}

const EMPTY_BRANCH: Branch = Branch { m_rect: EMPTY_RECT, m_child: NULL_IDX, m_data: 0 };

struct RTreeNode {
    m_count: i32,
    m_level: i32,
    m_branch: [Branch; MAXNODES + 1],
}

impl RTreeNode {
    fn new() -> Self {
        RTreeNode { m_count: 0, m_level: 0, m_branch: [EMPTY_BRANCH; MAXNODES + 1] }
    }
    fn is_leaf(&self) -> bool { self.m_level == 0 }
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

pub struct RTree {
    nodes: Vec<RTreeNode>,
    free_list: Vec<usize>,
    m_root: usize,
    m_size: i32,
}

impl RTree {
    pub fn new() -> Self {
        let mut rt = RTree { nodes: Vec::new(), free_list: Vec::new(), m_root: 0, m_size: 0 };
        rt.m_root = rt.alloc_node();
        rt
    }

    fn alloc_node(&mut self) -> usize {
        if let Some(idx) = self.free_list.pop() {
            self.nodes[idx] = RTreeNode::new();
            idx
        } else {
            let idx = self.nodes.len();
            self.nodes.push(RTreeNode::new());
            idx
        }
    }

    fn free_node(&mut self, idx: usize) {
        self.free_list.push(idx);
    }

    #[allow(dead_code)]
    fn free_subtree(&mut self, node_idx: usize) {
        if !self.nodes[node_idx].is_leaf() {
            let count = self.nodes[node_idx].m_count as usize;
            let children: Vec<usize> = (0..count).map(|i| self.nodes[node_idx].m_branch[i].m_child).collect();
            for child in children {
                self.free_subtree(child);
            }
        }
        self.free_node(node_idx);
    }

    fn make_rect(&self, a_min: [f64; 3], a_max: [f64; 3]) -> Rect {
        Rect { m_min: a_min, m_max: a_max }
    }

    fn calc_rect_volume(&self, rect: &Rect) -> f64 {
        let mut volume = 1.0f64;
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

    fn node_cover(&self, node_idx: usize) -> Rect {
        let mut rect = self.nodes[node_idx].m_branch[0].m_rect;
        let count = self.nodes[node_idx].m_count as usize;
        for i in 1..count {
            let br = self.nodes[node_idx].m_branch[i].m_rect;
            rect = self.combine_rect(&rect, &br);
        }
        rect
    }

    fn add_branch(&mut self, branch: Branch, node_idx: usize, new_node: &mut Option<usize>) -> bool {
        let count = self.nodes[node_idx].m_count as usize;
        if count < MAXNODES {
            self.nodes[node_idx].m_branch[count] = branch;
            self.nodes[node_idx].m_count += 1;
            false
        } else {
            let nn = self.split_node(node_idx, branch);
            *new_node = Some(nn);
            true
        }
    }

    fn disconnect_branch(&mut self, node_idx: usize, index: usize) {
        let last = self.nodes[node_idx].m_count as usize - 1;
        self.nodes[node_idx].m_branch[index] = self.nodes[node_idx].m_branch[last];
        self.nodes[node_idx].m_count -= 1;
    }

    fn pick_branch(&self, rect: &Rect, node_idx: usize) -> usize {
        let mut best_incr: f64 = -1.0;
        let mut best_area: f64 = -1.0;
        let mut best: usize = 0;
        let count = self.nodes[node_idx].m_count as usize;
        for i in 0..count {
            let cur = self.nodes[node_idx].m_branch[i].m_rect;
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

    fn get_branches(&mut self, node_idx: usize, branch: Branch, part_vars: &mut PartitionVars) {
        let count = self.nodes[node_idx].m_count as usize;
        assert!(count == MAXNODES);
        for i in 0..MAXNODES {
            part_vars.m_branch_buf[i] = self.nodes[node_idx].m_branch[i];
        }
        part_vars.m_branch_buf[MAXNODES] = branch;
        part_vars.m_branch_count = MAXNODES as i32 + 1;
        let mut cover_split = part_vars.m_branch_buf[0].m_rect;
        for i in 1..MAXNODES + 1 {
            cover_split = self.combine_rect(&cover_split, &part_vars.m_branch_buf[i].m_rect);
        }
        part_vars.m_cover_split = cover_split;
        part_vars.m_cover_split_area = self.calc_rect_volume(&cover_split);
        self.nodes[node_idx].m_count = 0;
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
            let combined = self.combine_rect(&part_vars.m_branch_buf[index].m_rect, &part_vars.m_cover[group]);
            part_vars.m_cover[group] = combined;
        }
        part_vars.m_area[group] = self.calc_rect_volume(&part_vars.m_cover[group]);
        part_vars.m_count[group] += 1;
    }

    fn pick_seeds(&self, part_vars: &mut PartitionVars) {
        let mut seed0: usize = 0;
        let mut seed1: usize = 1;
        let mut worst = -part_vars.m_cover_split_area - 1.0;
        let mut area = [0.0f64; MAXNODES + 1];
        for i in 0..part_vars.m_total as usize {
            area[i] = self.calc_rect_volume(&part_vars.m_branch_buf[i].m_rect);
        }
        for i in 0..part_vars.m_total as usize - 1 {
            for j in i + 1..part_vars.m_total as usize {
                let combined = self.combine_rect(&part_vars.m_branch_buf[i].m_rect, &part_vars.m_branch_buf[j].m_rect);
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
        while (part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total &&
              part_vars.m_count[0] < (part_vars.m_total - part_vars.m_min_fill) &&
              part_vars.m_count[1] < (part_vars.m_total - part_vars.m_min_fill)
        {
            let mut biggest_diff: f64 = -1.0;
            let mut chosen: usize = 0;
            let mut better_group: usize = 0;
            for i in 0..part_vars.m_total as usize {
                if part_vars.m_partition[i] == NOT_TAKEN {
                    let r0 = self.combine_rect(&part_vars.m_branch_buf[i].m_rect, &part_vars.m_cover[0]);
                    let r1 = self.combine_rect(&part_vars.m_branch_buf[i].m_rect, &part_vars.m_cover[1]);
                    let growth0 = self.calc_rect_volume(&r0) - part_vars.m_area[0];
                    let growth1 = self.calc_rect_volume(&r1) - part_vars.m_area[1];
                    let mut diff = growth1 - growth0;
                    let group: usize;
                    if diff >= 0.0 {
                        group = 0;
                    } else {
                        group = 1;
                        diff = -diff;
                    }
                    if diff > biggest_diff {
                        biggest_diff = diff;
                        chosen = i;
                        better_group = group;
                    } else if diff == biggest_diff && part_vars.m_count[group] < part_vars.m_count[better_group] {
                        chosen = i;
                        better_group = group;
                    }
                }
            }
            self.classify_branch(chosen, better_group, part_vars);
        }
        if (part_vars.m_count[0] + part_vars.m_count[1]) < part_vars.m_total {
            let group: usize = if part_vars.m_count[0] >= part_vars.m_total - part_vars.m_min_fill { 1 } else { 0 };
            for i in 0..part_vars.m_total as usize {
                if part_vars.m_partition[i] == NOT_TAKEN {
                    self.classify_branch(i, group, part_vars);
                }
            }
        }
    }

    fn load_nodes(&mut self, node_a: usize, node_b: usize, part_vars: &mut PartitionVars) {
        for i in 0..part_vars.m_total as usize {
            let g = part_vars.m_partition[i] as usize;
            let target = if g == 0 { node_a } else { node_b };
            let branch = part_vars.m_branch_buf[i];
            let mut dummy: Option<usize> = None;
            self.add_branch(branch, target, &mut dummy);
        }
    }

    fn split_node(&mut self, node_idx: usize, branch: Branch) -> usize {
        let mut part_vars = PartitionVars::new();
        self.get_branches(node_idx, branch, &mut part_vars);
        self.choose_partition(&mut part_vars, MINNODES as i32);
        let new_node = self.alloc_node();
        let level = self.nodes[node_idx].m_level;
        self.nodes[new_node].m_level = level;
        self.load_nodes(node_idx, new_node, &mut part_vars);
        new_node
    }

    fn insert_rect_rec(&mut self, branch: Branch, node_idx: usize, level: i32) -> Option<usize> {
        let node_level = self.nodes[node_idx].m_level;
        if node_level > level {
            let idx = self.pick_branch(&branch.m_rect, node_idx);
            let branch_rect = branch.m_rect;
            let child_idx = self.nodes[node_idx].m_branch[idx].m_child;
            let other = self.insert_rect_rec(branch, child_idx, level);
            if other.is_none() {
                let br_rect = self.nodes[node_idx].m_branch[idx].m_rect;
                let combined = self.combine_rect(&br_rect, &branch_rect);
                self.nodes[node_idx].m_branch[idx].m_rect = combined;
                None
            } else {
                let new_idx = other.unwrap();
                let child_cover = self.node_cover(child_idx);
                self.nodes[node_idx].m_branch[idx].m_rect = child_cover;
                let new_cover = self.node_cover(new_idx);
                let new_b = Branch { m_rect: new_cover, m_child: new_idx, m_data: 0 };
                let mut out: Option<usize> = None;
                self.add_branch(new_b, node_idx, &mut out);
                out
            }
        } else if node_level == level {
            let mut out: Option<usize> = None;
            self.add_branch(branch, node_idx, &mut out);
            out
        } else {
            None
        }
    }

    fn insert_branch_internal(&mut self, branch: Branch, level: i32) {
        let root = self.m_root;
        let maybe_split = self.insert_rect_rec(branch, root, level);
        if let Some(new_node) = maybe_split {
            let old_root = self.m_root;
            let new_root = self.alloc_node();
            let old_level = self.nodes[old_root].m_level;
            self.nodes[new_root].m_level = old_level + 1;
            let b1 = Branch { m_rect: self.node_cover(old_root), m_child: old_root, m_data: 0 };
            let b2 = Branch { m_rect: self.node_cover(new_node), m_child: new_node, m_data: 0 };
            let mut dummy: Option<usize> = None;
            self.add_branch(b1, new_root, &mut dummy);
            self.add_branch(b2, new_root, &mut dummy);
            self.m_root = new_root;
        }
    }

    fn search_internal(&self, rect: &Rect, node_idx: usize, count: &mut i32, callback: &mut impl FnMut(i32) -> bool) -> bool {
        if self.nodes[node_idx].is_leaf() {
            let node_count = self.nodes[node_idx].m_count as usize;
            for i in 0..node_count {
                let br_rect = self.nodes[node_idx].m_branch[i].m_rect;
                if self.overlaps(rect, &br_rect) {
                    *count += 1;
                    let br_data = self.nodes[node_idx].m_branch[i].m_data;
                    if !callback(br_data) {
                        return false;
                    }
                }
            }
        } else {
            let node_count = self.nodes[node_idx].m_count as usize;
            for i in 0..node_count {
                let br_rect = self.nodes[node_idx].m_branch[i].m_rect;
                if self.overlaps(rect, &br_rect) {
                    let child = self.nodes[node_idx].m_branch[i].m_child;
                    if !self.search_internal(rect, child, count, callback) {
                        return false;
                    }
                }
            }
        }
        true
    }

    fn remove_rect_internal(&mut self, rect: &Rect, data: i32, node_idx: usize, reinsert_list: &mut Vec<usize>) -> bool {
        if self.nodes[node_idx].is_leaf() {
            let count = self.nodes[node_idx].m_count as usize;
            for i in 0..count {
                let br_data = self.nodes[node_idx].m_branch[i].m_data;
                let br_rect = self.nodes[node_idx].m_branch[i].m_rect;
                if br_data == data && self.overlaps(rect, &br_rect) {
                    self.disconnect_branch(node_idx, i);
                    return true;
                }
            }
            false
        } else {
            let count = self.nodes[node_idx].m_count as usize;
            for i in 0..count {
                let child_rect = self.nodes[node_idx].m_branch[i].m_rect;
                if self.overlaps(rect, &child_rect) {
                    let child_idx = self.nodes[node_idx].m_branch[i].m_child;
                    if self.remove_rect_internal(rect, data, child_idx, reinsert_list) {
                        let child_count = self.nodes[child_idx].m_count;
                        if child_count >= MINNODES as i32 {
                            let new_cover = self.node_cover(child_idx);
                            self.nodes[node_idx].m_branch[i].m_rect = new_cover;
                        } else {
                            reinsert_list.push(child_idx);
                            self.disconnect_branch(node_idx, i);
                        }
                        return true;
                    }
                }
            }
            false
        }
    }

    pub fn insert(&mut self, a_min: [f64; 3], a_max: [f64; 3], a_data: i32) {
        let branch = Branch { m_rect: self.make_rect(a_min, a_max), m_child: NULL_IDX, m_data: a_data };
        self.insert_branch_internal(branch, 0);
        self.m_size += 1;
    }

    pub fn remove(&mut self, a_min: [f64; 3], a_max: [f64; 3], a_data: i32) {
        let rect = self.make_rect(a_min, a_max);
        let mut reinsert_list: Vec<usize> = Vec::new();
        let root = self.m_root;
        if !self.remove_rect_internal(&rect, a_data, root, &mut reinsert_list) {
            return;
        }
        for node_idx in reinsert_list {
            let count = self.nodes[node_idx].m_count;
            let level = self.nodes[node_idx].m_level;
            for i in 0..count as usize {
                let branch = self.nodes[node_idx].m_branch[i];
                self.insert_branch_internal(branch, level);
            }
            self.free_node(node_idx);
        }
        loop {
            let root = self.m_root;
            if !self.nodes[root].is_leaf() && self.nodes[root].m_count == 1 {
                let child = self.nodes[root].m_branch[0].m_child;
                self.free_node(root);
                self.m_root = child;
            } else {
                break;
            }
        }
        self.m_size -= 1;
    }

    pub fn search(&self, a_min: [f64; 3], a_max: [f64; 3], mut a_callback: impl FnMut(i32) -> bool) -> i32 {
        let rect = self.make_rect(a_min, a_max);
        let mut count: i32 = 0;
        self.search_internal(&rect, self.m_root, &mut count, &mut a_callback);
        count
    }

    pub fn remove_all(&mut self) {
        self.nodes.clear();
        self.free_list.clear();
        let root = self.alloc_node();
        self.m_root = root;
        self.m_size = 0;
    }

    pub fn count(&self) -> i32 { self.m_size }
}
