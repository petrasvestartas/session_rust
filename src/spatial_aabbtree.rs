// SpatialAABBTree — flat contiguous BVH over axis-aligned boxes (SAH median split).
// Use for: closest-point on static mesh faces, ray-mesh intersection.
//   Build once, query many times. Cache-friendly nodes.
// Prefer over SpatialBVH  when geometry is static and all volumes are world-aligned.
// Prefer over SpatialRTree when no dynamic insert/delete is needed.
// Prefer over SpatialKDTree when querying faces/volumes, not bare point clouds.
use crate::aabb::AABB;

#[derive(Clone, Debug, Default)]
pub struct Node {
    pub aabb: AABB,
    pub right: i32,      // right child index; left child = this_index + 1
    pub object_id: i32,  // leaf: primitive id (>=0), internal: -1
}

#[derive(Clone, Debug, Default)]
pub struct SpatialAABBTree {
    pub nodes: Vec<Node>,
}

impl SpatialAABBTree {
    pub fn new() -> Self { Self::default() }

    pub fn build(&mut self, aabbs: &[AABB]) {
        self.nodes.clear();
        if aabbs.is_empty() { return; }
        self.nodes.reserve(2 * aabbs.len() - 1);
        let mut ids: Vec<i32> = (0..aabbs.len() as i32).collect();
        self.build_node(&mut ids, aabbs);
    }

    pub fn empty(&self) -> bool { self.nodes.is_empty() }
    pub fn size(&self) -> usize { self.nodes.len() }

    pub fn query_aabb(&self, query: &AABB) -> Vec<i32> {
        let mut hits = Vec::new();
        if self.empty() { return hits; }
        let mut stack = vec![0i32];
        while let Some(idx) = stack.pop() {
            let node = &self.nodes[idx as usize];
            if !node.aabb.intersects(query) { continue; }
            if node.object_id >= 0 {
                hits.push(node.object_id);
            } else {
                stack.push(idx + 1);
                stack.push(node.right);
            }
        }
        hits
    }

    fn build_node(&mut self, ids: &mut [i32], aabbs: &[AABB]) {
        let idx = self.nodes.len();
        self.nodes.push(Node { aabb: AABB::default(), right: -1, object_id: -1 });

        let mut lo_x: f64 = 1e308; let mut lo_y: f64 = 1e308; let mut lo_z: f64 = 1e308;
        let mut hi_x: f64 = -1e308; let mut hi_y: f64 = -1e308; let mut hi_z: f64 = -1e308;
        for &id in ids.iter() {
            let b = &aabbs[id as usize];
            lo_x = lo_x.min(b.cx - b.hx); hi_x = hi_x.max(b.cx + b.hx);
            lo_y = lo_y.min(b.cy - b.hy); hi_y = hi_y.max(b.cy + b.hy);
            lo_z = lo_z.min(b.cz - b.hz); hi_z = hi_z.max(b.cz + b.hz);
        }
        self.nodes[idx].aabb = AABB::new(
            (lo_x + hi_x) * 0.5, (lo_y + hi_y) * 0.5, (lo_z + hi_z) * 0.5,
            (hi_x - lo_x) * 0.5, (hi_y - lo_y) * 0.5, (hi_z - lo_z) * 0.5,
        );

        if ids.len() == 1 {
            self.nodes[idx].object_id = ids[0];
            return;
        }

        let dx = hi_x - lo_x; let dy = hi_y - lo_y; let dz = hi_z - lo_z;
        let axis = if dx >= dy && dx >= dz { 0 } else if dy >= dz { 1 } else { 2 };
        let mid = ids.len() / 2;

        ids.select_nth_unstable_by(mid, |&a, &b| {
            let ba = &aabbs[a as usize]; let bb = &aabbs[b as usize];
            let (ka, kb) = match axis { 0 => (ba.cx, bb.cx), 1 => (ba.cy, bb.cy), _ => (ba.cz, bb.cz) };
            ka.partial_cmp(&kb).unwrap()
        });

        let (left_ids, right_ids) = ids.split_at_mut(mid);
        self.build_node(left_ids, aabbs);
        self.nodes[idx].right = self.nodes.len() as i32;
        self.build_node(right_ids, aabbs);
    }
}
