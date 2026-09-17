use crate::aabb::AABB;
use crate::obb::OBB;
use crate::point::Point;
use crate::vector::Vector;

const STACK_SIZE: usize = 64; // Depth bound of the explicit traversal stack.
const NULL_IDX: i32 = -1; // Index of a missing child or object.

/// Tree node.
#[derive(Clone, Copy, Debug)]
pub struct Node {
    pub aabb: AABB,     // Bounds of the subtree.
    pub left: i32,      // Left child index, NULL_IDX on a leaf.
    pub right: i32,     // Right child index, NULL_IDX on a leaf.
    pub object_id: i32, // Object id on a leaf, NULL_IDX on an internal node.
}

impl Node {
    /// Constructs an empty leafless node.
    pub fn new() -> Self {
        Node {
            aabb: AABB::default(),
            left: NULL_IDX,
            right: NULL_IDX,
            object_id: NULL_IDX,
        }
    }

    /// Returns whether the node holds an object.
    pub fn is_leaf(&self) -> bool {
        self.object_id != NULL_IDX
    }
}

impl Default for Node {
    /// Constructs an empty leafless node.
    fn default() -> Self {
        Self::new()
    }
}

/// Returns t clamped to [0, 1] scaled to 10 bits.
fn quantize(t: f64) -> u32 {
    (t.clamp(0.0, 1.0) * 1023.0) as u32
}

/// Linear BVH (Karras 2012): leaves in Morton order, internal node i splits the sorted range it covers, node 0 is the root.
#[derive(Clone, Debug)]
pub struct SpatialBVH {
    guid: std::sync::OnceLock<String>, // Lazy guid.
    pub name: String,                  // Tree name.
    pub world_size: f64,               // Extent of the Morton cube.
    pub object_guids: Vec<String>,     // Guid per object id, set by build_with_guids.
    pub nodes: Vec<Node>,              // Internal nodes first, then the leaves in Morton order.
}

impl Default for SpatialBVH {
    /// Constructs an empty tree over a Morton cube of 1000.
    fn default() -> Self {
        Self::new()
    }
}

impl SpatialBVH {
    /// Constructs an empty tree over a Morton cube of 1000.
    pub fn new() -> Self {
        SpatialBVH {
            guid: std::sync::OnceLock::new(),
            name: "my_bvh".to_string(),
            world_size: 1000.0,
            object_guids: Vec::new(),
            nodes: Vec::new(),
        }
    }

    /// Returns whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Returns the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Sets the guid if it has not already been created.
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Constructs and builds over the boxes with the given world size.
    pub fn from_boxes(bounding_boxes: &[OBB], world_size: f64) -> Self {
        let mut bvh = Self::new();
        bvh.world_size = world_size;
        bvh.build(bounding_boxes);

        bvh
    }

    /// Returns whether the tree has no nodes.
    pub fn empty(&self) -> bool {
        self.nodes.is_empty()
    }

    /// Returns the node count.
    pub fn size(&self) -> usize {
        self.nodes.len()
    }

    /// Returns the largest absolute box coordinate times 2.2, at least 10.
    pub fn compute_world_size(bounding_boxes: &[OBB]) -> f64 {
        if bounding_boxes.is_empty() {
            return 1000.0;
        }

        let mut max_extent: f64 = 0.0;

        for bbox in bounding_boxes {
            for k in 0..3 {
                max_extent = max_extent.max(bbox.center[k].abs() + bbox.half_size[k]);
            }
        }

        (max_extent * 2.2).max(10.0)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Build
    // ═══════════════════════════════════════════════════════════════════════════

    /// Builds over the boxes with the current world size.
    pub fn build(&mut self, bounding_boxes: &[OBB]) {
        self.build_from_boxes(bounding_boxes, self.world_size);
    }

    /// Builds over the boxes with world size ws.
    pub fn build_from_boxes(&mut self, boxes: &[OBB], ws: f64) {
        let mut aabbs: Vec<AABB> = Vec::with_capacity(boxes.len());

        for bbox in boxes {
            aabbs.push(Self::aabb_from_obb(bbox));
        }

        self.build_from_aabbs(&aabbs, ws);
    }

    /// Builds over the axis-aligned boxes with world size ws.
    pub fn build_from_aabbs(&mut self, aabbs: &[AABB], ws: f64) {
        self.world_size = ws;
        self.nodes.clear();
        let n = aabbs.len() as i32;

        if n == 0 {
            return;
        }

        let codes = self.sorted_codes(aabbs);
        let leaf = n - 1;
        self.nodes.resize((n - 1) as usize, Node::new());

        for code in &codes {
            self.nodes.push(Node {
                aabb: aabbs[code.1],
                left: NULL_IDX,
                right: NULL_IDX,
                object_id: code.1 as i32,
            });
        }

        let mut order: Vec<(i32, i32)> = vec![(0, 0); (n - 1) as usize];

        for i in 0..n - 1 {
            let (first, last) = self.determine_range(&codes, i);
            let split = self.find_split(&codes, first, last);
            self.nodes[i as usize].left = if split == first { leaf + split } else { split };
            self.nodes[i as usize].right = if split + 1 == last {
                leaf + split + 1
            } else {
                split + 1
            };
            order[i as usize] = (last - first, i);
        }

        order.sort_unstable();

        for item in &order {
            let node = self.nodes[item.1 as usize];
            self.nodes[item.1 as usize].aabb = AABB::merge(
                &self.nodes[node.left as usize].aabb,
                &self.nodes[node.right as usize].aabb,
            );
        }
    }

    /// Builds over boxes paired with their guids, world size computed from the boxes.
    pub fn build_with_guids(&mut self, boxes_with_guids: &[(OBB, String)]) {
        let mut bounding_boxes: Vec<OBB> = Vec::new();
        self.object_guids.clear();

        for (bbox, guid) in boxes_with_guids {
            bounding_boxes.push(bbox.clone());
            self.object_guids.push(guid.clone());
        }

        self.world_size = Self::compute_world_size(&bounding_boxes);
        self.build(&bounding_boxes);
    }

    /// Returns (morton code, id) sorted by code, codes quantized over the bounding cube of the box centers.
    fn sorted_codes(&self, aabbs: &[AABB]) -> Vec<(u32, usize)> {
        let mut lo = [0.0; 3];
        let mut hi = [0.0; 3];

        for k in 0..3 {
            lo[k] = self.center(&aabbs[0], k);
            hi[k] = lo[k];
        }

        for aabb in &aabbs[1..] {
            for k in 0..3 {
                lo[k] = lo[k].min(self.center(aabb, k));
                hi[k] = hi[k].max(self.center(aabb, k));
            }
        }

        let ext = (hi[0] - lo[0]).max(hi[1] - lo[1]).max(hi[2] - lo[2]);
        let mut codes: Vec<(u32, usize)> = Vec::with_capacity(aabbs.len());

        for (i, aabb) in aabbs.iter().enumerate() {
            let mut code = 0u32;

            for (k, low) in lo.iter().enumerate() {
                let t = if ext > 0.0 {
                    (self.center(aabb, k) - low) / ext
                } else {
                    0.0
                };
                code |= expand_bits(quantize(t)) << k;
            }

            codes.push((code, i));
        }

        codes.sort_unstable();

        codes
    }

    /// Returns the leading bits shared by codes i and j, ties broken by index; -1 when j is out of range.
    fn common_prefix(&self, codes: &[(u32, usize)], i: i32, j: i32) -> i32 {
        if j < 0 || j >= codes.len() as i32 {
            return -1;
        }

        if codes[i as usize].0 != codes[j as usize].0 {
            return (codes[i as usize].0 ^ codes[j as usize].0).leading_zeros() as i32;
        }

        32 + (i as u32 ^ j as u32).leading_zeros() as i32
    }

    /// Returns the sorted range [first, last] covered by internal node i.
    fn determine_range(&self, codes: &[(u32, usize)], i: i32) -> (i32, i32) {
        let d = if self.common_prefix(codes, i, i + 1) > self.common_prefix(codes, i, i - 1) {
            1
        } else {
            -1
        };
        let delta_min = self.common_prefix(codes, i, i - d);
        let mut length = 1;

        while self.common_prefix(codes, i, i + length * d) > delta_min {
            length *= 2;
        }

        let mut bound = 0;
        let mut step = length / 2;

        while step > 0 {
            if self.common_prefix(codes, i, i + (bound + step) * d) > delta_min {
                bound += step;
            }

            step /= 2;
        }

        let j = i + bound * d;

        (i.min(j), i.max(j))
    }

    /// Returns the last index of the left half of [first, last].
    fn find_split(&self, codes: &[(u32, usize)], first: i32, last: i32) -> i32 {
        let common = self.common_prefix(codes, first, last);
        let mut split = first;
        let mut step = last - first;

        while step > 1 {
            step = (step + 1) / 2;

            if split + step < last && self.common_prefix(codes, first, split + step) > common {
                split += step;
            }
        }

        split
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Queries
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the overlapping (i, j) pairs with i < j, the ids in any pair, and the number of nodes tested.
    pub fn check_all_collisions(
        &self,
        bounding_boxes: &[OBB],
    ) -> (Vec<(usize, usize)>, Vec<usize>, i32) {
        let mut pairs: Vec<(usize, usize)> = Vec::new();
        let mut visited = vec![false; bounding_boxes.len()];
        let mut total_checks = 0;

        for i in 0..bounding_boxes.len() {
            let found = self.find_collisions(i, &bounding_boxes[i], bounding_boxes);
            total_checks += found.1;

            for &j in &found.0 {
                if j < i {
                    continue;
                }

                pairs.push((i, j));
                visited[i] = true;
                visited[j] = true;
            }
        }

        let mut colliding_indices: Vec<usize> = Vec::new();

        for (i, flag) in visited.iter().enumerate() {
            if *flag {
                colliding_indices.push(i);
            }
        }

        (pairs, colliding_indices, total_checks)
    }

    /// Returns the overlapping pairs as guid pairs.
    pub fn check_all_collisions_guids(&self, bounding_boxes: &[OBB]) -> Vec<(String, String)> {
        let (pairs, _colliding_indices, _total_checks) = self.check_all_collisions(bounding_boxes);
        let mut guid_pairs: Vec<(String, String)> = Vec::new();

        for (i, j) in &pairs {
            if *i < self.object_guids.len() && *j < self.object_guids.len() {
                guid_pairs.push((self.object_guids[*i].clone(), self.object_guids[*j].clone()));
            }
        }

        guid_pairs
    }

    /// Returns the ids overlapping query_bbox other than object_id, and the number of nodes tested.
    pub fn find_collisions(
        &self,
        object_id: usize,
        query_bbox: &OBB,
        bounding_boxes: &[OBB],
    ) -> (Vec<usize>, i32) {
        let mut collisions: Vec<usize> = Vec::new();
        let mut check_count = 0;
        let query = Self::aabb_from_obb(query_bbox);
        let mut stack = [0usize; STACK_SIZE];
        let mut top = 0;

        if !self.nodes.is_empty() {
            stack[top] = 0;
            top += 1;
        }

        while top > 0 {
            top -= 1;
            let node = &self.nodes[stack[top]];

            if !node.aabb.intersects(&query) {
                continue;
            }

            check_count += 1;

            if node.is_leaf() {
                let id = node.object_id as usize;

                if id != object_id
                    && id < bounding_boxes.len()
                    && query.intersects(&Self::aabb_from_obb(&bounding_boxes[id]))
                {
                    collisions.push(id);
                }

                continue;
            }

            assert!(top + 2 <= STACK_SIZE);
            stack[top] = node.left as usize;
            top += 1;
            stack[top] = node.right as usize;
            top += 1;
        }

        (collisions, check_count)
    }

    /// Returns the ids of every leaf box that intersects query.
    pub fn query_aabb(&self, query: &AABB) -> Vec<usize> {
        let mut hits: Vec<usize> = Vec::new();
        let mut stack = [0usize; STACK_SIZE];
        let mut top = 0;

        if !self.nodes.is_empty() {
            stack[top] = 0;
            top += 1;
        }

        while top > 0 {
            top -= 1;
            let node = &self.nodes[stack[top]];

            if !node.aabb.intersects(query) {
                continue;
            }

            if node.is_leaf() {
                hits.push(node.object_id as usize);
                continue;
            }

            assert!(top + 2 <= STACK_SIZE);
            stack[top] = node.left as usize;
            top += 1;
            stack[top] = node.right as usize;
            top += 1;
        }

        hits
    }

    /// Returns the ids of every leaf box that intersects the axis-aligned bounds of query.
    pub fn query_obb(&self, query: &OBB) -> Vec<usize> {
        self.query_aabb(&Self::aabb_from_obb(query))
    }

    /// Returns the ids overlapping the box of object_id with its half-sizes scaled by inflate, object_id excluded.
    pub fn nearest_neighbors(
        &self,
        object_id: usize,
        bounding_boxes: &[OBB],
        inflate: f64,
    ) -> Vec<usize> {
        let mut result: Vec<usize> = Vec::new();

        if object_id >= bounding_boxes.len() {
            return result;
        }

        let mut query = Self::aabb_from_obb(&bounding_boxes[object_id]);
        query.hx *= inflate;
        query.hy *= inflate;
        query.hz *= inflate;

        for id in self.query_aabb(&query) {
            if id != object_id {
                result.push(id);
            }
        }

        result
    }

    /// Collects the leaf ids whose box the ray enters, nearest entry first; true when any.
    pub fn ray_cast(
        &self,
        origin: &Point,
        direction: &Vector,
        candidate_leaf_ids: &mut Vec<usize>,
        _find_all: bool,
    ) -> bool {
        candidate_leaf_ids.clear();
        let mut found: Vec<(f64, usize)> = Vec::new();
        let mut stack = [0usize; STACK_SIZE];
        let mut top = 0;

        if !self.nodes.is_empty() {
            stack[top] = 0;
            top += 1;
        }

        while top > 0 {
            top -= 1;
            let node = &self.nodes[stack[top]];
            let span = self.ray_aabb(origin, direction, &node.aabb);

            if span.1 < span.0 || span.1 < 0.0 {
                continue;
            }

            if node.is_leaf() {
                found.push((span.0, node.object_id as usize));
                continue;
            }

            assert!(top + 2 <= STACK_SIZE);
            stack[top] = node.left as usize;
            top += 1;
            stack[top] = node.right as usize;
            top += 1;
        }

        found.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));

        for hit in &found {
            candidate_leaf_ids.push(hit.1);
        }

        !candidate_leaf_ids.is_empty()
    }

    /// Returns the (entry, exit) ray parameters of the box slabs; a miss when exit < entry.
    fn ray_aabb(&self, origin: &Point, direction: &Vector, aabb: &AABB) -> (f64, f64) {
        let mut tmin = f64::NEG_INFINITY;
        let mut tmax = f64::INFINITY;

        for k in 0..3 {
            let inv = if direction[k] != 0.0 {
                1.0 / direction[k]
            } else {
                f64::INFINITY
            };
            let t1 = (self.center(aabb, k) - self.half(aabb, k) - origin[k]) * inv;
            let t2 = (self.center(aabb, k) + self.half(aabb, k) - origin[k]) * inv;
            tmin = tmin.max(t1.min(t2));
            tmax = tmax.min(t1.max(t2));
        }

        (tmin, tmax)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Boxes
    // ═══════════════════════════════════════════════════════════════════════════

    /// Returns the axis-aligned box enclosing both boxes.
    pub fn merge_aabb(&self, aabb1: &OBB, aabb2: &OBB) -> OBB {
        let merged = AABB::merge(&Self::aabb_from_obb(aabb1), &Self::aabb_from_obb(aabb2));
        OBB::new(
            merged.center(),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            Vector::new(merged.hx, merged.hy, merged.hz),
        )
    }

    /// Returns whether the boxes overlap.
    pub fn aabb_intersect(&self, aabb1: &AABB, aabb2: &AABB) -> bool {
        aabb1.intersects(aabb2)
    }

    /// Returns whether the axis-aligned bounds of the boxes overlap.
    pub fn obb_intersect(&self, obb1: &OBB, obb2: &OBB) -> bool {
        Self::aabb_from_obb(obb1).intersects(&Self::aabb_from_obb(obb2))
    }

    /// Returns the axis-aligned bounds of obb.
    fn aabb_from_obb(obb: &OBB) -> AABB {
        let mut half = [0.0; 3];

        for (k, extent) in half.iter_mut().enumerate() {
            *extent = obb.x_axis[k].abs() * obb.half_size[0]
                + obb.y_axis[k].abs() * obb.half_size[1]
                + obb.z_axis[k].abs() * obb.half_size[2];
        }

        AABB::new(
            obb.center[0],
            obb.center[1],
            obb.center[2],
            half[0],
            half[1],
            half[2],
        )
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

    /// Returns the half-size of aabb along axis.
    fn half(&self, aabb: &AABB, axis: usize) -> f64 {
        if axis == 0 {
            return aabb.hx;
        }

        if axis == 1 {
            return aabb.hy;
        }

        aabb.hz
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Morton codes
// ═══════════════════════════════════════════════════════════════════════════

/// Spreads the low 10 bits of v to every third bit.
pub fn expand_bits(v: u32) -> u32 {
    let mut v = v;
    v = v.wrapping_mul(0x00010001) & 0xFF0000FF;
    v = v.wrapping_mul(0x00000101) & 0x0F00F00F;
    v = v.wrapping_mul(0x00000011) & 0xC30C30C3;
    v = v.wrapping_mul(0x00000005) & 0x49249249;

    v
}

/// Returns the Morton code of a point in the cube of world_size centered at the origin.
pub fn calculate_morton_code(x: f64, y: f64, z: f64, world_size: f64) -> u32 {
    let half = world_size * 0.5;
    let ix = quantize((x + half) / world_size);
    let iy = quantize((y + half) / world_size);
    let iz = quantize((z + half) / world_size);

    expand_bits(ix) | (expand_bits(iy) << 1) | (expand_bits(iz) << 2)
}
