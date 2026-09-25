use crate::color::Color;
use crate::graph::Edge;
use crate::graph::Vertex;
use crate::interaction::Interaction;
use crate::session::Geometry;
use crate::session::Item;
use crate::session::Session;
use crate::tree::TreeNode;
use crate::xform::Xform;
use crate::BRep;
use crate::Mesh;
use std::cell::Cell;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fmt;
use std::rc::Rc;

pub const CAPACITY: usize = 64; // Committed transactions kept; past it the oldest is dropped.
pub const BUDGET: usize = 256 << 20; // Bytes the stacks may pin; past it the oldest is dropped.
pub const RECORD: usize = 256; // Bytes one record costs on top of what it pins.

/// A deep copy of geometry that keeps its guid and type, element feature guids included.
pub fn clone(obj: &Geometry) -> Geometry {
    match obj {
        Geometry::OBB(g) => Geometry::OBB(Rc::new((**g).clone())),
        Geometry::BRep(g) => Geometry::BRep(Rc::new((**g).clone())),
        Geometry::Element(g) => Geometry::Element(Rc::new((**g).clone())),
        Geometry::Line(g) => Geometry::Line(Rc::new((**g).clone())),
        Geometry::Mesh(g) => Geometry::Mesh(Rc::new((**g).clone())),
        Geometry::NurbsCurve(g) => Geometry::NurbsCurve(Rc::new((**g).clone())),
        Geometry::NurbsSurface(g) => Geometry::NurbsSurface(Rc::new((**g).clone())),
        Geometry::Plane(g) => Geometry::Plane(Rc::new((**g).clone())),
        Geometry::Point(g) => Geometry::Point(Rc::new((**g).clone())),
        Geometry::PointCloud(g) => Geometry::PointCloud(Rc::new((**g).clone())),
        Geometry::Polyline(g) => Geometry::Polyline(Rc::new((**g).clone())),
    }
}

/// Bytes a mesh pins, from its counts.
fn mesh_weight(mesh: &Mesh) -> usize {
    128 + 64 * mesh.number_of_vertices() + 48 * mesh.number_of_faces()
}

/// Bytes a brep pins, from its table lengths.
fn brep_weight(brep: &BRep) -> usize {
    512 + 256 * brep.m_surfaces.len()
        + 128 * (brep.m_curves_3d.len() + brep.m_curves_2d.len())
        + 24 * brep.m_vertices.len()
        + 64 * (brep.m_edges.len() + brep.m_faces.len())
}

/// An estimate of the bytes an item pins while a record holds it, O(1) from its container lengths.
pub fn weight(item: &Item) -> usize {
    match item {
        Item::Geometry(Geometry::Point(_)) => 64,
        Item::Geometry(Geometry::Line(_)) => 96,
        Item::Geometry(Geometry::Plane(_)) => 160,
        Item::Geometry(Geometry::OBB(_)) => 192,
        Item::Geometry(Geometry::Polyline(g)) => 64 + 24 * g.point_count(),
        Item::Geometry(Geometry::PointCloud(g)) => {
            64 + 24 * g.point_count() + 24 * g.normal_count() + 16 * g.color_count()
        }
        Item::Geometry(Geometry::Mesh(g)) => mesh_weight(g),
        Item::Geometry(Geometry::NurbsCurve(g)) => 96 + 32 * g.cv_count() + 8 * g.m_nurbsknot.len(),
        Item::Geometry(Geometry::NurbsSurface(g)) => {
            128 + 32 * g.cv_count_total() + 8 * (g.m_nurbsknot[0].len() + g.m_nurbsknot[1].len())
        }
        Item::Geometry(Geometry::BRep(g)) => brep_weight(g),
        Item::Geometry(Geometry::Element(g)) => {
            let geometry = match g.geometry() {
                crate::element::ElementGeometry::Mesh(mesh) => mesh_weight(mesh),
                crate::element::ElementGeometry::BRep(brep) => brep_weight(brep),
                crate::element::ElementGeometry::None => 0,
            };

            256 + geometry + 128 * g.features.len()
        }
        Item::InstanceRef(i) => 256 + 128 * i.features.len(),
        Item::Component(_) => 128,
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════
/// One dead or revivable entity: where it lives and what it parked while dead; slots and nodes pin it weakly, records strongly.
#[derive(Debug)]
pub struct Tomb {
    pub collection: String, // The Objects list of its slot, "" for a node-only tomb.
    pub definition: bool,   // Whether the slot is in Session::definitions.
    pub slot: Cell<usize>,  // Its raw slot, moved by compaction.
    pub node: Option<Rc<RefCell<TreeNode>>>, // Its tree node, None for a slot-only tomb.
    pub vertex: RefCell<Option<Vertex>>, // Its graph vertex while dead.
    pub edges: RefCell<Vec<Edge>>, // Its incident edges while dead.
    pub xform: RefCell<Option<Xform>>, // Its local transform while dead.
    pub interactions: RefCell<BTreeMap<String, Vec<Box<dyn Interaction>>>>, // Its edges' interactions while dead, by edge guid.
}

impl Tomb {
    /// Construct a tomb with nothing parked.
    pub fn new(
        collection: &str,
        definition: bool,
        slot: usize,
        node: Option<Rc<RefCell<TreeNode>>>,
    ) -> Rc<Tomb> {
        Rc::new(Self {
            collection: collection.to_string(),
            definition,
            slot: Cell::new(slot),
            node,
            vertex: RefCell::new(None),
            edges: RefCell::new(Vec::new()),
            xform: RefCell::new(None),
            interactions: RefCell::new(BTreeMap::new()),
        })
    }
}

/// An object added or removed: the tomb that flips it and where its node sits.
#[derive(Debug, Clone)]
pub struct Tombstone {
    pub guid: String,                        // The object's guid.
    pub collection: String,                  // The Objects list it lives in, or "definitions".
    pub parent_guid: Option<String>,         // Name of its tree parent, None when it has no node.
    pub index: usize, // Its raw index among the parent's children at record time, a hint.
    pub node: Option<Rc<RefCell<TreeNode>>>, // Its tree node, for adds too; None when it has none.
    pub tomb: Rc<Tomb>, // The tomb undo and redo flip.
}

impl Tombstone {
    /// Construct from every field of the record.
    pub fn new(
        guid: String,
        collection: String,
        parent_guid: Option<String>,
        index: usize,
        node: Option<Rc<RefCell<TreeNode>>>,
        tomb: Rc<Tomb>,
    ) -> Self {
        Self {
            guid,
            collection,
            parent_guid,
            index,
            node,
            tomb,
        }
    }
}

/// The entry a replace was taken on: an object by its tree node at record time (None outside the tree) or a definition by the tomb pinning its slot.
#[derive(Debug, Clone)]
pub enum Entry {
    Object(Option<Rc<RefCell<TreeNode>>>),
    Definition(Rc<Tomb>),
}

/// The object or definition under `guid` was swapped: the stored pointers before and after, never copies.
#[derive(Debug, Clone)]
pub struct ReplaceOp {
    pub guid: String, // The entry's guid.
    pub before: Item, // The entry before the swap.
    pub after: Item,  // The entry after the swap.
    pub entry: Entry, // The entry the swap was taken on.
}

impl ReplaceOp {
    /// Construct from the guid, the before and after items and the entry.
    pub fn new(guid: String, before: Item, after: Item, entry: Entry) -> Self {
        Self {
            guid,
            before,
            after,
            entry,
        }
    }
}

/// The local transform under `guid` changed; None on either side means "none set".
#[derive(Debug, Clone)]
pub struct XformOp {
    pub guid: String,                        // The object's guid.
    pub before: Option<Xform>,               // Transform before the change.
    pub after: Option<Xform>,                // Transform after the change.
    pub node: Option<Rc<RefCell<TreeNode>>>, // The entry's tree node at record time; None for a group or an object outside the tree.
}

impl XformOp {
    /// Construct from the guid, the before and after transforms and the entry's node.
    pub fn new(
        guid: String,
        before: Option<Xform>,
        after: Option<Xform>,
        node: Option<Rc<RefCell<TreeNode>>>,
    ) -> Self {
        Self {
            guid,
            before,
            after,
            node,
        }
    }
}

/// A tree node added, removed, moved, renamed or recoloured: its state before and after.
#[derive(Debug, Clone)]
pub struct TreeOp {
    pub guid: String,                         // The node name at record time.
    pub node: Rc<RefCell<TreeNode>>,          // The node itself.
    pub tomb: Rc<Tomb>,                       // Node-only; pins the ghost of a move, else the node.
    pub ghost: Option<Rc<RefCell<TreeNode>>>, // The dead ghost a move left in the old slot.
    pub name_before: String,                  // Name before.
    pub name_after: String,                   // Name after.
    pub color_before: Option<Color>,          // Colour before.
    pub color_after: Option<Color>,           // Colour after.
    pub dead_before: bool,                    // Whether it was dead or absent before.
    pub dead_after: bool,                     // Whether it is dead after.
}

impl TreeOp {
    /// Construct from every field of the record.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        guid: String,
        node: Rc<RefCell<TreeNode>>,
        tomb: Rc<Tomb>,
        ghost: Option<Rc<RefCell<TreeNode>>>,
        name_before: String,
        name_after: String,
        color_before: Option<Color>,
        color_after: Option<Color>,
        dead_before: bool,
        dead_after: bool,
    ) -> Self {
        Self {
            guid,
            node,
            tomb,
            ghost,
            name_before,
            name_after,
            color_before,
            color_after,
            dead_before,
            dead_after,
        }
    }
}

/// Any one recorded op.
#[derive(Debug, Clone)]
pub enum Op {
    Add(Tombstone),
    Remove(Tombstone),
    Replace(ReplaceOp),
    Xform(XformOp),
    Tree(TreeOp),
}

impl Op {
    /// Return "add", "remove", "replace", "xform" or "tree".
    pub fn kind(&self) -> &str {
        match self {
            Op::Add(_) => "add",
            Op::Remove(_) => "remove",
            Op::Replace(_) => "replace",
            Op::Xform(_) => "xform",
            Op::Tree(_) => "tree",
        }
    }

    /// Return the guid of the object or the node name the op touched.
    pub fn guid(&self) -> &str {
        match self {
            Op::Add(op) | Op::Remove(op) => &op.guid,
            Op::Replace(op) => &op.guid,
            Op::Xform(op) => &op.guid,
            Op::Tree(op) => &op.guid,
        }
    }

    /// Return a string representation of the record.
    pub fn str(&self) -> String {
        format!("{}({})", self.kind(), self.guid())
    }

    /// Return a string representation of the record for debugging.
    pub fn repr(&self) -> String {
        match self {
            Op::Add(op) | Op::Remove(op) => {
                format!("{}({}, {})", self.kind(), op.guid, op.collection)
            }
            _ => self.str(),
        }
    }
}

impl fmt::Display for Op {
    /// Write the record string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

/// One undoable step: a label, the ops it made in the order they happened, and the bytes they pin.
#[derive(Debug, Clone)]
pub struct Transaction {
    pub label: String, // What the step did.
    pub ops: Vec<Op>,  // Ops in the order they happened.
    pub bytes: usize,  // Bytes its records pin.
}

impl Transaction {
    /// Construct an empty transaction with a label.
    pub fn new(label: &str) -> Self {
        Self {
            label: label.to_string(),
            ops: Vec::new(),
            bytes: 0,
        }
    }

    /// Return a string representation of the transaction.
    pub fn str(&self) -> String {
        format!("Transaction({}, {} ops)", self.label, self.ops.len())
    }

    /// Return a string representation of the transaction for debugging.
    pub fn repr(&self) -> String {
        format!("Transaction({}, {} ops)", self.label, self.ops.len())
    }
}

impl Default for Transaction {
    /// Construct an empty transaction with the default label.
    fn default() -> Self {
        Self::new("my_transaction")
    }
}

impl fmt::Display for Transaction {
    /// Write the transaction string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════
/// CAD-style undo/redo over a Session, in memory only: records flip tombs in place, every save purges them.
#[derive(Debug, Clone)]
pub struct History {
    pub undo_stack: Vec<Transaction>, // Committed transactions, oldest first; capped at CAPACITY and budget.
    pub redo_stack: Vec<Transaction>, // Undone transactions, cleared the moment a new transaction commits.
    pub current: Option<Transaction>, // The open transaction, None between commit and the next begin.
    pub bytes: usize,                 // Bytes pinned by both stacks and the open transaction.
    pub budget: usize,                // Bytes the stacks may pin before the oldest is dropped.
    pub dropped: usize, // Ops dropped since the last purge cycle began, unrecorded kills included.
}

impl Default for History {
    /// Construct an empty history with the default budget.
    fn default() -> Self {
        Self {
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            current: None,
            bytes: 0,
            budget: BUDGET,
            dropped: 0,
        }
    }
}

impl History {
    /// Construct an empty history.
    pub fn new() -> Self {
        Self::default()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether a committed transaction can be undone.
    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    /// Return whether an undone transaction can be redone.
    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    /// Return the number of committed transactions.
    pub fn depth(&self) -> usize {
        self.undo_stack.len()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transactions
    // ═══════════════════════════════════════════════════════════════════════════
    /// Open a transaction; an already open one is committed first so no op is lost.
    pub fn begin(&mut self, label: &str) {
        self.commit();
        self.current = Some(Transaction::new(label));
    }

    /// Close the open transaction. An empty one is dropped; a real one clears redo and trims the oldest past the caps.
    pub fn commit(&mut self) {
        let Some(transaction) = self.current.take() else {
            return;
        };

        if transaction.ops.is_empty() {
            return;
        }

        self.undo_stack.push(transaction);

        for undone in self.redo_stack.drain(..) {
            self.dropped += undone.ops.len();
        }

        self.bytes = self._pinned();

        while self.undo_stack.len() > 1
            && (self.undo_stack.len() > CAPACITY || self.bytes > self.budget)
        {
            let oldest = self.undo_stack.remove(0);
            self.dropped += oldest.ops.len();
            self.bytes -= oldest.bytes;
        }
    }

    /// Append an op pinning `bytes` to the open transaction; a no-op when none is open.
    pub fn record(&mut self, op: Op, bytes: usize) {
        let Some(current) = self.current.as_mut() else {
            return;
        };

        current.ops.push(op);
        current.bytes += bytes;
        self.bytes += bytes;
    }

    /// Revert the open transaction's ops in reverse and drop it, leaving both stacks as they are; false when none is open.
    pub fn abort(&mut self, session: &mut Session) -> bool {
        let Some(transaction) = self.current.take() else {
            return false;
        };

        for i in (0..transaction.ops.len()).rev() {
            self._revert(&transaction.ops[i], session);
        }

        self.dropped += transaction.ops.len();
        self.bytes = self._pinned();

        true
    }

    /// Revert the newest transaction, ops in reverse order, and park it for redo.
    pub fn undo(&mut self, session: &mut Session) -> bool {
        self.commit();

        let Some(transaction) = self.undo_stack.pop() else {
            return false;
        };

        for i in (0..transaction.ops.len()).rev() {
            self._revert(&transaction.ops[i], session);
        }

        self.redo_stack.push(transaction);

        true
    }

    /// Re-apply the newest undone transaction, ops in their original order.
    pub fn redo(&mut self, session: &mut Session) -> bool {
        self.commit();

        let Some(transaction) = self.redo_stack.pop() else {
            return false;
        };

        for i in 0..transaction.ops.len() {
            self._apply(&transaction.ops[i], session);
        }

        self.undo_stack.push(transaction);

        true
    }

    /// Drop every transaction, open or committed; what they pinned is purgeable now.
    pub fn clear(&mut self) {
        for transaction in self.undo_stack.iter().chain(&self.redo_stack) {
            self.dropped += transaction.ops.len();
        }

        if let Some(current) = &self.current {
            self.dropped += current.ops.len();
        }

        self.undo_stack.clear();
        self.redo_stack.clear();
        self.current = None;
        self.bytes = 0;
    }

    /// Bytes pinned by both stacks.
    fn _pinned(&self) -> usize {
        self.undo_stack
            .iter()
            .chain(&self.redo_stack)
            .map(|transaction| transaction.bytes)
            .sum()
    }

    /// Undo one op against the session.
    fn _revert(&self, op: &Op, session: &mut Session) {
        match op {
            Op::Add(op) => session._kill(&op.tomb),
            Op::Remove(op) => session._revive(&op.tomb),
            Op::Replace(op) => session._swap(&op.guid, op.before.clone(), &op.entry),
            Op::Xform(op) => session._place(&op.guid, op.before.as_ref(), op.node.as_ref()),
            Op::Tree(op) => session._tree(op, true),
        }
    }

    /// Redo one op against the session.
    fn _apply(&self, op: &Op, session: &mut Session) {
        match op {
            Op::Add(op) => session._revive(&op.tomb),
            Op::Remove(op) => session._kill(&op.tomb),
            Op::Replace(op) => session._swap(&op.guid, op.after.clone(), &op.entry),
            Op::Xform(op) => session._place(&op.guid, op.after.as_ref(), op.node.as_ref()),
            Op::Tree(op) => session._tree(op, false),
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return a string representation of the history.
    pub fn str(&self) -> String {
        format!(
            "History({} undo, {} redo)",
            self.undo_stack.len(),
            self.redo_stack.len()
        )
    }

    /// Return a string representation of the history for debugging.
    pub fn repr(&self) -> String {
        format!(
            "History({} undo, {} redo)",
            self.undo_stack.len(),
            self.redo_stack.len()
        )
    }
}

impl fmt::Display for History {
    /// Write the history string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}
