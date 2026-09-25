use crate::graph::Edge;
use crate::graph::Vertex;
use crate::interaction::Interaction;
use crate::session::Geometry;
use crate::session::Item;
use crate::session::Session;
use crate::tree::TreeNode;
use crate::xform::Xform;
use std::cell::Cell;
use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fmt;
use std::rc::Rc;

pub const CAPACITY: usize = 64; // Committed transactions kept; past it the oldest is dropped.

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

/// A deep copy that keeps the guid, which `duplicate()` and most copy constructors would mint anew.
pub fn clone_item(obj: &Item) -> Item {
    match obj {
        Item::Geometry(g) => Item::Geometry(clone(g)),
        Item::Component(c) => Item::Component(c.clone()),
        Item::InstanceRef(i) => Item::InstanceRef(Rc::new((**i).clone())),
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

/// Everything needed to put one object back into every live table of a session.
#[derive(Debug, Clone)]
pub struct Tombstone {
    pub guid: String,         // The object's guid; the clone carries the same one.
    pub obj: Item,            // A clone_item() of the object, never the live instance.
    pub collection: String,   // The Objects list it lives in: "points", "lines", ... "instances".
    pub obj_index: i64, // Its position in that list, so the order() sequence survives a round trip.
    pub xform: Option<Xform>, // Its local transform, None when none was set.
    pub parent_guid: Option<String>, // Name of its tree parent, None when it was added without one.
    pub index: usize,   // Its position among the parent's children.
    pub node: Option<Rc<RefCell<TreeNode>>>, // The detached tree node with its whole subtree, None for an add.
    pub attribute: String,                   // Its graph node attribute.
    pub edges: Vec<(String, String, bool, String)>, // Incident edges as (other guid, attribute, forward, edge guid or "").
    pub interactions: BTreeMap<String, Vec<Box<dyn Interaction>>>, // Those edges' interactions by edge guid.
}

impl Tombstone {
    /// Construct from every field of the kit.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        guid: String,
        obj: Item,
        collection: String,
        obj_index: i64,
        xform: Option<Xform>,
        parent_guid: Option<String>,
        index: usize,
        node: Option<Rc<RefCell<TreeNode>>>,
        attribute: String,
        edges: Vec<(String, String, bool, String)>,
    ) -> Self {
        Self {
            guid,
            obj,
            collection,
            obj_index,
            xform,
            parent_guid,
            index,
            node,
            attribute,
            edges,
            interactions: BTreeMap::new(),
        }
    }
}

/// The object under `guid` was swapped: absolute before/after snapshots, never deltas.
#[derive(Debug, Clone)]
pub struct ReplaceOp {
    pub guid: String, // The object's guid.
    pub before: Item, // Snapshot before the swap.
    pub after: Item,  // Snapshot after the swap.
}

impl ReplaceOp {
    /// Construct from the guid and the before and after snapshots.
    pub fn new(guid: String, before: Item, after: Item) -> Self {
        Self {
            guid,
            before,
            after,
        }
    }
}

/// The local transform under `guid` changed; None on either side means "none set".
#[derive(Debug, Clone)]
pub struct XformOp {
    pub guid: String,          // The object's guid.
    pub before: Option<Xform>, // Transform before the change.
    pub after: Option<Xform>,  // Transform after the change.
}

impl XformOp {
    /// Construct from the guid and the before and after transforms.
    pub fn new(guid: String, before: Option<Xform>, after: Option<Xform>) -> Self {
        Self {
            guid,
            before,
            after,
        }
    }
}

/// A definition added (None before), removed (None after) or replaced.
#[derive(Debug, Clone)]
pub struct DefinitionOp {
    pub guid: String,             // The definition's guid.
    pub before: Option<Geometry>, // Snapshot before, None when it was added.
    pub after: Option<Geometry>,  // Snapshot after, None when it was removed.
}

impl DefinitionOp {
    /// Construct from the guid and the before and after snapshots.
    pub fn new(guid: String, before: Option<Geometry>, after: Option<Geometry>) -> Self {
        Self {
            guid,
            before,
            after,
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
    Definition(DefinitionOp),
}

impl Op {
    /// Return "add", "remove", "replace", "xform" or "definition".
    pub fn kind(&self) -> &str {
        match self {
            Op::Add(_) => "add",
            Op::Remove(_) => "remove",
            Op::Replace(_) => "replace",
            Op::Xform(_) => "xform",
            Op::Definition(_) => "definition",
        }
    }

    /// Return the guid of the object the op touched.
    pub fn guid(&self) -> &str {
        match self {
            Op::Add(op) | Op::Remove(op) => &op.guid,
            Op::Replace(op) => &op.guid,
            Op::Xform(op) => &op.guid,
            Op::Definition(op) => &op.guid,
        }
    }

    /// Return a string representation of the record.
    pub fn str(&self) -> String {
        format!("{}({})", self.kind(), self.guid())
    }

    /// Return a string representation of the record for debugging.
    pub fn repr(&self) -> String {
        match self {
            Op::Add(op) | Op::Remove(op) => format!(
                "{}({}, {}[{}])",
                self.kind(),
                op.guid,
                op.collection,
                op.obj_index
            ),
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

/// One undoable step: a label and the ops it made, in the order they happened.
#[derive(Debug, Clone)]
pub struct Transaction {
    pub label: String, // What the step did.
    pub ops: Vec<Op>,  // Ops in the order they happened.
}

impl Transaction {
    /// Construct an empty transaction with a label.
    pub fn new(label: &str) -> Self {
        Self {
            label: label.to_string(),
            ops: Vec::new(),
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
/// CAD-style undo/redo over a Session, in memory only: records exist between `begin` and `commit`, every save purges them.
#[derive(Debug, Clone, Default)]
pub struct History {
    pub undo_stack: Vec<Transaction>, // Committed transactions, oldest first; capped at CAPACITY.
    pub redo_stack: Vec<Transaction>, // Undone transactions, cleared the moment a new transaction commits.
    pub current: Option<Transaction>, // The open transaction, None between commit and the next begin.
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

    /// Close the open transaction. An empty one is dropped; a real one clears redo.
    pub fn commit(&mut self) {
        let Some(transaction) = self.current.take() else {
            return;
        };

        if transaction.ops.is_empty() {
            return;
        }

        self.undo_stack.push(transaction);
        self.redo_stack.clear();

        if self.undo_stack.len() > CAPACITY {
            self.undo_stack.remove(0);
        }
    }

    /// Append an op to the open transaction; a no-op when none is open.
    pub fn record(&mut self, op: Op) {
        let Some(current) = self.current.as_mut() else {
            return;
        };

        current.ops.push(op);
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

    /// Drop every transaction, open or committed.
    pub fn clear(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
        self.current = None;
    }

    /// Undo one op against the session.
    fn _revert(&self, op: &Op, session: &mut Session) {
        match op {
            Op::Add(op) => {
                session._detach(&op.guid);
            }
            Op::Remove(op) => session._attach(op),
            Op::Replace(op) => session._swap(&op.guid, clone_item(&op.before)),
            Op::Xform(op) => session._place(&op.guid, op.before.as_ref()),
            Op::Definition(op) => session._define(&op.guid, op.before.as_ref().map(clone)),
        }
    }

    /// Redo one op against the session.
    fn _apply(&self, op: &Op, session: &mut Session) {
        match op {
            Op::Add(op) => session._attach(op),
            Op::Remove(op) => {
                session._detach(&op.guid);
            }
            Op::Replace(op) => session._swap(&op.guid, clone_item(&op.after)),
            Op::Xform(op) => session._place(&op.guid, op.after.as_ref()),
            Op::Definition(op) => session._define(&op.guid, op.after.as_ref().map(clone)),
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
