use crate::session::{Geometry, Session};
use crate::tree::TreeNode;
use crate::xform::Xform;
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

pub const CAPACITY: usize = 64;

/// A deep copy that KEEPS the guid: a snapshot must still name the object it stands for.
/// `Geometry::clone` only bumps the Rc, so the inner value is cloned into a fresh Rc; every
/// geometry's `clone()` preserves the guid (`duplicate()` is what mints a new one).
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

// ═══════════════════════════════════════════════════════════════════════════
// Records
// ═══════════════════════════════════════════════════════════════════════════

/// Everything needed to put ONE object back into every live table of a session.
///
/// * `guid` - The object's guid; the clone carries the same one.
/// * `obj` - A `clone()` of the object, never the live instance.
/// * `collection` - The `Objects` vector it lives in: "points", "lines", ... "components".
/// * `obj_index` - Its position in that vector, so the `order()` sequence survives a round trip.
/// * `xform` - Its local transform, None when none was set.
/// * `parent_guid` - Name of its tree parent, None when it was added without one.
/// * `index` - Its position among the parent's children.
/// * `node` - The detached tree node with its whole subtree, None for an add.
/// * `attribute` - Its graph node attribute.
/// * `edges` - Incident graph edges as (other_guid, attribute, forward), forward when the
///   object was the edge's v0.
#[derive(Debug, Clone)]
pub struct Tombstone {
    pub guid: String,
    pub obj: Geometry,
    pub collection: String,
    pub obj_index: i64,
    pub xform: Option<Xform>,
    pub parent_guid: Option<String>,
    pub index: usize,
    pub node: Option<Rc<RefCell<TreeNode>>>,
    pub attribute: String,
    pub edges: Vec<(String, String, bool)>,
}

impl Tombstone {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        guid: String,
        obj: Geometry,
        collection: String,
        obj_index: i64,
        xform: Option<Xform>,
        parent_guid: Option<String>,
        index: usize,
        node: Option<Rc<RefCell<TreeNode>>>,
        attribute: String,
        edges: Vec<(String, String, bool)>,
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
        }
    }
}

/// The object under `guid` was swapped: absolute before/after snapshots, never deltas.
#[derive(Debug, Clone)]
pub struct ReplaceOp {
    pub guid: String,
    pub before: Geometry,
    pub after: Geometry,
}

impl ReplaceOp {
    pub fn new(guid: String, before: Geometry, after: Geometry) -> Self {
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
    pub guid: String,
    pub before: Option<Xform>,
    pub after: Option<Xform>,
}

impl XformOp {
    pub fn new(guid: String, before: Option<Xform>, after: Option<Xform>) -> Self {
        Self {
            guid,
            before,
            after,
        }
    }
}

/// One recorded op. `Add` and `Remove` share the tombstone shape: an add is undone by
/// detaching the kit, a remove by attaching it again.
#[derive(Debug, Clone)]
pub enum Op {
    Add(Tombstone),
    Remove(Tombstone),
    Replace(ReplaceOp),
    Xform(XformOp),
}

impl Op {
    pub fn kind(&self) -> &str {
        match self {
            Op::Add(_) => "add",
            Op::Remove(_) => "remove",
            Op::Replace(_) => "replace",
            Op::Xform(_) => "xform",
        }
    }

    pub fn guid(&self) -> &str {
        match self {
            Op::Add(op) | Op::Remove(op) => &op.guid,
            Op::Replace(op) => &op.guid,
            Op::Xform(op) => &op.guid,
        }
    }
}

impl fmt::Display for Op {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}({})", self.kind(), self.guid())
    }
}

/// One undoable step: a label and the ops it made, in the order they happened.
#[derive(Debug, Clone)]
pub struct Transaction {
    pub label: String,
    pub ops: Vec<Op>,
}

impl Transaction {
    pub fn new(label: &str) -> Self {
        Self {
            label: label.to_string(),
            ops: Vec::new(),
        }
    }
}

impl Default for Transaction {
    fn default() -> Self {
        Self::new("my_transaction")
    }
}

impl fmt::Display for Transaction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Transaction({}, {} ops)", self.label, self.ops.len())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// History
// ═══════════════════════════════════════════════════════════════════════════

/// CAD-style undo/redo over a Session, in memory only.
///
/// A removed object leaves every live table at once; its `Op::Remove` is the tombstone that
/// carries the resurrection kit. Records are only written while a transaction is open
/// (`begin` ... `commit`), and every save purges the buffer, as Rhino does: history never
/// crosses pb or JSON, and a loaded session starts with an empty one.
///
/// * `undo_stack` - Committed transactions, oldest first; capped at CAPACITY, the oldest dropped.
/// * `redo_stack` - Undone transactions, cleared the moment a new transaction commits.
/// * `current` - The open transaction, None between `commit` and the next `begin`.
#[derive(Debug, Clone, Default)]
pub struct History {
    pub undo_stack: Vec<Transaction>,
    pub redo_stack: Vec<Transaction>,
    pub current: Option<Transaction>,
}

impl History {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    pub fn depth(&self) -> usize {
        self.undo_stack.len()
    }

    /// Open a transaction; an already open one is committed first so no op is lost.
    pub fn begin(&mut self, label: &str) {
        self.commit();
        self.current = Some(Transaction::new(label));
    }

    /// Close the open transaction. An empty one is dropped; a real one clears redo.
    pub fn commit(&mut self) {
        let transaction = self.current.take();
        let Some(transaction) = transaction else {
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
        for op in transaction.ops.iter().rev() {
            self._revert(op, session);
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
        for op in transaction.ops.iter() {
            self._apply(op, session);
        }
        self.undo_stack.push(transaction);
        true
    }

    pub fn clear(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
        self.current = None;
    }

    fn _revert(&self, op: &Op, session: &mut Session) {
        match op {
            Op::Add(op) => {
                session._detach(&op.guid);
            }
            Op::Remove(op) => session._attach(op),
            Op::Replace(op) => session._swap(&op.guid, clone(&op.before)),
            Op::Xform(op) => session._place(&op.guid, op.before.as_ref()),
        }
    }

    fn _apply(&self, op: &Op, session: &mut Session) {
        match op {
            Op::Add(op) => session._attach(op),
            Op::Remove(op) => {
                session._detach(&op.guid);
            }
            Op::Replace(op) => session._swap(&op.guid, clone(&op.after)),
            Op::Xform(op) => session._place(&op.guid, op.after.as_ref()),
        }
    }
}

impl fmt::Display for History {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "History({} undo, {} redo)",
            self.undo_stack.len(),
            self.redo_stack.len()
        )
    }
}
