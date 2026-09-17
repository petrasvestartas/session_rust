use crate::session::{Geometry, Session};
use crate::tree::TreeNode;
use crate::xform::Xform;
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

pub const CAPACITY: usize = 64; // Committed transactions kept; past it the oldest is dropped.

/// Returns a deep copy that keeps the guid: Geometry::clone only bumps the Rc, so the inner value is cloned into a fresh one.
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

/// Everything needed to put one object back into every live table of a session.
#[derive(Debug, Clone)]
pub struct Tombstone {
    pub guid: String,         // The object's guid; the clone carries the same one.
    pub obj: Geometry,        // A clone() of the object, never the live instance.
    pub collection: String,   // The Objects list it lives in: "points", "lines", ... "components".
    pub obj_index: i64, // Its position in that list, so the order() sequence survives a round trip.
    pub xform: Option<Xform>, // Its local transform, None when none was set.
    pub parent_guid: Option<String>, // Name of its tree parent, None when it was added without one.
    pub index: usize,   // Its position among the parent's children.
    pub node: Option<Rc<RefCell<TreeNode>>>, // The detached tree node with its whole subtree, None for an add.
    pub attribute: String,                   // Its graph node attribute.
    pub edges: Vec<(String, String, bool)>,  // Incident edges as (guid, attribute, forward).
}

impl Tombstone {
    /// Constructs from every field of the kit.
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
    pub guid: String,     // The object guid.
    pub before: Geometry, // Snapshot before the change.
    pub after: Geometry,  // Snapshot after the change.
}

impl ReplaceOp {
    /// Constructs from the guid and the before and after snapshots.
    pub fn new(guid: String, before: Geometry, after: Geometry) -> Self {
        Self {
            guid,
            before,
            after,
        }
    }
}

/// The local transform under `guid` changed; nullopt on either side means "none set".
#[derive(Debug, Clone)]
pub struct XformOp {
    pub guid: String,          // The object guid.
    pub before: Option<Xform>, // Snapshot before the change.
    pub after: Option<Xform>,  // Snapshot after the change.
}

impl XformOp {
    /// Constructs from the guid and the before and after transforms.
    pub fn new(guid: String, before: Option<Xform>, after: Option<Xform>) -> Self {
        Self {
            guid,
            before,
            after,
        }
    }
}

/// One recorded op; `Add` and `Remove` share the tombstone, undone by detaching or attaching the kit.
#[derive(Debug, Clone)]
pub enum Op {
    Add(Tombstone),
    Remove(Tombstone),
    Replace(ReplaceOp),
    Xform(XformOp),
}

impl Op {
    /// Returns "add", "remove", "replace" or "xform".
    pub fn kind(&self) -> &str {
        match self {
            Op::Add(_) => "add",
            Op::Remove(_) => "remove",
            Op::Replace(_) => "replace",
            Op::Xform(_) => "xform",
        }
    }

    /// Returns the guid of the object the op touched.
    pub fn guid(&self) -> &str {
        match self {
            Op::Add(op) | Op::Remove(op) => &op.guid,
            Op::Replace(op) => &op.guid,
            Op::Xform(op) => &op.guid,
        }
    }
}

impl fmt::Display for Op {
    /// Writes the op string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}({})", self.kind(), self.guid())
    }
}

/// One undoable step: a label and the ops it made, in the order they happened.
#[derive(Debug, Clone)]
pub struct Transaction {
    pub label: String, // What the step did.
    pub ops: Vec<Op>,  // Ops in the order they happened.
}

impl Transaction {
    /// Constructs an empty transaction with a label.
    pub fn new(label: &str) -> Self {
        Self {
            label: label.to_string(),
            ops: Vec::new(),
        }
    }
}

impl Default for Transaction {
    /// Constructs an empty transaction with the default label.
    fn default() -> Self {
        Self::new("my_transaction")
    }
}

impl fmt::Display for Transaction {
    /// Writes the transaction string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Transaction({}, {} ops)", self.label, self.ops.len())
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
    /// Constructs an empty history.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns whether a committed transaction can be undone.
    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    /// Returns whether an undone transaction can be redone.
    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    /// Returns the number of committed transactions.
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

    /// Appends an op to the open transaction; a no-op when none is open.
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

    /// Drops every transaction, open or committed.
    pub fn clear(&mut self) {
        self.undo_stack.clear();
        self.redo_stack.clear();
        self.current = None;
    }

    /// Undoes one op against the session.
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

    /// Redoes one op against the session.
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
    /// Writes the history string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "History({} undo, {} redo)",
            self.undo_stack.len(),
            self.redo_stack.len()
        )
    }
}
