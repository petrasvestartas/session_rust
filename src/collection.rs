use crate::history::Tomb;
use crate::objects::Component;
use crate::BRep;
use crate::Element;
use crate::InstanceRef;
use crate::Line;
use crate::Mesh;
use crate::NurbsCurve;
use crate::NurbsSurface;
use crate::Plane;
use crate::Point;
use crate::PointCloud;
use crate::Polyline;
use crate::OBB;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use serde::Serializer;
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt;
use std::ops::Index;
use std::rc::Rc;
use std::rc::Weak;

/// The guid a Collection indexes an entry by.
pub trait Keyed {
    /// Return the guid of the entry.
    fn key(&self) -> &str;
}

macro_rules! impl_keyed {
    ($($type:ty),* $(,)?) => {
        $(impl Keyed for Rc<$type> {
            /// Return the guid.
            fn key(&self) -> &str {
                self.guid()
            }
        })*
    };
}

impl_keyed!(
    Point,
    Line,
    Plane,
    OBB,
    Polyline,
    PointCloud,
    Mesh,
    NurbsCurve,
    NurbsSurface,
    BRep,
    Element,
    InstanceRef,
);

impl Keyed for Component {
    /// Return the guid.
    fn key(&self) -> &str {
        &self.guid
    }
}

/// A list of objects that can hold dead slots: every public view skips them, in slot order.
pub struct Collection<E> {
    items: Vec<E>,                 // Raw slots in canonical order, dead ones included.
    dead: Vec<bool>,               // One flag per slot.
    slots: HashMap<String, usize>, // Live guid -> slot.
    tombs: HashMap<usize, Vec<Weak<Tomb>>>, // Weak pins per slot, newest last.
    live: usize,                   // Live count.
    count: usize,                  // Dead slots not yet purged.
    low: usize,                    // Lowest dead slot, where compaction starts.
    cursor: Option<(usize, usize)>, // (read, write) while a compaction is part way.
    positions: RefCell<Option<Vec<usize>>>, // Live slot positions, built lazily.
}

/// Live entries of a Collection, in slot order.
pub struct Iter<'a, E> {
    items: std::slice::Iter<'a, E>,   // Raw slots.
    dead: std::slice::Iter<'a, bool>, // Their flags.
    remaining: usize,                 // Live entries not yet yielded.
}

impl<'a, E> Iterator for Iter<'a, E> {
    type Item = &'a E;

    /// Return the next live entry.
    fn next(&mut self) -> Option<&'a E> {
        if self.remaining == 0 {
            return None;
        }

        loop {
            let item = self.items.next()?;

            if !*self.dead.next()? {
                self.remaining -= 1;

                return Some(item);
            }
        }
    }

    /// Return the exact number of live entries left.
    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl<E> DoubleEndedIterator for Iter<'_, E> {
    /// Return the last live entry not yet yielded.
    fn next_back(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }

        loop {
            let item = self.items.next_back()?;

            if !*self.dead.next_back()? {
                self.remaining -= 1;

                return Some(item);
            }
        }
    }
}

impl<E> ExactSizeIterator for Iter<'_, E> {}

impl<E> Default for Collection<E> {
    /// Construct an empty collection.
    fn default() -> Self {
        Self {
            items: Vec::new(),
            dead: Vec::new(),
            slots: HashMap::new(),
            tombs: HashMap::new(),
            live: 0,
            count: 0,
            low: 0,
            cursor: None,
            positions: RefCell::new(None),
        }
    }
}

impl<E> Collection<E> {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty collection.
    pub fn new() -> Self {
        Self::default()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the number of live entries.
    pub fn len(&self) -> usize {
        self.live
    }

    /// Return whether no entry is live.
    pub fn is_empty(&self) -> bool {
        self.live == 0
    }

    /// Iterate the live entries in slot order.
    pub fn iter(&self) -> Iter<'_, E> {
        Iter {
            items: self.items.iter(),
            dead: self.dead.iter(),
            remaining: self.live,
        }
    }

    /// Return the live entry at position i; O(n) once after an edit while dead slots exist, then O(1).
    pub fn get(&self, i: usize) -> Option<&E> {
        if self.count == 0 && self.cursor.is_none() {
            return self.items.get(i);
        }

        let mut positions = self.positions.borrow_mut();
        let positions = positions.get_or_insert_with(|| {
            let mut live = Vec::with_capacity(self.live);

            for (slot, dead) in self.dead.iter().enumerate() {
                if !dead {
                    live.push(slot);
                }
            }

            live
        });

        positions.get(i).map(|slot| &self.items[*slot])
    }

    /// Return the first live entry.
    pub fn first(&self) -> Option<&E> {
        self.get(0)
    }

    /// Return the last live entry.
    pub fn last(&self) -> Option<&E> {
        self.get(self.live.checked_sub(1)?)
    }

    /// Return the live entries as a Vec.
    pub fn to_vec(&self) -> Vec<E>
    where
        E: Clone,
    {
        self.iter().cloned().collect()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Kernel slots: raw access for Session, History and tests
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the slot of a live guid.
    pub fn get_slot(&self, guid: &str) -> Option<usize> {
        self.slots.get(guid).copied()
    }

    /// Return the entry in a slot, dead or alive.
    pub fn get_item(&self, slot: usize) -> &E {
        &self.items[slot]
    }

    /// Return whether a slot is dead.
    pub fn is_dead(&self, slot: usize) -> bool {
        self.dead[slot]
    }

    /// Return the newest tomb pinning a slot while a record still holds it.
    pub fn get_tomb(&self, slot: usize) -> Option<Rc<Tomb>> {
        held(self.tombs.get(&slot)).pop()
    }

    /// Pin a slot weakly to a tomb and point the tomb at the slot; older pins a record still holds stay.
    pub fn set_tomb(&mut self, slot: usize, tomb: &Rc<Tomb>) {
        tomb.slot.set(slot);
        let mut pins = Vec::new();

        for pin in held(self.tombs.get(&slot)) {
            if !Rc::ptr_eq(&pin, tomb) {
                pins.push(Rc::downgrade(&pin));
            }
        }

        pins.push(Rc::downgrade(tomb));
        self.tombs.insert(slot, pins);
    }

    /// Return the number of dead slots not yet purged.
    pub fn number_of_dead(&self) -> usize {
        self.count
    }

    /// Return the number of raw slots, dead ones included.
    pub fn number_of_slots(&self) -> usize {
        self.items.len()
    }

    /// Return whether a compaction is part way.
    pub fn is_compacting(&self) -> bool {
        self.cursor.is_some()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Make room for exactly `additional` more entries, so a bulk load never grows by doubling.
    pub fn reserve_exact(&mut self, additional: usize) {
        self.items.reserve_exact(additional);
        self.dead.reserve_exact(additional);
        self.slots.reserve(additional);
    }

    /// Drop every entry and pin.
    pub fn clear(&mut self) {
        self.items.clear();
        self.dead.clear();
        self.slots.clear();
        self.tombs.clear();
        self.live = 0;
        self.count = 0;
        self.low = 0;
        self.cursor = None;
        *self.positions.get_mut() = None;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Collection(3 live, 0 dead)".
    pub fn str(&self) -> String {
        format!("Collection({} live, {} dead)", self.live, self.count)
    }

    /// Return "Collection(3 live, 0 dead)".
    pub fn repr(&self) -> String {
        self.str()
    }
}

impl<E: Keyed> Collection<E> {
    /// Append a live entry and index its guid, O(1) amortised.
    pub fn push(&mut self, item: E) {
        let slot = self.items.len();
        self.slots.insert(item.key().to_string(), slot);
        self.items.push(item);
        self.dead.push(false);
        self.live += 1;

        if let Some(positions) = self.positions.get_mut() {
            positions.push(slot);
        }
    }

    /// Put an entry in a slot; a live slot re-indexes its guid.
    pub fn set_item(&mut self, slot: usize, item: E) {
        if !self.dead[slot] {
            let old = self.items[slot].key();

            if self.slots.get(old) == Some(&slot) {
                self.slots.remove(old);
            }

            self.slots.insert(item.key().to_string(), slot);
        }

        self.items[slot] = item;
    }

    /// Kill or revive a slot in O(1); a kill unindexes the guid only when it points at this slot.
    pub fn set_dead(&mut self, slot: usize, dead: bool) {
        if self.dead[slot] == dead {
            return;
        }

        if let Some((r, w)) = self.cursor {
            if w <= slot && slot < r {
                return;
            }
        }

        let key = self.items[slot].key();
        self.dead[slot] = dead;
        *self.positions.get_mut() = None;

        if dead {
            if self.slots.get(key) == Some(&slot) {
                self.slots.remove(key);
            }

            if self.count == 0 && self.cursor.is_none() {
                self.low = slot;
            }

            self.live -= 1;
            self.count += 1;
            self.low = self.low.min(slot);
        } else {
            self.slots.insert(key.to_string(), slot);
            self.live += 1;
            self.count -= 1;
        }
    }

    /// Purge unpinned dead slots for at most `work` slots, resuming where the last call stopped; returns the slots examined.
    pub fn compact_step(&mut self, work: usize) -> usize {
        if work == 0 || (self.cursor.is_none() && self.count == 0) {
            return 0;
        }

        if self.cursor.is_none() {
            let start = self.low.min(self.items.len());
            self.cursor = Some((start, start));
            self.low = usize::MAX;
        }

        let Some((mut r, mut w)) = self.cursor else {
            return 0;
        };
        let mut examined = 0;
        *self.positions.get_mut() = None;

        while examined < work && r < self.items.len() {
            let pins = held(self.tombs.get(&r));

            if self.dead[r] && pins.is_empty() {
                self.tombs.remove(&r);
                self.count -= 1;
            } else {
                if w != r {
                    self._move(r, w, &pins);
                }

                if self.dead[w] {
                    self.low = self.low.min(w);
                }

                w += 1;
            }

            r += 1;
            examined += 1;
        }

        if r < self.items.len() {
            self.cursor = Some((r, w));

            return examined;
        }

        self.items.truncate(w);
        self.dead.truncate(w);
        self.cursor = None;
        self.low = self.low.min(w);

        examined
    }

    /// Finish a running compaction, then purge every unpinned dead slot.
    pub fn compact(&mut self) {
        if self.cursor.is_some() {
            self.compact_step(usize::MAX);
        }

        self.compact_step(usize::MAX);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Details
    // ═══════════════════════════════════════════════════════════════════════════
    /// Move the entry of slot r down to slot w with its dead flag and the pins a record holds.
    fn _move(&mut self, r: usize, w: usize, pins: &[Rc<Tomb>]) {
        self.items.swap(w, r);
        self.dead.swap(w, r);
        self.tombs.remove(&r);

        for tomb in pins {
            tomb.slot.set(w);
        }

        if !pins.is_empty() {
            self.tombs
                .insert(w, pins.iter().map(Rc::downgrade).collect());
        }

        if !self.dead[w] {
            self.slots.insert(self.items[w].key().to_string(), w);
        }
    }
}

impl<E: Keyed> From<Vec<E>> for Collection<E> {
    /// Take every entry of a Vec as live.
    fn from(items: Vec<E>) -> Self {
        let mut slots = HashMap::with_capacity(items.len());

        for (slot, item) in items.iter().enumerate() {
            slots.insert(item.key().to_string(), slot);
        }

        Self {
            live: items.len(),
            dead: vec![false; items.len()],
            items,
            slots,
            ..Self::default()
        }
    }
}

impl<E: Keyed> FromIterator<E> for Collection<E> {
    /// Collect every entry as live.
    fn from_iter<I: IntoIterator<Item = E>>(iter: I) -> Self {
        Self::from(iter.into_iter().collect::<Vec<E>>())
    }
}

impl<E: Keyed> Extend<E> for Collection<E> {
    /// Push every entry.
    fn extend<I: IntoIterator<Item = E>>(&mut self, iter: I) {
        for item in iter {
            self.push(item);
        }
    }
}

impl<E> Index<usize> for Collection<E> {
    type Output = E;

    /// Return the live entry at position i; panics past the end like a Vec.
    fn index(&self, i: usize) -> &E {
        match self.get(i) {
            Some(item) => item,
            None => panic!(
                "Collection index {i} out of range for {} live entries",
                self.live
            ),
        }
    }
}

impl<'a, E> IntoIterator for &'a Collection<E> {
    type Item = &'a E;
    type IntoIter = Iter<'a, E>;

    /// Iterate the live entries in slot order.
    fn into_iter(self) -> Iter<'a, E> {
        self.iter()
    }
}

impl<E: Keyed + Clone> Clone for Collection<E> {
    /// Copy the live entries, compacted and without pins.
    fn clone(&self) -> Self {
        if self.count == 0 && self.cursor.is_none() {
            return Self {
                items: self.items.clone(),
                dead: self.dead.clone(),
                slots: self.slots.clone(),
                live: self.live,
                ..Self::default()
            };
        }

        self.iter().cloned().collect()
    }
}

impl<E: fmt::Debug> fmt::Debug for Collection<E> {
    /// Write the live entries as a list.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.iter()).finish()
    }
}

impl<E> fmt::Display for Collection<E> {
    /// Write the collection string to a formatter.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl<E: PartialEq> PartialEq for Collection<E> {
    /// Compare the live entries in order.
    fn eq(&self, other: &Self) -> bool {
        self.live == other.live && self.iter().eq(other.iter())
    }
}

impl<E: Serialize> Serialize for Collection<E> {
    /// Write the live entries as a plain sequence, the JSON shape of a Vec.
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_seq(self.iter())
    }
}

impl<'de, E: Deserialize<'de> + Keyed> Deserialize<'de> for Collection<E> {
    /// Read a sequence, every entry live.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Ok(Self::from(Vec::<E>::deserialize(deserializer)?))
    }
}

/// The tombs of a slot's pins that a record still holds, oldest first.
fn held(pins: Option<&Vec<Weak<Tomb>>>) -> Vec<Rc<Tomb>> {
    let mut held = Vec::new();

    for pin in pins.into_iter().flatten() {
        if let Some(tomb) = pin.upgrade() {
            held.push(tomb);
        }
    }

    held
}
