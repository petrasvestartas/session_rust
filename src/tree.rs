use crate::color::Color;
use serde::Deserialize;
use serde::Serialize;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::fmt;
use std::rc::Rc;
use std::rc::Weak;

// ═══════════════════════════════════════════════════════════════════════════
// TreeNode
// ═══════════════════════════════════════════════════════════════════════════

/// A node of a tree; geometry nodes are named by their object's guid, group nodes by a label.
#[derive(Debug)]
pub struct TreeNode {
    guid: std::sync::OnceLock<String>,       // Lazy guid.
    parent: Option<Weak<RefCell<TreeNode>>>, // Parent node, None for the root.
    children: Vec<Rc<RefCell<TreeNode>>>,    // Child nodes in order.
    weak_self: Weak<RefCell<TreeNode>>,      // Handle to this node's own cell.
    pub name: String,                        // Object guid or group label.
    pub color: Option<Color>,                // Display colour override.
}

impl TreeNode {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a node with a name.
    pub fn new(name: &str) -> Rc<RefCell<TreeNode>> {
        let node = Rc::new(RefCell::new(TreeNode {
            guid: std::sync::OnceLock::new(),
            parent: None,
            children: Vec::new(),
            weak_self: Weak::new(),
            name: name.to_string(),
            color: None,
        }));
        node.borrow_mut().weak_self = Rc::downgrade(&node);

        node
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = std::sync::OnceLock::from(guid);
    }

    /// Return whether this node has no parent.
    pub fn is_root(&self) -> bool {
        self.parent.is_none()
    }

    /// Return whether this node has no children.
    pub fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }

    /// Return the parent node, or None when this is the root.
    pub fn parent(&self) -> Option<Rc<RefCell<TreeNode>>> {
        self.parent.as_ref()?.upgrade()
    }

    /// Return all ancestors from the immediate parent up to the root.
    pub fn ancestors(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();
        let mut current = self.parent();

        while let Some(node) = current {
            current = node.borrow().parent();
            result.push(node);
        }

        result
    }

    /// Return all descendants of this node, depth-first.
    pub fn descendants(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = self.traverse("depthfirst", "preorder");
        result.remove(0);

        result
    }

    /// Return the direct children of this node.
    pub fn children(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();

        for child in &self.children {
            result.push(Rc::clone(child));
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Add a child node to this node.
    pub fn add(&mut self, child: &Rc<RefCell<TreeNode>>) {
        let Some(handle) = self.weak_self.upgrade() else {
            return;
        };

        if Rc::ptr_eq(&handle, child) {
            return;
        }

        let mut ancestor = self.parent();

        while let Some(node) = ancestor {
            if Rc::ptr_eq(&node, child) {
                return;
            }

            ancestor = node.borrow().parent();
        }

        child.borrow_mut().parent = Some(self.weak_self.clone());
        self.children.push(Rc::clone(child));
    }

    /// Remove a child node and returns it, or None when not found.
    pub fn remove(&mut self, child: &Rc<RefCell<TreeNode>>) -> Option<Rc<RefCell<TreeNode>>> {
        for i in 0..self.children.len() {
            if !Rc::ptr_eq(&self.children[i], child) {
                continue;
            }

            let removed = self.children.remove(i);
            removed.borrow_mut().parent = None;

            return Some(removed);
        }

        None
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Traversal
    // ═══════════════════════════════════════════════════════════════════════════
    /// Traverse from this node ("depthfirst"|"breadthfirst", "preorder"|"postorder").
    pub fn traverse(&self, strategy: &str, order: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();

        if strategy == "depthfirst" {
            if order != "preorder" && order != "postorder" {
                panic!("Unknown traversal order: {}", order);
            }

            let mut stack = vec![self.weak_self.upgrade().unwrap()];

            while let Some(current) = stack.pop() {
                result.push(Rc::clone(&current));
                let children = current.borrow().children();

                if order == "preorder" {
                    for i in (0..children.len()).rev() {
                        stack.push(Rc::clone(&children[i]));
                    }
                } else {
                    stack.extend(children);
                }
            }

            if order == "postorder" {
                result.reverse();
            }
        } else if strategy == "breadthfirst" {
            let mut queue = VecDeque::new();
            queue.push_back(self.weak_self.upgrade().unwrap());

            while let Some(current) = queue.pop_front() {
                result.push(Rc::clone(&current));

                for child in &current.borrow().children {
                    queue.push_back(Rc::clone(child));
                }
            }
        } else {
            panic!("Unknown traversal strategy: {}", strategy);
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(&node_to_serde(self))
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Rc<RefCell<TreeNode>>, Box<dyn std::error::Error>> {
        let data: TreeNodeSerde = serde_json::from_str(json_data)?;

        Ok(serde_to_node(data))
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the name and child count.
    pub fn str(&self) -> String {
        format!("TreeNode({}, {} children)", self.name, self.children.len())
    }

    /// Return the name, guid and child count.
    pub fn repr(&self) -> String {
        format!(
            "TreeNode({}, {}, {} children)",
            self.name,
            self.guid(),
            self.children.len()
        )
    }
}

impl PartialEq for TreeNode {
    fn eq(&self, other: &Self) -> bool {
        self.guid() == other.guid()
    }
}

impl fmt::Display for TreeNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tree
// ═══════════════════════════════════════════════════════════════════════════

/// A hierarchy of TreeNodes under one root.
#[derive(Debug)]
pub struct Tree {
    guid: std::sync::OnceLock<String>,   // Lazy guid.
    root: Option<Rc<RefCell<TreeNode>>>, // Root node, None when empty.
    pub name: String,                    // Tree name.
}

impl Tree {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty tree with a name.
    pub fn new(name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            root: None,
            name: name.to_string(),
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = std::sync::OnceLock::from(guid);
    }

    /// Return the root node, or None when empty.
    pub fn root(&self) -> Option<Rc<RefCell<TreeNode>>> {
        self.root.clone()
    }

    /// Return all nodes in the tree, breadth-first from the root.
    pub fn nodes(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();
        let Some(root) = &self.root else {
            return result;
        };
        let mut queue = VecDeque::new();
        queue.push_back(Rc::clone(root));

        while let Some(current) = queue.pop_front() {
            result.push(Rc::clone(&current));

            for child in &current.borrow().children {
                queue.push_back(Rc::clone(child));
            }
        }

        result
    }

    /// Return all nodes without children.
    pub fn leaves(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();

        for node in self.nodes() {
            if node.borrow().is_leaf() {
                result.push(node);
            }
        }

        result
    }

    /// Return the first node with the given name, or None when not found.
    pub fn get_node_by_name(&self, node_name: &str) -> Option<Rc<RefCell<TreeNode>>> {
        self.nodes()
            .into_iter()
            .find(|node| node.borrow().name == node_name)
    }

    /// Return all nodes with the given name.
    pub fn get_nodes_by_name(&self, node_name: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();

        for node in self.nodes() {
            if node.borrow().name == node_name {
                result.push(node);
            }
        }

        result
    }

    /// Return the node with the given guid, or None when not found.
    pub fn find_node_by_guid(&self, node_guid: &str) -> Option<Rc<RefCell<TreeNode>>> {
        self.nodes()
            .into_iter()
            .find(|node| node.borrow().guid() == node_guid)
    }

    /// Return the guids of the children of a node by guid, empty when not found.
    pub fn get_children_guids(&self, node_guid: &str) -> Vec<String> {
        let mut result = Vec::new();
        let Some(node) = self.find_node_by_guid(node_guid) else {
            return result;
        };

        for child in &node.borrow().children {
            result.push(child.borrow().guid().to_string());
        }

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Add a node to the tree; a None parent adds it as the root.
    pub fn add(&mut self, node: &Rc<RefCell<TreeNode>>, parent: Option<&Rc<RefCell<TreeNode>>>) {
        if let Some(parent) = parent {
            parent.borrow_mut().add(node);

            return;
        }

        if self.root.is_some() {
            panic!("Tree already has a root node");
        }

        self.root = Some(Rc::clone(node));
    }

    /// Remove a node and returns it with its subtree intact, or None when not in the tree.
    pub fn remove(&mut self, node: &Rc<RefCell<TreeNode>>) -> Option<Rc<RefCell<TreeNode>>> {
        if let Some(root) = &self.root {
            if Rc::ptr_eq(root, node) {
                self.root = None;

                return Some(Rc::clone(node));
            }
        }

        let parent = node.borrow().parent()?;
        let removed = parent.borrow_mut().remove(node);

        removed
    }

    /// Reparent a child by guid; false when either node is missing or the child is the root.
    pub fn add_child_by_guid(&mut self, parent_guid: &str, child_guid: &str) -> bool {
        let Some(parent) = self.find_node_by_guid(parent_guid) else {
            return false;
        };
        let Some(child) = self.find_node_by_guid(child_guid) else {
            return false;
        };

        if Rc::ptr_eq(&parent, &child) {
            return false;
        }

        let mut ancestor = Some(Rc::clone(&parent));

        while let Some(node) = ancestor {
            if Rc::ptr_eq(&node, &child) {
                return false;
            }

            ancestor = node.borrow().parent();
        }

        let Some(current) = child.borrow().parent() else {
            return false;
        };
        current.borrow_mut().remove(&child);
        parent.borrow_mut().add(&child);

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Traversal
    // ═══════════════════════════════════════════════════════════════════════════
    /// Traverse from the root ("depthfirst"|"breadthfirst", "preorder"|"postorder").
    pub fn traverse(&self, strategy: &str, order: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        let Some(root) = &self.root else {
            return Vec::new();
        };
        let traversed = root.borrow().traverse(strategy, order);

        traversed
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.jsondump()?)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::jsonload(&std::fs::read_to_string(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let mut proto = crate::proto::Tree::default();

        if self.has_guid() {
            proto.guid = self.guid().to_string();
        }

        proto.name = self.name.clone();

        if let Some(root) = &self.root {
            proto.root = Some(node_to_proto(&root.borrow()));
        }

        proto.encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Tree::decode(data)?;
        let mut tree = Tree::new(&proto.name);

        if !proto.guid.is_empty() {
            tree.set_guid(proto.guid.clone());
        }

        if let Some(root) = &proto.root {
            tree.add(&proto_to_node(root), None);
        }

        Ok(tree)
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.pb_dumps())?;

        Ok(())
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the node count and the hierarchy drawn with box-drawing connectors.
    pub fn str(&self) -> String {
        let mut text = format!("<Tree with {} nodes: {}>\n", self.nodes().len(), self.name);

        if let Some(start) = self.root() {
            draw_node(&mut text, &start, "", true);
        }

        text
    }

    /// Return the tree name and node count.
    pub fn repr(&self) -> String {
        format!("Tree({}, {} nodes)", self.name, self.nodes().len())
    }
}

/// Draw one node and its subtree, the last child of every level closing its branch.
fn draw_node(text: &mut String, node: &Rc<RefCell<TreeNode>>, prefix: &str, last: bool) {
    text.push_str(prefix);
    text.push_str(if last {
        "\u{2514}\u{2500}\u{2500} "
    } else {
        "\u{251c}\u{2500}\u{2500} "
    });
    text.push_str(&node.borrow().str());
    text.push('\n');

    let kids = node.borrow().children();
    let next = format!("{}{}", prefix, if last { "    " } else { "\u{2502}   " });
    for (i, child) in kids.iter().enumerate() {
        draw_node(text, child, &next, i + 1 == kids.len());
    }
}

/// A clone duplicates the node hierarchy; a derived clone would share the root between two trees.
impl Clone for Tree {
    fn clone(&self) -> Self {
        let mut tree = Tree::new(&self.name);

        if self.has_guid() {
            tree.set_guid(self.guid().to_string());
        }

        if let Some(root) = &self.root {
            tree.root = Some(clone_node(&root.borrow()));
        }

        tree
    }
}

impl Default for Tree {
    fn default() -> Self {
        Self::new("my_tree")
    }
}

impl fmt::Display for Tree {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Node helpers
// ═══════════════════════════════════════════════════════════════════════════

/// Duplicate one node and everything under it with the same names, guids and colours.
fn clone_node(node: &TreeNode) -> Rc<RefCell<TreeNode>> {
    let copy = TreeNode::new(&node.name);

    if node.has_guid() {
        copy.borrow_mut().set_guid(node.guid().to_string());
    }

    copy.borrow_mut().color = node.color.clone();

    for child in &node.children {
        copy.borrow_mut().add(&clone_node(&child.borrow()));
    }

    copy
}

/// Convert a node and its subtree to protobuf.
fn node_to_proto(node: &TreeNode) -> crate::proto::TreeNode {
    let mut proto = crate::proto::TreeNode {
        guid: node.guid().to_string(),
        name: node.name.clone(),
        parent_guid: String::new(),
        color: None,
        children: Vec::new(),
    };

    if let Some(color) = &node.color {
        proto.color = Some(crate::proto::Color {
            r: color.r,
            g: color.g,
            b: color.b,
            a: color.a,
            ..Default::default()
        });
    }

    for child in &node.children {
        proto.children.push(node_to_proto(&child.borrow()));
    }

    proto
}

/// Convert a protobuf node and its subtree to a TreeNode.
fn proto_to_node(proto: &crate::proto::TreeNode) -> Rc<RefCell<TreeNode>> {
    let node = TreeNode::new(&proto.name);
    node.borrow_mut().set_guid(proto.guid.clone());

    if let Some(color) = &proto.color {
        if color.a > 0.0 {
            node.borrow_mut().color = Some(Color::new(color.r, color.g, color.b, color.a));
        }
    }

    for child in &proto.children {
        node.borrow_mut().add(&proto_to_node(child));
    }

    node
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════

#[derive(Serialize, Deserialize)]
#[serde(tag = "type", rename = "TreeNode")]
struct TreeNodeSerde {
    guid: String,
    name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    color: Option<Color>,
    children: Vec<TreeNodeSerde>,
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "type", rename = "Tree")]
struct TreeSerde {
    guid: String,
    name: String,
    root: Option<TreeNodeSerde>,
}

/// Convert a node and its subtree to its serde mirror.
fn node_to_serde(node: &TreeNode) -> TreeNodeSerde {
    let mut children = Vec::new();

    for child in &node.children {
        children.push(node_to_serde(&child.borrow()));
    }

    TreeNodeSerde {
        guid: node.guid().to_string(),
        name: node.name.clone(),
        color: node.color.clone(),
        children,
    }
}

/// Convert a serde mirror and its subtree to a TreeNode.
fn serde_to_node(data: TreeNodeSerde) -> Rc<RefCell<TreeNode>> {
    let node = TreeNode::new(&data.name);
    node.borrow_mut().set_guid(data.guid);
    node.borrow_mut().color = data.color;

    for child in data.children {
        node.borrow_mut().add(&serde_to_node(child));
    }

    node
}

impl Serialize for Tree {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut root = None;

        if let Some(node) = &self.root {
            root = Some(node_to_serde(&node.borrow()));
        }

        TreeSerde {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            root,
        }
        .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Tree {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = TreeSerde::deserialize(deserializer)?;
        let mut tree = Tree::new(&data.name);
        tree.set_guid(data.guid);

        if let Some(root) = data.root {
            tree.add(&serde_to_node(root), None);
        }

        Ok(tree)
    }
}
