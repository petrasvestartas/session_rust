use serde::{Deserialize, Serialize};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::fmt;
use std::rc::{Rc, Weak};


/// A node of a tree data structure
#[derive(Debug)]
pub struct TreeNode {
    /// Lazily generated unique identifier
    guid: std::sync::OnceLock<String>,
    /// Node identifier/name. For geometry nodes, this is the geometry's GUID
    pub name: String,
    /// Optional display color, used for layer nodes
    pub color: Option<[u8; 4]>,
    /// Owning pointers to children
    children: Vec<Rc<RefCell<TreeNode>>>,
    /// Non-owning pointer to parent
    parent: Option<Weak<RefCell<TreeNode>>>,
    /// Self weak reference for parent setup
    weak_self: Weak<RefCell<TreeNode>>,
}

impl PartialEq for TreeNode {
    fn eq(&self, other: &Self) -> bool {
        self.guid() == other.guid()
    }
}

impl Serialize for TreeNode {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.to_serde().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for TreeNode {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let serde_repr = TreeNodeSerde::deserialize(deserializer)?;
        let rc = Self::from_serde(serde_repr);
        Rc::try_unwrap(rc)
            .map_err(|_| serde::de::Error::custom("shared ref"))
            .map(|cell| cell.into_inner())
    }
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "type", rename = "TreeNode")]
pub(crate) struct TreeNodeSerde {
    pub(crate) guid: String,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<[u8; 4]>,
    pub children: Vec<TreeNodeSerde>,
}

impl TreeNode {
    /// Default / named constructor — returns Rc<RefCell<TreeNode>> for shared ownership
    pub fn new(name: &str) -> Rc<RefCell<TreeNode>> {
        let node = Rc::new(RefCell::new(TreeNode {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            color: None,
            children: Vec::new(),
            parent: None,
            weak_self: Weak::new(),
        }));
        node.borrow_mut().weak_self = Rc::downgrade(&node);
        node
    }

    /// Lazy GUID accessor
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the GUID explicitly
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Optional display color
    pub fn color(&self) -> Option<[u8; 4]> {
        self.color
    }

    /// Set the display color from RGBA components
    pub fn set_color(&mut self, r: u8, g: u8, b: u8, a: u8) {
        self.color = Some([r, g, b, a]);
    }

    /// Add a child node to this node
    pub fn add(&mut self, child: &Rc<RefCell<TreeNode>>) {
        child.borrow_mut().parent = Some(self.weak_self.clone());
        self.children.push(Rc::clone(child));
    }

    /// Remove a child node, returning true if it was found and removed
    pub fn remove(&mut self, child: &Rc<RefCell<TreeNode>>) -> bool {
        if let Some(pos) = self.children.iter().position(|c| Rc::ptr_eq(c, child)) {
            let removed = self.children.remove(pos);
            removed.borrow_mut().parent = None;
            true
        } else {
            false
        }
    }

    /// Parent node, or None if this is the root
    pub fn parent(&self) -> Option<Rc<RefCell<TreeNode>>> {
        self.parent.as_ref()?.upgrade()
    }

    /// Direct children of this node
    pub fn children(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        self.children.iter().map(Rc::clone).collect()
    }

    /// True if this node has no parent
    pub fn is_root(&self) -> bool {
        self.parent.is_none()
    }

    /// True if this node has no children
    pub fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }

    /// All ancestors from immediate parent up to root
    pub fn ancestors(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();
        let mut current = self.parent();
        while let Some(node) = current {
            let next = node.borrow().parent();
            result.push(node);
            current = next;
        }
        result
    }

    /// All descendants of this node, depth-first
    pub fn descendants(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();
        for child in &self.children {
            result.push(Rc::clone(child));
            result.extend(child.borrow().descendants());
        }
        result
    }

    /// All nodes in the subtree rooted at this node (self + descendants)
    pub fn nodes(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = vec![self.weak_self.upgrade().unwrap()];
        for child in &self.children {
            result.extend(child.borrow().nodes());
        }
        result
    }

    /// Get the root of the tree this node belongs to
    pub fn root(&self) -> Rc<RefCell<TreeNode>> {
        if let Some(parent) = self.parent() {
            parent.borrow().root()
        } else {
            self.weak_self.upgrade().unwrap()
        }
    }

    /// Traverse from this node ("depthfirst"|"breadthfirst", "preorder"|"postorder")
    pub fn traverse(&self, strategy: &str, order: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        match strategy {
            "depthfirst" => self.depth_first_traverse(order),
            "breadthfirst" => self.breadth_first_traverse(),
            _ => panic!("Unknown traversal strategy: {}", strategy),
        }
    }

    fn depth_first_traverse(&self, order: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        match order {
            "preorder" => self.preorder_traverse(),
            "postorder" => self.postorder_traverse(),
            _ => panic!("Unknown traversal order: {}", order),
        }
    }

    fn preorder_traverse(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = vec![self.weak_self.upgrade().unwrap()];
        for child in &self.children {
            result.extend(child.borrow().preorder_traverse());
        }
        result
    }

    fn postorder_traverse(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();
        for child in &self.children {
            result.extend(child.borrow().postorder_traverse());
        }
        result.push(self.weak_self.upgrade().unwrap());
        result
    }

    fn breadth_first_traverse(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();
        let mut queue = VecDeque::new();
        queue.push_back(self.weak_self.upgrade().unwrap());
        while let Some(node) = queue.pop_front() {
            let children = node.borrow().children();
            result.push(Rc::clone(&node));
            for child in children {
                queue.push_back(child);
            }
        }
        result
    }

    /// Serialize to JSON string
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let serde_node = self.to_serde();
        crate::file_encoders::sorted_json_string(&serde_node)
    }

    /// Deserialize from a JSON file path
    pub fn jsonload(path: &str) -> Result<Rc<RefCell<TreeNode>>, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(path)?;
        let serde_node: TreeNodeSerde = serde_json::from_str(&json)?;
        Ok(Self::from_serde(serde_node))
    }

    pub(crate) fn to_serde(&self) -> TreeNodeSerde {
        TreeNodeSerde {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            color: self.color,
            children: self.children.iter().map(|c| c.borrow().to_serde()).collect(),
        }
    }

    pub(crate) fn from_serde(serde_node: TreeNodeSerde) -> Rc<RefCell<TreeNode>> {
        let node = TreeNode::new(&serde_node.name);
        node.borrow_mut().set_guid(serde_node.guid);
        node.borrow_mut().color = serde_node.color;
        for child_serde in serde_node.children {
            let child = Self::from_serde(child_serde);
            node.borrow_mut().add(&child);
        }
        node
    }
}

impl fmt::Display for TreeNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "TreeNode({}, {}, {} children)",
            self.name,
            self.guid(),
            self.children.len()
        )
    }
}

/// A hierarchical data structure with parent-child relationships.
#[derive(Debug, Clone)]
pub struct Tree {
    /// Lazily generated unique identifier
    guid: std::sync::OnceLock<String>,
    /// Tree identifier/name
    pub name: String,
    /// Root node of the tree (None for empty tree)
    root_node: Option<Rc<RefCell<TreeNode>>>,
}

impl Serialize for Tree {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let serde_tree = TreeSerde {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            root: self.root_node.as_ref().map(|r| r.borrow().to_serde()),
        };
        serde_tree.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Tree {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let serde_tree = TreeSerde::deserialize(deserializer)?;
        let mut tree = Tree {
            guid: { let lock = std::sync::OnceLock::new(); let _ = lock.set(serde_tree.guid); lock },
            name: serde_tree.name,
            root_node: None,
        };
        if let Some(root_serde) = serde_tree.root {
            tree.root_node = Some(TreeNode::from_serde(root_serde));
        }
        Ok(tree)
    }
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "type", rename = "Tree")]
struct TreeSerde {
    guid: String,
    name: String,
    root: Option<TreeNodeSerde>,
}

impl Tree {
    /// Lazy GUID accessor
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the GUID explicitly
    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Default / named constructor
    pub fn new(name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            root_node: None,
        }
    }

    /// Get the root node of the tree
    pub fn root(&self) -> Option<Rc<RefCell<TreeNode>>> {
        self.root_node.clone()
    }

    /// Add a node to the tree (parent=None adds as root)
    pub fn add(&mut self, node: &Rc<RefCell<TreeNode>>, parent: Option<&Rc<RefCell<TreeNode>>>) {
        if parent.is_none() {
            if self.root_node.is_some() {
                panic!("Tree already has a root node");
            }
            self.root_node = Some(Rc::clone(node));
        } else if let Some(parent_node) = parent {
            parent_node.borrow_mut().add(node);
        }
    }

    /// All nodes in the tree
    pub fn nodes(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        if let Some(root) = &self.root_node {
            root.borrow().nodes()
        } else {
            vec![]
        }
    }

    /// Remove a node from the tree
    pub fn remove(&mut self, node: &Rc<RefCell<TreeNode>>) -> bool {
        if let Some(root) = &self.root_node {
            if Rc::ptr_eq(root, node) {
                self.root_node = None;
                true
            } else {
                let parent = node.borrow().parent();
                if let Some(parent) = parent {
                    parent.borrow_mut().remove(node)
                } else {
                    false
                }
            }
        } else {
            false
        }
    }

    /// All leaf nodes
    pub fn leaves(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        self.nodes().into_iter().filter(|n| n.borrow().is_leaf()).collect()
    }

    /// Traverse from root ("depthfirst"|"breadthfirst", "preorder"|"postorder")
    pub fn traverse(&self, strategy: &str, order: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        if let Some(root) = &self.root_node {
            root.borrow().traverse(strategy, order)
        } else {
            vec![]
        }
    }

    /// First node with the given name
    pub fn get_node_by_name(&self, node_name: &str) -> Option<Rc<RefCell<TreeNode>>> {
        self.nodes().into_iter().find(|n| n.borrow().name == node_name)
    }

    /// All nodes with the given name
    pub fn get_nodes_by_name(&self, node_name: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        self.nodes()
            .into_iter()
            .filter(|n| n.borrow().name == node_name)
            .collect()
    }

    /// Find a node by its GUID
    pub fn find_node_by_guid(&self, node_guid: &str) -> Option<Rc<RefCell<TreeNode>>> {
        self.nodes().into_iter().find(|n| n.borrow().guid() == node_guid)
    }

    /// Reparent a child by GUID; returns true if both nodes were found
    pub fn add_child_by_guid(&mut self, parent_guid: &str, child_guid: &str) -> bool {
        let parent_node = self.find_node_by_guid(parent_guid);
        let child_node = self.find_node_by_guid(child_guid);
        if let (Some(parent), Some(child)) = (parent_node, child_node) {
            let current_parent = child.borrow().parent();
            if let Some(cp) = current_parent {
                cp.borrow_mut().remove(&child);
            } else {
                // Child is not currently in any parent, we can't move it
                return false;
            }
            parent.borrow_mut().add(&child);
            true
        } else {
            false
        }
    }

    /// GUIDs of children of a node by GUID
    pub fn get_children_guids(&self, node_guid: &str) -> Vec<String> {
        if let Some(node) = self.find_node_by_guid(node_guid) {
            node.borrow().children().iter().map(|c| c.borrow().guid().to_string()).collect()
        } else {
            vec![]
        }
    }

    /// Convenience overload taking a &str
    pub fn get_children(&self, node_guid: &str) -> Vec<String> {
        self.get_children_guids(node_guid)
    }

    /// Print the hierarchy to stdout
    pub fn print_hierarchy(&self) {
        if let Some(root) = &self.root_node {
            Self::print_node(root, 0);
        }
    }

    fn print_node(node: &Rc<RefCell<TreeNode>>, level: usize) {
        let indent = "  ".repeat(level);
        println!("{}├── {} ({})", indent, node.borrow().name, node.borrow().guid());
        for child in node.borrow().children() {
            Self::print_node(&child, level + 1);
        }
    }

    /// Serialize to JSON string
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let serde_tree = TreeSerde {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            root: self.root_node.as_ref().map(|r| r.borrow().to_serde()),
        };
        crate::file_encoders::sorted_json_string(&serde_tree)
    }

    /// Deserialize from JSON string
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let serde_tree: TreeSerde = serde_json::from_str(json_data)?;
        let mut tree = Tree::new(&serde_tree.name);
        tree.set_guid(serde_tree.guid);
        if let Some(root_serde) = serde_tree.root {
            let root = TreeNode::from_serde(root_serde);
            tree.root_node = Some(root);
        }
        Ok(tree)
    }

    /// Convert to JSON string (infallible fallback)
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Load from JSON string (infallible fallback)
    pub fn file_json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::default())
    }

    /// Write JSON to file
    pub fn file_json_dump(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(path, json)?;
        Ok(())
    }

    /// Read JSON from file
    pub fn file_json_load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(path)?;
        Self::jsonload(&json)
    }

    /// Convert to protobuf binary bytes
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        fn node_to_proto(node: &Rc<RefCell<TreeNode>>) -> crate::proto::TreeNode {
            let b = node.borrow();
            let children: Vec<crate::proto::TreeNode> =
                b.children().iter().map(|c| node_to_proto(c)).collect();
            crate::proto::TreeNode {
                guid: b.guid().to_string(),
                name: b.name.clone(),
                parent_guid: String::new(),
                color: b.color.map(|c| crate::proto::Color {
                    guid: String::new(),
                    name: String::new(),
                    r: c[0] as f32 / 255.0,
                    g: c[1] as f32 / 255.0,
                    b: c[2] as f32 / 255.0,
                    a: c[3] as f32 / 255.0,
                }),
                children,
            }
        }
        let root_proto = self.root_node.as_ref().map(|r| node_to_proto(r));
        let proto = crate::proto::Tree {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            root: root_proto,
        };
        proto.encode_to_vec()
    }

    /// Load from protobuf binary bytes
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Tree::decode(data)?;
        let mut tree = Tree::new(&proto.name);
        tree.set_guid(proto.guid.clone());
        fn proto_to_serde(
            proto_node: &crate::proto::TreeNode,
        ) -> TreeNodeSerde {
            TreeNodeSerde {
                guid: proto_node.guid.clone(),
                name: proto_node.name.clone(),
                color: proto_node.color.as_ref()
                    .filter(|c| c.a > 0.0)
                    .map(|c| [(c.r * 255.0).round() as u8, (c.g * 255.0).round() as u8, (c.b * 255.0).round() as u8, (c.a * 255.0).round() as u8]),
                children: proto_node
                    .children
                    .iter()
                    .map(|c| proto_to_serde(c))
                    .collect(),
            }
        }
        if let Some(root_proto) = &proto.root {
            let serde = proto_to_serde(root_proto);
            tree.root_node = Some(TreeNode::from_serde(serde));
        }
        Ok(tree)
    }

    /// Write protobuf to file
    pub fn pb_dump(&self, path: &str) {
        std::fs::write(path, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file
    pub fn pb_load(path: &str) -> Self {
        let data = std::fs::read(path).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl fmt::Display for Tree {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Tree: {}", self.name)
    }
}

impl Default for Tree {
    fn default() -> Self {
        Self::new("my_tree")
    }
}

#[cfg(test)]
#[path = "tree_test.rs"]
mod tree_test;
