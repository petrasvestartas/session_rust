use serde::{Deserialize, Serialize};
use std::cell::RefCell;
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
        let child_guid = child.borrow().guid().to_string();
        if let Some(pos) = self.children.iter().position(|c| c.borrow().guid() == child_guid) {
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
            _ => vec![],
        }
    }

    fn depth_first_traverse(&self, order: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        match order {
            "preorder" => self.preorder_traverse(),
            "postorder" => self.postorder_traverse(),
            _ => vec![],
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
        let mut queue = Vec::new();
        queue.push(self.weak_self.upgrade().unwrap());
        while let Some(node) = queue.pop() {
            let children = node.borrow().children();
            result.push(Rc::clone(&node));
            for child in children {
                queue.insert(0, child);
            }
        }
        result
    }

    /// Serialize to JSON string
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let serde_node = self.to_serde();
        crate::encoders::sorted_json_string(&serde_node)
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

#[cfg(test)]
#[path = "treenode_test.rs"]
mod treenode_test;
