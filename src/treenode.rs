use serde::{ser::Serialize as SerTrait, Deserialize, Serialize};
use std::cell::RefCell;
use std::fmt;
use std::rc::{Rc, Weak};
use uuid::Uuid;

#[derive(Debug)]
pub struct TreeNode {
    pub guid: String,
    pub name: String,
    pub color: Option<[u8; 4]>,
    children: Vec<Rc<RefCell<TreeNode>>>,
    parent: Option<Weak<RefCell<TreeNode>>>,
    weak_self: Weak<RefCell<TreeNode>>,
}

impl PartialEq for TreeNode {
    fn eq(&self, other: &Self) -> bool {
        self.guid == other.guid
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
    pub guid: String,
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<[u8; 4]>,
    pub children: Vec<TreeNodeSerde>,
}

impl TreeNode {
    pub fn new(name: &str) -> Rc<RefCell<TreeNode>> {
        let node = Rc::new(RefCell::new(TreeNode {
            guid: Uuid::new_v4().to_string(),
            name: name.to_string(),
            color: None,
            children: Vec::new(),
            parent: None,
            weak_self: Weak::new(),
        }));
        node.borrow_mut().weak_self = Rc::downgrade(&node);
        node
    }

    pub fn color(&self) -> Option<[u8; 4]> {
        self.color
    }

    pub fn set_color(&mut self, r: u8, g: u8, b: u8, a: u8) {
        self.color = Some([r, g, b, a]);
    }

    pub fn add(&mut self, child: &Rc<RefCell<TreeNode>>) {
        child.borrow_mut().parent = Some(self.weak_self.clone());
        self.children.push(Rc::clone(child));
    }

    pub fn remove(&mut self, child: &Rc<RefCell<TreeNode>>) -> bool {
        let child_guid = child.borrow().guid.clone();
        if let Some(pos) = self.children.iter().position(|c| c.borrow().guid == child_guid) {
            let removed = self.children.remove(pos);
            removed.borrow_mut().parent = None;
            true
        } else {
            false
        }
    }

    pub fn parent(&self) -> Option<Rc<RefCell<TreeNode>>> {
        self.parent.as_ref()?.upgrade()
    }

    pub fn children(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        self.children.iter().map(Rc::clone).collect()
    }

    pub fn is_root(&self) -> bool {
        self.parent.is_none()
    }

    pub fn is_leaf(&self) -> bool {
        self.children.is_empty()
    }

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

    pub fn descendants(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = Vec::new();
        for child in &self.children {
            result.push(Rc::clone(child));
            result.extend(child.borrow().descendants());
        }
        result
    }

    pub fn nodes(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        let mut result = vec![self.weak_self.upgrade().unwrap()];
        for child in &self.children {
            result.extend(child.borrow().nodes());
        }
        result
    }

    pub fn root(&self) -> Rc<RefCell<TreeNode>> {
        if let Some(parent) = self.parent() {
            parent.borrow().root()
        } else {
            self.weak_self.upgrade().unwrap()
        }
    }

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

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let serde_node = self.to_serde();
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        SerTrait::serialize(&serde_node, &mut ser)?;
        Ok(String::from_utf8(buf)?)
    }

    pub fn jsonload(path: &str) -> Result<Rc<RefCell<TreeNode>>, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(path)?;
        let serde_node: TreeNodeSerde = serde_json::from_str(&json)?;
        Ok(Self::from_serde(serde_node))
    }

    pub(crate) fn to_serde(&self) -> TreeNodeSerde {
        TreeNodeSerde {
            guid: self.guid.clone(),
            name: self.name.clone(),
            color: self.color,
            children: self.children.iter().map(|c| c.borrow().to_serde()).collect(),
        }
    }

    pub(crate) fn from_serde(serde_node: TreeNodeSerde) -> Rc<RefCell<TreeNode>> {
        let node = TreeNode::new(&serde_node.name);
        node.borrow_mut().guid = serde_node.guid;
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
            self.guid,
            self.children.len()
        )
    }
}

#[cfg(test)]
#[path = "treenode_test.rs"]
mod treenode_test;
