use crate::treenode::{TreeNode, TreeNodeSerde};
use serde::{ser::Serialize as SerTrait, Deserialize, Serialize};
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

#[derive(Debug, Clone)]
pub struct Tree {
    guid: std::sync::OnceLock<String>,
    pub name: String,
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
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    pub fn new(name: &str) -> Self {
        Self {
            guid: std::sync::OnceLock::new(),
            name: name.to_string(),
            root_node: None,
        }
    }

    pub fn root(&self) -> Option<Rc<RefCell<TreeNode>>> {
        self.root_node.clone()
    }

    pub fn add(&mut self, node: &Rc<RefCell<TreeNode>>, parent: Option<&Rc<RefCell<TreeNode>>>) {
        if parent.is_none() {
            self.root_node = Some(Rc::clone(node));
        } else if let Some(parent_node) = parent {
            parent_node.borrow_mut().add(node);
        }
    }

    pub fn nodes(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        if let Some(root) = &self.root_node {
            root.borrow().nodes()
        } else {
            vec![]
        }
    }

    pub fn remove(&mut self, node: &Rc<RefCell<TreeNode>>) -> bool {
        if let Some(root) = &self.root_node {
            let node_guid = node.borrow().guid().to_string();
            if root.borrow().guid() == node_guid {
                self.root_node = None;
                true
            } else if let Some(parent) = self.find_parent_of_node(&node_guid) {
                parent.borrow_mut().remove(node)
            } else {
                false
            }
        } else {
            false
        }
    }

    fn find_parent_of_node(&self, node_guid: &String) -> Option<Rc<RefCell<TreeNode>>> {
        if let Some(root) = &self.root_node {
            Self::find_parent_recursive(root, node_guid)
        } else {
            None
        }
    }

    fn find_parent_recursive(
        node: &Rc<RefCell<TreeNode>>,
        target_guid: &String,
    ) -> Option<Rc<RefCell<TreeNode>>> {
        let children = node.borrow().children();
        for child in children {
            if child.borrow().guid() == *target_guid {
                return Some(Rc::clone(node));
            }
            if let Some(found) = Self::find_parent_recursive(&child, target_guid) {
                return Some(found);
            }
        }
        None
    }

    pub fn leaves(&self) -> Vec<Rc<RefCell<TreeNode>>> {
        self.nodes().into_iter().filter(|n| n.borrow().is_leaf()).collect()
    }

    pub fn traverse(&self, strategy: &str, order: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        if let Some(root) = &self.root_node {
            root.borrow().traverse(strategy, order)
        } else {
            vec![]
        }
    }

    pub fn get_node_by_name(&self, node_name: &str) -> Option<Rc<RefCell<TreeNode>>> {
        self.nodes().into_iter().find(|n| n.borrow().name == node_name)
    }

    pub fn get_nodes_by_name(&self, node_name: &str) -> Vec<Rc<RefCell<TreeNode>>> {
        self.nodes()
            .into_iter()
            .filter(|n| n.borrow().name == node_name)
            .collect()
    }

    pub fn find_node_by_guid(&self, node_guid: &String) -> Option<Rc<RefCell<TreeNode>>> {
        self.nodes().into_iter().find(|n| n.borrow().guid() == *node_guid)
    }

    pub fn add_child_by_guid(&mut self, parent_guid: &String, child_guid: &String) -> bool {
        let parent_node = self.find_node_by_guid(parent_guid);
        let child_node = self.find_node_by_guid(child_guid);
        if let (Some(parent), Some(child)) = (parent_node, child_node) {
            if let Some(current_parent) = child.borrow().parent() {
                current_parent.borrow_mut().remove(&child);
            }
            parent.borrow_mut().add(&child);
            true
        } else {
            false
        }
    }

    pub fn get_children_guids(&self, node_guid: &String) -> Vec<String> {
        if let Some(node) = self.find_node_by_guid(node_guid) {
            node.borrow().children().iter().map(|c| c.borrow().guid().to_string()).collect()
        } else {
            vec![]
        }
    }

    pub fn get_children(&self, node_guid: &str) -> Vec<String> {
        self.get_children_guids(&node_guid.to_string())
    }

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

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let serde_tree = TreeSerde {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            root: self.root_node.as_ref().map(|r| r.borrow().to_serde()),
        };
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        SerTrait::serialize(&serde_tree, &mut ser)?;
        Ok(String::from_utf8(buf)?)
    }

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

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::default())
    }

    pub fn json_dump(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(path, json)?;
        Ok(())
    }

    pub fn json_load(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(path)?;
        Self::jsonload(&json)
    }

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
                    r: c[0] as i32,
                    g: c[1] as i32,
                    b: c[2] as i32,
                    a: c[3] as i32,
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

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Tree::decode(data)?;
        let mut tree = Tree::new(&proto.name);
        tree.set_guid(proto.guid.clone());
        fn proto_to_serde(
            proto_node: &crate::proto::TreeNode,
        ) -> crate::treenode::TreeNodeSerde {
            crate::treenode::TreeNodeSerde {
                guid: proto_node.guid.clone(),
                name: proto_node.name.clone(),
                color: proto_node.color.as_ref()
                    .filter(|c| c.a > 0)
                    .map(|c| [c.r as u8, c.g as u8, c.b as u8, c.a as u8]),
                children: proto_node
                    .children
                    .iter()
                    .map(|c| proto_to_serde(c))
                    .collect(),
            }
        }
        if let Some(root_proto) = &proto.root {
            let serde = proto_to_serde(root_proto);
            tree.root_node = Some(crate::treenode::TreeNode::from_serde(serde));
        }
        Ok(tree)
    }

    pub fn pb_dump(&self, path: &str) {
        std::fs::write(path, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(path: &str) -> Self {
        let data = std::fs::read(path).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl fmt::Display for Tree {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Tree({}, {})", self.name, self.guid())
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
