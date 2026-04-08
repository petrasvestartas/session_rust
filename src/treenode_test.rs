use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_treenode_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::treenode::TreeNode;

        // Default constructor
        let n0 = TreeNode::new("my_node");

        // Constructor with name
        let n = TreeNode::new("my_named_node");
        n.borrow_mut().set_color(255, 0, 0, 255);

        // Minimal string representation
        let nstr = format!("{}", n.borrow());

        // Copies (compared by guid in Rust)
        let nother = TreeNode::new("my_named_node");

        MINI_CHECK!(n0.borrow().name == "my_node");
        MINI_CHECK!(!n0.borrow().guid().is_empty());
        MINI_CHECK!(n.borrow().name == "my_named_node");
        MINI_CHECK!(n.borrow().color().is_some() && n.borrow().color().unwrap()[0] == 255);
        MINI_CHECK!(nstr.contains("TreeNode(my_named_node"));
        MINI_CHECK!(*n.borrow() == *n.borrow());
        MINI_CHECK!(*n.borrow() != *nother.borrow());
    })
}

pub fn run_treenode_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::treenode::TreeNode;

        let original = TreeNode::new("test_node");
        let child = TreeNode::new("child_node");
        original.borrow_mut().add(&child);

        let json = original.borrow().jsondump().unwrap();
        std::fs::write("serialization/test_treenode.json", &json).unwrap();
        let loaded = TreeNode::jsonload("serialization/test_treenode.json").unwrap();

        MINI_CHECK!(loaded.borrow().name == original.borrow().name);
        MINI_CHECK!(loaded.borrow().children().len() == 1);
        MINI_CHECK!(loaded.borrow().children()[0].borrow().name == "child_node");
    })
}

pub fn run_treenode_is_root() -> TestResult {
    MINI_TEST!("Is Root", {
        use crate::treenode::TreeNode;

        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        root.borrow_mut().add(&child);

        MINI_CHECK!(root.borrow().is_root());
        MINI_CHECK!(!child.borrow().is_root());
    })
}

pub fn run_treenode_is_leaf() -> TestResult {
    MINI_TEST!("Is Leaf", {
        use crate::treenode::TreeNode;

        let parent = TreeNode::new("parent");
        let child = TreeNode::new("child");
        parent.borrow_mut().add(&child);

        MINI_CHECK!(child.borrow().is_leaf());
        MINI_CHECK!(!parent.borrow().is_leaf());
    })
}

pub fn run_treenode_add() -> TestResult {
    MINI_TEST!("Add", {
        use crate::treenode::TreeNode;

        let parent = TreeNode::new("parent");
        let child = TreeNode::new("child");
        parent.borrow_mut().add(&child);

        MINI_CHECK!(parent.borrow().children().len() == 1);
        MINI_CHECK!(child.borrow().parent().is_some());
    })
}

pub fn run_treenode_remove() -> TestResult {
    MINI_TEST!("Remove", {
        use crate::treenode::TreeNode;

        let parent = TreeNode::new("parent");
        let child = TreeNode::new("child");
        parent.borrow_mut().add(&child);
        let removed = parent.borrow_mut().remove(&child);

        MINI_CHECK!(removed);
        MINI_CHECK!(parent.borrow().children().is_empty());
        MINI_CHECK!(child.borrow().parent().is_none());
    })
}

pub fn run_treenode_parent() -> TestResult {
    MINI_TEST!("Parent", {
        use crate::treenode::TreeNode;

        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        root.borrow_mut().add(&child);

        MINI_CHECK!(root.borrow().parent().is_none());
        MINI_CHECK!(child.borrow().parent().is_some());
    })
}

pub fn run_treenode_ancestors() -> TestResult {
    MINI_TEST!("Ancestors", {
        use crate::treenode::TreeNode;

        let root = TreeNode::new("root");
        let mid = TreeNode::new("mid");
        let leaf = TreeNode::new("leaf");
        root.borrow_mut().add(&mid);
        mid.borrow_mut().add(&leaf);

        let anc = leaf.borrow().ancestors();

        MINI_CHECK!(anc.len() == 2);
        MINI_CHECK!(anc[0].borrow().name == "mid");
        MINI_CHECK!(anc[1].borrow().name == "root");
    })
}

pub fn run_treenode_descendants() -> TestResult {
    MINI_TEST!("Descendants", {
        use crate::treenode::TreeNode;

        let root = TreeNode::new("root");
        let mid = TreeNode::new("mid");
        let leaf = TreeNode::new("leaf");
        root.borrow_mut().add(&mid);
        mid.borrow_mut().add(&leaf);

        let desc = root.borrow().descendants();

        MINI_CHECK!(desc.len() == 2);
        MINI_CHECK!(desc[0].borrow().name == "mid");
        MINI_CHECK!(desc[1].borrow().name == "leaf");
    })
}

pub fn run_treenode_children() -> TestResult {
    MINI_TEST!("Children", {
        use crate::treenode::TreeNode;

        let parent = TreeNode::new("parent");
        let c1 = TreeNode::new("c1");
        let c2 = TreeNode::new("c2");
        parent.borrow_mut().add(&c1);
        parent.borrow_mut().add(&c2);

        let kids = parent.borrow().children();

        MINI_CHECK!(kids.len() == 2);
        MINI_CHECK!(kids[0].borrow().name == "c1");
        MINI_CHECK!(kids[1].borrow().name == "c2");
    })
}

pub fn run_treenode_traverse() -> TestResult {
    MINI_TEST!("Traverse", {
        use crate::treenode::TreeNode;

        let root = TreeNode::new("root");
        let a = TreeNode::new("a");
        let b = TreeNode::new("b");
        root.borrow_mut().add(&a);
        root.borrow_mut().add(&b);

        let preorder = root.borrow().traverse("depthfirst", "preorder");
        let postorder = root.borrow().traverse("depthfirst", "postorder");
        let bfs = root.borrow().traverse("breadthfirst", "preorder");

        MINI_CHECK!(preorder.len() == 3 && preorder[0].borrow().name == "root");
        MINI_CHECK!(postorder.len() == 3 && postorder[2].borrow().name == "root");
        MINI_CHECK!(bfs.len() == 3 && bfs[0].borrow().name == "root");
    })
}

REGISTER_MINI_TEST!("TreeNode", "Constructor", crate::treenode_test::run_treenode_constructor);
REGISTER_MINI_TEST!("TreeNode", "Json Roundtrip", crate::treenode_test::run_treenode_json_roundtrip);
REGISTER_MINI_TEST!("TreeNode", "Is Root", crate::treenode_test::run_treenode_is_root);
REGISTER_MINI_TEST!("TreeNode", "Is Leaf", crate::treenode_test::run_treenode_is_leaf);
REGISTER_MINI_TEST!("TreeNode", "Add", crate::treenode_test::run_treenode_add);
REGISTER_MINI_TEST!("TreeNode", "Remove", crate::treenode_test::run_treenode_remove);
REGISTER_MINI_TEST!("TreeNode", "Parent", crate::treenode_test::run_treenode_parent);
REGISTER_MINI_TEST!("TreeNode", "Ancestors", crate::treenode_test::run_treenode_ancestors);
REGISTER_MINI_TEST!("TreeNode", "Descendants", crate::treenode_test::run_treenode_descendants);
REGISTER_MINI_TEST!("TreeNode", "Children", crate::treenode_test::run_treenode_children);
REGISTER_MINI_TEST!("TreeNode", "Traverse", crate::treenode_test::run_treenode_traverse);
