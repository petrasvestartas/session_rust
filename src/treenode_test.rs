use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

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
    })
}

REGISTER_MINI_TEST!("TreeNode", "Json Roundtrip", crate::treenode_test::run_treenode_json_roundtrip);
