use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_treenode_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::treenode::TreeNode;
        use crate::encoders::{json_dump, json_load};
        let original = TreeNode::new("test_node");
        let child = TreeNode::new("child_node");
        original.add(&child);
        json_dump(&original, "serialization/test_treenode.json", false).unwrap();
        let loaded = json_load::<TreeNode>("serialization/test_treenode.json").unwrap();
        MINI_CHECK!(loaded.name() == original.name());
    })
}

REGISTER_MINI_TEST!("TreeNode", "Json Roundtrip", crate::treenode_test::run_treenode_json_roundtrip);
