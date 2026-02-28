use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_tree_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::tree::Tree;
        use crate::TreeNode;
        use crate::Point;
        use crate::encoders::{json_dump, json_load};
        let mut original = Tree::new("./serialization/test_tree");
        let point1 = Point::new(1.0, 2.0, 3.0);
        let node1 = TreeNode::new(&point1.guid.to_string());
        original.add(&node1, None);
        json_dump(&original, "serialization/test_tree.json", false).unwrap();
        let loaded = json_load::<Tree>("serialization/test_tree.json").unwrap();
        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.nodes().len() == original.nodes().len());
    })
}

REGISTER_MINI_TEST!("Tree", "Json Roundtrip", crate::tree_test::run_tree_json_roundtrip);
