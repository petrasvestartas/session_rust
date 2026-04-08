use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_tree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::tree::Tree;

        // Default constructor
        let t0 = Tree::default();

        // Constructor with name
        let t = Tree::new("my_named_tree");

        // Minimal string representation
        let tstr = format!("{}", t);

        MINI_CHECK!(t0.name == "my_tree");
        MINI_CHECK!(!t0.guid().is_empty());
        MINI_CHECK!(t.name == "my_named_tree");
        MINI_CHECK!(tstr.contains("Tree"));
    })
}

pub fn run_tree_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut original = Tree::new("test_tree");
        let root_node = TreeNode::new("root_node");
        original.add(&root_node, None);

        //   jsondump()      │ String       │ to JSON string (internal use)
        //   jsonload(s)     │ String       │ from JSON string (internal use)
        //   json_dumps()    │ String       │ to JSON string
        //   json_loads(s)   │ String       │ from JSON string
        //   json_dump(path) │ file         │ write to file
        //   json_load(path) │ file         │ read from file

        let fname = "serialization/test_tree.json";
        original.json_dump(fname).unwrap();
        let loaded = Tree::json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.nodes().len() == original.nodes().len());
    })
}

pub fn run_tree_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut original = Tree::new("test_tree");
        let root_node = TreeNode::new("root_node");
        original.add(&root_node, None);

        let fname = "serialization/test_tree.bin";
        original.pb_dump(fname);
        let loaded = Tree::pb_load(fname);

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.nodes().len() == original.nodes().len());
    })
}

pub fn run_tree_root() -> TestResult {
    MINI_TEST!("Root", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        t.add(&root, None);

        MINI_CHECK!(t.root().is_some());
        MINI_CHECK!(t.root().unwrap().borrow().name == "root");
    })
}

pub fn run_tree_add() -> TestResult {
    MINI_TEST!("Add", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        t.add(&root, None);
        t.add(&child, Some(&root));

        MINI_CHECK!(t.nodes().len() == 2);
    })
}

pub fn run_tree_nodes() -> TestResult {
    MINI_TEST!("Nodes", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        t.add(&root, None);
        t.add(&child, Some(&root));

        let all_nodes = t.nodes();

        MINI_CHECK!(all_nodes.len() == 2);
        MINI_CHECK!(all_nodes[0].borrow().name == "root");
        MINI_CHECK!(all_nodes[1].borrow().name == "child");
    })
}

pub fn run_tree_remove() -> TestResult {
    MINI_TEST!("Remove", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        t.add(&root, None);
        t.add(&child, Some(&root));
        t.remove(&child);

        MINI_CHECK!(t.nodes().len() == 1);
    })
}

pub fn run_tree_leaves() -> TestResult {
    MINI_TEST!("Leaves", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let a = TreeNode::new("a");
        let b = TreeNode::new("b");
        t.add(&root, None);
        t.add(&a, Some(&root));
        t.add(&b, Some(&root));

        let lvs = t.leaves();

        MINI_CHECK!(lvs.len() == 2);
        MINI_CHECK!(lvs[0].borrow().name == "a");
        MINI_CHECK!(lvs[1].borrow().name == "b");
    })
}

pub fn run_tree_traverse() -> TestResult {
    MINI_TEST!("Traverse", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let a = TreeNode::new("a");
        let b = TreeNode::new("b");
        t.add(&root, None);
        t.add(&a, Some(&root));
        t.add(&b, Some(&root));

        let preorder = t.traverse("depthfirst", "preorder");
        let bfs = t.traverse("breadthfirst", "preorder");

        MINI_CHECK!(preorder.len() == 3 && preorder[0].borrow().name == "root");
        MINI_CHECK!(bfs.len() == 3 && bfs[0].borrow().name == "root");
    })
}

pub fn run_tree_get_node_by_name() -> TestResult {
    MINI_TEST!("Get Node By Name", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let child = TreeNode::new("target");
        t.add(&root, None);
        t.add(&child, Some(&root));

        let found = t.get_node_by_name("target");

        MINI_CHECK!(found.is_some() && found.unwrap().borrow().name == "target");
        MINI_CHECK!(t.get_node_by_name("missing").is_none());
    })
}

pub fn run_tree_get_nodes_by_name() -> TestResult {
    MINI_TEST!("Get Nodes By Name", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let a = TreeNode::new("dup");
        let b = TreeNode::new("dup");
        t.add(&root, None);
        t.add(&a, Some(&root));
        t.add(&b, Some(&root));

        let found = t.get_nodes_by_name("dup");

        MINI_CHECK!(found.len() == 2);
    })
}

pub fn run_tree_find_node_by_guid() -> TestResult {
    MINI_TEST!("Find Node By Guid", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        t.add(&root, None);
        let root_guid = root.borrow().guid().to_string();

        let found = t.find_node_by_guid(&root_guid);

        MINI_CHECK!(found.is_some());
        MINI_CHECK!(found.unwrap().borrow().guid() == root_guid);
        MINI_CHECK!(t.find_node_by_guid(&"missing-guid".to_string()).is_none());
    })
}

pub fn run_tree_add_child_by_guid() -> TestResult {
    MINI_TEST!("Add Child By Guid", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let a = TreeNode::new("a");
        let b = TreeNode::new("b");
        t.add(&root, None);
        t.add(&a, Some(&root));
        t.add(&b, Some(&root));
        let a_guid = a.borrow().guid().to_string();
        let b_guid = b.borrow().guid().to_string();
        let ok = t.add_child_by_guid(&a_guid, &b_guid);

        MINI_CHECK!(ok);
        MINI_CHECK!(a.borrow().children().len() == 1);
    })
}

pub fn run_tree_get_children_guids() -> TestResult {
    MINI_TEST!("Get Children Guids", {
        use crate::tree::Tree;
        use crate::TreeNode;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        let a = TreeNode::new("a");
        let b = TreeNode::new("b");
        t.add(&root, None);
        t.add(&a, Some(&root));
        t.add(&b, Some(&root));
        let root_guid = root.borrow().guid().to_string();

        let guids = t.get_children_guids(&root_guid);

        MINI_CHECK!(guids.len() == 2);
        MINI_CHECK!(guids[0] == a.borrow().guid());
        MINI_CHECK!(guids[1] == b.borrow().guid());
    })
}

REGISTER_MINI_TEST!("Tree", "Constructor", crate::tree_test::run_tree_constructor);
REGISTER_MINI_TEST!("Tree", "Json Roundtrip", crate::tree_test::run_tree_json_roundtrip);
REGISTER_MINI_TEST!("Tree", "Protobuf Roundtrip", crate::tree_test::run_tree_protobuf_roundtrip);
REGISTER_MINI_TEST!("Tree", "Root", crate::tree_test::run_tree_root);
REGISTER_MINI_TEST!("Tree", "Add", crate::tree_test::run_tree_add);
REGISTER_MINI_TEST!("Tree", "Nodes", crate::tree_test::run_tree_nodes);
REGISTER_MINI_TEST!("Tree", "Remove", crate::tree_test::run_tree_remove);
REGISTER_MINI_TEST!("Tree", "Leaves", crate::tree_test::run_tree_leaves);
REGISTER_MINI_TEST!("Tree", "Traverse", crate::tree_test::run_tree_traverse);
REGISTER_MINI_TEST!("Tree", "Get Node By Name", crate::tree_test::run_tree_get_node_by_name);
REGISTER_MINI_TEST!("Tree", "Get Nodes By Name", crate::tree_test::run_tree_get_nodes_by_name);
REGISTER_MINI_TEST!("Tree", "Find Node By Guid", crate::tree_test::run_tree_find_node_by_guid);
REGISTER_MINI_TEST!("Tree", "Add Child By Guid", crate::tree_test::run_tree_add_child_by_guid);
REGISTER_MINI_TEST!("Tree", "Get Children Guids", crate::tree_test::run_tree_get_children_guids);
