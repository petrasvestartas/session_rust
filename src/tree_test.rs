use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_treenode_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::TreeNode;

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
        use crate::TreeNode;

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
        use crate::TreeNode;

        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        root.borrow_mut().add(&child);

        MINI_CHECK!(root.borrow().is_root());
        MINI_CHECK!(!child.borrow().is_root());
    })
}

pub fn run_treenode_is_leaf() -> TestResult {
    MINI_TEST!("Is Leaf", {
        use crate::TreeNode;

        let parent = TreeNode::new("parent");
        let child = TreeNode::new("child");
        parent.borrow_mut().add(&child);

        MINI_CHECK!(child.borrow().is_leaf());
        MINI_CHECK!(!parent.borrow().is_leaf());
    })
}

pub fn run_treenode_tree() -> TestResult {
    MINI_TEST!("Tree", {
        use crate::TreeNode;

        let n = TreeNode::new("standalone");

        MINI_CHECK!(n.borrow().is_root());
    })
}

pub fn run_treenode_add() -> TestResult {
    MINI_TEST!("Add", {
        use crate::TreeNode;

        let parent = TreeNode::new("parent");
        let child = TreeNode::new("child");
        parent.borrow_mut().add(&child);

        MINI_CHECK!(parent.borrow().children().len() == 1);
        MINI_CHECK!(child.borrow().parent().is_some());
    })
}

pub fn run_treenode_remove() -> TestResult {
    MINI_TEST!("Remove", {
        use crate::TreeNode;

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
        use crate::TreeNode;

        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        root.borrow_mut().add(&child);

        MINI_CHECK!(root.borrow().parent().is_none());
        MINI_CHECK!(child.borrow().parent().is_some());
    })
}

pub fn run_treenode_ancestors() -> TestResult {
    MINI_TEST!("Ancestors", {
        use crate::TreeNode;

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
        use crate::TreeNode;

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
        use crate::TreeNode;

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
        use crate::TreeNode;

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

pub fn run_tree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Tree;

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
        use crate::Tree;
        use crate::TreeNode;

        let mut original = Tree::new("test_tree");
        let root_node = TreeNode::new("root_node");
        original.add(&root_node, None);

        //   jsondump()      │ String       │ to JSON string (internal use)
        //   jsonload(s)     │ String       │ from JSON string (internal use)
        //   file_json_dumps()    │ String       │ to JSON string
        //   file_json_loads(s)   │ String       │ from JSON string
        //   file_json_dump(path) │ file         │ write to file
        //   file_json_load(path) │ file         │ read from file

        let fname = "serialization/test_tree.json";
        original.file_json_dump(fname).unwrap();
        let loaded = Tree::file_json_load(fname).unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.nodes().len() == original.nodes().len());
    })
}

pub fn run_tree_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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
        use crate::Tree;
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

REGISTER_MINI_TEST!("TreeNode", "Constructor", crate::tree_test::run_treenode_constructor);
REGISTER_MINI_TEST!("TreeNode", "Json Roundtrip", crate::tree_test::run_treenode_json_roundtrip);
REGISTER_MINI_TEST!("TreeNode", "Is Root", crate::tree_test::run_treenode_is_root);
REGISTER_MINI_TEST!("TreeNode", "Is Leaf", crate::tree_test::run_treenode_is_leaf);
REGISTER_MINI_TEST!("TreeNode", "Tree", crate::tree_test::run_treenode_tree);
REGISTER_MINI_TEST!("TreeNode", "Add", crate::tree_test::run_treenode_add);
REGISTER_MINI_TEST!("TreeNode", "Remove", crate::tree_test::run_treenode_remove);
REGISTER_MINI_TEST!("TreeNode", "Parent", crate::tree_test::run_treenode_parent);
REGISTER_MINI_TEST!("TreeNode", "Ancestors", crate::tree_test::run_treenode_ancestors);
REGISTER_MINI_TEST!("TreeNode", "Descendants", crate::tree_test::run_treenode_descendants);
REGISTER_MINI_TEST!("TreeNode", "Children", crate::tree_test::run_treenode_children);
REGISTER_MINI_TEST!("TreeNode", "Traverse", crate::tree_test::run_treenode_traverse);
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
