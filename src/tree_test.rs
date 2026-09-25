use crate::mini_test::TestResult;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;

// ═══════════════════════════════════════════════════════════════════════════
// TreeNode
// ═══════════════════════════════════════════════════════════════════════════
pub fn run_treenode_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Color;
        use crate::TreeNode;

        let n0 = TreeNode::new("my_node");
        let n = TreeNode::new("my_named_node");
        n.borrow_mut().color = Some(Color::new(1.0, 0.0, 0.0, 1.0));
        let nstr = format!("{}", n.borrow());
        let nother = TreeNode::new("my_named_node");

        MINI_CHECK!(n0.borrow().name == "my_node");
        MINI_CHECK!(!n0.borrow().guid().is_empty());
        MINI_CHECK!(n.borrow().name == "my_named_node");
        MINI_CHECK!(n.borrow().color.is_some() && n.borrow().color.as_ref().unwrap().r == 1.0);
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

        let fname = "serialization/test_treenode.json";
        std::fs::write(fname, original.borrow().jsondump().unwrap()).unwrap();
        let loaded = TreeNode::jsonload(&std::fs::read_to_string(fname).unwrap()).unwrap();

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

pub fn run_treenode_add() -> TestResult {
    MINI_TEST!("Add", {
        use crate::TreeNode;
        use std::rc::Rc;

        let parent = TreeNode::new("parent");
        let child = TreeNode::new("child");
        parent.borrow_mut().add(&child);
        let parent_clone = Rc::clone(&parent);
        parent.borrow_mut().add(&parent_clone);

        MINI_CHECK!(parent.borrow().children().len() == 1);
        MINI_CHECK!(Rc::ptr_eq(&child.borrow().parent().unwrap(), &parent));
    })
}

pub fn run_treenode_remove() -> TestResult {
    MINI_TEST!("Remove", {
        use crate::TreeNode;
        use std::rc::Rc;

        let parent = TreeNode::new("parent");
        let child = TreeNode::new("child");
        parent.borrow_mut().add(&child);
        let removed = parent.borrow_mut().remove(&child);

        MINI_CHECK!(removed.is_some() && Rc::ptr_eq(&removed.unwrap(), &child));
        MINI_CHECK!(parent.borrow().children().is_empty());
        MINI_CHECK!(child.borrow().parent().is_none());
    })
}

pub fn run_treenode_parent() -> TestResult {
    MINI_TEST!("Parent", {
        use crate::TreeNode;
        use std::rc::Rc;

        let root = TreeNode::new("root");
        let child = TreeNode::new("child");
        root.borrow_mut().add(&child);

        MINI_CHECK!(root.borrow().parent().is_none());
        MINI_CHECK!(Rc::ptr_eq(&child.borrow().parent().unwrap(), &root));
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

pub fn run_treenode_set_dead() -> TestResult {
    MINI_TEST!("Set Dead", {
        use crate::TreeNode;
        use std::rc::Rc;

        let p = TreeNode::new("p");
        let a = TreeNode::new("a");
        let b = TreeNode::new("b");
        let c = TreeNode::new("c");
        let d = TreeNode::new("d");
        p.borrow_mut().add(&a);
        p.borrow_mut().add(&b);
        p.borrow_mut().add(&c);
        b.borrow_mut().add(&d);
        b.borrow_mut().set_dead(true);
        let kids = p.borrow().children();
        let ancestors = d.borrow().ancestors();

        MINI_CHECK!(kids.len() == 2 && Rc::ptr_eq(&kids[0], &a) && Rc::ptr_eq(&kids[1], &c));
        MINI_CHECK!(b.borrow().is_dead() && b.borrow().parent().is_none());
        MINI_CHECK!(Rc::ptr_eq(&b.borrow().children()[0], &d));
        MINI_CHECK!(Rc::ptr_eq(&d.borrow().parent().unwrap(), &b));
        MINI_CHECK!(ancestors.len() == 1 && Rc::ptr_eq(&ancestors[0], &b));
        MINI_CHECK!(!p.borrow().is_leaf());

        a.borrow_mut().set_dead(true);
        c.borrow_mut().set_dead(true);

        MINI_CHECK!(p.borrow().is_leaf());

        a.borrow_mut().set_dead(false);
        b.borrow_mut().set_dead(false);
        c.borrow_mut().set_dead(false);
        let kids = p.borrow().children();

        MINI_CHECK!(kids.len() == 3 && Rc::ptr_eq(&kids[1], &b));
        MINI_CHECK!(Rc::ptr_eq(&b.borrow().parent().unwrap(), &p));
    })
}

pub fn run_treenode_compact() -> TestResult {
    MINI_TEST!("Compact", {
        use crate::history::Tomb;
        use crate::TreeNode;
        use std::rc::Rc;

        let p = TreeNode::new("p");
        let mut kids = Vec::new();

        for i in 0..6 {
            kids.push(TreeNode::new(&format!("c{i}")));
            p.borrow_mut().add(&kids[i]);
        }

        let tomb = Tomb::new("", false, 0, Some(Rc::clone(&kids[3])));
        kids[1].borrow_mut().set_dead(true);
        kids[3].borrow_mut().set_dead(true);
        kids[4].borrow_mut().set_dead(true);
        kids[3].borrow_mut().set_tomb(&tomb);
        let before = p.borrow().children();
        p.borrow_mut().compact();
        let after = p.borrow().children();
        let raw = p.borrow_mut().compact_step(usize::MAX);
        let q = TreeNode::new("q");
        q.borrow_mut().add(&kids[5]);
        let moved = p.borrow().children();

        MINI_CHECK!(raw == 4 && !p.borrow().is_compacting());
        MINI_CHECK!(after.len() == 3 && after.iter().zip(&before).all(|(x, y)| Rc::ptr_eq(x, y)));
        MINI_CHECK!(kids[3].borrow().is_dead() && kids[3].borrow().get_tomb().is_some());
        MINI_CHECK!(kids[1].borrow().get_tomb().is_none());
        MINI_CHECK!(
            moved.len() == 2 && Rc::ptr_eq(&moved[0], &kids[0]) && Rc::ptr_eq(&moved[1], &kids[2])
        );
    })
}

pub fn run_treenode_add_moves() -> TestResult {
    MINI_TEST!("Add Moves", {
        use crate::Tree;
        use crate::TreeNode;
        use std::rc::Rc;

        let mut tree = Tree::new("t");
        let root = TreeNode::new("root");
        let p1 = TreeNode::new("p1");
        let p2 = TreeNode::new("p2");
        let w = TreeNode::new("w");
        let x = TreeNode::new("x");
        let z = TreeNode::new("z");
        let y = TreeNode::new("y");
        tree.add(&root, None);
        tree.add(&p1, Some(&root));
        tree.add(&p2, Some(&root));
        tree.add(&w, Some(&p1));
        tree.add(&x, Some(&p1));
        tree.add(&z, Some(&p1));
        tree.add(&y, Some(&x));
        let count = tree.nodes().len();
        let ghost = p2.borrow_mut().add(&x).unwrap();
        let again = p2.borrow_mut().add(&x);
        let old = p1.borrow().children();
        let new = p2.borrow().children();

        MINI_CHECK!(ghost.borrow().is_dead() && ghost.borrow().name.is_empty());
        MINI_CHECK!(old.len() == 2 && Rc::ptr_eq(&old[0], &w) && Rc::ptr_eq(&old[1], &z));
        MINI_CHECK!(Rc::ptr_eq(new.last().unwrap(), &x));
        MINI_CHECK!(Rc::ptr_eq(&x.borrow().parent().unwrap(), &p2));
        MINI_CHECK!(Rc::ptr_eq(&y.borrow().parent().unwrap(), &x));
        MINI_CHECK!(tree.nodes().len() == count);
        MINI_CHECK!(again.is_none() && new.len() == 1);
    })
}

// ═══════════════════════════════════════════════════════════════════════════
// Tree
// ═══════════════════════════════════════════════════════════════════════════
pub fn run_tree_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Tree;

        let t0 = Tree::default();
        let t = Tree::new("my_named_tree");
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
        original.pb_dump(fname).unwrap();
        let loaded = Tree::pb_load(fname).unwrap();

        MINI_CHECK!(loaded.name == original.name);
        MINI_CHECK!(loaded.nodes().len() == original.nodes().len());
    })
}

pub fn run_tree_root() -> TestResult {
    MINI_TEST!("Root", {
        use crate::Tree;
        use crate::TreeNode;
        use std::rc::Rc;

        let mut t = Tree::new("t");
        let root = TreeNode::new("root");
        t.add(&root, None);

        MINI_CHECK!(Rc::ptr_eq(&t.root().unwrap(), &root));
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

        MINI_CHECK!(found.is_some() && found.unwrap().borrow().guid() == root_guid);
        MINI_CHECK!(t.find_node_by_guid("missing-guid").is_none());
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
        let cycle = t.add_child_by_guid(&b_guid, &a_guid);

        MINI_CHECK!(ok);
        MINI_CHECK!(!cycle);
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

pub fn run_tree_dead_nodes() -> TestResult {
    MINI_TEST!("Dead Nodes", {
        use crate::Tree;
        use crate::TreeNode;

        let mut tree = Tree::new("t");
        let root = TreeNode::new("root");
        let g = TreeNode::new("group");
        let a = TreeNode::new("alpha");
        let b = TreeNode::new("beta");
        let c = TreeNode::new("gamma");
        let d = TreeNode::new("delta");
        tree.add(&root, None);
        tree.add(&g, Some(&root));
        tree.add(&a, Some(&g));
        tree.add(&b, Some(&g));
        tree.add(&c, Some(&b));
        tree.add(&d, Some(&g));
        b.borrow_mut().set_dead(true);
        let b_guid = b.borrow().guid().to_string();
        let c_guid = c.borrow().guid().to_string();
        let g_guid = g.borrow().guid().to_string();
        let names = |nodes: Vec<std::rc::Rc<std::cell::RefCell<TreeNode>>>| -> Vec<String> {
            let mut result = Vec::new();

            for node in &nodes {
                result.push(node.borrow().name.clone());
            }

            result
        };
        let json = tree.jsondump().unwrap();
        let from_json = Tree::jsonload(&json).unwrap();
        let from_pb = Tree::pb_loads(&tree.pb_dumps()).unwrap();
        let expected = ["root", "group", "alpha", "delta"];

        MINI_CHECK!(names(tree.nodes()) == expected);
        MINI_CHECK!(names(tree.leaves()) == ["alpha", "delta"]);
        MINI_CHECK!(
            tree.get_node_by_name("beta").is_none() && tree.get_nodes_by_name("gamma").is_empty()
        );
        MINI_CHECK!(
            tree.find_node_by_guid(&b_guid).is_none() && tree.find_node_by_guid(&c_guid).is_none()
        );
        MINI_CHECK!(tree.get_children_guids(&g_guid).len() == 2);
        MINI_CHECK!(names(tree.traverse("depthfirst", "preorder")) == expected);
        MINI_CHECK!(names(tree.traverse("breadthfirst", "preorder")) == expected);
        MINI_CHECK!(!tree.str().contains("beta") && !tree.str().contains("gamma"));
        MINI_CHECK!(
            tree.repr() == "Tree(t, 4 nodes)" && g.borrow().str() == "TreeNode(group, 2 children)"
        );
        MINI_CHECK!(!json.contains("beta") && !json.contains("gamma"));
        MINI_CHECK!(names(from_json.nodes()) == expected && names(from_pb.nodes()) == expected);
    })
}

REGISTER_MINI_TEST!(
    "TreeNode",
    "Constructor",
    crate::tree_test::run_treenode_constructor
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Json Roundtrip",
    crate::tree_test::run_treenode_json_roundtrip
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Is Root",
    crate::tree_test::run_treenode_is_root
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Is Leaf",
    crate::tree_test::run_treenode_is_leaf
);
REGISTER_MINI_TEST!("TreeNode", "Add", crate::tree_test::run_treenode_add);
REGISTER_MINI_TEST!("TreeNode", "Remove", crate::tree_test::run_treenode_remove);
REGISTER_MINI_TEST!("TreeNode", "Parent", crate::tree_test::run_treenode_parent);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Ancestors",
    crate::tree_test::run_treenode_ancestors
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Descendants",
    crate::tree_test::run_treenode_descendants
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Children",
    crate::tree_test::run_treenode_children
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Traverse",
    crate::tree_test::run_treenode_traverse
);
REGISTER_MINI_TEST!(
    "Tree",
    "Constructor",
    crate::tree_test::run_tree_constructor
);
REGISTER_MINI_TEST!(
    "Tree",
    "Json Roundtrip",
    crate::tree_test::run_tree_json_roundtrip
);
REGISTER_MINI_TEST!(
    "Tree",
    "Protobuf Roundtrip",
    crate::tree_test::run_tree_protobuf_roundtrip
);
REGISTER_MINI_TEST!("Tree", "Root", crate::tree_test::run_tree_root);
REGISTER_MINI_TEST!("Tree", "Add", crate::tree_test::run_tree_add);
REGISTER_MINI_TEST!("Tree", "Nodes", crate::tree_test::run_tree_nodes);
REGISTER_MINI_TEST!("Tree", "Remove", crate::tree_test::run_tree_remove);
REGISTER_MINI_TEST!("Tree", "Leaves", crate::tree_test::run_tree_leaves);
REGISTER_MINI_TEST!("Tree", "Traverse", crate::tree_test::run_tree_traverse);
REGISTER_MINI_TEST!(
    "Tree",
    "Get Node By Name",
    crate::tree_test::run_tree_get_node_by_name
);
REGISTER_MINI_TEST!(
    "Tree",
    "Get Nodes By Name",
    crate::tree_test::run_tree_get_nodes_by_name
);
REGISTER_MINI_TEST!(
    "Tree",
    "Find Node By Guid",
    crate::tree_test::run_tree_find_node_by_guid
);
REGISTER_MINI_TEST!(
    "Tree",
    "Add Child By Guid",
    crate::tree_test::run_tree_add_child_by_guid
);
REGISTER_MINI_TEST!(
    "Tree",
    "Get Children Guids",
    crate::tree_test::run_tree_get_children_guids
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Set Dead",
    crate::tree_test::run_treenode_set_dead
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Compact",
    crate::tree_test::run_treenode_compact
);
REGISTER_MINI_TEST!(
    "TreeNode",
    "Add Moves",
    crate::tree_test::run_treenode_add_moves
);
REGISTER_MINI_TEST!("Tree", "Dead Nodes", crate::tree_test::run_tree_dead_nodes);
