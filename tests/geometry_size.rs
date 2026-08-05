use session_rust::Geometry;

// P1 (SESSION_DATASTRUCTURE_PLAN.md): every Geometry variant is boxed so the enum stays
// pointer-sized instead of paying for the largest variant inline (Element was ~9 KB — 200k
// objects cost 2.5 GB). If this fails, someone added an unboxed variant.
#[test]
fn geometry_enum_stays_pointer_sized() {
    assert_eq!(std::mem::size_of::<Geometry>(), 16);
}
