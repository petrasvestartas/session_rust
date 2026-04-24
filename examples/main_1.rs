use session_rust::{file_obj, Element, Session};
use std::path::PathBuf;
use std::time::Instant;

fn main() {
    let base = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .to_path_buf();
    let t0 = Instant::now();

    // 1. Import + pair polylines
    let polylines = file_obj::read_file_obj_polylines(
        base.join("session_data")
            .join("annen_polylines.obj")
            .to_str()
            .unwrap(),
    )
    .expect("read obj");
    let pairs = file_obj::pair_polylines(&polylines, 500.0);
    let t1 = Instant::now();

    // 2. Create session with elements
    let mut session = Session::new("WoodComplete");
    let g = session.add_group("Elements");
    for (a, b) in &pairs {
        let el = Element::plate_from_top_bottom(
            polylines[*a].get_points(),
            polylines[*b].get_points(),
            &format!("plate_{}", a),
        );
        session.add_element(el, Some(&g));
    }
    let t2 = Instant::now();

    // 3. Compute face-to-face contacts
    session.compute_face_to_face(5.0, 50.0);
    let t3 = Instant::now();

    // 4. Save
    session.pb_dump(
        base.join("session_data")
            .join("WoodComplete.pb")
            .to_str()
            .unwrap(),
    );
    let t4 = Instant::now();

    let ms = |a: Instant, b: Instant| (b - a).as_secs_f64() * 1000.0;
    println!(
        "{} polylines -> {} elements -> {} joints",
        polylines.len(),
        pairs.len(),
        session.graph.number_of_edges()
    );
    println!(
        "  import+pair: {:.0}ms  elements: {:.0}ms  contacts: {:.0}ms  save: {:.0}ms  total: {:.0}ms",
        ms(t0, t1),
        ms(t1, t2),
        ms(t2, t3),
        ms(t3, t4),
        ms(t0, t4)
    );
}
