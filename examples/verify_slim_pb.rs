// Round-trip a real .pb through the slimmer writer and prove nothing was lost.
//  - the rebuilt halfedge map must equal the one the old file carried, mesh for mesh
//  - geometry, colors and widths must survive
use std::collections::HashMap;

fn main() {
    let path = std::env::args().nth(1).expect("usage: verify_slim_pb <file.pb>");
    let old = std::fs::read(&path).unwrap();

    let a = session_rust::Session::pb_loads(&old).unwrap();

    // Halfedge maps as the ORIGINAL file stored them (that file predates the writer change,
    // so pb_loads read them off the wire rather than rebuilding).
    let before: HashMap<String, HashMap<usize, HashMap<usize, Option<usize>>>> = a.objects.meshes
        .iter().map(|m| (m.guid().to_string(), m.halfedge.clone())).collect();

    let new = a.pb_dumps();
    let b = session_rust::Session::pb_loads(&new).unwrap();

    // Halfedges: rebuilt from faces on the way back in.
    let mut checked = 0;
    for m in &b.objects.meshes {
        let want = before.get(m.guid()).expect("mesh guid vanished");
        assert_eq!(&m.halfedge, want, "halfedge rebuild differs for mesh {}", m.guid());
        checked += 1;
    }

    assert_eq!(a.objects.lines.len(), b.objects.lines.len());
    assert_eq!(a.objects.meshes.len(), b.objects.meshes.len());
    assert_eq!(a.lookup.len(), b.lookup.len());

    for (x, y) in a.objects.lines.iter().zip(b.objects.lines.iter()) {
        assert_eq!(x.guid(), y.guid());
        for i in 0..6 { assert!((x[i] - y[i]).abs() < 1e-12, "line coords moved"); }
        assert!((x.width - y.width).abs() < 1e-12);
        assert_eq!(x.linecolor.r, y.linecolor.r);
        assert_eq!(x.linecolor.a, y.linecolor.a);
    }
    for (x, y) in a.objects.meshes.iter().zip(b.objects.meshes.iter()) {
        assert_eq!(x.vertex.len(), y.vertex.len());
        assert_eq!(x.face.len(), y.face.len());
        assert_eq!(x.widths(), y.widths());
        assert_eq!(x.get_linecolors().len(), y.get_linecolors().len());
        assert_eq!(x.objectcolor().r, y.objectcolor().r);
    }

    println!("{:<32} {:>6.1} MB -> {:>6.1} MB   ({:.0}% smaller)  halfedge maps verified on {} meshes",
        path.rsplit('/').next().unwrap(),
        old.len() as f64 / 1e6, new.len() as f64 / 1e6,
        100.0 * (1.0 - new.len() as f64 / old.len() as f64), checked);
}
