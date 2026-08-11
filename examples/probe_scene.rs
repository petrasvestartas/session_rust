// Throwaway diagnostic: what is actually inside a .pb — per-type counts, world bounds, whether
// xforms survived the round-trip, and the colour histogram of the linework.
use session_rust::{Session, Point, Xform};
use session_rust::session::Geometry;

fn main() {
    for path in std::env::args().skip(1) {
        let s = Session::pb_load(&path);
        let mut counts = std::collections::BTreeMap::<&str, usize>::new();
        let mut colors = std::collections::BTreeMap::<String, usize>::new();
        let mut widths = std::collections::BTreeMap::<String, usize>::new();
        let mut identity = 0usize;
        let mut moved = 0usize;
        let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
        let id = Xform::identity();

        for guid in s.order() {
            let Some(g) = s.get_object(&guid) else { continue };
            let (kind, xf, pts): (&str, &Xform, Vec<Point>) = match g {
                Geometry::Point(p) => ("point", &session.world_xform(&guid), vec![Point::new(p[0], p[1], p[2])]),
                Geometry::Line(l) => {
                    *colors.entry(format!("{:?}", l.linecolor.to_f32())).or_default() += 1;
                    *widths.entry(format!("{:.2}", l.width)).or_default() += 1;
                    ("line", &session.world_xform(&guid), vec![l.start(), l.end()])
                }
                Geometry::Polyline(pl) => {
                    *colors.entry(format!("{:?}", pl.linecolor.to_f32())).or_default() += 1;
                    *widths.entry(format!("{:.2}", pl.width)).or_default() += 1;
                    ("polyline", &session.world_xform(&guid), pl.get_points())
                }
                Geometry::PointCloud(pc) => ("pointcloud", &session.world_xform(&guid), pc.get_points()),
                Geometry::Mesh(m) => ("mesh", &session.world_xform(&guid), m.vertex.values().map(|v| Point::new(v.x, v.y, v.z)).collect()),
                Geometry::NurbsCurve(c) => ("nurbscurve", &session.world_xform(&guid), vec![c.point_at_start(), c.point_at_end()]),
                Geometry::NurbsSurface(sf) => ("nurbssurface", &session.world_xform(&guid), vec![]),
                Geometry::BRep(b) => ("brep", &session.world_xform(&guid), vec![]),
                Geometry::Plane(p) => ("plane", &session.world_xform(&guid), vec![p.origin()]),
                Geometry::OBB(o) => ("obb", &session.world_xform(&guid), vec![]),
                Geometry::Element(_) => ("element", &id, vec![]),
            };
            *counts.entry(kind).or_default() += 1;
            if xf.m == id.m { identity += 1 } else { moved += 1 }
            for p in pts {
                let w = xf.transform_point(&p);
                for k in 0..3 { lo[k] = lo[k].min(w[k]); hi[k] = hi[k].max(w[k]); }
            }
        }

        println!("\n=== {path}  ({} objects, name '{}')", s.lookup.len(), s.name);
        println!("  types    {counts:?}");
        println!("  xform    identity={identity} moved={moved}");
        println!("  bounds   x {:.0}..{:.0}  y {:.0}..{:.0}  z {:.0}..{:.0}", lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
        let mut c: Vec<_> = colors.into_iter().collect();
        c.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        println!("  colors   {:?}", &c[..c.len().min(6)]);
        let mut w: Vec<_> = widths.into_iter().collect();
        w.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
        println!("  widths   {:?}", &w[..w.len().min(6)]);
    }
}
