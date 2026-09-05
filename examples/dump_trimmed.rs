// Dump the EXACT demo trimmed-surface cases (as the viewer builds them) to OBJ + stats,
// so the actual Rust mesh output can be inspected. Throwaway diagnostic.
use session_rust::mesh::Mesh;
use session_rust::nurbscurve::NurbsCurve;
use session_rust::nurbssurface::NurbsSurface;
use session_rust::nurbssurface_trimmed::NurbsSurfaceTrimmed;
use session_rust::point::Point;
use session_rust::primitives::Primitives;
use std::io::Write;

fn surf_g(srf: &NurbsSurface, u: f32, v: f32, q0: [f32; 3], n: [f32; 3]) -> f32 {
    let p = srf
        .point_at(u as f64, v as f64)
        .unwrap_or(Point::new(0.0, 0.0, 0.0));
    (p[0] as f32 - q0[0]) * n[0] + (p[1] as f32 - q0[1]) * n[1] + (p[2] as f32 - q0[2]) * n[2]
}
fn plane_v(srf: &NurbsSurface, u: f32, v0: f32, v1: f32, q0: [f32; 3], n: [f32; 3]) -> f32 {
    let f = |v: f32| surf_g(srf, u, v, q0, n);
    let (f0, f1) = (f(v0), f(v1));
    if (f0 <= 0.0) == (f1 <= 0.0) {
        return if f0 <= 0.0 { v1 } else { v0 };
    }
    let (mut a, mut b, mut fa) = (v0, v1, f0);
    for _ in 0..30 {
        let m = (a + b) * 0.5;
        let fm = f(m);
        if (fm <= 0.0) == (fa <= 0.0) {
            a = m;
            fa = fm;
        } else {
            b = m;
        }
    }
    (a + b) * 0.5
}

fn cyl_band(
    srf: &NurbsSurface,
    vlo_of: &dyn Fn(f32) -> f32,
    vhi_of: &dyn Fn(f32) -> f32,
) -> NurbsSurfaceTrimmed {
    let (u0, u1) = srf.domain(0).unwrap();
    let nu = 96;
    let mut lp: Vec<Point> = Vec::new();
    for i in 0..=nu {
        let u = u0 + (u1 - u0) * i as f64 / nu as f64;
        lp.push(Point::new(u, vlo_of(u as f32) as f64, 0.0));
    }
    for i in (0..=nu).rev() {
        let u = u0 + (u1 - u0) * i as f64 / nu as f64;
        lp.push(Point::new(u, vhi_of(u as f32) as f64, 0.0));
    }
    let mut ts = NurbsSurfaceTrimmed::new();
    ts.m_surface = srf.clone();
    ts.m_outer_loop = Some(NurbsCurve::create(true, 1, &lp));
    ts
}

fn write_obj(name: &str, m: &Mesh) {
    let mut keys: Vec<usize> = m.vertex.keys().copied().collect();
    keys.sort();
    let mut idx = std::collections::HashMap::new();
    let mut f = std::fs::File::create(format!("../{name}.obj")).unwrap();
    for (i, &vk) in keys.iter().enumerate() {
        idx.insert(vk, i + 1);
        let p = m.vertex_point(vk).unwrap();
        writeln!(f, "v {} {} {}", p[0], p[1], p[2]).unwrap();
    }
    for fk in m.faces() {
        if let Some(vs) = m.face_vertices(fk) {
            let s: Vec<String> = vs.iter().map(|vk| idx[vk].to_string()).collect();
            writeln!(f, "f {}", s.join(" ")).unwrap();
        }
    }
    let naked = m.naked_edges(true).len();
    println!(
        "{name}: v={} f={} naked_edges={}",
        m.number_of_vertices(),
        m.number_of_faces(),
        naked
    );
}

fn flat_srf(cx: f32, cy: f32, r: f32) -> NurbsSurface {
    let mut srf = NurbsSurface::create_raw(3, false, 2, 2, 2, 2, false, false, 1.0, 1.0).unwrap();
    srf.set_cv(0, 0, &Point::new((cx - r) as f64, (cy - r) as f64, 0.0));
    srf.set_cv(1, 0, &Point::new((cx + r) as f64, (cy - r) as f64, 0.0));
    srf.set_cv(0, 1, &Point::new((cx - r) as f64, (cy + r) as f64, 0.0));
    srf.set_cv(1, 1, &Point::new((cx + r) as f64, (cy + r) as f64, 0.0));
    srf
}
fn uv_circle() -> NurbsCurve {
    let cw = (2.0_f32).sqrt() / 2.0;
    let ccx = [1.0f32, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0];
    let ccy = [0.0f32, 1.0, 1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 0.0];
    let cwt = [1.0f32, cw, 1.0, cw, 1.0, cw, 1.0, cw, 1.0];
    let mut c = NurbsCurve::new(3, true, 3, 9);
    c.m_nurbsknot = vec![0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0];
    for i in 0..9 {
        c.set_cv_4d(
            i,
            ((0.5 + 0.5 * ccx[i]) * cwt[i]) as f64,
            ((0.5 + 0.5 * ccy[i]) * cwt[i]) as f64,
            0.0,
            cwt[i] as f64,
        );
    }
    c
}

fn main() {
    let p5_y = 0.0f32;
    // Row 5 case 1: cylinder cut by tilted plane in X
    let cyl = Primitives::cylinder_surface(0.0, p5_y as f64, 0.0, 400.0, 900.0);
    let (cv0, cv1) = cyl.domain(1).unwrap();
    let (q0, n) = ([0.0f32, p5_y, 450.0], [0.8f32, 0.0, 1.0]);
    let cc = cyl.clone();
    let ts = cyl_band(&cyl, &move |_u| cv0 as f32, &move |u| {
        plane_v(&cc, u, cv0 as f32, cv1 as f32, q0, n)
    });
    write_obj("dump_cyl_cut", &ts.mesh_q(10.0, 0.003));

    // Row 2: flat circle
    let mut tsc = NurbsSurfaceTrimmed::new();
    tsc.m_surface = flat_srf(0.0, 0.0, 700.0);
    tsc.m_outer_loop = Some(uv_circle());
    write_obj("dump_flat_circle", &tsc.mesh_q(10.0, 0.003));
}
