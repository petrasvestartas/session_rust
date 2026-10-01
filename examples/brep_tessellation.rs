// Tessellation harness: triangle count, min angle, slivers, degenerate triangles, empty faces,
// unmatched boundary edges (watertightness), winding against normals and time per BRep, over the
// kernel primitives, the drill STEP cases and every BRep of a scene given on the command line.
//   cargo run --profile quick --example brep_tessellation -- [angle chord] [scene.pb|case.stp ...]
use session_rust::file_step::read_file_step_breps;
use session_rust::session::Geometry;
use session_rust::{BRep, Mesh, Point, Session};
use std::collections::HashMap;
use std::time::Instant;

#[derive(Default)]
struct Stats {
    faces: usize,
    triangles: usize,
    min_angle: f64,
    slivers: usize,
    degenerate: usize,
    empty_faces: usize,
    open_edges: usize,
    wrong_winding: usize,
    seconds: f64,
}

fn key(p: &Point) -> (u64, u64, u64) {
    ((p[0] + 0.0).to_bits(), (p[1] + 0.0).to_bits(), (p[2] + 0.0).to_bits())
}

fn triangle_angles(a: &Point, b: &Point, c: &Point) -> [f64; 3] {
    let mut out = [0.0; 3];
    let pts = [a, b, c];

    for i in 0..3 {
        let p = pts[i];
        let q = pts[(i + 1) % 3];
        let r = pts[(i + 2) % 3];
        let u = q - p;
        let v = r - p;
        let lu = u.magnitude();
        let lv = v.magnitude();

        if lu == 0.0 || lv == 0.0 {
            return [0.0; 3];
        }

        out[i] = (u.dot(&v) / (lu * lv)).clamp(-1.0, 1.0).acos().to_degrees();
    }

    out
}

fn measure(fms: &[Mesh], stats: &mut Stats, verbose: bool) {
    let mut naked: HashMap<((u64, u64, u64), (u64, u64, u64)), i32> = HashMap::new();
    let mut owner: HashMap<((u64, u64, u64), (u64, u64, u64)), usize> = HashMap::new();
    stats.min_angle = 180.0;

    for (fi, fm) in fms.iter().enumerate() {
        stats.faces += 1;
        let mut face_min = 180.0f64;
        let mut face_tris = 0usize;

        if fm.face.is_empty() {
            stats.empty_faces += 1;
        }

        let mut local: HashMap<((u64, u64, u64), (u64, u64, u64)), i32> = HashMap::new();

        for verts in fm.face.values() {
            let pts: Vec<Point> = verts.iter().map(|v| fm.vertex[v].position()).collect();
            let n = pts.len();

            for i in 0..n {
                let a = key(&pts[i]);
                let b = key(&pts[(i + 1) % n]);
                *local.entry((a, b)).or_default() += 1;
                *local.entry((b, a)).or_default() -= 1;
            }

            for i in 1..n - 1 {
                let (a, b, c) = (&pts[0], &pts[i], &pts[i + 1]);
                stats.triangles += 1;
                let angles = triangle_angles(a, b, c);
                let area = (b - a).cross(&(c - a)).magnitude();
                let min = angles[0].min(angles[1]).min(angles[2]);

                if area == 0.0 || min == 0.0 {
                    stats.degenerate += 1;
                    continue;
                }

                stats.min_angle = stats.min_angle.min(min);
                face_min = face_min.min(min);
                face_tris += 1;

                if min < 1.0 {
                    stats.slivers += 1;
                }

                let fn_ = (b - a).cross(&(c - a));
                let mut agree = 0.0;

                for v in verts {
                    if let Some(nrm) = fm.vertex[v].normal() {
                        agree += fn_[0] * nrm[0] + fn_[1] * nrm[1] + fn_[2] * nrm[2];
                    }
                }

                if agree < 0.0 {
                    stats.wrong_winding += 1;
                }
            }
        }

        for (edge, count) in local {
            if count > 0 {
                *naked.entry(edge).or_default() += count;
                owner.insert(edge, fi);
            }
        }

        if verbose {
            println!("  face {fi:>3} tris {face_tris:>6} min_angle {face_min:>6.2} verts {:>5}", fm.vertex.len());
        }
    }

    for ((a, b), count) in &naked {
        let reverse = naked.get(&(*b, *a)).copied().unwrap_or(0);

        if *count != reverse {
            stats.open_edges += 1;

            if verbose {
                let p = |k: &(u64, u64, u64)| Point::new(f64::from_bits(k.0), f64::from_bits(k.1), f64::from_bits(k.2));
                println!("  open edge face {} {} -> {} ({count} vs {reverse})", owner[&(*a, *b)], p(a), p(b));
            }
        }
    }
}

fn run(label: &str, b: &BRep, quality: (f64, f64), verbose: bool) -> Stats {
    let start = Instant::now();
    let fms = b.face_meshes_q(Some(quality));
    let mut stats = Stats {
        seconds: start.elapsed().as_secs_f64(),
        ..Default::default()
    };
    measure(&fms, &mut stats, verbose);
    println!(
        "{label:<32} faces {:>4} tris {:>7} min_angle {:>6.2} slivers {:>5} degen {:>3} empty {:>2} open {:>4} wind {:>4} {:>8.1} ms",
        stats.faces,
        stats.triangles,
        stats.min_angle,
        stats.slivers,
        stats.degenerate,
        stats.empty_faces,
        stats.open_edges,
        stats.wrong_winding,
        stats.seconds * 1e3
    );

    stats
}

fn scene_breps(path: &str) -> Vec<(String, BRep)> {
    let s = Session::pb_load(path).unwrap();
    let mut out = Vec::new();

    for guid in s.order() {
        let Some(g) = s.get_object(&guid) else {
            continue;
        };

        match g {
            Geometry::BRep(b) => out.push((b.name.clone(), (**b).clone())),
            Geometry::Element(e) => {
                let b = e.geometry_brep();

                if b.face_count() > 0 {
                    out.push((e.name.clone(), b.clone()));
                }
            }
            _ => {}
        }
    }

    out
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut quality = (5.0, 0.001);
    let mut files: Vec<String> = Vec::new();
    let verbose = args.iter().any(|a| a == "-v");

    for a in &args {
        if a.ends_with(".pb") || a.ends_with(".stp") || a.ends_with(".step") {
            files.push(a.clone());
        }
    }

    let numbers: Vec<f64> = args.iter().filter_map(|a| a.parse().ok()).collect();

    if numbers.len() >= 2 {
        quality = (numbers[0], numbers[1]);
    }

    println!("quality angle {} chord {}", quality.0, quality.1);
    let cases: Vec<(&str, BRep)> = vec![
        ("box", BRep::create_box(8.0, 6.0, 4.0)),
        ("cylinder", BRep::create_cylinder(2.0, 5.0)),
        ("sphere", BRep::create_sphere(2.0)),
        ("cone", BRep::create_cone(2.0, 5.0)),
        ("torus", BRep::create_torus(4.0, 1.0)),
        ("block_with_hole", BRep::create_block_with_hole(8.0, 6.0, 4.0, 1.5)),
    ];

    for (label, b) in &cases {
        run(label, b, quality, verbose);
    }

    let mut total = Stats::default();
    total.min_angle = 180.0;

    for f in &files {
        let breps: Vec<(String, BRep)> = if f.ends_with(".pb") {
            scene_breps(f)
        } else {
            read_file_step_breps(f)
                .into_iter()
                .enumerate()
                .map(|(i, b)| (format!("{}#{i}", f.rsplit('/').next().unwrap_or(f)), b))
                .collect()
        };

        for (name, b) in &breps {
            let s = run(name, b, quality, verbose);
            total.faces += s.faces;
            total.triangles += s.triangles;
            total.min_angle = total.min_angle.min(s.min_angle);
            total.slivers += s.slivers;
            total.degenerate += s.degenerate;
            total.empty_faces += s.empty_faces;
            total.open_edges += s.open_edges;
            total.wrong_winding += s.wrong_winding;
            total.seconds += s.seconds;
        }
    }

    if !files.is_empty() {
        println!(
            "{:<32} faces {:>4} tris {:>7} min_angle {:>6.2} slivers {:>5} degen {:>3} empty {:>2} open {:>4} wind {:>4} {:>8.1} ms",
            "TOTAL(files)",
            total.faces,
            total.triangles,
            total.min_angle,
            total.slivers,
            total.degenerate,
            total.empty_faces,
            total.open_edges,
            total.wrong_winding,
            total.seconds * 1e3
        );
    }
}
