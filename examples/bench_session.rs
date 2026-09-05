// Session datastructure benchmark (SESSION_DATASTRUCTURE_PLAN.md).
// Run: cargo run --release --example bench_session
// Baseline 2026-08-01 pre-P1:  add 3161ms, +2530MB, pb_loads 2819ms, clone 1503ms
// After P1 (boxed Geometry):   add  862ms,  +768MB, pb_loads  712ms, clone  613ms
// After P4 (shared Rc) clone is COW-shallow on geometry (tree/graph still deep).

use session_rust::{Geometry, Line, Session};
use std::time::Instant;

fn rss_mb() -> f64 {
    let s = std::fs::read_to_string("/proc/self/status").unwrap_or_default();
    s.lines()
        .find(|l| l.starts_with("VmRSS:"))
        .and_then(|l| l.split_whitespace().nth(1))
        .and_then(|v| v.parse::<f64>().ok())
        .map(|kb| kb / 1024.0)
        .unwrap_or(0.0)
}

fn main() {
    const N: usize = 200_000;
    println!("size_of Geometry = {} B", std::mem::size_of::<Geometry>());
    let m0 = rss_mb();
    let t = Instant::now();
    let mut s = Session::new("bench");
    for i in 0..N {
        let l = Line::new(i as f64, 0.0, 0.0, i as f64, 1.0, 0.0);
        s.add_line(l, None);
    }
    println!(
        "add_line x{}k   {:>7.0} ms   rss +{:.0} MB",
        N / 1000,
        t.elapsed().as_secs_f64() * 1e3,
        rss_mb() - m0
    );
    let t = Instant::now();
    let mut acc = 0.0;
    for g in s.lookup.values() {
        if let Geometry::Line(l) = g {
            acc += l.start()[0];
        }
    }
    println!(
        "iterate lookup  {:>7.1} ms   (acc {acc})",
        t.elapsed().as_secs_f64() * 1e3
    );
    let t = Instant::now();
    let bytes = s.pb_dumps();
    println!(
        "pb_dumps        {:>7.0} ms   ({:.1} MB)",
        t.elapsed().as_secs_f64() * 1e3,
        bytes.len() as f64 / 1e6
    );
    let t = Instant::now();
    let s2 = Session::pb_loads(&bytes).unwrap();
    println!(
        "pb_loads        {:>7.0} ms   ({} objects)",
        t.elapsed().as_secs_f64() * 1e3,
        s2.lookup.len()
    );
    let t = Instant::now();
    let s3 = s.clone();
    println!(
        "clone           {:>7.0} ms   ({} objects)",
        t.elapsed().as_secs_f64() * 1e3,
        s3.lookup.len()
    );
    println!("final rss       {:>7.0} MB", rss_mb());
}
