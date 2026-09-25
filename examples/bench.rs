use session_rust::history::CAPACITY;
use session_rust::session::PURGE_WORK;
use session_rust::Geometry;
use session_rust::Mesh;
use session_rust::Point;
use session_rust::Session;
use session_rust::TreeNode;
use session_rust::Xform;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

const LARGEST: usize = 1_000_000; // The largest flat scene; the others are a tenth and a thousandth of it.
const SCALE: f64 = 1.0; // Slack on bulk and slice budgets for a slower kernel.
const WARMUP: usize = 5; // Untimed runs before the timed ones.
const RUNS: usize = 101; // Timed runs; their median is the cost.
const PER_OBJECT: f64 = 0.005; // Milliseconds each object of a bulk step may cost.

/// Return whether cpu0 runs at its full clock, so absolute budgets mean something.
fn unthrottled() -> bool {
    let read = |name: &str| {
        std::fs::read_to_string(format!("/sys/devices/system/cpu/cpu0/cpufreq/{name}"))
            .map(|text| text.trim().to_string())
    };

    matches!((read("scaling_max_freq"), read("cpuinfo_max_freq")), (Ok(a), Ok(b)) if a == b)
}

/// Return the milliseconds one call of f takes.
fn clock(f: impl FnOnce()) -> f64 {
    let start = Instant::now();
    f();

    start.elapsed().as_secs_f64() * 1e3
}

/// Return the median of the timings.
fn median(times: &[f64]) -> f64 {
    let mut sorted = times.to_vec();
    sorted.sort_by(f64::total_cmp);

    sorted[sorted.len() / 2]
}

/// Print one budget line.
fn verdict(name: &str, pass: bool) {
    println!("{name:<20} {}", if pass { "ok" } else { "OVER BUDGET" });
}

/// A session with n points in one flat group, and their guids.
fn flat(n: usize) -> (Session, Rc<RefCell<TreeNode>>, Vec<String>) {
    let mut session = Session::default();
    let group = session.add_group("flat");
    let guids = (0..n)
        .map(|i| {
            session
                .add_point(Point::new(i as f64, 0.0, 0.0), Some(&group))
                .borrow()
                .name
                .clone()
        })
        .collect();

    (session, group, guids)
}

/// A quad grid mesh of n by n vertices.
fn grid_mesh(n: usize) -> Mesh {
    let vertices = (0..n * n)
        .map(|at| Point::new((at / n) as f64, (at % n) as f64, 0.0))
        .collect();
    let faces = (0..(n - 1) * (n - 1))
        .map(|at| at / (n - 1) * n + at % (n - 1))
        .map(|at| vec![at, at + n, at + n + 1, at + 1])
        .collect();

    Mesh::from_vertices_and_faces(vertices, faces)
}

/// Median of remove, undo, redo, replace, move and add for each flat size: must not grow with the size.
fn edit_latency(sizes: [usize; 3]) {
    let mut medians = Vec::new();

    for n in sizes {
        let (mut session, group, guids) = flat(n);
        let mut times = vec![Vec::new(); 6];

        for run in 0..WARMUP + RUNS {
            let guid = &guids[run];
            let other = &guids[n - 1 - run];
            let point = Geometry::Point(Rc::new(Point::new(run as f64, 1.0, 0.0)));
            let lap = [
                clock(|| {
                    session.begin("remove");
                    session.remove_object(guid);
                    session.commit();
                }),
                clock(|| {
                    session.undo();
                }),
                clock(|| {
                    session.redo();
                }),
                clock(|| {
                    session.begin("replace");
                    session.replace(other, point);
                    session.commit();
                }),
                clock(|| {
                    session.begin("move");
                    session.set_xform(other, Xform::translation(run as f64, 0.0, 0.0));
                    session.commit();
                }),
                clock(|| {
                    session.begin("add");
                    session.add_point(Point::new(run as f64, 2.0, 0.0), Some(&group));
                    session.commit();
                }),
            ];

            for (k, time) in lap.into_iter().enumerate().filter(|_| run >= WARMUP) {
                times[k].push(time);
            }
        }

        let row: Vec<f64> = times.iter().map(|laps| median(laps)).collect();
        println!(
            "edit latency n={n:<8} remove {:.4} undo {:.4} redo {:.4} replace {:.4} move {:.4} add {:.4} ms",
            row[0], row[1], row[2], row[3], row[4], row[5]
        );
        medians.push(row);
    }

    let fast = medians.iter().flatten().all(|time| *time < 1.0);
    let level = (0..6).all(|k| medians[2][k] < 3.0 * medians[0][k] + 0.05);
    verdict("edit latency fast", !unthrottled() || fast);
    verdict("edit latency level", level);
}

/// A bulk remove of `bulk` objects with its undo and redo: under PER_OBJECT each, level across sizes.
fn bulk_undo(n: usize, bulk: usize) {
    let mut medians = Vec::new();

    for n in [n, n / 5] {
        let (mut session, _, guids) = flat(n);
        let mut times = vec![Vec::new(); 3];

        for run in 0..WARMUP + RUNS {
            let lap = [
                clock(|| {
                    session.begin("remove");

                    for guid in &guids[..bulk] {
                        session.remove_object(guid);
                    }

                    session.commit();
                }),
                clock(|| {
                    session.undo();
                }),
                clock(|| {
                    session.redo();
                }),
            ];
            session.undo();

            for (k, time) in lap.into_iter().enumerate().filter(|_| run >= WARMUP) {
                times[k].push(time);
            }
        }

        let row: Vec<f64> = times.iter().map(|laps| median(laps)).collect();
        println!(
            "bulk undo n={n:<8} remove {:.2} undo {:.2} redo {:.2} ms for {bulk} objects",
            row[0], row[1], row[2]
        );
        medians.push(row);
    }

    let budget = PER_OBJECT * bulk as f64 * SCALE;
    let over = medians[0].iter().all(|time| *time < budget);
    let level = (0..3).all(|k| medians[0][k] < 1.5 * medians[1][k]);
    println!("bulk budget {budget:.1} ms");
    verdict("bulk undo budget", !unthrottled() || over);
    verdict("bulk undo level", level);
}

/// Purge and checkpoint slices after a dropped bulk remove: every slice under a frame.
fn no_pauses(n: usize, bulk: usize) {
    let (mut session, _, guids) = flat(n);
    session.begin("remove");

    for guid in &guids[..bulk] {
        session.remove_object(guid);
    }

    session.commit();

    for step in 0..CAPACITY {
        session.begin("move");
        session.set_xform(&guids[n - 1], Xform::translation(step as f64, 0.0, 0.0));
        session.commit();
    }

    let mut purges = Vec::new();
    let mut purging = true;

    while purging {
        purges.push(clock(|| purging = session.purge_step(PURGE_WORK)));
    }

    let mut writes = Vec::new();
    let mut bytes = None;

    while bytes.is_none() {
        writes.push(clock(|| bytes = session.checkpoint(PURGE_WORK)));
    }

    let sliced = purges.iter().chain(&writes).all(|time| *time < 16.0);
    println!(
        "no pauses n={n:<8} {} purge slices median {:.3} ms max {:.3} ms, {} write slices max {:.3} ms",
        purges.len(),
        median(&purges),
        purges.iter().cloned().fold(0.0, f64::max),
        writes.len(),
        writes.iter().cloned().fold(0.0, f64::max)
    );
    verdict("slices under 16 ms", !unthrottled() || sliced);
    verdict(
        "purge slice median",
        !unthrottled() || median(&purges) < 2.0 * SCALE,
    );
}

/// Edit and undo cost after 10k remove/undo/redo/add cycles with idle purging: level with the first hundred.
fn steady_state(n: usize) {
    let cycles = 10_000.min(n);
    let (mut session, group, guids) = flat(n);
    let mut edits = Vec::new();
    let mut undos = Vec::new();

    for (cycle, guid) in guids.iter().enumerate().take(cycles) {
        edits.push(clock(|| {
            session.begin("remove");
            session.remove_object(guid);
            session.commit();
        }));
        undos.push(clock(|| {
            session.undo();
        }));
        session.redo();
        session.begin("add");
        session.add_point(Point::new(cycle as f64, 1.0, 0.0), Some(&group));
        session.commit();
        session.purge_step(PURGE_WORK);
    }

    let late = cycles - 100;
    println!(
        "steady state n={n:<8} edit {:.4} -> {:.4} ms, undo {:.4} -> {:.4} ms, {} dead, {} bytes",
        median(&edits[..100]),
        median(&edits[late..]),
        median(&undos[..100]),
        median(&undos[late..]),
        session.number_of_dead(),
        session.history.bytes
    );
    verdict(
        "steady edit level",
        median(&edits[late..]) <= 1.5 * median(&edits[..100]) + 0.05,
    );
    verdict(
        "steady undo level",
        median(&undos[late..]) <= 1.5 * median(&undos[..100]) + 0.05,
    );
}

/// Commit cost of 200 mesh removes under an 8 MiB budget: level once the budget evicts.
fn history_memory() {
    let mut session = Session::default();
    session.history.budget = 8 << 20;
    let guids: Vec<String> = (0..200)
        .map(|_| {
            let mesh = grid_mesh(100);
            let guid = mesh.guid().to_string();
            session.add_mesh(mesh, None);

            guid
        })
        .collect();
    let mut commits = Vec::new();

    for guid in &guids {
        commits.push(clock(|| {
            session.begin("remove");
            session.remove_object(guid);
            session.commit();
        }));
    }

    println!(
        "history memory     commit {:.4} -> {:.4} ms, depth {}, {} bytes",
        median(&commits[..50]),
        median(&commits[150..]),
        session.history.depth(),
        session.history.bytes
    );
    verdict(
        "history commit level",
        median(&commits[150..]) <= 2.0 * median(&commits[..50]),
    );
}

/// Add and remove with a transaction open against the same unrecorded: the record must cost nothing visible.
fn record_cost(n: usize) {
    let (mut session, group, guids) = flat(n);
    let mut plain = [Vec::new(), Vec::new()];
    let mut recorded = [Vec::new(), Vec::new()];

    for run in 0..WARMUP + RUNS {
        let x = run as f64;
        let add = clock(|| {
            session.add_point(Point::new(x, 1.0, 0.0), Some(&group));
        });
        let remove = clock(|| {
            session.remove_object(&guids[run]);
        });
        session.begin("record");
        let add_recorded = clock(|| {
            session.add_point(Point::new(x, 2.0, 0.0), Some(&group));
        });
        let remove_recorded = clock(|| {
            session.remove_object(&guids[WARMUP + RUNS + run]);
        });
        session.commit();

        if run >= WARMUP {
            plain[0].push(add);
            plain[1].push(remove);
            recorded[0].push(add_recorded);
            recorded[1].push(remove_recorded);
        }
    }

    println!(
        "record cost n={n:<8} add {:.4} vs {:.4} ms, remove {:.4} vs {:.4} ms",
        median(&plain[0]),
        median(&recorded[0]),
        median(&plain[1]),
        median(&recorded[1])
    );
    let cheap = (0..2).all(|k| median(&recorded[k]) - median(&plain[k]) < 0.02 * SCALE);
    verdict("record cost", !unthrottled() || cheap);
}

/// Moving 10k nodes between two groups with 1k then 100k unrelated objects: the cost must not follow the unrelated count.
fn layer_move(unrelated_sizes: [usize; 2]) {
    let mut medians = Vec::new();

    for unrelated in unrelated_sizes {
        let mut session = Session::default();
        let a = session.add_group("a");
        let b = session.add_group("b");
        let elsewhere = session.add_group("elsewhere");
        let nodes: Vec<_> = (0..10_000)
            .map(|i| session.add_point(Point::new(i as f64, 0.0, 0.0), Some(&a)))
            .collect();
        let mut times = vec![Vec::new(); 3];

        for i in 0..unrelated {
            session.add_point(Point::new(i as f64, 1.0, 0.0), Some(&elsewhere));
        }

        for run in 0..WARMUP + RUNS {
            let target = if run % 2 == 0 { &b } else { &a };
            let lap = [
                clock(|| {
                    session.begin("move");

                    for node in &nodes {
                        session.add(node, Some(target));
                    }

                    session.commit();
                }),
                clock(|| {
                    session.undo();
                }),
                clock(|| {
                    session.redo();
                }),
            ];

            while session.purge_step(PURGE_WORK) {}

            for (k, time) in lap.into_iter().enumerate().filter(|_| run >= WARMUP) {
                times[k].push(time);
            }
        }

        let row: Vec<f64> = times.iter().map(|laps| median(laps)).collect();
        println!(
            "layer move unrelated={unrelated:<7} move {:.2} undo {:.2} redo {:.2} ms",
            row[0], row[1], row[2]
        );
        medians.push(row);
    }

    let bulk = medians[1].iter().all(|time| *time < 50.0 * SCALE);
    verdict("layer move budget", !unthrottled() || bulk);
    verdict(
        "layer move level",
        (0..3).all(|k| medians[1][k] < 3.0 * medians[0][k]),
    );
}

fn main() {
    let sizes = [LARGEST / 1_000, LARGEST / 10, LARGEST];
    let bulk = LARGEST / 10;
    println!(
        "sizes {sizes:?}, bulk {bulk}, cpu {}",
        if unthrottled() {
            "unthrottled"
        } else {
            "throttled"
        }
    );
    edit_latency(sizes);
    bulk_undo(LARGEST, bulk);
    no_pauses(LARGEST, bulk);
    steady_state(sizes[1]);
    history_memory();
    record_cost(sizes[1]);
    layer_move([sizes[0], sizes[1]]);
}

// Undo/redo cost of the tombstone kernel, printed, never asserted: cargo run --release --example bench
