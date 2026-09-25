use crate::history::CAPACITY;
use crate::mini_test::TestResult;
use crate::session::PURGE_WORK;
use crate::tree::TreeNode;
use crate::Mesh;
use crate::Point;
use crate::Session;
use crate::MINI_CHECK;
use crate::MINI_TEST;
use crate::REGISTER_MINI_TEST;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

const SIZES: [usize; 3] = [1_000, 100_000, 1_000_000]; // Flat scene sizes the latency must not grow with.
const SCALE: f64 = 1.0; // Slack on bulk and slice budgets for a slower kernel.
const WARMUP: usize = 5; // Untimed runs before the timed ones.
const RUNS: usize = 101; // Timed runs; their median is the cost.
const BULK: usize = 100_000; // Objects one bulk transaction removes.

/// Return whether SESSION_BENCH=1 asks for the benchmarks.
fn bench_enabled() -> bool {
    std::env::var("SESSION_BENCH").is_ok_and(|value| value == "1")
}

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

pub fn run_bench_edit_latency() -> TestResult {
    MINI_TEST!("Edit Latency", {
        use crate::Geometry;
        use crate::Xform;

        if !bench_enabled() {
            return Ok(());
        }

        let mut medians = Vec::new();

        for n in SIZES {
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

            medians.push(times.iter().map(|laps| median(laps)).collect::<Vec<f64>>());
        }

        let fast = medians.iter().flatten().all(|time| *time < 1.0);
        let level = (0..6).all(|k| medians[2][k] < 3.0 * medians[0][k] + 0.05);

        MINI_CHECK!(!unthrottled() || fast);
        MINI_CHECK!(level);
    })
}

pub fn run_bench_bulk_undo() -> TestResult {
    MINI_TEST!("Bulk Undo", {
        if !bench_enabled() {
            return Ok(());
        }

        let mut medians = Vec::new();

        for n in [SIZES[2], 200_000] {
            let (mut session, _, guids) = flat(n);
            let mut times = vec![Vec::new(); 3];

            for run in 0..WARMUP + RUNS {
                let lap = [
                    clock(|| {
                        session.begin("remove");

                        for guid in &guids[..BULK] {
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

            medians.push(times.iter().map(|laps| median(laps)).collect::<Vec<f64>>());
        }

        let bulk = medians[0].iter().all(|time| *time < 50.0 * SCALE);

        MINI_CHECK!(medians[0][1] < 1.5 * medians[1][1]);
        MINI_CHECK!(!unthrottled() || bulk);
    })
}

pub fn run_bench_no_pauses() -> TestResult {
    MINI_TEST!("No Pauses", {
        use crate::Xform;
        use prost::Message;

        if !bench_enabled() {
            return Ok(());
        }

        let n = SIZES[2];
        let (mut session, _, guids) = flat(n);

        session.begin("remove");

        for guid in &guids[..BULK] {
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

        let bytes = bytes.unwrap();
        let loaded = Session::pb_loads(&bytes).unwrap();
        let sliced = purges.iter().chain(&writes).all(|time| *time < 16.0);

        MINI_CHECK!(purges.len() > 1);
        MINI_CHECK!(!unthrottled() || sliced);
        MINI_CHECK!(!unthrottled() || median(&purges) < 2.0 * SCALE);
        MINI_CHECK!(loaded.objects.points.len() == n - BULK);
        MINI_CHECK!(bytes == session.to_proto().encode_to_vec());
        MINI_CHECK!(session.number_of_dead() == 0);
    })
}

pub fn run_bench_steady_state() -> TestResult {
    MINI_TEST!("Steady State", {
        if !bench_enabled() {
            return Ok(());
        }

        let n = SIZES[1];
        let cycles = 10_000;
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

        let bound = 2 * CAPACITY + 2 * (n / PURGE_WORK + 1);
        let late = cycles - 100;
        let points = &session.objects.points;

        MINI_CHECK!(median(&edits[late..]) <= 1.5 * median(&edits[..100]) + 0.05);
        MINI_CHECK!(median(&undos[late..]) <= 1.5 * median(&undos[..100]) + 0.05);
        MINI_CHECK!(session.history.bytes <= session.history.budget);
        MINI_CHECK!(session.number_of_dead() <= bound);
        MINI_CHECK!(points.number_of_slots() <= points.len() + bound);
    })
}

pub fn run_bench_history_memory() -> TestResult {
    MINI_TEST!("History Memory", {
        if !bench_enabled() {
            return Ok(());
        }

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
        let mut bounded = true;

        for guid in &guids {
            commits.push(clock(|| {
                session.begin("remove");
                session.remove_object(guid);
                session.commit();
            }));
            let newest = session.history.undo_stack[session.history.depth() - 1].bytes;
            bounded &= session.history.bytes <= session.history.budget + newest;
        }

        MINI_CHECK!(bounded);
        MINI_CHECK!(session.history.depth() < CAPACITY);
        MINI_CHECK!(median(&commits[150..]) <= 2.0 * median(&commits[..50]));
    })
}

pub fn run_bench_record_cost() -> TestResult {
    MINI_TEST!("Record Cost", {
        if !bench_enabled() {
            return Ok(());
        }

        let n = SIZES[1];
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

        let cheap = (0..2).all(|k| median(&recorded[k]) - median(&plain[k]) < 0.02 * SCALE);
        let slot = session.objects.points.get_slot(&guids[n - 1]).unwrap();
        let original = Rc::clone(session.objects.points.get_item(slot));

        session.begin("remove");
        session.remove_object(&guids[n - 1]);
        session.commit();

        MINI_CHECK!(!unthrottled() || cheap);
        MINI_CHECK!(Rc::ptr_eq(session.objects.points.get_item(slot), &original));
    })
}

pub fn run_bench_layer_move() -> TestResult {
    MINI_TEST!("Layer Move", {
        if !bench_enabled() {
            return Ok(());
        }

        let mut medians = Vec::new();

        for unrelated in [1_000, 100_000] {
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

            medians.push(times.iter().map(|laps| median(laps)).collect::<Vec<f64>>());
        }

        let bulk = medians[1].iter().all(|time| *time < 50.0 * SCALE);

        MINI_CHECK!(!unthrottled() || bulk);
        MINI_CHECK!((0..3).all(|k| medians[1][k] < 3.0 * medians[0][k]));
    })
}

REGISTER_MINI_TEST!(
    "Bench",
    "Edit Latency",
    crate::bench_test::run_bench_edit_latency
);
REGISTER_MINI_TEST!("Bench", "Bulk Undo", crate::bench_test::run_bench_bulk_undo);
REGISTER_MINI_TEST!("Bench", "No Pauses", crate::bench_test::run_bench_no_pauses);
REGISTER_MINI_TEST!(
    "Bench",
    "Steady State",
    crate::bench_test::run_bench_steady_state
);
REGISTER_MINI_TEST!(
    "Bench",
    "History Memory",
    crate::bench_test::run_bench_history_memory
);
REGISTER_MINI_TEST!(
    "Bench",
    "Record Cost",
    crate::bench_test::run_bench_record_cost
);
REGISTER_MINI_TEST!(
    "Bench",
    "Layer Move",
    crate::bench_test::run_bench_layer_move
);
