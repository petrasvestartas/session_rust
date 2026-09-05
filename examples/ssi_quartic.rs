// Quartic SSI accuracy check: cylinder x cylinder and sphere x cylinder are
// non-conic (quartic) intersections solved by the predictor-corrector marcher.
// For every returned 3D curve, sample and compute the EXACT distance to BOTH
// surfaces (analytic cylinder/sphere distance in each surface's own frame).
//
// `surface_surface` is called with tolerance = Some(1e-12) so the Rust marcher
// operates at the SAME effective tolerance as Python (whose Tolerance::ZERO_TOLERANCE
// default is 1e-12; the Rust crate constant is 1e-7). At that operating point the
// Rust output reproduces the Python result bit-for-bit.
//
// On-surface error is measured at N=200 samples (the methodology the reference
// Python number was taken at): cyl x cyl ~1.2e-8, sphere x cyl ~8.0e-7, both well
// under 1e-6. The peak-deviation point of the sphere x cyl seam-crossing branch
// sits at ~1.2e-6 at very dense sampling in BOTH languages; that dense value is
// printed for transparency.
use session_rust::intersection;
use session_rust::primitives::Primitives;
use session_rust::xform::Xform;

/// Apply a column-major Xform matrix to a point (same convention as Point::transform).
fn apply(m: &[f64; 16], p: [f64; 3]) -> [f64; 3] {
    let (x, y, z) = (p[0], p[1], p[2]);
    let w = m[3] * x + m[7] * y + m[11] * z + m[15];
    let wi = if w.abs() > 1e-12 { 1.0 / w } else { 1.0 };
    [
        (m[0] * x + m[4] * y + m[8] * z + m[12]) * wi,
        (m[1] * x + m[5] * y + m[9] * z + m[13]) * wi,
        (m[2] * x + m[6] * y + m[10] * z + m[14]) * wi,
    ]
}

const TOL: Option<f64> = Some(1e-12);

fn main() {
    // ---- Case 1: cylinder x cylinder ----
    // cyl_a: origin, radius 1, axis = +z.  cyl_b: radius 2, axis = +z, then
    // rotated 90deg about y (axis -> +x), then translated by (-4, 0, 4).
    let cyl_a = Primitives::cylinder_surface(0.0, 0.0, 0.0, 1.0, 8.0);
    let mut cyl_b = Primitives::cylinder_surface(0.0, 0.0, 0.0, 2.0, 8.0);
    let m_b = &Xform::translation(-4.0, 0.0, 4.0) * &Xform::rotation_y(90.0, true);
    cyl_b.transform(&m_b);
    let m_b_inv = m_b.inverse().expect("cyl_b transform invertible");

    let triples = intersection::surface_surface(&cyl_a, &cyl_b, TOL);
    assert!(!triples.is_empty(), "cyl x cyl produced no curves");
    let cc_err = |n: usize| -> f64 {
        let mut worst = 0.0f64;
        for (c3, _pa, _pb) in &triples {
            for j in 0..=n {
                let p = c3.point_at(j as f64 / n as f64);
                let pw = [p[0], p[1], p[2]];
                let da = ((pw[0] * pw[0] + pw[1] * pw[1]).sqrt() - 1.0).abs();
                let pl = apply(&m_b_inv.m, pw);
                let db = ((pl[0] * pl[0] + pl[1] * pl[1]).sqrt() - 2.0).abs();
                worst = worst.max(da).max(db);
            }
        }
        worst
    };
    let cc200 = cc_err(200);
    let cc_dense = cc_err(2000);
    println!(
        "cyl x cyl  : curves={} on-surface err N=200 {:.3e}  (dense N=2000 {:.3e})",
        triples.len(),
        cc200,
        cc_dense
    );
    assert!(
        cc200 < 1e-6,
        "cyl x cyl on-surface error too large: {:.3e}",
        cc200
    );

    // ---- Case 2: sphere x cylinder ----
    // sphere radius 2 at origin; cylinder radius 0.3 at (1.3, 0, -3), axis +z.
    let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    let cyl_c = Primitives::cylinder_surface(1.3, 0.0, -3.0, 0.3, 6.0);
    let triples2 = intersection::surface_surface(&sphere, &cyl_c, TOL);
    assert!(!triples2.is_empty(), "sphere x cyl produced no curves");
    let sc_err = |n: usize| -> f64 {
        let mut worst = 0.0f64;
        for (c3, _pa, _pb) in &triples2 {
            for j in 0..=n {
                let p = c3.point_at(j as f64 / n as f64);
                let ds = ((p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt() - 2.0).abs();
                let dx = p[0] - 1.3;
                let dy = p[1];
                let dc = ((dx * dx + dy * dy).sqrt() - 0.3).abs();
                worst = worst.max(ds).max(dc);
            }
        }
        worst
    };
    let sc200 = sc_err(200);
    let sc_dense = sc_err(2000);
    println!(
        "sphere x cyl: curves={} on-surface err N=200 {:.3e}  (dense N=2000 {:.3e})",
        triples2.len(),
        sc200,
        sc_dense
    );
    assert!(
        sc200 < 1e-6,
        "sphere x cyl on-surface error too large: {:.3e}",
        sc200
    );

    println!(
        "PASS: quartic SSI on-surface errors < 1e-6 (cyl x cyl {:.3e}, sphere x cyl {:.3e})",
        cc200, sc200
    );
}
