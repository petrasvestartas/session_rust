// Sanity check: analytic plane x sphere is geometrically exact in Rust.
use session_rust::intersection;
use session_rust::NurbsSurface;
use session_rust::Point;
use session_rust::primitives::Primitives;

fn main() {
    // Sphere of radius 2 centered at origin, flat plane patch at z=0.4.
    // Section circle radius = sqrt(4 - 0.16) = sqrt(3.84) = 1.959591794...
    let sphere = Primitives::sphere_surface(0.0, 0.0, 0.0, 2.0);
    let flat = NurbsSurface::create(false, false, 1, 1, 2, 2, &[
        Point::new(-3.0, -3.0, 0.4),
        Point::new(-3.0, 3.0, 0.4),
        Point::new(3.0, -3.0, 0.4),
        Point::new(3.0, 3.0, 0.4),
    ]).unwrap();

    let triples = intersection::surface_surface(&flat, &sphere, None);
    assert_eq!(triples.len(), 1, "expected exactly one analytic curve");
    let (c3, _pa, _pb) = &triples[0];
    assert!(c3.is_valid() && c3.is_closed());

    let expected = 3.84_f64.sqrt();
    let mut worst_r = 0.0f64;
    let mut worst_z = 0.0f64;
    for j in 0..=256 {
        let p = c3.point_at(j as f64 / 256.0);
        let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
        worst_r = worst_r.max((r - expected).abs());
        worst_z = worst_z.max((p[2] - 0.4).abs());
    }
    println!("expected radius = {:.15}", expected);
    println!("max |hypot(x,y) - r| = {:.3e}", worst_r);
    println!("max |z - 0.4|        = {:.3e}", worst_z);
    assert!(worst_r < 1e-9, "radius deviation too large: {:.3e}", worst_r);
    assert!(worst_z < 1e-9, "z deviation too large: {:.3e}", worst_z);
    println!("PASS: analytic plane x sphere is exact (< 1e-9)");
}
