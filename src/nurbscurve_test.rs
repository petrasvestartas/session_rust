#[cfg(test)]
mod tests {
    use crate::{nurbscurve::NurbsCurve, point::Point, vector::Vector};
    use std::f64::consts::PI;

    #[test]
    fn test_nurbscurve_frames_3d() {
        // Build a clearly 3D curve (wavy helix)
        let mut ctrl: Vec<Point> = Vec::new();
        for k in 0..8 {
            let t = (k as f64) / 7.0 * 2.0 * PI;
            let r = 1.5 + 0.3 * (3.0 * t).cos();
            let x = r * t.cos();
            let y = r * t.sin();
            let z = 0.6 * t;
            ctrl.push(Point::new(x, y, z));
        }

        let crv = NurbsCurve::create(false, 3, &ctrl).expect("curve create");
        let (t0, t1) = crv.domain();
        let t = 0.5 * (t0 + t1);

        // Normal plane (plane normal = tangent)
        let T = crv.tangent_at(t);
        assert!((T.compute_length() - 1.0).abs() < 1e-6);
        let fallback = if T.z() .abs() < 0.9 { Vector::new(0.0, 0.0, 1.0) } else { Vector::new(0.0, 1.0, 0.0) };
        let e1 = T.cross(&fallback).normalize();
        let e2 = T.cross(&e1).normalize();
        assert!((e1.compute_length() - 1.0).abs() < 1e-6);
        assert!((e2.compute_length() - 1.0).abs() < 1e-6);
        assert!(e1.dot(&T).abs() < 1e-6);
        assert!(e2.dot(&T).abs() < 1e-6);
        assert!(e1.dot(&e2).abs() < 1e-6);

        // Frenet frame (T, N, B)
        let ders = crv.evaluate(t, 2);
        assert!(ders.len() >= 3);
        let d1 = ders[1].clone();
        let d2 = ders[2].clone();
        let T_f = d1.normalize();
        let proj = d2.dot(&T_f);
        let N_raw = &d2 - &(T_f.clone() * proj);
        assert!(N_raw.compute_length() > 1e-8);
        let N = N_raw.normalize();
        let B = T_f.cross(&N).normalize();
        assert!(T_f.dot(&N).abs() < 1e-6);
        assert!(T_f.dot(&B).abs() < 1e-6);
        assert!(N.dot(&B).abs() < 1e-6);
        let rhs = T_f.cross(&N);
        assert!(rhs.dot(&B) > 0.999);
    }
}

