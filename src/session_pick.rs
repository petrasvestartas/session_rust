//! CPU-side picking: unproject a screen-space cursor into a world-space ray,
//! delegate to `Session::ray_cast` which already covers Point, Line, Polyline,
//! Plane, OBB, Mesh narrow-phase intersection via the BVH.
//!
//! `Session::ray_cast` is the underlying API; this module adds the camera
//! unprojection convenience so a UI can call `pick_by_screen(view, proj, ...)`
//! directly.

use crate::session::{RayHit, Session};
use crate::{Point, Vector};

/// World-space ray; `direction` is assumed normalized.
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: [f32; 3],
    pub direction: [f32; 3],
}

impl Ray {
    pub fn new(origin: [f32; 3], direction: [f32; 3]) -> Self {
        Self { origin, direction }
    }
}

impl Session {
    /// Pick by world-space ray. Wraps `ray_cast` to give the cleaner Ray API.
    pub fn pick_by_ray(&mut self, ray: Ray, pick_radius: f32) -> Vec<RayHit> {
        let origin = Point::new(ray.origin[0], ray.origin[1], ray.origin[2]);
        let direction = Vector::new(ray.direction[0], ray.direction[1], ray.direction[2]);
        self.ray_cast(&origin, &direction, pick_radius)
    }

    /// Pick by screen-space cursor. Unprojects through the view+proj matrices
    /// to construct the world-space ray, then delegates to `pick_by_ray`.
    ///
    /// - `view` / `proj` are column-major 4×4 (as wgpu expects).
    /// - `cursor` is in pixels with top-left origin.
    /// - `viewport` is (width, height) in pixels.
    pub fn pick_by_screen(
        &mut self,
        view: &[[f32; 4]; 4],
        proj: &[[f32; 4]; 4],
        viewport: (f32, f32),
        cursor: (f32, f32),
        pick_radius: f32,
    ) -> Vec<RayHit> {
        let ray = screen_to_world_ray(view, proj, viewport, cursor);
        self.pick_by_ray(ray, pick_radius)
    }
}

/// Build a world-space ray from a screen-space cursor.
///
/// Translates the cursor to NDC space, unprojects two depth points (near and
/// far) into world space, and returns origin + normalized direction.
pub fn screen_to_world_ray(
    view: &[[f32; 4]; 4],
    proj: &[[f32; 4]; 4],
    viewport: (f32, f32),
    cursor: (f32, f32),
) -> Ray {
    let (w, h) = viewport;
    let (cx, cy) = cursor;
    let ndc_x = (cx / w) * 2.0 - 1.0;
    // NDC y is up; pixel y is down — flip.
    let ndc_y = 1.0 - (cy / h) * 2.0;

    let inv = mat4_inverse(&mat4_mul(proj, view));
    let p_near = mat4_unproject(&inv, [ndc_x, ndc_y, 0.0]);
    let p_far = mat4_unproject(&inv, [ndc_x, ndc_y, 1.0]);

    let dx = p_far[0] - p_near[0];
    let dy = p_far[1] - p_near[1];
    let dz = p_far[2] - p_near[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-30);
    Ray {
        origin: p_near,
        direction: [dx / len, dy / len, dz / len],
    }
}

// ---------- Tiny 4x4 utilities (column-major) ----------

fn mat4_mul(a: &[[f32; 4]; 4], b: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[k][r] * b[c][k];
            }
            out[c][r] = s;
        }
    }
    out
}

/// Unproject NDC point through inverse view-projection matrix → world space.
fn mat4_unproject(inv_vp: &[[f32; 4]; 4], ndc: [f32; 3]) -> [f32; 3] {
    let x = ndc[0];
    let y = ndc[1];
    let z = ndc[2];
    let w = 1.0;
    let out_x = inv_vp[0][0] * x + inv_vp[1][0] * y + inv_vp[2][0] * z + inv_vp[3][0] * w;
    let out_y = inv_vp[0][1] * x + inv_vp[1][1] * y + inv_vp[2][1] * z + inv_vp[3][1] * w;
    let out_z = inv_vp[0][2] * x + inv_vp[1][2] * y + inv_vp[2][2] * z + inv_vp[3][2] * w;
    let out_w = inv_vp[0][3] * x + inv_vp[1][3] * y + inv_vp[2][3] * z + inv_vp[3][3] * w;
    let inv_w = if out_w.abs() > 1e-30 { 1.0 / out_w } else { 1.0 };
    [out_x * inv_w, out_y * inv_w, out_z * inv_w]
}

/// 4x4 inverse via cofactor expansion. Slow but readable; for the pick path
/// it runs once per click — not in a hot loop.
fn mat4_inverse(m: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    // Flatten to a single array for indexing clarity (column-major).
    // m[c][r] stored at index c*4 + r.
    let f = |c: usize, r: usize| m[c][r];

    let m00 = f(0, 0); let m01 = f(1, 0); let m02 = f(2, 0); let m03 = f(3, 0);
    let m10 = f(0, 1); let m11 = f(1, 1); let m12 = f(2, 1); let m13 = f(3, 1);
    let m20 = f(0, 2); let m21 = f(1, 2); let m22 = f(2, 2); let m23 = f(3, 2);
    let m30 = f(0, 3); let m31 = f(1, 3); let m32 = f(2, 3); let m33 = f(3, 3);

    let a2323 = m22 * m33 - m23 * m32;
    let a1323 = m21 * m33 - m23 * m31;
    let a1223 = m21 * m32 - m22 * m31;
    let a0323 = m20 * m33 - m23 * m30;
    let a0223 = m20 * m32 - m22 * m30;
    let a0123 = m20 * m31 - m21 * m30;
    let a2313 = m12 * m33 - m13 * m32;
    let a1313 = m11 * m33 - m13 * m31;
    let a1213 = m11 * m32 - m12 * m31;
    let a2312 = m12 * m23 - m13 * m22;
    let a1312 = m11 * m23 - m13 * m21;
    let a1212 = m11 * m22 - m12 * m21;
    let a0313 = m10 * m33 - m13 * m30;
    let a0213 = m10 * m32 - m12 * m30;
    let a0312 = m10 * m23 - m13 * m20;
    let a0212 = m10 * m22 - m12 * m20;
    let a0113 = m10 * m31 - m11 * m30;
    let a0112 = m10 * m21 - m11 * m20;

    let det = m00 * (m11 * a2323 - m12 * a1323 + m13 * a1223)
        - m01 * (m10 * a2323 - m12 * a0323 + m13 * a0223)
        + m02 * (m10 * a1323 - m11 * a0323 + m13 * a0123)
        - m03 * (m10 * a1223 - m11 * a0223 + m12 * a0123);
    let inv_det = if det.abs() > 1e-30 { 1.0 / det } else { 0.0 };

    let r = |a: f32, b: f32, c: f32, d: f32| (a - b + c - d) * inv_det;

    // Compute the inverse (column-major output)
    let mut out = [[0.0; 4]; 4];
    out[0][0] = r(m11 * a2323, m12 * a1323, m13 * a1223, 0.0);
    out[0][1] = r(0.0, m01 * a2323, m02 * a1323, m03 * a1223) * -1.0;
    out[0][2] = r(m01 * a2313, m02 * a1313, m03 * a1213, 0.0);
    out[0][3] = r(0.0, m01 * a2312, m02 * a1312, m03 * a1212) * -1.0;

    out[1][0] = r(0.0, m10 * a2323, m12 * a0323, m13 * a0223) * -1.0;
    out[1][1] = r(m00 * a2323, m02 * a0323, m03 * a0223, 0.0);
    out[1][2] = r(0.0, m00 * a2313, m02 * a0313, m03 * a0213) * -1.0;
    out[1][3] = r(m00 * a2312, m02 * a0312, m03 * a0212, 0.0);

    out[2][0] = r(m10 * a1323, m11 * a0323, m13 * a0123, 0.0);
    out[2][1] = r(0.0, m00 * a1323, m01 * a0323, m03 * a0123) * -1.0;
    out[2][2] = r(m00 * a1313, m01 * a0313, m03 * a0113, 0.0);
    out[2][3] = r(0.0, m00 * a1312, m01 * a0312, m03 * a0112) * -1.0;

    out[3][0] = r(0.0, m10 * a1223, m11 * a0223, m12 * a0123) * -1.0;
    out[3][1] = r(m00 * a1223, m01 * a0223, m02 * a0123, 0.0);
    out[3][2] = r(0.0, m00 * a1213, m01 * a0213, m02 * a0113) * -1.0;
    out[3][3] = r(m00 * a1212, m01 * a0212, m02 * a0112, 0.0);

    out
}

#[cfg(test)]
#[path = "session_pick_test.rs"]
mod session_pick_test;
