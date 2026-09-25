#![allow(
    clippy::needless_range_loop,
    clippy::too_many_arguments,
    clippy::type_complexity,
    clippy::wrong_self_convention
)]
use crate::brep::BRep;
use crate::closest::Closest;
use crate::color::Color;
use crate::intersection;
use crate::line::Line;
use crate::mesh::Mesh;
use crate::nurbscurve::NurbsCurve;
use crate::nurbsknot;
use crate::nurbssurface_trimmed::NurbsSurfaceTrimmed;
use crate::plane::Plane;
use crate::point::Point;
use crate::remesh_nurbssurface_adaptive::RemeshNurbsSurfaceAdaptive;
use crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid;
use crate::tolerance::Tolerance;
use crate::vector::Vector;
use crate::xform::Xform;
use prost::Message;
use serde::ser::SerializeMap;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use serde::Serializer;
use std::fmt;
use std::sync::OnceLock;

// ═══════════════════════════════════════════════════════════════════════════
// File helpers
// ═══════════════════════════════════════════════════════════════════════════
/// Repeat each distinct knot by its multiplicity.
fn expand_nurbsknots(knots: &[f64], mults: &[usize]) -> Vec<f64> {
    let mut full = Vec::new();

    for i in 0..knots.len() {
        for _ in 0..mults[i] {
            full.push(knots[i]);
        }
    }

    full
}

/// C(n, k).
fn binomial(n: usize, k: usize) -> f64 {
    let mut r = 1.0;

    for i in 0..k {
        r = r * (n - i) as f64 / (i + 1) as f64;
    }

    r
}

/// Index of partial (k, m) in the (k, m) loop order of evaluate.
fn skl_index(n: usize, k: usize, m: usize) -> usize {
    k * (n + 1) - k * k.saturating_sub(1) / 2 + m
}

/// True when any weight differs from one.
fn is_rational_weights(weights: &[Vec<f64>]) -> bool {
    for row in weights {
        for &w in row {
            if (w - 1.0).abs() > Tolerance::ZERO_TOLERANCE {
                return true;
            }
        }
    }

    false
}

/// Triangular table of basis values and knot differences (Piegl & Tiller A2.3).
fn basis_table(knot: &[f64], degree: usize, base: usize, t: f64) -> Vec<Vec<f64>> {
    let order = degree + 1;
    let mut ndu = vec![vec![0.0; order]; order];
    ndu[0][0] = 1.0;
    let mut left = vec![0.0; order];
    let mut right = vec![0.0; order];

    for j in 1..=degree {
        left[j] = t - knot[base - j];
        right[j] = knot[base + j - 1] - t;
        let mut saved = 0.0;

        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        ndu[j][j] = saved;
    }

    ndu
}

/// Basis derivatives ders[k][j] from the triangular table (Piegl & Tiller A2.3).
fn basis_table_derivatives(ndu: &[Vec<f64>], degree: usize, deriv_order: usize) -> Vec<Vec<f64>> {
    let order = degree + 1;
    let mut ders = vec![vec![0.0; order]; deriv_order + 1];

    for j in 0..=degree {
        ders[0][j] = ndu[j][degree];
    }

    let mut a = vec![vec![0.0; order]; 2];

    for r in 0..=degree {
        let mut s1 = 0;
        let mut s2 = 1;
        a[0][0] = 1.0;

        for k in 1..=deriv_order {
            let mut d = 0.0;
            let rk = r as isize - k as isize;
            let pk = degree as isize - k as isize;

            if r >= k {
                a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }

            let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
            let j2 = if r as isize - 1 <= pk {
                k - 1
            } else {
                degree - r
            };

            for j in j1..=j2 {
                a[s2][j] =
                    (a[s1][j] - a[s1][j - 1]) / ndu[(pk + 1) as usize][(rk + j as isize) as usize];

                d += a[s2][j] * ndu[(rk + j as isize) as usize][pk as usize];
            }

            if r as isize <= pk {
                a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                d += a[s2][k] * ndu[r][pk as usize];
            }

            ders[k][r] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }

    let mut factor = degree as f64;

    for k in 1..=deriv_order {
        for j in 0..=degree {
            ders[k][j] *= factor;
        }

        factor *= (degree as isize - k as isize) as f64;
    }

    ders
}

/// Bounding box of a 7 x 7 sample of the surface.
fn surface_aabb(srf: &NurbsSurface) -> ([f64; 3], [f64; 3]) {
    let n = 6;
    let (u0, u1) = srf.domain(0).unwrap_or((0.0, 0.0));
    let (v0, v1) = srf.domain(1).unwrap_or((0.0, 0.0));
    let mut lo = [1e30_f64; 3];
    let mut hi = [-1e30_f64; 3];

    for i in 0..=n {
        for j in 0..=n {
            let u = u0 + (u1 - u0) * i as f64 / n as f64;
            let v = v0 + (v1 - v0) * j as f64 / n as f64;
            let p = srf.point_at(u, v).unwrap_or_default();

            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }

    (lo, hi)
}

/// Boxes overlap once a is padded by a thousandth of its longest side.
fn aabb_overlap_pad(a: &([f64; 3], [f64; 3]), b: &([f64; 3], [f64; 3])) -> bool {
    let m = (a.1[0] - a.0[0]).max(a.1[1] - a.0[1]).max(a.1[2] - a.0[2]) * 1e-3;

    for k in 0..3 {
        if a.0[k] - m > b.1[k] || b.0[k] - m > a.1[k] {
            return false;
        }
    }

    true
}

/// Flattens colors to r, g, b, a values.
fn colors_to_json(colors: &[Color]) -> Vec<f32> {
    let mut arr = Vec::new();

    for c in colors {
        arr.extend_from_slice(&[c.r, c.g, c.b, c.a]);
    }

    arr
}

/// Reads colors from a flat r, g, b, a list.
fn colors_from_json(arr: &[f32]) -> Vec<Color> {
    let mut colors = Vec::new();
    let mut i = 0;

    while i + 3 < arr.len() {
        colors.push(Color::new(arr[i], arr[i + 1], arr[i + 2], arr[i + 3]));
        i += 4;
    }

    colors
}

/// Converts colors to repeated proto messages.
fn colors_to_proto(colors: &[Color]) -> Vec<crate::proto::Color> {
    let mut field = Vec::new();

    for c in colors {
        field.push(crate::proto::Color {
            guid: String::new(),
            name: String::new(),
            r: c.r,
            g: c.g,
            b: c.b,
            a: c.a,
        });
    }

    field
}

/// Reads colors from repeated proto messages.
fn colors_from_proto(field: &[crate::proto::Color]) -> Vec<Color> {
    let mut colors = Vec::new();

    for c in field {
        colors.push(Color::new(c.r, c.g, c.b, c.a));
    }

    colors
}

/// A NURBS surface: OpenNURBS layout, nurbsknot count = order + cv_count - 2 per direction, homogeneous row-major CVs when rational.
#[derive(Clone, Debug)]
pub struct NurbsSurface {
    guid: OnceLock<String>,         // Lazily minted GUID.
    pub name: String,               // Surface name.
    pub width: f64,                 // Display width.
    pub pointcolors: Vec<Color>,    // Display color per control point.
    pub facecolors: Vec<Color>,     // Display color per mesh face.
    pub linecolors: Vec<Color>,     // Display color per control polygon segment.
    pub m_dim: usize,               // Coordinate dimension.
    pub m_is_rat: bool,             // True when rational.
    pub m_order: [usize; 2],        // Degree + 1 per direction.
    pub m_cv_count: [usize; 2],     // Number of control vertices per direction.
    pub m_cv_stride: [usize; 2],    // Doubles between consecutive CVs per direction.
    pub m_nurbsknot: [Vec<f64>; 2], // NurbsKnot vector per direction, order + cv_count - 2 values.
    pub m_cv: Vec<f64>,             // Flat CV array, homogeneous when rational.
    pub m_mesh: Option<Mesh>,       // Cached mesh from mesh() or mesh_adaptive().
}

impl Default for NurbsSurface {
    /// Construct an empty surface.
    fn default() -> Self {
        NurbsSurface {
            guid: OnceLock::new(),
            name: "my_nurbssurface".to_string(),
            width: 1.0,
            pointcolors: Vec::new(),
            facecolors: Vec::new(),
            linecolors: Vec::new(),
            m_dim: 0,
            m_is_rat: false,
            m_order: [0, 0],
            m_cv_count: [0, 0],
            m_cv_stride: [0, 0],
            m_nurbsknot: [Vec::new(), Vec::new()],
            m_cv: Vec::new(),
            m_mesh: None,
        }
    }
}

impl NurbsSurface {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an unset surface with the given layout.
    pub fn new(
        dimension: usize,
        is_rational: bool,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
    ) -> Self {
        let mut surface = Self::default();
        surface.create_raw(
            dimension,
            is_rational,
            order0,
            order1,
            cv_count0,
            cv_count1,
            false,
            false,
            1.0,
            1.0,
        );

        surface
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = OnceLock::new();

        copy
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Static constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a clamped or periodic uniform surface through cv_count_u x cv_count_v points in row-major order (u slowest).
    pub fn create(
        periodic_u: bool,
        periodic_v: bool,
        degree_u: usize,
        degree_v: usize,
        cv_count_u: usize,
        cv_count_v: usize,
        points: &[Point],
    ) -> Result<Self, String> {
        if degree_u < 1 || degree_v < 1 {
            return Err(format!(
                "NurbsSurface::create: degree must be >= 1, got degree_u={}, degree_v={}",
                degree_u, degree_v
            ));
        }

        if cv_count_u < degree_u + 1 {
            return Err(format!(
                "NurbsSurface::create: cv_count_u ({}) must be >= degree_u+1 ({})",
                cv_count_u,
                degree_u + 1
            ));
        }

        if cv_count_v < degree_v + 1 {
            return Err(format!(
                "NurbsSurface::create: cv_count_v ({}) must be >= degree_v+1 ({})",
                cv_count_v,
                degree_v + 1
            ));
        }

        let expected = cv_count_u * cv_count_v;

        if points.len() != expected {
            return Err(format!(
                "NurbsSurface::create: expected {} points ({}x{}), got {}",
                expected,
                cv_count_u,
                cv_count_v,
                points.len()
            ));
        }

        let mut surface = Self::default();
        surface.create_raw(
            3,
            false,
            degree_u + 1,
            degree_v + 1,
            cv_count_u,
            cv_count_v,
            periodic_u,
            periodic_v,
            1.0,
            1.0,
        );

        for i in 0..cv_count_u {
            for j in 0..cv_count_v {
                surface.set_cv(i, j, &points[i * cv_count_v + j]);
            }
        }

        Ok(surface)
    }

    /// Construct from points[iv][iu], weights[iv][iu], distinct knots and multiplicities per direction (OCCT convention).
    pub fn create_from_parameters(
        points: &[Vec<Point>],
        weights: &[Vec<f64>],
        knots_u: &[f64],
        knots_v: &[f64],
        mults_u: &[usize],
        mults_v: &[usize],
        degree_u: usize,
        degree_v: usize,
        periodic_u: bool,
        periodic_v: bool,
    ) -> Self {
        let nv = points.len();
        let nu = if nv > 0 { points[0].len() } else { 0 };
        let order_u = degree_u + 1;
        let order_v = degree_v + 1;

        if nu < order_u || nv < order_v || periodic_u || periodic_v {
            return Self::default();
        }

        if knots_u.len() != mults_u.len() || knots_v.len() != mults_v.len() {
            return Self::default();
        }

        let rational = is_rational_weights(weights);
        let full_u = expand_nurbsknots(knots_u, mults_u);
        let full_v = expand_nurbsknots(knots_v, mults_v);
        let kc_u = order_u + nu - 2;
        let kc_v = order_v + nv - 2;

        if full_u.len() != kc_u + 2 || full_v.len() != kc_v + 2 {
            return Self::default();
        }

        let mut surface = Self::default();

        if !surface.create_raw(
            3, rational, order_u, order_v, nu, nv, false, false, 1.0, 1.0,
        ) {
            return Self::default();
        }

        for i in 0..kc_u {
            surface.set_nurbsknot(0, i, full_u[i + 1]);
        }

        for i in 0..kc_v {
            surface.set_nurbsknot(1, i, full_v[i + 1]);
        }

        for i in 0..nu {
            for j in 0..nv {
                let p = &points[j][i];

                if rational {
                    let w = weights[j][i];
                    surface.set_cv_4d(i, j, p[0] * w, p[1] * w, p[2] * w, w);
                } else {
                    surface.set_cv(i, j, p);
                }
            }
        }

        surface
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Transform every CV in place.
    pub fn transform(&mut self, xform: &Xform) -> bool {
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let mut p = self.get_cv(i, j).unwrap_or_default();
                p.transform(xform);
                self.set_cv(i, j, &p);
            }
        }

        true
    }

    /// Return a transformed copy.
    pub fn transformed(&self, xform: &Xform) -> Self {
        let mut result = self.duplicate();
        result.transform(xform);

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Initialization
    // ═══════════════════════════════════════════════════════════════════════════
    /// Reset every field to the empty invalid surface.
    pub fn initialize(&mut self) {
        *self = Self::default();
    }

    /// Allocate nurbsknots (clamped or periodic uniform) and zeroed CVs; false when order < 2 or cv_count < order.
    pub fn create_raw(
        &mut self,
        dimension: usize,
        is_rational: bool,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
        is_periodic_u: bool,
        is_periodic_v: bool,
        nurbsknot_delta_u: f64,
        nurbsknot_delta_v: f64,
    ) -> bool {
        if dimension < 1 || order0 < 2 || order1 < 2 || cv_count0 < order0 || cv_count1 < order1 {
            return false;
        }

        self.destroy();
        self.m_dim = dimension;
        self.m_is_rat = is_rational;
        self.m_order = [order0, order1];
        self.m_cv_count = [cv_count0, cv_count1];
        self.m_cv_stride = [self.cv_size() * cv_count1, self.cv_size()];
        self.m_nurbsknot = [
            vec![0.0; order0 + cv_count0 - 2],
            vec![0.0; order1 + cv_count1 - 2],
        ];
        self.m_cv = vec![0.0; cv_count0 * cv_count1 * self.cv_size()];
        self.zero_cvs();

        if is_periodic_u {
            self.make_periodic_uniform_nurbsknot_vector(0, nurbsknot_delta_u);
        } else {
            self.make_clamped_uniform_nurbsknot_vector(0, nurbsknot_delta_u);
        }

        if is_periodic_v {
            self.make_periodic_uniform_nurbsknot_vector(1, nurbsknot_delta_v);
        } else {
            self.make_clamped_uniform_nurbsknot_vector(1, nurbsknot_delta_v);
        }

        true
    }

    /// Allocate a non-rational surface with clamped uniform nurbsknots of the given spacing.
    pub fn create_clamped_uniform(
        &mut self,
        dimension: usize,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
        nurbsknot_delta0: f64,
        nurbsknot_delta1: f64,
    ) -> bool {
        self.create_raw(
            dimension,
            false,
            order0,
            order1,
            cv_count0,
            cv_count1,
            false,
            false,
            nurbsknot_delta0,
            nurbsknot_delta1,
        )
    }

    /// Clear all data; is_valid() is false afterwards.
    pub fn destroy(&mut self) {
        self.initialize();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean queries
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether orders >= 2, cv_count >= order, nurbsknot vectors are valid and the CV array is large enough.
    pub fn is_valid(&self) -> bool {
        if self.m_dim < 1 || self.m_order[0] < 2 || self.m_order[1] < 2 {
            return false;
        }

        if self.m_cv_count[0] < self.m_order[0] || self.m_cv_count[1] < self.m_order[1] {
            return false;
        }

        if !self.is_valid_nurbsknot_vector(0) || !self.is_valid_nurbsknot_vector(1) {
            return false;
        }

        self.m_cv.len() >= self.cv_count_total() * self.cv_size()
    }

    /// Return whether the nurbsknot vector in dir has the right length and is non-decreasing.
    pub fn is_valid_nurbsknot_vector(&self, dir: usize) -> bool {
        if dir > 1 {
            return false;
        }

        let kc = self.nurbsknot_count(dir);

        if self.m_nurbsknot[dir].len() != kc {
            return false;
        }

        for i in 1..kc {
            if self.m_nurbsknot[dir][i] < self.m_nurbsknot[dir][i - 1] {
                return false;
            }
        }

        true
    }

    /// Return whether the CVs carry weights.
    pub fn is_rational(&self) -> bool {
        self.m_is_rat
    }

    /// Return whether the first and last CV rows across dir coincide when clamped, else whether dir is periodic.
    pub fn is_closed(&self, dir: usize) -> bool {
        if dir > 1 || !self.is_valid() {
            return false;
        }

        if !self.is_clamped(dir, 2) {
            return self.is_periodic(dir);
        }

        let last = self.m_cv_count[dir] - 1;

        for k in 0..self.m_cv_count[1 - dir] {
            let a = if dir == 1 {
                self.get_cv(k, 0)
            } else {
                self.get_cv(0, k)
            };
            let b = if dir == 1 {
                self.get_cv(k, last)
            } else {
                self.get_cv(last, k)
            };

            if a.unwrap_or_default().distance(&b.unwrap_or_default(), None)
                > Tolerance::ZERO_TOLERANCE
            {
                return false;
            }
        }

        true
    }

    /// Return whether dir has uniform nurbsknot spacing and the first degree CV rows repeat the last.
    pub fn is_periodic(&self, dir: usize) -> bool {
        if dir > 1 || !self.is_valid() {
            return false;
        }

        if !nurbsknot::is_periodic(
            self.m_order[dir],
            self.m_cv_count[dir],
            &self.m_nurbsknot[dir],
        ) {
            return false;
        }

        let deg = self.degree(dir);
        let n = self.m_cv_count[dir];

        for k in 0..self.m_cv_count[1 - dir] {
            for i in 0..deg {
                let a = if dir == 1 {
                    self.get_cv(k, i)
                } else {
                    self.get_cv(i, k)
                };
                let b = if dir == 1 {
                    self.get_cv(k, n - deg + i)
                } else {
                    self.get_cv(n - deg + i, k)
                };

                if a.unwrap_or_default().distance(&b.unwrap_or_default(), None)
                    > Tolerance::ZERO_TOLERANCE
                {
                    return false;
                }
            }
        }

        true
    }

    /// Return whether every CV is within tolerance of one plane, written to plane when given.
    pub fn is_planar(&self, plane: Option<&mut Plane>, tolerance: f64) -> bool {
        if !self.is_valid() {
            return false;
        }

        let p0 = self.get_cv(0, 0).unwrap_or_default();
        let mut va = Vector::new(0.0, 0.0, 0.0);
        let mut normal = Vector::new(0.0, 0.0, 0.0);

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let p = self.get_cv(i, j).unwrap_or_default();
                let v = &p - &p0;

                if va.magnitude() < 1e-14 {
                    va = v;
                } else if normal.magnitude() < 1e-14 {
                    normal = va.cross(&v);
                }
            }
        }

        if normal.magnitude() < 1e-14 {
            return true;
        }

        normal = &normal / normal.magnitude();

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let p = self.get_cv(i, j).unwrap_or_default();
                let v = &p - &p0;

                if v.dot(&normal).abs() > tolerance {
                    return false;
                }
            }
        }

        if let Some(plane) = plane {
            *plane = Plane::from_point_normal(p0, normal, None);
        }

        true
    }

    /// Return whether a clamped side collapses to one point; side: 0 south (v0), 1 east (u1), 2 north (v1), 3 west (u0).
    pub fn is_singular(&self, side: usize) -> bool {
        if side > 3 || !self.is_valid() {
            return false;
        }

        let fix = if side.is_multiple_of(2) { 1 } else { 0 };
        let end = if side == 0 || side == 3 { 0 } else { 1 };

        if !self.is_clamped(fix, end) {
            return false;
        }

        let at = if end == 1 {
            self.m_cv_count[fix] - 1
        } else {
            0
        };
        let first = if fix == 1 {
            self.get_cv(0, at)
        } else {
            self.get_cv(at, 0)
        };

        for k in 1..self.m_cv_count[1 - fix] {
            let p = if fix == 1 {
                self.get_cv(k, at)
            } else {
                self.get_cv(at, k)
            };

            if p.unwrap_or_default()
                .distance(&first.clone().unwrap_or_default(), None)
                > Tolerance::ZERO_TOLERANCE
            {
                return false;
            }
        }

        true
    }

    /// Return whether dir has full end multiplicity; end: 0 start, 1 end, 2 both.
    pub fn is_clamped(&self, dir: usize, end: i32) -> bool {
        if dir > 1 {
            return false;
        }

        nurbsknot::is_clamped(
            self.m_order[dir],
            self.m_cv_count[dir],
            &self.m_nurbsknot[dir],
            end,
        )
    }

    /// Return whether layout, CVs and weights match within tolerance; nurbsknots too unless ignore_parameterization.
    pub fn is_duplicate(
        &self,
        other: &Self,
        ignore_parameterization: bool,
        tolerance: f64,
    ) -> bool {
        if !self.is_valid() || !other.is_valid() {
            return false;
        }

        if self.m_dim != other.m_dim || self.m_is_rat != other.m_is_rat {
            return false;
        }

        if self.m_order != other.m_order || self.m_cv_count != other.m_cv_count {
            return false;
        }

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let a = self.get_cv(i, j).unwrap_or_default();
                let b = other.get_cv(i, j).unwrap_or_default();

                if a.distance(&b, None) > tolerance {
                    return false;
                }

                if (self.weight(i, j) - other.weight(i, j)).abs() > tolerance {
                    return false;
                }
            }
        }

        if ignore_parameterization {
            return true;
        }

        for dir in 0..2 {
            for i in 0..self.nurbsknot_count(dir) {
                if (self.nurbsknot(dir, i).unwrap_or(0.0) - other.nurbsknot(dir, i).unwrap_or(0.0))
                    .abs()
                    > tolerance
                {
                    return false;
                }
            }
        }

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return whether the lazy guid has been created.
    pub fn has_guid(&self) -> bool {
        self.guid.get().is_some()
    }

    /// Return the guid, creating it on first access.
    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    /// Set the guid.
    pub fn set_guid(&mut self, guid: String) {
        self.guid = OnceLock::from(guid);
    }

    /// Clear the guid so a fresh one mints lazily on the next read.
    pub fn refresh_guid(&mut self) {
        self.guid = OnceLock::new();
    }

    /// Return the coordinate dimension.
    pub fn dimension(&self) -> usize {
        self.m_dim
    }

    /// Return the order (degree + 1) in dir.
    pub fn order(&self, dir: usize) -> usize {
        if dir < 2 {
            self.m_order[dir]
        } else {
            0
        }
    }

    /// Return the degree in dir.
    pub fn degree(&self, dir: usize) -> usize {
        if dir < 2 && self.m_order[dir] > 0 {
            self.m_order[dir] - 1
        } else {
            0
        }
    }

    /// Return the number of control vertices in dir.
    pub fn cv_count(&self, dir: usize) -> usize {
        if dir < 2 {
            self.m_cv_count[dir]
        } else {
            0
        }
    }

    /// Return cv_count(0) * cv_count(1).
    pub fn cv_count_total(&self) -> usize {
        self.m_cv_count[0] * self.m_cv_count[1]
    }

    /// Return the doubles per CV: dimension + 1 when rational.
    pub fn cv_size(&self) -> usize {
        if self.m_is_rat {
            self.m_dim + 1
        } else {
            self.m_dim
        }
    }

    /// Return order + cv_count - 2 in dir.
    pub fn nurbsknot_count(&self, dir: usize) -> usize {
        if dir < 2 && self.m_order[dir] + self.m_cv_count[dir] >= 2 {
            self.m_order[dir] + self.m_cv_count[dir] - 2
        } else {
            0
        }
    }

    /// Return cv_count - order + 1 in dir.
    pub fn span_count(&self, dir: usize) -> usize {
        if dir < 2 && self.m_cv_count[dir] + 1 >= self.m_order[dir] {
            self.m_cv_count[dir] + 1 - self.m_order[dir]
        } else {
            0
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Control vertex access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the pointer to CV[i][j], nullptr when out of range.
    pub fn cv(&self, i: usize, j: usize) -> Option<&[f64]> {
        if i >= self.m_cv_count[0] || j >= self.m_cv_count[1] {
            return None;
        }

        let idx = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];

        Some(&self.m_cv[idx..idx + self.cv_size()])
    }

    /// Return the mutable pointer to CV[i][j], cv_size() doubles (x*w, y*w, z*w, w when rational), nullptr when out of range.
    pub fn cv_mut(&mut self, i: usize, j: usize) -> Option<&mut [f64]> {
        if i >= self.m_cv_count[0] || j >= self.m_cv_count[1] {
            return None;
        }

        let idx = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
        let size = self.cv_size();

        Some(&mut self.m_cv[idx..idx + size])
    }

    /// Return the Euclidean CV (divided by weight when rational), None when out of range.
    pub fn get_cv(&self, i: usize, j: usize) -> Option<Point> {
        let cv_ptr = self.cv(i, j)?;

        Some(self.dehomogenize(cv_ptr))
    }

    /// Return the homogeneous CV (x, y, z, w), w = 1 when non-rational, None when out of range.
    pub fn get_cv_4d(&self, i: usize, j: usize) -> Option<(f64, f64, f64, f64)> {
        let cv_ptr = self.cv(i, j)?;
        let x = cv_ptr[0];
        let y = if self.m_dim > 1 { cv_ptr[1] } else { 0.0 };
        let z = if self.m_dim > 2 { cv_ptr[2] } else { 0.0 };
        let w = if self.m_is_rat {
            cv_ptr[self.m_dim]
        } else {
            1.0
        };

        Some((x, y, z, w))
    }

    /// Set the Euclidean CV, keeping its weight.
    pub fn set_cv(&mut self, i: usize, j: usize, point: &Point) -> bool {
        let dim = self.m_dim;
        let is_rat = self.m_is_rat;
        let Some(cv_ptr) = self.cv_mut(i, j) else {
            return false;
        };

        let w = if is_rat && cv_ptr[dim].abs() > 1e-14 {
            cv_ptr[dim]
        } else {
            1.0
        };
        cv_ptr[0] = point[0] * w;

        if dim > 1 {
            cv_ptr[1] = point[1] * w;
        }

        if dim > 2 {
            cv_ptr[2] = point[2] * w;
        }

        true
    }

    /// Set the homogeneous CV; w ignored when non-rational.
    pub fn set_cv_4d(&mut self, i: usize, j: usize, x: f64, y: f64, z: f64, w: f64) -> bool {
        let dim = self.m_dim;
        let is_rat = self.m_is_rat;
        let Some(cv_ptr) = self.cv_mut(i, j) else {
            return false;
        };

        cv_ptr[0] = x;

        if dim > 1 {
            cv_ptr[1] = y;
        }

        if dim > 2 {
            cv_ptr[2] = z;
        }

        if is_rat {
            cv_ptr[dim] = w;
        }

        true
    }

    /// Return the weight of CV[i][j], 1 when non-rational.
    pub fn weight(&self, i: usize, j: usize) -> f64 {
        match self.cv(i, j) {
            Some(cv_ptr) if self.m_is_rat => cv_ptr[self.m_dim],
            _ => 1.0,
        }
    }

    /// Rescale the homogeneous CV to the new weight so the Euclidean point stays; false when non-rational.
    pub fn set_weight(&mut self, i: usize, j: usize, w: f64) -> bool {
        let dim = self.m_dim;

        if !self.m_is_rat {
            return false;
        }

        let Some(cv_ptr) = self.cv_mut(i, j) else {
            return false;
        };

        let old_w = if cv_ptr[dim].abs() > 1e-14 {
            cv_ptr[dim]
        } else {
            1.0
        };
        let new_w = if w.abs() > 1e-14 { w } else { 1.0 };
        let scale = new_w / old_w;

        for d in 0..dim {
            cv_ptr[d] *= scale;
        }

        cv_ptr[dim] = new_w;

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // NurbsKnot access
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the nurbsknot at nurbsknot_index in dir, None when out of range.
    pub fn nurbsknot(&self, dir: usize, nurbsknot_index: usize) -> Option<f64> {
        if dir > 1 || nurbsknot_index >= self.m_nurbsknot[dir].len() {
            return None;
        }

        Some(self.m_nurbsknot[dir][nurbsknot_index])
    }

    /// Set the nurbsknot at nurbsknot_index in dir.
    pub fn set_nurbsknot(
        &mut self,
        dir: usize,
        nurbsknot_index: usize,
        nurbsknot_value: f64,
    ) -> bool {
        if dir > 1 || nurbsknot_index >= self.m_nurbsknot[dir].len() {
            return false;
        }

        self.m_nurbsknot[dir][nurbsknot_index] = nurbsknot_value;

        true
    }

    /// Return the multiplicity of the nurbsknot at nurbsknot_index in dir.
    pub fn nurbsknot_multiplicity(&self, dir: usize, nurbsknot_index: usize) -> usize {
        if dir > 1 {
            return 0;
        }

        nurbsknot::multiplicity(
            self.m_order[dir],
            self.m_cv_count[dir],
            &self.m_nurbsknot[dir],
            nurbsknot_index,
        )
    }

    /// Return a copy of the nurbsknot vector in dir.
    pub fn get_nurbsknots(&self, dir: usize) -> Vec<f64> {
        if dir < 2 {
            self.m_nurbsknot[dir].clone()
        } else {
            Vec::new()
        }
    }

    /// Insert a nurbsknot with the given multiplicity in dir without changing the shape.
    pub fn insert_nurbsknot(
        &mut self,
        dir: usize,
        nurbsknot_value: f64,
        nurbsknot_multiplicity: usize,
    ) -> bool {
        if dir > 1
            || !self.is_valid()
            || nurbsknot_multiplicity == 0
            || nurbsknot_multiplicity >= self.m_order[dir]
        {
            return false;
        }

        let Some((t0, t1)) = self.domain(dir) else {
            return false;
        };

        if nurbsknot_value < t0 || nurbsknot_value > t1 {
            return false;
        }

        let mut crv = self.to_curve(dir);

        if !crv.insert_nurbsknot(nurbsknot_value, nurbsknot_multiplicity) {
            return false;
        }

        self.from_curve(&crv, dir)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Domain
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return [nurbsknot[order - 2], nurbsknot[cv_count - 1]] in dir, None when invalid.
    pub fn domain(&self, dir: usize) -> Option<(f64, f64)> {
        if dir > 1 || !self.is_valid() {
            return None;
        }

        Some((
            self.m_nurbsknot[dir][self.m_order[dir] - 2],
            self.m_nurbsknot[dir][self.m_cv_count[dir] - 1],
        ))
    }

    /// Linearly remap the nurbsknots in dir onto [t0, t1].
    pub fn set_domain(&mut self, dir: usize, t0: f64, t1: f64) -> bool {
        if dir > 1 || !self.is_valid() || t0 >= t1 {
            return false;
        }

        let Some((d0, d1)) = self.domain(dir) else {
            return false;
        };

        if (d1 - d0).abs() < 1e-14 {
            return false;
        }

        let scale = (t1 - t0) / (d1 - d0);

        for k in self.m_nurbsknot[dir].iter_mut() {
            *k = t0 + (*k - d0) * scale;
        }

        true
    }

    /// Return the distinct nurbsknot values inside the domain of dir.
    pub fn get_span_vector(&self, dir: usize) -> Vec<f64> {
        let mut spans = Vec::new();

        if dir > 1 || !self.is_valid() {
            return spans;
        }

        spans.push(self.m_nurbsknot[dir][self.m_order[dir] - 2]);

        for i in self.m_order[dir] - 1..self.m_cv_count[dir] {
            if self.m_nurbsknot[dir][i] > spans[spans.len() - 1] {
                spans.push(self.m_nurbsknot[dir][i]);
            }
        }

        spans
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Division
    // ═══════════════════════════════════════════════════════════════════════════
    /// Points, normals and (u, v) on a (nu + 1) x (nv + 1) grid over the domain.
    pub fn divide_by_count_points(
        &self,
        nu: usize,
        nv: usize,
    ) -> (Vec<Vec<Point>>, Vec<Vec<Vector>>, Vec<Vec<(f64, f64)>>) {
        let mut grid = Vec::new();
        let mut normals = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() {
            return (grid, normals, params);
        }

        let (u0, u1) = self.domain(0).unwrap_or((0.0, 0.0));
        let (v0, v1) = self.domain(1).unwrap_or((0.0, 0.0));

        for i in 0..=nu {
            let u = if nu > 0 {
                u0 + (u1 - u0) * i as f64 / nu as f64
            } else {
                u0
            };
            grid.push(Vec::new());
            normals.push(Vec::new());
            params.push(Vec::new());

            for j in 0..=nv {
                let v = if nv > 0 {
                    v0 + (v1 - v0) * j as f64 / nv as f64
                } else {
                    v0
                };
                grid[i].push(self.point_at(u, v).unwrap_or_default());
                normals[i].push(self.normal_at(u, v));
                params[i].push((u, v));
            }
        }

        (grid, normals, params)
    }

    /// Frames (x = dS/du, y = dS/dv) and (u, v) on a (nu + 1) x (nv + 1) grid over the domain.
    pub fn divide_by_count_planes(
        &self,
        nu: usize,
        nv: usize,
    ) -> (Vec<Vec<Plane>>, Vec<Vec<(f64, f64)>>) {
        let mut grid = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() {
            return (grid, params);
        }

        let (u0, u1) = self.domain(0).unwrap_or((0.0, 0.0));
        let (v0, v1) = self.domain(1).unwrap_or((0.0, 0.0));

        for i in 0..=nu {
            let u = if nu > 0 {
                u0 + (u1 - u0) * i as f64 / nu as f64
            } else {
                u0
            };
            grid.push(Vec::new());
            params.push(Vec::new());

            for j in 0..=nv {
                let v = if nv > 0 {
                    v0 + (v1 - v0) * j as f64 / nv as f64
                } else {
                    v0
                };
                let derivs = self.evaluate(u, v, 1);
                let mut x_axis = derivs[2].clone();
                let mut y_axis = derivs[1].clone();

                if x_axis.magnitude() > 1e-14 {
                    x_axis = x_axis.normalized();
                }

                if y_axis.magnitude() > 1e-14 {
                    y_axis = y_axis.normalized();
                }

                grid[i].push(Plane::from_frame(
                    self.point_at(u, v).unwrap_or_default(),
                    x_axis,
                    y_axis,
                    self.normal_at(u, v),
                ));
                params[i].push((u, v));
            }
        }

        (grid, params)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return S(u, v) by the tensor-product basis, None when invalid.
    pub fn point_at(&self, u: f64, v: f64) -> Option<Point> {
        if !self.is_valid() {
            return None;
        }

        let span_u = self.find_span(0, u);
        let span_v = self.find_span(1, v);
        let nu = nurbsknot::eval_basis(self.m_order[0], &self.m_nurbsknot[0], span_u, u);
        let nv = nurbsknot::eval_basis(self.m_order[1], &self.m_nurbsknot[1], span_v, v);
        let size = self.cv_size();
        let mut sum = vec![0.0; size];

        for i in 0..self.m_order[0] {
            for j in 0..self.m_order[1] {
                let c = nu[i] * nv[j];
                let cv_ptr = self.cv(span_u + i, span_v + j)?;

                for d in 0..size {
                    sum[d] += c * cv_ptr[d];
                }
            }
        }

        Some(self.dehomogenize(&sum))
    }

    /// Return (u, v) of the closest surface point (grid seed + Newton).
    pub fn closest_parameters(&self, test_point: &Point) -> (f64, f64) {
        let hit = Closest::surface_point(self, test_point, 0.0, 0.0, 0.0, 0.0);

        (hit.0, hit.1)
    }

    /// Return the closest surface point to test_point.
    pub fn closest_point(&self, test_point: &Point) -> Point {
        let (u, v) = self.closest_parameters(test_point);

        self.point_at(u, v).unwrap_or_default()
    }

    /// Return K = (LN - M^2) / (EG - F^2).
    pub fn gaussian_curvature(&self, u: f64, v: f64) -> f64 {
        let Some((e, f, g, l, m, n)) = self.fundamental_forms(u, v) else {
            return 0.0;
        };

        let denom = e * g - f * f;

        if denom.abs() < Tolerance::ZERO_TOLERANCE {
            return 0.0;
        }

        (l * n - m * m) / denom
    }

    /// Return H = (EN - 2FM + GL) / (2(EG - F^2)), sign following Su x Sv.
    pub fn mean_curvature(&self, u: f64, v: f64) -> f64 {
        let Some((e, f, g, l, m, n)) = self.fundamental_forms(u, v) else {
            return 0.0;
        };

        let denom = e * g - f * f;

        if denom.abs() < Tolerance::ZERO_TOLERANCE {
            return 0.0;
        }

        (e * n - 2.0 * f * m + g * l) / (2.0 * denom)
    }

    /// Return the unit normal dS/dv x dS/du, z-axis at singular points.
    pub fn normal_at(&self, u: f64, v: f64) -> Vector {
        let derivs = self.evaluate(u, v, 1);

        if derivs.len() < 3 {
            return Vector::new(0.0, 0.0, 1.0);
        }

        let normal = derivs[2].cross(&derivs[1]);
        let len = normal.magnitude();

        if len < 1e-14 {
            return Vector::new(0.0, 0.0, 1.0);
        }

        &normal / len
    }

    /// Return the frame at (u, v): origin S, x-axis dS/du, y-axis dS/dv.
    pub fn frame_at(&self, u: f64, v: f64) -> Plane {
        let derivs = self.evaluate(u, v, 1);

        if derivs.len() < 3 {
            return Plane::new(
                Point::new(0.0, 0.0, 0.0),
                Vector::new(1.0, 0.0, 0.0),
                Vector::new(0.0, 1.0, 0.0),
            );
        }

        Plane::new(
            Point::new(derivs[0][0], derivs[0][1], derivs[0][2]),
            derivs[2].clone(),
            derivs[1].clone(),
        )
    }

    /// Return the points where the infinite line pierces the surface (grid seed + Newton).
    pub fn intersections_with_line(&self, line: &Line) -> Vec<Point> {
        let mut results: Vec<Point> = Vec::new();

        if !self.is_valid() {
            return results;
        }

        let p0 = line.start();
        let pe = line.end();
        let mut d = &pe - &p0;

        if d.magnitude() < 1e-14 {
            return results;
        }

        d = d.normalized();
        let helper = if d[0].abs() < 0.9 {
            Vector::new(1.0, 0.0, 0.0)
        } else {
            Vector::new(0.0, 1.0, 0.0)
        };
        let n1 = d.cross(&helper).normalized();
        let n2 = d.cross(&n1).normalized();
        let (u0, u1) = self.domain(0).unwrap_or((0.0, 0.0));
        let (v0, v1) = self.domain(1).unwrap_or((0.0, 0.0));
        let nu = (self.cv_count(0) * 4).max(12);
        let nv = (self.cv_count(1) * 4).max(12);

        for a in 0..=nu {
            for b in 0..=nv {
                let mut u = u0 + (u1 - u0) * a as f64 / nu as f64;
                let mut v = v0 + (v1 - v0) * b as f64 / nv as f64;

                if !self.line_newton(&mut u, &mut v, &p0, &n1, &n2) {
                    continue;
                }

                let p = self.point_at(u, v).unwrap_or_default();
                let r = &p - &p0;

                if n1.dot(&r).abs() > 1e-7 || n2.dot(&r).abs() > 1e-7 {
                    continue;
                }

                let mut dup = false;

                for q in &results {
                    if p.distance(q, None) < 1e-6 {
                        dup = true;
                    }
                }

                if !dup {
                    results.push(p);
                }
            }
        }

        results
    }

    /// Return the point and partials up to num_derivs (max 2) in (k, m) loop order: [S, Sv, Svv, Su, Suv, Suu].
    pub fn evaluate(&self, u: f64, v: f64, num_derivs: usize) -> Vec<Vector> {
        let mut result = Vec::new();

        if !self.is_valid() {
            return result;
        }

        let n = num_derivs.min(2);
        let span_u = self.find_span(0, u);
        let span_v = self.find_span(1, v);
        let ders_u = self.basis_functions_derivatives(0, span_u, u, n);
        let ders_v = self.basis_functions_derivatives(1, span_v, v, n);
        let size = self.cv_size();
        let mut skl: Vec<Vec<f64>> = Vec::new();

        for k in 0..=n {
            for m in 0..=n - k {
                let mut sum = vec![0.0; size];

                for i in 0..self.m_order[0] {
                    for j in 0..self.m_order[1] {
                        let c = ders_u[k][i] * ders_v[m][j];
                        let cv_ptr = self.cv(span_u + i, span_v + j).unwrap_or(&[]);

                        for d in 0..size {
                            sum[d] += c * cv_ptr[d];
                        }
                    }
                }

                skl.push(sum);
            }
        }

        if self.m_is_rat {
            return self.rational_derivatives(&skl, n);
        }

        for s in &skl {
            result.push(Vector::new(
                s[0],
                if self.m_dim > 1 { s[1] } else { 0.0 },
                if self.m_dim > 2 { s[2] } else { 0.0 },
            ));
        }

        result
    }

    /// Return the corner CV, None when out of range; u_end and v_end are 0 or 1.
    pub fn point_at_corner(&self, u_end: usize, v_end: usize) -> Option<Point> {
        let i = if u_end == 0 {
            0
        } else {
            self.m_cv_count[0] - 1
        };
        let j = if v_end == 0 {
            0
        } else {
            self.m_cv_count[1] - 1
        };

        self.get_cv(i, j)
    }

    /// Return the iso-curve along dir at the other parameter c, None when invalid; rational surfaces give their exact rational curve.
    pub fn iso_curve(&self, dir: usize, c: f64) -> Option<NurbsCurve> {
        if dir > 1 || !self.is_valid() {
            return None;
        }

        let mut crv = NurbsCurve::new(
            self.m_dim,
            self.m_is_rat,
            self.m_order[dir],
            self.m_cv_count[dir],
        );

        for i in 0..crv.nurbsknot_count() {
            crv.set_nurbsknot(i, self.nurbsknot(dir, i)?);
        }

        let other = 1 - dir;
        let span = self.find_span(other, c);
        let basis = nurbsknot::eval_basis(self.m_order[other], &self.m_nurbsknot[other], span, c);
        let size = self.cv_size();

        for i in 0..self.m_cv_count[dir] {
            let mut sum = vec![0.0; size];

            for k in 0..self.m_order[other] {
                let cv_ptr = if dir == 1 {
                    self.cv(span + k, i)?
                } else {
                    self.cv(i, span + k)?
                };

                for d in 0..size {
                    sum[d] += basis[k] * cv_ptr[d];
                }
            }

            let p = Point::new(
                sum[0],
                if self.m_dim > 1 { sum[1] } else { 0.0 },
                if self.m_dim > 2 { sum[2] } else { 0.0 },
            );

            if self.m_is_rat {
                crv.set_cv_4d(i, p[0], p[1], p[2], sum[self.m_dim]);
            } else {
                crv.set_cv(i, &p);
            }
        }

        Some(crv)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Modifications
    // ═══════════════════════════════════════════════════════════════════════════
    /// Flip the parameterization in dir.
    pub fn reverse(&mut self, dir: usize) -> bool {
        if dir > 1 || !self.is_valid() {
            return false;
        }

        nurbsknot::reverse(
            self.m_order[dir],
            self.m_cv_count[dir],
            &mut self.m_nurbsknot[dir],
        );
        let n = self.m_cv_count[dir];
        let size = self.cv_size();

        for k in 0..self.m_cv_count[1 - dir] {
            for i in 0..n / 2 {
                let (ai, aj) = if dir == 1 { (k, i) } else { (i, k) };
                let (bi, bj) = if dir == 1 {
                    (k, n - 1 - i)
                } else {
                    (n - 1 - i, k)
                };
                let a = ai * self.m_cv_stride[0] + aj * self.m_cv_stride[1];
                let b = bi * self.m_cv_stride[0] + bj * self.m_cv_stride[1];

                for d in 0..size {
                    self.m_cv.swap(a + d, b + d);
                }
            }
        }

        true
    }

    /// Swap u and v.
    pub fn transpose(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let size = self.cv_size();
        let mut new_cv = vec![0.0; self.m_cv.len()];

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let src = self.cv(i, j).unwrap_or(&[]);
                let dst = (j * self.m_cv_count[0] + i) * size;
                new_cv[dst..dst + size].copy_from_slice(src);
            }
        }

        self.m_cv = new_cv;
        self.m_order.swap(0, 1);
        self.m_cv_count.swap(0, 1);
        self.m_nurbsknot.swap(0, 1);
        self.m_cv_stride[0] = size * self.m_cv_count[1];

        true
    }

    /// Swap two coordinate axes in every CV.
    pub fn swap_coordinates(&mut self, axis_i: usize, axis_j: usize) -> bool {
        if axis_i >= self.m_dim || axis_j >= self.m_dim {
            return false;
        }

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(cv_ptr) = self.cv_mut(i, j) {
                    cv_ptr.swap(axis_i, axis_j);
                }
            }
        }

        true
    }

    /// Restrict dir to the sub-domain.
    pub fn trim(&mut self, dir: usize, domain: (f64, f64)) -> bool {
        if dir > 1 || !self.is_valid() {
            return false;
        }

        let mut crv = self.to_curve(dir);

        if !crv.trim(domain.0, domain.1) {
            return false;
        }

        self.from_curve(&crv, dir)
    }

    /// Return two surfaces split at c in dir; both None when c is outside the domain.
    pub fn split(&self, dir: usize, c: f64) -> (Option<Self>, Option<Self>) {
        if dir > 1 || !self.is_valid() {
            return (None, None);
        }

        let Some((t0, t1)) = self.domain(dir) else {
            return (None, None);
        };

        if c <= t0 || c >= t1 {
            return (None, None);
        }

        let mut lo = self.duplicate();
        let mut hi = self.duplicate();

        if !lo.trim(dir, (t0, c)) || !hi.trim(dir, (c, t1)) {
            return (None, None);
        }

        (Some(lo), Some(hi))
    }

    /// Add weights of 1.
    pub fn make_rational(&mut self) -> bool {
        if self.m_is_rat {
            return true;
        }

        let dim = self.m_dim;
        let mut new_cv = vec![0.0; self.cv_count_total() * (dim + 1)];

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let src = self.cv(i, j).unwrap_or(&[]);
                let dst = (i * self.m_cv_count[1] + j) * (dim + 1);
                new_cv[dst..dst + dim].copy_from_slice(src);
                new_cv[dst + dim] = 1.0;
            }
        }

        self.m_cv = new_cv;
        self.m_is_rat = true;
        self.m_cv_stride = [(dim + 1) * self.m_cv_count[1], dim + 1];

        true
    }

    /// Drop weights, dividing each CV by its own.
    pub fn make_non_rational(&mut self) -> bool {
        if !self.m_is_rat {
            return true;
        }

        let dim = self.m_dim;
        let mut new_cv = vec![0.0; self.cv_count_total() * dim];

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let src = self.cv(i, j).unwrap_or(&[]);
                let dst = (i * self.m_cv_count[1] + j) * dim;
                let w = if src[dim].abs() > 1e-14 {
                    src[dim]
                } else {
                    1.0
                };

                for d in 0..dim {
                    new_cv[dst + d] = src[d] / w;
                }
            }
        }

        self.m_cv = new_cv;
        self.m_is_rat = false;
        self.m_cv_stride = [dim * self.m_cv_count[1], dim];

        true
    }

    /// Elevate the degree in dir without changing the shape.
    pub fn increase_degree(&mut self, dir: usize, desired_degree: usize) -> bool {
        if dir > 1 || !self.is_valid() || desired_degree < self.degree(dir) {
            return false;
        }

        if desired_degree == self.degree(dir) {
            return true;
        }

        let mut crv = self.to_curve(dir);

        if !crv.increase_degree(desired_degree) {
            return false;
        }

        self.from_curve(&crv, dir)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Splitting
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the trimmed faces on each side of the plane.
    pub fn split_by_plane(&self, plane: &Plane, tolerance: f64) -> Vec<NurbsSurfaceTrimmed> {
        let mut pcurves = Vec::new();

        for pair in intersection::surface_plane_uv(self, plane, Some(tolerance)) {
            pcurves.push(pair.1);
        }

        NurbsSurfaceTrimmed::split_by_uv_curves(self, &pcurves, tolerance)
    }

    /// Return the trimmed faces cut by curves pulled onto the surface; off-surface curves are skipped.
    pub fn split_by_curves(
        &self,
        curves: &[NurbsCurve],
        tolerance: f64,
    ) -> Vec<NurbsSurfaceTrimmed> {
        let mut pcurves = Vec::new();

        for crv in curves {
            for pcurve in Closest::surface_curve(self, crv, 0.0, 0.0, tolerance) {
                pcurves.push(pcurve);
            }
        }

        NurbsSurfaceTrimmed::split_by_uv_curves(self, &pcurves, tolerance)
    }

    /// Return the trimmed faces cut by a line pulled onto the surface.
    pub fn split_by_line(&self, line: &Line, tolerance: f64) -> Vec<NurbsSurfaceTrimmed> {
        let points = [line.start(), line.end()];

        self.split_by_curves(&[NurbsCurve::create(false, 1, &points)], tolerance)
    }

    /// Return the trimmed faces cut by the surface/surface intersection.
    pub fn split_by_surface(
        &self,
        cutter: &NurbsSurface,
        tolerance: f64,
    ) -> Vec<NurbsSurfaceTrimmed> {
        let mut pcurves = Vec::new();

        for triple in intersection::surface_surface(self, cutter, Some(tolerance)) {
            pcurves.push(triple.1);
        }

        NurbsSurfaceTrimmed::split_by_uv_curves(self, &pcurves, tolerance)
    }

    /// Return the trimmed faces cut by every overlapping face of the brep.
    pub fn split_by_brep(&self, brep: &BRep, tolerance: f64) -> Vec<NurbsSurfaceTrimmed> {
        let target_bb = surface_aabb(self);
        let mut pcurves = Vec::new();

        for cutter in &brep.m_surfaces {
            if !aabb_overlap_pad(&target_bb, &surface_aabb(cutter)) {
                continue;
            }

            for pcurve in intersection::cut_curves_on_surface(self, cutter, Some(tolerance)) {
                pcurves.push(pcurve);
            }
        }

        NurbsSurfaceTrimmed::split_by_uv_curves(self, &pcurves, tolerance)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Meshing
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return m_mesh when cached, else the quadtree subdivision in UV up to depth 8.
    pub fn mesh_adaptive(
        &self,
        max_angle: f64,
        max_edge_length: f64,
        min_edge_length: f64,
        max_chord_height: f64,
    ) -> Mesh {
        if let Some(m) = &self.m_mesh {
            return m.clone();
        }

        if !self.is_valid() {
            return Mesh::new();
        }

        let mut mesher = RemeshNurbsSurfaceAdaptive::new(self.clone());
        mesher
            .set_max_angle(max_angle)
            .set_max_edge_length(max_edge_length)
            .set_min_edge_length(min_edge_length)
            .set_max_chord_height(max_chord_height);

        mesher.mesh()
    }

    /// Return m_mesh when cached, else two triangles for a planar surface or the span grid.
    pub fn mesh(&self) -> Mesh {
        if let Some(m) = &self.m_mesh {
            return m.clone();
        }

        if !self.is_valid() {
            return Mesh::new();
        }

        if self.is_planar(None, 1e-6) {
            return self.mesh_planar();
        }

        RemeshNurbsSurfaceGrid::from_u_v(self, 0, 0)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to a sorted JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_default()
    }

    /// Write to a JSON file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(filepath, json)?;

        Ok(())
    }

    /// Read from a JSON file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let contents = std::fs::read_to_string(filepath)?;

        Ok(serde_json::from_str(&contents)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::NurbsSurface {
        let mut cvs = Vec::new();

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                cvs.extend_from_slice(self.cv(i, j).unwrap_or(&[]));
            }
        }

        let mut cached_mesh = None;

        if let Some(m) = &self.m_mesh {
            if m.number_of_vertices() > 0 {
                cached_mesh = Some(m.to_proto());
            }
        }

        crate::proto::NurbsSurface {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            dimension: self.m_dim as i32,
            is_rational: self.m_is_rat,
            order_u: self.m_order[0] as i32,
            order_v: self.m_order[1] as i32,
            cv_count_u: self.m_cv_count[0] as i32,
            cv_count_v: self.m_cv_count[1] as i32,
            cv_stride_u: self.m_cv_stride[0] as i32,
            cv_stride_v: self.m_cv_stride[1] as i32,
            nurbsknots_u: self.m_nurbsknot[0].clone(),
            nurbsknots_v: self.m_nurbsknot[1].clone(),
            cvs,
            width: self.width,
            pointcolors: colors_to_proto(&self.pointcolors),
            facecolors: colors_to_proto(&self.facecolors),
            linecolors: colors_to_proto(&self.linecolors),
            cached_mesh,
        }
    }

    /// Construct from the protobuf message.
    pub fn from_proto(
        proto: crate::proto::NurbsSurface,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut surface = Self::default();
        let created = surface.create_raw(
            proto.dimension as usize,
            proto.is_rational,
            proto.order_u as usize,
            proto.order_v as usize,
            proto.cv_count_u as usize,
            proto.cv_count_v as usize,
            false,
            false,
            1.0,
            1.0,
        );

        if !proto.guid.is_empty() {
            surface.set_guid(proto.guid.clone());
        }

        surface.name = proto.name;
        surface.width = proto.width;
        surface.pointcolors = colors_from_proto(&proto.pointcolors);
        surface.facecolors = colors_from_proto(&proto.facecolors);
        surface.linecolors = colors_from_proto(&proto.linecolors);

        if let Some(cached) = proto.cached_mesh {
            if !cached.vertices.is_empty() {
                surface.m_mesh = Some(Mesh::from_proto(cached));
            }
        }

        if !created {
            return Ok(surface);
        }

        for i in 0..proto.nurbsknots_u.len().min(surface.m_nurbsknot[0].len()) {
            surface.m_nurbsknot[0][i] = proto.nurbsknots_u[i];
        }

        for i in 0..proto.nurbsknots_v.len().min(surface.m_nurbsknot[1].len()) {
            surface.m_nurbsknot[1][i] = proto.nurbsknots_v[i];
        }

        let size = surface.cv_size();
        let stride_u = if proto.cv_stride_u > 0 {
            proto.cv_stride_u as usize
        } else {
            size * surface.m_cv_count[1]
        };
        let stride_v = if proto.cv_stride_v > 0 {
            proto.cv_stride_v as usize
        } else {
            size
        };

        for i in 0..surface.m_cv_count[0] {
            for j in 0..surface.m_cv_count[1] {
                let src = i * stride_u + j * stride_v;
                let dst = i * surface.m_cv_stride[0] + j * surface.m_cv_stride[1];

                for d in 0..size {
                    if src + d < proto.cvs.len() {
                        surface.m_cv[dst + d] = proto.cvs[src + d];
                    }
                }
            }
        }

        Ok(surface)
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        Self::from_proto(crate::proto::NurbsSurface::decode(data)?)
    }

    /// Write to a protobuf file.
    pub fn pb_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::write(filepath, self.pb_dumps())?;

        Ok(())
    }

    /// Read from a protobuf file.
    pub fn pb_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Self::pb_loads(&std::fs::read(filepath)?)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "NurbsSurface(name=..., degree=(u, v), cvs=(u, v))".
    pub fn str(&self) -> String {
        format!(
            "NurbsSurface(name={}, degree=({},{}), cvs=({},{}))",
            self.name,
            self.degree(0),
            self.degree(1),
            self.m_cv_count[0],
            self.m_cv_count[1]
        )
    }

    /// Return the multi-line form with every control point.
    pub fn repr(&self) -> String {
        let mut result = format!("NurbsSurface(\n  name={},\n  degree=({},{}),\n  cvs=({},{}),\n  rational={},\n  control_points=[\n", self.name, self.degree(0), self.degree(1), self.m_cv_count[0], self.m_cv_count[1], self.m_is_rat);

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let p = self.get_cv(i, j).unwrap_or_default();
                result += &format!("    {}, {}, {}\n", p[0], p[1], p[2]);
            }
        }

        result += "  ]\n)";

        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Private helpers
    // ═══════════════════════════════════════════════════════════════════════════
    /// Zero every CV, false when the layout is unset.
    fn zero_cvs(&mut self) -> bool {
        self.m_cv.fill(0.0);

        if self.m_is_rat {
            let dim = self.m_dim;

            for i in 0..self.m_cv_count[0] {
                for j in 0..self.m_cv_count[1] {
                    if let Some(cv_ptr) = self.cv_mut(i, j) {
                        cv_ptr[dim] = 1.0;
                    }
                }
            }
        }

        true
    }

    /// Fill the nurbsknot vector in dir with clamped uniform values of the given spacing.
    fn make_clamped_uniform_nurbsknot_vector(&mut self, dir: usize, delta: f64) -> bool {
        if dir > 1 || delta <= 0.0 {
            return false;
        }

        self.m_nurbsknot[dir] =
            nurbsknot::make_clamped_uniform(self.m_order[dir], self.m_cv_count[dir], delta);

        !self.m_nurbsknot[dir].is_empty()
    }

    /// Fill the nurbsknot vector in dir with periodic uniform values of the given spacing.
    fn make_periodic_uniform_nurbsknot_vector(&mut self, dir: usize, delta: f64) -> bool {
        if dir > 1 || delta <= 0.0 {
            return false;
        }

        self.m_nurbsknot[dir] =
            nurbsknot::make_periodic_uniform(self.m_order[dir], self.m_cv_count[dir], delta);

        !self.m_nurbsknot[dir].is_empty()
    }

    /// Return the Euclidean point of a homogeneous CV or blend.
    fn dehomogenize(&self, h: &[f64]) -> Point {
        let w = if self.m_is_rat && h[self.m_dim].abs() > 1e-14 {
            h[self.m_dim]
        } else {
            1.0
        };

        Point::new(
            h[0] / w,
            if self.m_dim > 1 { h[1] / w } else { 0.0 },
            if self.m_dim > 2 { h[2] / w } else { 0.0 },
        )
    }

    /// Return the span index in dir containing t.
    fn find_span(&self, dir: usize, t: f64) -> usize {
        nurbsknot::find_span(
            self.m_order[dir],
            self.m_cv_count[dir],
            &self.m_nurbsknot[dir],
            t,
            0,
            0,
        )
    }

    /// Return the basis derivatives ders[k][j] of the order functions on the span (Piegl & Tiller A2.3).
    fn basis_functions_derivatives(
        &self,
        dir: usize,
        span: usize,
        t: f64,
        deriv_order: usize,
    ) -> Vec<Vec<f64>> {
        let order = self.m_order[dir];
        let degree = order - 1;
        let knot = &self.m_nurbsknot[dir];
        let base = span + degree;

        if knot[base - 1] == knot[base] {
            return vec![vec![0.0; order]; deriv_order + 1];
        }

        basis_table_derivatives(&basis_table(knot, degree, base, t), degree, deriv_order)
    }

    /// Apply the rational quotient rule to homogeneous partials in (k, m) loop order (Piegl & Tiller A4.4).
    fn rational_derivatives(&self, skl: &[Vec<f64>], num_derivs: usize) -> Vec<Vector> {
        let mut result: Vec<Vector> = Vec::new();
        let n = num_derivs;
        let w00 = skl[0][self.m_dim];

        if w00.abs() < 1e-14 {
            return vec![Vector::new(0.0, 0.0, 0.0); skl.len()];
        }

        for k in 0..=n {
            for m in 0..=n - k {
                let s = &skl[skl_index(n, k, m)];
                let mut a = Vector::new(
                    s[0],
                    if self.m_dim > 1 { s[1] } else { 0.0 },
                    if self.m_dim > 2 { s[2] } else { 0.0 },
                );

                for i in 0..=k {
                    for j in 0..=m {
                        if i == 0 && j == 0 {
                            continue;
                        }

                        let c =
                            binomial(k, i) * binomial(m, j) * skl[skl_index(n, i, j)][self.m_dim];
                        a -= &result[skl_index(n, k - i, m - j)] * c;
                    }
                }

                result.push(&a / w00);
            }
        }

        result
    }

    /// Run Newton on (n1, n2) . (S - p0) = 0 from (u, v); false when it leaves the domain or stalls.
    fn line_newton(&self, u: &mut f64, v: &mut f64, p0: &Point, n1: &Vector, n2: &Vector) -> bool {
        let (u0, u1) = self.domain(0).unwrap_or((0.0, 0.0));
        let (v0, v1) = self.domain(1).unwrap_or((0.0, 0.0));

        for _ in 0..40 {
            let der = self.evaluate(*u, *v, 1);

            if der.len() < 3 {
                return false;
            }

            let r = Vector::new(der[0][0] - p0[0], der[0][1] - p0[1], der[0][2] - p0[2]);
            let f1 = n1.dot(&r);
            let f2 = n2.dot(&r);

            if f1.abs() < 1e-12 && f2.abs() < 1e-12 {
                return true;
            }

            let j11 = n1.dot(&der[2]);
            let j12 = n1.dot(&der[1]);
            let j21 = n2.dot(&der[2]);
            let j22 = n2.dot(&der[1]);
            let det = j11 * j22 - j12 * j21;

            if det.abs() < 1e-14 {
                return false;
            }

            let du = -(j22 * f1 - j12 * f2) / det;
            let dv = -(-j21 * f1 + j11 * f2) / det;
            *u += du;
            *v += dv;

            if *u < u0 || *u > u1 || *v < v0 || *v > v1 {
                return false;
            }

            if du.abs() < 1e-13 && dv.abs() < 1e-13 {
                return true;
            }
        }

        true
    }

    /// Compute the first and second fundamental forms at (u, v); None at a singular point.
    fn fundamental_forms(&self, u: f64, v: f64) -> Option<(f64, f64, f64, f64, f64, f64)> {
        let d = self.evaluate(u, v, 2);

        if d.len() < 6 {
            return None;
        }

        let sv = &d[1];
        let svv = &d[2];
        let su = &d[3];
        let suv = &d[4];
        let suu = &d[5];
        let cr = su.cross(sv);

        if cr.magnitude() < Tolerance::ZERO_TOLERANCE {
            return None;
        }

        let n = cr.normalized();

        Some((
            su.dot(su),
            su.dot(sv),
            sv.dot(sv),
            suu.dot(&n),
            suv.dot(&n),
            svv.dot(&n),
        ))
    }

    /// Return two triangles through the four corners with one shared normal.
    fn mesh_planar(&self) -> Mesh {
        let mut result = Mesh::new();
        let p00 = self.point_at_corner(0, 0).unwrap_or_default();
        let p10 = self.point_at_corner(1, 0).unwrap_or_default();
        let p11 = self.point_at_corner(1, 1).unwrap_or_default();
        let p01 = self.point_at_corner(0, 1).unwrap_or_default();
        let v0 = result.add_vertex(p00.clone(), None);
        let v1 = result.add_vertex(p10.clone(), None);
        let v2 = result.add_vertex(p11.clone(), None);
        result.add_face(vec![v0, v1, v2], None);
        let mut normal;

        if p00.distance(&p01, None) < 1e-10 {
            let e1 = &p10 - &p00;
            let e2 = &p11 - &p00;
            normal = e1.cross(&e2);
        } else {
            let v3 = result.add_vertex(p01.clone(), None);
            result.add_face(vec![v0, v2, v3], None);
            let derivs = self.evaluate(0.5, 0.5, 1);
            normal = derivs[1].cross(&derivs[2]);
        }

        if normal.magnitude() > 1e-15 {
            normal = normal.normalized();
        }

        for vertex in result.vertex.values_mut() {
            vertex.set_normal(normal[0], normal[1], normal[2]);
        }

        result
    }

    /// Pack the CV rows across dir into one curve along dir with cv_size * cv_count(1 - dir) doubles per CV.
    fn to_curve(&self, dir: usize) -> NurbsCurve {
        let other = 1 - dir;
        let size = self.cv_size();
        let mut crv = NurbsCurve::new(
            size * self.m_cv_count[other],
            false,
            self.m_order[dir],
            self.m_cv_count[dir],
        );

        for i in 0..crv.nurbsknot_count() {
            crv.set_nurbsknot(i, self.m_nurbsknot[dir][i]);
        }

        for i in 0..self.m_cv_count[dir] {
            for j in 0..self.m_cv_count[other] {
                let src = if dir == 1 {
                    self.cv(j, i)
                } else {
                    self.cv(i, j)
                };
                let Some(dst) = crv.cv_mut(i) else { continue };
                dst[j * size..(j + 1) * size].copy_from_slice(src.unwrap_or(&[]));
            }
        }

        crv
    }

    /// Unpack a curve made by to_curve back into this surface along dir.
    fn from_curve(&mut self, crv: &NurbsCurve, dir: usize) -> bool {
        let other = 1 - dir;
        let size = self.cv_size();

        if crv.m_is_rat || crv.m_dim != size * self.m_cv_count[other] {
            return false;
        }

        let mut srf = Self::default();
        let created = if dir == 0 {
            srf.create_raw(
                self.m_dim,
                self.m_is_rat,
                crv.m_order,
                self.m_order[1],
                crv.m_cv_count,
                self.m_cv_count[1],
                false,
                false,
                1.0,
                1.0,
            )
        } else {
            srf.create_raw(
                self.m_dim,
                self.m_is_rat,
                self.m_order[0],
                crv.m_order,
                self.m_cv_count[0],
                crv.m_cv_count,
                false,
                false,
                1.0,
                1.0,
            )
        };

        if !created {
            return false;
        }

        srf.m_nurbsknot[dir] = crv.m_nurbsknot.clone();
        srf.m_nurbsknot[other] = self.m_nurbsknot[other].clone();

        for i in 0..crv.m_cv_count {
            let src = crv.cv(i).unwrap_or(&[]);

            for j in 0..self.m_cv_count[other] {
                let Some(dst) = (if dir == 1 {
                    srf.cv_mut(j, i)
                } else {
                    srf.cv_mut(i, j)
                }) else {
                    continue;
                };
                dst.copy_from_slice(&src[j * size..(j + 1) * size]);
            }
        }

        self.m_order[dir] = srf.m_order[dir];
        self.m_cv_count[dir] = srf.m_cv_count[dir];
        self.m_cv_stride = srf.m_cv_stride;
        self.m_nurbsknot[dir] = srf.m_nurbsknot[dir].clone();
        self.m_cv = srf.m_cv;

        true
    }
}

impl fmt::Display for NurbsSurface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl PartialEq for NurbsSurface {
    /// Compare name, width, colors, layout, nurbsknots and CVs; guid ignored.
    fn eq(&self, other: &Self) -> bool {
        if self.name != other.name || self.width != other.width {
            return false;
        }

        if self.pointcolors != other.pointcolors
            || self.facecolors != other.facecolors
            || self.linecolors != other.linecolors
        {
            return false;
        }

        if self.m_dim != other.m_dim || self.m_is_rat != other.m_is_rat {
            return false;
        }

        if self.m_order != other.m_order
            || self.m_cv_count != other.m_cv_count
            || self.m_cv_stride != other.m_cv_stride
        {
            return false;
        }

        if self.m_nurbsknot != other.m_nurbsknot {
            return false;
        }

        self.m_cv == other.m_cv
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════
impl Serialize for NurbsSurface {
    /// Serializes to flat JSON fields.
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut control_points = Vec::new();

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                control_points.extend_from_slice(self.cv(i, j).unwrap_or(&[]));
            }
        }

        let mut map = serializer.serialize_map(None)?;
        map.serialize_entry("control_points", &control_points)?;
        map.serialize_entry("cv_count_u", &self.m_cv_count[0])?;
        map.serialize_entry("cv_count_v", &self.m_cv_count[1])?;
        map.serialize_entry("dimension", &self.m_dim)?;
        map.serialize_entry("facecolors", &colors_to_json(&self.facecolors))?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("is_rational", &self.m_is_rat)?;
        map.serialize_entry("linecolors", &colors_to_json(&self.linecolors))?;

        if let Some(m) = &self.m_mesh {
            if m.number_of_vertices() > 0 {
                map.serialize_entry("mesh", m)?;
            }
        }

        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("nurbsknots_u", &self.m_nurbsknot[0])?;
        map.serialize_entry("nurbsknots_v", &self.m_nurbsknot[1])?;
        map.serialize_entry("order_u", &self.m_order[0])?;
        map.serialize_entry("order_v", &self.m_order[1])?;
        map.serialize_entry("pointcolors", &colors_to_json(&self.pointcolors))?;
        map.serialize_entry("type", "NurbsSurface")?;
        map.serialize_entry("width", &self.width)?;

        map.end()
    }
}

/// Flat JSON fields of a surface.
#[derive(Deserialize)]
struct NurbsSurfaceData {
    #[serde(default)]
    control_points: Vec<f64>,
    #[serde(default)]
    cv_count_u: usize,
    #[serde(default)]
    cv_count_v: usize,
    #[serde(default)]
    dimension: usize,
    #[serde(default)]
    facecolors: Vec<f32>,
    #[serde(default)]
    guid: String,
    #[serde(default)]
    is_rational: bool,
    #[serde(default)]
    linecolors: Vec<f32>,
    #[serde(default)]
    mesh: Option<Mesh>,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    nurbsknots_u: Vec<f64>,
    #[serde(default)]
    nurbsknots_v: Vec<f64>,
    #[serde(default)]
    order_u: usize,
    #[serde(default)]
    order_v: usize,
    #[serde(default)]
    pointcolors: Vec<f32>,
    #[serde(default)]
    width: Option<f64>,
}

impl<'de> Deserialize<'de> for NurbsSurface {
    /// Deserializes from flat JSON fields.
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let data = NurbsSurfaceData::deserialize(deserializer)?;
        let mut surface = NurbsSurface::default();
        let created = surface.create_raw(
            data.dimension,
            data.is_rational,
            data.order_u,
            data.order_v,
            data.cv_count_u,
            data.cv_count_v,
            false,
            false,
            1.0,
            1.0,
        );

        if !data.guid.is_empty() {
            surface.set_guid(data.guid);
        }

        surface.name = data.name.unwrap_or_else(|| "my_nurbssurface".to_string());
        surface.width = data.width.unwrap_or(1.0);
        surface.pointcolors = colors_from_json(&data.pointcolors);
        surface.facecolors = colors_from_json(&data.facecolors);
        surface.linecolors = colors_from_json(&data.linecolors);
        surface.m_mesh = data.mesh;

        if !created {
            return Ok(surface);
        }

        if !data.nurbsknots_u.is_empty() {
            surface.m_nurbsknot[0] = data.nurbsknots_u;
        }

        if !data.nurbsknots_v.is_empty() {
            surface.m_nurbsknot[1] = data.nurbsknots_v;
        }

        if !data.control_points.is_empty() {
            surface.m_cv = data.control_points;
        }

        Ok(surface)
    }
}
