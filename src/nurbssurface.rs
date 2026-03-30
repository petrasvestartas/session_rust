use crate::point::Point;
use crate::nurbscurve::NurbsCurve;
use crate::xform::Xform;
use crate::color::Color;
use crate::vector::Vector;
use crate::obb::OBB;
use crate::mesh::Mesh;
use crate::plane::Plane;
use crate::knot;
use serde::{Deserialize, Deserializer, Serializer};
use serde::ser::SerializeMap;

/// Non-Uniform Rational B-Spline (NURBS) surface implementation
///
/// Based on OpenNURBS ground truth implementation.
/// Matches the C++ and Python implementations exactly.
#[derive(Debug)]
pub struct NurbsSurface {
    // Metadata
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub width: f64,
    pub pointcolors: Vec<Color>,
    pub facecolors: Vec<Color>,
    pub linecolors: Vec<Color>,
    pub xform: Xform,

    // Core NURBS data
    pub m_dim: usize,
    pub m_is_rat: bool,
    pub m_order: [usize; 2],
    pub m_cv_count: [usize; 2],
    pub m_cv_stride: [usize; 2],
    pub m_knot: [Vec<f64>; 2],
    pub m_cv: Vec<f64>,

    // Cached mesh
    pub m_mesh: Option<Mesh>,
}

impl serde::Serialize for NurbsSurface {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let cv_sz = if self.m_is_rat { self.m_dim + 1 } else { self.m_dim };
        let mut cvs: Vec<f64> = Vec::with_capacity(self.m_cv_count[0] * self.m_cv_count[1] * cv_sz);
        for ci in 0..self.m_cv_count[0] {
            for cj in 0..self.m_cv_count[1] {
                let base = ci * self.m_cv_stride[1] + cj * self.m_cv_stride[0];
                for d in 0..cv_sz {
                    if base + d < self.m_cv.len() {
                        cvs.push(self.m_cv[base + d]);
                    }
                }
            }
        }
        let mut map = serializer.serialize_map(None)?;
        // Alphabetical order
        map.serialize_entry("control_points", &cvs)?;
        map.serialize_entry("cv_count_u", &self.m_cv_count[0])?;
        map.serialize_entry("cv_count_v", &self.m_cv_count[1])?;
        map.serialize_entry("dimension", &self.m_dim)?;
        let facecolors_flat: Vec<u8> = self.facecolors.iter()
            .flat_map(|c| vec![c.r, c.g, c.b, c.a]).collect();
        map.serialize_entry("facecolors", &facecolors_flat)?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("is_rational", &self.m_is_rat)?;
        map.serialize_entry("knots_u", &self.m_knot[0])?;
        map.serialize_entry("knots_v", &self.m_knot[1])?;
        let linecolors_flat: Vec<u8> = self.linecolors.iter()
            .flat_map(|c| vec![c.r, c.g, c.b, c.a]).collect();
        map.serialize_entry("linecolors", &linecolors_flat)?;
        if let Some(ref m) = self.m_mesh {
            if m.number_of_vertices() > 0 {
                map.serialize_entry("mesh", m)?;
            }
        }
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("order_u", &self.m_order[0])?;
        map.serialize_entry("order_v", &self.m_order[1])?;
        let pointcolors_flat: Vec<u8> = self.pointcolors.iter()
            .flat_map(|c| vec![c.r, c.g, c.b, c.a]).collect();
        map.serialize_entry("pointcolors", &pointcolors_flat)?;
        map.serialize_entry("type", "NurbsSurface")?;
        map.serialize_entry("width", &self.width)?;
        map.serialize_entry("xform", &self.xform)?;
        map.end()
    }
}

impl<'de> Deserialize<'de> for NurbsSurface {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct NurbsSurfaceData {
            #[serde(default)]
            guid: Option<String>,
            #[serde(default)]
            name: Option<String>,
            #[serde(default = "default_width")]
            width: f64,
            #[serde(default)]
            pointcolors: Vec<u8>,
            #[serde(default)]
            facecolors: Vec<u8>,
            #[serde(default)]
            linecolors: Vec<u8>,
            #[serde(default)]
            xform: Option<Xform>,
            #[serde(default = "default_dim")]
            dimension: usize,
            #[serde(default)]
            is_rational: bool,
            #[serde(default = "default_order")]
            order_u: usize,
            #[serde(default = "default_order")]
            order_v: usize,
            #[serde(default)]
            cv_count_u: usize,
            #[serde(default)]
            cv_count_v: usize,
            #[serde(default)]
            knots_u: Vec<f64>,
            #[serde(default)]
            knots_v: Vec<f64>,
            #[serde(default)]
            control_points: Vec<f64>,
        }
        fn default_width() -> f64 { 1.0 }
        fn default_dim() -> usize { 3 }
        fn default_order() -> usize { 4 }

        let data = NurbsSurfaceData::deserialize(deserializer)?;
        let cv_sz = if data.is_rational { data.dimension + 1 } else { data.dimension };
        let cv_stride_u = cv_sz;
        let cv_stride_v = cv_sz * data.cv_count_v;

        let pointcolors = data.pointcolors.chunks(4)
            .filter(|c| c.len() == 4)
            .map(|c| Color::new(c[0], c[1], c[2], c[3]))
            .collect();
        let facecolors = data.facecolors.chunks(4)
            .filter(|c| c.len() == 4)
            .map(|c| Color::new(c[0], c[1], c[2], c[3]))
            .collect();
        let linecolors = data.linecolors.chunks(4)
            .filter(|c| c.len() == 4)
            .map(|c| Color::new(c[0], c[1], c[2], c[3]))
            .collect();

        Ok(NurbsSurface {
            guid: { let c = std::sync::OnceLock::new(); let _ = c.set(data.guid.unwrap_or_else(|| uuid::Uuid::new_v4().to_string())); c },
            name: data.name.unwrap_or_else(|| "my_nurbssurface".to_string()),
            width: data.width,
            pointcolors,
            facecolors,
            linecolors,
            xform: data.xform.unwrap_or_else(Xform::identity),
            m_dim: data.dimension,
            m_is_rat: data.is_rational,
            m_order: [data.order_u, data.order_v],
            m_cv_count: [data.cv_count_u, data.cv_count_v],
            m_cv_stride: [cv_stride_u, cv_stride_v],
            m_knot: [data.knots_u, data.knots_v],
            m_cv: data.control_points,
            m_mesh: None,
        })
    }
}

impl NurbsSurface {
    /// Create a new empty NURBS surface
    pub fn new() -> Self {
        NurbsSurface {
            guid: std::sync::OnceLock::new(),
            name: "my_nurbssurface".to_string(),
            width: 1.0,
            pointcolors: Vec::new(),
            facecolors: Vec::new(),
            linecolors: Vec::new(),
            xform: Xform::identity(),
            m_dim: 0,
            m_is_rat: false,
            m_order: [0, 0],
            m_cv_count: [0, 0],
            m_cv_stride: [0, 0],
            m_knot: [Vec::new(), Vec::new()],
            m_cv: Vec::new(),
            m_mesh: None,
        }
    }

    /// Create NURBS surface with specified parameters and optional knot vector initialization
    ///
    /// # Parameters
    /// - `dimension`: Dimension of the surface (typically 3)
    /// - `is_rational`: Whether the surface should be rational
    /// - `order0`: Order in u direction (degree + 1)
    /// - `order1`: Order in v direction (degree + 1)
    /// - `cv_count0`: Number of control vertices in u direction
    /// - `cv_count1`: Number of control vertices in v direction
    /// - `is_periodic_u`: If true, creates periodic uniform knot vector in u direction
    /// - `is_periodic_v`: If true, creates periodic uniform knot vector in v direction
    /// - `knot_delta_u`: Knot spacing in u direction
    /// - `knot_delta_v`: Knot spacing in v direction
    pub fn create_raw(
        dimension: usize,
        is_rational: bool,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
        is_periodic_u: bool,
        is_periodic_v: bool,
        knot_delta_u: f64,
        knot_delta_v: f64,
    ) -> Option<Self> {
        if dimension < 1 || order0 < 2 || order1 < 2
           || cv_count0 < order0 || cv_count1 < order1 {
            return None;
        }

        let mut srf = Self::new();
        srf.m_dim = dimension;
        srf.m_is_rat = is_rational;
        srf.m_order[0] = order0;
        srf.m_order[1] = order1;
        srf.m_cv_count[0] = cv_count0;
        srf.m_cv_count[1] = cv_count1;

        // Calculate CV size and strides
        let cv_size = if is_rational { dimension + 1 } else { dimension };
        srf.m_cv_stride[0] = cv_size;
        srf.m_cv_stride[1] = cv_size * cv_count0;

        // Allocate knot vectors
        let knot_count0 = order0 + cv_count0 - 2;
        let knot_count1 = order1 + cv_count1 - 2;
        srf.m_knot[0] = vec![0.0; knot_count0];
        srf.m_knot[1] = vec![0.0; knot_count1];

        // Allocate CV array
        let cv_size_total = cv_size * cv_count0 * cv_count1;
        srf.m_cv = vec![0.0; cv_size_total];

        // Initialize knot vectors
        // TODO: Add make_periodic_uniform_knot_vector implementation
        if is_periodic_u {
            eprintln!("Warning: Periodic uniform knot vectors not yet implemented in Rust. Using clamped uniform.");
        }
        srf.make_clamped_uniform_knot_vector(0, knot_delta_u);

        if is_periodic_v {
            eprintln!("Warning: Periodic uniform knot vectors not yet implemented in Rust. Using clamped uniform.");
        }
        srf.make_clamped_uniform_knot_vector(1, knot_delta_v);

        Some(srf)
    }

    /// Create NURBS surface from flat list of control points (row-major: u varies slowest)
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
            return Err(format!("NurbsSurface::create: degree must be >= 1, got degree_u={}, degree_v={}", degree_u, degree_v));
        }
        if cv_count_u < degree_u + 1 {
            return Err(format!("NurbsSurface::create: cv_count_u ({}) must be >= degree_u+1 ({})", cv_count_u, degree_u + 1));
        }
        if cv_count_v < degree_v + 1 {
            return Err(format!("NurbsSurface::create: cv_count_v ({}) must be >= degree_v+1 ({})", cv_count_v, degree_v + 1));
        }
        let expected = cv_count_u * cv_count_v;
        if points.len() != expected {
            return Err(format!("NurbsSurface::create: expected {} points ({}x{}), got {}", expected, cv_count_u, cv_count_v, points.len()));
        }
        let order_u = degree_u + 1;
        let order_v = degree_v + 1;
        let mut srf = Self::create_raw(
            3, false, order_u, order_v, cv_count_u, cv_count_v,
            periodic_u, periodic_v, 1.0, 1.0,
        ).ok_or_else(|| format!("NurbsSurface::create: create_raw failed for order=({},{}) cv=({},{})", order_u, order_v, cv_count_u, cv_count_v))?;
        for i in 0..cv_count_u {
            for j in 0..cv_count_v {
                srf.set_cv(i, j, &points[i * cv_count_v + j]);
            }
        }
        Ok(srf)
    }

    /// Create NURBS surface with default knot vectors (clamped uniform, delta=1.0)
    ///
    /// Convenience method for backward compatibility. Equivalent to:
    /// `create(dimension, is_rational, order0, order1, cv_count0, cv_count1, false, false, 1.0, 1.0)`
    pub fn create_simple(
        dimension: usize,
        is_rational: bool,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
    ) -> Option<Self> {
        Self::create_raw(dimension, is_rational, order0, order1, cv_count0, cv_count1,
                    false, false, 1.0, 1.0)
    }

    /// Create clamped uniform NURBS surface
    pub fn create_clamped_uniform(
        &mut self,
        dimension: usize,
        order0: usize,
        order1: usize,
        cv_count0: usize,
        cv_count1: usize,
        knot_delta0: f64,
        knot_delta1: f64,
    ) -> bool {
        // Create surface with given parameters and knot vectors
        let srf = match Self::create_raw(dimension, false, order0, order1, cv_count0, cv_count1,
                                     false, false, knot_delta0, knot_delta1) {
            Some(s) => s,
            None => return false,
        };

        *self = srf;

        true
    }

    fn to_curve_internal(&self, dir: usize) -> Option<NurbsCurve> {
        let dim = self.m_dim;
        let (n_along, n_other) = if dir == 0 {
            (self.m_cv_count[0], self.m_cv_count[1])
        } else {
            (self.m_cv_count[1], self.m_cv_count[0])
        };
        let hdim = dim * n_other;
        let mut crv = NurbsCurve::new(hdim, false, self.m_order[dir], n_along);
        for k in 0..self.knot_count(dir) {
            crv.set_knot(k, self.m_knot[dir][k]);
        }
        for i in 0..n_along {
            let mut cv_data = Vec::with_capacity(hdim);
            for j in 0..n_other {
                let p = if dir == 0 {
                    self.get_cv(i, j)
                } else {
                    self.get_cv(j, i)
                };
                if let Some(pt) = p {
                    cv_data.push(pt[0]);
                    cv_data.push(pt[1]);
                    cv_data.push(pt[2]);
                } else {
                    cv_data.extend_from_slice(&[0.0, 0.0, 0.0]);
                }
            }
            for d in 0..hdim {
                crv.m_cv[i * crv.m_cv_stride + d] = cv_data[d];
            }
        }
        Some(crv)
    }

    fn from_curve_internal(&mut self, crv: &NurbsCurve, dir: usize) -> bool {
        let dim = self.m_dim;
        let n_other = if dir == 0 { self.m_cv_count[1] } else { self.m_cv_count[0] };
        let new_n_along = crv.cv_count();
        let new_order = crv.order();

        let (order0, order1, cv0, cv1) = if dir == 0 {
            (new_order, self.m_order[1], new_n_along, self.m_cv_count[1])
        } else {
            (self.m_order[0], new_order, self.m_cv_count[0], new_n_along)
        };

        let mut new_srf = match Self::create_raw(dim, false, order0, order1, cv0, cv1, false, false, 1.0, 1.0) {
            Some(s) => s,
            None => return false,
        };

        if dir == 0 {
            for k in 0..crv.knot_count() {
                if let Some(kv) = crv.knot(k) {
                    new_srf.set_knot(0, k, kv);
                }
            }
            for k in 0..self.knot_count(1) {
                new_srf.set_knot(1, k, self.m_knot[1][k]);
            }
        } else {
            for k in 0..self.knot_count(0) {
                new_srf.set_knot(0, k, self.m_knot[0][k]);
            }
            for k in 0..crv.knot_count() {
                if let Some(kv) = crv.knot(k) {
                    new_srf.set_knot(1, k, kv);
                }
            }
        }

        for i in 0..new_n_along {
            for j in 0..n_other {
                let base = i * crv.m_cv_stride + j * dim;
                let x = crv.m_cv[base];
                let y = crv.m_cv[base + 1];
                let z = crv.m_cv[base + 2];
                if dir == 0 {
                    new_srf.set_cv(i, j, &Point::new(x, y, z));
                } else {
                    new_srf.set_cv(j, i, &Point::new(x, y, z));
                }
            }
        }

        self.m_order = new_srf.m_order;
        self.m_cv_count = new_srf.m_cv_count;
        self.m_knot = new_srf.m_knot;
        self.m_cv = new_srf.m_cv;
        self.m_cv_stride = new_srf.m_cv_stride;
        true
    }

    pub fn insert_knot(&mut self, dir: usize, knot_value: f64, knot_multiplicity: usize) -> bool {
        if dir > 1 { return false; }
        let mut crv = match self.to_curve_internal(dir) {
            Some(c) => c,
            None => return false,
        };
        for _ in 0..knot_multiplicity {
            if !crv.insert_knot(knot_value, 1) {
                return false;
            }
        }
        self.from_curve_internal(&crv, dir)
    }

    pub fn increase_degree(&mut self, dir: usize, desired_degree: usize) -> bool {
        if dir > 1 { return false; }
        if desired_degree < self.degree(dir) { return false; }
        if desired_degree == self.degree(dir) { return true; }
        let mut crv = match self.to_curve_internal(dir) {
            Some(c) => c,
            None => return false,
        };
        if !crv.increase_degree(desired_degree) {
            return false;
        }
        self.from_curve_internal(&crv, dir)
    }

    fn compute_bbox_diagonal(&self) -> f64 {
        let (mut minx, mut miny, mut minz) = (1e30f64, 1e30f64, 1e30f64);
        let (mut maxx, mut maxy, mut maxz) = (-1e30f64, -1e30f64, -1e30f64);
        for i in 0..self.cv_count_dir(Some(0)) {
            for j in 0..self.cv_count_dir(Some(1)) {
                if let Some(p) = self.get_cv(i, j) {
                    if p[0] < minx { minx = p[0]; }
                    if p[1] < miny { miny = p[1]; }
                    if p[2] < minz { minz = p[2]; }
                    if p[0] > maxx { maxx = p[0]; }
                    if p[1] > maxy { maxy = p[1]; }
                    if p[2] > maxz { maxz = p[2]; }
                }
            }
        }
        let (dx, dy, dz) = (maxx-minx, maxy-miny, maxz-minz);
        (dx*dx + dy*dy + dz*dz).sqrt()
    }

    fn span_subs(&self, dir: usize, sp: &[f64], osp: &[f64], max_angle_deg: f64, bbox_diag: f64) -> Vec<usize> {
        let n = sp.len() - 1;
        let n_other = osp.len() - 1;
        let mut subs = vec![1usize; n];
        let deg_u = self.degree(0);
        let deg_v = self.degree(1);
        let degree_dir = if dir == 0 { deg_u } else { deg_v };
        let s_positions: Vec<f64> = (0..n_other).map(|k| (osp[k] + osp[k + 1]) * 0.5).collect();
        for i in 0..n {
            let t0 = sp[i];
            let t1 = sp[i + 1];
            if degree_dir > 1 {
                let mut max_angle = 0.0f64;
                for &s in &s_positions {
                    let mut prev_n: Option<Vector> = None;
                    let mut total_angle = 0.0f64;
                    for k in 0..=4 {
                        let t = t0 + k as f64 * (t1 - t0) / 4.0;
                        let nv = if dir == 0 { self.normal_at(t, s) } else { self.normal_at(s, t) };
                        if let Some(ref pn) = prev_n {
                            let dot = (pn[0]*nv[0] + pn[1]*nv[1] + pn[2]*nv[2]).clamp(-1.0, 1.0);
                            total_angle += dot.acos() * 180.0 / crate::tolerance::PI;
                        }
                        prev_n = Some(nv);
                    }
                    if total_angle > max_angle { max_angle = total_angle; }
                }
                subs[i] = 1.max((max_angle / max_angle_deg).ceil() as usize).min(24);
            }
            // Direct chord-height deviation check
            {
                let chord_tol = bbox_diag * 0.005;
                let mut max_dev = 0.0f64;
                let nc = n_other.min(3);
                for ci in 0..=nc {
                    let s = osp[0] + ci as f64 * (osp[osp.len()-1] - osp[0]) / nc.max(1) as f64;
                    let pa = if dir == 0 { self.point_at(t0, s) } else { self.point_at(s, t0) }.unwrap_or(Point::new(0.0, 0.0, 0.0));
                    let pb = if dir == 0 { self.point_at(t1, s) } else { self.point_at(s, t1) }.unwrap_or(Point::new(0.0, 0.0, 0.0));
                    for k in 1..=3 {
                        let frac = k as f64 / 4.0;
                        let tm = t0 + frac * (t1 - t0);
                        let pm = if dir == 0 { self.point_at(tm, s) } else { self.point_at(s, tm) }.unwrap_or(Point::new(0.0, 0.0, 0.0));
                        let lx = pa[0] + frac * (pb[0] - pa[0]);
                        let ly = pa[1] + frac * (pb[1] - pa[1]);
                        let lz = pa[2] + frac * (pb[2] - pa[2]);
                        let dx = pm[0] - lx;
                        let dy = pm[1] - ly;
                        let dz = pm[2] - lz;
                        let dev = (dx*dx + dy*dy + dz*dz).sqrt();
                        if dev > max_dev { max_dev = dev; }
                    }
                }
                if max_dev > chord_tol {
                    let chord_subs = 2.max((max_dev / chord_tol).sqrt().ceil() as usize);
                    subs[i] = subs[i].max(chord_subs.min(24));
                }
            }
            if degree_dir > 1 { subs[i] = subs[i].max(2); }
        }
        subs
    }

    pub fn mesh_grid(&self) -> Mesh {
        if let Some(ref m) = self.m_mesh {
            return m.clone();
        }
        if !self.is_valid() {
            return Mesh::new();
        }
        crate::remesh_nurbssurface_grid::RemeshNurbsSurfaceGrid::from_u_v(self.clone(), 0, 0)
    }

    pub fn mesh(&self) -> Mesh {
        if let Some(ref m) = self.m_mesh {
            return m.clone();
        }
        let usp = self.get_span_vector(0);
        let vsp = self.get_span_vector(1);
        if usp.len() < 2 || vsp.len() < 2 {
            return Mesh::new();
        }
        if self.is_planar(1e-6) {
            let mut result = Mesh::new();
            let p00 = self.point_at_corner(0, 0).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let p10 = self.point_at_corner(1, 0).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let p11 = self.point_at_corner(1, 1).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let p01 = self.point_at_corner(0, 1).unwrap_or(Point::new(0.0, 0.0, 0.0));
            let d2 = (p00[0]-p01[0]).powi(2) + (p00[1]-p01[1]).powi(2) + (p00[2]-p01[2]).powi(2);
            let normal;
            if d2 < 1e-20 {
                let v0 = result.add_vertex(p00.clone(), None);
                let v1 = result.add_vertex(p10.clone(), None);
                let v2 = result.add_vertex(p11.clone(), None);
                result.add_face(vec![v0, v1, v2], None);
                let e1 = Vector::new(p10[0]-p00[0], p10[1]-p00[1], p10[2]-p00[2]);
                let e2 = Vector::new(p11[0]-p00[0], p11[1]-p00[1], p11[2]-p00[2]);
                normal = e1.cross(&e2);
            } else {
                let v0 = result.add_vertex(p00.clone(), None);
                let v1 = result.add_vertex(p10.clone(), None);
                let v2 = result.add_vertex(p11.clone(), None);
                let v3 = result.add_vertex(p01.clone(), None);
                result.add_face(vec![v0, v1, v2], None);
                result.add_face(vec![v0, v2, v3], None);
                let derivs = self.evaluate(0.5, 0.5, 1);
                normal = if derivs.len() >= 3 { derivs[1].cross(&derivs[2]) } else { Vector::new(0.0, 0.0, 1.0) };
            }
            let nlen = normal.magnitude();
            let n = if nlen > 1e-15 { &normal * (1.0 / nlen) } else { normal };
            for (_, v) in result.vertex.iter_mut() {
                v.set_normal(n[0], n[1], n[2]);
            }
            return result;
        }
        self.mesh_grid()
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // BOOLEAN QUERIES
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Check if surface is valid
    pub fn is_valid(&self) -> bool {
        if self.m_dim < 1 || self.m_order[0] < 2 || self.m_order[1] < 2 {
            return false;
        }
        if self.m_cv_count[0] < self.m_order[0] || self.m_cv_count[1] < self.m_order[1] {
            return false;
        }
        let cv_size = self.cv_size();
        let required_cv_size = cv_size * self.m_cv_count[0] * self.m_cv_count[1];
        if self.m_cv.len() < required_cv_size {
            return false;
        }
        for dir in 0..2 {
            let knot_count = self.m_order[dir] + self.m_cv_count[dir] - 2;
            if self.m_knot[dir].len() < knot_count {
                return false;
            }
        }
        true
    }

    /// Check if knot vector is valid in specified direction
    pub fn is_valid_knot_vector(&self, dir: usize) -> bool {
        if dir >= 2 { return false; }
        let kc = self.knot_count(dir);
        if self.m_knot[dir].len() != kc { return false; }
        for i in 1..kc {
            if self.m_knot[dir][i] < self.m_knot[dir][i - 1] { return false; }
        }
        true
    }

    /// Check if surface is rational
    pub fn is_rational(&self) -> bool {
        self.m_is_rat
    }

    /// Check if surface is closed in specified direction
    pub fn is_closed(&self, dir: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }

        // Check if first and last rows/columns match
        let tol = 1e-10;
        let cv_size = self.cv_size();

        for i in 0..if dir == 0 { self.m_cv_count[1] } else { self.m_cv_count[0] } {
            let (cv1, cv2) = if dir == 0 {
                (self.cv(0, i), self.cv(self.m_cv_count[0] - 1, i))
            } else {
                (self.cv(i, 0), self.cv(i, self.m_cv_count[1] - 1))
            };

            if let (Some(c1), Some(c2)) = (cv1, cv2) {
                for k in 0..cv_size {
                    if (c1[k] - c2[k]).abs() > tol {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Check if surface is periodic in specified direction
    pub fn is_periodic(&self, dir: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }

        // Check knot vector periodicity
        let order = self.m_order[dir];
        if self.m_knot[dir].len() < order * 2 {
            return false;
        }

        let delta = self.m_knot[dir][order] - self.m_knot[dir][0];
        let tol = 1e-10;

        for i in 0..order {
            let expected = self.m_knot[dir][i] + delta;
            let actual = self.m_knot[dir][i + order];
            if (expected - actual).abs() > tol {
                return false;
            }
        }

        // Must also be closed
        self.is_closed(dir)
    }

    /// Check if surface is planar within tolerance
    pub fn is_planar(&self, tolerance: f64) -> bool {
        if !self.is_valid() || self.m_cv_count[0] < 2 || self.m_cv_count[1] < 2 {
            return false;
        }

        let p0 = match self.get_cv(0, 0) {
            Some(p) => p,
            None => return false,
        };

        // Find three non-colinear CVs to define the plane
        let (mut nx, mut ny, mut nz) = (0.0, 0.0, 0.0);
        let mut n_len = 0.0_f64;
        'outer: for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                for ii in i..self.m_cv_count[0] {
                    let jj_start = if ii == i { j + 1 } else { 0 };
                    for jj in jj_start..self.m_cv_count[1] {
                        if let (Some(pa), Some(pb)) = (self.get_cv(i, j), self.get_cv(ii, jj)) {
                            let (ax, ay, az) = (pa[0]-p0[0], pa[1]-p0[1], pa[2]-p0[2]);
                            let (bx, by, bz) = (pb[0]-p0[0], pb[1]-p0[1], pb[2]-p0[2]);
                            nx = ay*bz - az*by;
                            ny = az*bx - ax*bz;
                            nz = ax*by - ay*bx;
                            n_len = (nx*nx + ny*ny + nz*nz).sqrt();
                            if n_len >= 1e-14 { break 'outer; }
                        }
                    }
                }
            }
        }
        if n_len < 1e-14 {
            return true; // all CVs coincident or colinear
        }

        let nx = nx / n_len;
        let ny = ny / n_len;
        let nz = nz / n_len;

        // Check all CVs are on the plane
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(p) = self.get_cv(i, j) {
                    let dx = p[0] - p0[0];
                    let dy = p[1] - p0[1];
                    let dz = p[2] - p0[2];
                    let dist = (nx * dx + ny * dy + nz * dz).abs();

                    if dist > tolerance {
                        return false;
                    }
                }
            }
        }

        true
    }

    /// Check if a surface side is singular (all CVs along edge coincide)
    /// side: 0=south (v=0), 1=east (u=max), 2=north (v=max), 3=west (u=0)
    pub fn is_singular(&self, side: usize) -> bool {
        if !self.is_valid() { return false; }
        let tol = 1e-10;
        let (count, get_pt): (usize, Box<dyn Fn(usize) -> Option<Point>>) = match side {
            0 => (self.m_cv_count[0], Box::new(|i| self.get_cv(i, 0))),
            1 => (self.m_cv_count[1], Box::new(|j| self.get_cv(self.m_cv_count[0] - 1, j))),
            2 => (self.m_cv_count[0], Box::new(|i| self.get_cv(i, self.m_cv_count[1] - 1))),
            3 => (self.m_cv_count[1], Box::new(|j| self.get_cv(0, j))),
            _ => return false,
        };
        if count < 2 { return true; }
        let first = match get_pt(0) { Some(p) => p, None => return false };
        for k in 1..count {
            if let Some(p) = get_pt(k) {
                let dx = (p[0] - first[0]).abs();
                let dy = (p[1] - first[1]).abs();
                let dz = (p[2] - first[2]).abs();
                if dx > tol || dy > tol || dz > tol { return false; }
            }
        }
        true
    }

    /// Check if surface is clamped in specified direction (at both ends by default)
    /// end: 0=start only, 1=end only, 2=both
    pub fn is_clamped(&self, dir: usize, end: usize) -> bool {
        if dir >= 2 || self.m_knot[dir].is_empty() {
            return false;
        }

        // Use knot module function
        knot::is_clamped(self.m_order[dir], self.m_cv_count[dir], &self.m_knot[dir], end as i32)
    }

    pub fn is_duplicate(&self, other: &Self, ignore_parameterization: bool, tolerance: f64) -> bool {
        if !self.is_valid() || !other.is_valid() { return false; }
        if self.m_dim != other.m_dim { return false; }
        if self.m_is_rat != other.m_is_rat { return false; }
        if self.m_order[0] != other.m_order[0] || self.m_order[1] != other.m_order[1] { return false; }
        if self.m_cv_count[0] != other.m_cv_count[0] || self.m_cv_count[1] != other.m_cv_count[1] { return false; }

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                match (self.get_cv(i, j), other.get_cv(i, j)) {
                    (Some(p1), Some(p2)) => {
                        if p1.distance(&p2, None) > tolerance { return false; }
                    }
                    _ => return false,
                }
                if self.m_is_rat {
                    if (self.weight(i, j) - other.weight(i, j)).abs() > tolerance { return false; }
                }
            }
        }

        if !ignore_parameterization {
            for dir in 0..2 {
                for i in 0..self.knot_count(dir) {
                    match (self.knot(dir, i), other.knot(dir, i)) {
                        (Some(k1), Some(k2)) => {
                            if (k1 - k2).abs() > tolerance { return false; }
                        }
                        _ => return false,
                    }
                }
            }
        }

        true
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // ACCESSORS
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Get dimension
    pub fn dimension(&self) -> usize {
        self.m_dim
    }

    /// Get order (degree + 1) in specified direction
    pub fn order(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        self.m_order[dir]
    }
    
    /// Get degree (order - 1) in specified direction
    pub fn degree(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        if self.m_order[dir] > 0 { self.m_order[dir] - 1 } else { 0 }
    }
    
    /// Get number of control vertices in specified direction (or total if no direction)
    pub fn cv_count_dir(&self, dir: Option<usize>) -> usize {
        match dir {
            None => self.m_cv_count[0] * self.m_cv_count[1],
            Some(d) if d < 2 => self.m_cv_count[d],
            _ => 0,
        }
    }

    /// Get size of each control vertex (dimension + 1 if rational, else dimension)
    pub fn cv_size(&self) -> usize {
        if self.m_is_rat { self.m_dim + 1 } else { self.m_dim }
    }

    /// Get knot count in specified direction
    pub fn knot_count(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        self.m_knot[dir].len()
    }
    
    /// Get number of spans in specified direction
    pub fn span_count(&self, dir: usize) -> usize {
        if dir >= 2 { return 0; }
        if self.m_cv_count[dir] < self.m_order[dir] { return 0; }
        self.m_cv_count[dir] - self.m_order[dir] + 1
    }

    /// Get knot value at index in specified direction
    pub fn knot(&self, dir: usize, index: usize) -> Option<f64> {
        if dir >= 2 || index >= self.m_knot[dir].len() {
            return None;
        }
        Some(self.m_knot[dir][index])
    }

    /// Set knot value at index in specified direction
    pub fn set_knot(&mut self, dir: usize, index: usize, value: f64) -> bool {
        if dir >= 2 || index >= self.m_knot[dir].len() {
            return false;
        }
        self.m_knot[dir][index] = value;
        true
    }

    /// Get pointer to CV data at indices (i, j)
    pub fn cv(&self, i: usize, j: usize) -> Option<&[f64]> {
        if i >= self.m_cv_count[0] || j >= self.m_cv_count[1] {
            return None;
        }
        let index = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
        let cv_size = self.cv_size();
        if index + cv_size > self.m_cv.len() {
            return None;
        }
        Some(&self.m_cv[index..index + cv_size])
    }

    /// Get mutable pointer to CV data at indices (i, j)
    pub fn cv_mut(&mut self, i: usize, j: usize) -> Option<&mut [f64]> {
        if i >= self.m_cv_count[0] || j >= self.m_cv_count[1] {
            return None;
        }
        let index = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
        let cv_size = self.cv_size();
        if index + cv_size > self.m_cv.len() {
            return None;
        }
        Some(&mut self.m_cv[index..index + cv_size])
    }

    /// Get control vertex as Point
    pub fn get_cv(&self, i: usize, j: usize) -> Option<Point> {
        let cv = self.cv(i, j)?;
        if self.m_is_rat && cv.len() > self.m_dim {
            let w = cv[self.m_dim];
            if w.abs() > 1e-14 {
                Some(Point::new(cv[0] / w, cv[1] / w, cv[2] / w))
            } else {
                Some(Point::new(0.0, 0.0, 0.0))
            }
        } else {
            Some(Point::new(
                if cv.len() > 0 { cv[0] } else { 0.0 },
                if cv.len() > 1 { cv[1] } else { 0.0 },
                if cv.len() > 2 { cv[2] } else { 0.0 },
            ))
        }
    }

    /// Get control vertex as homogeneous coordinates (x, y, z, w)
    pub fn get_cv_4d(&self, i: usize, j: usize) -> Option<(f64, f64, f64, f64)> {
        let cv = self.cv(i, j)?;
        let x = if cv.len() > 0 { cv[0] } else { 0.0 };
        let y = if cv.len() > 1 { cv[1] } else { 0.0 };
        let z = if cv.len() > 2 { cv[2] } else { 0.0 };
        let w = if self.m_is_rat && cv.len() > self.m_dim { cv[self.m_dim] } else { 1.0 };
        Some((x, y, z, w))
    }

    /// Set control vertex from homogeneous coordinates (x, y, z, w)
    pub fn set_cv_4d(&mut self, i: usize, j: usize, x: f64, y: f64, z: f64, w: f64) -> bool {
        let is_rat = self.m_is_rat;
        let dim = self.m_dim;
        if let Some(cv) = self.cv_mut(i, j) {
            cv[0] = x;
            if cv.len() > 1 { cv[1] = y; }
            if cv.len() > 2 { cv[2] = z; }
            if is_rat && cv.len() > dim { cv[dim] = w; }
            true
        } else {
            false
        }
    }

    /// Set control vertex from Point
    pub fn set_cv(&mut self, i: usize, j: usize, point: &Point) -> bool {
        let is_rat = self.m_is_rat;
        let dim = self.m_dim;

        if let Some(cv) = self.cv_mut(i, j) {
            if is_rat && cv.len() > dim {
                // For rational surfaces, store homogeneous coordinates (x*w, y*w, z*w, w)
                let mut w = cv[dim];
                if w.abs() < 1e-14 { w = 1.0; }
                cv[0] = point[0] * w;
                if cv.len() > 1 { cv[1] = point[1] * w; }
                if cv.len() > 2 { cv[2] = point[2] * w; }
            } else {
                cv[0] = point[0];
                if cv.len() > 1 { cv[1] = point[1]; }
                if cv.len() > 2 { cv[2] = point[2]; }
            }
            true
        } else {
            false
        }
    }
    
    /// Get weight at control vertex index
    pub fn weight(&self, i: usize, j: usize) -> f64 {
        if !self.m_is_rat {
            return 1.0;
        }
        if let Some(cv) = self.cv(i, j) {
            if cv.len() > self.m_dim {
                return cv[self.m_dim];
            }
        }
        1.0
    }
    
    /// Set weight at control vertex index
    pub fn set_weight(&mut self, i: usize, j: usize, w: f64) -> bool {
        if !self.m_is_rat {
            return false;
        }
        let dim = self.m_dim;
        if let Some(cv) = self.cv_mut(i, j) {
            if cv.len() > dim {
                // Rescale homogeneous coordinates when changing weight
                let mut old_w = cv[dim];
                if old_w.abs() < 1e-14 { old_w = 1.0; }
                let mut new_w = w;
                if new_w.abs() < 1e-14 { new_w = 1.0; }
                let scale = new_w / old_w;
                cv[0] *= scale;
                if cv.len() > 1 { cv[1] *= scale; }
                if cv.len() > 2 { cv[2] *= scale; }
                cv[dim] = w;
                return true;
            }
        }
        false
    }

    /// Make knot vector a clamped uniform knot vector
    /// Matches OpenNURBS algorithm exactly
    pub fn make_clamped_uniform_knot_vector(&mut self, dir: usize, delta: f64) -> bool {
        if dir >= 2 {
            return false;
        }
        if self.m_order[dir] < 2 || self.m_cv_count[dir] < self.m_order[dir] {
            return false;
        }

        // Use knot module function
        let result = knot::make_clamped_uniform(self.m_order[dir], self.m_cv_count[dir], delta);
        if result.is_empty() {
            return false;
        }
        self.m_knot[dir] = result;
        true
    }

    /// Get parameter domain in specified direction
    pub fn domain(&self, dir: usize) -> Option<(f64, f64)> {
        if dir >= 2 {
            return None;
        }
        let order = self.m_order[dir];
        let cv_count = self.m_cv_count[dir];
        if order < 2 || cv_count < order || self.m_knot[dir].len() < order + cv_count - 2 {
            return None;
        }
        Some((self.m_knot[dir][order - 2], self.m_knot[dir][cv_count - 1]))
    }

    /// Set surface domain in specified direction
    pub fn set_domain(&mut self, dir: usize, t0: f64, t1: f64) -> bool {
        if !self.is_valid() || dir >= 2 || t0 >= t1 {
            return false;
        }

        let (d0, d1) = match self.domain(dir) {
            Some(d) => d,
            None => return false,
        };

        if (d1 - d0).abs() < 1e-14 {
            return false;
        }

        let scale = (t1 - t0) / (d1 - d0);
        for i in 0..self.m_knot[dir].len() {
            self.m_knot[dir][i] = t0 + (self.m_knot[dir][i] - d0) * scale;
        }
        true
    }

    /// Get span (distinct knot intervals) values in specified direction
    pub fn get_span_vector(&self, dir: usize) -> Vec<f64> {
        if dir >= 2 || !self.is_valid() {
            return Vec::new();
        }

        let mut spans = Vec::new();
        let tol = 1e-10;

        if self.m_knot[dir].is_empty() {
            return spans;
        }

        spans.push(self.m_knot[dir][0]);

        for i in 1..self.m_knot[dir].len() {
            let diff = self.m_knot[dir][i] - *spans.last().unwrap();
            if diff.abs() > tol {
                spans.push(self.m_knot[dir][i]);
            }
        }

        spans
    }

    /// Get knot multiplicity at index in specified direction
    pub fn knot_multiplicity(&self, dir: usize, knot_index: usize) -> usize {
        if dir >= 2 {
            return 0;
        }
        
        // Use knot module function
        knot::multiplicity(self.m_order[dir], self.m_cv_count[dir], &self.m_knot[dir], knot_index)
    }

    /// Get all knot values for specified direction
    pub fn get_knots(&self, dir: usize) -> Vec<f64> {
        if dir < 2 {
            self.m_knot[dir].clone()
        } else {
            Vec::new()
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // GEOMETRIC QUERIES
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Evaluate point and first derivatives at (u, v)
    /// Returns [point, du, dv] if num_derivs > 0, else [point]
    pub fn evaluate(&self, u: f64, v: f64, num_derivs: usize) -> Vec<Vector> {
        let mut result = Vec::new();
        if !self.is_valid() || num_derivs > 2 { return result; }
        let max_derivs = num_derivs.min(2);

        let span_u = self.find_span(0, u);
        let span_v = self.find_span(1, v);
        if span_u < 0 || span_v < 0 { return result; }
        let span_u = span_u as usize;
        let span_v = span_v as usize;

        let ders_u = self.basis_functions_derivatives(0, span_u, u, max_derivs);
        let ders_v = self.basis_functions_derivatives(1, span_v, v, max_derivs);

        let cv_size_val = if self.m_is_rat { self.m_dim + 1 } else { self.m_dim };

        // Compute all homogeneous derivatives
        let mut skl_all: Vec<(usize, usize, Vec<f64>)> = Vec::new();
        for k in 0..=max_derivs {
            for l in 0..=(max_derivs - k) {
                let mut skl = vec![0.0; cv_size_val];
                for i in 0..self.m_order[0] {
                    let cv_i = span_u + i;
                    for j in 0..self.m_order[1] {
                        let cv_j = span_v + j;
                        let coeff = ders_u[k][i] * ders_v[l][j];
                        if let Some(cv_ptr) = self.cv(cv_i, cv_j) {
                            for d in 0..cv_size_val {
                                skl[d] += coeff * cv_ptr[d];
                            }
                        }
                    }
                }
                skl_all.push((k, l, skl));
            }
        }

        if !self.m_is_rat {
            for (_, _, skl) in &skl_all {
                result.push(Vector::new(
                    skl[0],
                    if self.m_dim > 1 { skl[1] } else { 0.0 },
                    if self.m_dim > 2 { skl[2] } else { 0.0 },
                ));
            }
            return result;
        }

        // Rational: proper quotient rule (NURBS Book A4.2)
        let w00 = skl_all[0].2[self.m_dim];
        if w00.abs() < 1e-14 {
            return vec![Vector::new(0.0, 0.0, 0.0); skl_all.len()];
        }
        let dim = self.m_dim;
        let pt = Vector::new(
            skl_all[0].2[0] / w00,
            if dim > 1 { skl_all[0].2[1] / w00 } else { 0.0 },
            if dim > 2 { skl_all[0].2[2] / w00 } else { 0.0 },
        );
        result.push(pt.clone());

        // Build lookup for weight derivatives
        let mut wders = std::collections::HashMap::new();
        for (k, l, skl) in &skl_all {
            wders.insert((*k, *l), skl[dim]);
        }

        // Cartesian derivatives lookup
        let mut aders: std::collections::HashMap<(usize, usize), Vector> = std::collections::HashMap::new();
        aders.insert((0, 0), pt);

        fn binom(n: usize, k: usize) -> f64 {
            if k > n { return 0.0; }
            let mut r = 1.0;
            for i in 0..k { r = r * (n - i) as f64 / (i + 1) as f64; }
            r
        }

        for idx in 1..skl_all.len() {
            let (k, l, ref skl) = skl_all[idx];
            let mut a = [
                skl[0],
                if dim > 1 { skl[1] } else { 0.0 },
                if dim > 2 { skl[2] } else { 0.0 },
            ];
            for i in 1..=k {
                if let Some(prev) = aders.get(&(k - i, l)) {
                    let c = binom(k, i) * wders.get(&(i, 0)).copied().unwrap_or(0.0);
                    a[0] -= c * prev[0];
                    a[1] -= c * prev[1];
                    a[2] -= c * prev[2];
                }
            }
            for j in 1..=l {
                if let Some(prev) = aders.get(&(k, l - j)) {
                    let c = binom(l, j) * wders.get(&(0, j)).copied().unwrap_or(0.0);
                    a[0] -= c * prev[0];
                    a[1] -= c * prev[1];
                    a[2] -= c * prev[2];
                }
            }
            let v = Vector::new(a[0] / w00, a[1] / w00, a[2] / w00);
            aders.insert((k, l), v.clone());
            result.push(v);
        }

        result
    }

    /// Get normal vector at parameter (u, v)
    pub fn normal_at(&self, u: f64, v: f64) -> Vector {
        let derivs = self.evaluate(u, v, 1);
        if derivs.len() < 3 {
            return Vector::new(0.0, 0.0, 1.0);
        }
        let du = &derivs[1];
        let dv = &derivs[2];
        let n = dv.cross(du);
        if n.magnitude() < 1e-14 {
            Vector::new(0.0, 0.0, 1.0)
        } else {
            n.normalized()
        }
    }

    /// Find span index for parameter value (OpenNURBS algorithm)
    /// 
    /// Matches ON_NurbsSpanIndex from opennurbs_knot.cpp exactly
    fn find_span(&self, dir: usize, t: f64) -> isize {
        if dir >= 2 {
            return -1;
        }

        // Use knot module function
        knot::find_span(self.m_order[dir], self.m_cv_count[dir], &self.m_knot[dir], t) as isize
    }

    /// Compute basis functions (OpenNURBS ON_EvaluateNurbsBasis algorithm)
    fn basis_functions(&self, dir: usize, span_index: usize, t: f64) -> Vec<f64> {
        let order = self.m_order[dir];
        
        if order < 2 {
            return vec![0.0; order];
        }

        let degree = order - 1;  // d = order - 1
        
        // OpenNURBS shifts knot by (order-2) + span, then by d inside basis
        let knot_base = span_index + degree;
        let knot = &self.m_knot[dir];
        
        if knot[knot_base - 1] == knot[knot_base] {
            let mut out = vec![0.0; order];
            if t <= knot[knot_base] { out[0] = 1.0; } else { out[order - 1] = 1.0; }
            return out;
        }

        let mut big_n = vec![0.0; order * order];
        big_n[order * order - 1] = 1.0;

        let mut left = vec![0.0; degree];
        let mut right = vec![0.0; degree];

        let mut n_idx = order * order - 1;
        let mut k_right = knot_base;
        let mut k_left = knot_base - 1;

        for j in 0..degree {
            let n0_idx = n_idx;
            n_idx -= order + 1;
            left[j] = t - knot[k_left];
            right[j] = knot[k_right] - t;
            k_left = k_left.wrapping_sub(1);
            k_right += 1;

            let mut x = 0.0;
            for r in 0..=j {
                let a0 = left[j - r];
                let a1 = right[r];
                let denom = a0 + a1;
                let y = if denom.abs() > 0.0 { big_n[n0_idx + r] / denom } else { 0.0 };
                big_n[n_idx + r] = x + a1 * y;
                x = a0 * y;
            }
            big_n[n_idx + j + 1] = x;
        }
        
        // Return just the final row of basis functions
        big_n[0..order].to_vec()
    }

    fn basis_functions_derivatives(&self, dir: usize, span: usize, t: f64, deriv_order: usize) -> Vec<Vec<f64>> {
        if dir >= 2 { return vec![]; }
        let order = self.m_order[dir];
        let degree = order - 1;
        let knot = &self.m_knot[dir];
        let knot_base = span + degree;

        let mut ders = vec![vec![0.0; order]; deriv_order + 1];

        if knot[knot_base - 1] == knot[knot_base] {
            return ders;
        }

        let mut ndu = vec![vec![0.0; order]; order];
        ndu[0][0] = 1.0;
        let mut left = vec![0.0; degree + 1];
        let mut right = vec![0.0; degree + 1];

        for j in 1..=degree {
            left[j] = t - knot[knot_base - j];
            right[j] = knot[knot_base + j - 1] - t;
            let mut saved = 0.0;
            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }

        for j in 0..=degree {
            ders[0][j] = ndu[j][degree];
        }

        let mut a = vec![vec![0.0; order]; 2];
        for r in 0..=degree {
            let mut s1: usize = 0;
            let mut s2: usize = 1;
            a[0][0] = 1.0;
            for k in 1..=deriv_order {
                let mut d = 0.0;
                let rk = r as isize - k as isize;
                let pk = degree as isize - k as isize;
                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }
                let j1: usize = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2: usize = if (r as isize - 1) <= pk { k - 1 } else { degree - r };
                for j in j1..=j2 {
                    a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[(pk + 1) as usize][(rk + j as isize) as usize];
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

        let mut factorial = degree as f64;
        for k in 1..=deriv_order {
            for j in 0..=degree {
                ders[k][j] *= factorial;
            }
            factorial *= (degree - k) as f64;
        }

        ders
    }

    /// Evaluate point on surface at parameters (u, v)
    /// Matches OpenNURBS EvPoint algorithm
    pub fn point_at(&self, u: f64, v: f64) -> Option<Point> {
        // Find span indices
        let u_span = self.find_span(0, u);
        let v_span = self.find_span(1, v);

        if u_span < 0 || v_span < 0 {
            return None;
        }

        let u_span = u_span as usize;
        let v_span = v_span as usize;

        // Compute basis functions
        let nu = self.basis_functions(0, u_span, u);
        let nv = self.basis_functions(1, v_span, v);

        // Evaluate point using tensor product
        let cv_size = self.cv_size();
        let mut temp = vec![0.0; cv_size];

        let order_u = self.m_order[0];
        let order_v = self.m_order[1];

        for k in 0..order_u {
            for l in 0..order_v {
                let i = u_span + k;
                let j = v_span + l;
                
                if let Some(cv_ptr) = self.cv(i, j) {
                    let weight = nu[k] * nv[l];
                    for m in 0..cv_size {
                        temp[m] += weight * cv_ptr[m];
                    }
                }
            }
        }

        // Convert from homogeneous coordinates if rational
        if self.m_is_rat && temp.len() > self.m_dim {
            let w = temp[self.m_dim];
            if w.abs() > 1e-14 {
                Some(Point::new(temp[0] / w, temp[1] / w, temp[2] / w))
            } else {
                Some(Point::new(0.0, 0.0, 0.0))
            }
        } else {
            Some(Point::new(
                if temp.len() > 0 { temp[0] } else { 0.0 },
                if temp.len() > 1 { temp[1] } else { 0.0 },
                if temp.len() > 2 { temp[2] } else { 0.0 },
            ))
        }
    }

    /// Get point at corner (u_end, v_end) where end is 0 or 1
    pub fn point_at_corner(&self, u_end: usize, v_end: usize) -> Option<Point> {
        let (u0, u1) = self.domain(0)?;
        let (v0, v1) = self.domain(1)?;

        let u = if u_end == 0 { u0 } else { u1 };
        let v = if v_end == 0 { v0 } else { v1 };

        self.point_at(u, v)
    }

    /// Extract isoparametric curve from surface
    /// 
    /// # Arguments
    /// * `dir` - Direction (0=iso-u curve where v varies, 1=iso-v curve where u varies)
    /// * `c` - Parameter value at which to extract the curve
    /// 
    /// # Returns
    /// Option containing the NurbsCurve, or None if invalid
    pub fn iso_curve(&self, dir: usize, c: f64) -> Option<NurbsCurve> {
        if dir >= 2 || !self.is_valid() {
            return None;
        }

        // Create output curve
        let mut nurbs_crv = NurbsCurve::default();
        nurbs_crv.m_dim = self.m_dim;
        nurbs_crv.m_is_rat = self.m_is_rat;
        nurbs_crv.m_order = self.m_order[dir];
        nurbs_crv.m_cv_count = self.m_cv_count[dir];
        
        let cv_size = if self.m_is_rat { self.m_dim + 1 } else { self.m_dim };
        nurbs_crv.m_cv_stride = cv_size;
        
        // Allocate knot vector
        let knot_count = nurbs_crv.m_order + nurbs_crv.m_cv_count - 2;
        nurbs_crv.m_knot = vec![0.0; knot_count];
        
        // Copy knot vector for varying direction
        for i in 0..knot_count {
            nurbs_crv.m_knot[i] = self.m_knot[dir][i];
        }
        
        // Allocate CV array
        nurbs_crv.m_cv = vec![0.0; cv_size * nurbs_crv.m_cv_count];
        
        // Find span in constant direction
        let mut span_index = self.find_span(1 - dir, c);
        if span_index < 0 {
            span_index = 0;
        } else if span_index as usize > self.m_cv_count[1 - dir] - self.m_order[1 - dir] {
            span_index = (self.m_cv_count[1 - dir] - self.m_order[1 - dir]) as isize;
        }
        let span_index = span_index as usize;
        
        // Compute basis functions in constant direction
        let basis = self.basis_functions(1 - dir, span_index, c);
        
        // Evaluate CVs for isocurve
        for i in 0..nurbs_crv.m_cv_count {
            let mut cv_sum = vec![0.0; cv_size];
            
            for k in 0..self.m_order[1 - dir] {
                let (row, col) = if dir == 0 {
                    // iso-u: v varies, u is constant at c
                    (span_index + k, i)
                } else {
                    // iso-v: u varies, v is constant at c
                    (i, span_index + k)
                };
                
                if let Some(cv_ptr) = self.cv(row, col) {
                    for m in 0..cv_size {
                        cv_sum[m] += basis[k] * cv_ptr[m];
                    }
                }
            }
            
            // Set CV in curve
            let cv_index = i * nurbs_crv.m_cv_stride;
            for m in 0..cv_size {
                nurbs_crv.m_cv[cv_index + m] = cv_sum[m];
            }
        }
        
        Some(nurbs_crv)
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    // MODIFICATION OPERATIONS
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Make surface rational (if not already)
    pub fn make_rational(&mut self) -> bool {
        if self.m_is_rat {
            return true; // Already rational
        }
        
        if !self.is_valid() {
            return false;
        }
        
        let old_cv_size = self.m_dim;
        let new_cv_size = self.m_dim + 1;
        let cv_count_total = self.m_cv_count[0] * self.m_cv_count[1];
        
        // Create new CV array with weights
        let mut new_cv = vec![0.0; new_cv_size * cv_count_total];
        
        // Copy existing CVs and add weight=1.0
        for i in 0..cv_count_total {
            for d in 0..self.m_dim {
                new_cv[i * new_cv_size + d] = self.m_cv[i * old_cv_size + d];
            }
            new_cv[i * new_cv_size + self.m_dim] = 1.0; // Set weight
        }
        
        self.m_cv = new_cv;
        self.m_is_rat = true;
        self.m_cv_stride[0] = new_cv_size;
        self.m_cv_stride[1] = new_cv_size * self.m_cv_count[0];

        true
    }

    /// Make surface non-rational if all weights are equal
    pub fn make_non_rational(&mut self) -> bool {
        if !self.m_is_rat { return true; }

        let new_cv_size = self.m_dim;
        let new_stride_0 = new_cv_size;
        let new_stride_1 = new_cv_size * self.m_cv_count[0];
        let total = self.m_cv_count[0] * self.m_cv_count[1] * self.m_dim;
        let mut new_cv = vec![0.0; total];

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let base = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
                let w = self.m_cv[base + self.m_dim];
                let inv_w = if w.abs() > 1e-14 { 1.0 / w } else { 1.0 };
                let dst = i * new_stride_0 + j * new_stride_1;
                for d in 0..self.m_dim {
                    new_cv[dst + d] = self.m_cv[base + d] * inv_w;
                }
            }
        }

        self.m_cv = new_cv;
        self.m_is_rat = false;
        self.m_cv_stride[0] = new_stride_0;
        self.m_cv_stride[1] = new_stride_1;
        true
    }

    /// Reverse surface direction
    pub fn reverse(&mut self, dir: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        let cv_size = self.cv_size();
        
        // Reverse control points in specified direction
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                let (i1, j1) = if dir == 0 {
                    (self.m_cv_count[0] - 1 - i, j)
                } else {
                    (i, self.m_cv_count[1] - 1 - j)
                };
                
                if dir == 0 && i >= (self.m_cv_count[0] + 1) / 2 {
                    break;
                }
                if dir == 1 && j >= (self.m_cv_count[1] + 1) / 2 {
                    break;
                }
                
                // Swap CVs
                let idx1 = i * self.m_cv_stride[0] + j * self.m_cv_stride[1];
                let idx2 = i1 * self.m_cv_stride[0] + j1 * self.m_cv_stride[1];
                
                for k in 0..cv_size {
                    self.m_cv.swap(idx1 + k, idx2 + k);
                }
            }
        }
        
        // Reverse knot vector using knot module function
        knot::reverse(self.m_order[dir], self.m_cv_count[dir], &mut self.m_knot[dir])
    }
    
    /// Transpose surface (swap u and v parameters)
    pub fn transpose(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        // Save original values before swapping
        let old_cv_count_0 = self.m_cv_count[0];
        let old_cv_count_1 = self.m_cv_count[1];
        let old_cv_stride_0 = self.m_cv_stride[0];
        let old_cv_stride_1 = self.m_cv_stride[1];

        // Swap orders
        self.m_order.swap(0, 1);

        // Swap CV counts
        self.m_cv_count.swap(0, 1);

        // Rebuild CV array with transposed indices
        let cv_size = self.cv_size();
        let cv_count_total = self.m_cv_count[0] * self.m_cv_count[1];
        let mut new_cv = vec![0.0; cv_size * cv_count_total];

        // Use OLD counts and strides to read from old array
        for i in 0..old_cv_count_0 {
            for j in 0..old_cv_count_1 {
                let old_idx = i * old_cv_stride_0 + j * old_cv_stride_1;
                // Transpose: old (i,j) becomes new (j,i)
                let new_idx = j * cv_size * self.m_cv_count[1] + i * cv_size;

                for k in 0..cv_size {
                    new_cv[new_idx + k] = self.m_cv[old_idx + k];
                }
            }
        }

        self.m_cv = new_cv;

        // Update strides for new layout
        self.m_cv_stride[0] = cv_size * self.m_cv_count[1];
        self.m_cv_stride[1] = cv_size;

        // Swap knot vectors
        self.m_knot.swap(0, 1);

        true
    }

    /// Swap two coordinate axes
    pub fn swap_coordinates(&mut self, axis_i: usize, axis_j: usize) -> bool {
        if axis_i >= self.m_dim || axis_j >= self.m_dim || axis_i == axis_j {
            return false;
        }
        if !self.is_valid() {
            return false;
        }

        // Swap coordinates in all control vertices
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(cv_slice) = self.cv_mut(i, j) {
                    cv_slice.swap(axis_i, axis_j);
                }
            }
        }
        true
    }

    /// Zero all control vertices (set weights to 1 if rational)
    pub fn zero_cvs(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        // Store values before borrowing
        let dim = self.m_dim;
        let is_rat = self.m_is_rat;

        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(cv_slice) = self.cv_mut(i, j) {
                    // Zero coordinates
                    for k in 0..dim {
                        cv_slice[k] = 0.0;
                    }
                    // Set weight to 1 if rational
                    if is_rat && cv_slice.len() > dim {
                        cv_slice[dim] = 1.0;
                    }
                }
            }
        }
        true
    }

    /// Clamp end in specified direction
    /// end: 0=start, 1=end, 2=both
    pub fn clamp_end(&mut self, dir: usize, end: usize) -> bool {
        if dir >= 2 || !self.is_valid() {
            return false;
        }
        
        // Use knot module function
        knot::clamp(self.m_order[dir], self.m_cv_count[dir], &mut self.m_knot[dir], end as i32)
    }
    
    /// Subdivide surface into a grid of points.
    ///
    /// Evaluates the surface at regular intervals in both parameter directions
    /// to create a grid of points.
    ///
    /// # Arguments
    ///
    /// * `nu` - Number of subdivisions in u direction
    /// * `nv` - Number of subdivisions in v direction
    ///
    /// # Returns
    ///
    /// 2D vector of points, where grid\[i\]\[j\] is the point at subdivision (i, j).
    /// Grid dimensions are (nu+1) x (nv+1).
    pub fn divide_by_count(&self, nu: usize, nv: usize) -> (Vec<Vec<Point>>, Vec<Vec<(f64, f64)>>) {
        let (u0, u1) = match self.domain(0) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new()),
        };
        let (v0, v1) = match self.domain(1) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new()),
        };

        let mut grid = vec![vec![Point::new(0.0, 0.0, 0.0); nv + 1]; nu + 1];
        let mut params = vec![vec![(0.0, 0.0); nv + 1]; nu + 1];

        for i in 0..=nu {
            let u = if nu > 0 {
                u0 + (u1 - u0) * (i as f64 / nu as f64)
            } else {
                u0
            };

            for j in 0..=nv {
                let v = if nv > 0 {
                    v0 + (v1 - v0) * (j as f64 / nv as f64)
                } else {
                    v0
                };

                grid[i][j] = self.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
                params[i][j] = (u, v);
            }
        }

        (grid, params)
    }

    pub fn divide_by_count_points(&self, nu: usize, nv: usize) -> (Vec<Vec<Point>>, Vec<Vec<Vector>>, Vec<Vec<(f64, f64)>>) {
        if !self.is_valid() {
            return (Vec::new(), Vec::new(), Vec::new());
        }

        let (u0, u1) = match self.domain(0) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new(), Vec::new()),
        };
        let (v0, v1) = match self.domain(1) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new(), Vec::new()),
        };

        let mut grid = vec![vec![Point::new(0.0, 0.0, 0.0); nv + 1]; nu + 1];
        let mut grid_vector = vec![vec![Vector::new(0.0, 0.0, 0.0); nv + 1]; nu + 1];
        let mut params = vec![vec![(0.0, 0.0); nv + 1]; nu + 1];

        for i in 0..=nu {
            let u = if nu > 0 {
                u0 + (u1 - u0) * (i as f64 / nu as f64)
            } else {
                u0
            };
            for j in 0..=nv {
                let v = if nv > 0 {
                    v0 + (v1 - v0) * (j as f64 / nv as f64)
                } else {
                    v0
                };
                grid[i][j] = self.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
                grid_vector[i][j] = self.normal_at(u, v);
                params[i][j] = (u, v);
            }
        }

        (grid, grid_vector, params)
    }

    pub fn divide_by_count_planes(&self, nu: usize, nv: usize) -> (Vec<Vec<Plane>>, Vec<Vec<(f64, f64)>>) {
        if !self.is_valid() {
            return (Vec::new(), Vec::new());
        }

        let (u0, u1) = match self.domain(0) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new()),
        };
        let (v0, v1) = match self.domain(1) {
            Some(d) => d,
            None => return (Vec::new(), Vec::new()),
        };

        let mut grid = Vec::with_capacity(nu + 1);
        let mut params = vec![vec![(0.0, 0.0); nv + 1]; nu + 1];

        for i in 0..=nu {
            let u = if nu > 0 {
                u0 + (u1 - u0) * (i as f64 / nu as f64)
            } else {
                u0
            };
            let mut row = Vec::with_capacity(nv + 1);
            for j in 0..=nv {
                let v = if nv > 0 {
                    v0 + (v1 - v0) * (j as f64 / nv as f64)
                } else {
                    v0
                };
                let origin = self.point_at(u, v).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let derivs = self.evaluate(u, v, 1);
                let su = &derivs[2];
                let sv = &derivs[1];
                let mut x_axis = su.clone();
                if x_axis.magnitude() > 1e-14 { x_axis.normalize(); }
                let mut y_axis = sv.clone();
                if y_axis.magnitude() > 1e-14 { y_axis.normalize(); }
                let n = self.normal_at(u, v);
                let plane = Plane::from_axes(origin, x_axis, y_axis, n);
                row.push(plane);
                params[i][j] = (u, v);
            }
            grid.push(row);
        }

        (grid, params)
    }

    /// Get axis-aligned bounding box from control vertices
    pub fn get_bounding_box(&self) -> OBB {
        let mut min_pt = Point::new(f64::MAX, f64::MAX, f64::MAX);
        let mut max_pt = Point::new(f64::MIN, f64::MIN, f64::MIN);
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(pt) = self.get_cv(i, j) {
                    if pt[0] < min_pt[0] { min_pt[0] = pt[0]; }
                    if pt[1] < min_pt[1] { min_pt[1] = pt[1]; }
                    if pt[2] < min_pt[2] { min_pt[2] = pt[2]; }
                    if pt[0] > max_pt[0] { max_pt[0] = pt[0]; }
                    if pt[1] > max_pt[1] { max_pt[1] = pt[1]; }
                    if pt[2] > max_pt[2] { max_pt[2] = pt[2]; }
                }
            }
        }
        let center = Point::new(
            (min_pt[0] + max_pt[0]) * 0.5,
            (min_pt[1] + max_pt[1]) * 0.5,
            (min_pt[2] + max_pt[2]) * 0.5,
        );
        let half_size = Vector::new(
            (max_pt[0] - min_pt[0]) * 0.5,
            (max_pt[1] - min_pt[1]) * 0.5,
            (max_pt[2] - min_pt[2]) * 0.5,
        );
        OBB::new(
            center,
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 1.0),
            half_size,
        )
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // TRANSFORMATION
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Apply stored xform transformation (in-place)
    pub fn transform_self(&mut self) {
        let xf = self.xform.clone();
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(mut pt) = self.get_cv(i, j) {
                    xf.transform_point(&mut pt);
                    self.set_cv(i, j, &pt);
                }
            }
        }
    }
    
    /// Apply custom transformation matrix (in-place)
    pub fn transform(&mut self, xform: &Xform) -> bool {
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(mut pt) = self.get_cv(i, j) {
                    xform.transform_point(&mut pt);
                    if !self.set_cv(i, j, &pt) {
                        return false;
                    }
                }
            }
        }
        true
    }
    
    pub fn transformed(&self, xform: Option<&Xform>) -> Self {
        let mut result = self.clone();
        match xform {
            Some(xf) => { result.transform(xf); }
            None => { result.transform_self(); }
        }
        result
    }

    pub fn make_periodic_uniform_knot_vector(&mut self, dir: usize, delta: f64) -> bool {
        if dir > 1 || delta <= 0.0 { return false; }
        let knots = knot::make_periodic_uniform(self.m_order[dir], self.m_cv_count[dir], delta);
        if knots.is_empty() { return false; }
        self.m_knot[dir] = knots;
        true
    }

    pub fn trim(&mut self, dir: usize, domain: (f64, f64)) -> bool {
        if dir > 1 || !self.is_valid() { return false; }
        let mut crv = match self.to_curve_internal(dir) {
            Some(c) => c,
            None => return false,
        };
        if !crv.trim(domain.0, domain.1) { return false; }
        self.from_curve_internal(&crv, dir)
    }

    pub fn split(&self, dir: usize, c: f64) -> (Option<Self>, Option<Self>) {
        if dir > 1 || !self.is_valid() { return (None, None); }
        let (t0, t1) = match self.domain(dir) {
            Some(d) => d,
            None => return (None, None),
        };
        if c <= t0 || c >= t1 { return (None, None); }
        let mut lo = self.clone();
        let mut hi = self.clone();
        if !lo.trim(dir, (t0, c)) || !hi.trim(dir, (c, t1)) {
            return (None, None);
        }
        (Some(lo), Some(hi))
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // STRING REPRESENTATION
    ///////////////////////////////////////////////////////////////////////////////////////////
    
    /// Get string representation (matches Python to_string format)
    pub fn to_string(&self) -> String {
        format!(
            "NurbsSurface(name={}, degree=({},{}), cvs=({},{}))",
            self.name,
            self.degree(0),
            self.degree(1),
            self.m_cv_count[0],
            self.m_cv_count[1]
        )
    }

    pub fn repr(&self) -> String {
        let mut result = format!("NurbsSurface(\n  name={},\n  degree=({},{}),\n  cvs=({},{}),\n  rational={},\n  control_points=[\n",
            self.name, self.degree(0), self.degree(1), self.m_cv_count[0], self.m_cv_count[1], self.m_is_rat);
        for i in 0..self.m_cv_count[0] {
            for j in 0..self.m_cv_count[1] {
                if let Some(p) = self.get_cv(i, j) {
                    result += &format!("    {}, {}, {}\n", p[0], p[1], p[2]);
                }
            }
        }
        result += "  ]\n)";
        result
    }

    /// Create a duplicate with a new GUID (copies all data except generates new GUID)
    pub fn duplicate(&self) -> Self {
        self.clone()
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Serialize to JSON and write to file
    pub fn json_dump(&self, filename: &str) {
        use std::fs::File;
        use std::io::Write;
        if let Ok(json) = serde_json::to_string_pretty(self) {
            if let Ok(mut file) = File::create(filename) {
                let _ = file.write_all(json.as_bytes());
            }
        }
    }

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        let mut buf = Vec::new();
        let formatter = serde_json::ser::PrettyFormatter::with_indent(b"    ");
        let mut ser = serde_json::Serializer::with_formatter(&mut buf, formatter);
        serde::Serialize::serialize(self, &mut ser)?;
        Ok(String::from_utf8(buf)?)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(json_string: &str) -> Self {
        serde_json::from_str(json_string).unwrap_or_else(|_| Self::default())
    }

    /// Load from JSON file
    pub fn json_load(filename: &str) -> Self {
        use std::fs::File;
        use std::io::Read;
        let mut file = match File::open(filename) {
            Ok(f) => f,
            Err(_) => return Self::default(),
        };
        let mut contents = String::new();
        if file.read_to_string(&mut contents).is_err() {
            return Self::default();
        }
        serde_json::from_str(&contents).unwrap_or_else(|_| Self::default())
    }

    /// Serialize to protobuf binary data.
    ///
    /// # Returns
    ///
    /// A Vec<u8> containing the serialized protobuf data.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        let proto = crate::proto::NurbsSurface {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            dimension: self.m_dim as i32,
            is_rational: self.m_is_rat,
            order_u: self.m_order[0] as i32,
            order_v: self.m_order[1] as i32,
            cv_count_u: self.m_cv_count[0] as i32,
            cv_count_v: self.m_cv_count[1] as i32,
            cv_stride_u: self.m_cv_stride[0] as i32,
            cv_stride_v: self.m_cv_stride[1] as i32,
            knots_u: self.m_knot[0].clone(),
            knots_v: self.m_knot[1].clone(),
            cvs: self.m_cv.clone(),
            width: self.width,
            pointcolors: self.pointcolors.iter().map(|c| crate::proto::Color {
                guid: String::new(), name: String::new(),
                r: c.r as i32, g: c.g as i32, b: c.b as i32, a: c.a as i32,
            }).collect(),
            facecolors: self.facecolors.iter().map(|c| crate::proto::Color {
                guid: String::new(), name: String::new(),
                r: c.r as i32, g: c.g as i32, b: c.b as i32, a: c.a as i32,
            }).collect(),
            linecolors: self.linecolors.iter().map(|c| crate::proto::Color {
                guid: String::new(), name: String::new(),
                r: c.r as i32, g: c.g as i32, b: c.b as i32, a: c.a as i32,
            }).collect(),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid().to_string(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
            cached_mesh: if let Some(ref m) = self.m_mesh {
                if m.number_of_vertices() > 0 {
                    let mesh_data = m.pb_dumps();
                    crate::proto::Mesh::decode(mesh_data.as_slice()).ok()
                } else { None }
            } else { None },
        };
        proto.encode_to_vec()
    }

    /// Create NurbsSurface from protobuf binary data.
    ///
    /// # Arguments
    ///
    /// * `data` - Byte slice containing protobuf-encoded surface data.
    ///
    /// # Returns
    ///
    /// A Result containing the deserialized NurbsSurface or an error.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::NurbsSurface::decode(data)?;

        // Create surface with correct dimensions
        let mut surface = if let Some(srf) = Self::create_raw(
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
        ) {
            srf
        } else {
            return Err("Failed to create NurbsSurface from protobuf".into());
        };

        // Load metadata
        surface.set_guid(proto.guid.clone());
        surface.name = proto.name;
        surface.width = proto.width;

        // Load knot vectors
        if proto.knots_u.len() == surface.m_knot[0].len() {
            surface.m_knot[0] = proto.knots_u;
        }
        if proto.knots_v.len() == surface.m_knot[1].len() {
            surface.m_knot[1] = proto.knots_v;
        }

        // Load control vertices
        if proto.cvs.len() == surface.m_cv.len() {
            surface.m_cv = proto.cvs;
        }

        surface.pointcolors = proto.pointcolors.iter().map(|c| Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8)).collect();
        surface.facecolors = proto.facecolors.iter().map(|c| Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8)).collect();
        surface.linecolors = proto.linecolors.iter().map(|c| Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8)).collect();

        // Load transform
        if let Some(xform) = proto.xform {
            surface.xform.set_guid(xform.guid.clone());
            surface.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    surface.xform.m[i] = *val;
                }
            }
        }

        // Load cached mesh
        if let Some(cached) = proto.cached_mesh {
            use prost::Message as _;
            let mesh_data = cached.encode_to_vec();
            if let Ok(m) = Mesh::pb_loads(&mesh_data) {
                if m.number_of_vertices() > 0 {
                    surface.m_mesh = Some(m);
                }
            }
        }

        Ok(surface)
    }

    /// Write protobuf to file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the output file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf from file.
    ///
    /// # Arguments
    ///
    /// * `filepath` - Path to the protobuf file.
    ///
    /// # Returns
    ///
    /// The deserialized NurbsSurface.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl Default for NurbsSurface {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for NurbsSurface {
    fn eq(&self, other: &Self) -> bool {
        // Compare metadata (excluding guid)
        if self.name != other.name { return false; }
        if self.width != other.width { return false; }
        if self.pointcolors != other.pointcolors { return false; }
        if self.facecolors != other.facecolors { return false; }
        if self.linecolors != other.linecolors { return false; }
        if self.xform != other.xform { return false; }

        // Compare NURBS structure
        if self.m_dim != other.m_dim { return false; }
        if self.m_is_rat != other.m_is_rat { return false; }
        if self.m_order != other.m_order { return false; }
        if self.m_cv_count != other.m_cv_count { return false; }
        if self.m_cv_stride != other.m_cv_stride { return false; }

        // Compare knot vectors
        if self.m_knot[0] != other.m_knot[0] { return false; }
        if self.m_knot[1] != other.m_knot[1] { return false; }

        // Compare control vertices
        if self.m_cv != other.m_cv { return false; }

        true
    }
}

impl Eq for NurbsSurface {}

impl Clone for NurbsSurface {
    fn clone(&self) -> Self {
        NurbsSurface {
            guid: std::sync::OnceLock::new(),
            name: self.name.clone(),
            width: self.width,
            pointcolors: self.pointcolors.clone(),
            facecolors: self.facecolors.clone(),
            linecolors: self.linecolors.clone(),
            xform: self.xform.clone(),
            m_dim: self.m_dim,
            m_is_rat: self.m_is_rat,
            m_order: self.m_order,
            m_cv_count: self.m_cv_count,
            m_cv_stride: self.m_cv_stride,
            m_knot: self.m_knot.clone(),
            m_cv: self.m_cv.clone(),
            m_mesh: self.m_mesh.clone(),
        }
    }
}

impl std::fmt::Display for NurbsSurface {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}

#[cfg(test)]
#[path = "nurbssurface_test.rs"]
mod nurbssurface_test;
