use crate::color::Color;
use crate::nurbsknot;
use crate::plane::Plane;
use crate::point::Point;
use crate::tolerance::Tolerance;
use crate::vector::Vector;
use crate::xform::Xform;
use serde::ser::SerializeMap;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Non-Uniform Rational B-Spline (NURBS) curve implementation
///
/// Based on OpenNURBS ground truth implementation.
/// All methods match the fixed C++ and Python versions.
#[derive(Clone, Debug)]
pub struct NurbsCurve {
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub width: f64,
    pub pointcolors: Vec<Color>,
    pub linecolors: Vec<Color>,
    pub m_dim: usize,
    pub m_is_rat: bool,
    pub m_order: usize,
    pub m_cv_count: usize,
    pub m_cv_stride: usize,
    pub m_nurbsknot: Vec<f64>,
    pub m_cv: Vec<f64>,
}

impl PartialEq for NurbsCurve {
    fn eq(&self, other: &Self) -> bool {
        self.m_dim == other.m_dim
            && self.m_is_rat == other.m_is_rat
            && self.m_order == other.m_order
            && self.m_cv_count == other.m_cv_count
            && self.m_cv_stride == other.m_cv_stride
            && self.m_nurbsknot == other.m_nurbsknot
            && self.m_cv == other.m_cv
    }
}

impl Serialize for NurbsCurve {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(14))?;
        let control_points: Vec<Vec<f64>> = (0..self.m_cv_count)
            .map(|i| {
                // 4D for rational curves: dropping w loses the weights and the reloaded
                // curve is invalid (cv array too short for its stride).
                if self.m_is_rat {
                    match self.get_cv_4d(i) {
                        Some((x, y, z, w)) => vec![x, y, z, w],
                        None => vec![0.0, 0.0, 0.0, 0.0],
                    }
                } else if let Some(p) = self.get_cv(i) {
                    vec![p[0], p[1], p[2]]
                } else {
                    vec![0.0, 0.0, 0.0]
                }
            })
            .collect();
        // Fields in alphabetical order
        map.serialize_entry("control_points", &control_points)?;
        map.serialize_entry("cv_count", &self.m_cv_count)?;
        map.serialize_entry("cv_stride", &self.m_cv_stride)?;
        map.serialize_entry("dimension", &self.m_dim)?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("is_rational", &self.m_is_rat)?;
        map.serialize_entry("nurbsknots", &self.m_nurbsknot)?;
        let linecolors_flat: Vec<u8> = self
            .linecolors
            .iter()
            .flat_map(|c| {
                vec![
                    (c.r * 255.0) as u8,
                    (c.g * 255.0) as u8,
                    (c.b * 255.0) as u8,
                    (c.a * 255.0) as u8,
                ]
            })
            .collect();
        map.serialize_entry("linecolors", &linecolors_flat)?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("order", &self.m_order)?;
        let pointcolors_flat: Vec<u8> = self
            .pointcolors
            .iter()
            .flat_map(|c| {
                vec![
                    (c.r * 255.0) as u8,
                    (c.g * 255.0) as u8,
                    (c.b * 255.0) as u8,
                    (c.a * 255.0) as u8,
                ]
            })
            .collect();
        map.serialize_entry("pointcolors", &pointcolors_flat)?;
        map.serialize_entry("type", "NurbsCurve")?;
        map.serialize_entry("width", &self.width)?;
        map.end()
    }
}

impl<'de> Deserialize<'de> for NurbsCurve {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct NurbsCurveData {
            control_points: Vec<Vec<f64>>,
            cv_count: usize,
            #[serde(default)]
            cv_stride: Option<usize>,
            dimension: usize,
            guid: String,
            is_rational: bool,
            nurbsknots: Vec<f64>,
            #[serde(default)]
            pointcolors: Vec<u8>,
            #[serde(default)]
            linecolors: Vec<u8>,
            name: String,
            order: usize,
            #[serde(default)]
            width: Option<f64>,
        }
        let data = NurbsCurveData::deserialize(deserializer)?;
        let cv_stride = data.cv_stride.unwrap_or_else(|| {
            if data.is_rational {
                data.dimension + 1
            } else {
                data.dimension
            }
        });
        let mut m_cv = Vec::with_capacity(data.cv_count * cv_stride);
        for cp in &data.control_points {
            for j in 0..cv_stride.min(cp.len()) {
                m_cv.push(cp[j]);
            }
            for _ in cp.len()..cv_stride {
                m_cv.push(0.0);
            }
        }
        let pointcolors = data
            .pointcolors
            .chunks(4)
            .filter(|c| c.len() == 4)
            .map(|c| {
                Color::new(
                    c[0] as f32 / 255.0,
                    c[1] as f32 / 255.0,
                    c[2] as f32 / 255.0,
                    c[3] as f32 / 255.0,
                )
            })
            .collect();
        let linecolors = data
            .linecolors
            .chunks(4)
            .filter(|c| c.len() == 4)
            .map(|c| {
                Color::new(
                    c[0] as f32 / 255.0,
                    c[1] as f32 / 255.0,
                    c[2] as f32 / 255.0,
                    c[3] as f32 / 255.0,
                )
            })
            .collect();
        Ok(NurbsCurve {
            guid: {
                let c = std::sync::OnceLock::new();
                let _ = c.set(data.guid);
                c
            },
            name: data.name,
            width: data.width.unwrap_or(1.0),
            pointcolors,
            linecolors,
            m_dim: data.dimension,
            m_is_rat: data.is_rational,
            m_order: data.order,
            m_cv_count: data.cv_count,
            m_cv_stride: cv_stride,
            m_nurbsknot: data.nurbsknots,
            m_cv,
        })
    }
}

impl NurbsCurve {
    // ═══════════════════════════════════════════════════════════════════════════
    // Static Factory Methods
    // ═══════════════════════════════════════════════════════════════════════════

    /// Create NURBS curve from points (unified API)
    ///
    /// # Arguments
    /// * `periodic` - If true, creates a periodic curve; if false, creates a clamped curve
    /// * `degree` - Degree of the curve (order = degree + 1)
    /// * `points` - Control points for the curve
    /// Create a NURBS curve from explicit parameters (OCCT / compas_occt convention:
    /// distinct knots + per-knot multiplicities). Mirrors OCCNurbsCurve.from_parameters and
    /// underlies from_points / from_line / from_circle / from_ellipse. The internal (OpenNURBS)
    /// knot vector is the expanded full knot vector with first and last entries dropped; the
    /// domain becomes [knots.first(), knots.last()].
    pub fn create_from_parameters(
        points: &[Point],
        weights: &[f64],
        knots: &[f64],
        mults: &[usize],
        degree: usize,
        periodic: bool,
    ) -> Self {
        let n = points.len();
        let order = degree + 1;
        if n < order {
            return Self::default();
        }
        if weights.len() != n {
            return Self::default();
        }
        if knots.len() != mults.len() || knots.is_empty() {
            return Self::default();
        }
        if periodic {
            return Self::default();
        } // periodic from_parameters not yet supported

        let rational = weights
            .iter()
            .any(|&w| (w - 1.0).abs() > crate::tolerance::Tolerance::ZERO_TOLERANCE);

        // Expand distinct knots by multiplicity into the full (OCCT-style) knot vector.
        let mut full: Vec<f64> = Vec::new();
        for (i, &v) in knots.iter().enumerate() {
            for _ in 0..mults[i] {
                full.push(v);
            }
        }

        let kc = order + n - 2; // OpenNURBS knot count
        if full.len() != kc + 2 {
            return Self::default();
        } // must equal n + order

        let mut c = Self::new(3, rational, order, n);
        for i in 0..kc {
            c.set_nurbsknot(i, full[i + 1]);
        }
        for i in 0..n {
            if rational {
                let w = weights[i];
                c.set_cv_4d(i, points[i][0] * w, points[i][1] * w, points[i][2] * w, w);
            } else {
                c.set_cv(i, &points[i]);
            }
        }
        c
    }

    pub fn create(periodic: bool, degree: usize, points: &[Point]) -> Self {
        let order = degree + 1;

        let mut curve = if periodic {
            Self::create_periodic_uniform(3, order, points, 1.0)
        } else {
            Self::create_clamped_uniform(3, order, points, 1.0)
        };
        if curve.is_valid() {
            // A degree-1 curve is a polyline: its arc length is the exact sum of segment lengths
            // (plus the closing segment when periodic). Computing it directly avoids the general
            // quadrature length(), which was ~3.5 ms for a 40-point polyline and dominated every
            // polyline build (lift / mesh / split) -- a kernel-wide hot path.
            let l = if degree == 1 {
                let np = points.len();
                let seg = |a: usize, b: usize| {
                    let dx = points[b][0] - points[a][0];
                    let dy = points[b][1] - points[a][1];
                    let dz = points[b][2] - points[a][2];
                    (dx * dx + dy * dy + dz * dz).sqrt()
                };
                let mut s = 0.0;
                for i in 1..np {
                    s += seg(i - 1, i);
                }
                if periodic && np > 1 {
                    s += seg(np - 1, 0);
                }
                s
            } else {
                curve.length(Some(1e-6))
            };
            if l > 0.0 {
                curve.set_domain(0.0, l);
            }
        }
        curve
    }

    /// parameterization maps to Rhino's CurveKnotStyle: Uniform/Chord/ChordSquareRoot
    /// (centripetal). Rhino's CreateInterpolatedCurve(points, degree) API defaults to Uniform;
    /// the InterpCrv command commonly uses Chord. Pass the style explicitly to match Rhino.
    /// Uses Rhino (Bessel) end tangents; for OCCT-matching results use
    /// [`create_interpolated_styled`] with [`nurbsknot::CurveInterpStyle::Occt`].
    pub fn create_interpolated(
        points: &[Point],
        parameterization: nurbsknot::CurveNurbsKnotStyle,
    ) -> NurbsCurve {
        Self::create_interpolated_styled(
            points,
            parameterization,
            nurbsknot::CurveInterpStyle::Rhino,
        )
    }

    /// As [`create_interpolated`], but `end_condition` selects the boundary tangent rule:
    /// Rhino (Bessel) or Occt (cubic Lagrange derivative, reproduces OCCT GeomAPI_Interpolate).
    pub fn create_interpolated_styled(
        points: &[Point],
        parameterization: nurbsknot::CurveNurbsKnotStyle,
        end_condition: nurbsknot::CurveInterpStyle,
    ) -> NurbsCurve {
        let n = points.len();
        if n < 2 {
            return NurbsCurve::new(3, false, 4, 0);
        }
        let dim = 3usize;
        let degree = 3usize;
        let order = degree + 1;

        let periodic = matches!(
            parameterization,
            nurbsknot::CurveNurbsKnotStyle::UniformPeriodic
                | nurbsknot::CurveNurbsKnotStyle::ChordPeriodic
                | nurbsknot::CurveNurbsKnotStyle::ChordSquareRootPeriodic
        );

        if periodic && n < 3 {
            return NurbsCurve::new(3, false, 4, 0);
        }

        // Two points: Rhino emits a degree-1 line (2 CVs), not a cubic.
        if n == 2 && !periodic {
            return NurbsCurve::create(false, 1, points);
        }

        let pdist = |a: &Point, b: &Point| -> f64 {
            let dx = a[0] - b[0];
            let dy = a[1] - b[1];
            let dz = a[2] - b[2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        };

        if periodic {
            let cv_count = n + 3;
            let kc = cv_count + order - 2;

            let base_style = match parameterization {
                nurbsknot::CurveNurbsKnotStyle::UniformPeriodic => {
                    nurbsknot::CurveNurbsKnotStyle::Uniform
                }
                nurbsknot::CurveNurbsKnotStyle::ChordSquareRootPeriodic => {
                    nurbsknot::CurveNurbsKnotStyle::ChordSquareRoot
                }
                _ => nurbsknot::CurveNurbsKnotStyle::Chord,
            };

            let mut params = vec![0.0; n + 1];
            if matches!(base_style, nurbsknot::CurveNurbsKnotStyle::Uniform) {
                for i in 1..=n {
                    params[i] = i as f64;
                }
            } else {
                for i in 1..n {
                    let mut d = pdist(&points[i - 1], &points[i]);
                    if matches!(base_style, nurbsknot::CurveNurbsKnotStyle::ChordSquareRoot) {
                        d = d.sqrt();
                    }
                    params[i] = params[i - 1] + d;
                }
                let mut d_close = pdist(&points[n - 1], &points[0]);
                if matches!(base_style, nurbsknot::CurveNurbsKnotStyle::ChordSquareRoot) {
                    d_close = d_close.sqrt();
                }
                params[n] = params[n - 1] + d_close;
            }

            let mut dmin = f64::INFINITY;
            let mut dmax = 0.0_f64;
            for i in 0..n {
                let d = params[i + 1] - params[i];
                if d < dmin {
                    dmin = d;
                }
                if d > dmax {
                    dmax = d;
                }
            }
            if dmax <= 0.0 || dmax * 1.490116119385e-8 >= dmin {
                return NurbsCurve::new(3, false, 4, 0);
            }

            let mut nurbsknots_vec = vec![0.0; kc];
            for i in 0..=n {
                nurbsknots_vec[i + 2] = params[i];
            }
            nurbsknots_vec[cv_count] =
                nurbsknots_vec[3] - nurbsknots_vec[2] + nurbsknots_vec[cv_count - 1];
            nurbsknots_vec[1] =
                nurbsknots_vec[cv_count - 2] - nurbsknots_vec[cv_count - 1] + nurbsknots_vec[2];
            nurbsknots_vec[cv_count + 1] =
                nurbsknots_vec[4] - nurbsknots_vec[3] + nurbsknots_vec[cv_count];
            nurbsknots_vec[0] =
                nurbsknots_vec[cv_count - 3] - nurbsknots_vec[cv_count - 2] + nurbsknots_vec[1];

            let mut a = vec![vec![0.0; n]; n];
            let mut rhs = vec![0.0; n * dim];

            for i in 0..n {
                let basis = nurbsknot::eval_basis(order, &nurbsknots_vec, i, params[i]);
                let c0 = i % n;
                let c1 = (i + 1) % n;
                let c2 = (i + 2) % n;
                a[i][c0] += basis[0];
                a[i][c1] += basis[1];
                a[i][c2] += basis[2];
                for d in 0..dim {
                    rhs[i * dim + d] = points[i][d];
                }
            }

            let mut cv = vec![0.0; n * dim];
            for i in 0..n {
                for d in 0..dim {
                    cv[i * dim + d] = rhs[i * dim + d];
                }
            }

            for col in 0..n {
                let mut pivot = col;
                for row in (col + 1)..n {
                    if a[row][col].abs() > a[pivot][col].abs() {
                        pivot = row;
                    }
                }
                if pivot != col {
                    a.swap(col, pivot);
                    for d in 0..dim {
                        cv.swap(col * dim + d, pivot * dim + d);
                    }
                }
                if a[col][col].abs() < 1e-300 {
                    return NurbsCurve::new(3, false, 4, 0);
                }
                for row in (col + 1)..n {
                    let factor = a[row][col] / a[col][col];
                    for j in col..n {
                        a[row][j] -= factor * a[col][j];
                    }
                    for d in 0..dim {
                        cv[row * dim + d] -= factor * cv[col * dim + d];
                    }
                }
            }
            for i in (0..n).rev() {
                for d in 0..dim {
                    let mut sum = cv[i * dim + d];
                    for j in (i + 1)..n {
                        sum -= a[i][j] * cv[j * dim + d];
                    }
                    cv[i * dim + d] = sum / a[i][i];
                }
            }

            let mut curve = NurbsCurve::new(dim, false, order, cv_count);
            for i in 0..kc {
                curve.set_nurbsknot(i, nurbsknots_vec[i]);
            }
            for i in 0..n {
                curve.set_cv(i, &Point::new(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));
            }
            let cv0 = curve.get_cv(0).unwrap();
            let cv1 = curve.get_cv(1).unwrap();
            let cv2 = curve.get_cv(2).unwrap();
            curve.set_cv(n, &cv0);
            curve.set_cv(n + 1, &cv1);
            curve.set_cv(n + 2, &cv2);
            return curve;
        }

        // Open interpolation
        let cv_count = n + 2;

        let mut pts = vec![0.0; n * dim];
        for i in 0..n {
            pts[i * 3] = points[i][0];
            pts[i * 3 + 1] = points[i][1];
            pts[i * 3 + 2] = points[i][2];
        }

        let params = nurbsknot::compute_parameters(&pts, dim, parameterization);
        let nurbsknots_vec = nurbsknot::build_interp_nurbsknots(&params, degree);
        let kc = nurbsknots_vec.len();

        let estimate_tangent = |i0: usize, i1: usize, i2: usize| -> Vector {
            let d01 = pdist(&points[i0], &points[i1]);
            let d21 = pdist(&points[i2], &points[i1]);
            if d01 + d21 < 1e-300 {
                return Vector::new(0.0, 0.0, 0.0);
            }
            let s = d01 / (d01 + d21);
            let t = 1.0 - s;
            let denom = 2.0 * s * t;
            if denom < 1e-16 {
                let dx = points[i1][0] - points[i0][0];
                let dy = points[i1][1] - points[i0][1];
                let dz = points[i1][2] - points[i0][2];
                let len = (dx * dx + dy * dy + dz * dz).sqrt();
                return if len > 0.0 {
                    Vector::new(dx / len, dy / len, dz / len)
                } else {
                    Vector::new(0.0, 0.0, 0.0)
                };
            }
            let cvx = (-t * t * points[i0][0] + points[i1][0] - s * s * points[i2][0]) / denom;
            let cvy = (-t * t * points[i0][1] + points[i1][1] - s * s * points[i2][1]) / denom;
            let cvz = (-t * t * points[i0][2] + points[i1][2] - s * s * points[i2][2]) / denom;
            let dx = cvx - points[i0][0];
            let dy = cvy - points[i0][1];
            let dz = cvz - points[i0][2];
            let len = (dx * dx + dy * dy + dz * dz).sqrt();
            if len > 0.0 {
                Vector::new(dx / len, dy / len, dz / len)
            } else {
                Vector::new(0.0, 0.0, 0.0)
            }
        };

        // Un-normalized derivative of the cubic (or quadratic, when n==3) Lagrange
        // polynomial through `m` consecutive points, evaluated at parameter t.
        // Reproduces OCCT GeomAPI_Interpolate::BuildTangents (PLib::EvalLagrange).
        let lagrange_tangent = |i0: usize, m: usize, t: f64| -> Vector {
            let mut res = [0.0f64; 3];
            for j in 0..m {
                let uj = params[i0 + j];
                let mut dsum = 0.0;
                for i in 0..m {
                    if i == j {
                        continue;
                    }
                    let mut term = 1.0 / (uj - params[i0 + i]);
                    for k in 0..m {
                        if k == j || k == i {
                            continue;
                        }
                        term *= (t - params[i0 + k]) / (uj - params[i0 + k]);
                    }
                    dsum += term;
                }
                let pj = &points[i0 + j];
                for d in 0..3 {
                    res[d] += pj[d] * dsum;
                }
            }
            Vector::new(res[0], res[1], res[2])
        };

        let (tan_start, tan_end, s0, s1) =
            if matches!(end_condition, nurbsknot::CurveInterpStyle::Occt) && n >= 3 {
                // OCCT mode: un-normalized Lagrange derivative at the endpoints. The
                // derivative-constraint poles satisfy C'(u0) = 3/(params[1]-params[0])*(P1-P0),
                // so P1 = P0 + (params[1]-params[0])/3 * tan_start (symmetric at the end).
                let deg_t = if n == 3 { 2 } else { 3 };
                let ts = lagrange_tangent(0, deg_t + 1, params[0]);
                let te = lagrange_tangent(n - 1 - deg_t, deg_t + 1, params[n - 1]);
                (
                    ts,
                    te,
                    (params[1] - params[0]) / 3.0,
                    -(params[n - 1] - params[n - 2]) / 3.0,
                )
            } else if n >= 3 {
                let ts = estimate_tangent(0, 1, 2);
                let er = estimate_tangent(n - 1, n - 2, n - 3);
                (
                    ts,
                    Vector::new(-er[0], -er[1], -er[2]),
                    pdist(&points[0], &points[1]) / 3.0,
                    -pdist(&points[n - 1], &points[n - 2]) / 3.0,
                )
            } else {
                let dx = points[1][0] - points[0][0];
                let dy = points[1][1] - points[0][1];
                let dz = points[1][2] - points[0][2];
                let len = (dx * dx + dy * dy + dz * dz).sqrt();
                let v = if len > 0.0 {
                    Vector::new(dx / len, dy / len, dz / len)
                } else {
                    Vector::new(0.0, 0.0, 0.0)
                };
                (
                    v.clone(),
                    v,
                    pdist(&points[0], &points[1]) / 3.0,
                    -pdist(&points[n - 1], &points[n - 2]) / 3.0,
                )
            };

        let mut cv = vec![0.0; cv_count * dim];
        for d in 0..dim {
            cv[d] = points[0][d];
        }
        for d in 0..dim {
            cv[dim + d] = points[0][d] + s0 * tan_start[d];
        }
        for i in 1..n - 1 {
            for d in 0..dim {
                cv[(i + 1) * dim + d] = points[i][d];
            }
        }
        for d in 0..dim {
            cv[n * dim + d] = points[n - 1][d] + s1 * tan_end[d];
        }
        for d in 0..dim {
            cv[(n + 1) * dim + d] = points[n - 1][d];
        }

        let sys_n = n;
        let mut lower = vec![0.0; sys_n];
        let mut diag = vec![0.0; sys_n];
        let mut upper = vec![0.0; sys_n];
        let mut rhs = vec![0.0; sys_n * dim];

        diag[0] = 1.0;
        for d in 0..dim {
            rhs[d] = cv[dim + d];
        }

        for i in 1..n - 1 {
            let basis = nurbsknot::eval_basis(order, &nurbsknots_vec, i, params[i]);
            lower[i] = basis[0];
            diag[i] = basis[1];
            upper[i] = basis[2];
            for d in 0..dim {
                rhs[i * dim + d] = points[i][d];
            }
        }

        diag[n - 1] = 1.0;
        for d in 0..dim {
            rhs[(n - 1) * dim + d] = cv[n * dim + d];
        }

        let solution = match nurbsknot::solve_tridiagonal(dim, &lower, &diag, &upper, &rhs) {
            Some(s) => s,
            None => return NurbsCurve::new(3, false, 4, 0),
        };

        for i in 0..sys_n {
            for d in 0..dim {
                cv[(i + 1) * dim + d] = solution[i * dim + d];
            }
        }

        let mut curve = NurbsCurve::new(dim, false, order, cv_count);
        for i in 0..kc {
            curve.set_nurbsknot(i, nurbsknots_vec[i]);
        }
        for i in 0..cv_count {
            curve.set_cv(i, &Point::new(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));
        }

        curve
    }

    pub fn create_fitted(
        points: &[Point],
        num_cvs: usize,
        degree: usize,
        is_periodic: bool,
    ) -> NurbsCurve {
        let m = points.len();
        let dim = 3;
        let order = degree + 1;

        let pdist = |a: &Point, b: &Point| -> f64 {
            let dx = a[0] - b[0];
            let dy = a[1] - b[1];
            let dz = a[2] - b[2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        };

        if is_periodic {
            let mut n = m;
            if n >= 2 && pdist(&points[0], &points[n - 1]) < 1e-10 {
                n -= 1;
            }
            if n <= num_cvs || num_cvs < order {
                if n < 3 {
                    return NurbsCurve::new(3, false, 4, 0);
                }
                return NurbsCurve::create_interpolated(
                    &points[..n],
                    nurbsknot::CurveNurbsKnotStyle::ChordPeriodic,
                );
            }

            let cv_count = num_cvs + degree;
            let kc = cv_count + order - 2;

            let mut params = vec![0.0; n + 1];
            for i in 1..n {
                params[i] = params[i - 1] + pdist(&points[i - 1], &points[i]);
            }
            params[n] = params[n - 1] + pdist(&points[n - 1], &points[0]);
            let big_t = params[n];
            if big_t < 1e-14 {
                return NurbsCurve::new(3, false, 4, 0);
            }

            let mut ppts = vec![0.0; n * dim];
            for i in 0..n {
                ppts[i * 3] = points[i][0];
                ppts[i * 3 + 1] = points[i][1];
                ppts[i * 3 + 2] = points[i][2];
            }
            let nurbsknots_vec = nurbsknot::build_fitted_nurbsknots_periodic_adaptive(
                &params, &ppts, n, dim, num_cvs, degree, 3.0,
            );

            let mut ntn = vec![vec![0.0; num_cvs]; num_cvs];
            let mut ntq = vec![0.0; num_cvs * dim];

            for k in 0..n {
                let span = nurbsknot::find_span(order, cv_count, &nurbsknots_vec, params[k]);
                let basis = nurbsknot::eval_basis(order, &nurbsknots_vec, span, params[k]);
                for a in 0..order {
                    let ci = (span + a) % num_cvs;
                    for d in 0..dim {
                        ntq[ci * dim + d] += basis[a] * points[k][d];
                    }
                    for b in 0..order {
                        let cj = (span + b) % num_cvs;
                        ntn[ci][cj] += basis[a] * basis[b];
                    }
                }
            }

            let mut cv = ntq.clone();

            for col in 0..num_cvs {
                let mut pivot = col;
                for row in (col + 1)..num_cvs {
                    if ntn[row][col].abs() > ntn[pivot][col].abs() {
                        pivot = row;
                    }
                }
                if pivot != col {
                    ntn.swap(col, pivot);
                    for d in 0..dim {
                        cv.swap(col * dim + d, pivot * dim + d);
                    }
                }
                if ntn[col][col].abs() < 1e-300 {
                    return NurbsCurve::new(3, false, 4, 0);
                }
                for row in (col + 1)..num_cvs {
                    let factor = ntn[row][col] / ntn[col][col];
                    for j in col..num_cvs {
                        ntn[row][j] -= factor * ntn[col][j];
                    }
                    for d in 0..dim {
                        cv[row * dim + d] -= factor * cv[col * dim + d];
                    }
                }
            }
            for i in (0..num_cvs).rev() {
                for d in 0..dim {
                    let mut s = cv[i * dim + d];
                    for j in (i + 1)..num_cvs {
                        s -= ntn[i][j] * cv[j * dim + d];
                    }
                    cv[i * dim + d] = s / ntn[i][i];
                }
            }

            let mut curve = NurbsCurve::new(dim, false, order, cv_count);
            for i in 0..kc {
                curve.set_nurbsknot(i, nurbsknots_vec[i]);
            }
            for i in 0..num_cvs {
                curve.set_cv(i, &Point::new(cv[i * 3], cv[i * 3 + 1], cv[i * 3 + 2]));
            }
            for i in 0..degree {
                let p = curve.get_cv(i).unwrap();
                curve.set_cv(num_cvs + i, &p);
            }
            return curve;
        }

        // Open fitting
        if m <= num_cvs || num_cvs < order {
            return NurbsCurve::create_interpolated(points, nurbsknot::CurveNurbsKnotStyle::Chord);
        }

        let mut pts = vec![0.0; m * dim];
        for i in 0..m {
            pts[i * 3] = points[i][0];
            pts[i * 3 + 1] = points[i][1];
            pts[i * 3 + 2] = points[i][2];
        }

        let params =
            nurbsknot::compute_parameters(&pts, dim, nurbsknot::CurveNurbsKnotStyle::Chord);
        let nurbsknots_vec =
            nurbsknot::build_fitted_nurbsknots_adaptive(&params, &pts, dim, num_cvs, degree, 3.0);
        let n = num_cvs - 1;
        let sys_n = num_cvs - 2;
        let bw = degree;
        let bw1 = bw + 1;

        let mut band = vec![0.0; sys_n * bw1];
        let mut rhs = vec![0.0; sys_n * dim];

        for k in 1..(m - 1) {
            let span = nurbsknot::find_span(order, num_cvs, &nurbsknots_vec, params[k]);
            let basis = nurbsknot::eval_basis(order, &nurbsknots_vec, span, params[k]);

            let mut rk = [points[k][0], points[k][1], points[k][2]];
            for a in 0..order {
                let ci = span + a;
                if ci == 0 {
                    for d in 0..dim {
                        rk[d] -= basis[a] * points[0][d];
                    }
                }
                if ci == n {
                    for d in 0..dim {
                        rk[d] -= basis[a] * points[m - 1][d];
                    }
                }
            }

            for a in 0..order {
                let ci = span + a;
                if ci < 1 || ci > n - 1 {
                    continue;
                }
                let ri = ci - 1;
                for d in 0..dim {
                    rhs[ri * dim + d] += basis[a] * rk[d];
                }
                for b in a..order {
                    let cj = span + b;
                    if cj < 1 || cj > n - 1 {
                        continue;
                    }
                    let rj = cj - 1;
                    band[rj * bw1 + (rj - ri)] += basis[a] * basis[b];
                }
            }
        }

        if !nurbsknot::solve_banded_spd(dim, sys_n, bw, &mut band, &mut rhs) {
            return NurbsCurve::create_interpolated(points, nurbsknot::CurveNurbsKnotStyle::Chord);
        }

        let mut curve = NurbsCurve::new(dim, false, order, num_cvs);
        for i in 0..nurbsknots_vec.len() {
            curve.set_nurbsknot(i, nurbsknots_vec[i]);
        }
        curve.set_cv(0, &points[0]);
        for i in 0..sys_n {
            curve.set_cv(
                i + 1,
                &Point::new(rhs[i * 3], rhs[i * 3 + 1], rhs[i * 3 + 2]),
            );
        }
        curve.set_cv(n, &points[m - 1]);
        curve
    }

    /// Join curve segments into chains by endpoint matching.
    ///
    /// Segments are greedily chained (reversed as needed), made compatible
    /// (common degree, common rationality), and concatenated with C0
    /// continuity (junction nurbsknot at multiplicity = degree).
    pub fn join(curves: &[NurbsCurve], tolerance: Option<f64>) -> Vec<NurbsCurve> {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        let mut segs: Vec<NurbsCurve> = Vec::new();
        for c in curves {
            if c.is_valid() {
                segs.push(c.duplicate());
            }
        }
        let mut chains: Vec<Vec<NurbsCurve>> = Vec::new();
        let mut used = vec![false; segs.len()];
        for i in 0..segs.len() {
            if used[i] {
                continue;
            }
            used[i] = true;
            let mut chain: Vec<NurbsCurve> = vec![segs[i].duplicate()];
            if !segs[i].is_closed() {
                let mut grown = true;
                while grown {
                    grown = false;
                    let start = chain[0].point_at_start();
                    let end = chain[chain.len() - 1].point_at_end();
                    for j in 0..segs.len() {
                        if used[j] || segs[j].is_closed() {
                            continue;
                        }
                        let s = segs[j].point_at_start();
                        let e = segs[j].point_at_end();
                        if s.distance(&end, None) <= tol {
                            chain.push(segs[j].duplicate());
                        } else if e.distance(&end, None) <= tol {
                            let mut r = segs[j].duplicate();
                            r.reverse();
                            chain.push(r);
                        } else if e.distance(&start, None) <= tol {
                            chain.insert(0, segs[j].duplicate());
                        } else if s.distance(&start, None) <= tol {
                            let mut r = segs[j].duplicate();
                            r.reverse();
                            chain.insert(0, r);
                        } else {
                            continue;
                        }
                        used[j] = true;
                        grown = true;
                        break;
                    }
                }
            }
            chains.push(chain);
        }
        let mut result: Vec<NurbsCurve> = Vec::new();
        for mut chain in chains {
            if chain.len() == 1 {
                result.push(chain.remove(0));
                continue;
            }
            let mut rational = false;
            let mut max_degree = 1;
            for c in &chain {
                if c.is_rational() {
                    rational = true;
                }
                if c.degree() > max_degree {
                    max_degree = c.degree();
                }
            }
            for c in chain.iter_mut() {
                if rational {
                    c.make_rational();
                }
                c.clamp_end(2);
                c.increase_degree(max_degree);
            }
            let mut joined = chain.remove(0);
            for mut c in chain {
                let stride = joined.m_cv_stride;
                let cvdim = joined.cv_size();
                let (_, a1) = joined.domain();
                let (s0, s1) = c.domain();
                c.set_domain(a1, a1 + (s1 - s0));
                if rational {
                    let w_end = joined.weight(joined.m_cv_count - 1);
                    let w_start = c.weight(0);
                    if w_start.abs() > Tolerance::ZERO_TOLERANCE {
                        let scale = w_end / w_start;
                        for k in 0..c.m_cv.len() {
                            c.m_cv[k] = c.m_cv[k] * scale;
                        }
                    }
                }
                let last = (joined.m_cv_count - 1) * stride;
                for k in 0..cvdim {
                    joined.m_cv[last + k] = 0.5 * (joined.m_cv[last + k] + c.m_cv[k]);
                }
                joined
                    .m_nurbsknot
                    .extend_from_slice(&c.m_nurbsknot[joined.m_order - 1..]);
                joined.m_cv.extend_from_slice(&c.m_cv[stride..]);
                joined.m_cv_count = joined.m_cv_count + c.m_cv_count - 1;
            }
            result.push(joined);
        }
        result
    }

    /// Create clamped uniform NURBS curve from control points
    ///
    /// Implementation matches OpenNURBS ON_MakeClampedUniformNurbsKnotVector exactly.
    pub fn create_clamped_uniform(
        dimension: usize,
        order: usize,
        points: &[Point],
        nurbsknot_delta: f64,
    ) -> Self {
        let point_count = points.len();

        if order < 2 || point_count < order {
            return Self::default();
        }

        let mut curve = Self::default();
        if !curve.initialize_curve(dimension, false, order, point_count) {
            return Self::default();
        }

        // Set control points
        for (i, point) in points.iter().enumerate() {
            curve.set_cv(i, point);
        }

        // Create clamped uniform nurbsknot vector - matches OpenNURBS exactly
        let nurbsknot_count = order + point_count - 2;

        // Fill interior nurbsknots with uniform spacing
        // Start from index (order-2) up to (cv_count-1)
        let mut k = 0.0;
        for i in (order - 2)..point_count {
            curve.m_nurbsknot[i] = k;
            k += nurbsknot_delta;
        }

        // Clamp both ends: sets first (order-2) and last (order-2) nurbsknots
        // Left clamp: nurbsknot[0..order-3] = nurbsknot[order-2]
        let i0 = order - 2;
        for i in 0..i0 {
            curve.m_nurbsknot[i] = curve.m_nurbsknot[i0];
        }

        // Right clamp: nurbsknot[cv_count..nurbsknot_count-1] = nurbsknot[cv_count-1]
        let i0 = point_count - 1;
        for i in (i0 + 1)..nurbsknot_count {
            curve.m_nurbsknot[i] = curve.m_nurbsknot[i0];
        }

        curve
    }

    /// Create periodic uniform NURBS curve from control points
    pub fn create_periodic_uniform(
        dimension: usize,
        order: usize,
        points: &[Point],
        nurbsknot_delta: f64,
    ) -> Self {
        let point_count = points.len();

        if order < 2 || point_count < order {
            return Self::default();
        }

        let mut curve = Self::default();
        let cv_count = point_count + order - 1;

        if !curve.initialize_curve(dimension, false, order, cv_count) {
            return Self::default();
        }

        // Set control points with wrapping
        for (i, point) in points.iter().enumerate() {
            curve.set_cv(i, point);
        }

        // Wrap control points for periodicity
        for i in 0..(order - 1) {
            let idx = i % point_count;
            curve.set_cv(point_count + i, &points[idx]);
        }

        // Create uniform nurbsknot vector
        let nurbsknot_count = order + cv_count - 2;
        for i in 0..nurbsknot_count {
            curve.m_nurbsknot[i] = i as f64 * nurbsknot_delta;
        }

        curve
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors & Destructor
    // ═══════════════════════════════════════════════════════════════════════════

    /// Create a NURBS curve with specified parameters (matches C++/Python constructor)
    pub fn new(dimension: usize, is_rational: bool, order: usize, cv_count: usize) -> Self {
        let cv_stride = if is_rational {
            dimension + 1
        } else {
            dimension
        };
        let nurbsknot_count = if order > 0 && cv_count >= order {
            order + cv_count - 2
        } else {
            0
        };

        NurbsCurve {
            guid: std::sync::OnceLock::new(),
            name: "my_nurbscurve".to_string(),
            width: 1.0,
            pointcolors: Vec::new(),
            linecolors: Vec::new(),
            m_dim: dimension,
            m_is_rat: is_rational,
            m_order: order,
            m_cv_count: cv_count,
            m_cv_stride: cv_stride,
            m_nurbsknot: vec![0.0; nurbsknot_count],
            m_cv: vec![0.0; cv_count * cv_stride],
        }
    }

    /// Create an empty NURBS curve (default constructor)
    pub fn default() -> Self {
        NurbsCurve {
            guid: std::sync::OnceLock::new(),
            name: "my_nurbscurve".to_string(),
            width: 1.0,
            pointcolors: Vec::new(),
            linecolors: Vec::new(),
            m_dim: 0,
            m_is_rat: false,
            m_order: 0,
            m_cv_count: 0,
            m_cv_stride: 0,
            m_nurbsknot: Vec::new(),
            m_cv: Vec::new(),
        }
    }

    /// Create a duplicate with new GUID
    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = std::sync::OnceLock::new();
        copy
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    /// Clear the guid so a FRESH one mints lazily on next read — the duplicate/copy enabler.
    pub fn refresh_guid(&mut self) {
        self.guid = std::sync::OnceLock::new();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Initialization & Creation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Initialize curve with specified parameters
    fn initialize_curve(
        &mut self,
        dimension: usize,
        is_rational: bool,
        order: usize,
        cv_count: usize,
    ) -> bool {
        if dimension < 1 || order < 2 || cv_count < order {
            return false;
        }

        self.m_dim = dimension;
        self.m_is_rat = is_rational;
        self.m_order = order;
        self.m_cv_count = cv_count;
        self.m_cv_stride = if is_rational {
            dimension + 1
        } else {
            dimension
        };

        let nurbsknot_count = order + cv_count - 2;
        self.m_nurbsknot = vec![0.0; nurbsknot_count];
        self.m_cv = vec![0.0; cv_count * self.m_cv_stride];

        // Initialize weights to 1.0 for rational curves
        if is_rational {
            for i in 0..cv_count {
                self.m_cv[i * self.m_cv_stride + dimension] = 1.0;
            }
        }

        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Boolean Queries
    // ═══════════════════════════════════════════════════════════════════════════

    /// Check if curve is valid
    pub fn is_valid(&self) -> bool {
        if self.m_order < 2 || self.m_cv_count < self.m_order {
            return false;
        }
        if self.m_nurbsknot.len() != self.m_order + self.m_cv_count - 2 {
            return false;
        }
        // Check for sufficient distinct nurbsknots
        if self.m_order >= 2 && self.m_cv_count >= self.m_order {
            let idx1 = self.m_order - 2;
            let idx2 = self.m_cv_count - 1;
            if idx2 < self.m_nurbsknot.len() && self.m_nurbsknot[idx1] >= self.m_nurbsknot[idx2] {
                return false;
            }
        }
        true
    }

    /// Check if curve is rational
    pub fn is_rational(&self) -> bool {
        self.m_is_rat
    }

    /// Check if curve is closed (start point == end point)
    pub fn is_closed(&self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let start = self.point_at_start();
        let end = self.point_at_end();

        start.distance(&end, None) < Tolerance::ABSOLUTE
    }

    /// Check if curve is periodic (wraps around seamlessly)
    pub fn is_periodic(&self) -> bool {
        // For now, return false - full implementation would check
        // if the curve is clamped and if removing end nurbsknots makes it periodic
        false
    }

    /// Check if curve is a straight line within tolerance
    pub fn is_linear(&self, tolerance: Option<f64>) -> bool {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);

        if !self.is_valid() || self.m_cv_count < 2 {
            return false;
        }

        if self.m_cv_count == 2 {
            return true;
        }

        // Check if all control points are collinear
        let p0 = self.get_cv(0).unwrap();
        let p1 = self.get_cv(self.m_cv_count - 1).unwrap();

        let line_vec = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        let line_len = line_vec.magnitude();

        if line_len < tol {
            return true; // Degenerate to a point
        }

        for i in 1..(self.m_cv_count - 1) {
            let p = self.get_cv(i).unwrap();
            let v = Vector::new(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);

            // Cross product to check collinearity
            let cross = line_vec.cross(&v);

            if cross.magnitude() > tol * line_len {
                return false;
            }
        }

        true
    }

    /// Check if curve is planar (all CVs lie in a single plane)
    pub fn is_planar(&self, tolerance: Option<f64>) -> bool {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        if !self.is_valid() || self.m_cv_count < 3 {
            return true;
        }
        let p0 = self.get_cv(0).unwrap();
        let p1 = self.get_cv(1).unwrap();
        let mut normal = None;
        for i in 2..self.m_cv_count {
            let p = self.get_cv(i).unwrap();
            let v1 = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
            let v2 = Vector::new(p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]);
            let cross = v1.cross(&v2);
            if cross.magnitude() > tol {
                if normal.is_none() {
                    normal = Some(cross.normalized());
                } else {
                    let n = normal.as_ref().unwrap();
                    if (1.0 - n.dot(&cross.normalized()).abs()).abs() > tol {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Check if curve is an arc (matches C++ is_arc stub)
    pub fn is_arc(&self, _tolerance: Option<f64>) -> bool {
        false
    }

    /// Check if curve lies entirely in a given plane
    pub fn is_in_plane(&self, plane: &Plane, tolerance: Option<f64>) -> bool {
        let tol = tolerance.unwrap_or(Tolerance::ZERO_TOLERANCE);
        if !self.is_valid() {
            return false;
        }
        for i in 0..self.m_cv_count {
            let p = self.get_cv(i).unwrap();
            let v = Vector::new(
                p[0] - plane.origin()[0],
                p[1] - plane.origin()[1],
                p[2] - plane.origin()[2],
            );
            if v.dot(&plane.z_axis()).abs() > tol {
                return false;
            }
        }
        true
    }

    /// Check if curve has "natural" end conditions (zero 2nd derivative at endpoints)
    pub fn is_natural(&self, _tolerance: Option<f64>) -> bool {
        if !self.is_valid() {
            return false;
        }

        let tol_factor = 1e-8;
        let (t0, t1) = self.domain();

        // Check both endpoints
        for pass in 0..2 {
            let t = if pass == 0 { t0 } else { t1 };

            // Evaluate 2nd derivative
            let derivs = self.evaluate(t, 2);
            if derivs.len() < 3 {
                return false;
            }

            let d2 = &derivs[2];
            let d2_len = d2.magnitude();

            // Get control polygon length for tolerance
            let (cv0, cv2) = if pass == 0 {
                (self.get_cv(0), self.get_cv(2.min(self.m_cv_count - 1)))
            } else {
                (
                    self.get_cv(self.m_cv_count - 1),
                    self.get_cv(0.max(self.m_cv_count as i32 - 3) as usize),
                )
            };

            if cv0.is_none() || cv2.is_none() {
                return false;
            }

            let tol = cv0.unwrap().distance(&cv2.unwrap(), None) * tol_factor;

            if d2_len > tol {
                return false;
            }
        }

        true
    }

    /// Check if curve is a polyline (all CVs connected by straight segments)
    pub fn is_polyline(&self, _tolerance: Option<f64>) -> bool {
        if !self.is_valid() {
            return false;
        }
        self.degree() == 1
    }

    pub fn is_singular(&self) -> bool {
        if !self.is_valid() {
            return false;
        }
        let sc = self.span_count();
        for i in 0..sc {
            if !self.span_is_singular(i) {
                return false;
            }
        }
        true
    }

    pub fn is_duplicate(&self, other: &NurbsCurve, ignore_parameterization: bool) -> bool {
        if !self.is_valid() || !other.is_valid() {
            return false;
        }
        if self.m_dim != other.m_dim {
            return false;
        }
        if self.m_is_rat != other.m_is_rat {
            return false;
        }
        if self.m_order != other.m_order {
            return false;
        }
        if self.m_cv_count != other.m_cv_count {
            return false;
        }
        let tolerance = Tolerance::ZERO_TOLERANCE;
        for i in 0..self.m_cv_count {
            if let (Some(p1), Some(p2)) = (self.get_cv(i), other.get_cv(i)) {
                if p1.distance(&p2, None) > tolerance {
                    return false;
                }
                if self.m_is_rat && (self.weight(i) - other.weight(i)).abs() > tolerance {
                    return false;
                }
            } else {
                return false;
            }
        }
        if !ignore_parameterization {
            for i in 0..self.nurbsknot_count() {
                if (self.m_nurbsknot[i] - other.m_nurbsknot[i]).abs() > tolerance {
                    return false;
                }
            }
        }
        true
    }

    pub fn is_continuous(&self, continuity_type: i32, t: f64) -> bool {
        if !self.is_valid() {
            return false;
        }
        let (d0, d1) = self.domain();
        if t < d0 || t > d1 {
            return false;
        }
        let mut at_nurbsknot = false;
        let mut nurbsknot_idx: usize = 0;
        for i in 0..self.nurbsknot_count() {
            if (self.m_nurbsknot[i] - t).abs() < Tolerance::ZERO_TOLERANCE {
                at_nurbsknot = true;
                nurbsknot_idx = i;
                break;
            }
        }
        if !at_nurbsknot {
            return true;
        }
        let mult = self.nurbsknot_multiplicity(nurbsknot_idx);
        if continuity_type == 0 {
            return mult < self.m_order;
        }
        if continuity_type == 1 {
            return mult < self.m_order - 1;
        }
        if continuity_type == 2 {
            return mult < self.m_order - 2;
        }
        mult < self.m_order - 1
    }

    /// Check if nurbsknot vector is valid
    pub fn is_valid_nurbsknot_vector(&self) -> bool {
        if self.m_nurbsknot.len() != self.m_order + self.m_cv_count - 2 {
            return false;
        }
        // Check non-decreasing
        for i in 1..self.m_nurbsknot.len() {
            if self.m_nurbsknot[i] < self.m_nurbsknot[i - 1] - Tolerance::ZERO_TOLERANCE {
                return false;
            }
        }
        // Check valid domain exists
        if self.m_order >= 2 && self.m_cv_count >= self.m_order {
            let idx1 = self.m_order - 2;
            let idx2 = self.m_cv_count - 1;
            if idx2 < self.m_nurbsknot.len() && self.m_nurbsknot[idx1] >= self.m_nurbsknot[idx2] {
                return false;
            }
        }
        true
    }

    /// Check if nurbsknot vector is clamped at ends (0=start, 1=end, 2=both)
    pub fn is_clamped(&self, end: i32) -> bool {
        if !self.is_valid() {
            return false;
        }
        nurbsknot::is_clamped(self.m_order, self.m_cv_count, &self.m_nurbsknot, end)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════

    /// Get dimension
    pub fn dimension(&self) -> usize {
        self.m_dim
    }

    /// Get curve degree
    pub fn degree(&self) -> usize {
        if self.m_order < 2 {
            0
        } else {
            self.m_order - 1
        }
    }

    /// Get curve order
    pub fn order(&self) -> usize {
        self.m_order
    }

    /// Get number of control vertices
    pub fn cv_count(&self) -> usize {
        self.m_cv_count
    }

    /// Get nurbsknot count
    pub fn nurbsknot_count(&self) -> usize {
        self.m_nurbsknot.len()
    }

    /// Get size of each control vertex (dimension + 1 if rational, else dimension)
    pub fn cv_size(&self) -> usize {
        self.m_cv_stride
    }

    /// Get number of spans
    pub fn span_count(&self) -> usize {
        if !self.is_valid() {
            return 0;
        }
        let spans = self.get_span_vector();
        if spans.len() > 1 {
            spans.len() - 1
        } else {
            0
        }
    }

    /// Get all nurbsknot values
    pub fn get_nurbsknots(&self) -> Vec<f64> {
        self.m_nurbsknot.clone()
    }

    /// Get nurbsknot array pointer (for compatibility)
    pub fn nurbsknot_array(&self) -> &[f64] {
        &self.m_nurbsknot
    }

    /// Get CV array pointer (for compatibility)
    pub fn cv_array(&self) -> &[f64] {
        &self.m_cv
    }

    /// Get CV array mutable pointer (for expert use)
    pub fn cv_array_mut(&mut self) -> &mut [f64] {
        &mut self.m_cv
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Control Vertex Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Set control vertex at index
    pub fn set_cv(&mut self, index: usize, point: &Point) {
        if index >= self.m_cv_count {
            return;
        }

        let idx = index * self.m_cv_stride;
        self.m_cv[idx] = point[0];
        if self.m_dim > 1 {
            self.m_cv[idx + 1] = point[1];
        }
        if self.m_dim > 2 {
            self.m_cv[idx + 2] = point[2];
        }
        if self.m_is_rat {
            // The point is euclidean, so the weight resets: keeping a stale one makes
            // get_cv divide by it and return a different point than was set.
            self.m_cv[idx + self.m_dim] = 1.0;
        }
    }

    /// Get control vertex at index
    pub fn get_cv(&self, index: usize) -> Option<Point> {
        if index >= self.m_cv_count {
            return None;
        }

        let idx = index * self.m_cv_stride;
        let x = self.m_cv[idx];
        let y = if self.m_dim > 1 {
            self.m_cv[idx + 1]
        } else {
            0.0
        };
        let z = if self.m_dim > 2 {
            self.m_cv[idx + 2]
        } else {
            0.0
        };

        if self.m_is_rat {
            let w = self.m_cv[idx + self.m_dim];
            if w.abs() < 1e-14 {
                return Some(Point::new(0.0, 0.0, 0.0));
            }
            Some(Point::new(x / w, y / w, z / w))
        } else {
            Some(Point::new(x, y, z))
        }
    }

    /// Set control vertex at index (public version)
    pub fn set_cv_point(&mut self, index: usize, point: &Point) -> bool {
        if index >= self.m_cv_count {
            return false;
        }
        self.set_cv(index, point);
        true
    }

    /// Get raw CV data at index (like C++ double* cv(int))
    pub fn cv(&self, cv_index: usize) -> Option<&[f64]> {
        if cv_index >= self.m_cv_count {
            return None;
        }
        let idx = cv_index * self.m_cv_stride;
        Some(&self.m_cv[idx..idx + self.m_cv_stride])
    }

    /// Get control point at index as homogeneous point (x, y, z, w)
    pub fn get_cv_4d(&self, cv_index: usize) -> Option<(f64, f64, f64, f64)> {
        if cv_index >= self.m_cv_count {
            return None;
        }
        let idx = cv_index * self.m_cv_stride;
        let x = self.m_cv[idx];
        let y = if self.m_dim > 1 {
            self.m_cv[idx + 1]
        } else {
            0.0
        };
        let z = if self.m_dim > 2 {
            self.m_cv[idx + 2]
        } else {
            0.0
        };
        let w = if self.m_is_rat {
            self.m_cv[idx + self.m_dim]
        } else {
            1.0
        };
        Some((x, y, z, w))
    }

    /// Set control point at index from homogeneous coordinates
    pub fn set_cv_4d(&mut self, cv_index: usize, x: f64, y: f64, z: f64, w: f64) -> bool {
        if cv_index >= self.m_cv_count {
            return false;
        }

        // Make rational if w != 1.0 (matches C++ implementation)
        if !self.m_is_rat && w != 1.0 {
            self.make_rational();
        }

        let idx = cv_index * self.m_cv_stride;
        self.m_cv[idx] = x;
        if self.m_dim > 1 {
            self.m_cv[idx + 1] = y;
        }
        if self.m_dim > 2 {
            self.m_cv[idx + 2] = z;
        }
        if self.m_is_rat {
            self.m_cv[idx + self.m_dim] = w;
        }
        true
    }

    /// Get weight at control vertex index (returns 1.0 if non-rational)
    pub fn weight(&self, cv_index: usize) -> f64 {
        if !self.m_is_rat || cv_index >= self.m_cv_count {
            return 1.0;
        }
        let idx = cv_index * self.m_cv_stride + self.m_dim;
        self.m_cv[idx]
    }

    /// Set weight at control vertex index
    pub fn set_weight(&mut self, cv_index: usize, weight: f64) -> bool {
        if cv_index >= self.m_cv_count {
            return false;
        }
        if !self.m_is_rat {
            return false;
        }
        let idx = cv_index * self.m_cv_stride + self.m_dim;
        self.m_cv[idx] = weight;
        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // NurbsKnot Access
    // ═══════════════════════════════════════════════════════════════════════════

    /// Get nurbsknot value at index
    pub fn nurbsknot(&self, nurbsknot_index: usize) -> Option<f64> {
        if nurbsknot_index >= self.m_nurbsknot.len() {
            return None;
        }
        Some(self.m_nurbsknot[nurbsknot_index])
    }

    /// Set nurbsknot value at index
    pub fn set_nurbsknot(&mut self, nurbsknot_index: usize, nurbsknot_value: f64) -> bool {
        if nurbsknot_index >= self.m_nurbsknot.len() {
            return false;
        }
        self.m_nurbsknot[nurbsknot_index] = nurbsknot_value;
        true
    }

    /// Get nurbsknot multiplicity at index
    pub fn nurbsknot_multiplicity(&self, nurbsknot_index: usize) -> usize {
        if nurbsknot_index >= self.m_nurbsknot.len() {
            return 0;
        }
        let val = self.m_nurbsknot[nurbsknot_index];
        let mut count = 1;
        // Count forward
        let mut i = nurbsknot_index + 1;
        while i < self.m_nurbsknot.len()
            && (self.m_nurbsknot[i] - val).abs() < Tolerance::ZERO_TOLERANCE
        {
            count += 1;
            i += 1;
        }
        // Count backward
        let mut j = nurbsknot_index;
        while j > 0 {
            j -= 1;
            if (self.m_nurbsknot[j] - val).abs() < Tolerance::ZERO_TOLERANCE {
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    /// Get superfluous nurbsknot value at end (0=start, 1=end)
    pub fn superfluous_nurbsknot(&self, end: usize) -> f64 {
        if !self.is_valid() {
            return 0.0;
        }
        let kc = self.nurbsknot_count();
        if end == 0 {
            // First superfluous nurbsknot: reflect first nurbsknot across nurbsknot[order-2]
            return 2.0 * self.m_nurbsknot[0] - self.m_nurbsknot[self.m_order - 2];
        } else {
            // Last superfluous nurbsknot: reflect last nurbsknot across nurbsknot[cv_count-order]
            return 2.0 * self.m_nurbsknot[kc - 1]
                - self.m_nurbsknot[self.m_cv_count - self.m_order];
        }
    }

    /// Create a clamped uniform nurbsknot vector for this curve
    pub fn make_clamped_uniform_nurbsknot_vector(&mut self, delta: f64) -> bool {
        if delta <= 0.0 || self.m_dim == 0 || self.m_order < 2 || self.m_cv_count < self.m_order {
            return false;
        }
        self.m_nurbsknot = nurbsknot::make_clamped_uniform(self.m_order, self.m_cv_count, delta);
        !self.m_nurbsknot.is_empty()
    }

    /// Insert nurbsknot into curve (Boehm's algorithm)
    pub fn insert_nurbsknot(
        &mut self,
        nurbsknot_value: f64,
        nurbsknot_multiplicity: usize,
    ) -> bool {
        if !self.is_valid() {
            return false;
        }

        let p = self.degree();
        if nurbsknot_multiplicity < 1 || nurbsknot_multiplicity > p {
            return false;
        }

        let (d0, d1) = self.domain();
        if nurbsknot_value < d0 || nurbsknot_value > d1 {
            return false;
        }

        // Handle end nurbsknots
        if nurbsknot_value == d0 {
            if nurbsknot_multiplicity == p {
                self.clamp_end(0);
                return true;
            }
            if nurbsknot_multiplicity == 1 {
                return true;
            }
            return false;
        }
        if nurbsknot_value == d1 {
            if nurbsknot_multiplicity == p {
                self.clamp_end(1);
                return true;
            }
            if nurbsknot_multiplicity == 1 {
                return true;
            }
            return false;
        }

        let mut n = self.m_cv_count - 1;
        let mut full_nurbsknot_count = self.m_cv_count + self.m_order;

        for _insert_iter in 0..nurbsknot_multiplicity {
            // Build full nurbsknot vector
            let mut u = vec![0.0f64; full_nurbsknot_count];
            u[0] = self.m_nurbsknot[0];
            for i in 0..self.m_nurbsknot.len() {
                u[i + 1] = self.m_nurbsknot[i];
            }
            u[full_nurbsknot_count - 1] = *self.m_nurbsknot.last().unwrap();

            // Count current multiplicity
            let tol = (d0.abs() + d1.abs() + (d1 - d0).abs()) * f64::EPSILON.sqrt();
            let mult = u
                .iter()
                .filter(|&&v| (v - nurbsknot_value).abs() <= tol)
                .count();
            if mult >= nurbsknot_multiplicity {
                // Already at the requested multiplicity (e.g. splitting a degree-1 polyline
                // exactly at a vertex nurbsknot) -- nothing to insert, and that is success.
                return true;
            }
            if mult >= p {
                // Cannot increase multiplicity beyond degree for interior nurbsknots
                return false;
            }

            // Find span
            let span = self.find_span(nurbsknot_value);
            let k = span + self.m_order - 1;

            let m_full = full_nurbsknot_count - 1;
            let new_full_nurbsknot_count = full_nurbsknot_count + 1;
            let new_cv_count = self.m_cv_count + 1;

            let mut u_new = vec![0.0f64; new_full_nurbsknot_count];
            let mut cv_new = vec![0.0f64; new_cv_count * self.m_cv_stride];

            // Copy unaffected nurbsknots
            for i in 0..=k {
                u_new[i] = u[i];
            }
            u_new[k + 1] = nurbsknot_value;
            for i in (k + 1)..=m_full {
                u_new[i + 1] = u[i];
            }

            // Copy unaffected CVs before
            for i in 0..=(k - p) {
                let src = i * self.m_cv_stride;
                let dst = i * self.m_cv_stride;
                cv_new[dst..dst + self.m_cv_stride]
                    .copy_from_slice(&self.m_cv[src..src + self.m_cv_stride]);
            }

            // Copy unaffected CVs after
            for i in (k + 1)..=(n + 1) {
                let src = (i - 1) * self.m_cv_stride;
                let dst = i * self.m_cv_stride;
                cv_new[dst..dst + self.m_cv_stride]
                    .copy_from_slice(&self.m_cv[src..src + self.m_cv_stride]);
            }

            // Compute new CVs in affected region
            for i in (k - p + 1)..=(k) {
                let denom = u[i + p] - u[i];
                let alpha = if denom != 0.0 {
                    (nurbsknot_value - u[i]) / denom
                } else {
                    0.0
                };

                let src_prev = (i - 1) * self.m_cv_stride;
                let src_curr = i * self.m_cv_stride;
                let dst = i * self.m_cv_stride;

                for d in 0..self.m_cv_stride {
                    cv_new[dst + d] =
                        (1.0 - alpha) * self.m_cv[src_prev + d] + alpha * self.m_cv[src_curr + d];
                }
            }

            // Update internal state
            self.m_cv_count = new_cv_count;
            self.m_cv = cv_new;

            let new_compressed = self.m_order + self.m_cv_count - 2;
            self.m_nurbsknot = (0..new_compressed).map(|i| u_new[i + 1]).collect();

            full_nurbsknot_count = new_full_nurbsknot_count;
            n = self.m_cv_count - 1;
        }

        true
    }

    /// Get Greville abcissa for a control point (aligned with opennurbs)
    pub fn greville_abcissa(&self, cv_index: usize) -> f64 {
        if cv_index >= self.m_cv_count {
            return 0.0;
        }

        let nurbsknot = &self.m_nurbsknot[cv_index..];
        let order = self.m_order;

        if order <= 2 || nurbsknot[0] == nurbsknot[order - 2] {
            return nurbsknot[0];
        }

        let p = order - 1;
        let k0 = nurbsknot[0];
        let k = nurbsknot[p / 2];
        let k1 = nurbsknot[p - 1];
        let tol = (k1 - k0) * 1.490116119385e-8_f64;
        let dp = p as f64;

        let mut g: f64 = nurbsknot[..p].iter().sum();
        g /= dp;

        // Snap to exact middle nurbsknot for uniform nurbsknot vectors
        if (2.0 * k - (k0 + k1)).abs() <= tol
            && (g - k).abs() <= (g.abs() * 1.490116119385e-8_f64 + tol)
        {
            g = k;
        }

        g
    }

    /// Get all Greville abcissae
    pub fn get_greville_abcissae(&self) -> Vec<f64> {
        (0..self.m_cv_count)
            .map(|i| self.greville_abcissa(i))
            .collect()
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Domain & Parameterization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Get curve domain [t_start, t_end]
    pub fn domain(&self) -> (f64, f64) {
        if !self.is_valid() {
            return (0.0, 0.0);
        }
        let t0 = self.m_nurbsknot[self.m_order - 2];
        let t1 = self.m_nurbsknot[self.m_cv_count - 1];
        (t0, t1)
    }

    /// Get start of domain
    pub fn domain_start(&self) -> f64 {
        self.domain().0
    }

    /// Get end of domain
    pub fn domain_end(&self) -> f64 {
        self.domain().1
    }

    /// Get middle of domain
    pub fn domain_middle(&self) -> f64 {
        let (t0, t1) = self.domain();
        (t0 + t1) * 0.5
    }

    /// Set curve domain
    pub fn set_domain(&mut self, t0: f64, t1: f64) -> bool {
        if !self.is_valid() || t0 >= t1 {
            return false;
        }

        let (old_t0, old_t1) = self.domain();
        if (old_t0 - old_t1).abs() < 1e-14 {
            return false;
        }

        let clamped_start = self.m_order >= 2
            && (self.m_nurbsknot[0] - self.m_nurbsknot[self.m_order - 2]).abs()
                < Tolerance::ZERO_TOLERANCE;
        let clamped_end = self.m_cv_count < self.m_nurbsknot.len()
            && (*self.m_nurbsknot.last().unwrap() - self.m_nurbsknot[self.m_cv_count - 1]).abs()
                < Tolerance::ZERO_TOLERANCE;

        let scale = (t1 - t0) / (old_t1 - old_t0);

        for i in 0..self.m_nurbsknot.len() {
            self.m_nurbsknot[i] = t0 + (self.m_nurbsknot[i] - old_t0) * scale;
        }

        if clamped_start {
            for i in 0..self.m_order - 1 {
                self.m_nurbsknot[i] = t0;
            }
        }
        if clamped_end {
            for i in self.m_cv_count - 1..self.m_nurbsknot.len() {
                self.m_nurbsknot[i] = t1;
            }
        }

        true
    }

    /// Get span vector (parameter values at span boundaries)
    pub fn get_span_vector(&self) -> Vec<f64> {
        let mut spans = Vec::new();
        if !self.is_valid() {
            return spans;
        }

        let offset = self.m_order - 2;
        spans.push(self.m_nurbsknot[offset]);

        for i in (offset + 1)..self.m_cv_count {
            if i == offset
                || (self.m_nurbsknot[i] - self.m_nurbsknot[i - 1]).abs() > Tolerance::ZERO_TOLERANCE
            {
                spans.push(self.m_nurbsknot[i]);
            }
        }

        spans
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Geometric Queries
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn get_next_discontinuity(&self, continuity_type: i32, t0: f64, t1: f64) -> (bool, f64) {
        if !self.is_valid() || t0 >= t1 {
            return (false, 0.0);
        }
        let (d0, d1) = self.domain();
        let t0 = t0.max(d0);
        let t1 = t1.min(d1);
        if t0 >= t1 {
            return (false, 0.0);
        }
        for i in (self.m_order - 1)..(self.m_cv_count - 1) {
            let t = self.m_nurbsknot[i];
            if t <= t0 || t >= t1 {
                continue;
            }
            let mult = self.nurbsknot_multiplicity(i);
            if continuity_type == 0 && mult >= self.m_order {
                return (true, t);
            }
            if continuity_type == 1 && mult >= self.m_order - 1 {
                return (true, t);
            }
            if continuity_type == 2 && mult >= self.m_order - 2 {
                return (true, t);
            }
            if (continuity_type == 3 || continuity_type == 4) && mult >= self.m_order - 1 {
                return (true, t);
            }
        }
        (false, 0.0)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Conversion Methods
    // ═══════════════════════════════════════════════════════════════════════════

    /// Get curve length using Gauss-Legendre quadrature
    pub fn length(&self, _tolerance: Option<f64>) -> f64 {
        if !self.is_valid() {
            return 0.0;
        }

        const GL_X: [f64; 10] = [
            -0.9739065285171717,
            -0.8650633666889845,
            -0.6794095682990244,
            -0.4333953941292472,
            -0.1488743389816312,
            0.1488743389816312,
            0.4333953941292472,
            0.6794095682990244,
            0.8650633666889845,
            0.9739065285171717,
        ];
        const GL_W: [f64; 10] = [
            0.0666713443086881,
            0.1494513491505806,
            0.2190863625159820,
            0.2692667193099963,
            0.2955242247147529,
            0.2955242247147529,
            0.2692667193099963,
            0.2190863625159820,
            0.1494513491505806,
            0.0666713443086881,
        ];

        let mut total = 0.0;
        // Count nurbsknot INTERVALS, not span_count(): a repeated interior nurbsknot makes
        // span_count() smaller than the interval count, and the trailing spans go unintegrated.
        let n_spans = self.m_cv_count - self.m_order + 1;
        const SUBDIVISIONS: usize = 4;

        for span in 0..n_spans {
            let span_a = self.m_nurbsknot[self.m_order - 2 + span];
            let span_b = self.m_nurbsknot[self.m_order - 1 + span];
            if span_b <= span_a {
                continue;
            }

            let span_width = (span_b - span_a) / SUBDIVISIONS as f64;
            for sub in 0..SUBDIVISIONS {
                let a = span_a + sub as f64 * span_width;
                let b = a + span_width;
                let mid = (a + b) * 0.5;
                let half = (b - a) * 0.5;
                let mut s = 0.0;

                for i in 0..10 {
                    let t = mid + half * GL_X[i];
                    let derivs = self.evaluate(t, 1);
                    s += GL_W[i] * derivs[1].magnitude();
                }
                total += half * s;
            }
        }
        total
    }

    /// Convert curve to polyline using adaptive subdivision
    pub fn to_polyline_adaptive(
        &self,
        angle_tolerance: f64,
        min_edge_length: f64,
        max_edge_length: f64,
    ) -> (Vec<Point>, Vec<f64>) {
        let mut points = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() {
            return (points, params);
        }

        let angle_tol = if angle_tolerance <= 0.0 {
            0.1
        } else {
            angle_tolerance
        };

        let (t0, t1) = self.domain();
        let curve_len = self.length(Some(1e-6));

        let max_len = if max_edge_length <= 0.0 {
            curve_len / 10.0
        } else {
            max_edge_length
        };
        let mut min_len = if min_edge_length <= 0.0 {
            curve_len / 1000.0
        } else {
            min_edge_length
        };
        if min_len > max_len {
            min_len = max_len * 0.1;
        }

        // Collect (param, point) pairs, then sort by param
        let mut samples: Vec<(f64, Point)> = Vec::new();
        samples.push((t0, self.point_at(t0)));
        samples.push((t1, self.point_at(t1)));

        // Work queue: segments to potentially subdivide (ta, tb)
        let mut work_queue: Vec<(f64, f64)> = Vec::new();
        work_queue.push((t0, t1));

        // Closed curves: start == end, so the initial (t0,t1) chord has zero length
        // and the adaptive loop skips it. Force-subdivide into thirds to bootstrap.
        if self.point_at(t0).distance(&self.point_at(t1), None) < 1e-6 && curve_len > max_len {
            let span = (t1 - t0) / 3.0;
            let tm1 = t0 + span;
            let tm2 = t0 + 2.0 * span;
            samples.push((tm1, self.point_at(tm1)));
            samples.push((tm2, self.point_at(tm2)));
            work_queue.clear();
            work_queue.push((t0, tm1));
            work_queue.push((tm1, tm2));
            work_queue.push((tm2, t1));
        }

        const MAX_ITERATIONS: i32 = 10000;
        let mut iterations = 0;

        while !work_queue.is_empty() && iterations < MAX_ITERATIONS {
            iterations += 1;
            let (ta, tb) = work_queue.pop().unwrap();

            let pa = self.point_at(ta);
            let pb = self.point_at(tb);
            let chord_length = pa.distance(&pb, None);

            if chord_length < min_len {
                continue;
            }

            let tm = (ta + tb) * 0.5;
            let pm = self.point_at(tm);

            // Check deviation: distance from midpoint to chord
            let chord = Vector::new(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]);
            let to_mid = Vector::new(pm[0] - pa[0], pm[1] - pa[1], pm[2] - pa[2]);
            let chord_len_sq = chord.dot(&chord);
            let mut deviation = 0.0;

            if chord_len_sq > 1e-20 {
                let proj = to_mid.dot(&chord) / chord_len_sq;
                let projected = Point::new(
                    pa[0] + proj * chord[0],
                    pa[1] + proj * chord[1],
                    pa[2] + proj * chord[2],
                );
                deviation = pm.distance(&projected, None);
            }

            // Convert angle tolerance to approximate deviation tolerance
            // For small angles: deviation ≈ chord_length * sin(angle/2) ≈ chord_length * angle/2
            let deviation_tolerance = chord_length * angle_tol * 0.5;

            let need_subdivide = (deviation > deviation_tolerance) || (chord_length > max_len);

            if need_subdivide {
                samples.push((tm, pm));
                work_queue.push((ta, tm));
                work_queue.push((tm, tb));
            }
        }

        // Sort by parameter
        samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // Extract results
        for (t, p) in samples {
            points.push(p);
            params.push(t);
        }

        (points, params)
    }

    /// Divide curve into equal arc-length segments using Gauss-Legendre quadrature.
    /// Matches C++ implementation exactly.
    ///
    /// # Arguments
    /// * `count` - Number of points to generate
    /// * `include_endpoints` - If true, includes start and end points
    ///
    /// # Returns
    /// Tuple of (points, parameters)
    pub fn divide_by_count(&self, count: usize, include_endpoints: bool) -> (Vec<Point>, Vec<f64>) {
        let mut points = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() || count < 2 {
            return (points, params);
        }

        let (t0, t1) = self.domain();
        let dom_len = t1 - t0;
        let h = dom_len * 1e-8;

        // 5-point Gauss-Legendre nodes and weights for [-1, 1]
        const GL_NODES: [f64; 5] = [
            -0.9061798459386640,
            -0.5384693101056831,
            0.0,
            0.5384693101056831,
            0.9061798459386640,
        ];
        const GL_WEIGHTS: [f64; 5] = [
            0.2369268850561891,
            0.4786286704993665,
            0.5688888888888889,
            0.4786286704993665,
            0.2369268850561891,
        ];

        // Compute derivative (un-normalized) at parameter t
        let derivative_at = |curve: &NurbsCurve, t: f64| -> Vector {
            let (p1, p2, dt);
            if t <= t0 + h {
                p1 = curve.point_at(t0);
                p2 = curve.point_at(t0 + h);
                dt = h;
            } else if t >= t1 - h {
                p1 = curve.point_at(t1 - h);
                p2 = curve.point_at(t1);
                dt = h;
            } else {
                p1 = curve.point_at(t - h);
                p2 = curve.point_at(t + h);
                dt = 2.0 * h;
            }
            Vector::new(
                (p2[0] - p1[0]) / dt,
                (p2[1] - p1[1]) / dt,
                (p2[2] - p1[2]) / dt,
            )
        };

        // Arc length via Gauss-Legendre quadrature
        let arc_length_gauss = |curve: &NurbsCurve, ta: f64, tb: f64| -> f64 {
            let mid = (ta + tb) * 0.5;
            let half = (tb - ta) * 0.5;
            let mut sum = 0.0;
            for i in 0..5 {
                let t = mid + half * GL_NODES[i];
                sum += GL_WEIGHTS[i] * derivative_at(curve, t).magnitude();
            }
            half * sum
        };

        // Build arc-length table with high resolution
        let n_samples = (1000usize).max(count * 100);
        let dt = (t1 - t0) / n_samples as f64;

        let mut t_vals = vec![0.0; n_samples + 1];
        let mut s_vals = vec![0.0; n_samples + 1];

        t_vals[0] = t0;
        s_vals[0] = 0.0;

        for i in 1..=n_samples {
            t_vals[i] = t0 + i as f64 * dt;
            s_vals[i] = s_vals[i - 1] + arc_length_gauss(self, t_vals[i - 1], t_vals[i]);
        }

        let total_len = s_vals[n_samples];
        let n_segs = if include_endpoints {
            count - 1
        } else {
            count + 1
        };
        let seg_len = total_len / n_segs as f64;

        // Find parameter at target arc length with Newton-Raphson refinement
        let find_t_at_s = |curve: &NurbsCurve, s_target: f64| -> f64 {
            if s_target <= 0.0 {
                return t0;
            }
            if s_target >= total_len {
                return t1;
            }

            // Binary search for bracket
            let mut lo = 0usize;
            let mut hi = n_samples;
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if s_vals[mid] < s_target {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }

            // Initial guess: linear interpolation
            let frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
            let mut t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);

            // Newton-Raphson refinement
            let mut t_lo = t_vals[lo];
            let mut t_hi = t_vals[hi];
            for _ in 0..20 {
                let s_cur = s_vals[lo] + arc_length_gauss(curve, t_vals[lo], t);
                let error = s_cur - s_target;

                if error.abs() < 1e-12 {
                    break;
                }

                let speed = derivative_at(curve, t).magnitude();
                if speed < 1e-14 {
                    if error > 0.0 {
                        t_hi = t;
                        t = (t_lo + t_hi) * 0.5;
                    } else {
                        t_lo = t;
                        t = (t_lo + t_hi) * 0.5;
                    }
                    continue;
                }

                let t_new = t - error / speed;
                if t_new <= t_lo || t_new >= t_hi {
                    if error > 0.0 {
                        t_hi = t;
                        t = (t_lo + t_hi) * 0.5;
                    } else {
                        t_lo = t;
                        t = (t_lo + t_hi) * 0.5;
                    }
                } else {
                    t = t_new;
                }
            }
            t
        };

        points.reserve(count);
        params.reserve(count);

        for i in 0..count {
            let s_target = if include_endpoints {
                seg_len * i as f64
            } else {
                seg_len * (i + 1) as f64
            };

            let t = find_t_at_s(self, s_target);
            points.push(self.point_at(t));
            params.push(t);
        }

        (points, params)
    }

    /// Divide curve by arc length using Gauss-Legendre quadrature.
    /// Matches C++ implementation exactly.
    pub fn divide_by_length(&self, segment_length: f64) -> (Vec<Point>, Vec<f64>) {
        let mut points = Vec::new();
        let mut params = Vec::new();

        if !self.is_valid() || segment_length <= 0.0 {
            return (points, params);
        }

        let (t0, t1) = self.domain();
        let dom_len = t1 - t0;
        let h = dom_len * 1e-8;

        // 5-point Gauss-Legendre nodes and weights for [-1, 1]
        const GL_NODES: [f64; 5] = [
            -0.9061798459386640,
            -0.5384693101056831,
            0.0,
            0.5384693101056831,
            0.9061798459386640,
        ];
        const GL_WEIGHTS: [f64; 5] = [
            0.2369268850561891,
            0.4786286704993665,
            0.5688888888888889,
            0.4786286704993665,
            0.2369268850561891,
        ];

        // Compute derivative (un-normalized) at parameter t
        let derivative_at = |curve: &NurbsCurve, t: f64| -> Vector {
            let (p1, p2, dt);
            if t <= t0 + h {
                p1 = curve.point_at(t0);
                p2 = curve.point_at(t0 + h);
                dt = h;
            } else if t >= t1 - h {
                p1 = curve.point_at(t1 - h);
                p2 = curve.point_at(t1);
                dt = h;
            } else {
                p1 = curve.point_at(t - h);
                p2 = curve.point_at(t + h);
                dt = 2.0 * h;
            }
            Vector::new(
                (p2[0] - p1[0]) / dt,
                (p2[1] - p1[1]) / dt,
                (p2[2] - p1[2]) / dt,
            )
        };

        // Arc length via Gauss-Legendre quadrature
        let arc_length_gauss = |curve: &NurbsCurve, ta: f64, tb: f64| -> f64 {
            let mid = (ta + tb) * 0.5;
            let half = (tb - ta) * 0.5;
            let mut sum = 0.0;
            for i in 0..5 {
                let t = mid + half * GL_NODES[i];
                sum += GL_WEIGHTS[i] * derivative_at(curve, t).magnitude();
            }
            half * sum
        };

        // Build arc-length table with high resolution
        let approx_len = self.length(Some(1e-6));
        let n_samples = (1000usize).max((approx_len / segment_length) as usize * 100);
        let dt = (t1 - t0) / n_samples as f64;

        let mut t_vals = vec![0.0; n_samples + 1];
        let mut s_vals = vec![0.0; n_samples + 1];

        t_vals[0] = t0;
        s_vals[0] = 0.0;

        for i in 1..=n_samples {
            t_vals[i] = t0 + i as f64 * dt;
            s_vals[i] = s_vals[i - 1] + arc_length_gauss(self, t_vals[i - 1], t_vals[i]);
        }

        let total_len = s_vals[n_samples];

        // Find parameter at target arc length with Newton-Raphson refinement
        let find_t_at_s = |curve: &NurbsCurve, s_target: f64| -> f64 {
            if s_target <= 0.0 {
                return t0;
            }
            if s_target >= total_len {
                return t1;
            }

            // Binary search for bracket
            let mut lo = 0usize;
            let mut hi = n_samples;
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if s_vals[mid] < s_target {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }

            // Initial guess: linear interpolation
            let frac = (s_target - s_vals[lo]) / (s_vals[hi] - s_vals[lo]);
            let mut t = t_vals[lo] + frac * (t_vals[hi] - t_vals[lo]);

            // Newton-Raphson refinement
            let mut t_lo = t_vals[lo];
            let mut t_hi = t_vals[hi];
            for _ in 0..20 {
                let s_cur = s_vals[lo] + arc_length_gauss(curve, t_vals[lo], t);
                let error = s_cur - s_target;

                if error.abs() < 1e-12 {
                    break;
                }

                let speed = derivative_at(curve, t).magnitude();
                if speed < 1e-14 {
                    if error > 0.0 {
                        t_hi = t;
                        t = (t_lo + t_hi) * 0.5;
                    } else {
                        t_lo = t;
                        t = (t_lo + t_hi) * 0.5;
                    }
                    continue;
                }

                let t_new = t - error / speed;
                if t_new <= t_lo || t_new >= t_hi {
                    if error > 0.0 {
                        t_hi = t;
                        t = (t_lo + t_hi) * 0.5;
                    } else {
                        t_lo = t;
                        t = (t_lo + t_hi) * 0.5;
                    }
                } else {
                    t = t_new;
                }
            }
            t
        };

        // Add points at each segment_length interval
        let mut s = 0.0;
        while s <= total_len + 1e-10 {
            let t = find_t_at_s(self, s);
            points.push(self.point_at(t));
            params.push(t);
            s += segment_length;
        }

        (points, params)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Evaluation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Evaluate point at parameter t
    ///
    /// Implementation matches OpenNURBS evaluation approach.
    pub fn point_at(&self, t: f64) -> Point {
        if !self.is_valid() {
            return Point::new(0.0, 0.0, 0.0);
        }

        // Find span (returns index relative to shifted nurbsknot array)
        let span = self.find_span(t);

        // Evaluate using Cox-de Boor algorithm
        let basis = self.basis_functions(span, t);

        // Compute point
        let mut x = 0.0;
        let mut y = 0.0;
        let mut z = 0.0;
        let mut w = 0.0;

        // In OpenNURBS, span index directly corresponds to CV starting index
        for i in 0..self.m_order {
            let cv_idx = span + i;
            if cv_idx >= self.m_cv_count {
                continue;
            }

            let idx = cv_idx * self.m_cv_stride;
            let n = basis[i];

            if self.m_is_rat {
                // CVs stored in homogeneous form: (x*w, y*w, z*w, w)
                let weight = self.m_cv[idx + self.m_dim];
                w += n * weight;
                x += n * self.m_cv[idx];
                if self.m_dim > 1 {
                    y += n * self.m_cv[idx + 1];
                }
                if self.m_dim > 2 {
                    z += n * self.m_cv[idx + 2];
                }
            } else {
                x += n * self.m_cv[idx];
                if self.m_dim > 1 {
                    y += n * self.m_cv[idx + 1];
                }
                if self.m_dim > 2 {
                    z += n * self.m_cv[idx + 2];
                }
                w = 1.0;
            }
        }

        if self.m_is_rat && w.abs() > 1e-10 {
            Point::new(x / w, y / w, z / w)
        } else {
            Point::new(x, y, z)
        }
    }

    /// Evaluate point and derivatives on curve at parameter t.
    /// Returns [point, d1, d2, ...] depending on derivative_count.
    /// Uses analytical basis function derivatives matching C++ implementation.
    pub fn evaluate(&self, t: f64, derivative_count: usize) -> Vec<Vector> {
        let mut result = Vec::new();

        if !self.is_valid() {
            result.push(Vector::new(0.0, 0.0, 0.0));
            return result;
        }

        // Clamp derivative order to degree
        let max_derivs = derivative_count.min(self.degree());

        let span = self.find_span(t);
        let ders = self.basis_functions_derivatives(span, t, max_derivs);

        // Evaluate homogeneous coordinates and derivatives
        let p = self.degree();
        let mut aders: Vec<[f64; 4]> = vec![[0.0; 4]; max_derivs + 1];

        for k in 0..=max_derivs {
            for j in 0..=p {
                let cv_idx = span + j;
                if cv_idx >= self.m_cv_count {
                    continue;
                }
                let idx = cv_idx * self.m_cv_stride;

                let nx = ders[k][j];
                let cx = self.m_cv[idx];
                let cy = if self.m_dim > 1 {
                    self.m_cv[idx + 1]
                } else {
                    0.0
                };
                let cz = if self.m_dim > 2 {
                    self.m_cv[idx + 2]
                } else {
                    0.0
                };
                let wv = if self.m_is_rat {
                    self.m_cv[idx + self.m_dim]
                } else {
                    1.0
                };

                // CVs stored in homogeneous form: cx=x*w, cy=y*w, cz=z*w
                aders[k][0] += nx * cx;
                aders[k][1] += nx * cy;
                aders[k][2] += nx * cz;
                aders[k][3] += nx * wv;
            }
        }

        // Convert from homogeneous derivatives (Aders) to Cartesian derivatives
        let mut cders: Vec<[f64; 3]> = vec![[0.0; 3]; max_derivs + 1];

        if !self.m_is_rat {
            // Non-rational: derivatives are directly Aders (w == 1)
            for k in 0..=max_derivs {
                cders[k] = [aders[k][0], aders[k][1], aders[k][2]];
            }
        } else {
            // Rational: use standard formula (Piegl & Tiller, Eq. 2.28)
            for k in 0..=max_derivs {
                let w = aders[0][3];
                let inv_w = if w != 0.0 { 1.0 / w } else { 0.0 };

                let mut ck_x = aders[k][0];
                let mut ck_y = aders[k][1];
                let mut ck_z = aders[k][2];

                // Subtract contributions of weight derivatives
                for j in 1..=k {
                    let coeff = Self::binomial(k, j) as f64;
                    let wj = aders[j][3];
                    ck_x -= coeff * wj * cders[k - j][0];
                    ck_y -= coeff * wj * cders[k - j][1];
                    ck_z -= coeff * wj * cders[k - j][2];
                }

                cders[k] = [ck_x * inv_w, ck_y * inv_w, ck_z * inv_w];
            }
        }

        // Fill result vectors (0th derivative = point)
        for k in 0..=max_derivs {
            result.push(Vector::new(cders[k][0], cders[k][1], cders[k][2]));
        }

        // If caller requested more derivatives than degree, pad with zeros
        for _ in (max_derivs + 1)..=derivative_count {
            result.push(Vector::new(0.0, 0.0, 0.0));
        }

        result
    }

    /// Curvature magnitude (1/radius) at parameter t, from analytic 1st/2nd derivatives:
    /// kappa = |C' x C''| / |C'|^3. Matches OCCT GeomLProp_CLProps::Curvature.
    pub fn curvature_at(&self, t: f64) -> f64 {
        let d = self.evaluate(t, 2);
        if d.len() < 3 {
            return 0.0;
        }
        let s = d[1].magnitude();
        if s < 1e-12 {
            return 0.0;
        }
        d[1].cross(&d[2]).magnitude() / (s * s * s)
    }

    /// Parameter of the closest point on the curve to test_point (grid seed + Newton).
    /// Matches OCCT GeomAPI_ProjectPointOnCurve.
    pub fn closest_parameter(&self, test_point: &Point) -> f64 {
        crate::closest::Closest::curve_point(self, test_point, 0.0, 0.0).0
    }

    /// Closest point on the curve to test_point.
    pub fn closest_point(&self, test_point: &Point) -> Point {
        self.point_at(self.closest_parameter(test_point))
    }

    /// Parameters (u, v) where this curve is closest to another curve.
    /// Matches OCCT GeomAPI_ExtremaCurveCurve.
    pub fn closest_parameters_curve(&self, other: &NurbsCurve) -> (f64, f64) {
        let (u, v, _d) = crate::closest::Closest::curve_curve(self, other);
        (u, v)
    }

    /// Points (this(u), other(v)) where this curve is closest to another curve.
    pub fn closest_points_curve(&self, other: &NurbsCurve) -> (Point, Point) {
        let (u, v, _d) = crate::closest::Closest::curve_curve(self, other);
        (self.point_at(u), other.point_at(v))
    }

    /// Get tangent vector at parameter t
    pub fn tangent_at(&self, t: f64) -> Vector {
        if !self.is_valid() {
            return Vector::new(0.0, 0.0, 0.0);
        }

        let (t0, t1) = self.domain();
        let h = (t1 - t0) * 1e-7;

        let (p1, p2) = if t <= t0 + h {
            (self.point_at(t0), self.point_at(t0 + h))
        } else if t >= t1 - h {
            (self.point_at(t1 - h), self.point_at(t1))
        } else {
            (self.point_at(t - h), self.point_at(t + h))
        };

        let tan = Vector::new(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
        let mag = tan.magnitude();
        if mag > 1e-14 {
            tan.normalized()
        } else {
            tan
        }
    }

    /// Get Frenet frame at parameter t (tangent, normal, binormal)
    pub fn plane_at(&self, t: f64, normalized: bool) -> Plane {
        if !self.is_valid() {
            return Plane::invalid();
        }

        let (t0, t1) = self.domain();
        let param = if normalized {
            if t < 0.0 || t > 1.0 {
                return Plane::invalid();
            }
            t0 + t * (t1 - t0)
        } else {
            if t < t0 || t > t1 {
                return Plane::invalid();
            }
            t
        };

        let origin = self.point_at(param);
        let derivs = self.evaluate(param, 2);
        if derivs.len() < 3 {
            return Plane::invalid();
        }

        let d1 = &derivs[1];
        let d2 = &derivs[2];

        let d1_mag = d1.magnitude();
        if d1_mag < 1e-14 {
            return Plane::invalid();
        }

        let tangent = d1.normalized();

        let d2_dot_t = d2.dot(&tangent);
        let mut normal = Vector::new(
            d2[0] - d2_dot_t * tangent[0],
            d2[1] - d2_dot_t * tangent[1],
            d2[2] - d2_dot_t * tangent[2],
        );

        let n_mag = normal.magnitude();
        if n_mag < 1e-14 {
            let world_z = Vector::new(0.0, 0.0, 1.0);
            normal = tangent.cross(&world_z);
            if normal.magnitude() < 1e-14 {
                let world_y = Vector::new(0.0, 1.0, 0.0);
                normal = tangent.cross(&world_y);
            }
        }
        normal = normal.normalized();

        let binormal = tangent.cross(&normal).normalized();

        Plane::from_frame(origin, tangent, normal, binormal)
    }

    /// Get rotation minimizing perpendicular plane at parameter t
    /// Uses the exact Double Reflection algorithm for accuracy
    pub fn perpendicular_plane_at(&self, t: f64, normalized: bool) -> Plane {
        if !self.is_valid() {
            return Plane::invalid();
        }

        let (t0, t1) = self.domain();
        let param = if normalized {
            if t < 0.0 || t > 1.0 {
                return Plane::invalid();
            }
            t0 + t * (t1 - t0)
        } else {
            if t < t0 || t > t1 {
                return Plane::invalid();
            }
            t
        };

        // Get initial frame at t0 using Frenet (curvature-based)
        let derivs0 = self.evaluate(t0, 2);
        let d1_0 = &derivs0[1];
        let d2_0 = &derivs0[2];

        let d1_0_mag = d1_0.magnitude();
        if d1_0_mag < 1e-14 {
            return Plane::invalid();
        }

        let tangent0 = d1_0.normalized();

        // Initial normal from curvature (Frenet)
        let d2_dot_d1 = d2_0.dot(d1_0);
        let d1_0_mag_sq = d1_0_mag * d1_0_mag;
        let mut n0_unnorm = Vector::new(
            d2_0[0] - (d2_dot_d1 / d1_0_mag_sq) * d1_0[0],
            d2_0[1] - (d2_dot_d1 / d1_0_mag_sq) * d1_0[1],
            d2_0[2] - (d2_dot_d1 / d1_0_mag_sq) * d1_0[2],
        );

        let mut n0_mag = n0_unnorm.magnitude();
        if n0_mag < 1e-14 {
            let world_z = Vector::new(0.0, 0.0, 1.0);
            n0_unnorm = world_z.cross(&tangent0);
            n0_mag = n0_unnorm.magnitude();
            if n0_mag < 1e-14 {
                let world_y = Vector::new(0.0, 1.0, 0.0);
                n0_unnorm = world_y.cross(&tangent0);
                n0_mag = n0_unnorm.magnitude();
            }
        }
        let r0 = Vector::new(
            n0_unnorm[0] / n0_mag,
            n0_unnorm[1] / n0_mag,
            n0_unnorm[2] / n0_mag,
        );

        let origin = self.point_at(param);

        // If at start, return Frenet frame directly
        if (param - t0).abs() < 1e-14 {
            let s0 = tangent0.cross(&r0).normalized();
            return Plane::from_frame(origin, r0, s0, tangent0);
        }

        // Propagate frame using Double Reflection (RMF) algorithm
        let num_steps = 10.max(((param - t0) / (t1 - t0) * 100.0) as i32);
        let dt = (param - t0) / num_steps as f64;

        let mut ri = r0;
        let mut ti_param = t0;
        let mut xi = self.point_at(ti_param);
        let mut tangent_i = tangent0;

        for _ in 0..num_steps {
            if ti_param >= param - 1e-14 {
                break;
            }
            let ti_next = (ti_param + dt).min(param);
            let xi_next = self.point_at(ti_next);
            let mut tangent_next = self.tangent_at(ti_next);
            tangent_next = tangent_next.normalized();

            let v1 = Vector::new(xi_next[0] - xi[0], xi_next[1] - xi[1], xi_next[2] - xi[2]);
            let c1 = v1.dot(&v1);
            if c1 < 1e-28 {
                ti_param = ti_next;
                xi = xi_next;
                tangent_i = tangent_next;
                continue;
            }

            let ri_dot_v1 = ri.dot(&v1);
            let r_l = Vector::new(
                ri[0] - 2.0 * ri_dot_v1 / c1 * v1[0],
                ri[1] - 2.0 * ri_dot_v1 / c1 * v1[1],
                ri[2] - 2.0 * ri_dot_v1 / c1 * v1[2],
            );

            let ti_dot_v1 = tangent_i.dot(&v1);
            let t_l = Vector::new(
                tangent_i[0] - 2.0 * ti_dot_v1 / c1 * v1[0],
                tangent_i[1] - 2.0 * ti_dot_v1 / c1 * v1[1],
                tangent_i[2] - 2.0 * ti_dot_v1 / c1 * v1[2],
            );

            let v2 = Vector::new(
                tangent_next[0] - t_l[0],
                tangent_next[1] - t_l[1],
                tangent_next[2] - t_l[2],
            );
            let c2 = v2.dot(&v2);
            ri = if c2 < 1e-28 {
                r_l
            } else {
                let rl_dot_v2 = r_l.dot(&v2);
                Vector::new(
                    r_l[0] - 2.0 * rl_dot_v2 / c2 * v2[0],
                    r_l[1] - 2.0 * rl_dot_v2 / c2 * v2[1],
                    r_l[2] - 2.0 * rl_dot_v2 / c2 * v2[2],
                )
            };

            ri = ri.normalized();
            ti_param = ti_next;
            xi = xi_next;
            tangent_i = tangent_next;
        }

        let mut tangent = self.tangent_at(param);
        tangent = tangent.normalized();

        let ri_dot_t = ri.dot(&tangent);
        ri = Vector::new(
            ri[0] - ri_dot_t * tangent[0],
            ri[1] - ri_dot_t * tangent[1],
            ri[2] - ri_dot_t * tangent[2],
        )
        .normalized();

        let s = tangent.cross(&ri).normalized();

        Plane::from_frame(origin, ri, s, tangent)
    }

    /// Get multiple perpendicular planes along the curve
    pub fn get_perpendicular_planes(&self, count: usize) -> Vec<Plane> {
        let (_pts, params) = self.divide_by_count(count + 1, true);
        params
            .iter()
            .map(|&t| self.perpendicular_plane_at(t, false))
            .collect()
    }

    /// Get start point of curve
    pub fn point_at_start(&self) -> Point {
        let (t0, _) = self.domain();
        self.point_at(t0)
    }

    /// Get end point of curve
    pub fn point_at_end(&self) -> Point {
        let (_, t1) = self.domain();
        self.point_at(t1)
    }

    /// Get middle point of curve
    pub fn point_at_middle(&self) -> Point {
        self.point_at(self.domain_middle())
    }

    /// Set the start point of the curve (modifies first CV)
    pub fn set_start_point(&mut self, point: &Point) {
        if self.m_cv_count > 0 {
            self.set_cv(0, point);
        }
    }

    /// Set the end point of the curve (modifies last CV)
    pub fn set_end_point(&mut self, point: &Point) {
        if self.m_cv_count > 0 {
            self.set_cv(self.m_cv_count - 1, point);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Modification Operations
    // ═══════════════════════════════════════════════════════════════════════════

    /// Reverse curve direction
    pub fn reverse(&mut self) -> bool {
        if !self.is_valid() {
            return false;
        }

        let mut temp_cv = vec![0.0; self.m_cv_stride];
        for i in 0..(self.m_cv_count / 2) {
            let j = self.m_cv_count - 1 - i;

            for k in 0..self.m_cv_stride {
                temp_cv[k] = self.m_cv[i * self.m_cv_stride + k];
                self.m_cv[i * self.m_cv_stride + k] = self.m_cv[j * self.m_cv_stride + k];
                self.m_cv[j * self.m_cv_stride + k] = temp_cv[k];
            }
        }

        nurbsknot::reverse(self.m_order, self.m_cv_count, &mut self.m_nurbsknot)
    }

    /// Swap two coordinate indices for all CVs
    pub fn swap_coordinates(&mut self, i: usize, j: usize) {
        if i >= self.m_dim || j >= self.m_dim || i == j {
            return;
        }
        for cv_idx in 0..self.m_cv_count {
            let idx = cv_idx * self.m_cv_stride;
            let temp = self.m_cv[idx + i];
            self.m_cv[idx + i] = self.m_cv[idx + j];
            self.m_cv[idx + j] = temp;
        }
    }

    /// Split curve at parameter t into two curves
    pub fn split(&self, t: f64) -> (NurbsCurve, NurbsCurve) {
        if !self.is_valid() {
            return (NurbsCurve::default(), NurbsCurve::default());
        }

        let (t0, t1) = self.domain();
        if t <= t0 || t >= t1 {
            return (NurbsCurve::default(), NurbsCurve::default());
        }

        // Copy the curve and trim each half. Resampling instead would return an
        // approximation with a rebuilt parameterization, and a failed trim MUST fail the
        // split: handing back the whole curve as a piece is silent overlap corruption.
        let mut left = self.duplicate();
        let mut right = self.duplicate();

        if !left.trim(t0, t) || !right.trim(t, t1) {
            return (NurbsCurve::default(), NurbsCurve::default());
        }

        (left, right)
    }

    /// Extend curve domain using de Boor extrapolation (matches C++ implementation)
    pub fn extend(&mut self, new_t0: f64, new_t1: f64) -> bool {
        if !self.is_valid() || self.is_closed() {
            return false;
        }

        let (d0, d1) = self.domain();
        let cvdim = self.cv_size();
        let mut changed = false;

        // Extend start (new_t0 < current domain start)
        if new_t0 < d0 {
            self.clamp_end(0);
            self.evaluate_nurbs_de_boor_inplace(cvdim, self.m_order, 0, 1, new_t0);
            for i in 0..(self.m_order - 1) {
                self.m_nurbsknot[i] = new_t0;
            }
            changed = true;
        }

        // Extend end (new_t1 > current domain end)
        if new_t1 > d1 {
            self.clamp_end(1);
            let i0 = self.m_cv_count - self.m_order;
            self.evaluate_nurbs_de_boor_inplace(cvdim, self.m_order, i0, -1, new_t1);
            let kc = self.nurbsknot_count();
            for i in (self.m_cv_count - 1)..kc {
                self.m_nurbsknot[i] = new_t1;
            }
            changed = true;
        }

        changed
    }

    /// Make curve rational (all weights = 1.0)
    pub fn make_rational(&mut self) -> bool {
        if self.m_is_rat {
            return true; // Already rational
        }
        if !self.is_valid() {
            return false;
        }

        // Create new CV array with weights
        let new_stride = self.m_dim + 1;
        let mut new_cv = vec![0.0; self.m_cv_count * new_stride];

        for i in 0..self.m_cv_count {
            let old_idx = i * self.m_cv_stride;
            let new_idx = i * new_stride;

            // Copy coordinates
            for j in 0..self.m_dim {
                new_cv[new_idx + j] = self.m_cv[old_idx + j];
            }
            // Set weight to 1.0
            new_cv[new_idx + self.m_dim] = 1.0;
        }

        self.m_cv = new_cv;
        self.m_cv_stride = new_stride;
        self.m_is_rat = true;
        true
    }

    /// Make curve non-rational. If force=false (default), fails when weights differ.
    /// If force=true, sets all weights to 1.0 (changes geometry!).
    pub fn make_non_rational(&mut self, force: bool) -> bool {
        if !self.m_is_rat {
            return true;
        }

        if force {
            for i in 0..self.m_cv_count {
                let idx = i * self.m_cv_stride + self.m_dim;
                self.m_cv[idx] = 1.0;
            }
        } else {
            let w0 = self.weight(0);
            for i in 1..self.m_cv_count {
                if (self.weight(i) - w0).abs() > Tolerance::ZERO_TOLERANCE {
                    return false;
                }
            }
        }

        let new_stride = self.m_dim;
        let mut new_cv = vec![0.0; self.m_cv_count * new_stride];

        for i in 0..self.m_cv_count {
            if let Some(p) = self.get_cv(i) {
                let new_idx = i * new_stride;
                new_cv[new_idx] = p[0];
                if self.m_dim > 1 {
                    new_cv[new_idx + 1] = p[1];
                }
                if self.m_dim > 2 {
                    new_cv[new_idx + 2] = p[2];
                }
            }
        }

        self.m_cv = new_cv;
        self.m_cv_stride = new_stride;
        self.m_is_rat = false;
        true
    }

    /// Clamp curve end (0=start, 1=end, 2=both)
    pub fn clamp_end(&mut self, end: i32) {
        if !self.is_valid() || self.m_order < 2 {
            return;
        }

        // Clamp start
        if end == 0 || end == 2 {
            let nurbsknot_val = self.m_nurbsknot[self.m_order - 2];
            for i in 0..(self.m_order - 2) {
                self.m_nurbsknot[i] = nurbsknot_val;
            }
        }

        // Clamp end
        if end == 1 || end == 2 {
            let kc = self.nurbsknot_count();
            let nurbsknot_val = self.m_nurbsknot[self.m_cv_count - 1];
            for i in self.m_cv_count..kc {
                self.m_nurbsknot[i] = nurbsknot_val;
            }
        }
    }

    pub fn trim(&mut self, t0: f64, t1: f64) -> bool {
        if !self.is_valid() || t0 >= t1 {
            return false;
        }
        let (d0, d1) = self.domain();
        if t0 < d0 - Tolerance::ZERO_TOLERANCE || t1 > d1 + Tolerance::ZERO_TOLERANCE {
            return false;
        }
        let t0 = t0.max(d0);
        let t1 = t1.min(d1);
        if (t0 - d0).abs() < Tolerance::ZERO_TOLERANCE
            && (t1 - d1).abs() < Tolerance::ZERO_TOLERANCE
        {
            return true;
        }
        let p = self.degree();
        let trim_start = t0 > d0 + Tolerance::ZERO_TOLERANCE;
        let trim_end = t1 < d1 - Tolerance::ZERO_TOLERANCE;
        if trim_start {
            if !self.insert_nurbsknot(t0, p) {
                return false;
            }
        }
        if trim_end {
            if !self.insert_nurbsknot(t1, p) {
                return false;
            }
        }
        let full_nurbsknot_count = self.m_cv_count + self.m_order;
        let mut u = vec![0.0; full_nurbsknot_count];
        u[0] = *self.m_nurbsknot.first().unwrap();
        for i in 0..self.m_nurbsknot.len() {
            u[i + 1] = self.m_nurbsknot[i];
        }
        u[full_nurbsknot_count - 1] = *self.m_nurbsknot.last().unwrap();
        let tol = Tolerance::ZERO_TOLERANCE;
        let mut start_span: i32 = -1;
        for i in (0..full_nurbsknot_count).rev() {
            if (u[i] - t0).abs() < tol {
                start_span = i as i32;
                break;
            }
        }
        let mut end_span: i32 = -1;
        for i in 0..full_nurbsknot_count {
            if (u[i] - t1).abs() < tol {
                end_span = i as i32;
                break;
            }
        }
        if start_span < 0 || end_span < 0 || start_span >= end_span {
            return false;
        }
        let mut first_cv = start_span as i32 - p as i32;
        if first_cv < 0 {
            first_cv = 0;
        }
        let first_cv = first_cv as usize;
        let last_cv = (end_span as usize - 1).min(self.m_cv_count - 1);
        let mut new_cv_count = last_cv - first_cv + 1;
        if new_cv_count < self.m_order {
            new_cv_count = self.m_order;
            if first_cv + new_cv_count - 1 >= self.m_cv_count {
                return false;
            }
        }
        let new_nurbsknot_count = new_cv_count + self.m_order - 2;
        let mut new_nurbsknot = vec![0.0; new_nurbsknot_count];
        for i in 0..(p - 1) {
            new_nurbsknot[i] = t0;
        }
        let mid_count = new_nurbsknot_count as i32 - 2 * (p as i32 - 1);
        if mid_count > 0 {
            let src_start = start_span as usize;
            for i in 0..mid_count as usize {
                let src_idx = src_start + i;
                new_nurbsknot[p - 1 + i] = if src_idx < full_nurbsknot_count {
                    u[src_idx]
                } else {
                    t1
                };
            }
        }
        for i in 0..(p - 1) {
            new_nurbsknot[new_nurbsknot_count - p + 1 + i] = t1;
        }
        let mut new_cv = vec![0.0; new_cv_count * self.m_cv_stride];
        for i in 0..new_cv_count {
            let src = (first_cv + i) * self.m_cv_stride;
            let dst = i * self.m_cv_stride;
            for j in 0..self.m_cv_stride {
                new_cv[dst + j] = self.m_cv[src + j];
            }
        }
        self.m_cv_count = new_cv_count;
        self.m_cv = new_cv;
        self.m_nurbsknot = new_nurbsknot;
        true
    }

    pub fn increase_degree(&mut self, desired_degree: usize) -> bool {
        if !self.is_valid() {
            return false;
        }
        if desired_degree < 1 || desired_degree < self.degree() {
            return false;
        }
        if desired_degree == self.degree() {
            return true;
        }
        self.clamp_end(2);
        let del = desired_degree - self.degree();
        for _ in 0..del {
            if !self.increment_nurbs_degree() {
                return false;
            }
        }
        true
    }

    pub fn change_closed_curve_seam(&mut self, t: f64) -> bool {
        if !self.is_valid() || !self.is_closed() {
            return false;
        }
        let (t0, t1) = self.domain();
        let dom_len = t1 - t0;
        let mut s = (t - t0) / dom_len;
        if s < 0.0 || s > 1.0 {
            s = s % 1.0;
            if s < 0.0 {
                s += 1.0;
            }
        }
        let t = t0 + s * dom_len;
        if (t - t0).abs() < Tolerance::ZERO_TOLERANCE || (t - t1).abs() < Tolerance::ZERO_TOLERANCE
        {
            return true;
        }
        if t <= t0 || t >= t1 {
            return true;
        }
        let (left_crv, right_crv) = self.split(t);
        let order = self.m_order;
        let cvdim = self.cv_size();
        let shift = t1 - t0;
        let new_cv_count = right_crv.m_cv_count + left_crv.m_cv_count - 1;
        let new_kc = order + new_cv_count - 2;
        let mut new_cv = vec![0.0; new_cv_count * self.m_cv_stride];
        let mut new_nurbsknots = vec![0.0; new_kc];
        for i in 0..right_crv.m_cv_count {
            for j in 0..cvdim {
                new_cv[i * self.m_cv_stride + j] = right_crv.m_cv[i * right_crv.m_cv_stride + j];
            }
        }
        for i in 1..left_crv.m_cv_count {
            let dst = right_crv.m_cv_count + i - 1;
            for j in 0..cvdim {
                new_cv[dst * self.m_cv_stride + j] = left_crv.m_cv[i * left_crv.m_cv_stride + j];
            }
        }
        let rkc = right_crv.nurbsknot_count();
        for i in 0..rkc {
            new_nurbsknots[i] = right_crv.m_nurbsknot[i];
        }
        let lkc = left_crv.nurbsknot_count();
        for i in (order - 1)..lkc {
            new_nurbsknots[rkc + i - (order - 1)] = left_crv.m_nurbsknot[i] + shift;
        }
        self.m_cv_count = new_cv_count;
        self.m_cv = new_cv;
        self.m_nurbsknot = new_nurbsknots;
        self.set_domain(t, t + dom_len);
        true
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Transformation
    // ═══════════════════════════════════════════════════════════════════════════

    pub fn transform(&mut self, xform: &Xform) {
        for i in 0..self.m_cv_count {
            if let Some(p) = self.get_cv(i) {
                let x = xform.m[0] * p[0] + xform.m[4] * p[1] + xform.m[8] * p[2] + xform.m[12];
                let y = xform.m[1] * p[0] + xform.m[5] * p[1] + xform.m[9] * p[2] + xform.m[13];
                let z = xform.m[2] * p[0] + xform.m[6] * p[1] + xform.m[10] * p[2] + xform.m[14];
                if self.m_is_rat {
                    let w = self.weight(i);
                    self.set_cv_4d(i, x * w, y * w, z * w, w);
                } else {
                    self.set_cv(i, &Point::new(x, y, z));
                }
            }
        }
    }

    pub fn transformed(&self, xform: &Xform) -> NurbsCurve {
        let mut result = self.clone();
        result.guid = std::sync::OnceLock::new();
        result.transform(xform);
        result
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON Serialization
    // ═══════════════════════════════════════════════════════════════════════════

    /// Serialize to JSON and write to file
    pub fn file_json_dump(&self, filename: &str) {
        use std::fs::File;
        use std::io::Write;
        if let Ok(json) = serde_json::to_string_pretty(self) {
            if let Ok(mut file) = File::create(filename) {
                let _ = file.write_all(json.as_bytes());
            }
        }
    }

    /// Load from JSON file
    pub fn file_json_load(filename: &str) -> Self {
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

    /// Convert to protobuf binary format
    pub fn to_protobuf(&self) -> Vec<u8> {
        use prost::Message;
        self.to_proto().encode_to_vec()
    }

    /// The proto struct itself — pb_dumps encodes it; Session embeds it directly.
    pub fn to_proto(&self) -> crate::proto::NurbsCurve {
        crate::proto::NurbsCurve {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            dimension: self.m_dim as i32,
            is_rational: self.m_is_rat,
            order: self.m_order as i32,
            cv_count: self.m_cv_count as i32,
            cv_stride: self.m_cv_stride as i32,
            nurbsknots: self.m_nurbsknot.iter().map(|&v| v as f64).collect(),
            cvs: self.m_cv.iter().map(|&v| v as f64).collect(),
            width: self.width as f64,
            pointcolors: self
                .pointcolors
                .iter()
                .map(|c| crate::proto::Color {
                    guid: String::new(),
                    name: String::new(),
                    r: c.r,
                    g: c.g,
                    b: c.b,
                    a: c.a,
                })
                .collect(),
            linecolors: self
                .linecolors
                .iter()
                .map(|c| crate::proto::Color {
                    guid: String::new(),
                    name: String::new(),
                    r: c.r,
                    g: c.g,
                    b: c.b,
                    a: c.a,
                })
                .collect(),
        }
    }

    /// Create NurbsCurve from protobuf binary data
    pub fn from_protobuf(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        Ok(Self::from_proto(crate::proto::NurbsCurve::decode(data)?))
    }

    /// Build from an already-decoded proto — pb_loads decodes then calls this.
    pub fn from_proto(proto: crate::proto::NurbsCurve) -> Self {
        let mut curve = Self::new(
            proto.dimension as usize,
            proto.is_rational,
            proto.order as usize,
            proto.cv_count as usize,
        );
        curve.set_guid(proto.guid.clone());
        curve.name = proto.name;
        curve.m_nurbsknot = proto.nurbsknots.into_iter().map(|v| v as f64).collect();
        curve.m_cv = proto.cvs.into_iter().map(|v| v as f64).collect();
        curve.pointcolors = proto
            .pointcolors
            .iter()
            .map(|c| Color::new(c.r, c.g, c.b, c.a))
            .collect();
        curve.linecolors = proto
            .linecolors
            .iter()
            .map(|c| Color::new(c.r, c.g, c.b, c.a))
            .collect();
        curve
    }

    /// Serialize to protobuf and write to file
    pub fn protobuf_dump(&self, filename: &str) {
        let data = self.to_protobuf();
        std::fs::write(filename, data).expect("Failed to write protobuf file");
    }

    /// Load from protobuf file
    pub fn protobuf_load(filename: &str) -> Self {
        let data = std::fs::read(filename).expect("Failed to read protobuf file");
        Self::from_protobuf(&data).expect("Failed to parse protobuf")
    }

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_else(|_| Self::default())
    }

    pub fn pb_dumps(&self) -> Vec<u8> {
        self.to_protobuf()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        Self::from_protobuf(data)
    }

    pub fn pb_dump(&self, filename: &str) {
        self.protobuf_dump(filename);
    }

    pub fn pb_load(filename: &str) -> Self {
        Self::protobuf_load(filename)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String Representation
    // ═══════════════════════════════════════════════════════════════════════════

    /// Simple string representation
    pub fn str(&self) -> String {
        format!(
            "NurbsCurve(name={}, degree={}, cvs={})",
            self.name,
            self.degree(),
            self.cv_count()
        )
    }

    /// Detailed representation
    pub fn repr(&self) -> String {
        let mut lines = vec![
            "NurbsCurve(".to_string(),
            format!("  name={},", self.name),
            format!("  degree={},", self.degree()),
            format!("  cvs={},", self.m_cv_count),
            format!("  rational={},", self.m_is_rat),
            "  control_points=[".to_string(),
        ];
        for i in 0..self.m_cv_count {
            if let Some(p) = self.get_cv(i) {
                lines.push(format!("    {}, {}, {}", p[0], p[1], p[2]));
            }
        }
        lines.push("  ]".to_string());
        lines.push(")".to_string());
        lines.join("\n")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Internal Helpers
    // ═══════════════════════════════════════════════════════════════════════════

    fn span_is_singular(&self, span_index: usize) -> bool {
        if !self.is_valid() {
            return false;
        }
        if span_index >= self.m_cv_count - self.m_order {
            return false;
        }
        let ki = span_index + self.m_order - 2;
        if self.m_nurbsknot[ki] >= self.m_nurbsknot[ki + 1] {
            return true;
        }
        if let Some(p0) = self.get_cv(span_index) {
            for i in 1..self.m_order {
                if let Some(p) = self.get_cv(span_index + i) {
                    if p0.distance(&p, None) > Tolerance::ZERO_TOLERANCE {
                        return false;
                    }
                }
            }
        }
        true
    }

    fn increment_nurbs_degree(&mut self) -> bool {
        let m = self.clone();
        let sc = m.span_count();
        let new_kcount = m.nurbsknot_count() + sc + 1;
        let new_order = m.order() + 1;
        let new_cv_count = new_kcount - new_order + 2;
        let cvdim = m.cv_size();

        self.m_order = new_order;
        self.m_cv_count = new_cv_count;
        self.m_nurbsknot.resize(new_order + new_cv_count - 2, 0.0);
        self.m_cv.resize(new_cv_count * self.m_cv_stride, 0.0);

        let mut ki: usize = 0;
        let mut ko: usize = 0;
        let mkc = m.nurbsknot_count();
        while ki < mkc {
            let kn = m.m_nurbsknot[ki];
            let mut mult = 1;
            while ki + mult < mkc
                && (m.m_nurbsknot[ki + mult] - kn).abs() < Tolerance::ZERO_TOLERANCE
            {
                mult += 1;
            }
            for _ in 0..=mult {
                self.m_nurbsknot[ko] = kn;
                ko += 1;
            }
            ki += mult;
        }

        for v in self.m_cv.iter_mut() {
            *v = 0.0;
        }

        let mut si_n: usize = 0;
        let mut si_m: usize = 0;
        for _ in 0..sc {
            let span_mult = self.nurbsknot_multiplicity(si_n + self.degree() - 1);
            let skip = self.order() - span_mult;
            for j in skip..self.order() {
                let mut cv_n = vec![0.0; cvdim];
                get_raised_degree_cv(
                    m.order(),
                    cvdim,
                    m.m_cv_stride,
                    &m.m_cv[si_m * m.m_cv_stride..],
                    &m.m_nurbsknot[si_m..],
                    &self.m_nurbsknot[si_n..],
                    j,
                    &mut cv_n,
                );
                let dst = (si_n + j) * self.m_cv_stride;
                for k in 0..cvdim {
                    self.m_cv[dst + k] = cv_n[k];
                }
            }
            si_n = next_span_index(self.order(), self.cv_count(), &self.m_nurbsknot, si_n);
            si_m = next_span_index(m.order(), m.cv_count(), &m.m_nurbsknot, si_m);
        }

        let last_dst = (self.m_cv_count - 1) * self.m_cv_stride;
        let last_src = (m.m_cv_count - 1) * m.m_cv_stride;
        for i in 0..cvdim {
            self.m_cv[i] = m.m_cv[i];
            self.m_cv[last_dst + i] = m.m_cv[last_src + i];
        }
        true
    }

    fn find_span(&self, t: f64) -> usize {
        // Use nurbsknot module function
        nurbsknot::find_span(self.m_order, self.m_cv_count, &self.m_nurbsknot, t)
    }

    /// Compute non-zero basis functions at parameter t
    ///
    /// Implementation matches OpenNURBS Cox-de Boor algorithm with offset nurbsknot pointer.
    fn basis_functions(&self, span: usize, t: f64) -> Vec<f64> {
        let mut basis = vec![0.0; self.m_order];
        let mut left = vec![0.0; self.m_order];
        let mut right = vec![0.0; self.m_order];

        // Offset nurbsknot pointer like OpenNURBS does
        let offset = self.m_order - 2 + span;

        basis[0] = 1.0;

        for j in 1..self.m_order {
            left[j] = t - self.m_nurbsknot[offset + 1 - j];
            right[j] = self.m_nurbsknot[offset + j] - t;
            let mut saved = 0.0;

            for r in 0..j {
                let denom = right[r + 1] + left[j - r];
                let temp = if denom != 0.0 { basis[r] / denom } else { 0.0 };
                basis[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }

            basis[j] = saved;
        }

        basis
    }

    /// Compute basis functions and their derivatives at parameter t
    /// Returns ders[k][j] = k-th derivative of j-th basis function
    fn basis_functions_derivatives(
        &self,
        span: usize,
        t: f64,
        deriv_order: usize,
    ) -> Vec<Vec<f64>> {
        let p = self.degree();
        let n_der = deriv_order.min(p);

        let mut ders = vec![vec![0.0; p + 1]; n_der + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];

        // Use same nurbsknot offset as basis_functions
        let offset = self.m_order - 2 + span;

        ndu[0][0] = 1.0;
        for j in 1..=p {
            left[j] = t - self.m_nurbsknot[offset + 1 - j];
            right[j] = self.m_nurbsknot[offset + j] - t;
            let mut saved = 0.0;
            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }

        // Load basis functions
        for j in 0..=p {
            ders[0][j] = ndu[j][p];
        }

        // Compute derivatives using Eq. 2.10 from The NURBS Book
        let mut a = vec![vec![0.0; p + 1]; 2];
        for r in 0..=p {
            let mut s1 = 0usize;
            let mut s2 = 1usize;
            a[0][0] = 1.0;

            for k in 1..=n_der {
                let mut d = 0.0;
                let rk = r as i32 - k as i32;
                let pk = p as i32 - k as i32;

                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }

                let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2 = if (r as i32 - 1) <= pk { k - 1 } else { p - r };

                for j in j1..=j2 {
                    a[s2][j] = (a[s1][j] - a[s1][j - 1])
                        / ndu[(pk + 1) as usize][(rk + j as i32) as usize];
                    d += a[s2][j] * ndu[(rk + j as i32) as usize][pk as usize];
                }

                if r <= pk as usize {
                    a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                    d += a[s2][k] * ndu[r][pk as usize];
                }

                ders[k][r] = d;
                std::mem::swap(&mut s1, &mut s2);
            }
        }

        // Apply factorial scaling: p!/(p-k)!
        let mut scale = p as f64;
        for k in 1..=n_der {
            for j in 0..=p {
                ders[k][j] *= scale;
            }
            scale *= (p - k) as f64;
        }

        ders
    }

    /// Internal de Boor evaluation for curve extension (modifies CVs in place)
    fn evaluate_nurbs_de_boor_inplace(
        &mut self,
        cvdim: usize,
        order: usize,
        cv_start: usize,
        direction: i32,
        t: f64,
    ) {
        if order < 2 {
            return;
        }

        let stride = self.m_cv_stride;
        for i in 1..order {
            let k0 = if direction > 0 {
                cv_start + i - 1
            } else {
                cv_start + order - i
            };
            let k1 = if direction > 0 {
                k0 + 1
            } else {
                k0.saturating_sub(1)
            };

            let a = self.m_nurbsknot[cv_start + if direction > 0 { order - 1 } else { 0 }];
            let b = self.m_nurbsknot[cv_start + if direction > 0 { i } else { order - 1 - i }];

            if (b - a).abs() < 1e-14 {
                continue;
            }

            let s = (t - a) / (b - a);

            for j in 0..cvdim {
                let cv0_val = self.m_cv[k0 * stride + j];
                let cv1_val = self.m_cv[k1 * stride + j];
                self.m_cv[k0 * stride + j] = cv0_val + s * (cv0_val - cv1_val);
            }
        }
    }

    /// Binomial coefficient C(n, k)
    fn binomial(n: usize, k: usize) -> usize {
        if k > n {
            return 0;
        }
        if k == 0 || k == n {
            return 1;
        }
        let k = k.min(n - k);
        let mut c = 1usize;
        for i in 0..k {
            c = c * (n - i) / (i + 1);
        }
        c
    }
}

fn evaluate_nurbs_blossom(
    cvdim: usize,
    order: usize,
    cv_stride: usize,
    cv: &[f64],
    nurbsknot: &[f64],
    t: &[f64],
    p: &mut [f64],
) -> bool {
    if cv_stride < cvdim {
        return false;
    }
    let degree = order - 1;
    for i in 1..(2 * degree) {
        if nurbsknot[i] - nurbsknot[i - 1] < 0.0 {
            return false;
        }
    }
    if nurbsknot[degree] - nurbsknot[degree - 1] < Tolerance::ZERO_TOLERANCE {
        return false;
    }
    let mut space = vec![0.0; order];
    for i in 0..cvdim {
        for j in 0..order {
            space[j] = cv[j * cv_stride + i];
        }
        for j in 1..order {
            for k in j..order {
                let denom = nurbsknot[degree + k - j] - nurbsknot[k - 1];
                space[k - j] = (nurbsknot[degree + k - j] - t[j - 1]) / denom * space[k - j]
                    + (t[j - 1] - nurbsknot[k - 1]) / denom * space[k - j + 1];
            }
        }
        p[i] = space[0];
    }
    true
}

fn get_raised_degree_cv(
    old_order: usize,
    cvdim: usize,
    old_cv_stride: usize,
    old_cv: &[f64],
    old_kn: &[f64],
    new_kn: &[f64],
    cv_id: usize,
    new_cv: &mut [f64],
) -> bool {
    let old_degree = old_order - 1;
    let new_degree = old_degree + 1;
    let mut t = vec![0.0; old_degree];
    let mut pp = vec![0.0; cvdim];
    for v in new_cv.iter_mut() {
        *v = 0.0;
    }
    let kn = &new_kn[cv_id..];
    for i in 0..new_degree {
        let mut k = 0;
        for j in 0..new_degree {
            if j != i {
                t[k] = kn[j];
                k += 1;
            }
        }
        if !evaluate_nurbs_blossom(cvdim, old_order, old_cv_stride, old_cv, old_kn, &t, &mut pp) {
            return false;
        }
        for k in 0..cvdim {
            new_cv[k] += pp[k];
        }
    }
    let denom = new_degree as f64;
    for i in 0..cvdim {
        new_cv[i] /= denom;
    }
    true
}

fn next_span_index(
    order: usize,
    cv_count: usize,
    nurbsknot: &[f64],
    mut span_index: usize,
) -> usize {
    if span_index > cv_count - order {
        return span_index;
    }
    if span_index < cv_count - order {
        span_index += 1;
        while span_index < cv_count - order
            && nurbsknot[span_index + order - 2] == nurbsknot[span_index + order - 1]
        {
            span_index += 1;
        }
    }
    span_index
}

impl Default for NurbsCurve {
    fn default() -> Self {
        Self::default()
    }
}
