use crate::tolerance::Tolerance;
use serde::ser::SerializeMap;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serializer;
use std::cmp::Ordering;
use std::fmt;
use std::ops::Add;
use std::ops::Index;
use std::ops::IndexMut;
use std::ops::Mul;
use std::ops::Sub;
use std::sync::OnceLock;

const COMPARISON_TOLERANCE: f64 = Tolerance::ABSOLUTE / 10.0;
const PIVOT_TOLERANCE: f64 = Tolerance::ZERO_TOLERANCE / 100.0;
const SINGULAR_TOLERANCE: f64 = Tolerance::ZERO_TOLERANCE;

/// Return the checked element count for C++-compatible dimensions.
fn matrix_size(rows: usize, cols: usize) -> Option<usize> {
    if rows > i32::MAX as usize || cols > i32::MAX as usize {
        return None;
    }

    rows.checked_mul(cols)
}

/// Order (eigenvalue, eigenvector) pairs by descending eigenvalue.
fn eigen_pair_greater(a: &(f64, Vec<f64>), b: &(f64, Vec<f64>)) -> Ordering {
    b.0.partial_cmp(&a.0).unwrap_or(Ordering::Equal)
}

/// An NxM matrix with row-major storage.
#[derive(Clone)]
pub struct Matrix {
    guid: OnceLock<String>, // Lazily minted GUID.
    pub name: String,       // Matrix name.
    pub rows: usize,        // Row count.
    pub cols: usize,        // Column count.
    pub data: Vec<f64>,     // Row-major values.
}

impl Matrix {
    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct a rows x cols matrix of zeros; panics for invalid dimensions.
    pub fn new(rows: usize, cols: usize) -> Self {
        let size = matrix_size(rows, cols).expect("Matrix dimensions are too large");

        Matrix {
            guid: OnceLock::new(),
            name: "my_matrix".to_string(),
            rows,
            cols,
            data: vec![0.0; size],
        }
    }

    /// Construct a rows x cols zero matrix.
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self::new(rows, cols)
    }

    /// Construct an n x n identity matrix.
    pub fn identity(n: usize) -> Self {
        let mut m = Self::new(n, n);

        for i in 0..n {
            m[(i, i)] = 1.0;
        }

        m
    }

    /// Construct from exact row-major data; panics when the size does not match.
    pub fn from_vec(rows: usize, cols: usize, data: Vec<f64>) -> Self {
        let size = matrix_size(rows, cols).expect("Matrix dimensions are too large");
        assert_eq!(
            data.len(),
            size,
            "Matrix data size does not match its dimensions"
        );

        let mut m = Self::new(rows, cols);
        m.data = data;

        m
    }

    /// Construct from equal-length rows.
    pub fn from_rows(rows_list: &[Vec<f64>]) -> Self {
        let r = rows_list.len();
        let c = if r > 0 { rows_list[0].len() } else { 0 };

        for row in rows_list {
            assert_eq!(row.len(), c, "Matrix rows must have equal lengths");
        }

        let mut m = Self::new(r, c);

        for i in 0..r {
            for j in 0..c {
                m[(i, j)] = rows_list[i][j];
            }
        }

        m
    }

    /// Construct from equal-length columns.
    pub fn from_cols(cols_list: &[Vec<f64>]) -> Self {
        let c = cols_list.len();
        let r = if c > 0 { cols_list[0].len() } else { 0 };

        for col in cols_list {
            assert_eq!(col.len(), r, "Matrix columns must have equal lengths");
        }

        let mut m = Self::new(r, c);

        for j in 0..c {
            for i in 0..r {
                m[(i, j)] = cols_list[j][i];
            }
        }

        m
    }

    /// Copy with a new guid and the same data.
    pub fn duplicate(&self) -> Self {
        let mut m = Self::from_vec(self.rows, self.cols, self.data.clone());
        m.name = self.name.clone();

        m
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

    /// Set the guid if it has not already been created.
    pub fn set_guid(&self, guid: String) {
        let _ = self.guid.set(guid);
    }

    /// Return whether the matrix has equal row and column counts.
    pub fn is_square(&self) -> bool {
        self.rows == self.cols
    }

    /// Return whether the matrix is square and symmetric.
    pub fn is_symmetric(&self) -> bool {
        if !self.is_square() {
            return false;
        }

        for i in 0..self.rows {
            for j in (i + 1)..self.cols {
                if (self[(i, j)] - self[(j, i)]).abs() > COMPARISON_TOLERANCE {
                    return false;
                }
            }
        }

        true
    }

    /// Return the diagonal sum; panics unless the matrix is square.
    pub fn trace(&self) -> f64 {
        assert!(self.is_square(), "Matrix trace requires a square matrix");

        let mut s = 0.0;

        for i in 0..self.rows {
            s += self[(i, i)];
        }

        s
    }
}

impl Default for Matrix {
    /// Construct an empty matrix.
    fn default() -> Self {
        Self::new(0, 0)
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Operators
// ═══════════════════════════════════════════════════════════════════════════
impl Index<(usize, usize)> for Matrix {
    type Output = f64;

    /// Return the element at (row, col).
    fn index(&self, (r, c): (usize, usize)) -> &f64 {
        &self.data[r * self.cols + c]
    }
}

impl IndexMut<(usize, usize)> for Matrix {
    /// Return the mutable element at (row, col).
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut f64 {
        let idx = r * self.cols + c;

        &mut self.data[idx]
    }
}

impl Add<&Matrix> for &Matrix {
    type Output = Matrix;

    /// Add an equal-sized matrix.
    fn add(self, other: &Matrix) -> Matrix {
        assert!(
            self.rows == other.rows && self.cols == other.cols,
            "Matrix dimensions must match for addition"
        );

        let mut result = Matrix::new(self.rows, self.cols);

        for i in 0..self.data.len() {
            result.data[i] = self.data[i] + other.data[i];
        }

        result
    }
}

impl Add for Matrix {
    type Output = Matrix;

    /// Add an equal-sized matrix.
    fn add(self, other: Matrix) -> Matrix {
        &self + &other
    }
}

impl Sub<&Matrix> for &Matrix {
    type Output = Matrix;

    /// Subtract an equal-sized matrix.
    fn sub(self, other: &Matrix) -> Matrix {
        assert!(
            self.rows == other.rows && self.cols == other.cols,
            "Matrix dimensions must match for subtraction"
        );

        let mut result = Matrix::new(self.rows, self.cols);

        for i in 0..self.data.len() {
            result.data[i] = self.data[i] - other.data[i];
        }

        result
    }
}

impl Sub for Matrix {
    type Output = Matrix;

    /// Subtract an equal-sized matrix.
    fn sub(self, other: Matrix) -> Matrix {
        &self - &other
    }
}

impl Mul<&Matrix> for &Matrix {
    type Output = Matrix;

    /// Multiply by a dimension-compatible matrix.
    fn mul(self, other: &Matrix) -> Matrix {
        assert_eq!(
            self.cols, other.rows,
            "Matrix dimensions are incompatible for multiplication"
        );

        let mut result = Matrix::new(self.rows, other.cols);

        for i in 0..self.rows {
            for j in 0..other.cols {
                let mut s = 0.0;

                for k in 0..self.cols {
                    s += self[(i, k)] * other[(k, j)];
                }

                result[(i, j)] = s;
            }
        }

        result
    }
}

impl Mul for Matrix {
    type Output = Matrix;

    /// Multiply by a dimension-compatible matrix.
    fn mul(self, other: Matrix) -> Matrix {
        &self * &other
    }
}

impl Mul<f64> for &Matrix {
    type Output = Matrix;

    /// Multiply every element by a scalar.
    fn mul(self, s: f64) -> Matrix {
        let mut result = Matrix::new(self.rows, self.cols);

        for i in 0..self.data.len() {
            result.data[i] = self.data[i] * s;
        }

        result
    }
}

impl Mul<f64> for Matrix {
    type Output = Matrix;

    /// Multiply every element by a scalar.
    fn mul(self, s: f64) -> Matrix {
        &self * s
    }
}

impl PartialEq for Matrix {
    /// Compare dimensions and values within the matrix comparison tolerance.
    fn eq(&self, other: &Self) -> bool {
        if self.rows != other.rows || self.cols != other.cols {
            return false;
        }

        for i in 0..self.data.len() {
            if (self.data[i] - other.data[i]).abs() > COMPARISON_TOLERANCE {
                return false;
            }
        }

        true
    }
}

impl Matrix {
    // ═══════════════════════════════════════════════════════════════════════════
    // Linear algebra
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the transpose.
    pub fn transpose(&self) -> Matrix {
        let mut result = Self::new(self.cols, self.rows);

        for i in 0..self.rows {
            for j in 0..self.cols {
                result[(j, i)] = self[(i, j)];
            }
        }

        result
    }

    /// Return (L, U, P, swaps) by partial pivoting.
    fn _lu_internal(&self) -> (Matrix, Matrix, Matrix, usize) {
        let n = self.rows;
        let mut u = self.duplicate();
        let mut lower = Self::identity(n);
        let mut p = Self::identity(n);
        let mut swaps = 0;

        for k in 0..n {
            let mut max_val = u[(k, k)].abs();
            let mut max_row = k;

            for i in (k + 1)..n {
                if u[(i, k)].abs() > max_val {
                    max_val = u[(i, k)].abs();
                    max_row = i;
                }
            }

            if max_row != k {
                for j in 0..n {
                    u.data.swap(k * n + j, max_row * n + j);
                }

                for j in 0..n {
                    p.data.swap(k * n + j, max_row * n + j);
                }

                for j in 0..k {
                    lower.data.swap(k * n + j, max_row * n + j);
                }

                swaps += 1;
            }

            if u[(k, k)].abs() < PIVOT_TOLERANCE {
                continue;
            }

            for i in (k + 1)..n {
                let factor = u[(i, k)] / u[(k, k)];
                lower[(i, k)] = factor;

                for j in k..n {
                    u[(i, j)] -= factor * u[(k, j)];
                }
            }
        }

        (lower, u, p, swaps)
    }

    /// Return (L, U, P) with P * A = L * U; panics unless square.
    pub fn lu_decompose(&self) -> (Matrix, Matrix, Matrix) {
        assert!(
            self.is_square(),
            "LU decomposition requires a square matrix"
        );

        let (lower, u, p, _swaps) = self._lu_internal();

        (lower, u, p)
    }

    /// Return the determinant; panics unless the matrix is square.
    pub fn determinant(&self) -> f64 {
        assert!(
            self.is_square(),
            "Matrix determinant requires a square matrix"
        );

        let n = self.rows;

        if n == 1 {
            return self[(0, 0)];
        }

        if n == 2 {
            return self[(0, 0)] * self[(1, 1)] - self[(0, 1)] * self[(1, 0)];
        }

        let (_lower, u, _p, swaps) = self._lu_internal();
        let sign = if swaps % 2 == 0 { 1.0 } else { -1.0 };
        let mut prod = 1.0;

        for i in 0..n {
            prod *= u[(i, i)];
        }

        sign * prod
    }

    /// Return the inverse, or None for a non-square or singular matrix.
    pub fn inverse(&self) -> Option<Matrix> {
        if !self.is_square() {
            return None;
        }

        let n = self.rows;
        let (lower, u, p, _swaps) = self._lu_internal();

        for i in 0..n {
            if u[(i, i)].abs() < PIVOT_TOLERANCE {
                return None;
            }
        }

        let mut result = Self::new(n, n);
        let eye = Self::identity(n);

        for col in 0..n {
            let mut pb = vec![0.0; n];

            for i in 0..n {
                for j in 0..n {
                    pb[i] += p[(i, j)] * eye[(j, col)];
                }
            }

            let mut y = vec![0.0; n];

            for i in 0..n {
                y[i] = pb[i];

                for j in 0..i {
                    y[i] -= lower[(i, j)] * y[j];
                }
            }

            let mut x = vec![0.0; n];

            for i in (0..n).rev() {
                x[i] = y[i];

                for j in (i + 1)..n {
                    x[i] -= u[(i, j)] * x[j];
                }

                x[i] /= u[(i, i)];
            }

            for i in 0..n {
                result[(i, col)] = x[i];
            }
        }

        Some(result)
    }

    /// Return x with A * x = b, or None when no compatible unique solution exists.
    pub fn solve(&self, b: &Matrix) -> Option<Matrix> {
        if !self.is_square() || b.rows != self.rows || b.cols != 1 {
            return None;
        }

        let n = self.rows;
        let (lower, u, p, _swaps) = self._lu_internal();

        for i in 0..n {
            if u[(i, i)].abs() < PIVOT_TOLERANCE {
                return None;
            }
        }

        let mut pb = vec![0.0; n];

        for i in 0..n {
            for j in 0..n {
                pb[i] += p[(i, j)] * b[(j, 0)];
            }
        }

        let mut y = vec![0.0; n];

        for i in 0..n {
            y[i] = pb[i];

            for j in 0..i {
                y[i] -= lower[(i, j)] * y[j];
            }
        }

        let mut x = vec![0.0; n];

        for i in (0..n).rev() {
            x[i] = y[i];

            for j in (i + 1)..n {
                x[i] -= u[(i, j)] * x[j];
            }

            x[i] /= u[(i, i)];
        }

        let mut result = Self::new(n, 1);

        for i in 0..n {
            result[(i, 0)] = x[i];
        }

        Some(result)
    }

    /// Return (Q, R) from Gram-Schmidt decomposition.
    pub fn qr_decompose(&self) -> (Matrix, Matrix) {
        let m = self.rows;
        let n = self.cols;
        let mut a_cols = vec![vec![0.0; m]; n];

        for j in 0..n {
            for i in 0..m {
                a_cols[j][i] = self[(i, j)];
            }
        }

        let mut q_cols: Vec<Vec<f64>> = Vec::new();
        let mut r = Self::zeros(n, n);

        for j in 0..n {
            let mut v = a_cols[j].clone();

            for i in 0..j {
                let mut rij = 0.0;

                for k in 0..m {
                    rij += q_cols[i][k] * v[k];
                }

                r[(i, j)] = rij;

                for k in 0..m {
                    v[k] -= rij * q_cols[i][k];
                }
            }

            let mut norm = 0.0;

            for value in &v {
                norm += value * value;
            }

            norm = f64::sqrt(norm);
            r[(j, j)] = norm;

            let mut qcol = vec![0.0; m];

            if norm > PIVOT_TOLERANCE {
                for k in 0..m {
                    qcol[k] = v[k] / norm;
                }
            }

            q_cols.push(qcol);
        }

        let mut q = Self::zeros(m, n);

        for j in 0..n {
            for i in 0..m {
                q[(i, j)] = q_cols[j][i];
            }
        }

        (q, r)
    }

    /// Return lower L with A = L * L^T, or None when not positive definite.
    pub fn cholesky(&self) -> Option<Matrix> {
        if !self.is_square() {
            return None;
        }

        let n = self.rows;
        let mut lower = Self::new(n, n);

        for i in 0..n {
            for j in 0..=i {
                let mut s = self[(i, j)];

                for k in 0..j {
                    s -= lower[(i, k)] * lower[(j, k)];
                }

                if i == j {
                    if s <= 0.0 {
                        return None;
                    }

                    lower[(i, j)] = s.sqrt();
                } else {
                    lower[(i, j)] = s / lower[(j, j)];
                }
            }
        }

        Some(lower)
    }

    /// Return eigenvalues by bounded unshifted QR iteration; panics unless square.
    pub fn eigenvalues(&self) -> Vec<f64> {
        assert!(
            self.is_square(),
            "Matrix eigenvalues require a square matrix"
        );

        let n = self.rows;
        let mut a = self.duplicate();

        for _ in 0..(1000 * n) {
            let (q, r) = a.qr_decompose();
            a = &r * &q;

            let mut converged = true;

            for i in 1..n {
                if a[(i, i - 1)].abs() >= COMPARISON_TOLERANCE {
                    converged = false;
                    break;
                }
            }

            if converged {
                break;
            }
        }

        let mut ev = vec![0.0; n];

        for i in 0..n {
            ev[i] = a[(i, i)];
        }

        ev
    }

    /// Return (eigenvalue, eigenvector) pairs by QR iteration with accumulated Q.
    fn _eigen_decompose_symmetric(&self) -> Vec<(f64, Vec<f64>)> {
        let n = self.rows;
        let mut a = self.duplicate();
        let mut v = Self::identity(n);

        for _ in 0..(1000 * n) {
            let (q, r) = a.qr_decompose();
            a = &r * &q;
            v = &v * &q;

            let mut converged = true;

            for i in 1..n {
                if a[(i, i - 1)].abs() >= COMPARISON_TOLERANCE {
                    converged = false;
                    break;
                }
            }

            if converged {
                break;
            }
        }

        let mut pairs = Vec::new();

        for i in 0..n {
            let mut evec = vec![0.0; n];

            for j in 0..n {
                evec[j] = v[(j, i)];
            }

            pairs.push((a[(i, i)], evec));
        }

        pairs
    }

    /// Return (U, singular values, V^T).
    pub fn svd(&self) -> (Matrix, Vec<f64>, Matrix) {
        let m = self.rows;
        let n = self.cols;
        let at = self.transpose();
        let ata = &at * self;
        let mut pairs = ata._eigen_decompose_symmetric();
        pairs.sort_by(eigen_pair_greater);

        let k = m.min(n);
        let mut sv = Vec::new();
        let mut v_cols: Vec<Vec<f64>> = Vec::new();

        for pair in &pairs[..k] {
            sv.push(f64::sqrt(f64::max(0.0, pair.0)));
            v_cols.push(pair.1.clone());
        }

        let mut v = Self::zeros(n, k);

        for j in 0..k {
            for i in 0..n {
                v[(i, j)] = v_cols[j][i];
            }
        }

        let mut u = Self::zeros(m, k);

        for j in 0..k {
            if sv[j] <= SINGULAR_TOLERANCE {
                continue;
            }

            for i in 0..m {
                let mut val = 0.0;

                for t in 0..n {
                    val += self[(i, t)] * v[(t, j)];
                }

                u[(i, j)] = val / sv[j];
            }
        }

        (u, sv, v.transpose())
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Norms
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the Frobenius norm.
    pub fn norm_frobenius(&self) -> f64 {
        let mut s = 0.0;

        for x in &self.data {
            s += x * x;
        }

        f64::sqrt(s)
    }

    /// Return the maximum absolute column sum.
    pub fn norm_1(&self) -> f64 {
        let mut max_sum = 0.0;

        for j in 0..self.cols {
            let mut col_sum = 0.0;

            for i in 0..self.rows {
                col_sum += self[(i, j)].abs();
            }

            if col_sum > max_sum {
                max_sum = col_sum;
            }
        }

        max_sum
    }

    /// Return the maximum absolute row sum.
    pub fn norm_inf(&self) -> f64 {
        let mut max_sum = 0.0;

        for i in 0..self.rows {
            let mut row_sum = 0.0;

            for j in 0..self.cols {
                row_sum += self[(i, j)].abs();
            }

            if row_sum > max_sum {
                max_sum = row_sum;
            }
        }

        max_sum
    }

    /// Return the numerical rank.
    pub fn rank(&self) -> usize {
        let (_u, sv, _vt) = self.svd();

        if sv.is_empty() {
            return 0;
        }

        let mut max_sv = 0.0;

        for s in &sv {
            max_sv = f64::max(max_sv, *s);
        }

        let threshold = self.rows.max(self.cols) as f64 * max_sv * COMPARISON_TOLERANCE;
        let mut count = 0;

        for s in &sv {
            if *s > threshold {
                count += 1;
            }
        }

        count
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // JSON
    // ═══════════════════════════════════════════════════════════════════════════
    /// Serialize to an ordered JSON string.
    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    /// Deserialize from a JSON string.
    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    /// Serialize to a JSON string.
    pub fn file_json_dumps(&self) -> String {
        self.jsondump().expect("Failed to serialize Matrix JSON")
    }

    /// Deserialize from a JSON string.
    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).expect("Failed to parse Matrix JSON")
    }

    /// Write JSON to a file.
    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;

        Ok(())
    }

    /// Read JSON from a file.
    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;

        Self::jsonload(&json)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Protobuf
    // ═══════════════════════════════════════════════════════════════════════════
    /// Convert to the protobuf message.
    pub fn to_proto(&self) -> crate::proto::Matrix {
        crate::proto::Matrix {
            guid: self.guid.get().cloned().unwrap_or_default(),
            name: self.name.clone(),
            rows: self.rows as i32,
            cols: self.cols as i32,
            data: self.data.clone(),
        }
    }

    /// Construct from a shape-valid protobuf message.
    pub fn from_proto(proto: crate::proto::Matrix) -> Result<Self, Box<dyn std::error::Error>> {
        if proto.rows < 0 || proto.cols < 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Matrix dimensions cannot be negative",
            )
            .into());
        }

        let rows = proto.rows as usize;
        let cols = proto.cols as usize;

        if matrix_size(rows, cols) != Some(proto.data.len()) {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Matrix data size does not match its dimensions",
            )
            .into());
        }

        let mut m = Self::from_vec(rows, cols, proto.data);

        if !proto.guid.is_empty() {
            m.set_guid(proto.guid);
        }

        m.name = proto.name;

        Ok(m)
    }

    /// Serialize to protobuf bytes.
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        self.to_proto().encode_to_vec()
    }

    /// Deserialize from protobuf bytes.
    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        Self::from_proto(crate::proto::Matrix::decode(data)?)
    }

    /// Write protobuf bytes to a file.
    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();

        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    /// Read protobuf bytes from a file.
    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");

        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the compact dimension string.
    pub fn str(&self) -> String {
        format!("Matrix({}x{})", self.rows, self.cols)
    }

    /// Return the detailed representation.
    pub fn repr(&self) -> String {
        let mut rows_str: Vec<String> = Vec::new();

        for i in 0..self.rows {
            let mut row: Vec<String> = Vec::new();

            for j in 0..self.cols {
                row.push(format!("{:.6}", self[(i, j)]));
            }

            rows_str.push(format!("[{}]", row.join(", ")));
        }

        let guid: String = self.guid().chars().take(8).collect();

        format!(
            "Matrix(name='{}', guid='{}...', rows={}, cols={}, data=[{}])",
            self.name,
            guid,
            self.rows,
            self.cols,
            rows_str.join("; ")
        )
    }
}

impl fmt::Display for Matrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.str())
    }
}

impl fmt::Debug for Matrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.repr())
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Serde
// ═══════════════════════════════════════════════════════════════════════════
impl serde::Serialize for Matrix {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = serializer.serialize_map(Some(6))?;
        map.serialize_entry("cols", &(self.cols as i64))?;
        map.serialize_entry("data", &self.data)?;
        map.serialize_entry("guid", self.guid())?;
        map.serialize_entry("name", &self.name)?;
        map.serialize_entry("rows", &(self.rows as i64))?;
        map.serialize_entry("type", "Matrix")?;
        map.end()
    }
}

impl<'de> Deserialize<'de> for Matrix {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct MatrixData {
            cols: usize,
            data: Vec<f64>,
            guid: String,
            name: String,
            rows: usize,
        }

        let d = MatrixData::deserialize(deserializer)?;

        if matrix_size(d.rows, d.cols) != Some(d.data.len()) {
            return Err(serde::de::Error::custom(
                "Matrix data size does not match its dimensions",
            ));
        }

        let mut m = Matrix::from_vec(d.rows, d.cols, d.data);

        if !d.guid.is_empty() {
            m.set_guid(d.guid);
        }

        m.name = d.name;

        Ok(m)
    }
}
