use serde::{Deserialize, Deserializer, Serializer};
use serde::ser::SerializeMap;
use std::fmt;
use std::ops::{Add, Index, IndexMut, Mul, Sub};

/// An NxM general-purpose matrix with row-major storage
#[derive(Clone)]
pub struct Matrix {
    pub typ: String,
    guid: std::sync::OnceLock<String>,
    pub name: String,
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<f64>,
}

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
            #[serde(default)]
            guid: Option<String>,
            #[serde(default)]
            name: Option<String>,
            #[serde(default)]
            rows: i64,
            #[serde(default)]
            cols: i64,
            #[serde(default)]
            data: Vec<f64>,
        }
        let d = MatrixData::deserialize(deserializer)?;
        let guid = std::sync::OnceLock::new();
        if let Some(g) = d.guid {
            let _ = guid.set(g);
        }
        Ok(Matrix {
            typ: "Matrix".to_string(),
            guid,
            name: d.name.unwrap_or_else(|| "my_matrix".to_string()),
            rows: d.rows as usize,
            cols: d.cols as usize,
            data: d.data,
        })
    }
}

impl PartialEq for Matrix {
    fn eq(&self, other: &Self) -> bool {
        if self.rows != other.rows || self.cols != other.cols {
            return false;
        }
        for (a, b) in self.data.iter().zip(other.data.iter()) {
            if (a - b).abs() > 1e-10 {
                return false;
            }
        }
        true
    }
}

impl fmt::Display for Matrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Matrix({}x{})", self.rows, self.cols)
    }
}

impl fmt::Debug for Matrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.repr())
    }
}

impl Index<(usize, usize)> for Matrix {
    type Output = f64;
    fn index(&self, (r, c): (usize, usize)) -> &f64 {
        &self.data[r * self.cols + c]
    }
}

impl IndexMut<(usize, usize)> for Matrix {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut f64 {
        let idx = r * self.cols + c;
        &mut self.data[idx]
    }
}

impl Add for Matrix {
    type Output = Matrix;
    fn add(self, other: Matrix) -> Matrix {
        self.add_mat(&other)
    }
}

impl Sub for Matrix {
    type Output = Matrix;
    fn sub(self, other: Matrix) -> Matrix {
        self.subtract(&other)
    }
}

impl Mul for Matrix {
    type Output = Matrix;
    fn mul(self, other: Matrix) -> Matrix {
        self.multiply(&other)
    }
}

impl Matrix {
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Constructors
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn zeros(rows: usize, cols: usize) -> Self {
        Matrix {
            typ: "Matrix".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_matrix".to_string(),
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    pub fn identity(n: usize) -> Self {
        let mut m = Self::zeros(n, n);
        for i in 0..n {
            m[(i, i)] = 1.0;
        }
        m
    }

    pub fn from_vec(rows: usize, cols: usize, data: Vec<f64>) -> Self {
        Matrix {
            typ: "Matrix".to_string(),
            guid: std::sync::OnceLock::new(),
            name: "my_matrix".to_string(),
            rows,
            cols,
            data,
        }
    }

    pub fn from_rows(rows_list: &[Vec<f64>]) -> Self {
        let r = rows_list.len();
        let c = if r > 0 { rows_list[0].len() } else { 0 };
        let mut m = Self::zeros(r, c);
        for i in 0..r {
            for j in 0..c {
                m[(i, j)] = rows_list[i][j];
            }
        }
        m
    }

    pub fn from_cols(cols_list: &[Vec<f64>]) -> Self {
        let c = cols_list.len();
        let r = if c > 0 { cols_list[0].len() } else { 0 };
        let mut m = Self::zeros(r, c);
        for j in 0..c {
            for i in 0..r {
                m[(i, j)] = cols_list[j][i];
            }
        }
        m
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Properties
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn is_square(&self) -> bool {
        self.rows == self.cols
    }

    pub fn is_symmetric(&self) -> bool {
        if !self.is_square() {
            return false;
        }
        for i in 0..self.rows {
            for j in (i + 1)..self.cols {
                if (self[(i, j)] - self[(j, i)]).abs() > 1e-10 {
                    return false;
                }
            }
        }
        true
    }

    pub fn trace(&self) -> f64 {
        let mut sum = 0.0;
        for i in 0..self.rows {
            sum += self[(i, i)];
        }
        sum
    }

    pub fn duplicate(&self) -> Self {
        let mut m = Self::from_vec(self.rows, self.cols, self.data.clone());
        m.name = self.name.clone();
        m
    }

    pub fn str(&self) -> String {
        format!("Matrix({}x{})", self.rows, self.cols)
    }

    pub fn repr(&self) -> String {
        let guid_str = self.guid();
        let guid_prefix = &guid_str[..8.min(guid_str.len())];
        let rows_str: Vec<String> = (0..self.rows)
            .map(|i| {
                let row: Vec<String> = (0..self.cols)
                    .map(|j| format!("{:.6}", self[(i, j)]))
                    .collect();
                format!("[{}]", row.join(", "))
            })
            .collect();
        format!(
            "Matrix(name='{}', guid='{}...', rows={}, cols={}, data=[{}])",
            self.name, guid_prefix, self.rows, self.cols, rows_str.join("; ")
        )
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Basic Operations
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn add_mat(&self, other: &Matrix) -> Matrix {
        assert!(self.rows == other.rows && self.cols == other.cols);
        let data: Vec<f64> = self.data.iter().zip(other.data.iter()).map(|(a, b)| a + b).collect();
        Self::from_vec(self.rows, self.cols, data)
    }

    pub fn subtract(&self, other: &Matrix) -> Matrix {
        assert!(self.rows == other.rows && self.cols == other.cols);
        let data: Vec<f64> = self.data.iter().zip(other.data.iter()).map(|(a, b)| a - b).collect();
        Self::from_vec(self.rows, self.cols, data)
    }

    pub fn scale(&self, s: f64) -> Matrix {
        let data: Vec<f64> = self.data.iter().map(|x| x * s).collect();
        Self::from_vec(self.rows, self.cols, data)
    }

    pub fn multiply(&self, other: &Matrix) -> Matrix {
        assert_eq!(self.cols, other.rows);
        let mut result = Self::zeros(self.rows, other.cols);
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

    pub fn transpose(&self) -> Matrix {
        let mut result = Self::zeros(self.cols, self.rows);
        for i in 0..self.rows {
            for j in 0..self.cols {
                result[(j, i)] = self[(i, j)];
            }
        }
        result
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Linear Algebra
    ///////////////////////////////////////////////////////////////////////////////////////////

    fn lu_internal(&self) -> (Matrix, Matrix, Matrix, usize) {
        let n = self.rows;
        let mut u = self.duplicate();
        let mut l = Self::identity(n);
        let mut p = Self::identity(n);
        let mut swaps = 0usize;
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
                    let tmp = u[(k, j)]; u[(k, j)] = u[(max_row, j)]; u[(max_row, j)] = tmp;
                }
                for j in 0..n {
                    let tmp = p[(k, j)]; p[(k, j)] = p[(max_row, j)]; p[(max_row, j)] = tmp;
                }
                for j in 0..k {
                    let tmp = l[(k, j)]; l[(k, j)] = l[(max_row, j)]; l[(max_row, j)] = tmp;
                }
                swaps += 1;
            }
            if u[(k, k)].abs() < 1e-14 {
                continue;
            }
            for i in (k + 1)..n {
                let factor = u[(i, k)] / u[(k, k)];
                l[(i, k)] = factor;
                for j in k..n {
                    let sub = factor * u[(k, j)];
                    u[(i, j)] -= sub;
                }
            }
        }
        (l, u, p, swaps)
    }

    pub fn lu_decompose(&self) -> (Matrix, Matrix, Matrix) {
        assert!(self.is_square());
        let (l, u, p, _) = self.lu_internal();
        (l, u, p)
    }

    pub fn determinant(&self) -> f64 {
        assert!(self.is_square());
        let n = self.rows;
        if n == 1 {
            return self[(0, 0)];
        }
        if n == 2 {
            return self[(0, 0)] * self[(1, 1)] - self[(0, 1)] * self[(1, 0)];
        }
        let (_, u, _, swaps) = self.lu_internal();
        let sign = if swaps % 2 == 0 { 1.0 } else { -1.0 };
        let mut prod = 1.0;
        for i in 0..n {
            prod *= u[(i, i)];
        }
        sign * prod
    }

    pub fn inverse(&self) -> Option<Matrix> {
        if !self.is_square() {
            return None;
        }
        let n = self.rows;
        let (l, u, p, _) = self.lu_internal();
        for i in 0..n {
            if u[(i, i)].abs() < 1e-14 {
                return None;
            }
        }
        let mut result = Self::zeros(n, n);
        let eye = Self::identity(n);
        for col in 0..n {
            let pb: Vec<f64> = (0..n).map(|i| (0..n).map(|j| p[(i, j)] * eye[(j, col)]).sum()).collect();
            let mut y = vec![0.0f64; n];
            for i in 0..n {
                y[i] = pb[i];
                for j in 0..i {
                    y[i] -= l[(i, j)] * y[j];
                }
            }
            let mut x = vec![0.0f64; n];
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

    pub fn solve(&self, b: &Matrix) -> Option<Matrix> {
        if !self.is_square() || b.rows != self.rows || b.cols != 1 {
            return None;
        }
        let n = self.rows;
        let (l, u, p, _) = self.lu_internal();
        for i in 0..n {
            if u[(i, i)].abs() < 1e-14 {
                return None;
            }
        }
        let pb: Vec<f64> = (0..n).map(|i| (0..n).map(|j| p[(i, j)] * b[(j, 0)]).sum()).collect();
        let mut y = vec![0.0f64; n];
        for i in 0..n {
            y[i] = pb[i];
            for j in 0..i {
                y[i] -= l[(i, j)] * y[j];
            }
        }
        let mut x = vec![0.0f64; n];
        for i in (0..n).rev() {
            x[i] = y[i];
            for j in (i + 1)..n {
                x[i] -= u[(i, j)] * x[j];
            }
            x[i] /= u[(i, i)];
        }
        let mut result = Self::zeros(n, 1);
        for i in 0..n {
            result[(i, 0)] = x[i];
        }
        Some(result)
    }

    pub fn qr_decompose(&self) -> (Matrix, Matrix) {
        let (m, n) = (self.rows, self.cols);
        let a_cols: Vec<Vec<f64>> = (0..n).map(|j| (0..m).map(|i| self[(i, j)]).collect()).collect();
        let mut q_cols: Vec<Vec<f64>> = Vec::new();
        let mut r = Self::zeros(n, n);
        for j in 0..n {
            let mut v = a_cols[j].clone();
            for i in 0..j {
                let rij: f64 = q_cols[i].iter().zip(v.iter()).map(|(a, b)| a * b).sum();
                r[(i, j)] = rij;
                for k in 0..m {
                    v[k] -= rij * q_cols[i][k];
                }
            }
            let norm: f64 = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            r[(j, j)] = norm;
            if norm > 1e-14 {
                q_cols.push(v.iter().map(|x| x / norm).collect());
            } else {
                q_cols.push(vec![0.0; m]);
            }
        }
        let mut q = Self::zeros(m, n);
        for j in 0..n {
            for i in 0..m {
                q[(i, j)] = q_cols[j][i];
            }
        }
        (q, r)
    }

    pub fn cholesky(&self) -> Option<Matrix> {
        if !self.is_square() {
            return None;
        }
        let n = self.rows;
        let mut l = Self::zeros(n, n);
        for i in 0..n {
            for j in 0..=i {
                let mut s = self[(i, j)];
                for k in 0..j {
                    s -= l[(i, k)] * l[(j, k)];
                }
                if i == j {
                    if s <= 0.0 {
                        return None;
                    }
                    l[(i, j)] = s.sqrt();
                } else {
                    l[(i, j)] = s / l[(j, j)];
                }
            }
        }
        Some(l)
    }

    pub fn eigenvalues(&self) -> Vec<f64> {
        assert!(self.is_square());
        let n = self.rows;
        let mut a = self.duplicate();
        for _ in 0..(1000 * n) {
            let (q, r) = a.qr_decompose();
            a = r.multiply(&q);
            let converged = (0..(n - 1)).all(|i| a[(i + 1, i)].abs() < 1e-10);
            if converged {
                break;
            }
        }
        (0..n).map(|i| a[(i, i)]).collect()
    }

    fn eigen_decompose_symmetric(&self) -> Vec<(f64, Vec<f64>)> {
        let n = self.rows;
        let mut a = self.duplicate();
        let mut v = Self::identity(n);
        for _ in 0..(1000 * n) {
            let (q, r) = a.qr_decompose();
            a = r.multiply(&q);
            v = v.multiply(&q);
            let converged = (0..(n - 1)).all(|i| a[(i + 1, i)].abs() < 1e-10);
            if converged {
                break;
            }
        }
        (0..n).map(|i| (a[(i, i)], (0..n).map(|j| v[(j, i)]).collect())).collect()
    }

    pub fn svd(&self) -> (Matrix, Vec<f64>, Matrix) {
        let (m, n) = (self.rows, self.cols);
        let at = self.transpose();
        let ata = at.multiply(self);
        let mut pairs = ata.eigen_decompose_symmetric();
        pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let k = m.min(n);
        let mut singular_values = Vec::with_capacity(k);
        let mut v_cols: Vec<Vec<f64>> = Vec::with_capacity(k);
        for i in 0..k {
            let (ev, evec) = &pairs[i];
            singular_values.push(ev.max(0.0).sqrt());
            v_cols.push(evec.clone());
        }
        let mut v = Self::zeros(n, k);
        for j in 0..k {
            for i in 0..n {
                v[(i, j)] = v_cols[j][i];
            }
        }
        let mut u = Self::zeros(m, k);
        for j in 0..k {
            if singular_values[j] > 1e-12 {
                for i in 0..m {
                    let val: f64 = (0..n).map(|l| self[(i, l)] * v[(l, j)]).sum();
                    u[(i, j)] = val / singular_values[j];
                }
            }
        }
        let vt = v.transpose();
        (u, singular_values, vt)
    }

    pub fn norm_frobenius(&self) -> f64 {
        self.data.iter().map(|x| x * x).sum::<f64>().sqrt()
    }

    pub fn norm_1(&self) -> f64 {
        let mut max_sum = 0.0f64;
        for j in 0..self.cols {
            let col_sum: f64 = (0..self.rows).map(|i| self[(i, j)].abs()).sum();
            if col_sum > max_sum {
                max_sum = col_sum;
            }
        }
        max_sum
    }

    pub fn norm_inf(&self) -> f64 {
        let mut max_sum = 0.0f64;
        for i in 0..self.rows {
            let row_sum: f64 = (0..self.cols).map(|j| self[(i, j)].abs()).sum();
            if row_sum > max_sum {
                max_sum = row_sum;
            }
        }
        max_sum
    }

    pub fn rank(&self) -> usize {
        let (_, sv, _) = self.svd();
        if sv.is_empty() {
            return 0;
        }
        let max_sv = sv.iter().cloned().fold(0.0f64, f64::max);
        let threshold = self.rows.max(self.cols) as f64 * max_sv * 1e-10;
        sv.iter().filter(|&&s| s > threshold).count()
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn jsondump(&self) -> Result<String, Box<dyn std::error::Error>> {
        crate::file_encoders::sorted_json_string(self)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(serde_json::from_str(json_data)?)
    }

    pub fn file_json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn file_json_loads(json_string: &str) -> Self {
        Self::jsonload(json_string).unwrap_or_else(|_| Self::zeros(0, 0))
    }

    pub fn file_json_dump(&self, filepath: &str) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.jsondump()?;
        std::fs::write(filepath, json)?;
        Ok(())
    }

    pub fn file_json_load(filepath: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let json = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json)
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::Matrix {
            guid: self.guid().to_string(),
            name: self.name.clone(),
            rows: self.rows as i32,
            cols: self.cols as i32,
            data: self.data.clone(),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::Matrix::decode(data)?;
        let mut m = Self::from_vec(proto.rows as usize, proto.cols as usize, proto.data);
        m.set_guid(proto.guid);
        m.name = proto.name;
        Ok(m)
    }

    pub fn pb_dump(&self, filepath: &str) {
        let data = self.pb_dumps();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

pub use Matrix as MatrixAlias;
