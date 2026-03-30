use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;

pub fn run_matrix_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Matrix;
        let m = Matrix::zeros(2, 3);
        let eye = Matrix::identity(3);
        let ml = Matrix::from_vec(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let mr = Matrix::from_rows(&[vec![1.0, 2.0], vec![3.0, 4.0]]);
        let mc = Matrix::from_cols(&[vec![1.0, 3.0], vec![2.0, 4.0]]);
        let v00 = ml[(0, 0)];
        let v01 = ml[(0, 1)];
        let v10 = ml[(1, 0)];
        let v11 = ml[(1, 1)];
        let eq = ml == mr;
        let ne = ml != Matrix::identity(2);
        let sstr = m.str();
        let srepr = eye.repr();
        let d = ml.duplicate();

        MINI_CHECK!(m.rows == 2 && m.cols == 3);
        MINI_CHECK!(m.name == "my_matrix" && !m.guid().is_empty());
        MINI_CHECK!(eye[(0, 0)] == 1.0 && eye[(1, 1)] == 1.0 && eye[(2, 2)] == 1.0);
        MINI_CHECK!(eye[(0, 1)] == 0.0);
        MINI_CHECK!(v00 == 1.0 && v01 == 2.0 && v10 == 3.0 && v11 == 4.0);
        MINI_CHECK!(mc == ml);
        MINI_CHECK!(eq);
        MINI_CHECK!(ne);
        MINI_CHECK!(sstr.contains("Matrix(2x3)"));
        MINI_CHECK!(srepr.contains("Matrix("));
        MINI_CHECK!(d == ml && d.guid() != ml.guid());
    })
}

pub fn run_matrix_properties() -> TestResult {
    MINI_TEST!("Properties", {
        use crate::Matrix;
        let m1 = Matrix::identity(3);
        let m2 = Matrix::zeros(2, 3);
        let m3 = Matrix::from_vec(3, 3, vec![1.0, 2.0, 3.0, 2.0, 5.0, 6.0, 3.0, 6.0, 9.0]);
        let m4 = Matrix::from_vec(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let sq1 = m1.is_square();
        let sq2 = m2.is_square();
        let sym1 = m3.is_symmetric();
        let sym2 = m4.is_symmetric();
        let tr = m1.trace();

        MINI_CHECK!(sq1);
        MINI_CHECK!(!sq2);
        MINI_CHECK!(sym1);
        MINI_CHECK!(!sym2);
        MINI_CHECK!(TOLERANCE.is_close(tr, 3.0));
    })
}

pub fn run_matrix_add() -> TestResult {
    MINI_TEST!("Add", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let b = Matrix::from_vec(2, 2, vec![5.0, 6.0, 7.0, 8.0]);
        let c = a.add_mat(&b);
        let d = a.clone() + b;

        MINI_CHECK!(c[(0, 0)] == 6.0 && c[(0, 1)] == 8.0);
        MINI_CHECK!(c[(1, 0)] == 10.0 && c[(1, 1)] == 12.0);
        MINI_CHECK!(c == d);
    })
}

pub fn run_matrix_subtract() -> TestResult {
    MINI_TEST!("Subtract", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 2, vec![5.0, 6.0, 7.0, 8.0]);
        let b = Matrix::from_vec(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let c = a.subtract(&b);
        let d = a.clone() - b;

        MINI_CHECK!(c[(0, 0)] == 4.0 && c[(0, 1)] == 4.0);
        MINI_CHECK!(c[(1, 0)] == 4.0 && c[(1, 1)] == 4.0);
        MINI_CHECK!(c == d);
    })
}

pub fn run_matrix_scale() -> TestResult {
    MINI_TEST!("Scale", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let b = a.scale(2.0);
        let c = a.scale(3.0);

        MINI_CHECK!(b[(0, 0)] == 2.0 && b[(0, 1)] == 4.0 && b[(1, 0)] == 6.0 && b[(1, 1)] == 8.0);
        MINI_CHECK!(c[(0, 0)] == 3.0 && c[(1, 1)] == 12.0);
    })
}

pub fn run_matrix_multiply() -> TestResult {
    MINI_TEST!("Multiply", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 3, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let b = Matrix::from_vec(3, 2, vec![7.0, 8.0, 9.0, 10.0, 11.0, 12.0]);
        let c = a.multiply(&b);
        let d = a.clone() * b;

        MINI_CHECK!(c.rows == 2 && c.cols == 2);
        MINI_CHECK!(TOLERANCE.is_close(c[(0, 0)], 58.0) && TOLERANCE.is_close(c[(0, 1)], 64.0));
        MINI_CHECK!(TOLERANCE.is_close(c[(1, 0)], 139.0));
        MINI_CHECK!(TOLERANCE.is_close(c[(1, 1)], 154.0));
        MINI_CHECK!(c == d);
    })
}

pub fn run_matrix_transpose() -> TestResult {
    MINI_TEST!("Transpose", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 3, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let t = a.transpose();

        MINI_CHECK!(t.rows == 3 && t.cols == 2);
        MINI_CHECK!(t[(0, 0)] == 1.0 && t[(1, 0)] == 2.0 && t[(2, 0)] == 3.0);
        MINI_CHECK!(t[(0, 1)] == 4.0 && t[(1, 1)] == 5.0 && t[(2, 1)] == 6.0);
    })
}

pub fn run_matrix_determinant() -> TestResult {
    MINI_TEST!("Determinant", {
        use crate::Matrix;
        let a1 = Matrix::from_vec(1, 1, vec![5.0]);
        let a2 = Matrix::from_vec(2, 2, vec![4.0, 7.0, 2.0, 6.0]);
        let a3 = Matrix::from_vec(3, 3, vec![1.0, 2.0, 3.0, 0.0, 1.0, 4.0, 5.0, 6.0, 0.0]);
        let eye3 = Matrix::identity(3);

        MINI_CHECK!(TOLERANCE.is_close(a1.determinant(), 5.0));
        MINI_CHECK!(TOLERANCE.is_close(a2.determinant(), 10.0));
        MINI_CHECK!(TOLERANCE.is_close(a3.determinant(), 1.0));
        MINI_CHECK!(TOLERANCE.is_close(eye3.determinant(), 1.0));
    })
}

pub fn run_matrix_inverse() -> TestResult {
    MINI_TEST!("Inverse", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 2, vec![4.0, 7.0, 2.0, 6.0]);
        let inv = a.inverse();
        let singular = Matrix::from_vec(2, 2, vec![1.0, 2.0, 2.0, 4.0]);
        let inv_none = singular.inverse();
        let prod = a.multiply(inv.as_ref().unwrap());

        MINI_CHECK!(inv.is_some());
        let inv = inv.unwrap();
        MINI_CHECK!(TOLERANCE.is_close(inv[(0, 0)], 0.6));
        MINI_CHECK!(TOLERANCE.is_close(inv[(0, 1)], -0.7));
        MINI_CHECK!(TOLERANCE.is_close(inv[(1, 0)], -0.2));
        MINI_CHECK!(TOLERANCE.is_close(inv[(1, 1)], 0.4));
        MINI_CHECK!(inv_none.is_none());
        MINI_CHECK!(TOLERANCE.is_close(prod[(0, 0)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(prod[(1, 1)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(prod[(0, 1)], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(prod[(1, 0)], 0.0));
    })
}

pub fn run_matrix_solve() -> TestResult {
    MINI_TEST!("Solve", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 2, vec![2.0, 1.0, 1.0, 3.0]);
        let b = Matrix::from_vec(2, 1, vec![5.0, 10.0]);
        let x = a.solve(&b);
        // 2x+y=5, x+3y=10 → x=1, y=3

        MINI_CHECK!(x.is_some());
        let x = x.unwrap();
        let residual_0 = 2.0 * x[(0, 0)] + 1.0 * x[(1, 0)];
        let residual_1 = 1.0 * x[(0, 0)] + 3.0 * x[(1, 0)];

        MINI_CHECK!(TOLERANCE.is_close(x[(0, 0)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(x[(1, 0)], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(residual_0, 5.0));
        MINI_CHECK!(TOLERANCE.is_close(residual_1, 10.0));
    })
}

pub fn run_matrix_lu_decompose() -> TestResult {
    MINI_TEST!("LuDecompose", {
        use crate::Matrix;
        let a = Matrix::from_vec(3, 3, vec![2.0, 1.0, 1.0, 4.0, 3.0, 3.0, 8.0, 7.0, 9.0]);
        let (l, u, p) = a.lu_decompose();
        let pa = p.multiply(&a);
        let lu = l.multiply(&u);

        MINI_CHECK!(l.rows == 3 && u.cols == 3);
        MINI_CHECK!(TOLERANCE.is_close(pa[(0, 0)], lu[(0, 0)]));
        MINI_CHECK!(TOLERANCE.is_close(pa[(0, 1)], lu[(0, 1)]));
        MINI_CHECK!(TOLERANCE.is_close(pa[(1, 0)], lu[(1, 0)]));
        MINI_CHECK!(TOLERANCE.is_close(pa[(2, 2)], lu[(2, 2)]));
        MINI_CHECK!(TOLERANCE.is_close(l[(0, 1)], 0.0) && TOLERANCE.is_close(l[(0, 2)], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(l[(1, 2)], 0.0));
    })
}

pub fn run_matrix_qr_decompose() -> TestResult {
    MINI_TEST!("QrDecompose", {
        use crate::Matrix;
        let a = Matrix::from_vec(3, 3, vec![12.0, -51.0, 4.0, 6.0, 167.0, -68.0, -4.0, 24.0, -41.0]);
        let (q, r) = a.qr_decompose();
        let qt = q.transpose();
        let qtq = qt.multiply(&q);
        let qr_prod = q.multiply(&r);

        MINI_CHECK!(TOLERANCE.is_close(qtq[(0, 0)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(qtq[(1, 1)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(qtq[(2, 2)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(qtq[(0, 1)], 0.0) && TOLERANCE.is_close(qtq[(0, 2)], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(qr_prod[(0, 0)], 12.0));
        MINI_CHECK!(TOLERANCE.is_close(qr_prod[(1, 1)], 167.0));
        MINI_CHECK!(TOLERANCE.is_close(qr_prod[(2, 2)], -41.0));
    })
}

pub fn run_matrix_cholesky() -> TestResult {
    MINI_TEST!("Cholesky", {
        use crate::Matrix;
        let a = Matrix::from_vec(3, 3, vec![4.0, 2.0, 2.0, 2.0, 5.0, 3.0, 2.0, 3.0, 6.0]);
        let l = a.cholesky();

        MINI_CHECK!(l.is_some());
        let l = l.unwrap();
        let lt = l.transpose();
        let llt = l.multiply(&lt);
        let not_spd = Matrix::from_vec(2, 2, vec![1.0, 2.0, 2.0, 1.0]);
        let l_none = not_spd.cholesky();

        MINI_CHECK!(TOLERANCE.is_close(llt[(0, 0)], 4.0) && TOLERANCE.is_close(llt[(0, 1)], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(llt[(1, 0)], 2.0) && TOLERANCE.is_close(llt[(1, 1)], 5.0));
        MINI_CHECK!(TOLERANCE.is_close(llt[(2, 2)], 6.0));
        MINI_CHECK!(l_none.is_none());
    })
}

pub fn run_matrix_eigenvalues() -> TestResult {
    MINI_TEST!("Eigenvalues", {
        use crate::Matrix;
        let a = Matrix::from_vec(3, 3, vec![3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]);
        let mut evs = a.eigenvalues();
        evs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        MINI_CHECK!(evs.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(evs[0], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(evs[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(evs[2], 3.0));
    })
}

pub fn run_matrix_svd() -> TestResult {
    MINI_TEST!("Svd", {
        use crate::Matrix;
        let a = Matrix::from_vec(3, 3, vec![1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]);
        let (_u, mut sv, _vt) = a.svd();
        sv.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

        MINI_CHECK!(sv.len() == 3);
        MINI_CHECK!(TOLERANCE.is_close(sv[0], 3.0));
        MINI_CHECK!(TOLERANCE.is_close(sv[1], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(sv[2], 1.0));
    })
}

pub fn run_matrix_norms() -> TestResult {
    MINI_TEST!("Norms", {
        use crate::Matrix;
        let a = Matrix::from_vec(2, 2, vec![1.0, -2.0, 3.0, -4.0]);
        let nf = a.norm_frobenius();
        let n1 = a.norm_1();
        let ni = a.norm_inf();

        MINI_CHECK!(TOLERANCE.is_close(nf, 30.0f64.sqrt()));
        MINI_CHECK!(TOLERANCE.is_close(n1, 6.0));
        MINI_CHECK!(TOLERANCE.is_close(ni, 7.0));
    })
}

pub fn run_matrix_rank() -> TestResult {
    MINI_TEST!("Rank", {
        use crate::Matrix;
        let a = Matrix::identity(3);
        let b = Matrix::from_vec(3, 3, vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0]);
        let c = Matrix::zeros(3, 3);

        MINI_CHECK!(a.rank() == 3);
        MINI_CHECK!(b.rank() == 2);
        MINI_CHECK!(c.rank() == 0);
    })
}

pub fn run_matrix_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Matrix;
        let mut a = Matrix::from_vec(2, 3, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        a.name = "test_matrix".to_string();
        let filename = "serialization/test_matrix.json";
        a.json_dump(filename).unwrap();
        let loaded = Matrix::json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_matrix");
        MINI_CHECK!(loaded.rows == 2 && loaded.cols == 3);
        MINI_CHECK!(TOLERANCE.is_close(loaded[(0, 0)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[(1, 2)], 6.0));
    })
}

pub fn run_matrix_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Matrix;
        let mut a = Matrix::from_vec(2, 3, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        a.name = "test_matrix_proto".to_string();
        let filename = "serialization/test_matrix.bin";
        a.pb_dump(filename);
        let loaded = Matrix::pb_load(filename);

        MINI_CHECK!(loaded.name == "test_matrix_proto");
        MINI_CHECK!(loaded.rows == 2 && loaded.cols == 3);
        MINI_CHECK!(TOLERANCE.is_close(loaded[(0, 0)], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded[(1, 2)], 6.0));
    })
}

REGISTER_MINI_TEST!("Matrix", "Constructor", crate::matrix_test::run_matrix_constructor);
REGISTER_MINI_TEST!("Matrix", "Properties", crate::matrix_test::run_matrix_properties);
REGISTER_MINI_TEST!("Matrix", "Add", crate::matrix_test::run_matrix_add);
REGISTER_MINI_TEST!("Matrix", "Subtract", crate::matrix_test::run_matrix_subtract);
REGISTER_MINI_TEST!("Matrix", "Scale", crate::matrix_test::run_matrix_scale);
REGISTER_MINI_TEST!("Matrix", "Multiply", crate::matrix_test::run_matrix_multiply);
REGISTER_MINI_TEST!("Matrix", "Transpose", crate::matrix_test::run_matrix_transpose);
REGISTER_MINI_TEST!("Matrix", "Determinant", crate::matrix_test::run_matrix_determinant);
REGISTER_MINI_TEST!("Matrix", "Inverse", crate::matrix_test::run_matrix_inverse);
REGISTER_MINI_TEST!("Matrix", "Solve", crate::matrix_test::run_matrix_solve);
REGISTER_MINI_TEST!("Matrix", "LuDecompose", crate::matrix_test::run_matrix_lu_decompose);
REGISTER_MINI_TEST!("Matrix", "QrDecompose", crate::matrix_test::run_matrix_qr_decompose);
REGISTER_MINI_TEST!("Matrix", "Cholesky", crate::matrix_test::run_matrix_cholesky);
REGISTER_MINI_TEST!("Matrix", "Eigenvalues", crate::matrix_test::run_matrix_eigenvalues);
REGISTER_MINI_TEST!("Matrix", "Svd", crate::matrix_test::run_matrix_svd);
REGISTER_MINI_TEST!("Matrix", "Norms", crate::matrix_test::run_matrix_norms);
REGISTER_MINI_TEST!("Matrix", "Rank", crate::matrix_test::run_matrix_rank);
REGISTER_MINI_TEST!("Matrix", "Json Roundtrip", crate::matrix_test::run_matrix_json_roundtrip);
REGISTER_MINI_TEST!("Matrix", "Protobuf Roundtrip", crate::matrix_test::run_matrix_protobuf_roundtrip);
