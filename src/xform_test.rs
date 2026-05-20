use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;
use crate::tolerance::TOLERANCE;
use crate::tolerance::PI;

pub fn run_xform_constructor() -> TestResult {
    MINI_TEST!("Constructor", {
        use crate::Xform;
        use crate::Point;

        // Constructor (identity by default)
        let x = Xform::new();

        // Matrix access
        let m00 = x.m[0];
        let m11 = x.m[5];
        let m22 = x.m[10];
        let m33 = x.m[15];

        // Check identity
        let is_id = x.is_identity();

        // From matrix constructor
        let xfrom = Xform::from_matrix([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 10.0, 15.0, 1.0]);

        // Minimal and Full String Representation
        let xstr = x.str();
        let xrepr = x.repr();

        // Copy (duplicates everything except guid)
        let xcopy = x.duplicate();
        let xother = Xform::new();

        // Matrix multiplication (*)
        let t = Xform::translation(10.0, 0.0, 0.0);
        let s = Xform::scale_xyz(2.0, 1.0, 1.0);
        let combined = &t * &s;
        let mut p = Point::new(1.0, 0.0, 0.0);
        p.xform = combined;
        let result = p.transformed();

        // In-place multiplication (*=)
        let mut t2 = Xform::translation(10.0, 0.0, 0.0);
        t2 *= s;
        let mut p2 = Point::new(1.0, 0.0, 0.0);
        p2.xform = t2;
        let result2 = p2.transformed();

        MINI_CHECK!(x.name == "my_xform" && !x.guid().is_empty());
        MINI_CHECK!(m00 == 1.0 && m11 == 1.0 && m22 == 1.0 && m33 == 1.0);
        MINI_CHECK!(is_id == true);
        MINI_CHECK!(xfrom.m[12] == 5.0 && xfrom.m[13] == 10.0 && xfrom.m[14] == 15.0);
        MINI_CHECK!(xstr == "[1.000000, 0.000000, 0.000000, 0.000000]\n[0.000000, 1.000000, 0.000000, 0.000000]\n[0.000000, 0.000000, 1.000000, 0.000000]\n[0.000000, 0.000000, 0.000000, 1.000000]");
        MINI_CHECK!(xrepr == format!("Xform(my_xform, {})", &x.guid()[..8]));
        MINI_CHECK!(xcopy == x && xcopy.guid() != x.guid());
        MINI_CHECK!(xother == x);
        MINI_CHECK!(xfrom != x);
        MINI_CHECK!(result[0] == 12.0 && result[1] == 0.0 && result[2] == 0.0);
        MINI_CHECK!(result2[0] == 12.0 && result2[1] == 0.0 && result2[2] == 0.0);
    })
}

pub fn run_xform_translation() -> TestResult {
    MINI_TEST!("Translation", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let xf = Xform::translation(1.5, 1.0, 0.5);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(0.5, 0.0, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(2.5, 0.0, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(2.5, 2.0, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(0.5, 2.0, -0.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(0.5, 0.0, 1.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(2.5, 0.0, 1.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(2.5, 2.0, 1.5)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(0.5, 2.0, 1.5)));
    })
}

pub fn run_xform_rotation_x() -> TestResult {
    MINI_TEST!("Rotation X", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let s = f32::sqrt(2.0);
        let xf = Xform::rotation_x(PI / 4.0, false);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-1.0, 0.0, -s)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(1.0, 0.0, -s)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(1.0, s, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-1.0, s, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-1.0, -s, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(1.0, -s, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(1.0, 0.0, s)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-1.0, 0.0, s)));
    })
}

pub fn run_xform_rotation_y() -> TestResult {
    MINI_TEST!("Rotation Y", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let s = f32::sqrt(2.0);
        let xf = Xform::rotation_y(PI / 4.0, false);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-s, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(0.0, -1.0, -s)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(0.0, 1.0, -s)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-s, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(0.0, -1.0, s)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(s, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(s, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(0.0, 1.0, s)));
    })
}

pub fn run_xform_rotation_z() -> TestResult {
    MINI_TEST!("Rotation Z", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let s = f32::sqrt(2.0);
        let xf = Xform::rotation_z(PI / 4.0, false);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(0.0, -s, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(s, 0.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(0.0, s, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-s, 0.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(0.0, -s, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(s, 0.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(0.0, s, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-s, 0.0, 1.0)));
    })
}

pub fn run_xform_rotation_axis() -> TestResult {
    MINI_TEST!("Rotation Axis", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Mesh;

        let axis = Vector::new(1.0, 1.0, 1.0);
        let xf = Xform::rotation(&axis, 2.0 * PI / 4.0, false);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        let t = 1.0 / 3.0;
        let k = 2.0 / f32::sqrt(3.0);
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-1.0, -1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(-t, -t+k, -t-k)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(t-k, t+k, t)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-t-k, -t, -t+k)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-t+k, -t-k, -t)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(t+k, t, t-k)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(1.0, 1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(t, t-k, t+k)));
    })
}

pub fn run_xform_rotation_around_line() -> TestResult {
    MINI_TEST!("Rotation Around Line", {
        use crate::Xform;
        use crate::Point;
        use crate::Line;
        use crate::Mesh;

        let s = f32::sqrt(2.0);
        let line = Line::new(-1.0, -1.0, -1.0, -1.0, -1.0, 1.0);
        let xf = Xform::rotation_around_line(&line, PI / 4.0, false);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-1.0, -1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(s-1.0, s-1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(-1.0, 2.0*s-1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-s-1.0, s-1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-1.0, -1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(s-1.0, s-1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(-1.0, 2.0*s-1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-s-1.0, s-1.0, 1.0)));
    })
}

pub fn run_xform_change_basis() -> TestResult {
    MINI_TEST!("Change Basis", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Mesh;

        let o0 = Point::new(0.0, 0.0, 0.0);
        let x0 = Vector::new(1.0, 0.0, 0.0);
        let y0 = Vector::new(0.0, 1.0, 0.0);
        let z0 = Vector::new(0.0, 0.0, 1.0);
        let o1 = Point::new(0.5, -1.0, 0.5);
        let x1 = Vector::new(1.2, 0.0, 0.0);
        let y1 = Vector::new(0.3, -1.0, -0.15);
        let z1 = Vector::new(0.0, 0.0, 1.1);
        let xf = Xform::change_basis(&o0, &x0, &y0, &z0, &o1, &x1, &y1, &z1);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-1.0, 0.0, -0.45)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(1.4, 0.0, -0.45)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(2.0, -2.0, -0.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-0.4, -2.0, -0.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-1.0, 0.0, 1.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(1.4, 0.0, 1.75)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(2.0, -2.0, 1.45)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-0.4, -2.0, 1.45)));
    })
}

pub fn run_xform_plane_to_plane() -> TestResult {
    MINI_TEST!("Plane To Plane", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Plane;
        use crate::Mesh;

        let pf = Plane::new(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let pt = Plane::new(Point::new(2.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0), Vector::new(-1.0, 0.0, 0.0));
        let xf = Xform::plane_to_plane(&pf, &pt);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(1.0, 1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(1.0, -1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(3.0, -1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(3.0, 1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(1.0, 1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(1.0, -1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(3.0, -1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(3.0, 1.0, 1.0)));
    })
}

pub fn run_xform_scale_xyz() -> TestResult {
    MINI_TEST!("Scale XYZ", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let xf = Xform::scale_xyz(1.5, 1.2, 1.8);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-1.5, -1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(1.5, -1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(1.5, 1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-1.5, 1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-1.5, -1.2, 1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(1.5, -1.2, 1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(1.5, 1.2, 1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-1.5, 1.2, 1.8)));
    })
}

pub fn run_xform_scale_uniform() -> TestResult {
    MINI_TEST!("Scale Uniform", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let c = Point::new(0.0, 0.0, 0.0);
        let xf = Xform::scale_uniform(&c, 2.0);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-2.0, -2.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(2.0, -2.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(2.0, 2.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-2.0, 2.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-2.0, -2.0, 2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(2.0, -2.0, 2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(2.0, 2.0, 2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-2.0, 2.0, 2.0)));
    })
}

pub fn run_xform_scale_non_uniform() -> TestResult {
    MINI_TEST!("Scale Non Uniform", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let c = Point::new(0.0, 0.0, 0.0);
        let xf = Xform::scale_non_uniform(&c, 1.5, 1.2, 1.8);
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-1.5, -1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(1.5, -1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(1.5, 1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-1.5, 1.2, -1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-1.5, -1.2, 1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(1.5, -1.2, 1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(1.5, 1.2, 1.8)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-1.5, 1.2, 1.8)));
    })
}

pub fn run_xform_look_at_right_handed() -> TestResult {
    MINI_TEST!("Look At Right Handed", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Mesh;

        let eye = Point::new(0.0, 3.0, 0.0);
        let target = Point::new(0.0, 0.0, 0.0);
        let xf = Xform::look_at_right_handed(&eye, &target, &Vector::new(0.0, 0.0, 1.0));
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(1.0, -1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(-1.0, -1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(-1.0, -1.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(1.0, -1.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(1.0, 1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(-1.0, 1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(-1.0, 1.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(1.0, 1.0, -2.0)));
    })
}

pub fn run_xform_look_to_right_handed() -> TestResult {
    MINI_TEST!("Look To Right Handed", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Mesh;

        let eye = Point::new(0.0, 3.0, 0.0);
        let direction = Vector::new(0.0, -1.0, 0.0);
        let xf = Xform::look_to_right_handed(&eye, &direction, &Vector::new(0.0, 0.0, 1.0));
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(1.0, -1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(-1.0, -1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(-1.0, -1.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(1.0, -1.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(1.0, 1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(-1.0, 1.0, -4.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(-1.0, 1.0, -2.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(1.0, 1.0, -2.0)));
    })
}

pub fn run_xform_perspective() -> TestResult {
    MINI_TEST!("Perspective", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let view = Xform::translation(0.0, 0.0, -2.0);
        let proj = Xform::perspective(PI / 2.0, 1.0, 1.0, 3.0);
        let xf = &proj * &view;
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        let t = 1.0 / 3.0;
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-t, -t, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(t, -t, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(t, t, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-t, t, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-1.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(1.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(1.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-1.0, 1.0, 0.0)));
    })
}

pub fn run_xform_orthographic() -> TestResult {
    MINI_TEST!("Orthographic", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let view = Xform::translation(0.0, 0.0, -2.0);
        let proj = Xform::orthographic(-1.0, 1.0, -1.0, 1.0, 1.0, 3.0);
        let xf = &proj * &view;
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let result = mesh.transformed(Some(&xf));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(0).unwrap(), &Point::new(-1.0, -1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(1).unwrap(), &Point::new(1.0, -1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(2).unwrap(), &Point::new(1.0, 1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(3).unwrap(), &Point::new(-1.0, 1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(4).unwrap(), &Point::new(-1.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(5).unwrap(), &Point::new(1.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(6).unwrap(), &Point::new(1.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&result.vertex_point(7).unwrap(), &Point::new(-1.0, 1.0, 0.0)));
    })
}

pub fn run_xform_project_to_plane() -> TestResult {
    MINI_TEST!("Project To Plane", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Plane;
        use crate::Polyline;

        let plane = Plane::new(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let mv = Xform::translation(0.0, 0.0, 1.0);
        let proj = Xform::project_to_plane(&plane);
        let xf = &proj * &mv;
        let tp = |x: f32, y: f32, z: f32| -> Point { let mut p = Point::new(x, y, z); p.xform = xf.clone(); p.transformed() };
        let outline = Polyline::new(vec![
            tp(-1.0, -1.0, -1.0),
            tp(1.0, -1.0, -1.0),
            tp(1.0, 1.0, -1.0),
            tp(-1.0, 1.0, -1.0),
            tp(-1.0, -1.0, -1.0),
        ]);
        let pts = outline.get_points();
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[0], &Point::new(-1.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[1], &Point::new(1.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[2], &Point::new(1.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[3], &Point::new(-1.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[4], &Point::new(-1.0, -1.0, 0.0)));
    })
}

pub fn run_xform_project_to_plane_by_axis() -> TestResult {
    MINI_TEST!("Project To Plane By Axis", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Plane;
        use crate::Polyline;

        let plane = Plane::new(Point::new(0.0, 0.0, 0.0), Vector::new(1.0, 0.0, 0.0), Vector::new(0.0, 1.0, 0.0));
        let direction = Vector::new(1.0, 0.0, 1.0);
        let mv = Xform::translation(0.0, 0.0, 1.0);
        let proj = Xform::project_to_plane_by_axis(&plane, &direction);
        let xf = &proj * &mv;
        let tp = |x: f32, y: f32, z: f32| -> Point { let mut p = Point::new(x, y, z); p.xform = xf.clone(); p.transformed() };
        let outline = Polyline::new(vec![
            tp(-1.0, -1.0, 1.0),
            tp(1.0, -1.0, -1.0),
            tp(1.0, 1.0, -1.0),
            tp(-1.0, 1.0, 1.0),
            tp(-1.0, -1.0, 1.0),
        ]);
        let pts = outline.get_points();
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[0], &Point::new(-3.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[1], &Point::new(1.0, -1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[2], &Point::new(1.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[3], &Point::new(-3.0, 1.0, 0.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[4], &Point::new(-3.0, -1.0, 0.0)));
    })
}

pub fn run_xform_inverse() -> TestResult {
    MINI_TEST!("Inverse", {
        use crate::Xform;
        use crate::Point;
        use crate::Mesh;

        let t = Xform::translation(1.0, 0.5, 0.5);
        let s = Xform::scale_xyz(1.5, 1.2, 1.3);
        let composite = &t * &s;
        let inv = composite.inverse().unwrap();
        let mesh = Mesh::create_box(2.0, 2.0, 2.0);
        let forward = mesh.transformed(Some(&composite));
        let roundtrip = forward.transformed(Some(&inv));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(0).unwrap(), &Point::new(-1.0, -1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(1).unwrap(), &Point::new(1.0, -1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(2).unwrap(), &Point::new(1.0, 1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(3).unwrap(), &Point::new(-1.0, 1.0, -1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(4).unwrap(), &Point::new(-1.0, -1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(5).unwrap(), &Point::new(1.0, -1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(6).unwrap(), &Point::new(1.0, 1.0, 1.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&roundtrip.vertex_point(7).unwrap(), &Point::new(-1.0, 1.0, 1.0)));
    })
}

pub fn run_xform_transform_geometry() -> TestResult {
    MINI_TEST!("Transform Geometry", {
        use crate::Xform;
        use crate::Point;
        use crate::Vector;
        use crate::Line;
        use crate::Plane;
        use crate::Polyline;

        // Simple translation by (10, 20, 30)
        let t = Xform::translation(10.0, 20.0, 30.0);

        // Transform Point: (1,2,3) -> (11,22,33)
        let mut pt = Point::new(1.0, 2.0, 3.0);
        pt.xform = t.clone();
        let pt_transformed = pt.transformed();

        // Transform Vector: translation should NOT affect vectors
        let mut v = Vector::new(1.0, 0.0, 0.0);
        v.xform = t.clone();
        let v_transformed = v.transformed();

        // Transform Line: (0,0,0)-(1,0,0) -> (10,20,30)-(11,20,30)
        let mut ln = Line::new(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
        ln.xform = t.clone();
        let ln_transformed = ln.transformed();

        // Transform Plane: origin (0,0,0) -> (10,20,30)
        let mut pl = Plane::new(
            Point::new(0.0, 0.0, 0.0),
            Vector::new(1.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0),
        );
        pl.xform = t.clone();
        let pl_transformed = pl.transformed();

        // Transform Polyline: 3 points translated
        let mut poly = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
        ]);
        poly.xform = t.clone();
        let poly_transformed = poly.transformed();
        let pts = poly_transformed.get_points();

        MINI_CHECK!(TOLERANCE.is_point_close(&pt_transformed, &Point::new(11.0, 22.0, 33.0)));
        MINI_CHECK!(v_transformed[0] == 1.0 && v_transformed[1] == 0.0 && v_transformed[2] == 0.0);
        MINI_CHECK!(ln_transformed[0] == 10.0 && ln_transformed[1] == 20.0 && ln_transformed[2] == 30.0);
        MINI_CHECK!(ln_transformed[3] == 11.0 && ln_transformed[4] == 20.0 && ln_transformed[5] == 30.0);
        MINI_CHECK!(TOLERANCE.is_point_close(&pl_transformed.origin(), &Point::new(10.0, 20.0, 30.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[0], &Point::new(10.0, 20.0, 30.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[1], &Point::new(11.0, 20.0, 30.0)));
        MINI_CHECK!(TOLERANCE.is_point_close(&pts[2], &Point::new(11.0, 21.0, 30.0)));
    })
}

pub fn run_xform_json_roundtrip() -> TestResult {
    MINI_TEST!("Json Roundtrip", {
        use crate::Xform;

        let mut xform = Xform::translation(1.0, 2.0, 3.0);
        xform.name = "test_xform".to_string();

        let filename = "serialization/test_xform.json";
        xform.file_json_dump(filename).unwrap();
        let loaded = Xform::file_json_load(filename).unwrap();

        MINI_CHECK!(loaded.name == "test_xform");
        MINI_CHECK!(loaded.guid() == xform.guid());
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[0], 1.0) && TOLERANCE.is_close(loaded.m[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[2], 0.0) && TOLERANCE.is_close(loaded.m[3], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[4], 0.0) && TOLERANCE.is_close(loaded.m[5], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[6], 0.0) && TOLERANCE.is_close(loaded.m[7], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[8], 0.0) && TOLERANCE.is_close(loaded.m[9], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[10], 1.0) && TOLERANCE.is_close(loaded.m[11], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[12], 1.0) && TOLERANCE.is_close(loaded.m[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[14], 3.0) && TOLERANCE.is_close(loaded.m[15], 1.0));
    })
}

pub fn run_xform_protobuf_roundtrip() -> TestResult {
    MINI_TEST!("Protobuf Roundtrip", {
        use crate::Xform;

        let mut xform = Xform::translation(1.0, 2.0, 3.0);
        xform.name = "test_xform_proto".to_string();

        let filename = "serialization/test_xform.bin";
        xform.pb_dump(filename);
        let loaded = Xform::pb_load(filename);

        MINI_CHECK!(loaded.name == "test_xform_proto");
        MINI_CHECK!(loaded.guid() == xform.guid());
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[0], 1.0) && TOLERANCE.is_close(loaded.m[1], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[2], 0.0) && TOLERANCE.is_close(loaded.m[3], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[4], 0.0) && TOLERANCE.is_close(loaded.m[5], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[6], 0.0) && TOLERANCE.is_close(loaded.m[7], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[8], 0.0) && TOLERANCE.is_close(loaded.m[9], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[10], 1.0) && TOLERANCE.is_close(loaded.m[11], 0.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[12], 1.0) && TOLERANCE.is_close(loaded.m[13], 2.0));
        MINI_CHECK!(TOLERANCE.is_close(loaded.m[14], 3.0) && TOLERANCE.is_close(loaded.m[15], 1.0));
    })
}

REGISTER_MINI_TEST!("Xform", "Constructor", crate::xform_test::run_xform_constructor);
REGISTER_MINI_TEST!("Xform", "Translation", crate::xform_test::run_xform_translation);
REGISTER_MINI_TEST!("Xform", "Rotation X", crate::xform_test::run_xform_rotation_x);
REGISTER_MINI_TEST!("Xform", "Rotation Y", crate::xform_test::run_xform_rotation_y);
REGISTER_MINI_TEST!("Xform", "Rotation Z", crate::xform_test::run_xform_rotation_z);
REGISTER_MINI_TEST!("Xform", "Rotation Axis", crate::xform_test::run_xform_rotation_axis);
REGISTER_MINI_TEST!("Xform", "Rotation Around Line", crate::xform_test::run_xform_rotation_around_line);
REGISTER_MINI_TEST!("Xform", "Change Basis", crate::xform_test::run_xform_change_basis);
REGISTER_MINI_TEST!("Xform", "Plane To Plane", crate::xform_test::run_xform_plane_to_plane);
REGISTER_MINI_TEST!("Xform", "Scale XYZ", crate::xform_test::run_xform_scale_xyz);
REGISTER_MINI_TEST!("Xform", "Scale Uniform", crate::xform_test::run_xform_scale_uniform);
REGISTER_MINI_TEST!("Xform", "Scale Non Uniform", crate::xform_test::run_xform_scale_non_uniform);
REGISTER_MINI_TEST!("Xform", "Look At Right Handed", crate::xform_test::run_xform_look_at_right_handed);
REGISTER_MINI_TEST!("Xform", "Look To Right Handed", crate::xform_test::run_xform_look_to_right_handed);
REGISTER_MINI_TEST!("Xform", "Perspective", crate::xform_test::run_xform_perspective);
REGISTER_MINI_TEST!("Xform", "Orthographic", crate::xform_test::run_xform_orthographic);
REGISTER_MINI_TEST!("Xform", "Project To Plane", crate::xform_test::run_xform_project_to_plane);
REGISTER_MINI_TEST!("Xform", "Project To Plane By Axis", crate::xform_test::run_xform_project_to_plane_by_axis);
REGISTER_MINI_TEST!("Xform", "Inverse", crate::xform_test::run_xform_inverse);
REGISTER_MINI_TEST!("Xform", "Transform Geometry", crate::xform_test::run_xform_transform_geometry);
REGISTER_MINI_TEST!("Xform", "Json Roundtrip", crate::xform_test::run_xform_json_roundtrip);
REGISTER_MINI_TEST!("Xform", "Protobuf Roundtrip", crate::xform_test::run_xform_protobuf_roundtrip);

pub fn run_xform_from_change_of_basis() -> TestResult {
    MINI_TEST!("From Change Of Basis", {
        use crate::{Point, Polyline, Xform};
        let rect0 = Polyline::new(vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(2.0, 3.0, 0.0),
            Point::new(0.0, 3.0, 0.0),
        ]);
        let rect1 = Polyline::new(vec![Point::new(0.0, 0.0, 4.0)]);
        let xf = Xform::from_change_of_basis(&rect0, &rect1);
        MINI_CHECK!(TOLERANCE.is_close(xf.m[12], 1.0));
        MINI_CHECK!(TOLERANCE.is_close(xf.m[13], 1.5));
        MINI_CHECK!(TOLERANCE.is_close(xf.m[14], 2.0));
    })
}
REGISTER_MINI_TEST!("Xform", "From Change Of Basis", crate::xform_test::run_xform_from_change_of_basis);
