use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_primitives_circle() -> TestResult {
    MINI_TEST!("circle", {
        use crate::primitives::Primitives;

        let c = Primitives::circle(0.0, 0.0, 0.0, 1.0);

        MINI_CHECK!(c.cv_count() == 9);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == true);
    })
}

pub fn run_primitives_ellipse() -> TestResult {
    MINI_TEST!("ellipse", {
        use crate::primitives::Primitives;

        let c = Primitives::ellipse(0.0, 0.0, 0.0, 2.0, 1.0);

        MINI_CHECK!(c.cv_count() == 9);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == true);
    })
}

pub fn run_primitives_arc() -> TestResult {
    MINI_TEST!("arc", {
        use crate::primitives::Primitives;
        use crate::point::Point;

        let start = Point::new(0.0, 0.0, 0.0);
        let mid = Point::new(1.0, 1.0, 0.0);
        let end = Point::new(2.0, 0.0, 0.0);
        let c = Primitives::arc(&start, &mid, &end);

        MINI_CHECK!(c.cv_count() == 3);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == true);
    })
}

pub fn run_primitives_parabola() -> TestResult {
    MINI_TEST!("parabola", {
        use crate::primitives::Primitives;
        use crate::point::Point;

        let p0 = Point::new(-1.0, 1.0, 0.0);
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(1.0, 1.0, 0.0);
        let c = Primitives::parabola(&p0, &p1, &p2);

        MINI_CHECK!(c.cv_count() == 3);
        MINI_CHECK!(c.order() == 3);
        MINI_CHECK!(c.is_rational() == false);
    })
}

pub fn run_primitives_hyperbola() -> TestResult {
    MINI_TEST!("hyperbola", {
        use crate::primitives::Primitives;
        use crate::point::Point;

        let center = Point::new(0.0, 0.0, 0.0);
        let c = Primitives::hyperbola(&center, 1.0, 1.0, 1.0);

        MINI_CHECK!(c.cv_count() >= 4);
        MINI_CHECK!(c.order() == 4);
        MINI_CHECK!(c.is_rational() == false);
    })
}

pub fn run_primitives_spiral() -> TestResult {
    MINI_TEST!("spiral", {
        use crate::primitives::Primitives;

        let c = Primitives::spiral(1.0, 2.0, 1.0, 5.0);

        MINI_CHECK!(c.cv_count() >= 4);
        MINI_CHECK!(c.order() == 4);
        MINI_CHECK!(c.is_rational() == false);
    })
}

REGISTER_MINI_TEST!("Primitives", "circle", crate::primitives_test::run_primitives_circle);
REGISTER_MINI_TEST!("Primitives", "ellipse", crate::primitives_test::run_primitives_ellipse);
REGISTER_MINI_TEST!("Primitives", "arc", crate::primitives_test::run_primitives_arc);
REGISTER_MINI_TEST!("Primitives", "parabola", crate::primitives_test::run_primitives_parabola);
REGISTER_MINI_TEST!("Primitives", "hyperbola", crate::primitives_test::run_primitives_hyperbola);
REGISTER_MINI_TEST!("Primitives", "spiral", crate::primitives_test::run_primitives_spiral);
