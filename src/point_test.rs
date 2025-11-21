use crate::{MINI_CHECK, MINI_TEST, REGISTER_MINI_TEST};
use crate::mini_test::TestResult;

pub fn run_point_constructor() -> TestResult {
    MINI_TEST!("constructor", |checks: &mut Vec<_>| {
        use crate::Point;
        use crate::Vector;
        use crate::Color;

        // Constructor
        let mut p = Point::new(1.0, 2.0, 3.0);

        // Setters
        p[0] = 10.0;
        p[1] = 20.0;
        p[2] = 30.0;

        // Getters
        let x = p[0];
        let y = p[1];
        let z = p[2];

        // String representation
        let pstr = p.str();
        let prepr = p.repr();

        // Copy (duplicate everything but guid)
        let pcopy = p.deepcopy();
        let pother = Point::new(1.0, 2.0, 3.0);

        // No-copy operators
        let mut pmult = Point::new(p[0], p[1], p[2]);
        pmult *= 2.0;
        let mut pdiv = Point::new(p[0], p[1], p[2]);
        pdiv /= 2.0;
        let mut padd = Point::new(p[0], p[1], p[2]);
        padd += Vector::new(1.0, 1.0, 1.0);
        let mut psub = Point::new(p[0], p[1], p[2]);
        psub -= Vector::new(1.0, 1.0, 1.0);

        // Copy operators
        let result_mul = p.clone() * 2.0;
        let result_div = p.clone() / 2.0;
        let result_add = p.clone() + Vector::new(1.0, 1.0, 1.0);
        let diff_point = p.clone() - Vector::new(1.0, 1.0, 1.0);

        MINI_CHECK!(
            checks,
            p.name == "my_point"
                && p[0] == 10.0
                && p[1] == 20.0
                && p[2] == 30.0
                && p.width == 1.0
                && p.pointcolor == Color::blue()
                && !p.guid.is_empty()
        );

        MINI_CHECK!(checks, x == 10.0 && y == 20.0 && z == 30.0);

        MINI_CHECK!(checks, pstr == "10.000000, 20.000000, 30.000000");
        MINI_CHECK!(
            checks,
            prepr
                == "Point(my_point, 10.000000, 20.000000, 30.000000, Color(0, 0, 255, 255), 1.000000)"
        );
        MINI_CHECK!(checks, p == pcopy && pcopy.guid != p.guid);
        MINI_CHECK!(checks, pother != p);

        MINI_CHECK!(checks, pmult[0] == 20.0 && pmult[1] == 40.0 && pmult[2] == 60.0);
        MINI_CHECK!(checks, pdiv[0] == 5.0 && pdiv[1] == 10.0 && pdiv[2] == 15.0);
        MINI_CHECK!(checks, padd[0] == 11.0 && padd[1] == 21.0 && padd[2] == 31.0);
        MINI_CHECK!(checks, psub[0] == 9.0 && psub[1] == 19.0 && psub[2] == 29.0);

        MINI_CHECK!(
            checks,
            result_mul[0] == 20.0 && result_mul[1] == 40.0 && result_mul[2] == 60.0
        );
        MINI_CHECK!(
            checks,
            result_div[0] == 5.0 && result_div[1] == 10.0 && result_div[2] == 15.0
        );
        MINI_CHECK!(
            checks,
            result_add[0] == 11.0 && result_add[1] == 21.0 && result_add[2] == 31.0
        );
        MINI_CHECK!(
            checks,
            diff_point[0] == 9.0 && diff_point[1] == 19.0 && diff_point[2] == 29.0
        );

        Ok(())
    })
}

// Register tests with the shared registry for run_all("rust")
REGISTER_MINI_TEST!("Point", "constructor", crate::point_test::run_point_constructor);