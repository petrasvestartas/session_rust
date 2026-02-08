use crate::{Plane, Point, Vector, Xform};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "BoundingBox")]
pub struct BoundingBox {
    pub center: Point,
    pub x_axis: Vector,
    pub y_axis: Vector,
    pub z_axis: Vector,
    pub half_size: Vector,
    pub guid: String,
    pub name: String,
    #[serde(default = "Xform::identity")]
    pub xform: Xform,
}

impl BoundingBox {
    pub fn new(
        center: Point,
        x_axis: Vector,
        y_axis: Vector,
        z_axis: Vector,
        half_size: Vector,
    ) -> Self {
        BoundingBox {
            center,
            x_axis,
            y_axis,
            z_axis,
            half_size,
            guid: Uuid::new_v4().to_string(),
            name: "my_boundingbox".to_string(),
            xform: Xform::identity(),
        }
    }

    pub fn from_plane(plane: &Plane, dx: f64, dy: f64, dz: f64) -> Self {
        BoundingBox {
            center: plane.origin(),
            x_axis: plane.x_axis(),
            y_axis: plane.y_axis(),
            z_axis: plane.z_axis(),
            half_size: Vector::new(dx * 0.5, dy * 0.5, dz * 0.5),
            guid: Uuid::new_v4().to_string(),
            name: String::new(),
            xform: Xform::identity(),
        }
    }

    pub fn from_point(point: Point, inflate: f64) -> Self {
        BoundingBox {
            center: point,
            x_axis: Vector::new(1.0, 0.0, 0.0),
            y_axis: Vector::new(0.0, 1.0, 0.0),
            z_axis: Vector::new(0.0, 0.0, 1.0),
            half_size: Vector::new(inflate, inflate, inflate),
            guid: Uuid::new_v4().to_string(),
            xform: Xform::identity(),
            name: String::new(),
        }
    }

    pub fn from_points(points: &[Point], inflate: f64) -> Self {
        if points.is_empty() {
            return BoundingBox::default();
        }

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;

        for pt in points {
            min_x = min_x.min(pt[0]);
            min_y = min_y.min(pt[1]);
            min_z = min_z.min(pt[2]);
            max_x = max_x.max(pt[0]);
            max_y = max_y.max(pt[1]);
            max_z = max_z.max(pt[2]);
        }

        let center = Point::new(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        );
        let half_size = Vector::new(
            (max_x - min_x) * 0.5 + inflate,
            (max_y - min_y) * 0.5 + inflate,
            (max_z - min_z) * 0.5 + inflate,
        );

        BoundingBox {
            center,
            x_axis: Vector::new(1.0, 0.0, 0.0),
            y_axis: Vector::new(0.0, 1.0, 0.0),
            z_axis: Vector::new(0.0, 0.0, 1.0),
            half_size,
            guid: Uuid::new_v4().to_string(),
            name: String::new(),
            xform: Xform::identity(),
        }
    }

    pub fn from_line(line: &crate::line::Line, inflate: f64) -> Self {
        let points = vec![line.start(), line.end()];
        Self::from_points(&points, inflate)
    }

    pub fn from_polyline(polyline: &crate::polyline::Polyline, inflate: f64) -> Self {
        Self::from_points(&polyline.get_points(), inflate)
    }

    pub fn from_nurbscurve(curve: &crate::nurbscurve::NurbsCurve, inflate: f64, tight: bool) -> Self {
        if !curve.is_valid() || curve.cv_count() == 0 {
            return BoundingBox::default();
        }

        if !tight {
            let points: Vec<Point> = (0..curve.cv_count())
                .filter_map(|i| curve.get_cv(i))
                .collect();
            return Self::from_points(&points, inflate);
        }

        let (t0, t1) = curve.domain();
        let mut extrema_points = vec![curve.point_at(t0), curve.point_at(t1)];

        let spans = curve.get_span_vector();
        for t in spans {
            if t > t0 && t < t1 {
                extrema_points.push(curve.point_at(t));
            }
        }

        const NUM_SAMPLES: usize = 20;
        let dt = (t1 - t0) / NUM_SAMPLES as f64;

        for axis in 0..3 {
            for i in 0..NUM_SAMPLES {
                let t_start = t0 + i as f64 * dt;
                let t_end = t_start + dt;

                let deriv_start = curve.evaluate(t_start, 1);
                let deriv_end = curve.evaluate(t_end, 1);
                if deriv_start.len() < 2 || deriv_end.len() < 2 {
                    continue;
                }

                let mut d_start = deriv_start[1][axis];
                let d_end = deriv_end[1][axis];

                if d_start * d_end < 0.0 {
                    let mut t_lo = t_start;
                    let mut t_hi = t_end;
                    let mut t_root = (t_lo + t_hi) * 0.5;

                    for _ in 0..20 {
                        let deriv = curve.evaluate(t_root, 2);
                        if deriv.len() < 3 {
                            break;
                        }

                        let f = deriv[1][axis];
                        let fp = deriv[2][axis];

                        if f.abs() < 1e-12 {
                            break;
                        }

                        if fp.abs() > 1e-14 {
                            let t_new = t_root - f / fp;
                            if t_new >= t_lo && t_new <= t_hi {
                                t_root = t_new;
                            } else {
                                if f * d_start < 0.0 {
                                    t_hi = t_root;
                                } else {
                                    t_lo = t_root;
                                }
                                t_root = (t_lo + t_hi) * 0.5;
                            }
                        } else {
                            t_root = (t_lo + t_hi) * 0.5;
                        }

                        let deriv_check = curve.evaluate(t_root, 1);
                        if deriv_check.len() >= 2 {
                            let f_check = deriv_check[1][axis];
                            if f_check * d_start < 0.0 {
                                t_hi = t_root;
                            } else {
                                t_lo = t_root;
                                d_start = f_check;
                            }
                        }
                    }

                    extrema_points.push(curve.point_at(t_root));
                }
            }
        }

        Self::from_points(&extrema_points, inflate)
    }

    pub fn from_nurbscurve_with_plane(
        curve: &crate::nurbscurve::NurbsCurve,
        plane: &Plane,
        inflate: f64,
        tight: bool,
    ) -> Self {
        if !curve.is_valid() || curve.cv_count() == 0 {
            return BoundingBox::default();
        }

        if !tight {
            let points: Vec<Point> = (0..curve.cv_count())
                .filter_map(|i| curve.get_cv(i))
                .collect();
            return Self::from_points_with_plane(&points, plane, inflate);
        }

        let (t0, t1) = curve.domain();
        let mut extrema_points = vec![curve.point_at(t0), curve.point_at(t1)];

        let spans = curve.get_span_vector();
        for t in spans {
            if t > t0 && t < t1 {
                extrema_points.push(curve.point_at(t));
            }
        }

        let axes = [plane.x_axis(), plane.y_axis(), plane.z_axis()];
        const NUM_SAMPLES: usize = 20;
        let dt = (t1 - t0) / NUM_SAMPLES as f64;

        for axis in &axes {
            for i in 0..NUM_SAMPLES {
                let t_start = t0 + i as f64 * dt;
                let t_end = t_start + dt;

                let deriv_start = curve.evaluate(t_start, 1);
                let deriv_end = curve.evaluate(t_end, 1);
                if deriv_start.len() < 2 || deriv_end.len() < 2 {
                    continue;
                }

                let mut d_start = deriv_start[1].dot(axis);
                let d_end = deriv_end[1].dot(axis);

                if d_start * d_end < 0.0 {
                    let mut t_lo = t_start;
                    let mut t_hi = t_end;
                    let mut t_root = (t_lo + t_hi) * 0.5;

                    for _ in 0..20 {
                        let deriv = curve.evaluate(t_root, 2);
                        if deriv.len() < 3 {
                            break;
                        }

                        let f = deriv[1].dot(axis);
                        let fp = deriv[2].dot(axis);

                        if f.abs() < 1e-12 {
                            break;
                        }

                        if fp.abs() > 1e-14 {
                            let t_new = t_root - f / fp;
                            if t_new >= t_lo && t_new <= t_hi {
                                t_root = t_new;
                            } else {
                                if f * d_start < 0.0 {
                                    t_hi = t_root;
                                } else {
                                    t_lo = t_root;
                                }
                                t_root = (t_lo + t_hi) * 0.5;
                            }
                        } else {
                            t_root = (t_lo + t_hi) * 0.5;
                        }

                        let deriv_check = curve.evaluate(t_root, 1);
                        if deriv_check.len() >= 2 {
                            let f_check = deriv_check[1].dot(axis);
                            if f_check * d_start < 0.0 {
                                t_hi = t_root;
                            } else {
                                t_lo = t_root;
                                d_start = f_check;
                            }
                        }
                    }

                    extrema_points.push(curve.point_at(t_root));
                }
            }
        }

        Self::from_points_with_plane(&extrema_points, plane, inflate)
    }

    pub fn from_points_with_plane(points: &[Point], plane: &Plane, inflate: f64) -> Self {
        if points.is_empty() {
            return BoundingBox::default();
        }

        let origin = plane.origin();
        let x_axis = plane.x_axis();
        let y_axis = plane.y_axis();
        let z_axis = plane.z_axis();
        let plane_to_xy = Xform::plane_to_xy(&origin, &x_axis, &y_axis, &z_axis);

        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut min_z = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;
        let mut max_z = f64::MIN;

        for pt in points {
            let local_pt = plane_to_xy.transformed_point(pt);
            min_x = min_x.min(local_pt[0]);
            min_y = min_y.min(local_pt[1]);
            min_z = min_z.min(local_pt[2]);
            max_x = max_x.max(local_pt[0]);
            max_y = max_y.max(local_pt[1]);
            max_z = max_z.max(local_pt[2]);
        }

        let local_center = Point::new(
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        );
        let half_size = Vector::new(
            (max_x - min_x) * 0.5 + inflate,
            (max_y - min_y) * 0.5 + inflate,
            (max_z - min_z) * 0.5 + inflate,
        );

        let xy_to_plane = Xform::xy_to_plane(&origin, &x_axis, &y_axis, &z_axis);
        let world_center = xy_to_plane.transformed_point(&local_center);

        BoundingBox {
            center: world_center,
            x_axis,
            y_axis,
            z_axis,
            half_size,
            guid: Uuid::new_v4().to_string(),
            name: String::new(),
            xform: Xform::identity(),
        }
    }

    pub fn point_at(&self, x: f64, y: f64, z: f64) -> Point {
        Point::new(
            self.center[0] + x * self.x_axis[0] + y * self.y_axis[0] + z * self.z_axis[0],
            self.center[1] + x * self.x_axis[1] + y * self.y_axis[1] + z * self.z_axis[1],
            self.center[2] + x * self.x_axis[2] + y * self.y_axis[2] + z * self.z_axis[2],
        )
    }

    pub fn min_point(&self) -> Point {
        Point::new(
            self.center[0] - self.half_size[0],
            self.center[1] - self.half_size[1],
            self.center[2] - self.half_size[2],
        )
    }

    pub fn max_point(&self) -> Point {
        Point::new(
            self.center[0] + self.half_size[0],
            self.center[1] + self.half_size[1],
            self.center[2] + self.half_size[2],
        )
    }

    pub fn corners(&self) -> [Point; 8] {
        [
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(
                -self.half_size[0],
                -self.half_size[1],
                -self.half_size[2],
            ),
            self.point_at(self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], self.half_size[2]),
        ]
    }

    pub fn two_rectangles(&self) -> [Point; 10] {
        [
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(
                -self.half_size[0],
                -self.half_size[1],
                -self.half_size[2],
            ),
            self.point_at(self.half_size[0], -self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], -self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], self.half_size[1], self.half_size[2]),
            self.point_at(-self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], -self.half_size[1], self.half_size[2]),
            self.point_at(self.half_size[0], self.half_size[1], self.half_size[2]),
        ]
    }

    pub fn inflate(&mut self, amount: f64) {
        self.half_size = Vector::new(
            self.half_size[0] + amount,
            self.half_size[1] + amount,
            self.half_size[2] + amount,
        );
    }

    fn separating_plane_exists(
        relative_position: &Vector,
        axis: &Vector,
        box1: &BoundingBox,
        box2: &BoundingBox,
    ) -> bool {
        let dot_rp = relative_position.dot(axis).abs();

        let v1 = box1.x_axis.clone() * box1.half_size[0];
        let v2 = box1.y_axis.clone() * box1.half_size[1];
        let v3 = box1.z_axis.clone() * box1.half_size[2];
        let proj1 = v1.dot(axis).abs() + v2.dot(axis).abs() + v3.dot(axis).abs();

        let v4 = box2.x_axis.clone() * box2.half_size[0];
        let v5 = box2.y_axis.clone() * box2.half_size[1];
        let v6 = box2.z_axis.clone() * box2.half_size[2];
        let proj2 = v4.dot(axis).abs() + v5.dot(axis).abs() + v6.dot(axis).abs();

        dot_rp > (proj1 + proj2)
    }

    pub fn collides_with(&self, other: &BoundingBox) -> bool {
        let center_pt = Point::new(self.center[0], self.center[1], self.center[2]);
        let other_center_pt = Point::new(other.center[0], other.center[1], other.center[2]);
        let relative_position = Vector::from_points(&center_pt, &other_center_pt);

        !(Self::separating_plane_exists(&relative_position, &self.x_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &self.y_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &self.z_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &other.x_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &other.y_axis, self, other)
            || Self::separating_plane_exists(&relative_position, &other.z_axis, self, other)
            || Self::separating_plane_exists(
                &relative_position,
                &self.x_axis.cross(&other.x_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.x_axis.cross(&other.y_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.x_axis.cross(&other.z_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.y_axis.cross(&other.x_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.y_axis.cross(&other.y_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.y_axis.cross(&other.z_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.z_axis.cross(&other.x_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.z_axis.cross(&other.y_axis),
                self,
                other,
            )
            || Self::separating_plane_exists(
                &relative_position,
                &self.z_axis.cross(&other.z_axis),
                self,
                other,
            ))
    }

    pub fn transform(&mut self) {
        let xform = self.xform.clone();
        xform.transform_point(&mut self.center);
        xform.transform_vector(&mut self.x_axis);
        xform.transform_vector(&mut self.y_axis);
        xform.transform_vector(&mut self.z_axis);
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    pub fn jsondump(&self) -> Result<String, std::boxed::Box<dyn std::error::Error>> {
        let data = serde_json::json!({
            "type": "BoundingBox",
            "center": serde_json::from_str::<serde_json::Value>(&self.center.jsondump()?)?,
            "x_axis": serde_json::from_str::<serde_json::Value>(&self.x_axis.jsondump()?)?,
            "y_axis": serde_json::from_str::<serde_json::Value>(&self.y_axis.jsondump()?)?,
            "z_axis": serde_json::from_str::<serde_json::Value>(&self.z_axis.jsondump()?)?,
            "half_size": serde_json::from_str::<serde_json::Value>(&self.half_size.jsondump()?)?,
            "guid": self.guid,
            "name": self.name,
        });
        Ok(serde_json::to_string(&data)?)
    }

    pub fn jsonload(json_data: &str) -> Result<Self, std::boxed::Box<dyn std::error::Error>> {
        let data: serde_json::Value = serde_json::from_str(json_data)?;
        let mut bbox = BoundingBox::new(
            Point::jsonload(&data["center"].to_string())?,
            Vector::jsonload(&data["x_axis"].to_string())?,
            Vector::jsonload(&data["y_axis"].to_string())?,
            Vector::jsonload(&data["z_axis"].to_string())?,
            Vector::jsonload(&data["half_size"].to_string())?,
        );
        bbox.guid = data["guid"].as_str().unwrap().to_string();
        bbox.name = data["name"].as_str().unwrap().to_string();
        Ok(bbox)
    }

    pub fn to_json(&self, filepath: &str) -> Result<(), std::boxed::Box<dyn std::error::Error>> {
        let json_string = self.jsondump()?;
        let value: serde_json::Value = serde_json::from_str(&json_string)?;
        let pretty = serde_json::to_string_pretty(&value)?;
        std::fs::write(filepath, pretty)?;
        Ok(())
    }

    pub fn from_json(filepath: &str) -> Result<Self, std::boxed::Box<dyn std::error::Error>> {
        let json_string = std::fs::read_to_string(filepath)?;
        Self::jsonload(&json_string)
    }

    pub fn json_dumps(&self) -> String {
        self.jsondump().unwrap_or_default()
    }

    pub fn json_loads(s: &str) -> Self {
        Self::jsonload(s).unwrap_or_default()
    }

    pub fn json_dump(&self, filepath: &str) {
        let _ = self.to_json(filepath);
    }

    pub fn json_load(filepath: &str) -> Self {
        Self::from_json(filepath).unwrap_or_default()
    }

    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;
        let proto = crate::proto::BoundingBox {
            center: Some(crate::proto::Point::decode(self.center.pb_dumps().as_slice()).unwrap()),
            x_axis: Some(crate::proto::Vector::decode(self.x_axis.pb_dumps().as_slice()).unwrap()),
            y_axis: Some(crate::proto::Vector::decode(self.y_axis.pb_dumps().as_slice()).unwrap()),
            z_axis: Some(crate::proto::Vector::decode(self.z_axis.pb_dumps().as_slice()).unwrap()),
            half_size: Some(crate::proto::Vector::decode(self.half_size.pb_dumps().as_slice()).unwrap()),
            guid: self.guid.clone(),
            name: self.name.clone(),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    pub fn pb_loads(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;
        let proto = crate::proto::BoundingBox::decode(data)?;
        let center = if let Some(p) = &proto.center {
            crate::point::Point::pb_loads(&p.encode_to_vec())?
        } else {
            crate::point::Point::new(0.0, 0.0, 0.0)
        };
        let x_axis = if let Some(v) = &proto.x_axis {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(1.0, 0.0, 0.0)
        };
        let y_axis = if let Some(v) = &proto.y_axis {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(0.0, 1.0, 0.0)
        };
        let z_axis = if let Some(v) = &proto.z_axis {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(0.0, 0.0, 1.0)
        };
        let half_size = if let Some(v) = &proto.half_size {
            crate::vector::Vector::pb_loads(&v.encode_to_vec())?
        } else {
            crate::vector::Vector::new(0.5, 0.5, 0.5)
        };
        let mut bbox = BoundingBox::new(center, x_axis, y_axis, z_axis, half_size);
        bbox.guid = proto.guid;
        bbox.name = proto.name;
        if let Some(xform) = proto.xform {
            bbox.xform.guid = xform.guid;
            bbox.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 { bbox.xform.m[i] = *val; }
            }
        }
        Ok(bbox)
    }

    pub fn pb_dump(&self, filepath: &str) {
        std::fs::write(filepath, self.pb_dumps()).expect("Failed to write protobuf file");
    }

    pub fn pb_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::pb_loads(&data).expect("Failed to parse protobuf")
    }
}

impl Default for BoundingBox {
    fn default() -> Self {
        BoundingBox {
            center: Point::new(0.0, 0.0, 0.0),
            x_axis: Vector::new(1.0, 0.0, 0.0),
            y_axis: Vector::new(0.0, 1.0, 0.0),
            z_axis: Vector::new(0.0, 0.0, 1.0),
            half_size: Vector::new(0.5, 0.5, 0.5),
            guid: Uuid::new_v4().to_string(),
            name: String::new(),
            xform: Xform::identity(),
        }
    }
}

#[cfg(test)]
#[path = "boundingbox_test.rs"]
mod boundingbox_test;
