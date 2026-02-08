use crate::nurbssurface::NurbsSurface;
use crate::nurbscurve::NurbsCurve;
use crate::xform::Xform;
use crate::color::Color;
use crate::point::Point;
use crate::vector::Vector;
use crate::mesh::Mesh;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct TrimmedSurface {
    pub guid: String,
    pub name: String,
    pub width: f64,
    pub surfacecolor: Color,
    pub xform: Xform,
    pub m_surface: NurbsSurface,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub m_outer_loop: Option<NurbsCurve>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    #[serde(default)]
    pub m_inner_loops: Vec<NurbsCurve>,
}

impl TrimmedSurface {
    pub fn new() -> Self {
        TrimmedSurface {
            guid: uuid::Uuid::new_v4().to_string(),
            name: "my_trimmedsurface".to_string(),
            width: 1.0,
            surfacecolor: Color::black(),
            xform: Xform::identity(),
            m_surface: NurbsSurface::new(),
            m_outer_loop: None,
            m_inner_loops: Vec::new(),
        }
    }

    pub fn create(surface: &NurbsSurface, outer_loop: &NurbsCurve) -> Self {
        let mut ts = Self::new();
        ts.m_surface = surface.duplicate();
        ts.m_outer_loop = Some(outer_loop.duplicate());
        ts
    }

    pub fn create_planar(boundary: &NurbsCurve) -> Option<Self> {
        let srf = NurbsSurface::create_planar(&[boundary.clone()])?;
        let mut ts = Self::new();
        ts.m_outer_loop = srf.m_outer_loop.clone();
        let mut srf_clean = srf;
        srf_clean.m_outer_loop = None;
        srf_clean.m_inner_loops.clear();
        ts.m_surface = srf_clean;
        Some(ts)
    }

    pub fn surface(&self) -> &NurbsSurface { &self.m_surface }
    pub fn get_outer_loop(&self) -> Option<&NurbsCurve> { self.m_outer_loop.as_ref() }

    pub fn set_outer_loop(&mut self, loop_crv: NurbsCurve) {
        self.m_outer_loop = Some(loop_crv);
    }

    pub fn is_trimmed(&self) -> bool {
        self.m_outer_loop.as_ref().map_or(false, |c| c.is_valid())
    }

    pub fn is_valid(&self) -> bool { self.m_surface.is_valid() }

    pub fn add_inner_loop(&mut self, loop_2d: NurbsCurve) {
        self.m_inner_loops.push(loop_2d);
    }

    pub fn add_hole(&mut self, curve_3d: &NurbsCurve) {
        let mut temp = self.m_surface.clone();
        temp.add_hole(curve_3d);
        if temp.inner_loop_count() > 0 {
            self.m_inner_loops.push(temp.get_inner_loop(temp.inner_loop_count() - 1).unwrap().clone());
        }
    }

    pub fn add_holes(&mut self, curves_3d: &[NurbsCurve]) {
        for crv in curves_3d {
            self.add_hole(crv);
        }
    }

    pub fn get_inner_loop(&self, index: usize) -> Option<&NurbsCurve> {
        self.m_inner_loops.get(index)
    }

    pub fn inner_loop_count(&self) -> usize { self.m_inner_loops.len() }

    pub fn clear_inner_loops(&mut self) { self.m_inner_loops.clear(); }

    pub fn point_at(&self, u: f64, v: f64) -> Option<Point> { self.m_surface.point_at(u, v) }
    pub fn normal_at(&self, u: f64, v: f64) -> Vector { self.m_surface.normal_at(u, v) }

    pub fn mesh(&self) -> Mesh {
        let mut temp = self.m_surface.clone();
        if let Some(ref outer) = self.m_outer_loop {
            temp.set_outer_loop(outer.clone());
        }
        temp.clear_inner_loops();
        for loop_crv in &self.m_inner_loops {
            temp.add_inner_loop(loop_crv.clone());
        }
        temp.mesh()
    }

    pub fn transform_self(&mut self) {
        let xf = self.xform.clone();
        self.m_surface.transform(&xf);
        self.xform = Xform::identity();
    }

    pub fn transformed(&self) -> Self {
        let mut ts = self.clone();
        ts.transform_self();
        ts
    }

    pub fn duplicate(&self) -> Self {
        let mut copy = self.clone();
        copy.guid = uuid::Uuid::new_v4().to_string();
        copy
    }

    pub fn to_string(&self) -> String {
        format!("TrimmedSurface(name={}, trimmed={}, holes={})",
                self.name, self.is_trimmed(), self.inner_loop_count())
    }

    pub fn repr(&self) -> String {
        format!("TrimmedSurface(\n  name={},\n  trimmed={},\n  holes={},\n  surface={}\n)",
                self.name, self.is_trimmed(), self.inner_loop_count(), self.m_surface.to_string())
    }

    // JSON serialization
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

    // Protobuf serialization
    pub fn pb_dumps(&self) -> Vec<u8> {
        use prost::Message;

        let surface_data = self.m_surface.pb_dumps();
        let surface_proto = crate::proto::NurbsSurface::decode(surface_data.as_slice()).unwrap();

        let outer_loop = if self.is_trimmed() {
            let data = self.m_outer_loop.as_ref().unwrap().to_protobuf();
            Some(crate::proto::NurbsCurve::decode(data.as_slice()).unwrap())
        } else {
            None
        };

        let inner_loops: Vec<_> = self.m_inner_loops.iter().map(|c| {
            let data = c.to_protobuf();
            crate::proto::NurbsCurve::decode(data.as_slice()).unwrap()
        }).collect();

        let proto = crate::proto::TrimmedSurface {
            guid: self.guid.clone(),
            name: self.name.clone(),
            width: self.width,
            surface: Some(surface_proto),
            outer_loop,
            inner_loops,
            surfacecolor: Some(crate::proto::Color {
                guid: self.surfacecolor.guid.clone(),
                name: self.surfacecolor.name.clone(),
                r: self.surfacecolor.r as i32,
                g: self.surfacecolor.g as i32,
                b: self.surfacecolor.b as i32,
                a: self.surfacecolor.a as i32,
            }),
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

        let proto = crate::proto::TrimmedSurface::decode(data)?;
        let mut ts = Self::new();
        ts.guid = proto.guid;
        ts.name = proto.name;
        ts.width = proto.width;

        if let Some(srf_proto) = proto.surface {
            let srf_data = srf_proto.encode_to_vec();
            ts.m_surface = NurbsSurface::pb_loads(&srf_data)?;
        }

        if let Some(ol) = proto.outer_loop {
            let data = ol.encode_to_vec();
            if let Ok(crv) = NurbsCurve::from_protobuf(&data) {
                ts.m_outer_loop = Some(crv);
            }
        }

        for il in proto.inner_loops {
            let data = il.encode_to_vec();
            if let Ok(crv) = NurbsCurve::from_protobuf(&data) {
                ts.m_inner_loops.push(crv);
            }
        }

        if let Some(color) = proto.surfacecolor {
            ts.surfacecolor.guid = color.guid;
            ts.surfacecolor.name = color.name;
            ts.surfacecolor.r = color.r as u8;
            ts.surfacecolor.g = color.g as u8;
            ts.surfacecolor.b = color.b as u8;
            ts.surfacecolor.a = color.a as u8;
        }

        if let Some(xform) = proto.xform {
            ts.xform.guid = xform.guid;
            ts.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 { ts.xform.m[i] = *val; }
            }
        }

        Ok(ts)
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

impl Default for TrimmedSurface {
    fn default() -> Self { Self::new() }
}

impl PartialEq for TrimmedSurface {
    fn eq(&self, other: &Self) -> bool {
        if self.name != other.name { return false; }
        if self.width != other.width { return false; }
        if self.surfacecolor != other.surfacecolor { return false; }
        if self.xform != other.xform { return false; }
        if self.m_surface != other.m_surface { return false; }
        true
    }
}

impl std::fmt::Display for TrimmedSurface {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string())
    }
}
