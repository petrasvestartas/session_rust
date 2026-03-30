use crate::closest::Closest;
use crate::nurbssurface::NurbsSurface;
use crate::nurbscurve::NurbsCurve;
use crate::primitives::Primitives;
use crate::xform::Xform;
use crate::color::Color;
use crate::point::Point;
use crate::vector::Vector;
use crate::mesh::Mesh;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct TrimmedSurface {
    #[serde(serialize_with = "crate::guid_serde::serialize", deserialize_with = "crate::guid_serde::deserialize")]
    guid: std::sync::OnceLock<String>,
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
            guid: std::sync::OnceLock::new(),
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
        let srf = Primitives::create_planar(boundary);
        if !srf.is_valid() { return None; }

        let dom_u = srf.domain(0)?;
        let dom_v = srf.domain(1)?;
        let range_u = dom_u.1 - dom_u.0;
        let range_v = dom_v.1 - dom_v.0;

        let n_samples = 50usize.max(boundary.cv_count() * 4);
        let (pts3d, _) = boundary.divide_by_count(n_samples, true);
        let mut uv_pts = Vec::new();
        for pt in &pts3d {
            let (u, v, _) = Closest::surface_point(&srf, pt, 0.0, 0.0, 0.0, 0.0);
            let nu = (u - dom_u.0) / range_u;
            let nv = (v - dom_v.0) / range_v;
            uv_pts.push(Point::new(nu, nv, 0.0));
        }

        let mut ts = Self::new();
        ts.m_surface = srf;
        if uv_pts.len() >= 3 {
            ts.m_outer_loop = Some(NurbsCurve::create(false, 3, &uv_pts));
        }
        Some(ts)
    }

    pub fn guid(&self) -> &str {
        self.guid.get_or_init(|| uuid::Uuid::new_v4().to_string())
    }

    pub fn set_guid(&self, g: String) {
        let _ = self.guid.set(g);
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
        let dom = curve_3d.domain();
        let sdom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
        let sdom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
        let range_u = sdom_u.1 - sdom_u.0;
        let range_v = sdom_v.1 - sdom_v.0;

        let n_samples = std::cmp::max(curve_3d.cv_count() * 4, 32);
        let mut uv_pts = Vec::new();
        for i in 0..n_samples {
            let t = dom.0 + (dom.1 - dom.0) * i as f64 / n_samples as f64;
            let pt3d = curve_3d.point_at(t);
            let (u, v, _) = Closest::surface_point(&self.m_surface, &pt3d, 0.0, 0.0, 0.0, 0.0);
            let nu = (u - sdom_u.0) / range_u;
            let nv = (v - sdom_v.0) / range_v;
            uv_pts.push(Point::new(nu, nv, 0.0));
        }
        if uv_pts.len() >= 3 {
            self.m_inner_loops.push(NurbsCurve::create(true, 1, &uv_pts));
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
        if !self.is_trimmed() { return self.m_surface.mesh(); }

        // Planar: boundary-conforming ear-clip triangulation
        if self.m_surface.is_planar(1e-6) {
            let disc = |crv: &NurbsCurve| -> Vec<Point> {
                let n = if crv.degree() > 1 { (crv.cv_count() * 4).max(16) } else { (crv.cv_count().saturating_sub(1)).max(4) };
                let (pts, _) = crv.divide_by_count(n, true);
                pts
            };
            let outer_loop = self.m_outer_loop.as_ref().unwrap();
            let outer_pts = disc(outer_loop);
            let hole_pts: Vec<Vec<Point>> = self.m_inner_loops.iter().map(|c| disc(c)).collect();
            let mut pts3d: Vec<Point> = Vec::new();
            let mut add_pts = |uv_list: &[Point]| {
                let mut n = uv_list.len();
                if n > 1 && (uv_list[0][0]-uv_list[n-1][0]).abs() < 1e-12 &&
                   (uv_list[0][1]-uv_list[n-1][1]).abs() < 1e-12 { n -= 1; }
                for i in 0..n {
                    pts3d.push(self.m_surface.point_at(uv_list[i][0], uv_list[i][1])
                        .unwrap_or(Point::new(0.0, 0.0, 0.0)));
                }
            };
            add_pts(&outer_pts);
            for hp in &hole_pts { add_pts(hp); }
            let strip_close = |pts: &[Point]| -> Vec<Point> {
                let mut v: Vec<Point> = pts.to_vec();
                if v.len() > 1 && (v[0][0]-v[v.len()-1][0]).abs() < 1e-12 && (v[0][1]-v[v.len()-1][1]).abs() < 1e-12 { v.pop(); }
                v
            };
            let mut border = strip_close(&outer_pts);
            let mut holes: Vec<Vec<Point>> = hole_pts.iter().map(|h| strip_close(h)).collect();
            let area: f64 = (0..border.len()).map(|j| { let k=(j+1)%border.len(); border[j][0]*border[k][1]-border[k][0]*border[j][1] }).sum::<f64>() * 0.5;
            if area < 0.0 { border.reverse(); }
            for h in &mut holes {
                let ha: f64 = (0..h.len()).map(|j| { let k=(j+1)%h.len(); h[j][0]*h[k][1]-h[k][0]*h[j][1] }).sum::<f64>() * 0.5;
                if ha > 0.0 { h.reverse(); }
            }
            let tris = crate::remesh_cdt::cdt_triangulate(&border, &holes);
            let np = pts3d.len();
            let mut polygons: Vec<Vec<Point>> = Vec::new();
            for &(v0, v1, v2) in &tris {
                if v0 < np && v1 < np && v2 < np {
                    polygons.push(vec![pts3d[v0].clone(), pts3d[v1].clone(), pts3d[v2].clone()]);
                }
            }
            let mut result = Mesh::from_polylines(polygons, None);
            let dom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
            let dom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
            let nrm = self.m_surface.normal_at((dom_u.0+dom_u.1)/2.0, (dom_v.0+dom_v.1)/2.0);
            for (_, vd) in result.vertex.iter_mut() {
                vd.set_normal(nrm[0], nrm[1], nrm[2]);
            }
            return result;
        }

        // Non-planar: grid + point-in-polygon discard
        let dom_u = self.m_surface.domain(0).unwrap_or((0.0, 1.0));
        let dom_v = self.m_surface.domain(1).unwrap_or((0.0, 1.0));
        let range_u = dom_u.1 - dom_u.0;
        let range_v = dom_v.1 - dom_v.0;
        if range_u < 1e-15 || range_v < 1e-15 { return self.m_surface.mesh(); }
        let p00 = self.m_surface.point_at(dom_u.0, dom_v.0).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let p10 = self.m_surface.point_at(dom_u.1, dom_v.0).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let p01 = self.m_surface.point_at(dom_u.0, dom_v.1).unwrap_or(Point::new(0.0, 0.0, 0.0));
        let u_len = ((p10[0]-p00[0]).powi(2)+(p10[1]-p00[1]).powi(2)+(p10[2]-p00[2]).powi(2)).sqrt();
        let v_len = ((p01[0]-p00[0]).powi(2)+(p01[1]-p00[1]).powi(2)+(p01[2]-p00[2]).powi(2)).sqrt();
        let max_dim = u_len.max(v_len);
        let max_edge = if max_dim > 1e-10 { max_dim / 10.0 } else { 0.1 };
        let nu = if u_len > 1e-10 { 12usize.max((u_len / max_edge).ceil() as usize + 1) } else { 12 };
        let nv = if v_len > 1e-10 { 12usize.max((v_len / max_edge).ceil() as usize + 1) } else { 12 };
        let us: Vec<f64> = (0..nu).map(|i| dom_u.0 + i as f64 * range_u / (nu - 1) as f64).collect();
        let vs: Vec<f64> = (0..nv).map(|j| dom_v.0 + j as f64 * range_v / (nv - 1) as f64).collect();
        let mut full = Mesh::new();
        for i in 0..nu {
            for j in 0..nv {
                let pt = self.m_surface.point_at(us[i], vs[j]).unwrap_or(Point::new(0.0, 0.0, 0.0));
                let vk = full.add_vertex(pt, None);
                full.vertex.get_mut(&vk).unwrap().attributes.insert("u".to_string(), us[i]);
                full.vertex.get_mut(&vk).unwrap().attributes.insert("v".to_string(), vs[j]);
            }
        }
        for i in 0..nu-1 {
            for j in 0..nv-1 {
                let v00 = i * nv + j; let v10 = (i+1) * nv + j;
                let v01 = i * nv + (j+1); let v11 = (i+1) * nv + (j+1);
                if (i + j) % 2 == 0 {
                    full.add_face(vec![v00, v10, v11], None);
                    full.add_face(vec![v00, v11, v01], None);
                } else {
                    full.add_face(vec![v00, v10, v01], None);
                    full.add_face(vec![v10, v11, v01], None);
                }
            }
        }
        use crate::polyline::Polyline;
        let discretize = |crv: &NurbsCurve| -> Polyline {
            let n = std::cmp::max(crv.cv_count() * 4, 16);
            let (pts, _) = crv.divide_by_count(n, true);
            Polyline::new(pts.iter().map(|p| Point::new(p[0], p[1], 0.0)).collect())
        };
        let outer_loop = self.m_outer_loop.as_ref().unwrap();
        let outer_polygon = discretize(outer_loop);
        let inner_polygons: Vec<Polyline> = self.m_inner_loops.iter().map(|c| discretize(c)).collect();
        let mut keep_verts = std::collections::HashSet::new();
        for (&vk, vd) in &full.vertex {
            let u_raw = vd.attributes.get("u").copied().unwrap_or(0.0);
            let v_raw = vd.attributes.get("v").copied().unwrap_or(0.0);
            let pt = Point::new(u_raw, v_raw, 0.0);
            if !outer_polygon.point_in_polygon_2d(&pt) { continue; }
            let in_hole = inner_polygons.iter().any(|ip| ip.point_in_polygon_2d(&pt));
            if !in_hole { keep_verts.insert(vk); }
        }
        let mut polygons: Vec<Vec<Point>> = Vec::new();
        for (_, fverts) in &full.face {
            if fverts.iter().all(|vi| keep_verts.contains(vi)) {
                let poly: Vec<Point> = fverts.iter()
                    .filter_map(|vi| full.vertex.get(vi).map(|v| Point::new(v.x, v.y, v.z)))
                    .collect();
                polygons.push(poly);
            }
        }
        let mut result = Mesh::from_polylines(polygons, None);
        let nv_total = result.vertex.len();
        let mut vnx = vec![0.0f64; nv_total + 1];
        let mut vny = vec![0.0f64; nv_total + 1];
        let mut vnz = vec![0.0f64; nv_total + 1];
        for (_, vids) in &result.face {
            if vids.len() < 3 { continue; }
            let p0 = &result.vertex[&vids[0]];
            let p1 = &result.vertex[&vids[1]];
            let p2 = &result.vertex[&vids[2]];
            let e1x = p1.x - p0.x; let e1y = p1.y - p0.y; let e1z = p1.z - p0.z;
            let e2x = p2.x - p0.x; let e2y = p2.y - p0.y; let e2z = p2.z - p0.z;
            let fnx = e1y*e2z - e1z*e2y;
            let fny = e1z*e2x - e1x*e2z;
            let fnz = e1x*e2y - e1y*e2x;
            for &vi in vids { vnx[vi] += fnx; vny[vi] += fny; vnz[vi] += fnz; }
        }
        for i in 0..=nv_total {
            let len = (vnx[i]*vnx[i] + vny[i]*vny[i] + vnz[i]*vnz[i]).sqrt();
            if len > 1e-15 { vnx[i] /= len; vny[i] /= len; vnz[i] /= len; }
            if let Some(v) = result.vertex.get_mut(&i) {
                v.set_normal(vnx[i], vny[i], vnz[i]);
            }
        }
        result
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
        copy.guid = std::sync::OnceLock::new();
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
            guid: self.guid().to_string(),
            name: self.name.clone(),
            width: self.width,
            surface: Some(surface_proto),
            outer_loop,
            inner_loops,
            surfacecolor: Some(crate::proto::Color {
                guid: self.surfacecolor.guid().to_string(),
                name: self.surfacecolor.name.clone(),
                r: self.surfacecolor.r as i32,
                g: self.surfacecolor.g as i32,
                b: self.surfacecolor.b as i32,
                a: self.surfacecolor.a as i32,
            }),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid().to_string(),
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
        ts.set_guid(proto.guid.clone());
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
            ts.surfacecolor.set_guid(color.guid.clone());
            ts.surfacecolor.name = color.name;
            ts.surfacecolor.r = color.r as u8;
            ts.surfacecolor.g = color.g as u8;
            ts.surfacecolor.b = color.b as u8;
            ts.surfacecolor.a = color.a as u8;
        }

        if let Some(xform) = proto.xform {
            ts.xform.set_guid(xform.guid.clone());
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
