use crate::{BoundingBox, Color, Line, Point, Tolerance, Vector, Xform, BVH};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

/// Weighting scheme for vertex normal computation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum NormalWeighting {
    Area,
    Angle,
    Uniform,
}

/// A halfedge mesh data structure for representing polygonal surfaces
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename = "Mesh")]
pub struct Mesh {
    pub halfedge: HashMap<usize, HashMap<usize, Option<usize>>>, // Halfedge connectivity
    pub vertex: HashMap<usize, VertexData>,                      // Vertex data
    pub face: HashMap<usize, Vec<usize>>,                        // Face vertex lists
    pub facedata: HashMap<usize, HashMap<String, f64>>,          // Face attributes
    pub edgedata: HashMap<(usize, usize), HashMap<String, f64>>, // Edge attributes
    pub default_vertex_attributes: HashMap<String, f64>,         // Default vertex attrs
    pub default_face_attributes: HashMap<String, f64>,           // Default face attrs
    pub default_edge_attributes: HashMap<String, f64>,           // Default edge attrs
    #[serde(skip)]
    pub triangulation: HashMap<usize, Vec<[usize; 3]>>, // Cached triangulations
    max_vertex: usize,                                           // Next vertex key
    max_face: usize,                                             // Next face key
    pub guid: String,                                            // Unique identifier
    pub name: String,                                            // Mesh name
    #[serde(skip)]
    pub pointcolors: Vec<Color>,               // Vertex colors
    #[serde(skip)]
    pub facecolors: Vec<Color>,                // Face colors
    #[serde(skip)]
    pub linecolors: Vec<Color>,                // Edge colors
    #[serde(skip)]
    pub widths: Vec<f64>,                      // Edge widths
    #[serde(default = "Xform::identity")]
    pub xform: Xform,   // Transformation matrix
    // Cached triangle BVH for ray queries (not serialized)
    #[serde(skip)]
    pub tri_bvh: Option<BVH>,
    #[serde(skip)]
    pub tri_tris: Vec<[usize; 3]>,
    #[serde(skip)]
    pub tri_vertices: Vec<Point>,
}

/// Vertex data containing position and attributes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VertexData {
    pub x: f64,                           // X coordinate
    pub y: f64,                           // Y coordinate
    pub z: f64,                           // Z coordinate
    pub attributes: HashMap<String, f64>, // Vertex attributes
}

impl VertexData {
    pub fn new(point: Point) -> Self {
        Self {
            x: point[0],
            y: point[1],
            z: point[2],
            attributes: HashMap::new(),
        }
    }

    pub fn position(&self) -> Point {
        Point::new(self.x, self.y, self.z)
    }

    pub fn set_position(&mut self, point: Point) {
        self.x = point[0];
        self.y = point[1];
        self.z = point[2];
    }

    pub fn color(&self) -> [f64; 3] {
        [
            self.attributes.get("r").copied().unwrap_or(0.5),
            self.attributes.get("g").copied().unwrap_or(0.5),
            self.attributes.get("b").copied().unwrap_or(0.5),
        ]
    }

    pub fn set_color(&mut self, r: f64, g: f64, b: f64) {
        self.attributes.insert("r".to_string(), r);
        self.attributes.insert("g".to_string(), g);
        self.attributes.insert("b".to_string(), b);
    }

    pub fn normal(&self) -> Option<[f64; 3]> {
        let nx = self.attributes.get("nx")?;
        let ny = self.attributes.get("ny")?;
        let nz = self.attributes.get("nz")?;
        Some([*nx, *ny, *nz])
    }

    pub fn set_normal(&mut self, nx: f64, ny: f64, nz: f64) {
        self.attributes.insert("nx".to_string(), nx);
        self.attributes.insert("ny".to_string(), ny);
        self.attributes.insert("nz".to_string(), nz);
    }
}

impl Default for Mesh {
    fn default() -> Self {
        Self::new()
    }
}

impl Mesh {
    /// Creates a new empty halfedge mesh
    pub fn new() -> Self {
        let mut default_vertex_attributes = HashMap::new();
        default_vertex_attributes.insert("x".to_string(), 0.0);
        default_vertex_attributes.insert("y".to_string(), 0.0);
        default_vertex_attributes.insert("z".to_string(), 0.0);

        Mesh {
            halfedge: HashMap::new(),
            vertex: HashMap::new(),
            face: HashMap::new(),
            facedata: HashMap::new(),
            edgedata: HashMap::new(),
            default_vertex_attributes,
            default_face_attributes: HashMap::new(),
            default_edge_attributes: HashMap::new(),
            triangulation: HashMap::new(),
            max_vertex: 0,
            max_face: 0,
            guid: uuid::Uuid::new_v4().to_string(),
            name: "my_mesh".to_string(),
            pointcolors: Vec::new(),
            facecolors: Vec::new(),
            linecolors: Vec::new(),
            widths: Vec::new(),
            xform: Xform::identity(),
            tri_bvh: None,
            tri_tris: Vec::new(),
            tri_vertices: Vec::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.vertex.is_empty() && self.face.is_empty()
    }

    pub fn clear(&mut self) {
        self.halfedge.clear();
        self.vertex.clear();
        self.face.clear();
        self.facedata.clear();
        self.edgedata.clear();
        self.triangulation.clear();
        self.max_vertex = 0;
        self.max_face = 0;
        self.pointcolors.clear();
        self.facecolors.clear();
        self.linecolors.clear();
        self.widths.clear();
        self.invalidate_triangle_bvh();
    }

    pub fn number_of_vertices(&self) -> usize {
        self.vertex.len()
    }

    pub fn number_of_faces(&self) -> usize {
        self.face.len()
    }

    pub fn number_of_edges(&self) -> usize {
        let mut seen = HashSet::new();
        let mut count = 0;

        for u in self.halfedge.keys() {
            if let Some(neighbors) = self.halfedge.get(u) {
                for v in neighbors.keys() {
                    let edge = if u < v { (*u, *v) } else { (*v, *u) };
                    if seen.insert(edge) {
                        count += 1;
                    }
                }
            }
        }

        count
    }

    pub fn euler(&self) -> i32 {
        let v = self.number_of_vertices() as i32;
        let e = self.number_of_edges() as i32;
        let f = self.number_of_faces() as i32;
        v - e + f
    }

    pub fn add_vertex(&mut self, position: Point, key: Option<usize>) -> usize {
        let vertex_key = key.unwrap_or_else(|| {
            self.max_vertex += 1;
            self.max_vertex
        });

        if vertex_key >= self.max_vertex {
            self.max_vertex = vertex_key + 1;
        }

        let vertex_data = VertexData::new(position);
        self.vertex.insert(vertex_key, vertex_data);
        self.halfedge.entry(vertex_key).or_default();
        self.pointcolors.push(Color::white());
        self.invalidate_triangle_bvh();

        vertex_key
    }

    pub fn add_face(&mut self, vertices: Vec<usize>, fkey: Option<usize>) -> Option<usize> {
        if vertices.len() < 3 {
            return None;
        }

        if !vertices.iter().all(|v| self.vertex.contains_key(v)) {
            return None;
        }

        let mut unique_vertices = HashSet::new();
        for vertex in &vertices {
            if !unique_vertices.insert(*vertex) {
                return None;
            }
        }

        let face_key = fkey.unwrap_or_else(|| {
            self.max_face += 1;
            self.max_face
        });

        if face_key >= self.max_face {
            self.max_face = face_key + 1;
        }

        self.face.insert(face_key, vertices.clone());
        self.triangulation.remove(&face_key);
        self.facecolors.push(Color::white());
        self.invalidate_triangle_bvh();

        for i in 0..vertices.len() {
            let u = vertices[i];
            let v = vertices[(i + 1) % vertices.len()];

            self.halfedge.entry(u).or_default();
            self.halfedge.entry(v).or_default();

            let is_new_edge = !self.halfedge.get(&v).unwrap().contains_key(&u);

            self.halfedge.get_mut(&u).unwrap().insert(v, Some(face_key));

            if is_new_edge {
                self.halfedge.get_mut(&v).unwrap().insert(u, None);
                self.linecolors.push(Color::white());
                self.widths.push(1.0);
            }
        }

        Some(face_key)
    }

    pub fn vertex_position(&self, vertex_key: usize) -> Option<Point> {
        self.vertex.get(&vertex_key).map(|v| v.position())
    }

    pub fn face_vertices(&self, face_key: usize) -> Option<&Vec<usize>> {
        self.face.get(&face_key)
    }

    pub fn vertex_neighbors(&self, vertex_key: usize) -> Vec<usize> {
        self.halfedge
            .get(&vertex_key)
            .map(|neighbors| neighbors.keys().copied().collect())
            .unwrap_or_default()
    }

    pub fn vertex_faces(&self, vertex_key: usize) -> Vec<usize> {
        let mut faces = Vec::new();
        for (face_key, face_vertices) in &self.face {
            if face_vertices.contains(&vertex_key) {
                faces.push(*face_key);
            }
        }
        faces
    }

    pub fn is_vertex_on_boundary(&self, vertex_key: usize) -> bool {
        if let Some(neigh) = self.halfedge.get(&vertex_key) {
            for (_v, face_opt) in neigh.iter() {
                if face_opt.is_none() {
                    return true;
                }
            }
        }

        for (_u, neigh) in self.halfedge.iter() {
            if let Some(face_opt) = neigh.get(&vertex_key) {
                if face_opt.is_none() {
                    return true;
                }
            }
        }
        false
    }

    pub fn face_normal(&self, face_key: usize) -> Option<Vector> {
        let vertices = self.face.get(&face_key)?;
        if vertices.len() < 3 {
            return None;
        }

        let p0 = self.vertex_position(vertices[0])?;
        let p1 = self.vertex_position(vertices[1])?;
        let p2 = self.vertex_position(vertices[2])?;

        let u = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
        let v = Vector::new(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);

        let normal = u.cross(&v);
        let len = normal.magnitude();
        if len > Tolerance::ZERO_TOLERANCE {
            Some(Vector::new(
                normal[0] / len,
                normal[1] / len,
                normal[2] / len,
            ))
        } else {
            None
        }
    }

    pub fn vertex_normal(&self, vertex_key: usize) -> Option<Vector> {
        self.vertex_normal_weighted(vertex_key, NormalWeighting::Area)
    }

    pub fn vertex_normal_weighted(
        &self,
        vertex_key: usize,
        weighting: NormalWeighting,
    ) -> Option<Vector> {
        let faces = self.vertex_faces(vertex_key);
        if faces.is_empty() {
            return None;
        }

        let mut normal_acc = Vector::new(0.0, 0.0, 0.0);

        for face_key in faces {
            if let Some(face_normal) = self.face_normal(face_key) {
                let weight = match weighting {
                    NormalWeighting::Area => self.face_area(face_key).unwrap_or(1.0),
                    NormalWeighting::Angle => self
                        .vertex_angle_in_face(vertex_key, face_key)
                        .unwrap_or(1.0),
                    NormalWeighting::Uniform => 1.0,
                };

                normal_acc[0] = normal_acc[0] + face_normal[0] * weight;
                normal_acc[1] = normal_acc[1] + face_normal[1] * weight;
                normal_acc[2] = normal_acc[2] + face_normal[2] * weight;
            }
        }

        let len = normal_acc.magnitude();
        if len > Tolerance::ZERO_TOLERANCE {
            Some(Vector::new(
                normal_acc[0] / len,
                normal_acc[1] / len,
                normal_acc[2] / len,
            ))
        } else {
            None
        }
    }

    pub fn face_area(&self, face_key: usize) -> Option<f64> {
        let vertices = self.face.get(&face_key)?;
        if vertices.len() < 3 {
            return Some(0.0);
        }

        let mut area = 0.0;
        let p0 = self.vertex_position(vertices[0])?;

        for i in 1..(vertices.len() - 1) {
            let p1 = self.vertex_position(vertices[i])?;
            let p2 = self.vertex_position(vertices[i + 1])?;

            let u = Vector::new(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
            let v = Vector::new(p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]);

            area += u.cross(&v).magnitude() * 0.5;
        }

        Some(area)
    }

    pub fn vertex_angle_in_face(&self, vertex_key: usize, face_key: usize) -> Option<f64> {
        let vertices = self.face.get(&face_key)?;
        let vertex_index = vertices.iter().position(|&v| v == vertex_key)?;

        let n = vertices.len();
        let prev_vertex = vertices[(vertex_index + n - 1) % n];
        let next_vertex = vertices[(vertex_index + 1) % n];

        let center = self.vertex_position(vertex_key)?;
        let prev_pos = self.vertex_position(prev_vertex)?;
        let next_pos = self.vertex_position(next_vertex)?;

        let u = Vector::new(
            prev_pos[0] - center[0],
            prev_pos[1] - center[1],
            prev_pos[2] - center[2],
        );
        let v = Vector::new(
            next_pos[0] - center[0],
            next_pos[1] - center[1],
            next_pos[2] - center[2],
        );

        let u_len = u.magnitude();
        let v_len = v.magnitude();

        if u_len < Tolerance::ZERO_TOLERANCE || v_len < Tolerance::ZERO_TOLERANCE {
            return Some(0.0);
        }

        let cos_angle = u.dot(&v) / (u_len * v_len);
        let cos_angle = cos_angle.clamp(-1.0, 1.0);
        Some(cos_angle.acos())
    }

    pub fn face_normals(&self) -> HashMap<usize, Vector> {
        let mut normals = HashMap::new();
        for face_key in self.face.keys() {
            if let Some(normal) = self.face_normal(*face_key) {
                normals.insert(*face_key, normal);
            }
        }
        normals
    }

    pub fn vertex_normals(&self) -> HashMap<usize, Vector> {
        self.vertex_normals_weighted(NormalWeighting::Area)
    }

    pub fn vertex_normals_weighted(&self, weighting: NormalWeighting) -> HashMap<usize, Vector> {
        let mut normals = HashMap::new();
        for vertex_key in self.vertex.keys() {
            if let Some(normal) = self.vertex_normal_weighted(*vertex_key, weighting) {
                normals.insert(*vertex_key, normal);
            }
        }
        normals
    }

    pub fn vertex_index(&self) -> HashMap<usize, usize> {
        let mut keys: Vec<usize> = self.vertex.keys().copied().collect();
        keys.sort();
        keys.iter()
            .enumerate()
            .map(|(index, &key)| (key, index))
            .collect()
    }

    pub fn to_vertices_and_faces(&self) -> (Vec<Point>, Vec<Vec<usize>>) {
        let vertex_index = self.vertex_index();
        let mut vertices: Vec<Point> = vec![Point::default(); self.vertex.len()];

        for (&key, data) in &self.vertex {
            let idx = vertex_index[&key];
            vertices[idx] = data.position();
        }

        // Sort face keys to ensure consistent ordering
        let mut face_keys: Vec<usize> = self.face.keys().copied().collect();
        face_keys.sort();

        let mut faces = Vec::new();
        for face_key in face_keys {
            let face_vertices = &self.face[&face_key];
            let remapped: Vec<usize> = face_vertices.iter().map(|v| vertex_index[v]).collect();
            faces.push(remapped);
        }

        (vertices, faces)
    }

    pub fn from_polygons(polygons: Vec<Vec<Point>>, precision: Option<f64>) -> Self {
        let mut mesh = Mesh::new();
        let mut map_eps: HashMap<(i64, i64, i64), usize> = HashMap::new();
        let mut map_exact: HashMap<(u64, u64, u64), usize> = HashMap::new();
        let eps = precision.unwrap_or(0.0);
        let use_eps = eps > 0.0;

        let mut get_vkey = |p: &Point, mesh: &mut Mesh| -> usize {
            if use_eps {
                let kx = (p[0] / eps).round() as i64;
                let ky = (p[1] / eps).round() as i64;
                let kz = (p[2] / eps).round() as i64;
                let key = (kx, ky, kz);
                if let Some(&vk) = map_eps.get(&key) {
                    return vk;
                }
                let vk = mesh.add_vertex(p.clone(), None);
                map_eps.insert(key, vk);
                vk
            } else {
                let key = (p[0].to_bits(), p[1].to_bits(), p[2].to_bits());
                if let Some(&vk) = map_exact.get(&key) {
                    return vk;
                }
                let vk = mesh.add_vertex(p.clone(), None);
                map_exact.insert(key, vk);
                vk
            }
        };

        for poly in polygons.into_iter() {
            if poly.len() < 3 {
                continue;
            }
            let mut vkeys: Vec<usize> = Vec::with_capacity(poly.len());
            for p in &poly {
                let vk = get_vkey(p, &mut mesh);
                vkeys.push(vk);
            }
            let _ = mesh.add_face(vkeys, None);
        }

        mesh
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Triangle BVH cache and ray casting
    ///////////////////////////////////////////////////////////////////////////////////////////

    fn invalidate_triangle_bvh(&mut self) {
        self.tri_bvh = None;
        self.tri_tris.clear();
        self.tri_vertices.clear();
    }

    fn ensure_triangle_bvh(&mut self) {
        if self.tri_bvh.is_some() && !self.tri_tris.is_empty() && !self.tri_vertices.is_empty() {
            return;
        }

        let (vertices, faces) = self.to_vertices_and_faces();
        let mut tris: Vec<[usize; 3]> = Vec::new();
        let mut tri_boxes: Vec<BoundingBox> = Vec::new();

        for face in faces {
            if face.len() < 3 {
                continue;
            }
            let v0 = face[0];
            for i in 1..(face.len() - 1) {
                let t = [v0, face[i], face[i + 1]];
                tris.push(t);
                let pts = [
                    vertices[t[0]].clone(),
                    vertices[t[1]].clone(),
                    vertices[t[2]].clone(),
                ];
                tri_boxes.push(BoundingBox::from_points(&pts, 0.0));
            }
        }

        if tris.is_empty() {
            self.tri_bvh = None;
            self.tri_tris.clear();
            self.tri_vertices = vertices; // keep for consistency
            return;
        }

        let world_size = BVH::compute_world_size(&tri_boxes);
        let bvh = BVH::from_boxes(&tri_boxes, world_size);
        self.tri_vertices = vertices;
        self.tri_tris = tris;
        self.tri_bvh = Some(bvh);
    }

    pub fn ray_cast_bvh(&mut self, ray: &Line, epsilon: f64) -> Option<Point> {
        self.ensure_triangle_bvh();
        let bvh = match &self.tri_bvh {
            Some(b) => b,
            None => return None,
        };

        let origin = ray.start();
        let dir = ray.to_vector();
        let len = dir.magnitude();
        if len <= Tolerance::ZERO_TOLERANCE {
            return None;
        }
        let dir_unit = Vector::new(dir[0] / len, dir[1] / len, dir[2] / len);

        let mut candidate_ids: Vec<usize> = Vec::new();
        bvh.ray_cast(&origin, &dir_unit, &mut candidate_ids, true);
        if candidate_ids.is_empty() {
            return None;
        }

        let mut best_t = f64::INFINITY;
        let mut best_p: Option<Point> = None;

        for idx in candidate_ids {
            if idx >= self.tri_tris.len() {
                continue;
            }
            let tri = self.tri_tris[idx];
            let v0 = &self.tri_vertices[tri[0]];
            let v1 = &self.tri_vertices[tri[1]];
            let v2 = &self.tri_vertices[tri[2]];
            if let Some(p) = crate::intersection::ray_triangle(ray, v0, v1, v2, epsilon) {
                let dx = p[0] - origin[0];
                let dy = p[1] - origin[1];
                let dz = p[2] - origin[2];
                let t = dx * dir_unit[0] + dy * dir_unit[1] + dz * dir_unit[2];
                if t >= 0.0 && t < best_t {
                    best_t = t;
                    best_p = Some(p);
                }
            }
        }

        best_p
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Color and Width Management
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn set_vertex_color(&mut self, index: usize, color: Color) {
        if index < self.pointcolors.len() {
            self.pointcolors[index] = color;
        }
    }

    pub fn set_face_color(&mut self, index: usize, color: Color) {
        if index < self.facecolors.len() {
            self.facecolors[index] = color;
        }
    }

    pub fn set_edge_color(&mut self, index: usize, color: Color) {
        if index < self.linecolors.len() {
            self.linecolors[index] = color;
        }
    }

    pub fn set_edge_width(&mut self, index: usize, width: f64) {
        if index < self.widths.len() {
            self.widths[index] = width;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Transformation
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn transform(&mut self) {
        let xform = self.xform.clone();
        for v in self.vertex.values_mut() {
            let mut pt = Point::new(v.x, v.y, v.z);
            xform.transform_point(&mut pt);
            v.x = pt[0];
            v.y = pt[1];
            v.z = pt[2];
        }
        self.xform = Xform::identity();
        self.invalidate_triangle_bvh();
    }

    pub fn transformed(&self) -> Self {
        let mut result = self.clone();
        result.transform();
        result
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // JSON
    ///////////////////////////////////////////////////////////////////////////////////////////

    /// Serializes the Mesh to JSON data
    pub fn jsondump(&self) -> serde_json::Value {
        let pointcolors_flat: Vec<u8> = self
            .pointcolors
            .iter()
            .flat_map(|c| vec![c.r, c.g, c.b])
            .collect();

        let facecolors_flat: Vec<u8> = self
            .facecolors
            .iter()
            .flat_map(|c| vec![c.r, c.g, c.b])
            .collect();

        let linecolors_flat: Vec<u8> = self
            .linecolors
            .iter()
            .flat_map(|c| vec![c.r, c.g, c.b])
            .collect();

        serde_json::json!({
            "type": "Mesh",
            "guid": self.guid,
            "name": self.name,
            "vertex": self.vertex,
            "face": self.face,
            "halfedge": self.halfedge,
            "facedata": self.facedata,
            "edgedata": self.edgedata,
            "default_vertex_attributes": self.default_vertex_attributes,
            "default_face_attributes": self.default_face_attributes,
            "default_edge_attributes": self.default_edge_attributes,
            "max_vertex": self.max_vertex,
            "max_face": self.max_face,
            "pointcolors": pointcolors_flat,
            "facecolors": facecolors_flat,
            "linecolors": linecolors_flat,
            "widths": self.widths
        })
    }

    pub fn jsonload(data: &serde_json::Value) -> Option<Self> {
        let mut mesh = Mesh::new();

        if let Some(guid) = data.get("guid").and_then(|v| v.as_str()) {
            mesh.guid = guid.to_string();
        }
        if let Some(name) = data.get("name").and_then(|v| v.as_str()) {
            mesh.name = name.to_string();
        }

        if let Some(vertex_data) = data.get("vertex") {
            mesh.vertex = serde_json::from_value(vertex_data.clone()).ok()?;
        }
        if let Some(face_data) = data.get("face") {
            mesh.face = serde_json::from_value(face_data.clone()).ok()?;
        }
        if let Some(halfedge_data) = data.get("halfedge") {
            mesh.halfedge = serde_json::from_value(halfedge_data.clone()).ok()?;
        }
        if let Some(facedata) = data.get("facedata") {
            mesh.facedata = serde_json::from_value(facedata.clone()).ok()?;
        }
        if let Some(edgedata) = data.get("edgedata") {
            mesh.edgedata = serde_json::from_value(edgedata.clone()).ok()?;
        }
        if let Some(max_vertex) = data.get("max_vertex").and_then(|v| v.as_u64()) {
            mesh.max_vertex = max_vertex as usize;
        }
        if let Some(max_face) = data.get("max_face").and_then(|v| v.as_u64()) {
            mesh.max_face = max_face as usize;
        }

        // Deserialize flat color arrays
        if let Some(pointcolors_flat) = data.get("pointcolors").and_then(|v| v.as_array()) {
            let rgb_values: Vec<u8> = pointcolors_flat
                .iter()
                .filter_map(|v| v.as_u64().map(|n| n as u8))
                .collect();
            mesh.pointcolors = rgb_values
                .chunks(3)
                .map(|chunk| {
                    if chunk.len() == 3 {
                        Color::new(chunk[0], chunk[1], chunk[2], 255)
                    } else {
                        Color::white()
                    }
                })
                .collect();
        }

        if let Some(facecolors_flat) = data.get("facecolors").and_then(|v| v.as_array()) {
            let rgb_values: Vec<u8> = facecolors_flat
                .iter()
                .filter_map(|v| v.as_u64().map(|n| n as u8))
                .collect();
            mesh.facecolors = rgb_values
                .chunks(3)
                .map(|chunk| {
                    if chunk.len() == 3 {
                        Color::new(chunk[0], chunk[1], chunk[2], 255)
                    } else {
                        Color::white()
                    }
                })
                .collect();
        }

        if let Some(linecolors_flat) = data.get("linecolors").and_then(|v| v.as_array()) {
            let rgb_values: Vec<u8> = linecolors_flat
                .iter()
                .filter_map(|v| v.as_u64().map(|n| n as u8))
                .collect();
            mesh.linecolors = rgb_values
                .chunks(3)
                .map(|chunk| {
                    if chunk.len() == 3 {
                        Color::new(chunk[0], chunk[1], chunk[2], 255)
                    } else {
                        Color::white()
                    }
                })
                .collect();
        }

        if let Some(widths) = data.get("widths").and_then(|v| v.as_array()) {
            mesh.widths = widths.iter().filter_map(|v| v.as_f64()).collect();
        }

        Some(mesh)
    }

    pub fn to_json(&self, filename: &str) -> std::io::Result<()> {
        let data = self.jsondump();
        std::fs::write(filename, serde_json::to_string_pretty(&data)?)
    }

    pub fn from_json(filename: &str) -> std::io::Result<Self> {
        let content = std::fs::read_to_string(filename)?;
        let data: serde_json::Value = serde_json::from_str(&content)?;
        Self::jsonload(&data).ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, "Invalid mesh data")
        })
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Protobuf Serialization
    ///////////////////////////////////////////////////////////////////////////////////////////

    pub fn to_protobuf(&self) -> Vec<u8> {
        use prost::Message;
        use std::collections::HashMap;

        let mut vertices: HashMap<u64, crate::proto::VertexData> = HashMap::new();
        for (&vkey, vdata) in &self.vertex {
            let mut attrs: HashMap<String, f64> = HashMap::new();
            for (k, v) in &vdata.attributes {
                attrs.insert(k.clone(), *v);
            }
            vertices.insert(vkey as u64, crate::proto::VertexData {
                x: vdata.x,
                y: vdata.y,
                z: vdata.z,
                attributes: attrs,
            });
        }

        let mut faces: HashMap<u64, crate::proto::FaceData> = HashMap::new();
        for (&fkey, fverts) in &self.face {
            let mut attrs: HashMap<String, f64> = HashMap::new();
            if let Some(fdata) = self.facedata.get(&fkey) {
                for (k, v) in fdata {
                    attrs.insert(k.clone(), *v);
                }
            }
            faces.insert(fkey as u64, crate::proto::FaceData {
                vertices: fverts.iter().map(|&v| v as u64).collect(),
                attributes: attrs,
            });
        }

        let mut halfedges: HashMap<u64, crate::proto::HalfedgeMap> = HashMap::new();
        for (&u, neighbors) in &self.halfedge {
            let mut neighbor_map: HashMap<u64, u64> = HashMap::new();
            for (&v, &fkey_opt) in neighbors {
                neighbor_map.insert(v as u64, fkey_opt.unwrap_or(usize::MAX) as u64);
            }
            halfedges.insert(u as u64, crate::proto::HalfedgeMap {
                neighbors: neighbor_map,
            });
        }

        let mut edge_data_vec: Vec<crate::proto::EdgeData> = Vec::new();
        for ((v1, v2), attrs) in &self.edgedata {
            let mut attr_map: HashMap<String, f64> = HashMap::new();
            for (k, v) in attrs {
                attr_map.insert(k.clone(), *v);
            }
            edge_data_vec.push(crate::proto::EdgeData {
                vertex1: *v1 as u64,
                vertex2: *v2 as u64,
                attributes: attr_map,
            });
        }

        let pointcolors: Vec<crate::proto::Color> = self.pointcolors.iter().map(|c| {
            crate::proto::Color {
                guid: c.guid.clone(),
                name: c.name.clone(),
                r: c.r as i32,
                g: c.g as i32,
                b: c.b as i32,
                a: c.a as i32,
            }
        }).collect();

        let facecolors: Vec<crate::proto::Color> = self.facecolors.iter().map(|c| {
            crate::proto::Color {
                guid: c.guid.clone(),
                name: c.name.clone(),
                r: c.r as i32,
                g: c.g as i32,
                b: c.b as i32,
                a: c.a as i32,
            }
        }).collect();

        let linecolors: Vec<crate::proto::Color> = self.linecolors.iter().map(|c| {
            crate::proto::Color {
                guid: c.guid.clone(),
                name: c.name.clone(),
                r: c.r as i32,
                g: c.g as i32,
                b: c.b as i32,
                a: c.a as i32,
            }
        }).collect();

        let proto = crate::proto::Mesh {
            guid: self.guid.clone(),
            name: self.name.clone(),
            vertices,
            faces,
            halfedges,
            edge_data: edge_data_vec,
            default_vertex_attributes: self.default_vertex_attributes.clone(),
            default_face_attributes: self.default_face_attributes.clone(),
            default_edge_attributes: self.default_edge_attributes.clone(),
            pointcolors,
            facecolors,
            linecolors,
            widths: self.widths.clone(),
            xform: Some(crate::proto::Xform {
                guid: self.xform.guid.clone(),
                name: self.xform.name.clone(),
                matrix: self.xform.m.to_vec(),
            }),
        };
        proto.encode_to_vec()
    }

    pub fn from_protobuf(data: &[u8]) -> Result<Self, Box<dyn std::error::Error>> {
        use prost::Message;

        let proto = crate::proto::Mesh::decode(data)?;
        let mut mesh = Self::new();
        mesh.guid = proto.guid;
        mesh.name = proto.name;

        for (vkey, vdata) in proto.vertices {
            let mut attrs: std::collections::HashMap<String, f64> = std::collections::HashMap::new();
            for (k, v) in vdata.attributes {
                attrs.insert(k, v);
            }
            mesh.vertex.insert(vkey as usize, VertexData {
                x: vdata.x,
                y: vdata.y,
                z: vdata.z,
                attributes: attrs,
            });
            mesh.halfedge.entry(vkey as usize).or_insert_with(std::collections::HashMap::new);
        }

        for (fkey, fdata) in proto.faces {
            let verts: Vec<usize> = fdata.vertices.iter().map(|&v| v as usize).collect();
            mesh.face.insert(fkey as usize, verts);
            if !fdata.attributes.is_empty() {
                mesh.facedata.insert(fkey as usize, fdata.attributes);
            }
        }

        for (u, hmap) in proto.halfedges {
            let mut neighbors: std::collections::HashMap<usize, Option<usize>> = std::collections::HashMap::new();
            for (v, fkey) in hmap.neighbors {
                let fkey_opt = if fkey == u64::MAX { None } else { Some(fkey as usize) };
                neighbors.insert(v as usize, fkey_opt);
            }
            mesh.halfedge.insert(u as usize, neighbors);
        }

        for edata in proto.edge_data {
            let key = (edata.vertex1 as usize, edata.vertex2 as usize);
            mesh.edgedata.insert(key, edata.attributes);
        }

        mesh.default_vertex_attributes = proto.default_vertex_attributes;
        mesh.default_face_attributes = proto.default_face_attributes;
        mesh.default_edge_attributes = proto.default_edge_attributes;

        mesh.pointcolors = proto.pointcolors.iter().map(|c| {
            let mut color = Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8);
            color.guid = c.guid.clone();
            color.name = c.name.clone();
            color
        }).collect();

        mesh.facecolors = proto.facecolors.iter().map(|c| {
            let mut color = Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8);
            color.guid = c.guid.clone();
            color.name = c.name.clone();
            color
        }).collect();

        mesh.linecolors = proto.linecolors.iter().map(|c| {
            let mut color = Color::new(c.r as u8, c.g as u8, c.b as u8, c.a as u8);
            color.guid = c.guid.clone();
            color.name = c.name.clone();
            color
        }).collect();

        mesh.widths = proto.widths;

        if let Some(xform) = proto.xform {
            mesh.xform.guid = xform.guid;
            mesh.xform.name = xform.name;
            for (i, val) in xform.matrix.iter().enumerate() {
                if i < 16 {
                    mesh.xform.m[i] = *val;
                }
            }
        }

        // Update max_vertex and max_face
        if let Some(&max_v) = mesh.vertex.keys().max() {
            mesh.max_vertex = max_v + 1;
        }
        if let Some(&max_f) = mesh.face.keys().max() {
            mesh.max_face = max_f + 1;
        }

        Ok(mesh)
    }

    pub fn protobuf_dump(&self, filepath: &str) {
        let data = self.to_protobuf();
        std::fs::write(filepath, data).expect("Failed to write protobuf file");
    }

    pub fn protobuf_load(filepath: &str) -> Self {
        let data = std::fs::read(filepath).expect("Failed to read protobuf file");
        Self::from_protobuf(&data).expect("Failed to parse protobuf")
    }
}

#[cfg(test)]
#[path = "mesh_test.rs"]
mod mesh_test;
