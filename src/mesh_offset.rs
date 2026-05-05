use crate::mesh::Mesh;
use crate::point::Point;
use std::collections::HashMap;

pub struct MeshOffset;

pub struct MeshOffsetLayers {
    pub top: Mesh,
    pub bottom: Mesh,
    pub sides: Mesh,
}

fn offset_planes(mesh: &Mesh, distance: f64) -> HashMap<usize, (f64, f64, f64, f64)> {
    let mut planes = HashMap::new();
    for fk in mesh.faces() {
        let c = match mesh.face_centroid(fk) { Some(p) => p, None => continue };
        let n = match mesh.face_normal(fk) { Some(v) => v, None => continue };
        let (a, b, c_) = (n[0], n[1], n[2]);
        let ox = c[0] + distance * a;
        let oy = c[1] + distance * b;
        let oz = c[2] + distance * c_;
        let d = -(a * ox + b * oy + c_ * oz);
        planes.insert(fk, (a, b, c_, d));
    }
    planes
}

fn intersect_planes(planes: &[(f64, f64, f64, f64)], fallback: &Point) -> Point {
    let n = planes.len();
    if n == 0 {
        return fallback.clone();
    }
    if n == 1 {
        let (a, b, c, d) = planes[0];
        let d_rhs = -d;
        let t = d_rhs - (a * fallback[0] + b * fallback[1] + c * fallback[2]);
        return Point::new(fallback[0] + t * a, fallback[1] + t * b, fallback[2] + t * c);
    }
    const EPS: f64 = 1e-8;
    let mut mat = [[0.0f64; 3]; 3];
    let mut rhs = [0.0f64; 3];
    for &(a, b, c, d) in planes {
        let d_rhs = -d;
        mat[0][0] += a * a;
        mat[0][1] += a * b;
        mat[0][2] += a * c;
        mat[1][0] += b * a;
        mat[1][1] += b * b;
        mat[1][2] += b * c;
        mat[2][0] += c * a;
        mat[2][1] += c * b;
        mat[2][2] += c * c;
        rhs[0] += a * d_rhs;
        rhs[1] += b * d_rhs;
        rhs[2] += c * d_rhs;
    }
    mat[0][0] += EPS;
    mat[1][1] += EPS;
    mat[2][2] += EPS;
    rhs[0] += EPS * fallback[0];
    rhs[1] += EPS * fallback[1];
    rhs[2] += EPS * fallback[2];
    let det = mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
        - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
    if det.abs() < 1e-20 {
        return fallback.clone();
    }
    let inv = 1.0 / det;
    let x = inv * (rhs[0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1])
        - mat[0][1] * (rhs[1] * mat[2][2] - mat[1][2] * rhs[2])
        + mat[0][2] * (rhs[1] * mat[2][1] - mat[1][1] * rhs[2]));
    let y = inv * (mat[0][0] * (rhs[1] * mat[2][2] - mat[1][2] * rhs[2])
        - rhs[0] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * rhs[2] - rhs[1] * mat[2][0]));
    let z = inv * (mat[0][0] * (mat[1][1] * rhs[2] - rhs[1] * mat[2][1])
        - mat[0][1] * (mat[1][0] * rhs[2] - rhs[1] * mat[2][0])
        + rhs[0] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]));
    Point::new(x, y, z)
}

fn offset_vertices(mesh: &Mesh, planes: &HashMap<usize, (f64, f64, f64, f64)>) -> HashMap<usize, Point> {
    let mut result = HashMap::new();
    for vk in mesh.vertices() {
        let vp = match mesh.vertex_point(vk) { Some(p) => p, None => continue };
        let fkeys = match mesh.vertex_faces(vk) { Some(f) => f, None => { result.insert(vk, vp); continue; } };
        if fkeys.is_empty() {
            result.insert(vk, vp);
            continue;
        }
        let adj: Vec<(f64, f64, f64, f64)> = fkeys.iter()
            .filter_map(|fk| planes.get(fk).copied())
            .collect();
        result.insert(vk, intersect_planes(&adj, &vp));
    }
    result
}

impl MeshOffset {
    pub fn from_mesh(mesh: &Mesh, distance: f64) -> Mesh {
        let planes = offset_planes(mesh, distance);
        let off_verts = offset_vertices(mesh, &planes);

        let mut result = Mesh::new();
        let mut bot_vmap: HashMap<usize, usize> = HashMap::new();
        let mut top_vmap: HashMap<usize, usize> = HashMap::new();
        for vk in mesh.vertices() {
            let vp = mesh.vertex_point(vk).unwrap();
            bot_vmap.insert(vk, result.add_vertex(vp, None));
            top_vmap.insert(vk, result.add_vertex(off_verts[&vk].clone(), None));
        }

        for fk in mesh.faces() {
            let fv = mesh.face_vertices(fk).unwrap();
            let bkeys: Vec<usize> = fv.iter().map(|v| bot_vmap[v]).collect();
            let tkeys: Vec<usize> = fv.iter().map(|v| top_vmap[v]).collect();
            let mut brev = bkeys.clone();
            brev.reverse();
            result.add_face(brev, None);
            result.add_face(tkeys, None);
        }

        for (u, v) in mesh.naked_edges(true) {
            result.add_face(vec![bot_vmap[&u], bot_vmap[&v], top_vmap[&v], top_vmap[&u]], None);
        }

        result
    }

    pub fn from_mesh_layers(mesh: &Mesh, distance: f64) -> MeshOffsetLayers {
        let planes = offset_planes(mesh, distance);
        let off_verts = offset_vertices(mesh, &planes);

        let mut bot = Mesh::new();
        let mut top = Mesh::new();
        let mut sides = Mesh::new();
        let mut bot_vmap: HashMap<usize, usize> = HashMap::new();
        let mut top_vmap: HashMap<usize, usize> = HashMap::new();
        for vk in mesh.vertices() {
            let vp = mesh.vertex_point(vk).unwrap();
            bot_vmap.insert(vk, bot.add_vertex(vp, None));
            top_vmap.insert(vk, top.add_vertex(off_verts[&vk].clone(), None));
        }

        for fk in mesh.faces() {
            let fv = mesh.face_vertices(fk).unwrap();
            let bkeys: Vec<usize> = fv.iter().map(|v| bot_vmap[v]).collect();
            let tkeys: Vec<usize> = fv.iter().map(|v| top_vmap[v]).collect();
            let mut brev = bkeys.clone();
            brev.reverse();
            bot.add_face(brev, None);
            top.add_face(tkeys, None);
        }

        let mut s_bot: HashMap<usize, usize> = HashMap::new();
        let mut s_top: HashMap<usize, usize> = HashMap::new();
        for (u, v) in mesh.naked_edges(true) {
            if !s_bot.contains_key(&u) {
                s_bot.insert(u, sides.add_vertex(mesh.vertex_point(u).unwrap(), None));
            }
            if !s_bot.contains_key(&v) {
                s_bot.insert(v, sides.add_vertex(mesh.vertex_point(v).unwrap(), None));
            }
            if !s_top.contains_key(&u) {
                s_top.insert(u, sides.add_vertex(off_verts[&u].clone(), None));
            }
            if !s_top.contains_key(&v) {
                s_top.insert(v, sides.add_vertex(off_verts[&v].clone(), None));
            }
            sides.add_face(vec![s_bot[&u], s_bot[&v], s_top[&v], s_top[&u]], None);
        }

        MeshOffsetLayers { top, bottom: bot, sides }
    }
}
