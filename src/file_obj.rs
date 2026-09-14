use crate::{Mesh, Point, Polyline};
use std::io;

pub fn write_file_obj_to_string(mesh: &Mesh) -> String {
    let (vertices, faces) = mesh.to_vertices_and_faces();
    let mut s = String::new();
    for p in vertices.iter() {
        s.push_str(&format!("v {} {} {}\n", p[0], p[1], p[2]));
    }
    for face in faces.iter() {
        if face.len() < 3 {
            continue;
        }
        s.push('f');
        for i in face.iter() {
            s.push_str(&format!(" {}", i + 1));
        }
        s.push('\n');
    }
    s
}

pub fn write_file_obj(mesh: &Mesh, filepath: &str) -> io::Result<()> {
    std::fs::write(filepath, write_file_obj_to_string(mesh))
}

pub fn read_file_obj_from_str(content: &str) -> Mesh {
    let mut verts: Vec<Point> = Vec::new();
    let mut faces: Vec<Vec<usize>> = Vec::new();
    for line in content.lines() {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line.starts_with("v ") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 4 {
                continue;
            }
            let x = parts[1].parse::<f64>();
            let y = parts[2].parse::<f64>();
            let z = parts[3].parse::<f64>();
            if x.is_err() || y.is_err() || z.is_err() {
                continue;
            }
            verts.push(Point::new(x.unwrap(), y.unwrap(), z.unwrap()));
        } else if line.starts_with("f ") {
            let mut face: Vec<usize> = Vec::new();
            for tok in line.split_whitespace().skip(1) {
                let idx: i64 = tok.split('/').next().unwrap_or("").parse().unwrap_or(0);
                if idx == 0 {
                    continue;
                }
                let vidx = if idx > 0 {
                    idx - 1
                } else {
                    verts.len() as i64 + idx
                };
                face.push(vidx as usize);
            }
            if face.len() >= 3 {
                faces.push(face);
            }
        }
    }
    Mesh::from_vertices_and_faces(verts, faces)
}

pub fn read_file_obj(filepath: &str) -> io::Result<Mesh> {
    let content = std::fs::read_to_string(filepath)?;
    Ok(read_file_obj_from_str(&content))
}

pub fn read_file_obj_polylines(filepath: &str) -> io::Result<Vec<Polyline>> {
    let content = std::fs::read_to_string(filepath)?;
    let mut verts: Vec<Point> = Vec::new();
    let mut polylines: Vec<Polyline> = Vec::new();
    let mut curv: Vec<i64> = Vec::new();
    let mut in_curv = false;
    for line in content.lines() {
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if line.starts_with("v ") {
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 4 {
                continue;
            }
            let x = parts[1].parse::<f64>();
            let y = parts[2].parse::<f64>();
            let z = parts[3].parse::<f64>();
            if x.is_err() || y.is_err() || z.is_err() {
                continue;
            }
            verts.push(Point::new(x.unwrap(), y.unwrap(), z.unwrap()));
        } else if line.starts_with("curv ") {
            curv.clear();
            for tok in line.split_whitespace().skip(3) {
                let Ok(idx) = tok.parse::<i64>() else { break };
                curv.push(idx);
            }
            in_curv = true;
        } else if line.starts_with("end") && in_curv {
            let mut pts: Vec<Point> = Vec::new();
            for idx in curv.iter() {
                if *idx > 0 && *idx as usize <= verts.len() {
                    pts.push(verts[(*idx - 1) as usize].clone());
                }
            }
            if pts.len() >= 2 {
                polylines.push(Polyline::new(pts));
            }
            in_curv = false;
        }
    }
    Ok(polylines)
}
