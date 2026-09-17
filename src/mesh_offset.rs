use crate::matrix::Matrix;
use crate::mesh::Mesh;
use crate::plane::Plane;
use crate::point::Point;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::collections::HashSet;

/// Thick shell of a mesh: original faces, offset faces, quads on naked edges.
pub struct MeshOffset;

/// The shell as three meshes: top, bottom and sides.
pub struct MeshOffsetLayers {
    pub top: Mesh,    // Offset faces.
    pub bottom: Mesh, // Reversed original faces.
    pub sides: Mesh,  // One quad per naked edge.
}

/// Least-squares point on the planes, fallback fills any free direction.
fn intersect_planes(planes: &[Plane], fallback: &Point) -> Point {
    if planes.is_empty() {
        return fallback.clone();
    }

    if planes.len() == 1 {
        let plane = &planes[0];
        let t = -plane.d()
            - (plane.a() * fallback[0] + plane.b() * fallback[1] + plane.c() * fallback[2]);

        return Point::new(
            fallback[0] + t * plane.a(),
            fallback[1] + t * plane.b(),
            fallback[2] + t * plane.c(),
        );
    }

    let eps = 1e-8;
    let mut lhs = Matrix::new(3, 3);
    let mut rhs = Matrix::new(3, 1);

    for plane in planes {
        let row = [plane.a(), plane.b(), plane.c()];

        for i in 0..3 {
            for j in 0..3 {
                lhs[(i, j)] += row[i] * row[j];
            }

            rhs[(i, 0)] -= row[i] * plane.d();
        }
    }

    for i in 0..3 {
        lhs[(i, i)] += eps;
        rhs[(i, 0)] += eps * fallback[i];
    }

    let Some(solution) = lhs.solve(&rhs) else {
        return fallback.clone();
    };

    Point::new(solution[(0, 0)], solution[(1, 0)], solution[(2, 0)])
}

/// Naked edges wound the way their face walks them.
fn boundary_edges(mesh: &Mesh) -> Vec<(usize, usize)> {
    let mut directed: HashSet<(usize, usize)> = HashSet::new();

    for fkey in mesh.faces() {
        let vertices = &mesh.face[&fkey];

        for i in 0..vertices.len() {
            directed.insert((vertices[i], vertices[(i + 1) % vertices.len()]));
        }
    }

    let mut edges = Vec::new();

    for (u, v) in mesh.naked_edges(true) {
        if directed.contains(&(u, v)) {
            edges.push((u, v));
        } else {
            edges.push((v, u));
        }
    }

    edges
}

impl MeshOffset {
    /// One closed mesh: reversed bottom, offset top, one quad per naked edge.
    pub fn from_mesh(mesh: &Mesh, distance: f64) -> Mesh {
        let planes = MeshOffset::offset_planes(mesh, distance);
        let offsets = MeshOffset::offset_vertices(mesh, &planes);
        let mut result = Mesh::new();
        let mut bottom: HashMap<usize, usize> = HashMap::new();
        let mut top: HashMap<usize, usize> = HashMap::new();

        for vkey in mesh.vertices() {
            bottom.insert(
                vkey,
                result.add_vertex(mesh.vertex_point(vkey).unwrap(), None),
            );
            top.insert(vkey, result.add_vertex(offsets[&vkey].clone(), None));
        }

        for fkey in mesh.faces() {
            let vertices = mesh.face_vertices(fkey).unwrap();
            let mut bottom_face = Vec::new();
            let mut top_face = Vec::new();

            for vkey in vertices {
                bottom_face.push(bottom[vkey]);
                top_face.push(top[vkey]);
            }

            bottom_face.reverse();
            result.add_face(bottom_face, None);
            result.add_face(top_face, None);
        }

        for (u, v) in boundary_edges(mesh) {
            result.add_face(vec![bottom[&u], bottom[&v], top[&v], top[&u]], None);
        }

        result
    }

    /// The same shell as three meshes: top, bottom and sides.
    pub fn from_mesh_layers(mesh: &Mesh, distance: f64) -> MeshOffsetLayers {
        let planes = MeshOffset::offset_planes(mesh, distance);
        let offsets = MeshOffset::offset_vertices(mesh, &planes);
        let mut layers = MeshOffsetLayers {
            top: Mesh::new(),
            bottom: Mesh::new(),
            sides: Mesh::new(),
        };
        let mut bottom: HashMap<usize, usize> = HashMap::new();
        let mut top: HashMap<usize, usize> = HashMap::new();

        for vkey in mesh.vertices() {
            bottom.insert(
                vkey,
                layers
                    .bottom
                    .add_vertex(mesh.vertex_point(vkey).unwrap(), None),
            );
            top.insert(vkey, layers.top.add_vertex(offsets[&vkey].clone(), None));
        }

        for fkey in mesh.faces() {
            let vertices = mesh.face_vertices(fkey).unwrap();
            let mut bottom_face = Vec::new();
            let mut top_face = Vec::new();

            for vkey in vertices {
                bottom_face.push(bottom[vkey]);
                top_face.push(top[vkey]);
            }

            bottom_face.reverse();
            layers.bottom.add_face(bottom_face, None);
            layers.top.add_face(top_face, None);
        }

        let mut side_bottom: HashMap<usize, usize> = HashMap::new();
        let mut side_top: HashMap<usize, usize> = HashMap::new();

        for (u, v) in boundary_edges(mesh) {
            for vkey in [u, v] {
                if let Entry::Vacant(entry) = side_bottom.entry(vkey) {
                    entry.insert(
                        layers
                            .sides
                            .add_vertex(mesh.vertex_point(vkey).unwrap(), None),
                    );
                }

                if let Entry::Vacant(entry) = side_top.entry(vkey) {
                    entry.insert(layers.sides.add_vertex(offsets[&vkey].clone(), None));
                }
            }

            layers.sides.add_face(
                vec![side_bottom[&u], side_bottom[&v], side_top[&v], side_top[&u]],
                None,
            );
        }

        layers
    }

    /// Plane of each face translated by distance along its normal, by face key.
    pub fn offset_planes(mesh: &Mesh, distance: f64) -> HashMap<usize, Plane> {
        let mut planes = HashMap::new();

        for fkey in mesh.faces() {
            let Some(centroid) = mesh.face_centroid(fkey) else {
                continue;
            };
            let Some(normal) = mesh.face_normal(fkey) else {
                continue;
            };
            planes.insert(
                fkey,
                Plane::from_point_normal(&centroid + &normal * distance, normal, None),
            );
        }

        planes
    }

    /// Offsets position of each vertex: least-squares meet of its face planes, by vertex key.
    pub fn offset_vertices(mesh: &Mesh, planes: &HashMap<usize, Plane>) -> HashMap<usize, Point> {
        let mut vertex_faces: HashMap<usize, Vec<usize>> = HashMap::new();

        for fkey in mesh.faces() {
            for vkey in &mesh.face[&fkey] {
                vertex_faces.entry(*vkey).or_default().push(fkey);
            }
        }

        let mut result = HashMap::new();

        for vkey in mesh.vertices() {
            let Some(point) = mesh.vertex_point(vkey) else {
                continue;
            };
            let mut adjacent = Vec::new();

            for fkey in vertex_faces.entry(vkey).or_default().iter() {
                if let Some(plane) = planes.get(fkey) {
                    adjacent.push(plane.clone());
                }
            }

            result.insert(vkey, intersect_planes(&adjacent, &point));
        }

        result
    }
}
