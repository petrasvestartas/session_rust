use session_rust::Mesh;
use session_rust::Point;
fn main() {
    let pts = vec![
        Point::new(0., 0., 0.),
        Point::new(1., 0., 0.),
        Point::new(1., 1., 0.),
        Point::new(0., 1., 0.),
        Point::new(2., 0.5, 0.),
    ];
    let mesh = Mesh::from_vertices_and_faces(pts, vec![vec![0, 1, 2, 3], vec![1, 4, 2]]);
    println!("faces(): {:?}", mesh.faces());
    println!("face_faces(0): {:?}", mesh.face_faces(0));
    println!("vertex_edges(1): {:?}", mesh.vertex_edges(1));
    println!("vertex_faces(1): {:?}", mesh.vertex_faces(1));
    println!("edge_edges(1,2): {:?}", mesh.edge_edges(1, 2));
    println!("vertex_vertices(1): {:?}", mesh.vertex_vertices(1));
    println!("edge_faces(1,2): {:?}", mesh.edge_faces(1, 2));
    println!("halfedge empty? {}", mesh.halfedge.is_empty());
}
