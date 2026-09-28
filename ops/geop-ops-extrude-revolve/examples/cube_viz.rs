use geop_core_math::{
    primitives::{Color10, PrimitiveScene},
    scalars::{ScalInF64, Scalar},
    vector::Vector3,
};
use geop_ops::Part;
use geop_ops_extrude_revolve::cube::cube_solid;

fn main() {
    let mut part = Part::<ScalInF64>::new();
    let min = Vector3::from_array([ScalInF64::ZERO; 3]);
    let max = Vector3::from_array([ScalInF64::ONE; 3]);
    cube_solid(&mut part, "cube", min, max).expect("build cube");
    let model = part.topology();

    let mut scene = PrimitiveScene::<ScalInF64>::new();

    for face in model.faces.values() {
        let (u_min, u_max) = face.surface.domain_u();
        let (v_min, v_max) = face.surface.domain_v();
        scene
            .add_surface(&face.surface, Color10::Green, u_min, u_max, v_min, v_max, 4)
            .expect("render face");
        scene
            .add_surface_wireframe(&face.surface, Color10::Gray, u_min, u_max, v_min, v_max, 4)
            .expect("render face wireframe");
    }

    for edge in model.edges.values() {
        let (t_min, t_max) = edge.curve.domain();
        scene
            .add_curve(&edge.curve, t_min, t_max, Color10::Blue, 8)
            .expect("render edge");
    }

    for vertex in model.vertices.values() {
        scene.add_point(vertex.point, Color10::Red);
    }

    scene.save_to_file("cube.html").expect("save scene");
    println!("Wrote cube.html");
}
