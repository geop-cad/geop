//! A [`Robot`] as URDF's XML ([`Robot::to_urdf`]).

use std::fmt::Write;

use crate::{Inertial, JointType, Origin, Robot, Visual};

/// `text` fit for an XML attribute's value, between double quotes.
fn escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(c),
        }
    }
    out
}

/// `value` as short as reads back exactly: plainly, or in exponent
/// notation, which every URDF reader parses too, where that is shorter —
/// as for the tiny masses and inertias of small parts in SI units.
fn number(value: f64) -> String {
    let (plain, exponent) = (format!("{value}"), format!("{value:e}"));
    if exponent.len() < plain.len() {
        exponent
    } else {
        plain
    }
}

/// Numbers separated by spaces (see [`number`]).
fn numbers(values: &[f64]) -> String {
    let values: Vec<String> = values.iter().map(|&v| number(v)).collect();
    values.join(" ")
}

fn origin(out: &mut String, indent: &str, origin: &Origin) {
    let _ = writeln!(
        out,
        r#"{indent}<origin xyz="{}" rpy="{}"/>"#,
        numbers(&origin.xyz),
        numbers(&origin.rpy)
    );
}

fn inertial(out: &mut String, inertial: &Inertial) {
    let i = &inertial.inertia;
    out.push_str("    <inertial>\n");
    origin(
        out,
        "      ",
        &Origin {
            xyz: inertial.center,
            rpy: [0.0; 3],
        },
    );
    let _ = writeln!(out, r#"      <mass value="{}"/>"#, number(inertial.mass));
    let _ = writeln!(
        out,
        r#"      <inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}"/>"#,
        number(i[0][0]),
        number(i[0][1]),
        number(i[0][2]),
        number(i[1][1]),
        number(i[1][2]),
        number(i[2][2])
    );
    out.push_str("    </inertial>\n");
}

/// A `<visual>` or `<collision>` of the mesh `visual`.
fn shape(out: &mut String, element: &str, visual: &Visual) {
    let _ = writeln!(out, r#"    <{element} name="{}">"#, escape(&visual.name));
    origin(out, "      ", &visual.origin);
    let _ = writeln!(
        out,
        r#"      <geometry><mesh filename="{}" scale="0.001 0.001 0.001"/></geometry>"#,
        escape(&visual.mesh)
    );
    let _ = writeln!(out, "    </{element}>");
}

impl Robot {
    /// The robot as a URDF file.
    pub fn to_urdf(&self) -> String {
        let mut out = String::new();
        out.push_str("<?xml version=\"1.0\"?>\n");
        out.push_str(
            "<!-- Exported by geop. Lengths in metres, angles in radians, masses in kilograms; \
             meshes in millimetres, scaled. geop models no actuators: joint effort and \
             velocity are 0 — set them for the drives. -->\n",
        );
        let _ = writeln!(out, r#"<robot name="{}">"#, escape(&self.name));
        for link in &self.links {
            let _ = writeln!(out, r#"  <link name="{}">"#, escape(&link.name));
            if let Some(i) = &link.inertial {
                inertial(&mut out, i);
            }
            for visual in &link.visuals {
                shape(&mut out, "visual", visual);
            }
            for visual in &link.visuals {
                shape(&mut out, "collision", visual);
            }
            out.push_str("  </link>\n");
        }
        for joint in &self.joints {
            let _ = writeln!(
                out,
                r#"  <joint name="{}" type="{}">"#,
                escape(&joint.name),
                joint.kind.name()
            );
            let _ = writeln!(out, r#"    <parent link="{}"/>"#, escape(&joint.parent));
            let _ = writeln!(out, r#"    <child link="{}"/>"#, escape(&joint.child));
            origin(&mut out, "    ", &joint.origin);
            let _ = writeln!(out, r#"    <axis xyz="{}"/>"#, numbers(&joint.axis));
            match joint.kind {
                JointType::Revolute { lower, upper } | JointType::Prismatic { lower, upper } => {
                    let _ = writeln!(
                        out,
                        r#"    <limit lower="{}" upper="{}" effort="0" velocity="0"/>"#,
                        number(lower),
                        number(upper)
                    );
                }
                JointType::Continuous => {}
            }
            if let Some(mimic) = &joint.mimic {
                let _ = writeln!(
                    out,
                    r#"    <mimic joint="{}" multiplier="{}" offset="0"/>"#,
                    escape(&mimic.joint),
                    number(mimic.multiplier)
                );
            }
            out.push_str("  </joint>\n");
        }
        out.push_str("</robot>\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Joint, Link, Mimic};

    /// Names are escaped, limits written only where the joint type has
    /// them, a mimic where a coupling drives the joint.
    #[test]
    fn a_small_robot_is_written() {
        let at = |x: f64| Origin {
            xyz: [x, 0.0, 0.0],
            rpy: [0.0; 3],
        };
        let robot = Robot {
            name: "a<b>".into(),
            links: vec![
                Link {
                    name: "base".into(),
                    parts: Vec::new(),
                    inertial: None,
                    visuals: Vec::new(),
                },
                Link {
                    name: "wheel".into(),
                    parts: vec!["wheel".into()],
                    inertial: Some(Inertial {
                        mass: 0.5,
                        center: [0.0; 3],
                        inertia: [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 3.0]],
                    }),
                    visuals: vec![Visual {
                        name: "wheel".into(),
                        origin: at(0.0),
                        mesh: "meshes/wheel.stl".into(),
                    }],
                },
            ],
            joints: vec![Joint {
                name: "add_part(wheel,m1)".into(),
                kind: JointType::Continuous,
                parent: "base".into(),
                child: "wheel".into(),
                origin: at(0.25),
                axis: [0.0, 0.0, 1.0],
                mimic: Some(Mimic {
                    joint: "motor\"1".into(),
                    multiplier: -0.5,
                }),
            }],
        };
        let urdf = robot.to_urdf();
        assert!(urdf.contains(r#"<robot name="a&lt;b&gt;">"#), "{urdf}");
        assert!(urdf.contains(r#"<joint name="add_part(wheel,m1)" type="continuous">"#));
        assert!(!urdf.contains("<limit"));
        assert!(urdf.contains(r#"<origin xyz="0.25 0 0" rpy="0 0 0"/>"#));
        assert!(urdf.contains(r#"<mimic joint="motor&quot;1" multiplier="-0.5" offset="0"/>"#));
        assert!(urdf.contains(r#"<inertia ixx="1" ixy="0" ixz="0" iyy="2" iyz="0" izz="3"/>"#));
        assert_eq!(
            urdf.matches("<mesh ").count(),
            2,
            "a visual and a collision"
        );
        assert!(urdf.ends_with("</robot>\n"));
    }
}
