//! Drawings composed and written as DXF and SVG, then read back.

use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
use geop_ops::Part;
use geop_ops_extrude_revolve::shapes::{cube::cube_solid, cylinder::revolved_cylinder};

use crate::{DrawingArgs, ViewKind, compose, to_dxf, to_svg};

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
}

/// A DXF entity read back: its type, layer and text.
pub(crate) type Entity = (String, String, String);

/// A DXF file read back: every entity of its `ENTITIES` section.
pub(crate) fn read_dxf(dxf: &str) -> Vec<Entity> {
    let lines: Vec<&str> = dxf.lines().collect();
    assert!(lines.len() % 2 == 0, "a DXF file is pairs of lines");
    let pairs: Vec<(i32, &str)> = lines
        .chunks(2)
        .map(|p| (p[0].trim().parse().unwrap(), p[1]))
        .collect();
    assert_eq!(pairs.last().unwrap(), &(0, "EOF"));
    let start = pairs
        .iter()
        .position(|&(c, v)| c == 2 && v == "ENTITIES")
        .unwrap();
    let mut entities: Vec<Entity> = Vec::new();
    for &(code, value) in &pairs[start + 1..] {
        match code {
            0 if value == "ENDSEC" => break,
            0 => entities.push((value.to_string(), String::new(), String::new())),
            8 => entities.last_mut().unwrap().1 = value.to_string(),
            1 => entities.last_mut().unwrap().2 = value.to_string(),
            _ => {}
        }
    }
    entities
}

pub(crate) fn dxf_count(entities: &[Entity], kind: &str, layer: &str) -> usize {
    entities
        .iter()
        .filter(|(k, l, _)| k == kind && l == layer)
        .count()
}

/// The SVG group of `layer`.
fn svg_group<'a>(svg: &'a str, layer: &str) -> &'a str {
    let open = format!(r#"<g class="{layer}""#);
    let start = svg.find(&open).unwrap();
    let end = start + svg[start..].find("</g>").unwrap();
    &svg[start..end]
}

/// The front view of a box alone, written as DXF and SVG and read back:
/// four visible lines, its width and height dimensioned, and a title block.
#[test]
fn box_drawing_reads_back() {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "box", v(0.0, 0.0, 0.0), v(20.0, 10.0, 30.0)).unwrap();
    let args = DrawingArgs {
        views: vec![ViewKind::Front],
        name: "Block".into(),
        material: "Aluminium 6061".into(),
        ..Default::default()
    };
    let sheet = compose(&part, &args, "2026-10-04").unwrap();

    let entities = read_dxf(&to_dxf(&sheet));
    assert_eq!(dxf_count(&entities, "LINE", "VISIBLE"), 4);
    assert_eq!(dxf_count(&entities, "LINE", "HIDDEN"), 0);
    assert_eq!(dxf_count(&entities, "SOLID", "DIMENSIONS"), 4);
    let mut values: Vec<&str> = entities
        .iter()
        .filter(|(k, l, _)| k == "TEXT" && l == "DIMENSIONS")
        .map(|(_, _, t)| t.as_str())
        .collect();
    values.sort();
    assert_eq!(values, ["20", "30"]);
    for text in ["Block", "Aluminium 6061", "2026-10-04", "THIRD ANGLE", "mm"] {
        assert!(
            entities.iter().any(|(k, _, t)| k == "TEXT" && t == text),
            "{text} is missing from the title block"
        );
    }
    // On A3, the 30 high view fits at 5:1.
    assert!(entities.iter().any(|(k, _, t)| k == "TEXT" && t == "5:1"));

    let svg = to_svg(&sheet);
    assert!(svg.starts_with("<svg") && svg.trim_end().ends_with("</svg>"));
    assert_eq!(svg_group(&svg, "VISIBLE").matches("<line").count(), 4);
    assert_eq!(svg_group(&svg, "HIDDEN").matches("<line").count(), 0);
    assert_eq!(svg_group(&svg, "DIMENSIONS").matches("<polygon").count(), 4);
}

/// A cylinder seen from above is a circle with a centre mark; from the
/// front, a rectangle of two edges and two silhouettes.
#[test]
fn cylinder_drawing_has_circle_and_centre_mark() {
    let mut part = Part::<S>::new();
    revolved_cylinder(
        &mut part,
        "c",
        v(0.0, 0.0, 0.0),
        S::from_f64(5.0),
        S::from_f64(10.0),
    )
    .unwrap();
    let args = DrawingArgs {
        views: vec![ViewKind::Front, ViewKind::Top],
        ..Default::default()
    };
    let sheet = compose(&part, &args, "today").unwrap();
    let entities = read_dxf(&to_dxf(&sheet));
    assert_eq!(dxf_count(&entities, "CIRCLE", "VISIBLE"), 1, "{entities:?}");
    assert_eq!(dxf_count(&entities, "LINE", "CENTER"), 2);
    assert_eq!(dxf_count(&entities, "LINE", "VISIBLE"), 4, "{entities:?}");
}
