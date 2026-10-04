//! Purchased components as envelopes: ball bearings, T-slot aluminium
//! extrusions and a NEMA 17 stepper motor. Each stands on the `xy` plane
//! around the `z` axis — its datums `axis` and, for the face it is mounted
//! by, `side`, `end` or `face`.

use geop_core_math::geop_error::GeopResult;
use geop_ops::parameters::{Parameter, ParameterKind, Parameters};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};

use super::{
    StandardPart,
    drawing::Drawing,
    steps::{Around, Through, axis_datum, base_datum, col, cut_to, outline_plane, revolve, size},
    tables,
};
use crate::Program;

/// A deep-groove ball bearing as one solid: rings `d` bore and `D`
/// outside, `B` wide, their edges chamfered by `r`, with the gap between
/// the rings — where the balls and shields are — a shallow groove in
/// either side.
pub fn ball_bearing() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        color: Some("#b8bcc4".into()),
        values: vec![size(tables::ball_bearings())],
    };
    let (d, big, b, r) = (col("d"), col("D"), col("B"), col("r"));
    let bore = format!("{d} / 2");
    let outside = format!("{big} / 2");
    let inner = format!("{d} / 2 + 0.3 * ({big} - {d}) / 2");
    let outer = format!("{big} / 2 - 0.3 * ({big} - {d}) / 2");
    let groove = format!("0.05 * {b}");
    let far_groove = format!("{b} - {groove}");
    let zero = || "0".to_string();
    let mut drawing = Drawing::new(&program.parameters)?;
    drawing.polygon(&[
        [format!("{bore} + {r}"), zero()],
        [inner.clone(), zero()],
        [inner.clone(), groove.clone()],
        [outer.clone(), groove],
        [outer.clone(), zero()],
        [format!("{outside} - {r}"), zero()],
        [outside.clone(), r.clone()],
        [outside.clone(), format!("{b} - {r}")],
        [format!("{outside} - {r}"), b.clone()],
        [outer.clone(), b.clone()],
        [outer, far_groove.clone()],
        [inner.clone(), far_groove],
        [inner, b.clone()],
        [format!("{bore} + {r}"), b.clone()],
        [bore.clone(), format!("{b} - {r}")],
        [bore, r],
    ])?;
    revolve(
        &mut program,
        "bearing",
        "profile",
        drawing,
        Around::ZAxis,
        Combine::NewBody,
    )?;
    axis_datum(&mut program);
    base_datum(&mut program, "side");
    Ok(StandardPart {
        file: "std:ball_bearing.geop",
        title: "Deep-groove ball bearing",
        base: "side",
        program,
        threaded: Vec::new(),
    })
}

/// The profile of a 20-series B-type slot 6, in tenths of a millimetre:
/// `(along, in)` from where its centre line crosses the outside, along the
/// side and in from it — simplified to straight lines, its undercut a 45°
/// floor.
const SLOT: [[i32; 2]; 10] = [
    [-31, 0],
    [-31, 18],
    [-55, 18],
    [-55, 35],
    [-31, 59],
    [31, 59],
    [55, 35],
    [55, 18],
    [31, 18],
    [31, 0],
];

/// A 20-series T-slot extrusion `cells` of 20 x 20 long side by side
/// along `y`, `length` long up `z` — its length the parameter `length` —
/// with a slot down the middle of every 20 of its sides and a 4.2 bore,
/// for an M5 thread, down every cell.
fn tslot(cells: i32, file: &'static str, title: &'static str) -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        color: Some("#c8ccd2".into()),
        values: vec![Parameter {
            name: "length".into(),
            kind: ParameterKind::Number {
                expression: "100".into(),
                min: Some(20.0),
                max: Some(1000.0),
            },
        }],
    };
    // The envelope giving the length: a cylinder around the profile.
    let (half_w, half_h) = (100, 100 * cells);
    let radius = ((half_w * half_w + half_h * half_h) as f64).sqrt().ceil() / 10.0 + 1.0;
    let mut drawing = Drawing::new(&program.parameters)?;
    let lines = drawing.polygon(&[
        ["0".into(), "0".into()],
        [radius.to_string(), "0".into()],
        [radius.to_string(), "length".into()],
        ["0".into(), "length".into()],
    ])?;
    revolve(
        &mut program,
        "length",
        "length_profile",
        drawing,
        Around::Line(lines[3]),
        Combine::NewBody,
    )?;

    // Round the outline clockwise, from its top left corner; each side
    // `(start, direction, inward, slot centres along it)`.
    let centres: Vec<i32> = (0..cells).map(|i| 200 * i - 100 * (cells - 1)).collect();
    let down: Vec<i32> = centres.iter().rev().copied().collect();
    let sides: [([i32; 2], [i32; 2], [i32; 2], Vec<[i32; 2]>); 4] = [
        ([-half_w, half_h], [1, 0], [0, -1], vec![[0, half_h]]),
        (
            [half_w, half_h],
            [0, -1],
            [-1, 0],
            down.iter().map(|&y| [half_w, y]).collect(),
        ),
        ([half_w, -half_h], [-1, 0], [0, 1], vec![[0, -half_h]]),
        (
            [-half_w, -half_h],
            [0, 1],
            [1, 0],
            centres.iter().map(|&y| [-half_w, y]).collect(),
        ),
    ];
    let mm = |tenths: i32| format!("{}", f64::from(tenths) / 10.0);
    let mut corners = Vec::new();
    for (start, along, inward, slots) in &sides {
        corners.push([mm(start[0]), mm(start[1])]);
        for centre in slots {
            for [u, v] in SLOT {
                let at = |k: usize| centre[k] + along[k] * u + inward[k] * v;
                corners.push([mm(at(0)), mm(at(1))]);
            }
        }
    }
    let mut outline = Drawing::new(&program.parameters)?;
    outline.polygon(&corners)?;
    for y in &centres {
        let centre = outline.point("0", mm(*y))?;
        outline.circle(centre, "4.2")?;
    }
    cut_to(&mut program, "extrusion", "outline", outline, "revolve(length)", Through::Up)?;
    axis_datum(&mut program);
    base_datum(&mut program, "end");
    Ok(StandardPart {
        file,
        title,
        base: "end",
        program,
        threaded: Vec::new(),
    })
}

pub fn tslot_2020() -> GeopResult<StandardPart> {
    tslot(
        1,
        "std:tslot_2020.geop",
        "T-slot aluminium extrusion 20x20, B-type slot 6",
    )
}

pub fn tslot_2040() -> GeopResult<StandardPart> {
    tslot(
        2,
        "std:tslot_2040.geop",
        "T-slot aluminium extrusion 20x40, B-type slot 6",
    )
}

/// A NEMA 17 stepper motor as an envelope: a body 42.3 square with its
/// corners chamfered, `L` long down from its mounting face; on the face a
/// pilot boss 22 across and 2 high, a shaft 5 across standing 24 out, and
/// four M3 holes 4.5 deep on a 31 square.
pub fn nema17() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        color: Some("#3a3d42".into()),
        values: vec![size(tables::nema17())],
    };
    let below = format!("-{}", col("L"));
    let mut drawing = Drawing::new(&program.parameters)?;
    let lines = drawing.polygon(&[
        ["0".into(), below.clone()],
        ["30".into(), below],
        ["30".into(), "0".into()],
        ["11".into(), "0".into()],
        ["11".into(), "2".into()],
        ["2.5".into(), "2".into()],
        ["2.5".into(), "24".into()],
        ["0".into(), "24".into()],
    ])?;
    revolve(
        &mut program,
        "envelope",
        "profile",
        drawing,
        Around::Line(lines[7]),
        Combine::NewBody,
    )?;
    let mut outline = Drawing::new(&program.parameters)?;
    let (side, cut) = ("21.15", "17.15");
    let neg = |s: &str| format!("-{s}");
    outline.polygon(&[
        [side.into(), neg(cut)],
        [side.into(), cut.into()],
        [cut.into(), side.into()],
        [neg(cut), side.into()],
        [neg(side), cut.into()],
        [neg(side), neg(cut)],
        [neg(cut), neg(side)],
        [cut.into(), neg(side)],
    ])?;
    cut_to(&mut program, "body", "square", outline, "revolve(envelope)", Through::Both)?;
    let mut target = "extrude(body)".to_string();
    for (i, [x, y]) in [["15.5", "15.5"], ["-15.5", "15.5"], ["-15.5", "-15.5"], ["15.5", "-15.5"]]
        .into_iter()
        .enumerate()
    {
        let mut hole = Drawing::new(&program.parameters)?;
        let centre = hole.point(x, y)?;
        hole.circle(centre, "3")?;
        let (sketch, id) = (format!("hole{}_sketch", i + 1), format!("hole{}", i + 1));
        program.push(&sketch, hole.on(outline_plane())?);
        program.push(
            &id,
            ExtrudeArgs {
                sketch,
                extent: Extents {
                    reversed: true,
                    ..Extents::blind(4.5)
                },
                face: false,
                combine: Combine::Difference { target },
            },
        );
        target = format!("extrude({id})");
    }
    axis_datum(&mut program);
    base_datum(&mut program, "face");
    Ok(StandardPart {
        file: "std:nema17_stepper.geop",
        title: "NEMA 17 stepper motor",
        base: "face",
        program,
        threaded: Vec::new(),
    })
}
