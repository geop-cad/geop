//! Purchased components as envelopes: ball bearings, T-slot aluminium
//! extrusions, a NEMA 17 stepper motor, miniature linear guides and hobby
//! servos. Each stands on the `xy` plane around the `z` axis, or runs along
//! it — its datums `axis` and, for the face it is mounted by, `side`,
//! `end`, `face`, `base`, `top` or `mount`.

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
};
use geop_ops::parameters::Material;
use geop_ops::parameters::{Parameter, ParameterKind, Parameters};
use geop_ops::{EntityRef, ORIGIN};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};

use super::{
    StandardPart,
    drawing::Drawing,
    steps::{
        Around, axis_datum, col, extrude, offset_datum, outline_plane, plane_datum, revolve, size,
    },
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
        material: Some(Material {
            name: "Steel".into(),
            density: 7850.0,
        }),
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
    plane_datum(&mut program, "side", "0");
    Ok(StandardPart {
        file: "std:ball_bearing.geop",
        title: "Deep-groove ball bearing",
        designation: "Ball bearing",
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
/// for an M5 thread, down every cell: its profile extruded `length`.
fn tslot(
    cells: i32,
    file: &'static str,
    title: &'static str,
    designation: &'static str,
) -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: Some(Material {
            name: "Aluminium 6061".into(),
            density: 2700.0,
        }),
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
    let (half_w, half_h) = (100, 100 * cells);
    // Round the outline clockwise, from its top left corner; each side
    // `(start, direction, slot centres along it)`, in from it to its right.
    let centres: Vec<i32> = (0..cells).map(|i| 200 * i - 100 * (cells - 1)).collect();
    let down: Vec<i32> = centres.iter().rev().copied().collect();
    let sides = [
        ([-half_w, half_h], [1, 0], vec![[0, half_h]]),
        (
            [half_w, half_h],
            [0, -1],
            down.iter().map(|&y| [half_w, y]).collect(),
        ),
        ([half_w, -half_h], [-1, 0], vec![[0, -half_h]]),
        (
            [-half_w, -half_h],
            [0, 1],
            centres.iter().map(|&y| [-half_w, y]).collect(),
        ),
    ];
    let mm = |tenths: i32| format!("{}", f64::from(tenths) / 10.0);
    let mut corners = Vec::new();
    for (start, along, slots) in &sides {
        let inward = [along[1], -along[0]];
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
    extrude(
        &mut program,
        "extrusion",
        "outline",
        (outline, outline_plane()),
        Extents::blind("length"),
        Combine::NewBody,
    )?;
    axis_datum(&mut program);
    plane_datum(&mut program, "end", "0");
    Ok(StandardPart {
        file,
        title,
        designation,
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
        "T-slot 2020",
    )
}

pub fn tslot_2040() -> GeopResult<StandardPart> {
    tslot(
        2,
        "std:tslot_2040.geop",
        "T-slot aluminium extrusion 20x40, B-type slot 6",
        "T-slot 2040",
    )
}

/// A NEMA 17 stepper motor as an envelope: a body 42.3 square with its
/// corners chamfered, `L` long down from its mounting face; on the face a
/// pilot boss 22 across and 2 high, a shaft 5 across standing 24 out, and
/// four M3 holes 4.5 deep on a 31 square. The body is its outline extruded
/// `L` down, the boss and the shaft one profile turned on its face.
pub fn nema17() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: Some(Material {
            name: "Steel".into(),
            density: 7850.0,
        }),
        color: Some("#3a3d42".into()),
        values: vec![size(tables::nema17())],
    };
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
    extrude(
        &mut program,
        "body",
        "square",
        (outline, outline_plane()),
        Extents {
            reversed: true,
            ..Extents::blind(col("L"))
        },
        Combine::NewBody,
    )?;
    let mut drawing = Drawing::new(&program.parameters)?;
    let lines = drawing.polygon(&[
        ["0".into(), "0".into()],
        ["11".into(), "0".into()],
        ["11".into(), "2".into()],
        ["2.5".into(), "2".into()],
        ["2.5".into(), "24".into()],
        ["0".into(), "24".into()],
    ])?;
    revolve(
        &mut program,
        "top",
        "profile",
        drawing,
        Around::Line(lines[5]),
        Combine::Union {
            target: "extrude(body)".into(),
        },
    )?;
    let mut target = "revolve(top)".to_string();
    for (i, [x, y]) in [
        ["15.5", "15.5"],
        ["-15.5", "15.5"],
        ["-15.5", "-15.5"],
        ["15.5", "-15.5"],
    ]
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
    plane_datum(&mut program, "face", "0");
    Ok(StandardPart {
        file: "std:nema17_stepper.geop",
        title: "NEMA 17 stepper motor",
        designation: "NEMA 17 stepper",
        base: "face",
        program,
        threaded: Vec::new(),
    })
}

/// The `xz` plane: what a part running along `z` sits on, beside its axis.
fn xz_plane() -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Y))
}

fn steel() -> Option<Material> {
    Some(Material {
        name: "Steel".into(),
        density: 7850.0,
    })
}

/// A miniature linear guide rail, MGN9 or MGN12 by its size, `length`
/// long up `z` from the `xy` plane, standing on the `xz` plane — its datum
/// `base` — centred on `x`: a `W` x `H` section with a groove down either
/// side, where the carriage's balls run. Its datum `axis` is the `z` axis,
/// along the middle of its bottom: a [`linear_carriage`] of the same size
/// slides on it by a slider joint between the two `axis` datums. The
/// mounting holes are not modelled.
pub fn linear_rail() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: steel(),
        color: Some("#a9aeb6".into()),
        values: vec![
            size(tables::linear_rails()),
            Parameter {
                name: "length".into(),
                kind: ParameterKind::Number {
                    expression: "200".into(),
                    min: Some(20.0),
                    max: Some(1000.0),
                },
            },
        ],
    };
    let (w, h) = (col("W"), col("H"));
    let side = format!("{w} / 2");
    let groove = format!("{w} / 2 - 0.8");
    let (low, high) = (format!("{h} / 2"), format!("{h} / 2 + 1.2"));
    let neg = |s: &str| format!("-({s})");
    let zero = || "0".to_string();
    let mut outline = Drawing::new(&program.parameters)?;
    outline.polygon(&[
        [neg(&side), zero()],
        [side.clone(), zero()],
        [side.clone(), low.clone()],
        [groove.clone(), low.clone()],
        [groove.clone(), high.clone()],
        [side.clone(), high.clone()],
        [side.clone(), h.clone()],
        [neg(&side), h.clone()],
        [neg(&side), high.clone()],
        [neg(&groove), high],
        [neg(&groove), low.clone()],
        [neg(&side), low],
    ])?;
    extrude(
        &mut program,
        "rail",
        "section",
        (outline, outline_plane()),
        Extents::blind("length"),
        Combine::NewBody,
    )?;
    axis_datum(&mut program);
    offset_datum(&mut program, "base", xz_plane(), "0");
    offset_datum(&mut program, "top", xz_plane(), &h);
    plane_datum(&mut program, "end", "0");
    Ok(StandardPart {
        file: "std:linear_rail.geop",
        title: "Miniature linear guide rail, MGN series",
        designation: "Linear rail",
        base: "base",
        program,
        threaded: Vec::new(),
    })
}

/// The carriage of a miniature linear guide as an envelope: a block `W`
/// wide and `L` long, from `H1` above the rail's bottom to `H` — its top,
/// the datum `top` — centred on the origin along `z`, with a channel over
/// the rail and four holes `M` across and `M` + 1 deep in its top, `B`
/// apart across and `C` along. It is drawn where it sits on its rail at
/// `z = 0`: its datum `axis`, the `z` axis, is the rail's, and a slider
/// joint between the two moves it along.
pub fn linear_carriage() -> GeopResult<StandardPart> {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: steel(),
        color: Some("#8e949c".into()),
        values: vec![size(tables::linear_carriages())],
    };
    let (w, h, h1, wr, hr) = (col("W"), col("H"), col("H1"), col("WR"), col("HR"));
    let side = format!("{w} / 2");
    let channel = format!("{wr} / 2 + 0.5");
    let roof = format!("{hr} + 0.3");
    let neg = |s: &str| format!("-({s})");
    let mut outline = Drawing::new(&program.parameters)?;
    outline.polygon(&[
        [neg(&side), h1.clone()],
        [neg(&channel), h1.clone()],
        [neg(&channel), roof.clone()],
        [channel.clone(), roof],
        [channel, h1.clone()],
        [side.clone(), h1],
        [side.clone(), h.clone()],
        [neg(&side), h.clone()],
    ])?;
    extrude(
        &mut program,
        "block",
        "section",
        (outline, outline_plane()),
        Extents {
            symmetric: true,
            ..Extents::blind(col("L"))
        },
        Combine::NewBody,
    )?;
    // The holes, drawn on the top — whose sketch runs along `x` and `-z` —
    // and drilled down into it.
    offset_datum(&mut program, "top", xz_plane(), &h);
    let (b, c) = (format!("{} / 2", col("B")), format!("{} / 2", col("C")));
    let mut target = "extrude(block)".to_string();
    for (i, [x, y]) in [
        [b.clone(), c.clone()],
        [neg(&b), c.clone()],
        [neg(&b), neg(&c)],
        [b.clone(), neg(&c)],
    ]
    .into_iter()
    .enumerate()
    {
        let mut hole = Drawing::new(&program.parameters)?;
        let centre = hole.point(x, y)?;
        hole.circle(centre, &col("M"))?;
        let id = format!("hole{}", i + 1);
        extrude(
            &mut program,
            &id,
            &format!("{id}_sketch"),
            (hole, EntityRef::datum("top")),
            Extents {
                reversed: true,
                ..Extents::blind(format!("{} + 1", col("M")))
            },
            Combine::Difference { target },
        )?;
        target = format!("extrude({id})");
    }
    axis_datum(&mut program);
    Ok(StandardPart {
        file: "std:linear_carriage.geop",
        title: "Miniature linear guide carriage, MGN series",
        designation: "Linear carriage",
        base: "top",
        program,
        threaded: Vec::new(),
    })
}

/// The dimensions of a hobby servo, in millimetres, and its mass in grams.
struct Servo {
    /// The body: length along `x`, width along `y`, and how far it reaches
    /// below and above the mounting tabs' underside.
    length: f64,
    width: f64,
    below: f64,
    above: f64,
    /// How far the body's middle is from the output shaft, along `-x`.
    offset: f64,
    /// The tabs: their span along `x` and thickness.
    tabs: f64,
    tab: f64,
    /// The mounting holes: how far apart along `x`, along `y` (0 for one
    /// hole per tab), and across.
    holes: f64,
    pair: f64,
    hole: f64,
    /// The round boss the shaft stands on and the splined shaft: diameter
    /// and height each.
    boss: [f64; 2],
    spline: [f64; 2],
    grams: f64,
}

/// A hobby servo as an envelope: its body standing up `z` around the
/// output shaft, which is the `z` axis — the datum `axis`, a revolute
/// joint's connector — with the mounting tabs' underside on the `xy`
/// plane, the datum `mount`, and the top of the splined shaft the datum
/// `output`. The body and tabs are one side profile extruded across, the
/// boss and shaft one profile turned on its top; its material weighs what
/// the servo does.
fn servo(
    dims: &Servo,
    row: &'static str,
    file: &'static str,
    title: &'static str,
) -> GeopResult<StandardPart> {
    let Servo {
        length,
        width,
        below,
        above,
        offset,
        tabs,
        tab,
        holes,
        pair,
        hole,
        boss,
        spline,
        grams,
    } = *dims;
    let volume = length * width * (below + above)
        + (tabs - length) * width * tab
        + std::f64::consts::PI / 4.0 * (boss[0].powi(2) * boss[1] + spline[0].powi(2) * spline[1]);
    let mut program = Program::new();
    program.parameters = Parameters {
        material: Some(Material {
            name: format!("Plastic, as heavy as the {row} ({grams} g)"),
            density: grams * 1e6 / volume,
        }),
        color: Some("#2b5fa8".into()),
        values: vec![size(tables::Table {
            columns: &[],
            rows: vec![(row.to_string(), Vec::new())],
            selected: row,
        })],
    };
    // Side on, in the `xz` plane — whose sketch runs along `x` and `-z`.
    let n = |v: f64| format!("{v}");
    let (x0, x1) = (-offset - length / 2.0, -offset + length / 2.0);
    let (f0, f1) = (-offset - tabs / 2.0, -offset + tabs / 2.0);
    let mut side = Drawing::new(&program.parameters)?;
    side.polygon(&[
        [n(x0), n(below)],
        [n(x1), n(below)],
        [n(x1), "0".into()],
        [n(f1), "0".into()],
        [n(f1), n(-tab)],
        [n(x1), n(-tab)],
        [n(x1), n(-above)],
        [n(x0), n(-above)],
        [n(x0), n(-tab)],
        [n(f0), n(-tab)],
        [n(f0), "0".into()],
        [n(x0), "0".into()],
    ])?;
    extrude(
        &mut program,
        "body",
        "side",
        (side, xz_plane()),
        Extents {
            symmetric: true,
            ..Extents::blind(width)
        },
        Combine::NewBody,
    )?;
    // The boss starts 1 inside the body, so that none of its faces lies on
    // the body's top.
    let top = above + boss[1] + spline[1];
    let mut shaft = Drawing::new(&program.parameters)?;
    let lines = shaft.polygon(&[
        ["0".into(), n(above - 1.0)],
        [n(boss[0] / 2.0), n(above - 1.0)],
        [n(boss[0] / 2.0), n(above + boss[1])],
        [n(spline[0] / 2.0), n(above + boss[1])],
        [n(spline[0] / 2.0), n(top)],
        ["0".into(), n(top)],
    ])?;
    revolve(
        &mut program,
        "shaft",
        "profile",
        shaft,
        Around::Line(lines[5]),
        Combine::Union {
            target: "extrude(body)".into(),
        },
    )?;
    let mut target = "revolve(shaft)".to_string();
    let ys: &[f64] = if pair == 0.0 {
        &[0.0]
    } else {
        &[-pair / 2.0, pair / 2.0]
    };
    let mut k = 0;
    for x in [-offset - holes / 2.0, -offset + holes / 2.0] {
        for &y in ys {
            k += 1;
            let mut circle = Drawing::new(&program.parameters)?;
            let centre = circle.point(n(x), n(y))?;
            circle.circle(centre, &n(hole))?;
            let id = format!("hole{k}");
            program.push(format!("{id}_sketch"), circle.on(outline_plane())?);
            program.push(
                &id,
                ExtrudeArgs {
                    sketch: format!("{id}_sketch"),
                    extent: Extents {
                        symmetric: true,
                        ..Extents::blind(4.0 * tab)
                    },
                    face: false,
                    combine: Combine::Difference { target },
                },
            );
            target = format!("extrude({id})");
        }
    }
    axis_datum(&mut program);
    plane_datum(&mut program, "mount", "0");
    plane_datum(&mut program, "output", &n(top));
    Ok(StandardPart {
        file,
        title,
        designation: "Servo",
        base: "mount",
        program,
        threaded: Vec::new(),
    })
}

pub fn servo_sg90() -> GeopResult<StandardPart> {
    servo(
        &Servo {
            length: 22.8,
            width: 12.2,
            below: 15.9,
            above: 6.8,
            offset: 5.4,
            tabs: 32.2,
            tab: 2.5,
            holes: 27.8,
            pair: 0.0,
            hole: 2.0,
            boss: [11.4, 4.0],
            spline: [4.8, 3.2],
            grams: 9.0,
        },
        "SG90",
        "std:servo_sg90.geop",
        "Micro servo SG90",
    )
}

pub fn servo_mg996r() -> GeopResult<StandardPart> {
    servo(
        &Servo {
            length: 40.7,
            width: 19.7,
            below: 27.0,
            above: 10.0,
            offset: 10.2,
            tabs: 54.0,
            tab: 2.5,
            holes: 49.5,
            pair: 10.0,
            hole: 4.2,
            boss: [18.0, 5.0],
            spline: [5.8, 4.0],
            grams: 55.0,
        },
        "MG996R",
        "std:servo_mg996r.geop",
        "Standard servo MG996R",
    )
}
