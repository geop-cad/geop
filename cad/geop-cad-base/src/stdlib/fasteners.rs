//! Screws, nuts, washers, dowel pins and standoffs: each family one
//! program, its size a row of the table parameter [`SIZE`](super::steps::SIZE).
//!
//! Every part turns around the `z` axis, its datum `axis`. A screw's head
//! sits on the `xy` plane, its shank down `-z` — the datum `seat` is the
//! underside of its head; a countersunk screw's `top` is the face left
//! flush. A nut, washer, pin or standoff stands on the `xy` plane, its
//! datum `base`, and rises up `z`.

use geop_core_math::geop_error::GeopResult;
use geop_core_sketch::{CurveId, PointId};
use geop_ops::parameters::Material;
use geop_ops::parameters::Parameters;
use geop_ops_booleans::Combine;

use super::{
    StandardPart,
    drawing::Drawing,
    steps::{
        Around, Through, axis_datum, base_datum, col, cut_to, hexagon, revolve, size, subtract,
        swept,
    },
    tables::{self, Table},
};
use crate::Program;

/// A program whose size is a row of `table`.
fn sized(table: Table) -> Program {
    let mut program = Program::new();
    program.parameters = Parameters {
        material: Some(Material {
            name: "Steel".into(),
            density: 7850.0,
        }),
        color: None,
        values: vec![size(table)],
    };
    program
}

/// How a screw's head is shaped.
enum Head {
    /// A cylinder `dk` across and `k` high: ISO 4762.
    Cylinder,
    /// A dome `dk` across and `k` high: ISO 7380.
    Dome,
    /// A 90° cone from `dk` across at the top down to the shank, `k` deep,
    /// its top flush at `z = 0`: ISO 10642.
    Countersunk,
    /// A hexagon `s` across its flats and `k` high, its top corners
    /// chamfered: ISO 4017.
    Hex,
}

impl Head {
    /// The height of its top, from the seat.
    fn top(&self) -> String {
        match self {
            Head::Countersunk => "0".into(),
            _ => col("k"),
        }
    }

    /// Where the shank leaves it.
    fn bottom(&self) -> String {
        match self {
            Head::Countersunk => format!("-{}", col("k")),
            _ => "0".into(),
        }
    }
}

/// A screw with `head`, its sizes from `table` — `d`, `p`, `l`, and the
/// head's own columns — and a hex socket `s` across and `t` deep unless
/// its head is a hexagon.
///
/// The shank is the thread's nominal diameter, chamfered at its tip by a
/// pitch; it is where a thread goes (see [`StandardPart::threaded`]).
fn screw(head: Head, table: Table) -> GeopResult<(Program, Vec<String>)> {
    let mut program = sized(table);
    let (d, p, k, l) = (col("d"), col("p"), col("k"), col("l"));
    let r = format!("{d} / 2");
    let mut drawing = Drawing::new(&program.parameters)?;
    let top_centre = drawing.point("0", head.top())?;
    let mut chain = Chain::from(top_centre);
    match head {
        Head::Cylinder => {
            let rim = format!("{} / 2", col("dk"));
            chain.to(&mut drawing, &rim, &k)?;
            chain.to(&mut drawing, &rim, "0")?;
        }
        Head::Dome => {
            let rim = drawing.point(format!("{} / 2", col("dk")), "0")?;
            let radius = format!("({dk}^2 / 4 + {k}^2) / (2 * {k})", dk = col("dk"));
            drawing.arc(top_centre, rim, &radius, false)?;
            chain = Chain::from(rim);
        }
        Head::Countersunk => {
            chain.to(&mut drawing, &format!("{} / 2", col("dk")), "0")?;
        }
        Head::Hex => {
            // An envelope the hexagon is cut to: its top chamfered at 30°
            // from a circle inside the flats out past the corners.
            let s = col("s");
            let outer = format!("0.65 * {s}");
            chain.to(&mut drawing, &format!("0.45 * {s}"), &k)?;
            chain.to(&mut drawing, &outer, &format!("{k} - 0.2 * {s} / sqrt(3)"))?;
            chain.to(&mut drawing, &outer, "0")?;
        }
    }
    chain.to(&mut drawing, &r, &head.bottom())?;
    let shank = chain.to(&mut drawing, &r, &format!("{p} - {l}"))?;
    chain.to(&mut drawing, &format!("{r} - {p}"), &format!("-{l}"))?;
    chain.to(&mut drawing, "0", &format!("-{l}"))?;
    let axis = drawing.line(chain.last, top_centre);
    revolve(
        &mut program,
        "body",
        "profile",
        drawing,
        Around::Line(axis),
        Combine::NewBody,
    )?;
    let threaded = swept("body", "profile", shank);

    if let Head::Hex = head {
        let mut outline = Drawing::new(&program.parameters)?;
        hexagon(&mut outline, &col("s"))?;
        cut_to(
            &mut program,
            "head",
            "hex",
            outline,
            "revolve(body)",
            Through::Both,
        )?;
    } else {
        // A countersunk head's socket reaches below its top, at `z = 0`.
        let through = match head {
            Head::Countersunk => Through::Both,
            _ => Through::Up,
        };
        socket(&mut program, &head.top(), through)?;
        subtract(&mut program, "screw", "revolve(body)", "extrude(socket)");
    }
    axis_datum(&mut program);
    let seat = match head {
        Head::Countersunk => "top",
        _ => "seat",
    };
    base_datum(&mut program, seat);
    Ok((program, threaded))
}

/// Lines drawn one after the other, each from where the last ended.
struct Chain {
    last: PointId,
}

impl From<PointId> for Chain {
    fn from(last: PointId) -> Self {
        Self { last }
    }
}

impl Chain {
    /// The line on to `(x, y)`.
    fn to(&mut self, drawing: &mut Drawing, x: &str, y: &str) -> GeopResult<CurveId> {
        let next = drawing.point(x, y)?;
        let line = drawing.line(self.last, next);
        self.last = next;
        Ok(line)
    }
}

/// The tool a hex socket `s` across and `t` deep is cut with, its floor
/// `t` below `top`: the solid `extrude(socket)`, a hexagon cut to a
/// cylinder from the floor up past the top.
fn socket(program: &mut Program, top: &str, through: Through) -> GeopResult<()> {
    let (s, t) = (col("s"), col("t"));
    let mut drawing = Drawing::new(&program.parameters)?;
    let floor = format!("{top} - {t}");
    let above = format!("{top} + {s}");
    let outside = format!("0.7 * {s}");
    let corners = [
        ["0".to_string(), floor.clone()],
        [outside.clone(), floor],
        [outside, above.clone()],
        ["0".to_string(), above],
    ];
    let lines = drawing.polygon(&corners)?;
    revolve(
        program,
        "socket_envelope",
        "socket_profile",
        drawing,
        Around::Line(lines[3]),
        Combine::NewBody,
    )?;
    let mut outline = Drawing::new(&program.parameters)?;
    hexagon(&mut outline, &s)?;
    cut_to(
        program,
        "socket",
        "socket_hex",
        outline,
        "revolve(socket_envelope)",
        through,
    )
}

pub fn iso4762() -> GeopResult<StandardPart> {
    let (program, threaded) = screw(Head::Cylinder, tables::iso4762())?;
    Ok(StandardPart {
        file: "std:iso4762_socket_head_cap_screw.geop",
        title: "ISO 4762 socket head cap screw",
        base: "seat",
        program,
        threaded,
    })
}

pub fn iso7380() -> GeopResult<StandardPart> {
    let (program, threaded) = screw(Head::Dome, tables::iso7380())?;
    Ok(StandardPart {
        file: "std:iso7380_button_head_screw.geop",
        title: "ISO 7380 button head screw",
        base: "seat",
        program,
        threaded,
    })
}

pub fn iso10642() -> GeopResult<StandardPart> {
    let (program, threaded) = screw(Head::Countersunk, tables::iso10642())?;
    Ok(StandardPart {
        file: "std:iso10642_countersunk_screw.geop",
        title: "ISO 10642 countersunk screw",
        base: "top",
        program,
        threaded,
    })
}

pub fn iso4017() -> GeopResult<StandardPart> {
    let (program, threaded) = screw(Head::Hex, tables::iso4017())?;
    Ok(StandardPart {
        file: "std:iso4017_hex_head_screw.geop",
        title: "ISO 4017 hex head screw",
        base: "seat",
        program,
        threaded,
    })
}

/// A nut `s` across its flats, `m` high, bored `d` through — where a
/// thread goes — both faces chamfered at 30° from its bearing face `dw`
/// across; above it, `h` high overall, the collar of a nylon insert, `dw`
/// across, if `insert`.
fn nut(table: Table, insert: bool) -> GeopResult<(Program, Vec<String>)> {
    let mut program = sized(table);
    let (s, m) = (col("s"), col("m"));
    let bore = format!("{} / 2", col("d"));
    let face = format!("{} / 2", col("dw"));
    let outer = format!("0.65 * {s}");
    let chamfer = format!("(0.65 * {s} - {face}) / sqrt(3)");
    let mut corners = vec![
        [bore.clone(), "0".into()],
        [face.clone(), "0".into()],
        [outer.clone(), chamfer.clone()],
        [outer, format!("{m} - {chamfer}")],
        [face.clone(), m.clone()],
    ];
    let top = if insert {
        corners.push([face, col("h")]);
        col("h")
    } else {
        m
    };
    corners.push([bore, top]);
    let mut drawing = Drawing::new(&program.parameters)?;
    let lines = drawing.polygon(&corners)?;
    let bore_line = *lines.last().expect("a profile has lines");
    revolve(
        &mut program,
        "body",
        "profile",
        drawing,
        Around::ZAxis,
        Combine::NewBody,
    )?;
    let mut outline = Drawing::new(&program.parameters)?;
    hexagon(&mut outline, &s)?;
    cut_to(
        &mut program,
        "nut",
        "hex",
        outline,
        "revolve(body)",
        Through::Up,
    )?;
    axis_datum(&mut program);
    base_datum(&mut program, "base");
    Ok((program, swept("body", "profile", bore_line)))
}

pub fn iso4032() -> GeopResult<StandardPart> {
    let (program, threaded) = nut(tables::iso4032(), false)?;
    Ok(StandardPart {
        file: "std:iso4032_hex_nut.geop",
        title: "ISO 4032 hex nut",
        base: "base",
        program,
        threaded,
    })
}

pub fn iso10511() -> GeopResult<StandardPart> {
    let (program, threaded) = nut(tables::iso10511(), true)?;
    Ok(StandardPart {
        file: "std:iso10511_nylon_insert_nut.geop",
        title: "ISO 10511 nylon insert lock nut",
        base: "base",
        program,
        threaded,
    })
}

/// A washer `d1` across its hole, `d2` outside, `h` thick — its outer top
/// edge chamfered at 30°, a quarter of `h` deep, if `chamfered`.
fn washer(table: Table, chamfered: bool) -> GeopResult<Program> {
    let mut program = sized(table);
    let (hole, outside, h) = (
        format!("{} / 2", col("d1")),
        format!("{} / 2", col("d2")),
        col("h"),
    );
    let mut corners = vec![[hole.clone(), "0".into()], [outside.clone(), "0".into()]];
    if chamfered {
        corners.push([outside.clone(), format!("0.75 * {h}")]);
        corners.push([format!("{outside} - 0.25 * sqrt(3) * {h}"), h.clone()]);
    } else {
        corners.push([outside, h.clone()]);
    }
    corners.push([hole, h]);
    let mut drawing = Drawing::new(&program.parameters)?;
    drawing.polygon(&corners)?;
    revolve(
        &mut program,
        "washer",
        "profile",
        drawing,
        Around::ZAxis,
        Combine::NewBody,
    )?;
    axis_datum(&mut program);
    base_datum(&mut program, "base");
    Ok(program)
}

pub fn iso7089() -> GeopResult<StandardPart> {
    Ok(StandardPart {
        file: "std:iso7089_washer.geop",
        title: "ISO 7089 plain washer",
        base: "base",
        program: washer(tables::iso7089(), false)?,
        threaded: Vec::new(),
    })
}

pub fn iso7090() -> GeopResult<StandardPart> {
    Ok(StandardPart {
        file: "std:iso7090_chamfered_washer.geop",
        title: "ISO 7090 chamfered washer",
        base: "base",
        program: washer(tables::iso7090(), true)?,
        threaded: Vec::new(),
    })
}

/// ISO 8734: a pin `d` across and `l` long, both ends chamfered by `c`.
pub fn iso8734() -> GeopResult<StandardPart> {
    let mut program = sized(tables::iso8734());
    let (r, c, l) = (format!("{} / 2", col("d")), col("c"), col("l"));
    let mut drawing = Drawing::new(&program.parameters)?;
    let lines = drawing.polygon(&[
        ["0".into(), "0".into()],
        [format!("{r} - {c}"), "0".into()],
        [r.clone(), c.clone()],
        [r.clone(), format!("{l} - {c}")],
        [format!("{r} - {c}"), l.clone()],
        ["0".into(), l],
    ])?;
    revolve(
        &mut program,
        "pin",
        "profile",
        drawing,
        Around::Line(lines[5]),
        Combine::NewBody,
    )?;
    axis_datum(&mut program);
    base_datum(&mut program, "base");
    Ok(StandardPart {
        file: "std:iso8734_dowel_pin.geop",
        title: "ISO 8734 dowel pin",
        base: "base",
        program,
        threaded: Vec::new(),
    })
}

/// A hex standoff `s` across its flats and `l` long, bored `d` through —
/// threaded from both ends.
pub fn hex_standoff() -> GeopResult<StandardPart> {
    let mut program = sized(tables::hex_standoffs());
    let (bore, outer, l) = (
        format!("{} / 2", col("d")),
        format!("0.65 * {}", col("s")),
        col("l"),
    );
    let mut drawing = Drawing::new(&program.parameters)?;
    let lines = drawing.polygon(&[
        [bore.clone(), "0".into()],
        [outer.clone(), "0".into()],
        [outer, l.clone()],
        [bore, l],
    ])?;
    revolve(
        &mut program,
        "body",
        "profile",
        drawing,
        Around::ZAxis,
        Combine::NewBody,
    )?;
    let mut outline = Drawing::new(&program.parameters)?;
    hexagon(&mut outline, &col("s"))?;
    cut_to(
        &mut program,
        "standoff",
        "hex",
        outline,
        "revolve(body)",
        Through::Up,
    )?;
    axis_datum(&mut program);
    base_datum(&mut program, "base");
    Ok(StandardPart {
        file: "std:hex_standoff.geop",
        title: "Hex standoff, female",
        base: "base",
        program,
        threaded: swept("body", "profile", lines[3]),
    })
}
