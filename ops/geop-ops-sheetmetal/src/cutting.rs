//! A sheet-metal body's flat pattern laid out for cutting: its outline
//! and holes on the `CUT` layer, its bend lines on the `BEND` layer with
//! how each is bent beside it ([`cutting_sheet`]), written as DXF by the
//! drawings' own writer ([`flat_pattern_dxf`]).

use std::collections::BTreeMap;

use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops::Part;
use geop_ops_drawing::{
    sheet::{Anchor, Label, Layer, Shape, Sheet as Paper, strokes_of},
    to_dxf,
};

use crate::{flat_pattern::FlatPatternData, sheet::Sheet};

/// How high a bend's note is, as a share of the pattern's larger side.
const NOTE_HEIGHT: f64 = 0.025;

/// `x` to a thousandth, without trailing zeros.
fn number(x: f64) -> String {
    let s = format!("{x:.3}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
}

/// The flat pattern of `sheet` on paper, one model unit a unit, its
/// lower left corner at the origin, seen from its B side — the first
/// flat's sheet coordinates: every curve of the unfolded sheet's boundary
/// on the `CUT` layer, each bend's line on the `BEND` layer, and beside it
/// `UP 90° R0.1` — `UP` for a bend towards the viewer.
pub fn cutting_sheet<S: Scalar>(sheet: &Sheet<S>) -> GeopResult<Paper> {
    let ctx = with_context!("cutting_sheet");
    let layout = sheet.layout().with_context(ctx)?;
    // The boundary: the edges only one face has.
    let mut uses: BTreeMap<usize, usize> = BTreeMap::new();
    for face in &layout.faces {
        for &(e, _) in face.loops().flatten() {
            *uses.entry(e).or_default() += 1;
        }
    }
    let boundary: Vec<&NurbCurve2D<S>> = uses
        .iter()
        .filter(|(_, n)| **n == 1)
        .map(|(&e, _)| &layout.edges[e].curve)
        .collect();
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for curve in &boundary {
        for cp in &curve.control_points {
            for k in 0..2 {
                let x = cp[k].div(cp[2])?;
                lo[k] = lo[k].min(x.lower().to_f64());
                hi[k] = hi[k].max(x.upper().to_f64());
            }
        }
    }
    if !(lo[0].is_finite() && lo[1].is_finite()) {
        return Err(GeopError::new("the flat pattern has no outline")).with_context(ctx);
    }
    let place = |p: [f64; 2]| [p[0] - lo[0], p[1] - lo[1]];
    let curves: Vec<(Layer, &NurbCurve2D<S>)> = boundary.iter().map(|&c| (Layer::Cut, c)).collect();
    let mut paper = Paper {
        width: hi[0] - lo[0],
        height: hi[1] - lo[1],
        strokes: strokes_of(&curves, &place, 1.0).with_context(ctx)?,
        labels: Vec::new(),
    };
    let height = NOTE_HEIGHT * paper.width.max(paper.height);
    for (bend, [a, b]) in sheet.bends.iter().zip(sheet.bend_lines()?) {
        let (a, b) = (
            place([a[0].to_f64(), a[1].to_f64()]),
            place([b[0].to_f64(), b[1].to_f64()]),
        );
        paper.stroke(Layer::Bend, Shape::Line(a, b));
        // Along the line, reading left to right.
        let mut angle = (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees();
        if angle > 90.0 {
            angle -= 180.0;
        } else if angle <= -90.0 {
            angle += 180.0;
        }
        let turn = if bend.toward_b { "UP" } else { "DOWN" };
        paper.labels.push(Label {
            layer: Layer::Bend,
            at: [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0],
            height,
            angle,
            anchor: Anchor::Middle,
            text: format!(
                "{turn} {}° R{}",
                number(bend.angle.to_f64().to_degrees()),
                number(bend.radius.to_f64())
            ),
        });
    }
    Ok(paper)
}

/// The flat pattern of the body `solid` of `part` — a sheet-metal body, or
/// one a flat pattern unfolded — as a DXF file for cutting (see
/// [`cutting_sheet`]); without a name, of the newest such body. Returns the
/// body's name with the file.
pub fn flat_pattern_dxf<S: Scalar>(
    part: &Part<S>,
    solid: Option<&str>,
) -> GeopResult<(String, String)> {
    let candidates: Vec<String> = match solid {
        Some(name) => vec![name.to_string()],
        None => part.solid_names().into_iter().rev().collect(),
    };
    for name in candidates {
        if let Some(sheet) = part.body_data::<Sheet<S>>(&name) {
            return Ok((name.clone(), to_dxf(&cutting_sheet(sheet)?)));
        }
        if let Some(flat) = part.body_data::<FlatPatternData>(&name) {
            return Ok((name.clone(), to_dxf(&flat.pattern)));
        }
    }
    Err(GeopError::new(match solid {
        Some(name) => format!(
            "{name} is no sheet-metal body: only a body built by a base flange and its flanges, or its flat pattern, is laid out for cutting"
        ),
        None => "the part has no sheet-metal body to lay out for cutting".to_string(),
    }))
}
