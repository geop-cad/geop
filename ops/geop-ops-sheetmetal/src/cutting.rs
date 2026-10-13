//! A sheet-metal body's flat pattern laid out for cutting: its outline
//! and holes on the `CUT` layer, its bend lines on the `BEND` layer with
//! how each is bent beside it ([`cutting_sheet`]), written as DXF by the
//! [`FlatPatternDxf`] exporter.

use std::collections::BTreeMap;

use geop_core_geometry::{aabb::union_aabb, nurb_curve::NurbCurve2D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use geop_ops_model::read_model;
use geop_ops_output::{
    Content, Export, Exporter, ExporterInfo, Output,
    sheet::{Anchor, Format, Label, Layer, LineType, Shape, Sheet as Paper, decimal, strokes_of},
};
use serde::Deserialize;

use crate::{
    body::{SheetBody, sheet_body},
    sheet::Sheet,
};

/// What a laser or a punch cuts: the flat pattern's outline and holes.
pub const CUT: Layer = Layer {
    name: "CUT",
    width: 0.25,
    line_type: LineType::Continuous,
    color: 7,
};

/// Where the flat pattern is bent, and how.
pub const BEND: Layer = Layer {
    name: "BEND",
    width: 0.25,
    line_type: LineType::Center,
    color: 5,
};

/// How high a bend's note is, as a share of the pattern's larger side.
const NOTE_HEIGHT: f64 = 0.025;

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
    let Some(bounds) = union_aabb(boundary.iter().map(|c| c.aabb())) else {
        return Err(GeopError::new("the flat pattern has no outline")).with_context(ctx);
    };
    let lo: [f64; 2] = std::array::from_fn(|k| bounds[k].lower().to_f64());
    let hi: [f64; 2] = std::array::from_fn(|k| bounds[k].upper().to_f64());
    let place = |p: [f64; 2]| [p[0] - lo[0], p[1] - lo[1]];
    let curves: Vec<(Layer, &NurbCurve2D<S>)> = boundary.iter().map(|&c| (CUT, c)).collect();
    let mut paper = Paper {
        width: hi[0] - lo[0],
        height: hi[1] - lo[1],
        layers: vec![CUT, BEND],
        strokes: strokes_of(&curves, &place, 1.0).with_context(ctx)?,
        labels: Vec::new(),
    };
    let height = NOTE_HEIGHT * paper.width.max(paper.height);
    for (bend, [a, b]) in sheet.bends.iter().zip(sheet.bend_lines()?) {
        let (a, b) = (
            place([a[0].to_f64(), a[1].to_f64()]),
            place([b[0].to_f64(), b[1].to_f64()]),
        );
        paper.stroke(BEND, Shape::Line(a, b));
        // Along the line, reading left to right.
        let mut angle = (b[1] - a[1]).atan2(b[0] - a[0]).to_degrees();
        if angle > 90.0 {
            angle -= 180.0;
        } else if angle <= -90.0 {
            angle += 180.0;
        }
        let turn = if bend.toward_b { "UP" } else { "DOWN" };
        paper.labels.push(Label {
            layer: BEND,
            at: [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0],
            height,
            angle,
            anchor: Anchor::Middle,
            text: format!(
                "{turn} {}° R{}",
                decimal(bend.angle.to_f64().to_degrees(), 3),
                decimal(bend.radius.to_f64(), 3)
            ),
        });
    }
    Ok(paper)
}

/// Writes the flat pattern of a sheet-metal body as a DXF file for laser
/// cutting (see [`cutting_sheet`]): of a body as it is bent, or of one a
/// flat pattern unfolded.
pub struct FlatPatternDxf;

/// Which body's flat pattern is written.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct FlatPatternOptions {
    /// The body, by its solid's name: the newest sheet-metal body, if none.
    pub solid: Option<String>,
}

impl Exporter for FlatPatternDxf {
    const INFO: ExporterInfo = ExporterInfo {
        kind: "flat_pattern",
        label: "Flat pattern",
        doc: "a sheet-metal body laid flat, for laser cutting",
        formats: &["dxf"],
    };
    type Options = FlatPatternOptions;

    fn export(output: &mut Output, _: &str, options: FlatPatternOptions) -> GeopResult<Export> {
        let part = output.part();
        let candidates: Vec<String> = match &options.solid {
            Some(name) => vec![name.clone()],
            None => read_model(&part)?.solid_names().into_iter().rev().collect(),
        };
        let found = candidates.into_iter().find_map(|name| {
            let body = sheet_body(&part, &name)?;
            Some((name, body))
        });
        let Some((_, body)) = found else {
            return Err(GeopError::new(match &options.solid {
                Some(name) => format!(
                    "{name} is no sheet-metal body: only a body built by a base flange and its flanges, or its flat pattern, is laid out for cutting"
                ),
                None => "the part has no sheet-metal body to lay out for cutting".to_string(),
            }));
        };
        let paper = match body {
            SheetBody::Bent(sheet) => cutting_sheet(sheet)?,
            SheetBody::Flat(flat) => flat.pattern.clone(),
        };
        let format = Format::Dxf;
        Ok(Export {
            name: format!("{}_flat.{}", output.stem()?, format.extension()),
            media_type: format.media_type().into(),
            content: Content::Text(paper.write(format)),
        })
    }
}
