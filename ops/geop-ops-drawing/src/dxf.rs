//! A [`Sheet`] as an ASCII DXF file of release 12 — the oldest and most
//! widely read: lines, arcs, circles, polylines, solids and text, one layer
//! per [`Layer`] it draws on, with hidden, centre and bend lines dashed by
//! line type.

use std::fmt::Write;

use crate::sheet::{Anchor, Layer, Shape, Sheet};

/// The line type of a layer: its name, description and dash pattern
/// (lengths in millimetres, negative for gaps).
fn line_type(layer: Layer) -> &'static str {
    match layer {
        Layer::Hidden => "HIDDEN",
        Layer::Center | Layer::Bend => "CENTER",
        _ => "CONTINUOUS",
    }
}

/// The colour of a layer, by AutoCAD colour index.
fn color(layer: Layer) -> u8 {
    match layer {
        Layer::Visible | Layer::Border | Layer::Cut => 7,
        Layer::Hidden => 8,
        Layer::Center => 1,
        Layer::Dimension => 3,
        Layer::Hatch => 9,
        Layer::Thread => 4,
        Layer::Bend => 5,
    }
}

const LINE_TYPES: [(&str, &str, &[f64]); 3] = [
    ("CONTINUOUS", "Solid line", &[]),
    ("HIDDEN", "Hidden __ __ __", &[3.0, -1.5]),
    ("CENTER", "Center ____ _ ____", &[8.0, -1.5, 1.5, -1.5]),
];

/// Text as DXF writes it: its special characters as `%%` codes.
fn text(s: &str) -> String {
    s.replace('⌀', "%%c")
        .replace('°', "%%d")
        .replace('±', "%%p")
}

struct Writer(String);

impl Writer {
    fn pair(&mut self, code: i32, value: impl std::fmt::Display) {
        let _ = write!(self.0, "{code}\n{value}\n");
    }

    fn point(&mut self, base: i32, p: [f64; 2]) {
        self.pair(base, p[0]);
        self.pair(base + 10, p[1]);
        self.pair(base + 20, 0.0);
    }
}

/// The sheet as a DXF document.
pub fn to_dxf(sheet: &Sheet) -> String {
    let mut w = Writer(String::new());
    w.pair(0, "SECTION");
    w.pair(2, "HEADER");
    w.pair(9, "$ACADVER");
    w.pair(1, "AC1009");
    w.pair(9, "$EXTMIN");
    w.point(10, [0.0, 0.0]);
    w.pair(9, "$EXTMAX");
    w.point(10, [sheet.width, sheet.height]);
    w.pair(0, "ENDSEC");

    w.pair(0, "SECTION");
    w.pair(2, "TABLES");
    w.pair(0, "TABLE");
    w.pair(2, "LTYPE");
    w.pair(70, LINE_TYPES.len());
    for (name, description, dashes) in LINE_TYPES {
        w.pair(0, "LTYPE");
        w.pair(2, name);
        w.pair(70, 0);
        w.pair(3, description);
        w.pair(72, 65);
        w.pair(73, dashes.len());
        w.pair(40, dashes.iter().map(|d| d.abs()).sum::<f64>());
        for d in dashes {
            w.pair(49, d);
        }
    }
    w.pair(0, "ENDTAB");
    // The layers it draws on, each once.
    let layers: Vec<Layer> = Layer::ALL
        .into_iter()
        .filter(|&layer| {
            sheet.strokes.iter().any(|s| s.layer == layer)
                || sheet.labels.iter().any(|l| l.layer == layer)
        })
        .collect();
    w.pair(0, "TABLE");
    w.pair(2, "LAYER");
    w.pair(70, layers.len());
    for layer in layers {
        w.pair(0, "LAYER");
        w.pair(2, layer.name());
        w.pair(70, 0);
        w.pair(62, color(layer));
        w.pair(6, line_type(layer));
    }
    w.pair(0, "ENDTAB");
    w.pair(0, "ENDSEC");

    w.pair(0, "SECTION");
    w.pair(2, "ENTITIES");
    for stroke in &sheet.strokes {
        let layer = stroke.layer.name();
        match &stroke.shape {
            Shape::Line(a, b) => {
                w.pair(0, "LINE");
                w.pair(8, layer);
                w.point(10, *a);
                w.point(11, *b);
            }
            Shape::Circle { center, radius } => {
                w.pair(0, "CIRCLE");
                w.pair(8, layer);
                w.point(10, *center);
                w.pair(40, radius);
            }
            Shape::Arc {
                center,
                radius,
                start,
                end,
            } => {
                w.pair(0, "ARC");
                w.pair(8, layer);
                w.point(10, *center);
                w.pair(40, radius);
                w.pair(50, start.to_degrees());
                w.pair(51, end.to_degrees());
            }
            Shape::Polyline(points) => {
                w.pair(0, "POLYLINE");
                w.pair(8, layer);
                w.pair(66, 1);
                w.point(10, [0.0, 0.0]);
                w.pair(70, 0);
                for p in points {
                    w.pair(0, "VERTEX");
                    w.pair(8, layer);
                    w.point(10, *p);
                }
                w.pair(0, "SEQEND");
                w.pair(8, layer);
            }
            Shape::Filled(points) => {
                // A solid takes four corners, the third and fourth crossed:
                // a triangle repeats its last.
                let corner = |k: usize| points[k.min(points.len() - 1)];
                w.pair(0, "SOLID");
                w.pair(8, layer);
                w.point(10, corner(0));
                w.point(11, corner(1));
                w.point(12, corner(3));
                w.point(13, corner(2));
            }
        }
    }
    for label in &sheet.labels {
        w.pair(0, "TEXT");
        w.pair(8, label.layer.name());
        w.point(10, label.at);
        w.pair(40, label.height);
        w.pair(1, text(&label.text));
        if label.angle != 0.0 {
            w.pair(50, label.angle);
        }
        let justify = match label.anchor {
            Anchor::Start => 0,
            Anchor::Middle => 1,
            Anchor::End => 2,
        };
        if justify != 0 {
            w.pair(72, justify);
            w.point(11, label.at);
        }
    }
    w.pair(0, "ENDSEC");
    w.pair(0, "EOF");
    w.0
}
