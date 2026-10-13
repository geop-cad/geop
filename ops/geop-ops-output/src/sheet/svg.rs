//! A [`Sheet`] as SVG: one group per layer, sized in millimetres.

use std::{convert::Infallible, fmt::Write};

use geop_core_math::solvers::repeat::{Repeated, Step, repeat};

use super::{Anchor, LineType, Shape, Sheet, decimal};
use crate::xml_escape;

/// How a layer's strokes are dashed, as SVG's `stroke-dasharray`.
fn dashes(line_type: LineType) -> Option<String> {
    let dashes = line_type.dashes();
    (!dashes.is_empty()).then(|| {
        let lengths: Vec<String> = dashes.iter().map(|d| num(d.abs())).collect();
        lengths.join(" ")
    })
}

/// `n` to a thousandth of a millimetre (see [`decimal`]).
fn num(n: f64) -> String {
    decimal(n, 3)
}

/// The sheet as an SVG document. The paper's `y` points up, SVG's down: every
/// point is flipped, which turns counter-clockwise arcs into SVG's negative
/// sweep.
pub fn to_svg(sheet: &Sheet) -> String {
    let h = sheet.height;
    let p = |q: [f64; 2]| format!("{} {}", num(q[0]), num(h - q[1]));
    let mut out = String::new();
    let _ = writeln!(
        out,
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{hh}mm" viewBox="0 0 {w} {hh}">"#,
        w = num(sheet.width),
        hh = num(h)
    );
    let _ = writeln!(
        out,
        r#"<rect x="0" y="0" width="{}" height="{}" fill="white"/>"#,
        num(sheet.width),
        num(h)
    );
    for layer in sheet.layers() {
        let dash = dashes(layer.line_type)
            .map(|d| format!(r#" stroke-dasharray="{d}""#))
            .unwrap_or_default();
        let _ = writeln!(
            out,
            r#"<g class="{}" fill="none" stroke="black" stroke-width="{}" stroke-linecap="round"{dash}>"#,
            layer.name,
            num(layer.width)
        );
        for stroke in sheet.strokes.iter().filter(|s| s.layer == layer) {
            let _ = match &stroke.shape {
                Shape::Line(a, b) => writeln!(
                    out,
                    r#"<line x1="{}" y1="{}" x2="{}" y2="{}"/>"#,
                    num(a[0]),
                    num(h - a[1]),
                    num(b[0]),
                    num(h - b[1])
                ),
                Shape::Circle { center, radius } => writeln!(
                    out,
                    r#"<circle cx="{}" cy="{}" r="{}"/>"#,
                    num(center[0]),
                    num(h - center[1]),
                    num(*radius)
                ),
                Shape::Arc {
                    center,
                    radius,
                    start,
                    end,
                } => {
                    // Turned on by whole turns until positive: no more of
                    // them than the turns it is short by, and one for the
                    // rounding.
                    let mut sweep = end - start;
                    let turns = (-sweep / std::f64::consts::TAU).max(0.0).ceil() as usize;
                    let run = repeat::<_, Infallible>(turns + 2, || {
                        Ok(if sweep <= 0.0 {
                            sweep += std::f64::consts::TAU;
                            Step::Again
                        } else {
                            Step::Done(())
                        })
                    });
                    assert!(
                        matches!(run, Ok(Repeated::Done { .. })),
                        "an arc's sweep becomes positive within its turns: {sweep:?}"
                    );
                    let at = |a: f64| [center[0] + radius * a.cos(), center[1] + radius * a.sin()];
                    let large = if sweep > std::f64::consts::PI { 1 } else { 0 };
                    writeln!(
                        out,
                        r#"<path d="M {} A {r} {r} 0 {large} 0 {}"/>"#,
                        p(at(*start)),
                        p(at(*end)),
                        r = num(*radius)
                    )
                }
                Shape::Polyline(points) => writeln!(
                    out,
                    r#"<polyline points="{}"/>"#,
                    points.iter().map(|q| p(*q)).collect::<Vec<_>>().join(" ")
                ),
                Shape::Filled(points) => writeln!(
                    out,
                    r#"<polygon points="{}" fill="black" stroke="none"/>"#,
                    points.iter().map(|q| p(*q)).collect::<Vec<_>>().join(" ")
                ),
            };
        }
        for label in sheet.labels.iter().filter(|l| l.layer == layer) {
            let anchor = match label.anchor {
                Anchor::Start => "start",
                Anchor::Middle => "middle",
                Anchor::End => "end",
            };
            let (x, y) = (num(label.at[0]), num(h - label.at[1]));
            let rotate = if label.angle == 0.0 {
                String::new()
            } else {
                format!(r#" transform="rotate({} {x} {y})""#, num(-label.angle))
            };
            let _ = writeln!(
                out,
                r#"<text x="{x}" y="{y}" font-family="sans-serif" font-size="{}" text-anchor="{anchor}" fill="black" stroke="none"{rotate}>{}</text>"#,
                num(label.height),
                xml_escape(&label.text)
            );
        }
        out.push_str("</g>\n");
    }
    out.push_str("</svg>\n");
    out
}
