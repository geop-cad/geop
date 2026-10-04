//! A [`Sheet`] as SVG: one group per layer, sized in millimetres.

use std::fmt::Write;

use crate::sheet::{Anchor, Layer, Shape, Sheet};

/// How a layer's strokes look: width in millimetres, and dashes.
fn style(layer: Layer) -> (f64, Option<&'static str>) {
    match layer {
        Layer::Visible => (0.5, None),
        Layer::Hidden => (0.3, Some("3 1.5")),
        Layer::Center => (0.25, Some("8 1.5 1.5 1.5")),
        Layer::Dimension => (0.25, None),
        Layer::Hatch => (0.18, None),
        Layer::Border => (0.5, None),
        Layer::Thread => (0.25, None),
    }
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// `n` without trailing zeros, to a thousandth of a millimetre.
fn num(n: f64) -> String {
    let s = format!("{n:.3}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s == "-0" { "0".into() } else { s.into() }
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
    for layer in Layer::ALL {
        let (width, dashes) = style(layer);
        let dash = dashes
            .map(|d| format!(r#" stroke-dasharray="{d}""#))
            .unwrap_or_default();
        let _ = writeln!(
            out,
            r#"<g class="{}" fill="none" stroke="black" stroke-width="{}" stroke-linecap="round"{dash}>"#,
            layer.name(),
            num(width)
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
                    let mut sweep = end - start;
                    while sweep <= 0.0 {
                        sweep += std::f64::consts::TAU;
                    }
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
                escape(&label.text)
            );
        }
        out.push_str("</g>\n");
    }
    out.push_str("</svg>\n");
    out
}
