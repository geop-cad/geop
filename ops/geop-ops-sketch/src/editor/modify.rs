//! Mirror, patterns and offset: geometry made from what is selected, tied
//! to it by constraints so it follows it (see [`geop_core_sketch::copies`]
//! and [`geop_core_sketch::offset`]).
//!
//! - **Offset**: clicking a curve selects the chain through it (or takes
//!   it out again); clicking beside the selection offsets it to there, as
//!   far as the pointer is from it — to both sides, if so set — and asks
//!   for the distance.
//! - **Mirror**: the first click picks the line to mirror across, and
//!   mirrors what was selected; with nothing selected, every curve clicked
//!   after is mirrored in turn.
//! - **Patterns** start from a selection: a click on a line repeats it
//!   along that line, towards the end clicked nearer; one on a point — or
//!   a circle, for its center — repeats it round that point. The pattern's
//!   construction curve is selected after, to change its count, and its
//!   spacing asked for.

use geop_core_math::geop_error::GeopResult;
use geop_core_sketch::{
    copies::{Made, Step},
    offset::Offsetted,
};

use super::*;
use crate::geometry::{dot, scale};

/// A full turn, in degrees: a circular pattern spread over all of it has
/// its angle given by the formula `360/count`, which follows its count.
fn full_turn(count: usize) -> String {
    format!("360/{count}")
}

/// `curves`, a chain, offset in `sketch` as far as `at` is from it and to
/// that side — and to the other side too, if `both` — its corners as
/// `corners` says.
pub(super) fn offset_to(
    sketch: &mut Sketch,
    curves: &[CurveId],
    at: P2,
    both: bool,
    corners: Corners,
) -> GeopResult<Vec<Offsetted>> {
    let chain = sketch.chain(curves)?;
    let d = sketch.chain_side(&chain, at)?;
    let mut made = vec![sketch.offset(curves, Design::from_f64(d), corners)?];
    if both {
        made.push(sketch.offset(curves, Design::from_f64(-d), corners)?);
    }
    Ok(made)
}

impl<S: Scalar> Editing<'_, S> {
    /// `change` made to the sketch, and the sketch solved — or, refused,
    /// nothing changed, and the hint saying why.
    fn change<T>(&mut self, change: impl FnOnce(&mut Sketch) -> GeopResult<T>) -> Option<T> {
        let mut next = self.sketch().clone();
        match change(&mut next) {
            Ok(made) => {
                self.commit(next);
                Some(made)
            }
            Err(e) => {
                self.s.error = Some(e.root_message().to_string());
                None
            }
        }
    }

    /// A click with `tool` in hand, at `pointer`.
    pub(super) fn modify_click(&mut self, tool: ModifyTool, pointer: &Pointer<S>) {
        self.s.error = None;
        let Some((at, _)) = self.in_plane(pointer) else {
            return;
        };
        let visuals = visuals(self.args, self.s, self.selection, &self.frame);
        let curve = hit_key(&visuals, pointer, &[&is_curve])
            .as_deref()
            .and_then(curve_key);
        match tool {
            ModifyTool::Offset => self.offset_click(curve, at),
            ModifyTool::Mirror => self.mirror_click(curve),
            ModifyTool::LinearPattern => self.linear_pattern_click(curve, at),
            ModifyTool::CircularPattern => {
                let point = hit_key(&visuals, pointer, &[&is_point])
                    .as_deref()
                    .and_then(point_key);
                self.circular_pattern_click(point, curve);
            }
        }
    }

    fn is_line(&self, curve: CurveId) -> bool {
        matches!(self.sketch().curves[&curve].kind, CurveKind::Line { .. })
    }

    /// On a curve, the chain through it is selected — or, all selected,
    /// taken out; elsewhere, what is selected is offset to there.
    fn offset_click(&mut self, curve: Option<CurveId>, at: P2) {
        if let Some(curve) = curve {
            let Ok(chain) = self.sketch().chain_through(curve) else {
                return;
            };
            let keys: Vec<String> = chain.iter().map(|c| c.to_string()).collect();
            if keys.iter().all(|k| self.selection.contains(k)) {
                self.selection.retain(|k| !keys.contains(k));
            } else {
                for key in keys {
                    if !self.selection.contains(&key) {
                        self.selection.push(key);
                    }
                }
            }
            return;
        }
        let curves = selected_curves(self.sketch(), self.selection);
        if curves.is_empty() {
            return;
        }
        let (both, corners) = (self.s.modify.both, self.s.modify.corners);
        if let Some(made) = self.change(|s| offset_to(s, &curves, at, both, corners)) {
            self.selection.clear();
            self.s.prompt = made.first().map(|o| o.distance);
            self.s.tool = Tool::Select;
        }
    }

    /// The first line clicked is the one to mirror across: what is
    /// selected is mirrored across it at once — or, with nothing selected,
    /// every curve clicked after.
    fn mirror_click(&mut self, curve: Option<CurveId>) {
        let Some(curve) = curve else {
            return;
        };
        match self.s.modify.mirror_line {
            Some(line) if line != curve => {
                self.change(|s| s.mirror(&[curve], line));
            }
            Some(_) => {}
            None if !self.is_line(curve) => {
                self.s.error = Some(format!("{curve} is no line: mirror across a line"));
            }
            None => {
                let curves: Vec<CurveId> = selected_curves(self.sketch(), self.selection)
                    .into_iter()
                    .filter(|&c| c != curve)
                    .collect();
                if curves.is_empty() {
                    self.s.modify.mirror_line = Some(curve);
                } else if self.change(|s| s.mirror(&curves, curve)).is_some() {
                    self.selection.clear();
                    self.s.tool = Tool::Select;
                }
            }
        }
    }

    /// What is selected repeated along the line clicked, towards its end
    /// clicked nearer.
    fn linear_pattern_click(&mut self, curve: Option<CurveId>, at: P2) {
        let Some(line) = curve.filter(|&c| self.is_line(c)) else {
            self.s.error = Some("click a line for the pattern's direction".into());
            return;
        };
        let curves = selected_curves(self.sketch(), self.selection);
        let CurveKind::Line { start, end } = self.sketch().curves[&line].kind else {
            return;
        };
        let (a, b) = (pt(self.sketch(), start), pt(self.sketch(), end));
        let backwards = dot(sub(at, scale(add(a, b), 0.5)), sub(b, a)) < 0.0;
        let step = Step::Along {
            along: line,
            backwards,
            spacing: Design::from_f64(self.s.modify.spacing),
        };
        let count = self.s.modify.count;
        if let Some(made) = self.change(|s| s.pattern(&curves, &step, count)) {
            self.patterned(made);
        }
    }

    /// What is selected repeated round the point clicked, or the center of
    /// the circle clicked.
    fn circular_pattern_click(&mut self, point: Option<PointId>, curve: Option<CurveId>) {
        let center = point.or_else(|| match self.sketch().curves.get(&curve?)?.kind {
            CurveKind::Circle { center, .. } => Some(center),
            _ => None,
        });
        let Some(center) = center else {
            self.s.error = Some("click the pattern's center: a point, or a circle".into());
            return;
        };
        let curves = selected_curves(self.sketch(), self.selection);
        let count = self.s.modify.count;
        let pitch = self.s.modify.pitch;
        let angle = pitch.unwrap_or(360.0 / count as f64).to_radians();
        let step = Step::Round {
            center,
            angle: Design::from_f64(angle),
        };
        if let Some(made) = self.change(|s| s.pattern(&curves, &step, count)) {
            if pitch.is_none() {
                self.args.formulas.insert(made.spacing, full_turn(count));
            }
            self.patterned(made);
        }
    }

    /// A pattern made: its construction curve selected, for its count, and
    /// its spacing asked for.
    fn patterned(&mut self, made: Made) {
        *self.selection = vec![made.by.to_string()];
        self.s.prompt = Some(made.spacing);
        self.s.tool = Tool::Select;
    }

    /// The pattern `by` given `count` copies, the original among them. One
    /// spread over the full circle stays spread over it.
    pub(super) fn set_pattern_count(&mut self, by: CurveId, count: usize) {
        let sketch = self.sketch();
        let (Some(now), spacing) = (sketch.pattern_count(by), sketch.pattern_spacing(by)) else {
            return;
        };
        let full = spacing.filter(|k| self.args.formulas.get(k) == Some(&full_turn(now)));
        let changed = self.change(|s| {
            // Spread over the full circle anew first, so the copies added
            // are placed where they go.
            if let Some(c) = full.and_then(|k| s.constraints.get_mut(&k)) {
                constraints::set_value(c, (360.0 / count as f64).to_radians());
                s.solve()?;
            }
            s.set_pattern_count(by, count)
        });
        if let (Some(()), Some(k)) = (changed, full) {
            self.args.formulas.insert(k, full_turn(count));
        }
    }

    /// The spacing a linear pattern of what is selected starts with: as far
    /// as it reaches, and half as far again.
    pub(super) fn default_spacing(&self) -> f64 {
        let sketch = self.sketch();
        let points: Vec<P2> = selected_curves(self.sketch(), self.selection)
            .iter()
            .flat_map(|&c| polyline(sketch, c))
            .collect();
        let span = |k: usize| {
            let lo = points.iter().map(|p| p[k]).fold(f64::MAX, f64::min);
            let hi = points.iter().map(|p| p[k]).fold(f64::MIN, f64::max);
            hi - lo
        };
        let reach = span(0).max(span(1));
        if reach.is_finite() && reach > 0.0 {
            1.5 * reach
        } else {
            1.0
        }
    }
}
