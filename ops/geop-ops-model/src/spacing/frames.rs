//! [`Frames`]: where the instances of a pattern go, from whatever gives
//! them — a row or a grid, a turn round an axis, a curve, points, a table
//! of offsets, a face filled, the holes of features — each the motion from
//! where the seed is to an instance. A pattern of bodies, of features and
//! of placed parts lays its instances out by one of these; linear and
//! circular are two of them.
//!
//! What a source gives that is a place rather than a motion — a point, a
//! hole — is reached from where the seed is: the first of the points or
//! holes, unless the seed is said to be elsewhere (a point picked `from`);
//! a face is filled on a grid through the seed's anchor.

use geop_core_editor::{
    edit_form::{Edit, EditForm},
    editing::{Cell, Choice, Column, ColumnKind, Number, Table, TableEdit, TableRow, Track, Unit},
};
use geop_core_geometry::{
    contains::surface::surface_could_contain,
    nurb_curve::NurbCurve3D,
    search::{MAX_NODES, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, FrameAxis, Pose, Quaternion},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{element::ElementReference, numeric::NumericInput, reader::Reader};
use geop_core_topology::{
    contains::face::{PointClassification, face_contains},
    distance::{Feature as DistanceTo, closest_points},
};
use serde::{Deserialize, Serialize};

use super::{
    Row, Spacing, count_field,
    holes::{holes_of, pose_of},
    references::{PatternDirection, PointsRef, TurnAxis, resolve_points},
    whole_count,
};
use crate::{
    FeatureRef, NamedModel, PathRef, Plane, datums::origin_axis, parameters::located, read_model,
};

/// A handle at `at`, dragged along `direction`; none for a direction that
/// could be zero, which nothing could be dragged along.
pub fn track(at: Vector3, direction: Vector3) -> Option<Track> {
    let length = direction.norm_sq();
    length
        .definitely_greater(Scalar::ZERO)
        .then_some(Track { at, direction })
}

/// One direction of a row or a grid of instances.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Direction {
    /// What gives the direction: a line — a straight edge, an axis, a
    /// sketch line — or a plane, along its normal; none, for a step that
    /// has not picked one.
    pub along: Option<PatternDirection>,
    /// Go against it instead.
    #[serde(default)]
    pub reversed: bool,
    /// How many instances along it, the seed included: a number, or a
    /// formula of the part's parameters.
    pub count: NumericInput,
    /// How far apart they are.
    pub spacing: Spacing,
}

impl Direction {
    /// Along the origin's axis `axis`, three instances a unit apart.
    pub fn along(axis: FrameAxis) -> Self {
        Direction {
            along: Some(PatternDirection::Datum(origin_axis(axis))),
            reversed: false,
            count: NumericInput::Plain(3.0),
            spacing: Spacing::step(1.0),
        }
    }

    /// The unit vector it goes along, as `reader` reads the part.
    pub fn unit(&self, reader: &Reader) -> GeopResult<Vector3> {
        let along = (self.along.as_ref())
            .ok_or_else(|| GeopError::new("pick a direction to pattern along"))?;
        let d = along.direction(reader)?;
        Ok(if self.reversed { d.neg() } else { d })
    }

    /// The instances along it, as `reader` reads the parameters.
    pub fn row(&self, reader: &Reader) -> GeopResult<Row> {
        let count = whole_count(&self.count, reader)?;
        self.spacing.along(count, reader)
    }

    /// Its fields in `f`, keyed with `suffix`: what it goes along, its
    /// sense, the count and the spacing, with handles from `at`. `set` puts
    /// the direction set where the arguments keep it.
    pub fn fields<'a, A: 'a>(
        &self,
        f: &mut EditForm<'a, A>,
        before: &Reader<'a>,
        at: Option<Vector3>,
        suffix: &str,
        set: impl Fn(&mut A, Direction) + Clone + 'a,
    ) {
        let key = |name: &str| format!("{name}{suffix}");
        let set_along = with(self.clone(), set.clone(), |d, picked| {
            d.along = Some(picked)
        });
        let set_reversed = with(self.clone(), set.clone(), |d, b| d.reversed = b);
        f.pick_one(
            &key("direction"),
            "direction",
            before,
            self.along.as_ref(),
            move |edit, p| set_along(edit.args, p),
        )
        .checkbox(
            &key("reversed"),
            "reverse direction",
            self.reversed,
            move |edit, b| set_reversed(edit.args, b),
        );
        // Moving the spacing's handle by the unit direction adds one to it;
        // the count's, by one step, adds an instance.
        let unit = self.unit(before).ok();
        let row = self.row(before).ok();
        let handles = at.zip(unit);
        let count_handle =
            handles
                .zip(row.filter(|row| row.count > 1))
                .and_then(|((at, unit), row)| {
                    let step = row.step();
                    let end = (row.count - 1) as f64 * step;
                    track(
                        at.add(&unit.prod_scalar(Scalar::from_f64(end))),
                        unit.prod_scalar(Scalar::from_f64(step)),
                    )
                });
        let set_count = with(self.clone(), set.clone(), |d, count| d.count = count);
        count_field(
            f,
            &key("count"),
            &self.count,
            before,
            count_handle,
            move |edit, count| set_count(edit.args, count),
        );
        self.spacing.fields(
            f,
            &key("spacing"),
            ["spacing", "total"],
            |label, value| {
                let number = Number::numeric(value, label, before, Unit::Length);
                let at = handles.and_then(|(at, unit)| {
                    track(
                        at.add(&unit.prod_scalar(Scalar::from_f64(number.value))),
                        unit,
                    )
                });
                number.handle(at)
            },
            with(self.clone(), set.clone(), |d, spacing| d.spacing = spacing),
        );
    }
}

/// What setting one field of `value` to a `V` makes of it, put where the
/// arguments keep it by `set`.
fn with<A, T: Clone, V>(
    value: T,
    set: impl Fn(&mut A, T) + Clone,
    change: fn(&mut T, V),
) -> impl Fn(&mut A, V) + Clone {
    move |args, v| {
        let mut value = value.clone();
        change(&mut value, v);
        set(args, value);
    }
}

/// One instance of a [`Frames::Table`]: how far it is from the seed along
/// each axis — numbers, or formulas of the part's parameters.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Offset {
    pub x: NumericInput,
    pub y: NumericInput,
    pub z: NumericInput,
}

/// Where the instances of a pattern go (see the module's docs).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Frames {
    /// In a row along a direction, or in a grid along two.
    Linear {
        first: Direction,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        second: Option<Direction>,
    },
    /// Turned round an axis: counter-clockwise looking against it, the
    /// other way if `reversed`.
    Circular {
        axis: Option<TurnAxis>,
        #[serde(default)]
        reversed: bool,
        count: NumericInput,
        /// How far apart, in degrees.
        angle: Spacing,
    },
    /// Along a path from where it starts — the seed where it starts — the
    /// instances `spacing` apart along it, or spread over a length; each
    /// turned as the path turns, if `aligned`, or kept as the seed is.
    Curve {
        path: Option<PathRef>,
        count: NumericInput,
        spacing: Spacing,
        #[serde(default)]
        aligned: bool,
    },
    /// At points: the seed at the first, or where `from` is.
    Points {
        points: Vec<PointsRef>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        from: Option<PointsRef>,
    },
    /// At offsets from the seed, a row each.
    Table { rows: Vec<Offset> },
    /// Filling a flat face on a grid through the seed — square, or each
    /// row shifted half a step and the rows closer — every instance
    /// `margin` or more inside the face's edges.
    Fill {
        face: Option<ElementReference<NamedModel, Plane>>,
        spacing: NumericInput,
        #[serde(default)]
        margin: NumericInput,
        #[serde(default)]
        staggered: bool,
    },
    /// At the holes features drilled: the seed in the first, each copy in
    /// another, turned as the hole is.
    Holes { features: Vec<FeatureRef> },
}

/// The kinds of [`Frames`], as a dialog offers them: the value, the label.
const KINDS: [(&str, &str); 7] = [
    ("linear", "In a row or a grid"),
    ("circular", "Round an axis"),
    ("curve", "Along a curve"),
    ("points", "At points"),
    ("table", "From a table of offsets"),
    ("fill", "Filling a face"),
    ("holes", "At holes"),
];

/// The most instances a pattern lays out: a fill spaced finer than its
/// face could otherwise make millions.
const MOST: usize = 10_000;

impl Frames {
    /// Its kind, as [`KINDS`] has it.
    fn kind(&self) -> &'static str {
        match self {
            Frames::Linear { .. } => "linear",
            Frames::Circular { .. } => "circular",
            Frames::Curve { .. } => "curve",
            Frames::Points { .. } => "points",
            Frames::Table { .. } => "table",
            Frames::Fill { .. } => "fill",
            Frames::Holes { .. } => "holes",
        }
    }

    /// A fresh one of the kind `kind`, nothing picked.
    fn fresh(kind: &str) -> Self {
        match kind {
            "circular" => Frames::Circular {
                axis: Some(TurnAxis::Datum(origin_axis(FrameAxis::Z))),
                reversed: false,
                count: NumericInput::Plain(4.0),
                angle: Spacing::extent(360.0),
            },
            "curve" => Frames::Curve {
                path: None,
                count: NumericInput::Plain(4.0),
                spacing: Spacing::step(10.0),
                aligned: false,
            },
            "points" => Frames::Points {
                points: Vec::new(),
                from: None,
            },
            "table" => Frames::Table {
                rows: vec![Offset {
                    x: NumericInput::Plain(10.0),
                    ..Offset::default()
                }],
            },
            "fill" => Frames::Fill {
                face: None,
                spacing: NumericInput::Plain(10.0),
                margin: NumericInput::Plain(0.0),
                staggered: false,
            },
            "holes" => Frames::Holes {
                features: Vec::new(),
            },
            _ => Frames::Linear {
                first: Direction::along(FrameAxis::X),
                second: None,
            },
        }
    }

    /// Where its instances go, as `reader` reads the part: each a label
    /// and the motion from the seed, the seed itself left out. `anchor` is
    /// where the seed is, for a fill: asked only there.
    pub fn motions(
        &self,
        reader: &Reader,
        anchor: &dyn Fn() -> GeopResult<Vector3>,
    ) -> GeopResult<Vec<(String, Pose)>> {
        let motions = match self {
            Frames::Linear { first, second } => linear(reader, first, second.as_ref()),
            Frames::Circular {
                axis,
                reversed,
                count,
                angle,
            } => circular(reader, axis.as_ref(), *reversed, count, angle),
            Frames::Curve {
                path,
                count,
                spacing,
                aligned,
            } => along_curve(reader, path.as_ref(), count, spacing, *aligned),
            Frames::Points { points, from } => at_points(reader, points, from.as_ref()),
            Frames::Table { rows } => from_table(reader, rows),
            Frames::Fill {
                face,
                spacing,
                margin,
                staggered,
            } => fill(reader, face.as_ref(), spacing, margin, *staggered, anchor),
            Frames::Holes { features } => at_holes(reader, features),
        }?;
        if motions.len() > MOST {
            return Err(GeopError::new(format!(
                "{} instances, more than the {MOST} a pattern lays out: space them further apart",
                motions.len()
            )));
        }
        Ok(motions)
    }

    /// Its fields in `f`: which kind — a choice keyed `frames` — and that
    /// kind's own, picked in the part `before` reads; handles of a row's
    /// from `at`. `set` puts the frames set where the arguments keep them.
    pub fn fields<'a, A: 'a>(
        &self,
        f: &mut EditForm<'a, A>,
        before: &Reader<'a>,
        at: Option<Vector3>,
        set: impl Fn(&mut A, Frames) + Clone + 'a,
    ) {
        let choices = KINDS.iter().map(|&(v, l)| Choice::new(v, l)).collect();
        let (own, set_kind) = (self.kind(), set.clone());
        f.select(
            "frames",
            "pattern",
            own,
            choices,
            false,
            move |edit, kind| {
                if kind != own {
                    set_kind(edit.args, Frames::fresh(kind));
                }
            },
        );
        match self {
            Frames::Linear { first, second } => {
                let second = second.clone();
                let set_first = set.clone();
                first.fields(f, before, at, "", move |args, first| {
                    set_first(
                        args,
                        Frames::Linear {
                            first,
                            second: second.clone(),
                        },
                    )
                });
                let (first, set_grid) = (first.clone(), set.clone());
                let has_second = matches!(
                    self,
                    Frames::Linear {
                        second: Some(_),
                        ..
                    }
                );
                let grid_first = first.clone();
                f.checkbox("grid", "second direction", has_second, move |edit, on| {
                    set_grid(
                        edit.args,
                        Frames::Linear {
                            first: grid_first.clone(),
                            second: on.then(|| Direction::along(FrameAxis::Y)),
                        },
                    )
                });
                if let Frames::Linear {
                    second: Some(second),
                    ..
                } = self
                {
                    second.fields(f, before, at, "2", move |args, second| {
                        set(
                            args,
                            Frames::Linear {
                                first: first.clone(),
                                second: Some(second),
                            },
                        )
                    });
                }
            }
            Frames::Circular { .. } => self.circular_fields(f, before, set),
            Frames::Curve { .. } => self.curve_fields(f, before, set),
            Frames::Points { points, from } => {
                let (from_kept, set_points) = (from.clone(), set.clone());
                f.pick(
                    "points",
                    "points",
                    before,
                    points,
                    true,
                    move |edit, points| {
                        set_points(
                            edit.args,
                            Frames::Points {
                                points,
                                from: from_kept.clone(),
                            },
                        )
                    },
                );
                let points = points.clone();
                f.pick_one(
                    "from",
                    "seed at",
                    before,
                    from.as_ref(),
                    move |edit, from| {
                        set(
                            edit.args,
                            Frames::Points {
                                points: points.clone(),
                                from: Some(from),
                            },
                        )
                    },
                );
                // The seed is at the first point unless said otherwise.
                f.form.dialog.optional("from");
            }
            Frames::Table { rows } => table_fields(f, before, rows, set),
            Frames::Fill { .. } => self.fill_fields(f, before, set),
            Frames::Holes { features } => {
                f.pick(
                    "holes",
                    "holes",
                    before,
                    features,
                    true,
                    move |edit, features| set(edit.args, Frames::Holes { features }),
                );
            }
        }
    }

    /// The fields of a turn round an axis.
    fn circular_fields<'a, A: 'a>(
        &self,
        f: &mut EditForm<'a, A>,
        before: &Reader<'a>,
        set: impl Fn(&mut A, Frames) + Clone + 'a,
    ) {
        let Frames::Circular {
            axis,
            reversed,
            count,
            angle,
        } = self.clone()
        else {
            return;
        };
        let me = self.clone();
        let set_axis = set.clone();
        let me_axis = me.clone();
        f.pick_one(
            "axis",
            "axis",
            before,
            axis.as_ref(),
            move |edit, picked| {
                let mut frames = me_axis.clone();
                if let Frames::Circular { axis, .. } = &mut frames {
                    *axis = Some(picked);
                }
                set_axis(edit.args, frames)
            },
        );
        let (set_rev, me_rev) = (set.clone(), me.clone());
        f.checkbox("reversed", "reverse direction", reversed, move |edit, b| {
            let mut frames = me_rev.clone();
            if let Frames::Circular { reversed, .. } = &mut frames {
                *reversed = b;
            }
            set_rev(edit.args, frames)
        });
        let (set_count, me_count) = (set.clone(), me.clone());
        count_field(f, "count", &count, before, None, move |edit, c| {
            let mut frames = me_count.clone();
            if let Frames::Circular { count, .. } = &mut frames {
                *count = c;
            }
            set_count(edit.args, frames)
        });
        angle.fields(
            f,
            "angle",
            ["angle", "total angle"],
            |label, value| Number::numeric(value, label, before, Unit::Angle).range(-360.0, 360.0),
            move |args, a| {
                let mut frames = me.clone();
                if let Frames::Circular { angle, .. } = &mut frames {
                    *angle = a;
                }
                set(args, frames)
            },
        );
    }

    /// The fields of a pattern along a curve.
    fn curve_fields<'a, A: 'a>(
        &self,
        f: &mut EditForm<'a, A>,
        before: &Reader<'a>,
        set: impl Fn(&mut A, Frames) + Clone + 'a,
    ) {
        let Frames::Curve {
            path,
            count,
            spacing,
            aligned,
        } = self.clone()
        else {
            return;
        };
        let me = self.clone();
        let (set_path, me_path) = (set.clone(), me.clone());
        f.pick_one(
            "path",
            "along",
            before,
            path.as_ref(),
            move |edit, picked| {
                let mut frames = me_path.clone();
                if let Frames::Curve { path, .. } = &mut frames {
                    *path = Some(picked);
                }
                set_path(edit.args, frames)
            },
        );
        let (set_count, me_count) = (set.clone(), me.clone());
        count_field(f, "count", &count, before, None, move |edit, c| {
            let mut frames = me_count.clone();
            if let Frames::Curve { count, .. } = &mut frames {
                *count = c;
            }
            set_count(edit.args, frames)
        });
        let (set_spacing, me_spacing) = (set.clone(), me.clone());
        spacing.fields(
            f,
            "spacing",
            ["spacing", "total"],
            |label, value| Number::numeric(value, label, before, Unit::Length),
            move |args, s| {
                let mut frames = me_spacing.clone();
                if let Frames::Curve { spacing, .. } = &mut frames {
                    *spacing = s;
                }
                set_spacing(args, frames)
            },
        );
        f.checkbox("aligned", "turn with the curve", aligned, move |edit, b| {
            let mut frames = me.clone();
            if let Frames::Curve { aligned, .. } = &mut frames {
                *aligned = b;
            }
            set(edit.args, frames)
        });
    }

    /// The fields of a face filled.
    fn fill_fields<'a, A: 'a>(
        &self,
        f: &mut EditForm<'a, A>,
        before: &Reader<'a>,
        set: impl Fn(&mut A, Frames) + Clone + 'a,
    ) {
        let Frames::Fill {
            face,
            spacing,
            margin,
            staggered,
        } = self.clone()
        else {
            return;
        };
        let me = self.clone();
        let (set_face, me_face) = (set.clone(), me.clone());
        f.pick_one(
            "face",
            "face",
            before,
            face.as_ref(),
            move |edit, picked| {
                let mut frames = me_face.clone();
                if let Frames::Fill { face, .. } = &mut frames {
                    *face = Some(picked);
                }
                set_face(edit.args, frames)
            },
        );
        let (set_spacing, me_spacing) = (set.clone(), me.clone());
        let number = Number::numeric(&spacing, "spacing", before, Unit::Length);
        f.numeric("spacing", number, move |edit, s| {
            let mut frames = me_spacing.clone();
            if let Frames::Fill { spacing, .. } = &mut frames {
                *spacing = s;
            }
            set_spacing(edit.args, frames)
        });
        let (set_margin, me_margin) = (set.clone(), me.clone());
        let number = Number::numeric(&margin, "margin", before, Unit::Length);
        f.numeric("margin", number, move |edit, m| {
            let mut frames = me_margin.clone();
            if let Frames::Fill { margin, .. } = &mut frames {
                *margin = m;
            }
            set_margin(edit.args, frames)
        });
        f.checkbox("staggered", "staggered rows", staggered, move |edit, b| {
            let mut frames = me.clone();
            if let Frames::Fill { staggered, .. } = &mut frames {
                *staggered = b;
            }
            set(edit.args, frames)
        });
    }
}

/// The fields of a table of offsets: a row per instance, a column per axis,
/// each a number or a formula, what a formula comes to said beside it.
fn table_fields<'a, A: 'a>(
    f: &mut EditForm<'a, A>,
    before: &Reader<'a>,
    rows: &[Offset],
    set: impl Fn(&mut A, Frames) + 'a,
) {
    let columns = ["x", "y", "z"]
        .into_iter()
        .map(|axis| Column::new(axis, ColumnKind::Formula))
        .collect();
    let cell = |value: &NumericInput| {
        let cell = Cell::new(value.to_string());
        match value {
            NumericInput::Plain(_) => cell,
            NumericInput::Formula { .. } => match value.resolve(before) {
                Ok(v) => cell.detail(format!("= {v}"), geop_core_editor::editing::Tone::Hint),
                Err(e) => cell.detail(e.to_string(), geop_core_editor::editing::Tone::Error),
            },
        }
    };
    let shown = (rows.iter())
        .map(|row| TableRow::new(vec![cell(&row.x), cell(&row.y), cell(&row.z)]))
        .collect();
    let table = Table::new(columns, shown, "No instances: add a row.");
    let rows = rows.to_vec();
    f.table("rows", table, move |edit: Edit<'_, A, ()>, what| {
        let mut rows = rows.clone();
        match what {
            TableEdit::Cell { row, column, text } => {
                let Some(offset) = rows.get_mut(row) else {
                    return;
                };
                let value = NumericInput::from(text.as_str());
                match column {
                    0 => offset.x = value,
                    1 => offset.y = value,
                    2 => offset.z = value,
                    _ => return,
                }
            }
            TableEdit::AddRow => rows.push(Offset::default()),
            TableEdit::RemoveRow { row } if row < rows.len() => {
                rows.remove(row);
            }
            TableEdit::MoveRow { row, to } if row < rows.len() && to < rows.len() => {
                let moved = rows.remove(row);
                rows.insert(to, moved);
            }
            _ => return,
        }
        set(edit.args, Frames::Table { rows });
    });
}

/// A row along `first`, or a grid along it and `second`: instance `i`
/// labelled `i`, in a grid `i.j`.
fn linear(
    reader: &Reader,
    first: &Direction,
    second: Option<&Direction>,
) -> GeopResult<Vec<(String, Pose)>> {
    let unit = first.unit(reader)?;
    let first_row = first.row(reader)?;
    let second = match second {
        None => None,
        Some(direction) => {
            let across = direction.unit(reader)?;
            let cross = unit.prod_cross(&across);
            if (0..3).all(|k| cross[k].could_be_equal(Scalar::ZERO)) {
                return Err(GeopError::new(
                    "the two directions are parallel: the grid's rows would lie on top of each other",
                ));
            }
            Some((across, direction.row(reader)?))
        }
    };
    let rows = second.as_ref().map_or(1, |(_, row)| row.count);
    let mut motions = Vec::new();
    for i in 0..first_row.count {
        for j in 0..rows {
            let along = first_row.offset(&unit, i)?;
            let across = match &second {
                Some((across, row)) => row.offset(across, j)?,
                None => None,
            };
            let offset = match (along, across) {
                (None, None) => continue,
                (Some(a), None) => a,
                (None, Some(b)) => b,
                (Some(a), Some(b)) => a.add(&b),
            };
            let label = match second {
                Some(_) => format!("{i}.{j}"),
                None => i.to_string(),
            };
            motions.push((label, Pose::new(offset, Quaternion::identity())?));
        }
    }
    Ok(motions)
}

/// Turned round `axis`: instance `k` labelled `k`.
fn circular(
    reader: &Reader,
    axis: Option<&TurnAxis>,
    reversed: bool,
    count: &NumericInput,
    angle: &Spacing,
) -> GeopResult<Vec<(String, Pose)>> {
    let axis = axis
        .ok_or_else(|| GeopError::new("pick an axis to turn around"))?
        .placed(reader)?;
    let direction = match reversed {
        true => axis.direction.neg(),
        false => axis.direction,
    };
    let row = angle.round(whole_count(count, reader)?, reader)?;
    (1..row.count)
        .map(|k| Ok((k.to_string(), row.turn(&axis.point, &direction, k)?)))
        .collect()
}

/// Along `path` from where it starts: instance `k` labelled `k`, where
/// `k` spacings of the row are run along it.
fn along_curve(
    reader: &Reader,
    path: Option<&PathRef>,
    count: &NumericInput,
    spacing: &Spacing,
    aligned: bool,
) -> GeopResult<Vec<(String, Pose)>> {
    let path = path.ok_or_else(|| GeopError::new("pick a curve to pattern along"))?;
    let placed = path.resolve_placed(reader)?;
    let motion = located(reader, &placed.instance)?.motion();
    let curves: Vec<NurbCurve3D> = (placed.element.curves.iter())
        .map(|c| c.transform(&motion))
        .collect();
    let lengths = (curves.iter())
        .map(|c| Ok(c.length()?.0.to_f64()))
        .collect::<GeopResult<Vec<f64>>>()?;
    let total: f64 = lengths.iter().sum();
    let row = spacing.along(whole_count(count, reader)?, reader)?;
    let start = curves
        .first()
        .ok_or_else(|| GeopError::new("the path has no curves"))?;
    let (t0, _) = start.domain();
    let origin = start.evaluate(t0)?;
    let mut along = Transport::new(start, t0)?;
    let mut motions = Vec::new();
    for k in 1..row.count {
        // Where to put a copy is a free choice: a sharp distance.
        let s = row.distance(k)?.to_f64();
        if s > total {
            return Err(GeopError::new(format!(
                "copy {k} would be {s} along a path {total} long: fewer copies, or closer"
            )));
        }
        let (curve, t) = at_length(&curves, &lengths, s)?;
        let point = curves[curve].evaluate(t)?;
        let pose = match aligned {
            false => Pose::new(point.sub(&origin), Quaternion::identity())?,
            true => {
                let turn = along.to(&curves, curve, t)?;
                let to_origin = Pose::new(origin.neg(), Quaternion::identity())?;
                let there = Pose::new(point, turn)?;
                there.compose(&to_origin)
            }
        };
        motions.push((k.to_string(), pose));
    }
    Ok(motions)
}

/// The curve of `curves` — each `lengths` long — and the parameter on it
/// at which `s` of their length is run: the parameter a free choice,
/// sharp.
fn at_length(curves: &[NurbCurve3D], lengths: &[f64], s: f64) -> GeopResult<(usize, Scalar)> {
    let mut run = 0.0;
    for (k, (curve, &length)) in curves.iter().zip(lengths).enumerate() {
        let last = k + 1 == curves.len();
        if s <= run + length || last {
            let (t0, t1) = curve.domain();
            let fraction = (s - run) / length;
            let t = if fraction <= 0.0 {
                t0
            } else if fraction >= 1.0 {
                t1
            } else {
                curve.parameter_at_length_fraction(fraction)?
            };
            return Ok((k, t));
        }
        run += length;
    }
    Err(GeopError::new("the path has no curves"))
}

/// A frame carried along a chain of curves turning as little as it can —
/// the rotation-minimizing frame, by double reflection (Wang et al.) over
/// a fixed number of steps per knot span. How a copy is turned along a
/// curve is the pattern's free choice; this one turns it with the curve,
/// never about it.
struct Transport {
    /// Where it has come to: the curve, the parameter, the point, the
    /// tangent and the frame's first normal there — plain numbers, a
    /// chosen orientation, not a measured one.
    curve: usize,
    t: f64,
    point: [f64; 3],
    tangent: [f64; 3],
    normal: [f64; 3],
    /// The frame where it started, the tangent its `x`.
    first: [[f64; 3]; 3],
}

/// The steps a [`Transport`] takes per knot span.
const STEPS_PER_SPAN: usize = 16;

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scaled(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn unit(a: [f64; 3]) -> GeopResult<[f64; 3]> {
    let n = dot(a, a).sqrt();
    if !(n > 0.0 && n.is_finite()) {
        return Err(GeopError::new("the path stops: it has no tangent there"));
    }
    Ok(scaled(a, 1.0 / n))
}

/// The point and the unit tangent of `curve` at `t`, plain.
fn point_tangent(curve: &NurbCurve3D, t: f64) -> GeopResult<([f64; 3], [f64; 3])> {
    let t = Scalar::from_f64(t);
    let point = curve.evaluate(t)?.to_f64_array();
    let tangent = unit(curve.tangent(t)?.to_f64_array())?;
    Ok((point, tangent))
}

impl Transport {
    /// Starting on `curve` at `t`, its first normal the world axis the
    /// tangent runs least along, made square to it.
    fn new(curve: &NurbCurve3D, t: Scalar) -> GeopResult<Self> {
        let (point, tangent) = point_tangent(curve, t.to_f64())?;
        let axis = (0..3)
            .min_by(|&a, &b| tangent[a].abs().total_cmp(&tangent[b].abs()))
            .unwrap_or(0);
        let mut e = [0.0; 3];
        e[axis] = 1.0;
        let normal = unit(sub(e, scaled(tangent, dot(e, tangent))))?;
        Ok(Self {
            curve: 0,
            t: t.to_f64(),
            point,
            tangent,
            normal,
            first: [tangent, normal, cross(tangent, normal)],
        })
    }

    /// One reflection step to `t` on the curve it is on.
    fn step(&mut self, curve: &NurbCurve3D, t: f64) -> GeopResult<()> {
        let (point, tangent) = point_tangent(curve, t)?;
        let v1 = sub(point, self.point);
        let c1 = dot(v1, v1);
        let (normal, along) = if c1 > 0.0 {
            (
                sub(self.normal, scaled(v1, 2.0 / c1 * dot(v1, self.normal))),
                sub(self.tangent, scaled(v1, 2.0 / c1 * dot(v1, self.tangent))),
            )
        } else {
            (self.normal, self.tangent)
        };
        let v2 = sub(tangent, along);
        let c2 = dot(v2, v2);
        let normal = if c2 > 0.0 {
            sub(normal, scaled(v2, 2.0 / c2 * dot(v2, normal)))
        } else {
            normal
        };
        // Square to the tangent again, against the rounding of each step.
        self.normal = unit(sub(normal, scaled(tangent, dot(normal, tangent))))?;
        (self.point, self.tangent, self.t) = (point, tangent, t);
        Ok(())
    }

    /// Carried on to `t` on the curve `to` of `curves` — never back — and
    /// the turn from where it started to there.
    fn to(&mut self, curves: &[NurbCurve3D], to: usize, t: Scalar) -> GeopResult<Quaternion> {
        let t = t.to_f64();
        while self.curve <= to {
            let curve = &curves[self.curve];
            let (_, end) = curve.domain();
            let stop = if self.curve == to { t } else { end.to_f64() };
            let spans: Vec<f64> = (curve.breakpoints().iter())
                .map(|b| b.to_f64())
                .filter(|&b| b > self.t && b < stop)
                .chain(std::iter::once(stop))
                .collect();
            for b in spans {
                let a = self.t;
                for k in 1..=STEPS_PER_SPAN {
                    let s = a + (b - a) * k as f64 / STEPS_PER_SPAN as f64;
                    self.step(curve, s)?;
                }
            }
            if self.curve == to {
                break;
            }
            // On to the next curve, where this one ends.
            self.curve += 1;
            let (start, _) = curves[self.curve].domain();
            self.t = start.to_f64();
            self.step(&curves[self.curve], self.t)?;
        }
        let here = [self.tangent, self.normal, cross(self.tangent, self.normal)];
        // The rotation taking the first frame to this one: `H Fᵀ`.
        let mut columns = [[0.0; 3]; 3];
        for (j, column) in columns.iter_mut().enumerate() {
            for (i, value) in column.iter_mut().enumerate() {
                *value = (0..3).map(|k| here[k][i] * self.first[k][j]).sum();
            }
        }
        Quaternion::from_rotation_columns(
            columns.map(|c| Vector3::from_array(c.map(Scalar::from_f64))),
        )
    }
}

/// At the points `points`: the seed at the first, or at `from`. Instance
/// `k` — the `k`-th point, counting from one — labelled `k`; a point where
/// the seed is gets none.
fn at_points(
    reader: &Reader,
    points: &[PointsRef],
    from: Option<&PointsRef>,
) -> GeopResult<Vec<(String, Pose)>> {
    if points.is_empty() {
        return Err(GeopError::new("pick the points to pattern at"));
    }
    let found = resolve_points(reader, points)?;
    let (seed, first) = match from {
        Some(from) => {
            let [(_, at)] = resolve_points(reader, std::slice::from_ref(from))?[..] else {
                return Err(GeopError::new(
                    "the seed is at one point: pick a point, not a sketch of several",
                ));
            };
            (at, 0)
        }
        None => (found[0].1, 1),
    };
    let mut motions = Vec::new();
    for (k, (_, at)) in found.iter().enumerate().skip(first) {
        if at.could_be_equal(&seed) {
            continue;
        }
        let label = (k + 1 - first).to_string();
        motions.push((label, Pose::new(at.sub(&seed), Quaternion::identity())?));
    }
    Ok(motions)
}

/// At the offsets `rows`, instance `k` the `k`-th row, counting from one.
fn from_table(reader: &Reader, rows: &[Offset]) -> GeopResult<Vec<(String, Pose)>> {
    (rows.iter().enumerate())
        .map(|(k, row)| {
            let ctx = with_context!("row {} of the table", k + 1);
            let offset = [&row.x, &row.y, &row.z]
                .map(|v| v.resolve(reader).map(Scalar::from_f64))
                .into_iter()
                .collect::<GeopResult<Vec<_>>>()
                .with_context(ctx)?;
            let offset = Vector3::from_array([offset[0], offset[1], offset[2]]);
            Ok((
                (k + 1).to_string(),
                Pose::new(offset, Quaternion::identity())?,
            ))
        })
        .collect()
}

/// At the holes of `features`: the seed in the first, instance `k` in the
/// `k`-th from there, counting from one, turned as it is.
fn at_holes(reader: &Reader, features: &[FeatureRef]) -> GeopResult<Vec<(String, Pose)>> {
    if features.is_empty() {
        return Err(GeopError::new(
            "pick a hole's wall: the holes it was drilled with are patterned at",
        ));
    }
    let holes = holes_of(reader, features)?;
    let first = pose_of(&holes[0].1.seat_frame()?)?.inverse();
    (holes.iter().enumerate().skip(1))
        .map(|(k, (_, hole))| Ok((k.to_string(), pose_of(&hole.seat_frame()?)?.compose(&first))))
        .collect()
}

/// Filling the flat face `face` on a grid `spacing` apart through where
/// the seed is (`anchor`, projected onto the face's plane), every instance
/// `margin` or more inside the face's edges: instance `i.j` the `i`-th
/// along the plane's `u` and the `j`-th along its `v`.
fn fill(
    reader: &Reader,
    face: Option<&ElementReference<NamedModel, Plane>>,
    spacing: &NumericInput,
    margin: &NumericInput,
    staggered: bool,
    anchor: &dyn Fn() -> GeopResult<Vector3>,
) -> GeopResult<Vec<(String, Pose)>> {
    let face = face.ok_or_else(|| GeopError::new("pick the flat face to fill"))?;
    let step = spacing.resolve(reader)?;
    if !(step.is_finite() && step > 0.0) {
        return Err(GeopError::new(format!(
            "a spacing of {step} fills nothing: give one greater than zero"
        )));
    }
    let margin = margin.resolve(reader)?;
    if !(margin.is_finite() && margin >= 0.0) {
        return Err(GeopError::new(format!(
            "a margin of {margin}: give one of zero or more"
        )));
    }
    let instance = face.instance();
    let part = match instance.is_empty() {
        true => reader.clone(),
        false => reader.placed_part(instance)?,
    };
    let model = read_model(&part)?;
    let face_id = model.face_id(face.name())?;
    let plane: CoordinateSystem = model.frame_of_face(face_id)?;
    let placed = located(reader, instance)?;
    let (to_part, to_here) = (placed.inverse().motion(), placed.motion());
    // The grid's origin: the seed's anchor, onto the face's plane.
    let origin = plane.uv_to_xyz(&plane.to_uv(&to_part.apply(&anchor()?)));
    // Its rows, along the plane's `u`, a step apart; staggered, each row
    // half a step on from the last, and the rows nearer: a triangle's
    // height apart.
    let row_step = match staggered {
        true => step * 3f64.sqrt() / 2.0,
        false => step,
    };
    let topology = model.topology();
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for edge in topology.face_edges(face_id) {
        for cp in &topology.get_edge(edge)?.curve.control_points {
            let uv = plane.to_uv(&cp.dehomogenized()?).sub(&plane.to_uv(&origin));
            for k in 0..2 {
                lo[k] = lo[k].min(uv[k].lower().to_f64());
                hi[k] = hi[k].max(uv[k].upper().to_f64());
            }
        }
    }
    let range =
        |lo: f64, hi: f64, step: f64| (lo / step).floor() as i64..=(hi / step).ceil() as i64;
    let rows = range(lo[1], hi[1], row_step);
    let columns = range(lo[0] - step, hi[0] + step, step);
    let cells = (rows.end() - rows.start() + 1) * (columns.end() - columns.start() + 1);
    if cells as usize > 4 * MOST {
        return Err(GeopError::new(format!(
            "a spacing of {step} would try {cells} places on the face: space them further apart"
        )));
    }
    let surface = &topology.get_face(face_id)?.surface;
    let edges: Vec<_> = topology.face_edges(face_id).collect();
    let margin = Scalar::from_f64(margin);
    let mut motions = Vec::new();
    for j in rows {
        let shift = match staggered && j.rem_euclid(2) == 1 {
            true => step / 2.0,
            false => 0.0,
        };
        for i in columns.clone() {
            if (i, j) == (0, 0) {
                continue;
            }
            let (u, v) = (i as f64 * step + shift, j as f64 * row_step);
            let offset = plane
                .u()
                .prod_scalar(Scalar::from_f64(u))
                .add(&plane.v().prod_scalar(Scalar::from_f64(v)));
            let at = origin.add(&offset);
            let Some((su, sv)) =
                surface_could_contain(surface, &at, MAX_NODES, min_subdivision_size())?
            else {
                continue;
            };
            let inside = face_contains(
                topology,
                face_id,
                su,
                sv,
                MAX_NODES,
                min_subdivision_size(),
                FILL_SEED,
            )?;
            if inside != PointClassification::Inside {
                continue;
            }
            if margin.definitely_greater(Scalar::ZERO) {
                let mut clear = true;
                for &edge in &edges {
                    let curve = topology.get_edge(edge)?.curve.clone();
                    let near = closest_points(&DistanceTo::Point(at), &DistanceTo::Curve(curve))?;
                    // Kept only where it is surely clear of the edge.
                    if !near.distance.lower().definitely_greater(margin)
                        && !near.distance.lower().could_be_equal(margin)
                    {
                        clear = false;
                        break;
                    }
                }
                if !clear {
                    continue;
                }
            }
            let motion = Pose::new(to_here.rotate(&offset), Quaternion::identity())?;
            motions.push((format!("{i}.{j}"), motion));
        }
    }
    Ok(motions)
}

/// Seeds the ray directions a fill asks its face with.
const FILL_SEED: u64 = 0x4649_4C4C_0000_0001;
