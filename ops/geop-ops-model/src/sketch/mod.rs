//! The sketch target: [`PlacedSketch`], a sketch and the plane it was placed
//! on ([`SketchPlane`]) — the value of a sketch's target ([`sketch`]) — and
//! what it is to the steps that build on it: as a whole, its profile or a path ([`PlacedSketch`], [`Path`]),
//! its plane ([`Plane`]), and its points and curves, as what they are
//! ([`Point`], [`Line`], [`Circle`], [`Round`], [`Curve`]). It is drawn
//! into the viewport as itself ([`PlacedSketch::draw`]), and each of those
//! is picked from what is drawn of it.
//!
//! What a sketch's curves are named after, when they are built into the
//! model, is the sketch and its element ids: a sketch curve `c3` of sketch
//! `K` is `K,c3`, its ends `K,p1` after the points there (see
//! [`piece_names`]).
//!
//! The step that draws a sketch is `geop_ops_sketch`'s; what reads one —
//! an extrusion, a hole, a pattern — needs only this. [`figures`] draws the
//! sketches examples and tests write down; [`fields`] are the building
//! blocks of the forms of what is drawn in a sketch.

#[cfg(test)]
mod draw_tests;
pub mod fields;
pub mod figures;

use crate::{
    Chain, Circle, Curve, Line, NamedModel, Path, Plane, Point, Profile, Round, datums::Datums,
};
use geop_core_editor::{
    one_of,
    registry::{Drawer, StructureItem},
    selectable::SelectableElement,
    view::{PartView, ViewCurve, ViewSketch, ViewSketchPoint, drawn},
};
use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve3D},
    shape::{Arc, Axis},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    reader::Reader,
    target::{Target, TargetReference},
    values::Design,
};
use geop_core_sketch::{
    CurveId, Enclosure, PointId, Shape,
    profile::{Piece, curve_nurbs, curve_polyline},
};

/// A sketch as a program holds it.
pub type Sketch = geop_core_sketch::Sketch<Design>;
/// A sketch constraint as a program holds it.
pub type Constraint = geop_core_sketch::Constraint<Design>;
/// A sketch curve's kind as a program holds it.
pub type CurveKind = geop_core_sketch::CurveKind<Design>;

one_of! {
    /// The plane a sketch is placed on: a flat face, a datum plane — the
    /// origin's among them — or another sketch's plane.
    pub enum SketchPlane -> CoordinateSystem<Design> {
        Face(ElementReference<NamedModel<Design>, Plane>),
        Datum(ElementReference<Datums, Plane>),
        Sketch(ElementReference<PlacedSketch<Design>, Plane>),
    }
}

/// Draws a part's sketches, each as the target it is, and lists them.
pub struct SketchDrawer;

impl Drawer for SketchDrawer {
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()> {
        for target in reader.targets_of::<PlacedSketch<Design>>() {
            let name = target.name().to_string();
            let reference = ElementReference::<_, PlacedSketch<Design>>::whole(target);
            reader.retrieve_element(&reference)?.draw(&name, view)?;
        }
        Ok(())
    }

    fn structure(&self, reader: &Reader) -> GeopResult<Vec<StructureItem>> {
        (reader.targets_of::<PlacedSketch<Design>>().into_iter())
            .map(|target| {
                let reference = ElementReference::<_, PlacedSketch<Design>>::whole(target);
                let placed = reader.retrieve_element(&reference)?;
                Ok(StructureItem::drawn("sketch", "Sketches", &placed.name))
            })
            .collect()
    }
}

/// A sketch together with the plane it lies on: `plane.u`/`plane.v` are
/// the sketch's `x`/`y` (unit, orthogonal), `plane.w = u x v` its unit
/// normal — and its name, the id of the step that drew it, which what is
/// built from it is named after.
///
/// The plane is resolved once, when the sketch is added — a sketch drawn on
/// a face stays where that face *was*, even after a later operation
/// reshapes or consumes the face.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedSketch<S: Scalar> {
    pub name: String,
    pub plane: CoordinateSystem<S>,
    pub sketch: Sketch,
}

/// What the target of a sketch is named, before the sketch's name.
const PREFIX: &str = "sketch:";

/// The sketch `name`: what the step of that id draws.
pub fn sketch(name: &str) -> TargetReference<PlacedSketch<Design>> {
    TargetReference::new(format!("{PREFIX}{name}"))
}

/// A sketch of the part, read whole: what is swept.
pub type SketchRef = ElementReference<PlacedSketch<Design>, PlacedSketch<Design>>;

/// The sketch `name`, read whole: what is swept.
pub fn whole_sketch(name: &str) -> ElementReference<PlacedSketch<Design>, PlacedSketch<Design>> {
    ElementReference::whole(sketch(name))
}

/// The element `element` — a point `p3` or a curve `c3` — of the sketch
/// `name`, as what `K` it is.
pub fn sketch_element<K: Kind>(
    name: &str,
    element: impl ToString,
) -> ElementReference<PlacedSketch<Design>, K>
where
    PlacedSketch<Design>: HasElements<K>,
{
    ElementReference::new(sketch(name), element.to_string())
}

/// The names of the pieces of a sketch loop or chain, and of its joints:
/// `sketch,c3` for a piece and `sketch,p2` for a joint. An open chain also
/// names its end joint.
pub fn piece_names<C>(
    sketch: &str,
    pieces: &[Piece<C>],
    closed: bool,
) -> (Vec<String>, Vec<String>) {
    let curve_names = pieces
        .iter()
        .map(|p| format!("{sketch},{}", p.name()))
        .collect();
    let mut joint_names: Vec<String> = pieces
        .iter()
        .map(|p| format!("{sketch},{}", p.start))
        .collect();
    if !closed && let Some(last) = pieces.last() {
        joint_names.push(format!("{sketch},{}", last.end));
    }
    (curve_names, joint_names)
}

/// The pieces of a loop or a chain of the sketch `sketch` as a chain named
/// after it and its elements (see [`piece_names`]).
pub fn chain_of_pieces<S: Scalar, const D: usize>(
    sketch: &str,
    pieces: Vec<Piece<NurbCurve<S, D>>>,
    closed: bool,
) -> Chain<NurbCurve<S, D>> {
    let (curve_names, joint_names) = piece_names(sketch, &pieces, closed);
    Chain {
        name: sketch.to_string(),
        curves: pieces.into_iter().map(|p| p.curve).collect(),
        curve_names,
        joint_names,
    }
}

/// The profiles of `shape`, one of the sketch `name`'s, its points at
/// `geometry`: a region's loops, outer loop first, or the one open chain —
/// each named after the sketch and its elements (see [`chain_of_pieces`]).
pub fn shape_profiles<S: Scalar>(
    name: &str,
    sketch: &Sketch,
    geometry: &Enclosure<S>,
    shape: Shape,
) -> GeopResult<Vec<Profile<S>>> {
    match shape {
        Shape::Region(region) => std::iter::once(&region.outer)
            .chain(&region.holes)
            .map(|lp| Ok(chain_of_pieces(name, lp.to_nurbs(sketch, geometry)?, true)))
            .collect(),
        Shape::Chain(chain) => Ok(vec![chain_of_pieces(
            name,
            chain.to_nurbs(sketch, geometry)?,
            false,
        )]),
    }
}

impl<S: Scalar> PlacedSketch<S> {
    /// The curve `curve` of the sketch where its plane puts it: one NURBS
    /// from its start to its end — a circle round from its seam — of the
    /// sketch solved as `geometry` says (see [`Sketch::enclose`]).
    pub fn curve_in_space(
        &self,
        curve: CurveId,
        geometry: &Enclosure<S>,
    ) -> GeopResult<NurbCurve3D<S>> {
        let pieces = curve_nurbs(&self.sketch, geometry, curve)?;
        let plane = &self.plane;
        NurbCurve::join(&pieces)?.embed(plane.origin(), plane.u(), plane.v())
    }

    /// The sketch's one open chain of curves, or its one loop, in space, as
    /// a path runs along it: its pieces and joints named as a profile's
    /// are. Fails for a sketch of more than one loop.
    pub fn path(&self) -> GeopResult<Chain<NurbCurve3D<S>>> {
        let ctx = with_context!("the curves of the sketch {:?}", self.name);
        let sketch = &self.sketch;
        let geometry = sketch.enclose::<S>().with_context(ctx)?;
        let (lp, closed) = match sketch.shape().with_context(ctx)? {
            Shape::Chain(chain) => (chain, false),
            Shape::Region(region) if region.holes.is_empty() => (region.outer, true),
            Shape::Region(_) => {
                return Err(GeopError::new(
                    "it has more than one loop: a path or a rail is one chain of curves, or one loop",
                ))
                .with_context(ctx);
            }
        };
        let pieces = lp.to_nurbs(sketch, &geometry).with_context(ctx)?;
        let plane = &self.plane;
        let (curve_names, joint_names) = piece_names(&self.name, &pieces, closed);
        Ok(Chain {
            name: self.name.clone(),
            curves: (pieces.iter())
                .map(|p| p.curve.embed(plane.origin(), plane.u(), plane.v()))
                .collect::<GeopResult<_>>()
                .with_context(ctx)?,
            curve_names,
            joint_names,
        })
    }

    /// The curve `curve` of the sketch, in space, as a chain of one curve
    /// named after the sketch and it, its ends after the points there — a
    /// closed one's `K,c3,seam`.
    pub fn curve_chain(&self, curve: CurveId) -> GeopResult<Chain<NurbCurve3D<S>>> {
        let name = &self.name;
        let ctx = with_context!("curve {curve} of the sketch {name:?}");
        let geometry = self.sketch.enclose::<S>().with_context(ctx)?;
        let class = self.sketch.point_classes();
        let in_space = self.curve_in_space(curve, &geometry).with_context(ctx)?;
        let joint_name = |p: PointId| match class.get(&p) {
            Some(class) => Ok(format!("{name},{class}")),
            None => Err(GeopError::new(format!("point {p} has no class"))),
        };
        let joint_names = match self.sketch.curve(curve)?.endpoints() {
            Some((s, e)) => vec![joint_name(s)?, joint_name(e)?],
            None => vec![format!("{name},{curve},seam")],
        };
        Ok(Chain {
            name: format!("{name},{curve}"),
            curves: vec![in_space],
            curve_names: vec![format!("{name},{curve}")],
            joint_names,
        })
    }
}

impl PlacedSketch<Design> {
    /// Draws it into `view` as the target `target`: its regions, its curves
    /// and its points — but not what it was only given to draw against, its
    /// own axes and origin and what it projected, once it is done.
    pub fn draw(&self, target: &str, view: &mut PartView) -> GeopResult<()> {
        let s = &self.sketch;
        // A sketch whose curves form no region is still a sketch, hit on its
        // curves.
        let regions = (s.regions())
            .and_then(|regions| {
                (regions.iter())
                    .map(|r| {
                        (std::iter::once(&r.outer).chain(&r.holes))
                            .map(|l| l.polyline(s))
                            .collect()
                    })
                    .collect()
            })
            .unwrap_or_default();
        let curves = (s.curves.iter())
            .filter(|(_, c)| !(c.fixed && c.construction))
            .map(|(&id, c)| {
                Ok(ViewCurve {
                    id: id.to_string(),
                    construction: c.construction,
                    polyline: curve_polyline(s, id)?,
                })
            })
            .collect::<GeopResult<_>>()?;
        let points = (s.positions().iter())
            .filter(|(id, _)| s.points.get(id).is_some_and(|p| !p.fixed))
            .map(|(id, &at)| ViewSketchPoint {
                id: id.to_string(),
                at,
            })
            .collect();
        view.sketches.push(ViewSketch {
            name: self.name.clone(),
            target: target.to_string(),
            plane: self.plane.clone(),
            regions,
            curves,
            points,
        });
        Ok(())
    }

    /// Its point `name`, `p3`, in space.
    fn point_in_space(&self, name: &str) -> GeopResult<Vector3<Design>> {
        let id: PointId = name.parse()?;
        self.sketch.point(id)?;
        let geometry = self.sketch.enclose::<Design>()?;
        let at = (geometry.points.get(&id))
            .ok_or_else(|| GeopError::new(format!("point {id} is not solved")))?;
        Ok(self.plane.uv_to_xyz(at))
    }

    /// Its curve `name`, `c3`, and what kind it is.
    fn curve_named(&self, name: &str) -> GeopResult<(CurveId, &CurveKind)> {
        let id: CurveId = name.parse()?;
        Ok((id, &self.sketch.curve(id)?.kind))
    }

    /// Its curve `name`, a circle or an arc of one, in space.
    fn arc(&self, name: &str) -> GeopResult<Arc<Design>> {
        let (id, kind) = self.curve_named(name)?;
        if !matches!(kind, CurveKind::Arc { .. } | CurveKind::Circle { .. }) {
            return Err(GeopError::new(format!("curve {id} is no circle or arc")));
        }
        let geometry = self.sketch.enclose::<Design>()?;
        (self.curve_in_space(id, &geometry)?.as_arc()?)
            .ok_or_else(|| GeopError::new(format!("curve {id} is not circular")))
    }
}

/// Produced once, by the step that draws it.
impl Target for PlacedSketch<Design> {}

/// Read whole: what is swept, as its regions or its curves.
impl Kind for PlacedSketch<Design> {
    const NAME: &'static str = drawn::SKETCH;
}

/// The elements of a sketch's target, which a pick tells from those of
/// other targets by its name.
macro_rules! sketch_elements {
    ($($kind:ty => $element:ty, |$sketch:ident, $name:ident| $resolve:expr;)*) => {$(
        impl HasElements<$kind> for PlacedSketch<Design> {
            type Element<'a> = $element;

            fn element(&self, name: &str) -> GeopResult<$element> {
                let ($sketch, $name) = (self, name);
                ($resolve).with_context(&|e: GeopError| {
                    e.with_context(format!("{} {name:?} of the sketch {:?}", <$kind>::NAME, self.name))
                })
            }

            fn names_target(name: &str) -> bool {
                name.starts_with(PREFIX)
            }
        }
    )*};
}

impl HasElements<PlacedSketch<Design>> for PlacedSketch<Design> {
    type Element<'a> = &'a PlacedSketch<Design>;

    fn element(&self, name: &str) -> GeopResult<&PlacedSketch<Design>> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}

sketch_elements! {
    Plane => CoordinateSystem<Design>, |sketch, name| {
        whole(sketch, name).map(|sketch| sketch.plane.clone())
    };
    Path => Chain<NurbCurve3D<Design>>, |sketch, name| whole(sketch, name)?.path();
    Point => Vector3<Design>, |sketch, name| sketch.point_in_space(name);
    Line => Axis<Design>, |sketch, name| {
        match sketch.curve_named(name)? {
            (_, &CurveKind::Line { start, end }) => {
                let (a, b) = (sketch.point_in_space(&start.to_string())?, sketch.point_in_space(&end.to_string())?);
                Axis::try_new(a, b.sub(&a))
            }
            (id, _) => Err(GeopError::new(format!("curve {id} is no line"))),
        }
    };
    Circle => Arc<Design>, |sketch, name| sketch.arc(name);
    Round => Axis<Design>, |sketch, name| sketch.arc(name).map(|arc| arc.circle.axis());
    Curve => Chain<NurbCurve3D<Design>>, |sketch, name| {
        let (id, _) = sketch.curve_named(name)?;
        sketch.curve_chain(id)
    };
}

/// What is picked of a sketch from what is drawn of it: the sketch as a
/// whole, its plane or its path by a hit inside it or on it; a point by a
/// point; a curve, as what it is, by a curve — each only where it is that.
macro_rules! picked_from_its_drawing {
    ($($kind:ty => $from:expr;)*) => {$(
        impl SelectableElement<PlacedSketch<Design>> for $kind {
            const PICKED_FROM: &'static [&'static str] = &[$from];

            fn select(
                reader: &Reader,
                hit: &PickedElement,
            ) -> Option<ElementReference<PlacedSketch<Design>, Self>> {
                let is_sketch = <PlacedSketch<Design> as HasElements<$kind>>::names_target(hit.target.name());
                (is_sketch && hit.kind == $from).then_some(())?;
                let reference = ElementReference::new(TargetReference::located(hit.target.clone()), &hit.name);
                reader.retrieve_element(&reference).ok()?;
                Some(reference)
            }

            fn drawn(
                _: &Reader,
                element: &ElementReference<PlacedSketch<Design>, Self>,
            ) -> Vec<PickedElement> {
                vec![PickedElement {
                    target: element.target().location().clone(),
                    kind: $from.to_string(),
                    name: element.name().to_string(),
                }]
            }
        }
    )*};
}

picked_from_its_drawing! {
    PlacedSketch<Design> => drawn::SKETCH;
    Plane => drawn::SKETCH;
    Path => drawn::SKETCH;
    Point => drawn::SKETCH_POINT;
    Line => drawn::SKETCH_CURVE;
    Circle => drawn::SKETCH_CURVE;
    Round => drawn::SKETCH_CURVE;
    Curve => drawn::SKETCH_CURVE;
}
