//! A part's datums: reference geometry it is built *with*, not *of* — a
//! point, an axis, a plane or a frame, by name — in the one target
//! [`datums`], which the steps that add datums revise ([`Datums`]). Every
//! part also has [`ORIGIN`], the world's origin and axes, which no step
//! adds.
//!
//! A datum is an element of the target as a whole ([`AnyDatum`]), and as
//! what it is: a point datum a [`Point`], an axis a [`Line`], a plane a
//! [`Plane`]. A frame's components are elements of their own, named after
//! it: `origin:z` its `z` axis, `origin:xy` the plane normal to it (see
//! [`component_name`]); a frame itself is a point, its origin. A part no
//! step added a datum to has the origin all the same: the target reads as
//! it where no step made it ([`HasElements::undefined`]).

// Library code reports what goes wrong as an error, never by panicking;
// tests may unwrap.
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::indexing_slicing,
        clippy::todo,
        clippy::unimplemented,
        clippy::unreachable
    )
)]

use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    reader::Reader,
    target::{Target, TargetReference},
    values::Design,
};
use std::{borrow::Cow, collections::BTreeMap, sync::LazyLock};

use geop_core_editor::selectable::SelectableElement;
use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{CoordinateSystem, Datum, DatumComponent, DatumKind, FrameAxis},
    vector::Vector3,
};
use geop_ops_model::{Line, Plane, Point};
use geop_ops_view::{PartView, ViewDatum, drawn};

/// The name of the frame datum every part has: the world's origin and
/// axes, and the three planes between them.
pub const ORIGIN: &str = "origin";

/// The name the part's datums are a target by: there is one.
const DATUMS: &str = "datums";

/// What separates a frame's name from its component's in an element's
/// name: `origin:xy`. No step id has it, so no datum's name does.
const COMPONENT: char = ':';

/// The datums the steps of a part added, by name.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Datums(BTreeMap<String, Datum<Design>>);

/// The part's datums: revised by every step that adds one.
pub fn datums() -> TargetReference<Datums> {
    TargetReference::new(DATUMS)
}

/// The datum `name`, or a component of one (see [`component_name`]), of
/// the kind `K`: what a step's arguments refer to it by.
pub fn datum<K>(name: impl Into<String>) -> ElementReference<Datums, K> {
    ElementReference::new(datums(), name)
}

/// The datums of a part no step added any to: none but [`ORIGIN`], which
/// every part has to build on.
fn origin() -> &'static Datums {
    static ORIGIN_ONLY: LazyLock<Datums> = LazyLock::new(Datums::default);
    &ORIGIN_ONLY
}

/// The datums as `reader` reads them: none but [`ORIGIN`], before any step
/// defines them.
pub fn read_datums<'a>(reader: &Reader<'a>) -> GeopResult<Cow<'a, Datums>> {
    if !reader.defines(&datums()) {
        return Ok(Cow::Borrowed(origin()));
    }
    Ok(Cow::Borrowed(reader.retrieve_element(
        &ElementReference::<Datums, Datums>::whole(datums()),
    )?))
}

/// The name of the component `component` of the frame `frame`: `origin:z`
/// for an axis, `origin:xy` for the plane normal to `z`.
pub fn component_name(frame: &str, component: DatumComponent) -> String {
    let component = match component {
        DatumComponent::Axis(axis) => axis.letter().to_string(),
        DatumComponent::Plane(normal) => {
            let [a, b] = in_plane(normal);
            format!("{}{}", a.letter(), b.letter())
        }
    };
    format!("{frame}{COMPONENT}{component}")
}

/// The axes spanning the plane normal to `normal`, in order.
fn in_plane(normal: FrameAxis) -> [FrameAxis; 2] {
    match normal {
        FrameAxis::X => [FrameAxis::Y, FrameAxis::Z],
        FrameAxis::Y => [FrameAxis::Z, FrameAxis::X],
        FrameAxis::Z => [FrameAxis::X, FrameAxis::Y],
    }
}

/// The component `text` names (see [`component_name`]).
fn component(text: &str) -> GeopResult<DatumComponent> {
    for axis in FrameAxis::ALL {
        if text == axis.letter() {
            return Ok(DatumComponent::Axis(axis));
        }
        let [a, b] = in_plane(axis);
        if text == format!("{}{}", a.letter(), b.letter()) {
            return Ok(DatumComponent::Plane(axis));
        }
    }
    Err(GeopError::new(format!(
        "{text:?} is no component of a frame: name an axis, `x`, or a plane, `xy`"
    )))
}

/// The plane of [`ORIGIN`] normal to `normal`.
pub fn origin_plane(normal: FrameAxis) -> ElementReference<Datums, Plane> {
    ElementReference::new(
        datums(),
        component_name(ORIGIN, DatumComponent::Plane(normal)),
    )
}

/// The axis `axis` of [`ORIGIN`].
pub fn origin_axis(axis: FrameAxis) -> ElementReference<Datums, Line> {
    ElementReference::new(datums(), component_name(ORIGIN, DatumComponent::Axis(axis)))
}

impl Datums {
    /// The datum `name`: one a step added, [`ORIGIN`], or a component of a
    /// frame (see [`component_name`]) as a datum of its own.
    pub fn get(&self, name: &str) -> GeopResult<Datum<Design>> {
        let (frame, component) = match name.split_once(COMPONENT) {
            Some((frame, text)) => (frame, Some(self::component(text)?)),
            None => (name, None),
        };
        let datum = match frame {
            ORIGIN => Datum {
                kind: DatumKind::Frame,
                frame: CoordinateSystem::world_at(Vector3::zero()),
            },
            _ => (self.0.get(frame).cloned()).ok_or_else(|| {
                GeopError::new(format!(
                    "there is no datum {frame:?} (datums: {:?})",
                    self.names()
                ))
            })?,
        };
        match component {
            Some(component) => datum.component(component),
            None => Ok(datum),
        }
    }

    /// Adds `datum` as `name`: an error for a name taken.
    pub fn insert(&mut self, name: &str, datum: Datum<Design>) -> GeopResult<()> {
        if name == ORIGIN || name.contains(COMPONENT) || self.0.contains_key(name) {
            return Err(GeopError::new(format!("there is a datum {name:?} already")));
        }
        self.0.insert(name.to_string(), datum);
        Ok(())
    }

    /// The names of the datums the steps added, sorted.
    pub fn names(&self) -> Vec<&str> {
        self.0.keys().map(String::as_str).collect()
    }

    /// The datums the steps added, by name, sorted.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Datum<Design>)> {
        self.0.iter().map(|(name, datum)| (name.as_str(), datum))
    }
}

impl Datums {
    /// Draws [`ORIGIN`] and every datum into `view`, a frame with its axes
    /// and planes each picked on its own.
    pub fn draw(&self, view: &mut PartView) -> GeopResult<()> {
        view.datum_target = DATUMS.to_string();
        let origin = self.get(ORIGIN)?;
        let datums = std::iter::once((ORIGIN, &origin)).chain(self.iter());
        view.datums = datums
            .map(|(name, datum)| {
                let components = match datum.kind {
                    DatumKind::Frame => (FrameAxis::ALL.into_iter())
                        .flat_map(|axis| [DatumComponent::Axis(axis), DatumComponent::Plane(axis)])
                        .map(|c| (c, component_name(name, c)))
                        .collect(),
                    _ => Vec::new(),
                };
                ViewDatum {
                    name: name.to_string(),
                    kind: datum.kind,
                    frame: datum.frame.clone(),
                    components,
                }
            })
            .collect();
        Ok(())
    }
}

/// A datum of the part, whatever it is.
pub struct AnyDatum;

impl Kind for AnyDatum {
    const NAME: &'static str = drawn::DATUM;
}

/// Every step that adds a datum adds to them.
impl Target for Datums {
    const CHAIN: bool = true;
}

/// Read whole.
impl Kind for Datums {
    const NAME: &'static str = "datums";
}

impl HasElements<Datums> for Datums {
    type Element<'a> = &'a Datums;
    const SINGLETON: Option<&'static str> = Some(DATUMS);

    fn undefined() -> Option<&'static Self> {
        Some(origin())
    }

    fn element(&self, name: &str) -> GeopResult<&Datums> {
        whole(self, name)
    }
}

impl HasElements<AnyDatum> for Datums {
    type Element<'a> = Datum<Design>;
    const SINGLETON: Option<&'static str> = Some(DATUMS);

    fn undefined() -> Option<&'static Self> {
        Some(origin())
    }

    fn element(&self, name: &str) -> GeopResult<Datum<Design>> {
        self.get(name)
    }
}

/// A point datum, or a frame by its origin.
impl HasElements<Point> for Datums {
    type Element<'a> = Vector3<Design>;
    const SINGLETON: Option<&'static str> = Some(DATUMS);

    fn undefined() -> Option<&'static Self> {
        Some(origin())
    }

    fn element(&self, name: &str) -> GeopResult<Vector3<Design>> {
        let datum = self.get(name)?;
        match datum.kind {
            DatumKind::Point | DatumKind::Frame => Ok(*datum.frame.origin()),
            kind => Err(GeopError::new(format!(
                "the datum {name:?} is an {kind:?}, not a point"
            ))),
        }
    }
}

/// An axis datum, or a frame's axis.
impl HasElements<Line> for Datums {
    type Element<'a> = Axis<Design>;
    const SINGLETON: Option<&'static str> = Some(DATUMS);

    fn undefined() -> Option<&'static Self> {
        Some(origin())
    }

    fn element(&self, name: &str) -> GeopResult<Axis<Design>> {
        let datum = self.get(name)?;
        match datum.kind {
            DatumKind::Axis => Axis::try_new(*datum.frame.origin(), *datum.frame.w()),
            kind => Err(GeopError::new(format!(
                "the datum {name:?} is a {kind:?}, not an axis"
            ))),
        }
    }
}

/// A plane datum, or a frame's plane: its frame, `w` the normal.
impl HasElements<Plane> for Datums {
    type Element<'a> = CoordinateSystem<Design>;
    const SINGLETON: Option<&'static str> = Some(DATUMS);

    fn undefined() -> Option<&'static Self> {
        Some(origin())
    }

    fn element(&self, name: &str) -> GeopResult<CoordinateSystem<Design>> {
        let datum = self.get(name)?;
        match datum.kind {
            DatumKind::Plane => Ok(datum.frame),
            kind => Err(GeopError::new(format!(
                "the datum {name:?} is a {kind:?}, not a plane"
            ))),
        }
    }
}

/// Datums are drawn as themselves, a frame's components each on its own:
/// a click on one selects it as what it is asked for, if it is that.
macro_rules! picked_as_drawn {
    ($($kind:ty),*) => {$(
        impl SelectableElement<Datums> for $kind {
            const PICKED_FROM: &'static [&'static str] = &[AnyDatum::NAME];

            fn select(
                reader: &Reader,
                hit: &PickedElement,
            ) -> Option<ElementReference<Datums, Self>> {
                (hit.target == DATUMS && hit.kind == AnyDatum::NAME).then_some(())?;
                let reference = ElementReference::new(datums(), &hit.name);
                reader.retrieve_element(&reference).ok()?;
                Some(reference)
            }

            fn drawn(
                _: &Reader,
                element: &ElementReference<Datums, Self>,
            ) -> Vec<PickedElement> {
                vec![ElementReference::<Datums, AnyDatum>::new(datums(), element.name()).picked()]
            }
        }
    )*};
}

picked_as_drawn!(AnyDatum, Point, Line, Plane);

#[cfg(test)]
mod tests {
    use geop_core_editor::selectable::{Selectable, selected};
    use geop_core_math::scalars::Scalar;
    use geop_core_program::part::Part;

    use super::*;

    /// A part of one step, adding the axis datum `a`, along `x`, to the
    /// origin's datums.
    fn part() -> Part {
        let mut part = Part::default();
        part.begin_run(&Default::default(), &Default::default());
        part.begin_step();
        (part.revise_target(&datums(), &(), |reader, _| {
            let mut datums: Datums = reader.previous()?;
            let world = CoordinateSystem::world_at(Vector3::zero());
            let axis = Datum {
                kind: DatumKind::Frame,
                frame: world,
            }
            .component(DatumComponent::Axis(FrameAxis::X))?;
            datums.insert("a", axis)?;
            Ok(datums)
        }))
        .unwrap();
        part.end_run(0);
        part
    }

    /// The origin is drawn as a frame whose components are picked on their
    /// own, as the datums name them; a datum added as itself.
    #[test]
    fn datums_are_drawn_and_picked_by_their_names() {
        use geop_core_editor::editing::{Pointer, Reach};
        use geop_core_math::primitives::Ray;
        let part = part();
        let reader = part.current();
        let mut view = PartView::new();
        read_datums(&reader).unwrap().draw(&mut view).unwrap();
        let view = view.finish();
        assert_eq!(view.datums.len(), 2);
        let v = |p: [f64; 3]| Vector3::from_array(p.map(Design::from_f64));
        // Through the origin's xy square, as drawn at a reach of 0.1.
        let pointer = Pointer {
            ray: Ray::try_new(v([0.45, 0.45, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
            reach: Reach::Tube {
                radius: Design::from_f64(0.1),
            },
        };
        let hit = view.pick(&pointer, &[drawn::DATUM.to_string()]).unwrap();
        assert_eq!(hit.element, self::hit("origin:xy"));
        let planes: Vec<ElementReference<Datums, Plane>> = selected(&reader, &[hit.element]);
        assert_eq!(planes, [origin_plane(FrameAxis::Z)]);
    }

    fn hit(name: &str) -> PickedElement {
        ElementReference::<Datums, AnyDatum>::new(datums(), name).picked()
    }

    /// The origin is a frame, its origin a point, its components an axis
    /// and a plane; a datum added is what it is, and nothing else.
    #[test]
    fn datums_are_what_they_are() {
        let part = part();
        let reader = part.current();
        let origin =
            reader.retrieve_element(&ElementReference::<Datums, Point>::new(datums(), ORIGIN));
        assert_eq!(origin.unwrap(), Vector3::zero());
        let xy = reader
            .retrieve_element(&origin_plane(FrameAxis::Z))
            .unwrap();
        assert!(xy.w().to_array()[2].could_be_equal(Design::ONE));
        let z = reader.retrieve_element(&origin_axis(FrameAxis::Z)).unwrap();
        assert!(z.direction.to_array()[2].could_be_equal(Design::ONE));
        let a = reader.retrieve_element(&ElementReference::<Datums, Line>::new(datums(), "a"));
        assert!(a.unwrap().direction.to_array()[0].could_be_equal(Design::ONE));
        let not_a_plane = ElementReference::<Datums, Plane>::new(datums(), "a");
        assert!(reader.retrieve_element(&not_a_plane).is_err());
        assert!(
            reader
                .retrieve_element(&ElementReference::<Datums, Line>::new(datums(), "b"))
                .is_err()
        );
        assert_eq!(read_datums(&reader).unwrap().names(), ["a"]);
        assert_eq!(
            component_name(ORIGIN, DatumComponent::Plane(FrameAxis::Y)),
            "origin:zx"
        );
    }

    /// A click on a datum selects it as what a field asks for, if it is
    /// that; it is lit as the datum. Saved, a reference is the datum's name.
    #[test]
    fn a_datum_is_picked_as_what_it_is() {
        let part = part();
        let reader = part.current();
        let lines: Vec<ElementReference<Datums, Line>> =
            selected(&reader, &[hit("a"), hit("origin:xy"), hit("origin:z")]);
        let names: Vec<&str> = lines.iter().map(|l| l.name()).collect();
        assert_eq!(names, ["a", "origin:z"]);
        assert_eq!(lines[1].drawn(&reader), [hit("origin:z")]);
        assert_eq!(
            serde_json::to_value(&lines[0]).unwrap(),
            serde_json::json!("a")
        );
        let mut given = Part::default();
        given.begin_run(&Default::default(), &Default::default());
        given.begin_step();
        let reader = given.current();
        assert!(read_datums(&reader).unwrap().names().is_empty());
        let plane = reader.retrieve_element(&origin_plane(FrameAxis::X));
        assert!(plane.unwrap().w().to_array()[0].could_be_equal(Design::ONE));
    }
}
