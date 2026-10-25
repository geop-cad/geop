//! [`GivenDatum`]: a datum at given coordinates — what a test sketches on,
//! patterns along or revolves round instead of building the datum from
//! picked entities.

use geop_core_editor::{
    edit_form::{EditForm, FormState},
    editable::{Context, OperationGroup, OperationInfo, OperationTier},
    part_operation,
    part_operation::PartOperation,
};
use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    primitives::{CoordinateSystem, Datum, DatumKind},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_program::{part::StepContext, reader::Reader};
use geop_ops_model::datums::{Datums, datums};
use geop_ops_model::frame_along;
use serde::{Deserialize, Serialize};

/// A datum by its frame: a point at `origin`, an axis or a plane's normal
/// along `z`, a coordinate system with its `x` along `x` too.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GivenDatumArgs {
    pub kind: DatumKind,
    pub origin: [f64; 3],
    /// The frame's `x`, of any length; for a point, an axis or a plane, any
    /// direction across `z`.
    pub x: [f64; 3],
    /// The frame's `z`, of any length: the axis, or the plane's normal.
    pub z: [f64; 3],
}

/// The point `p`, as sharp scalars.
fn point(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

impl GivenDatumArgs {
    /// The datum of `kind` whose frame is at `origin`, its `z` along `z`
    /// and its `x` across it, as `frame_along` picks it.
    fn along(kind: DatumKind, origin: [f64; 3], z: [f64; 3]) -> Self {
        let frame = frame_along(point(origin), &point(z)).expect("a direction, not zero");
        GivenDatumArgs {
            kind,
            origin,
            x: frame.u().to_array().map(|c| c.to_f64()),
            z,
        }
    }

    /// The plane through `origin` with the normal `normal`.
    pub fn plane(origin: [f64; 3], normal: [f64; 3]) -> Self {
        Self::along(DatumKind::Plane, origin, normal)
    }

    /// The axis through `origin` along `direction`.
    pub fn axis(origin: [f64; 3], direction: [f64; 3]) -> Self {
        Self::along(DatumKind::Axis, origin, direction)
    }

    /// The point `origin`, its frame the world's moved there.
    pub fn point(origin: [f64; 3]) -> Self {
        GivenDatumArgs {
            kind: DatumKind::Point,
            origin,
            x: [1.0, 0.0, 0.0],
            z: [0.0, 0.0, 1.0],
        }
    }

    /// The coordinate system at `origin` with its `x` along `x` and its
    /// `z` along `z`.
    pub fn frame(origin: [f64; 3], x: [f64; 3], z: [f64; 3]) -> Self {
        GivenDatumArgs {
            kind: DatumKind::Frame,
            origin,
            x,
            z,
        }
    }

    /// The datum it describes: `x` and `z` are directions, so their lengths
    /// do not reach the frame (`try_from_xz` keeps them, and a plane given
    /// with the normal `(0, -1, 1)` would stretch what is sketched on it by
    /// `sqrt 2`).
    fn datum(&self) -> GeopResult<Datum> {
        let x = point(self.x).normalize()?;
        let z = point(self.z).normalize()?;
        let frame = CoordinateSystem::try_from_xz(point(self.origin), x, z)?;
        Ok(Datum {
            kind: self.kind,
            frame,
        })
    }
}

/// Adds the datum `args` describes to the part, named after the step. Only
/// for tests: an editor never offers it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GivenDatum;

/// The datums with the one the step `step` adds with `args`.
fn add(reader: &Reader, (step, args): &(String, GivenDatumArgs)) -> GeopResult<Datums> {
    let ctx = with_context!("given_datum({step}, {args:?})");
    let mut datums: Datums = reader.previous()?;
    datums
        .insert(step, args.datum().with_context(ctx)?)
        .with_context(ctx)?;
    Ok(datums)
}

part_operation!(GivenDatum(GivenDatumArgs));

impl PartOperation for GivenDatum {
    const KIND: &'static str = "given_datum";
    const INFO: OperationInfo = OperationInfo::new(
        "Given datum",
        "A point, an axis, a plane or a coordinate system at given coordinates (tests only).",
        OperationGroup::Sketch,
        OperationTier::Menu,
    );
    type Args = GivenDatumArgs;
    type Session = ();

    /// The plane `z = 0`.
    fn new_args(_: &Context) -> GivenDatumArgs {
        GivenDatumArgs::plane([0.0; 3], [0.0, 0.0, 1.0])
    }

    fn run(ctx: &mut StepContext, args: &GivenDatumArgs) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&datums(), &(step.to_string(), args.clone()), add)
    }

    /// Nothing: the datum is written, not edited.
    fn form<'a>(
        _: &'a Context<'a>,
        _: &GivenDatumArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, GivenDatumArgs> {
        EditForm::default()
    }
}
