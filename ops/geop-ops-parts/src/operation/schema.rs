//! A description of every operation and its arguments, for whatever edits
//! programs: a UI builds the form for an operation from its
//! [`OperationSchema`] rather than knowing the operation by hand, so a new
//! operation shows up in every editor without touching any of them.
//!
//! Written by `#[derive(OperationArgs)]` / `#[derive(Operations)]` from
//! each argument's `#[arg(...)]` and doc comment.

use geop_core_part::DatumKind;
use serde::Serialize;

use super::entity::Role;

/// What kind of value an argument holds — and so how an editor lets a user
/// enter it.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ArgKind {
    /// A real number: a length, a distance. `min`/`max` bound what a slider
    /// offers, not what is valid.
    Number {
        default: f64,
        min: f64,
        max: f64,
    },
    Bool {
        default: bool,
    },
    /// The name of a solid of the part — picked in the viewport.
    Solid,
    /// The name of a face of the part — picked in the viewport.
    Face,
    /// The name of a sketch of the part.
    Sketch,
    /// The id of a line of the sketch named by the argument `sketch`.
    SketchLine {
        sketch: &'static str,
    },
    /// One of a fixed set of values.
    Choice {
        options: &'static [&'static str],
        default: &'static str,
    },
    /// A plane to sketch on: a base plane, a planar face or a datum plane,
    /// picked in the viewport (see [`crate::EntityRef`]).
    Plane,
    /// Entities to build on — vertices, edges, faces, datums, the origin,
    /// world axes and base planes — picked in the viewport, in order (see
    /// [`crate::EntityRef`]).
    Selection,
    /// How to build a datum from the entities of the argument `selection`:
    /// one of `options`, each of which fits only some selections (see
    /// [`crate::operation::inspect_selection`]).
    Construction {
        selection: &'static str,
        options: &'static [ConstructionSchema],
    },
    /// Sketch geometry, drawn on the plane given by the argument `plane`.
    Drawing {
        plane: &'static str,
    },
    /// A new body, or a boolean with a target solid (see
    /// [`crate::operation::Combine`]): a choice of mode, and the target
    /// picked in the viewport. `sign` names the number argument whose sign
    /// picks the mode until the user chooses one: join when it is
    /// positive, cut when it is negative — an extrude up out of a face
    /// adds material, one down into it removes some.
    Combine {
        sign: Option<&'static str>,
    },
}

/// One argument of an operation.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ArgSchema {
    /// The field name, as the argument serializes.
    pub name: &'static str,
    pub doc: &'static str,
    pub kind: ArgKind,
}

/// One way to build a datum (see [`crate::Construction`]): what it builds,
/// what it needs selected — one entity per input, in any order — and the
/// values it takes besides.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ConstructionSchema {
    /// How the construction is spelled: `offset`.
    pub method: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    pub result: DatumKind,
    pub inputs: &'static [Role],
    pub params: &'static [ArgSchema],
}

/// One operation: its kind (as a program step spells it), a short label,
/// what it does, and its arguments.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct OperationSchema {
    pub kind: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    pub args: Vec<ArgSchema>,
}

/// An operation's arguments, described field by field — implemented by
/// `#[derive(OperationArgs)]`.
pub trait OperationArgs {
    fn schema() -> Vec<ArgSchema>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::PartOperation;

    /// Every operation is described, with its arguments in declaration
    /// order and their doc comments.
    #[test]
    fn schemas_describe_every_operation() {
        let schemas = PartOperation::schemas();
        let kinds: Vec<&str> = schemas.iter().map(|s| s.kind).collect();
        assert_eq!(
            kinds,
            ["add_sketch", "extrude", "revolve", "boolean", "add_datum"]
        );
        let extrude = &schemas[1];
        assert_eq!(extrude.label, "Extrude");
        let args: Vec<&str> = extrude.args.iter().map(|a| a.name).collect();
        assert_eq!(args, ["sketch", "distance", "symmetric", "combine"]);
        assert_eq!(
            extrude.args[3].kind,
            ArgKind::Combine {
                sign: Some("distance")
            }
        );
        assert_eq!(extrude.args[0].kind, ArgKind::Sketch);
        assert!(extrude.args[1].doc.starts_with("How far"));
        assert_eq!(schemas[0].label, "Sketch");
        serde_json::to_string(&schemas).unwrap();
    }

    /// The kind a schema gives an operation is the tag it serializes under,
    /// so an editor can build steps from schemas alone.
    #[test]
    fn schema_kinds_are_the_serialized_tags() {
        let kinds: Vec<&str> = PartOperation::schemas().iter().map(|s| s.kind).collect();
        for (_, program) in crate::examples::all() {
            for step in &program.steps {
                let json = serde_json::to_value(&step.operation).unwrap();
                assert_eq!(json["operation"], step.operation.kind());
                assert!(kinds.contains(&step.operation.kind()));
            }
        }
    }
}
