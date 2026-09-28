//! What the editor's operations are described as.

use geop_ops::{ArgKind, Operations};

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
