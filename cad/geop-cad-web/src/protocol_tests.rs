//! The protocol the web app speaks, checked against `web/src/geop.ts`.
//!
//! The editor is driven here as the app drives it, by JSON: a command of
//! every kind, an event of every kind, every operation's dialog opened,
//! panels, tools, inspectors and exporters asked. Every command must be
//! one the kernel reads, and what the editor answers is kept, reduced to
//! its shapes, in `web/test/protocol.samples.ts`: the commands typed as
//! `geop.ts`'s `Command`, the updates as its `Update`. `tsc -b` (the web
//! build) checks them there. So a field renamed, added or dropped on one
//! side fails on the other: here, while the samples written differ from
//! the ones committed; in the web build, once they are written again
//! (`GEOP_WRITE_PROTOCOL_SAMPLES=1 cargo test -p geop protocol`) and
//! `geop.ts` says otherwise.

use std::collections::{BTreeMap, BTreeSet};

use geop_cad_base::Command;
use serde_json::{Value, json};

/// The samples' file, beside the web app's tests.
const SAMPLES: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../web/test/protocol.samples.ts"
);

/// How many kinds of [`Command`] there are: what [`kind`] tells apart.
const COMMAND_KINDS: usize = 24;

/// The kind of `command`. A kind added to [`Command`] does not compile
/// here until it is named, and [`COMMAND_KINDS`] counted up, and then
/// needs a sample in [`session`].
fn kind(command: &Command) -> &'static str {
    match command {
        Command::Show => "show",
        Command::New { .. } => "new",
        Command::Open { .. } => "open",
        Command::Event { .. } => "event",
        Command::Commit => "commit",
        Command::Cancel => "cancel",
        Command::Preview { .. } => "preview",
        Command::Remove { .. } => "remove",
        Command::Move { .. } => "move",
        Command::Copy { .. } => "copy",
        Command::Paste { .. } => "paste",
        Command::Seek { .. } => "seek",
        Command::Load { .. } => "load",
        Command::Files { .. } => "files",
        Command::LoadExample { .. } => "load_example",
        Command::LoadWorkspaceExample { .. } => "load_workspace_example",
        Command::Undo => "undo",
        Command::Redo => "redo",
        Command::Panel { .. } => "panel",
        Command::Visibility { .. } => "visibility",
        Command::Tool { .. } => "tool",
        Command::Inspect { .. } => "inspect",
        Command::Export { .. } => "export",
        Command::Crash => "crash",
    }
}

/// A pointer at `origin` looking along `dir`, reaching as a cone does.
fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Value {
    json!({ "ray": { "origin": origin, "dir": dir }, "reach": { "type": "cone", "slope": 0.01 } })
}

/// The commands of a session, as the app sends them: each kind at least
/// once, and every event and value a dialog is sent. `kinds` are the
/// operations offered, each started on the drilled box and cancelled.
fn session(kinds: &[&str]) -> Vec<Value> {
    let above = pointer([1.0, 1.0, 5.0], [0.0, 0.0, -1.0]);
    let tube = json!({ "ray": { "origin": [1.0, 1.0, 5.0], "dir": [0.0, 0.0, -1.0] },
        "reach": { "type": "tube", "radius": 0.01 } });
    let event = |event: Value| json!({ "command": "event", "event": event });
    let dialog =
        |key: &str, value: Value| event(json!({ "type": "dialog", "key": key, "value": value }));
    let mut commands = vec![
        json!({ "command": "show" }),
        json!({ "command": "load_example", "name": "box_with_drill_hole" }),
    ];
    for kind in kinds {
        commands.push(json!({ "command": "new", "kind": kind }));
        commands.push(json!({ "command": "cancel" }));
    }
    commands.extend([
        json!({ "command": "open", "id": "hole" }),
        event(json!({ "type": "hover", "pointer": above, "shift": false })),
        event(json!({ "type": "hover", "pointer": tube, "shift": true })),
        event(json!({ "type": "click", "pointer": above, "button": "primary", "double": false, "shift": false })),
        event(json!({ "type": "click", "pointer": above, "button": "secondary", "double": true, "shift": true })),
        event(json!({ "type": "drag", "from": above, "to": pointer([1.5, 1.0, 5.0], [0.0, 0.0, -1.0]), "done": true, "shift": false })),
        event(json!({ "type": "leave" })),
        event(json!({ "type": "key", "key": "Escape" })),
        dialog("distance", json!({ "type": "number", "value": 0.25 })),
        dialog("distance", json!({ "type": "text", "value": "0.5" })),
        dialog("symmetric", json!({ "type": "bool", "value": true })),
        dialog("output", json!({ "type": "choice", "value": "solid" })),
        dialog("sketch", json!({ "type": "press" })),
        dialog("sketch", json!({ "type": "remove_at", "value": 0 })),
        dialog("sketch", json!({ "type": "clear" })),
        dialog("sketch", json!({ "type": "remove" })),
        json!({ "command": "preview", "preview": false }),
        json!({ "command": "cancel" }),
        // A table in a popup: every edit of it.
        json!({ "command": "new", "kind": "lookup_table" }),
        dialog("input", json!({ "type": "text", "value": "size" })),
        dialog("table", json!({ "type": "table", "value": { "edit": "cell", "row": 0, "column": 0, "text": "M4" } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "paste", "row": 0, "column": 1, "cells": [["4"], ["5"]] } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "add_row" } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "move_row", "row": 2, "to": 0 } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "select_row", "row": 1 } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "remove_row", "row": 0 } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "add_column" } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "column", "column": 2, "text": "k" } })),
        dialog("table", json!({ "type": "table", "value": { "edit": "remove_column", "column": 1 } })),
        json!({ "command": "cancel" }),
        json!({ "command": "new", "kind": "extrude" }),
        json!({ "command": "commit" }),
        json!({ "command": "move", "id": "extrude1", "index": 0 }),
        json!({ "command": "copy", "id": "hole" }),
        json!({ "command": "remove", "id": "extrude1" }),
        json!({ "command": "undo" }),
        json!({ "command": "redo" }),
        json!({ "command": "seek", "marker": 2 }),
        json!({ "command": "seek", "marker": null }),
        json!({ "command": "visibility", "name": "origin", "visible": false }),
        json!({ "command": "inspect", "inspector": "mass_properties" }),
        json!({ "command": "tool", "key": "pick:mass_properties" }),
        event(json!({ "type": "click", "pointer": above, "button": "primary", "double": false, "shift": false })),
        json!({ "command": "tool", "key": null }),
        json!({ "command": "export", "exporter": "stl" }),
        json!({ "command": "export", "exporter": "step", "format": "step", "options": null, "date": "2026-10-16" }),
        json!({ "command": "panel", "key": "joints", "payload": { "command": "set", "parameter": "angle", "value": 30.0 } }),
        json!({ "command": "paste", "text": "not steps" }),
        json!({ "command": "load", "program": { "steps": [] }, "path": "parts/empty.geop" }),
        json!({ "command": "files", "files": { "parts/notes.step": "ISO-10303-21;", "gone.geop": null } }),
        json!({ "command": "load_workspace_example", "name": "pin_in_plate", "folder": "pins" }),
        json!({ "command": "tool", "key": "drag" }),
        event(json!({ "type": "hover", "pointer": pointer([1.0, 1.0, 5.0], [0.0, 0.0, -1.0]), "shift": false })),
        json!({ "command": "tool", "key": null }),
        json!({ "command": "open", "id": "pin" }),
        json!({ "command": "cancel" }),
        json!({ "command": "new", "kind": "no_such_operation" }),
    ]);
    commands
}

/// The commands sampled that are not sent in the session: a crash ends it.
fn unsent() -> Vec<Value> {
    vec![json!({ "command": "crash" })]
}

/// `value` reduced to its shape: every number 0, which no type tells from
/// another and whose last bits differ between machines; long text cut, as
/// no literal type is long; and of every array longer than a tuple, one
/// element of each shape it holds, the least by its text — the order of
/// what the kernel lists need not be stable, the shapes are.
fn shape_of(value: &Value) -> Value {
    match value {
        Value::Number(_) => json!(0),
        Value::String(text) if text.chars().count() > 40 => {
            Value::String(text.chars().take(40).collect())
        }
        Value::Array(items) => {
            let items: Vec<Value> = items.iter().map(shape_of).collect();
            if items.len() <= 4 {
                return Value::Array(items);
            }
            let mut least: BTreeMap<String, String> = BTreeMap::new();
            for item in &items {
                let text = item.to_string();
                let slot = least
                    .entry(fingerprint(item))
                    .or_insert_with(|| text.clone());
                if text < *slot {
                    *slot = text;
                }
            }
            let mut kept: Vec<String> = least.into_values().collect();
            kept.sort();
            Value::Array(
                kept.iter()
                    .map(|t| serde_json::from_str(t).unwrap())
                    .collect(),
            )
        }
        Value::Object(fields) => Value::Object(
            fields
                .iter()
                .map(|(k, v)| (k.clone(), shape_of(v)))
                .collect(),
        ),
        other => other.clone(),
    }
}

/// What tells the shapes of two values apart: their kinds, fields, and the
/// tags of tagged unions (`type`, `shape`, `part`, `command`), not their
/// numbers or text.
fn fingerprint(value: &Value) -> String {
    match value {
        Value::Null => "null".into(),
        Value::Bool(_) => "bool".into(),
        Value::Number(_) => "number".into(),
        Value::String(_) => "string".into(),
        Value::Array(items) => {
            let shapes: BTreeSet<String> = items.iter().map(fingerprint).collect();
            format!("[{}]", shapes.into_iter().collect::<Vec<_>>().join("|"))
        }
        Value::Object(fields) => {
            let fields: Vec<String> = (fields.iter())
                .map(|(k, v)| match (k.as_str(), v) {
                    ("type" | "shape" | "part" | "command", Value::String(tag)) => {
                        format!("{k}={tag}")
                    }
                    _ => format!("{k}:{}", fingerprint(v)),
                })
                .collect();
            format!("{{{}}}", fields.join(","))
        }
    }
}

/// The samples' file: `commands` typed as `Command`, `updates` as
/// `Update`, and every kind of command, event and value `geop.ts` declares
/// required to have a sample.
fn samples_file(commands: &[Value], updates: &[Value]) -> String {
    // One to a line: small enough to read, and to diff.
    let list = |values: &[Value]| {
        let lines: Vec<String> = values.iter().map(|v| format!("  {v},\n")).collect();
        format!("[\n{}]", lines.concat())
    };
    format!(
        "// Generated by `cad/geop-cad-web/src/protocol_tests.rs` — do not edit. Write it\n\
         // again with `GEOP_WRITE_PROTOCOL_SAMPLES=1 cargo test -p geop protocol`.\n\
         //\n\
         // What the kernel reads and answers, typed as `src/geop.ts` types it: the\n\
         // web build (`tsc -b`) fails where the two disagree.\n\
         import type {{ Command, EditEvent, Update, Value }} from \"../src/geop.ts\";\n\
         \n\
         export const commands = {} satisfies Command[];\n\
         \n\
         export const updates = {} satisfies Update[];\n\
         \n\
         type Sent = (typeof commands)[number];\n\
         type Events = Extract<Sent, {{ command: \"event\" }}>[\"event\"];\n\
         type Values = Extract<Events, {{ type: \"dialog\" }}>[\"value\"];\n\
         /** What `geop.ts` declares that no sample is of: never. */\n\
         type Unsampled =\n  \
           | Exclude<Command[\"command\"], Sent[\"command\"]>\n  \
           | Exclude<EditEvent[\"type\"], Events[\"type\"]>\n  \
           | Exclude<Value[\"type\"], Values[\"type\"]>;\n\
         export const everyKindIsSampled: [Unsampled] extends [never] ? true : Unsampled = true;\n",
        list(commands),
        list(updates),
    )
}

/// Every command of the session and every update it is answered by, and
/// the commands not sent, are what `geop.ts` says they are (see the module
/// documentation).
#[test]
fn protocol_samples_are_current() {
    let registry = geop_cad_base::registry().unwrap();
    let kinds: Vec<&str> = registry.infos().iter().map(|i| i.kind).collect();
    let sent = session(&kinds);

    let mut read = BTreeSet::new();
    for command in sent.iter().chain(&unsent()) {
        let read_as: Command = serde_json::from_value(command.clone())
            .unwrap_or_else(|e| panic!("the kernel does not read {command}: {e}"));
        read.insert(kind(&read_as));
    }
    assert_eq!(
        read.len(),
        COMMAND_KINDS,
        "a kind of command has no sample: {read:?}"
    );

    // Of every field of an update, each shape it is sent in: an update
    // sends what changed, so which fields come together says nothing.
    let mut editor = geop_cad_base::editor();
    let mut fields: BTreeMap<String, BTreeMap<String, Value>> = BTreeMap::new();
    for command in &sent {
        let answer = editor.handle_json(&command.to_string()).unwrap();
        let Value::Object(update) = shape_of(&serde_json::from_str(&answer).unwrap()) else {
            panic!("an update is an object: {answer}");
        };
        for (field, value) in update {
            let shapes = fields.entry(field).or_default();
            shapes.entry(fingerprint(&value)).or_insert(value);
        }
    }
    let nothing: serde_json::Map<String, Value> = fields
        .keys()
        .map(|field| (field.clone(), Value::Null))
        .collect();
    let updates: Vec<Value> = (fields.iter())
        .flat_map(|(field, shapes)| shapes.values().map(move |shape| (field, shape)))
        .filter(|(_, shape)| !shape.is_null())
        .map(|(field, shape)| {
            let mut update = nothing.clone();
            update.insert(field.clone(), shape.clone());
            Value::Object(update)
        })
        .chain([Value::Object(nothing.clone())])
        .collect();
    // The commands, once each per shape.
    let mut commands: BTreeMap<String, Value> = BTreeMap::new();
    for command in sent.iter().chain(&unsent()) {
        let command = shape_of(command);
        commands.entry(fingerprint(&command)).or_insert(command);
    }
    let commands: Vec<Value> = commands.into_values().collect();

    let written = samples_file(&commands, &updates);
    if std::env::var_os("GEOP_WRITE_PROTOCOL_SAMPLES").is_some() {
        std::fs::write(SAMPLES, &written).unwrap();
    }
    let committed = std::fs::read_to_string(SAMPLES).unwrap_or_default();
    assert!(
        committed == written,
        "{SAMPLES} is not what the kernel reads and answers: write it again with \
         `GEOP_WRITE_PROTOCOL_SAMPLES=1 cargo test -p geop protocol`, and make \
         `web/src/geop.ts` agree with it (`npm run build` in web/)"
    );
}
