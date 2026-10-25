//! WebAssembly bindings for the browser editor: one editor, driven by
//! [`handle`] — and by the hosted assistant's tools, [`agent_call`].
//!
//! The browser sends every command — a step started, a click in the
//! viewport as a ray, a slider moved, undo — as JSON, and gets back what to
//! show now: the program, the part to draw, the step being edited. Every
//! decision is made by the editor (see `geop_core_editor::editor`); the browser
//! renders and forwards input.
//!
//! The assistant's tools ([`geop_core_editor::agent`]) run on the same
//! editor: a tool's answer carries every update it caused, which the
//! browser shows as it shows any other. The tools that read or write files
//! by path are not offered: the browser saves a file only when the user
//! says so.

use std::cell::RefCell;

use geop_core_editor::agent::{AgentTool, Session, tools};
use wasm_bindgen::prelude::*;

thread_local! {
    static EDITOR: RefCell<Session> = RefCell::new(Session::new(geop_cad_base::editor()));
}

#[wasm_bindgen]
extern "C" {
    /// Told every panic's message just before the module traps. The page
    /// runs the module in a worker that defines it (see
    /// `web/src/kernel.worker.ts`), and says why the kernel crashed with it.
    #[wasm_bindgen(js_name = geopPanicked)]
    fn panicked(message: &str);
}

/// Install a panic hook that forwards Rust panics to the JS console with a
/// proper stack trace, instead of an opaque "unreachable executed" trap,
/// and tells the page what panicked. Call once, right after the wasm module
/// is instantiated.
///
/// A panic aborts on wasm: nothing unwinds, so the editor stays borrowed
/// and this instance runs no further command. The page starts a fresh one
/// with the program it holds.
#[wasm_bindgen]
pub fn init_panic_hook() {
    std::panic::set_hook(Box::new(|info| {
        console_error_panic_hook::hook(info);
        panicked(&info.to_string());
    }));
}

/// `f` of the editor, unless a panic left it borrowed.
fn with_editor<T>(f: impl FnOnce(&mut Session) -> Result<T, String>) -> Result<T, String> {
    EDITOR.with(|editor| {
        f(&mut *editor.try_borrow_mut().map_err(|_| {
            "the kernel crashed in an earlier command: start a fresh one".to_string()
        })?)
    })
}

fn handle_json(command: &str) -> Result<String, String> {
    with_editor(|session| session.handle_json(command))
}

/// The tools the browser offers an agent: those that take no path.
fn browser_tools() -> Vec<AgentTool> {
    tools().into_iter().filter(|tool| !tool.files).collect()
}

/// The tools the browser offers the hosted assistant, as the Anthropic
/// Messages API takes them: a JSON array of `{name, description,
/// input_schema}`.
#[wasm_bindgen]
pub fn agent_tools() -> String {
    serde_json::Value::Array(browser_tools().iter().map(AgentTool::anthropic).collect()).to_string()
}

fn agent_call_json(name: &str, input: &str) -> Result<String, String> {
    let input: serde_json::Value =
        serde_json::from_str(input).map_err(|e| format!("reading the input of `{name}`: {e}"))?;
    with_editor(|session| {
        let answer = session.call(name, &input, None);
        serde_json::to_string(&answer).map_err(|e| format!("writing the answer of `{name}`: {e}"))
    })
}

/// Run the agent's tool `name` with `input` (JSON) on the editor the page
/// shows, and return what it answered: JSON `{text, is_error, updates}`,
/// `updates` being every `Update` the editor sent while it ran, to show in
/// order. A tool that takes a path is refused, as an error answer.
#[wasm_bindgen]
pub fn agent_call(name: &str, input: &str) -> Result<String, JsValue> {
    agent_call_json(name, input).map_err(|e| JsValue::from_str(&e))
}

/// Apply a command (JSON `geop_core_editor::editor::Command`) and return what to show
/// now (JSON `geop_core_editor::editor::Update`). A refused command says why in the
/// update's `error`, and changes nothing.
#[wasm_bindgen]
pub fn handle(command: &str) -> Result<String, JsValue> {
    handle_json(command).map_err(|e| JsValue::from_str(&e))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(command: serde_json::Value) -> serde_json::Value {
        serde_json::from_str(&handle_json(&command.to_string()).unwrap()).unwrap()
    }

    /// The editor's loop through JSON: an example loaded and drawn with its
    /// origin, then a new sketch started, its plane picked as a ray, a line
    /// drawn in it, and the sketch committed.
    #[test]
    fn a_sketch_is_drawn_through_json() {
        let loaded = send(serde_json::json!({
            "command": "load_example", "name": "box_with_drill_hole",
        }));
        assert!(loaded["error"].is_null(), "{loaded}");
        let part = &loaded["scene"]["part"];
        assert!(!part["faces"].as_array().unwrap().is_empty());
        assert_eq!(part["datums"][0]["name"], "origin");
        assert_eq!(
            part["datums"][0]["frame"]["w"],
            serde_json::json!([0.0, 0.0, 1.0])
        );

        let empty = send(serde_json::json!({"command": "load", "program": {"steps": []}}));
        // Drawn as big as a robot's part, not 1 mm: lengths are millimetres
        // (`geop_core_editor::view::EMPTY_EXTENT`).
        assert_eq!(
            empty["scene"]["part"]["extent"]["size"],
            serde_json::json!(100.0)
        );
        let started = send(serde_json::json!({"command": "new", "kind": "add_sketch"}));
        let step = &started["step"];
        assert_eq!(step["presentation"]["dialog"][0]["type"], "reference");
        assert!(step["presentation"]["focus"].is_null());

        let pointer = |origin: [f64; 3], dir: [f64; 3]| {
            serde_json::json!({
                "ray": {"origin": origin, "dir": dir},
                "reach": {"type": "tube", "radius": 0.009},
            })
        };
        let click = |pointer: serde_json::Value| {
            serde_json::json!({
                "command": "event",
                "event": {"type": "click", "pointer": pointer},
            })
        };
        // The origin's xy plane's square, from above.
        let drawing = send(click(pointer([0.04, 0.04, 10.0], [0.0, 0.0, -1.0])));
        assert_eq!(
            drawing["step"]["presentation"]["focus"]["w"],
            serde_json::json!([0.0, 0.0, 1.0])
        );
        send(serde_json::json!({
            "command": "event", "event": {"type": "key", "key": "l"},
        }));
        send(click(pointer([0.5, 0.5, 10.0], [0.0, 0.0, -1.0])));
        let drawn = send(click(pointer([1.5, 0.5, 10.0], [0.0, 0.0, -1.0])));
        let visuals = drawn["step"]["presentation"]["visuals"].as_array().unwrap();
        assert!(visuals.iter().any(|v| v["shape"] == "polyline"));
        let committed = send(serde_json::json!({"command": "commit"}));
        assert!(committed["error"].is_null(), "{committed}");
        assert_eq!(committed["program"]["steps"][0]["id"], "sketch1");
    }
}

#[cfg(test)]
mod agent_tests {
    use super::*;
    use serde_json::{Value, json};

    /// The tools the hosted assistant's server offers the model: the
    /// browser's, as `agent_tools` gives them.
    const TOOLS: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../cloudflare-functions/functions/api/agent/tools.json"
    );

    /// The server offers the model the tools the browser runs, and no
    /// other: written again with `GEOP_WRITE_AGENT_TOOLS=1 cargo test -p
    /// geop agent`.
    #[test]
    fn the_servers_tools_are_the_browsers() {
        let tools: Value = serde_json::from_str(&agent_tools()).unwrap();
        let names: Vec<&str> = (tools.as_array().unwrap().iter())
            .map(|t| t["name"].as_str().unwrap())
            .collect();
        for offered in [
            "new_part",
            "add_step",
            "edit_step",
            "list_elements",
            "inspect",
        ] {
            assert!(names.contains(&offered), "{names:?}");
        }
        for host in ["save", "load_program", "export"] {
            assert!(!names.contains(&host), "{names:?}");
        }
        assert!(tools[0]["input_schema"]["type"] == "object", "{tools}");
        let written = format!("{}\n", serde_json::to_string_pretty(&tools).unwrap());
        if std::env::var_os("GEOP_WRITE_AGENT_TOOLS").is_some() {
            std::fs::write(TOOLS, &written).unwrap();
        }
        let committed = std::fs::read_to_string(TOOLS).unwrap_or_default();
        assert!(
            committed == written,
            "{TOOLS} is not the browser's tools: write it again with \
             `GEOP_WRITE_AGENT_TOOLS=1 cargo test -p geop agent`"
        );
    }

    /// A tool runs on the editor the page drives: it sees the example the
    /// page loaded, and answers with the updates the page shows.
    #[test]
    fn a_tool_runs_on_the_editor_the_page_shows() {
        let loaded = handle_json(
            &json!({ "command": "load_example", "name": "box_with_drill_hole" }).to_string(),
        )
        .unwrap();
        assert!(
            serde_json::from_str::<Value>(&loaded).unwrap()["error"].is_null(),
            "{loaded}"
        );
        let answer: Value = serde_json::from_str(&agent_call_json("run", "{}").unwrap()).unwrap();
        assert_eq!(answer["is_error"], false, "{answer}");
        let status: Value = serde_json::from_str(answer["text"].as_str().unwrap()).unwrap();
        assert_eq!(status["failed"], 0, "{status}");
        assert!(!status["steps"].as_array().unwrap().is_empty(), "{status}");
        let updates = answer["updates"].as_array().unwrap();
        assert!(updates.iter().any(|u| u["program"].is_object()), "{answer}");

        let removed: Value = serde_json::from_str(
            &agent_call_json("delete_step", &json!({ "id": "hole" }).to_string()).unwrap(),
        )
        .unwrap();
        assert_eq!(removed["is_error"], false, "{removed}");
        let program = &removed["updates"][0]["program"];
        assert!(program["can_undo"] == true, "{removed}");

        let refused: Value =
            serde_json::from_str(&agent_call_json("save", r#"{"path": "a.geop"}"#).unwrap())
                .unwrap();
        assert_eq!(refused["is_error"], true, "{refused}");
        assert!(
            refused["updates"].as_array().unwrap().is_empty(),
            "{refused}"
        );
    }
}

#[cfg(test)]
mod protocol_tests;
