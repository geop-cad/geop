//! WebAssembly bindings for the browser editor: one [`Editor`], driven by
//! [`handle`].
//!
//! The browser sends every command — a step started, a click in the
//! viewport as a ray, a slider moved, undo — as JSON, and gets back what to
//! show now: the program, the part to draw, the step being edited. Every
//! decision is made by the editor (see `geop_cad_base::editor`); the browser
//! renders and forwards input.

use std::cell::RefCell;

use geop_cad_base::Editor;
use geop_core_math::scalars::scal_in_f64::ScalInF64;
use wasm_bindgen::prelude::*;

/// The scalar type used for every wasm-exposed part.
type S = ScalInF64;

thread_local! {
    static EDITOR: RefCell<Editor<S>> = RefCell::new(Editor::new());
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

fn handle_json(command: &str) -> Result<String, String> {
    EDITOR.with(|editor| {
        editor
            .try_borrow_mut()
            .map_err(|_| "the kernel crashed in an earlier command: start a fresh one".to_string())?
            .handle_json(command)
    })
}

/// Apply a command (JSON `geop_cad_base::Command`) and return what to show
/// now (JSON `geop_cad_base::Update`). A refused command says why in the
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
    /// datums, then a new sketch started, its plane picked as a ray, a line
    /// drawn in it, and the sketch committed.
    #[test]
    fn a_sketch_is_drawn_through_json() {
        let loaded = send(serde_json::json!({
            "command": "load_example", "name": "boss_on_reference_plane",
        }));
        assert!(loaded["error"].is_null(), "{loaded}");
        let part = &loaded["scene"]["part"];
        assert!(!part["faces"].as_array().unwrap().is_empty());
        assert_eq!(part["datums"][0]["name"], "origin");
        assert_eq!(
            part["datums"][1]["frame"]["w"],
            serde_json::json!([0.0, 0.0, 1.0])
        );

        send(serde_json::json!({"command": "load", "program": {"steps": []}}));
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
