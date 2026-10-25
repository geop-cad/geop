//! `geop mcp` driven as an agent drives it: JSON-RPC over the process's
//! pipes, a block with a hole modelled step by step, inspected, exported
//! and saved.

use std::{
    io::{BufRead, BufReader, Write},
    path::PathBuf,
    process::{Child, ChildStdin, ChildStdout, Command, Stdio},
};

use geop_ops_extrude_revolve::testing::drilled_box;
use serde_json::{Value, json};

/// A `geop mcp` process, spoken to one JSON-RPC message per line.
struct Server {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
    next_id: u64,
}

impl Server {
    fn start() -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_geop"))
            .arg("mcp")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .expect("geop mcp starts");
        let stdin = child.stdin.take().unwrap();
        let stdout = BufReader::new(child.stdout.take().unwrap());
        Self {
            child,
            stdin,
            stdout,
            next_id: 1,
        }
    }

    /// Sends a request and reads its answer.
    fn request(&mut self, method: &str, params: Value) -> Value {
        let id = self.next_id;
        self.next_id += 1;
        let message = json!({ "jsonrpc": "2.0", "id": id, "method": method, "params": params });
        writeln!(self.stdin, "{message}").unwrap();
        let mut line = String::new();
        self.stdout.read_line(&mut line).unwrap();
        assert!(!line.is_empty(), "the server closed its output");
        let answer: Value = serde_json::from_str(&line).unwrap_or_else(|e| panic!("{e}: {line}"));
        assert_eq!(answer["id"], json!(id), "{answer}");
        assert_eq!(answer["jsonrpc"], "2.0");
        answer
    }

    /// Sends a notification, which is not answered.
    fn notify(&mut self, method: &str) {
        let message = json!({ "jsonrpc": "2.0", "method": method });
        writeln!(self.stdin, "{message}").unwrap();
    }

    /// Calls the tool `name`: the text of its answer, and whether it is an
    /// error.
    fn call(&mut self, name: &str, arguments: Value) -> (String, bool) {
        let answer = self.request(
            "tools/call",
            json!({ "name": name, "arguments": arguments }),
        );
        let result = &answer["result"];
        assert!(!result.is_null(), "{answer}");
        let text = result["content"][0]["text"].as_str().unwrap().to_string();
        (text, result["isError"].as_bool().unwrap_or(false))
    }

    /// Calls the tool `name`, which must succeed: its answer as JSON.
    fn ok(&mut self, name: &str, arguments: Value) -> Value {
        let (text, is_error) = self.call(name, arguments);
        assert!(!is_error, "{name} failed: {text}");
        serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("{name} answered no JSON ({e}): {text}"))
    }
}

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// A fresh directory for one test's files.
fn scratch(test: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("geop-mcp-{test}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn a_block_with_a_hole_is_modelled_inspected_exported_and_saved() {
    let dir = scratch("block");
    let mut server = Server::start();

    let init = server.request(
        "initialize",
        json!({ "protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": { "name": "test", "version": "0" } }),
    );
    assert_eq!(init["result"]["serverInfo"]["name"], "geop", "{init}");
    assert_eq!(init["result"]["protocolVersion"], "2025-06-18");
    assert!(init["result"]["capabilities"]["tools"].is_object());
    server.notify("notifications/initialized");
    assert_eq!(server.request("ping", json!({}))["result"], json!({}));

    let tools = server.request("tools/list", json!({}));
    let names: Vec<&str> = tools["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    assert_eq!(
        names,
        [
            "new_part",
            "load_program",
            "list_operations",
            "add_step",
            "edit_step",
            "delete_step",
            "run",
            "list_elements",
            "inspect",
            "export",
            "save",
        ]
    );
    for tool in tools["result"]["tools"].as_array().unwrap() {
        assert_eq!(tool["inputSchema"]["type"], "object", "{tool}");
        assert!(tool["description"].as_str().is_some_and(|d| !d.is_empty()));
    }

    let status = server.ok("new_part", json!({}));
    assert_eq!(status["steps"], json!([]));

    // The operations: extrude is one, and its form names its sketch field.
    let operations = server.ok("list_operations", json!({}));
    let extrude = (operations["operations"].as_array().unwrap().iter())
        .find(|op| op["kind"] == "extrude")
        .expect("extrude is offered");
    assert_eq!(extrude["label"], "Extrude");
    assert!(extrude["default_args"].is_object(), "{extrude}");
    let keys: Vec<&str> = (extrude["form"].as_array().unwrap().iter())
        .filter_map(|f| f["key"].as_str())
        .collect();
    assert!(keys.contains(&"sketch"), "{keys:?}");

    // The drilled box, step by step, each as its saved JSON.
    for step in drilled_box().steps {
        let status = server.ok(
            "add_step",
            json!({ "kind": step.operation, "args": step.args, "id": step.id }),
        );
        assert_eq!(status["failed"], 0, "{status}");
    }
    let status = server.ok("run", json!({}));
    assert_eq!(status["steps"].as_array().unwrap().len(), 4);
    assert_eq!(status["failed"], 0, "{status}");
    assert_eq!(status["solids"], json!(["extrude(hole)"]), "{status}");

    // Its elements, by the names a reference uses.
    let elements = server.ok("list_elements", json!({}));
    let faces: Vec<&str> = (elements["faces"].as_array().unwrap().iter())
        .map(|f| f["name"].as_str().unwrap())
        .collect();
    assert!(faces.contains(&"extrude(box,end)"), "{faces:?}");
    assert!(faces.contains(&"extrude(hole,end)"), "{faces:?}");
    assert!(faces.len() > 6, "a box with a blind hole: {faces:?}");
    assert_eq!(elements["sketches"], json!(["outline", "hole_sketch"]));
    assert!(!elements["edges"].as_array().unwrap().is_empty());

    // Its volume: the box less the hole.
    let mass = server.ok("inspect", json!({ "inspector": "mass_properties" }));
    let volume = mass["total"]["volume"]["value"].as_f64().unwrap();
    let expected = 2.0 * 2.0 * 1.0 - std::f64::consts::PI * 0.4 * 0.4 * 0.5;
    assert!(
        (volume - expected).abs() < 1e-6,
        "volume {volume}, expected {expected}"
    );

    // The end cap's area, measured by name.
    let measured = server.ok(
        "inspect",
        json!({ "inspector": "measure", "elements": [{ "kind": "face", "name": "extrude(box,end)" }] }),
    );
    let text = measured.to_string();
    assert!(text.contains("rea"), "no area in {text}");

    // Exported as a mesh and as STEP, and saved.
    let stl = dir.join("block.stl");
    let written = server.ok("export", json!({ "path": stl }));
    assert_eq!(written["format"], "stl");
    let bytes = std::fs::read(&stl).unwrap();
    let triangles = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    assert!(triangles > 0);
    assert_eq!(bytes.len(), 84 + 50 * triangles);
    let step = dir.join("block.step");
    server.ok("export", json!({ "path": step, "format": "step" }));
    assert!(
        std::fs::read_to_string(&step)
            .unwrap()
            .starts_with("ISO-10303-21")
    );
    let saved = dir.join("block.geop");
    server.ok("save", json!({ "path": saved }));
    let program =
        geop_core_program::program::Program::from_json(&std::fs::read_to_string(&saved).unwrap())
            .unwrap();
    assert_eq!(program.steps.len(), 4);

    // Without the hole, and twice as tall: eight cubic units.
    server.ok("delete_step", json!({ "id": "hole" }));
    let status = server.ok("delete_step", json!({ "id": "hole_sketch" }));
    assert_eq!(status["solids"], json!(["extrude(box)"]), "{status}");
    let taller = geop_ops_extrude_revolve::testing::extrusion(
        "outline",
        geop_ops_extrude_revolve::Extents::blind(2.0),
        geop_ops_model::Combine::NewBody,
    );
    let extent = serde_json::to_value(&taller).unwrap()["extent"].clone();
    let status = server.ok(
        "edit_step",
        json!({ "id": "box", "args": { "extent": extent } }),
    );
    assert_eq!(status["failed"], 0, "{status}");
    let mass = server.ok("inspect", json!({ "inspector": "mass_properties" }));
    let volume = mass["total"]["volume"]["value"].as_f64().unwrap();
    assert!((volume - 8.0).abs() < 1e-6, "volume {volume}");

    // The saved file loads again, with its hole.
    let status = server.ok("load_program", json!({ "path": saved }));
    assert_eq!(status["solids"], json!(["extrude(hole)"]), "{status}");
}

#[test]
fn a_step_that_does_not_build_is_answered_as_an_error_naming_it() {
    let mut server = Server::start();
    server.ok("new_part", json!({}));
    // Arguments that are not an extrusion's: refused, and nothing is added.
    let (text, is_error) = server.call(
        "add_step",
        json!({ "kind": "extrude", "args": { "nothing": true }, "id": "bad" }),
    );
    assert!(is_error, "{text}");
    assert!(text.contains("bad"), "{text}");
    assert_eq!(server.ok("run", json!({}))["steps"], json!([]));

    // An extrusion of a sketch that does not exist builds nothing, and the
    // step says why.
    let mut args = drilled_box().steps[1].args.clone();
    args["sketch"]["target"] = json!("/:sketch:missing");
    let status = server.ok(
        "add_step",
        json!({ "kind": "extrude", "args": args, "id": "box" }),
    );
    assert_eq!(status["failed"], 1, "{status}");
    let error = status["steps"][0]["error"].as_str().unwrap();
    assert!(error.contains("missing"), "{error}");

    let (text, is_error) = server.call("no_such_tool", json!({}));
    assert!(is_error && text.contains("no_such_tool"), "{text}");
    let unknown = server.request("no/such/method", json!({}));
    assert_eq!(unknown["error"]["code"], -32601, "{unknown}");
}
