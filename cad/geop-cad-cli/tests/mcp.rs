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
    /// The notifications it sent, in order.
    notified: Vec<Value>,
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
            notified: Vec::new(),
        }
    }

    /// Sends a request and reads its answer.
    fn request(&mut self, method: &str, params: Value) -> Value {
        let id = self.next_id;
        self.next_id += 1;
        let message = json!({ "jsonrpc": "2.0", "id": id, "method": method, "params": params });
        writeln!(self.stdin, "{message}").unwrap();
        loop {
            let mut line = String::new();
            self.stdout.read_line(&mut line).unwrap();
            assert!(!line.is_empty(), "the server closed its output");
            let answer: Value =
                serde_json::from_str(&line).unwrap_or_else(|e| panic!("{e}: {line}"));
            assert_eq!(answer["jsonrpc"], "2.0");
            if answer.get("id").is_none() {
                self.notified.push(answer);
                continue;
            }
            assert_eq!(answer["id"], json!(id), "{answer}");
            return answer;
        }
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
    assert!(init["result"]["capabilities"]["resources"].is_object());
    assert!(init["result"]["capabilities"]["prompts"].is_object());
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
            "look",
            "undo",
            "redo",
            "open_step",
            "step_event",
            "commit_step",
            "cancel_step",
            "take_tool",
            "panel",
        ]
    );
    for tool in tools["result"]["tools"].as_array().unwrap() {
        assert_eq!(tool["inputSchema"]["type"], "object", "{tool}");
        assert!(tool["description"].as_str().is_some_and(|d| !d.is_empty()));
    }

    let status = server.ok("new_part", json!({}));
    assert_eq!(status["steps"], json!([]));

    // The operations, listed briefly: extrude is one.
    let operations = server.ok("list_operations", json!({}));
    let extrude = (operations["operations"].as_array().unwrap().iter())
        .find(|op| op["kind"] == "extrude")
        .expect("extrude is offered");
    assert_eq!(extrude["label"], "Extrude");
    assert!(extrude.get("form").is_none(), "{extrude}");

    // Extrude in full: its arguments, and a form naming its sketch field.
    let extrude = server.ok("list_operations", json!({ "kind": "extrude" }));
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

#[test]
fn a_step_is_added_before_another_and_a_wrong_target_names_the_right_ones() {
    let mut server = Server::start();
    server.ok("new_part", json!({}));
    let steps = drilled_box().steps;
    let (outline, extrude) = (&steps[0], &steps[1]);

    // The extrusion first: it builds nothing, its sketch not yet drawn.
    let status = server.ok(
        "add_step",
        json!({ "kind": extrude.operation, "args": extrude.args, "id": extrude.id }),
    );
    assert_eq!(status["failed"], 1, "{status}");

    // The sketch added before it: both build.
    let status = server.ok(
        "add_step",
        json!({ "kind": outline.operation, "args": outline.args, "id": outline.id, "before": extrude.id }),
    );
    assert_eq!(status["failed"], 0, "{status}");
    assert_eq!(status["steps"][0]["id"], json!(outline.id), "{status}");
    assert_eq!(status["solids"], json!(["extrude(box)"]), "{status}");

    // A sketch named by its step id alone: the error names the target the
    // sketch is.
    let status = server.ok(
        "edit_step",
        json!({ "id": extrude.id, "args": { "sketch": { "target": "/:outline:" } } }),
    );
    let error = status["steps"][1]["error"].as_str().unwrap();
    assert!(error.contains("`sketch:outline`"), "{error}");

    let (text, is_error) = server.call(
        "add_step",
        json!({ "kind": outline.operation, "args": outline.args, "before": "nothing" }),
    );
    assert!(is_error && text.contains("nothing"), "{text}");
    let (text, is_error) = server.call("list_operations", json!({ "kind": "nothing" }));
    assert!(is_error && text.contains("extrude"), "{text}");
}

/// What an agent reads besides the tools: the resources, a picture among
/// them, and a prompt filled in; and `look`'s picture, as MCP's image
/// content.
#[test]
fn resources_prompts_and_pictures_are_served() {
    let mut server = Server::start();
    server.request(
        "initialize",
        json!({ "protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": { "name": "test", "version": "0" } }),
    );
    for step in drilled_box().steps {
        server.ok(
            "add_step",
            json!({ "kind": step.operation, "args": step.args, "id": step.id }),
        );
    }

    let listed = server.request("resources/list", json!({}));
    let uris: Vec<&str> = (listed["result"]["resources"].as_array().unwrap().iter())
        .map(|r| r["uri"].as_str().unwrap())
        .collect();
    assert!(
        uris.contains(&"geop://program") && uris.contains(&"geop://picture"),
        "{listed}"
    );
    let templates = server.request("resources/templates/list", json!({}));
    assert_eq!(
        templates["result"]["resourceTemplates"][0]["uriTemplate"], "geop://operations/{kind}",
        "{templates}"
    );
    let program = server.request("resources/read", json!({ "uri": "geop://program" }));
    let contents = &program["result"]["contents"][0];
    assert_eq!(contents["mimeType"], "application/json", "{program}");
    assert!(contents["text"].as_str().unwrap().contains("\"hole\""));
    let picture = server.request("resources/read", json!({ "uri": "geop://picture" }));
    let blob = picture["result"]["contents"][0]["blob"].as_str().unwrap();
    assert!(
        blob.starts_with("iVBORw0KGgo"),
        "a PNG, as base64: {}",
        &blob[..16]
    );
    let missing = server.request("resources/read", json!({ "uri": "geop://nothing" }));
    assert!(
        missing["error"]["message"]
            .as_str()
            .unwrap()
            .contains("geop://program"),
        "{missing}"
    );

    let prompts = server.request("prompts/list", json!({}));
    assert_eq!(
        prompts["result"]["prompts"][0]["name"], "model_part",
        "{prompts}"
    );
    let prompt = server.request(
        "prompts/get",
        json!({ "name": "model_part", "arguments": { "description": "a hinge" } }),
    );
    let text = prompt["result"]["messages"][0]["content"]["text"]
        .as_str()
        .unwrap();
    assert!(text.contains("a hinge"), "{prompt}");

    let looked = server.request(
        "tools/call",
        json!({ "name": "look", "arguments": { "view": "iso" } }),
    );
    let content = looked["result"]["content"].as_array().unwrap();
    assert_eq!(content[1]["type"], "image", "{looked}");
    assert_eq!(content[1]["mimeType"], "image/png");
    assert!(
        content[1]["data"]
            .as_str()
            .unwrap()
            .starts_with("iVBORw0KGgo")
    );
}

/// A slow check given a limit answers with what it has and how far it got;
/// a call told how to report its progress reports it; and a call cancelled
/// before its turn is never run, nor answered.
#[test]
fn checks_are_limited_report_progress_and_are_cancelled() {
    let mut server = Server::start();
    server.request(
        "initialize",
        json!({ "protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": { "name": "test", "version": "0" } }),
    );
    let steps = drilled_box().steps;
    for step in &steps {
        server.ok(
            "add_step",
            json!({ "kind": step.operation, "args": step.args, "id": step.id }),
        );
    }
    let block = steps.iter().find(|step| step.id == "box").unwrap();
    server.ok(
        "add_step",
        json!({ "kind": block.operation, "args": block.args, "id": "second" }),
    );

    let stopped = server.ok(
        "inspect",
        json!({ "inspector": "interference", "limit_seconds": 0 }),
    );
    assert_eq!(
        stopped["stopped"],
        json!({ "checked": 0, "of": 1 }),
        "{stopped}"
    );
    let answered = server.request(
        "tools/call",
        json!({
            "name": "inspect",
            "arguments": { "inspector": "interference" },
            "_meta": { "progressToken": "watched" },
        }),
    );
    let text = answered["result"]["content"][0]["text"].as_str().unwrap();
    let whole: Value = serde_json::from_str(text).unwrap();
    assert!(whole["stopped"].is_null(), "{whole}");
    assert_eq!(whole["found"].as_array().unwrap().len(), 1, "{whole}");
    let told = &server.notified.last().expect("its progress was reported")["params"];
    assert_eq!(told["progressToken"], "watched", "{told}");
    assert!(told["message"].as_str().unwrap().contains("of 1"), "{told}");

    let refused = server.call(
        "inspect",
        json!({ "inspector": "interference", "limit_seconds": -1 }),
    );
    assert!(
        refused.1 && refused.0.contains("limit_seconds"),
        "{}",
        refused.0
    );

    // Cancelled before it came: the next answer is the next request's.
    let cancel = json!({ "jsonrpc": "2.0", "method": "notifications/cancelled", "params": { "requestId": 9999 } });
    writeln!(server.stdin, "{cancel}").unwrap();
    // The reader hears it before the call, as they are read in order.
    let cancelled = json!({ "jsonrpc": "2.0", "id": 9999, "method": "tools/call", "params": { "name": "new_part", "arguments": {} } });
    writeln!(server.stdin, "{cancelled}").unwrap();
    let (status, is_error) = server.call("run", json!({}));
    assert!(!is_error, "{status}");
    assert!(status.contains("second"), "new_part did not run: {status}");
}
