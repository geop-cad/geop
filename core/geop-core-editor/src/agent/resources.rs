//! What a [`Session`] offers an agent to read besides its tools — the
//! program, its status and elements, the operations, the examples — and
//! the prompts it starts a task with. MCP's resources and prompts are
//! these (`geop mcp`); like the tools, none names an operation.

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde_json::{Value, json};

use super::{Image, Session};

/// Something to read: its URI, name, what it is and its media type. A
/// `template` URI has one `{…}` part, filled in to read it.
#[derive(Clone, Debug, PartialEq)]
pub struct AgentResource {
    pub uri: &'static str,
    pub name: &'static str,
    pub description: &'static str,
    pub media_type: &'static str,
}

/// Every resource to read as it is, in the order they are offered.
pub fn resources() -> Vec<AgentResource> {
    let resource = |uri, name, description, media_type| AgentResource {
        uri,
        name,
        description,
        media_type,
    };
    vec![
        resource(
            "geop://program",
            "program",
            "The program as it stands, as a .geop file holds it: what `load_program` takes as `text`.",
            "application/json",
        ),
        resource(
            "geop://status",
            "status",
            "Every step's status (as `run` answers, without running), the step open, the tool in hand, the tools and panels there are.",
            "application/json",
        ),
        resource(
            "geop://elements",
            "elements",
            "The part's elements by their stable names, as `list_elements` answers.",
            "application/json",
        ),
        resource(
            "geop://operations",
            "operations",
            "Every operation a step can be, as `list_operations` answers.",
            "application/json",
        ),
        resource(
            "geop://examples",
            "examples",
            "The names of the example programs, and of the examples of several files.",
            "application/json",
        ),
        resource(
            "geop://picture",
            "picture",
            "The part as `look` last saw it — isometric, or facing the plane the open step works in, if it has not looked yet — as a PNG.",
            "image/png",
        ),
    ]
}

/// Every resource read by filling in its URI's one `{…}` part.
pub fn resource_templates() -> Vec<AgentResource> {
    vec![
        AgentResource {
            uri: "geop://operations/{kind}",
            name: "operation",
            description: "One operation in full, as `list_operations` given its `kind` answers: the arguments a new step of it starts with and its form.",
            media_type: "application/json",
        },
        AgentResource {
            uri: "geop://examples/{name}",
            name: "example",
            description: "An example program as a .geop file holds it — or, for an example of several files, each file's program by its path: how a whole part is written.",
            media_type: "application/json",
        },
    ]
}

/// What a resource holds: text, or a picture.
#[derive(Clone, Debug, PartialEq)]
pub enum Contents {
    Text(String),
    Image(Image),
}

/// A prompt an agent starts a task with: its name, what it is for and its
/// arguments, by name, with what each is and whether it is required.
#[derive(Clone, Debug, PartialEq)]
pub struct AgentPrompt {
    pub name: &'static str,
    pub description: &'static str,
    pub arguments: Vec<(&'static str, &'static str, bool)>,
}

/// Every prompt, in the order they are offered.
pub fn prompts() -> Vec<AgentPrompt> {
    vec![
        AgentPrompt {
            name: "model_part",
            description: "Model a part from a description, step by step, checking each step and looking at the result.",
            arguments: vec![(
                "description",
                "What to model: its shape, sizes and features.",
                true,
            )],
        },
        AgentPrompt {
            name: "fix_failing_steps",
            description: "Find out why the program's failing steps fail, and fix them.",
            arguments: vec![],
        },
        AgentPrompt {
            name: "review_part",
            description: "Look the part over from every side, weigh it and check it for overlapping solids, and say what is wrong with it.",
            arguments: vec![],
        },
    ]
}

/// How a part is modelled with the tools: what every prompt starts from.
const WORKFLOW: &str = "\
Model it with the geop tools, one step at a time:
1. `list_operations` for what there is, and `list_operations` with a `kind` for the arguments a step of it takes (`default_args`) and what each field is (`form`). The example programs (resources `geop://examples/{name}`) show how whole parts are written.
2. `add_step` each step, and read the status it answers with: a step that does not build says why. Fix it with `edit_step` before going on.
3. `list_elements` for the names of faces, edges and vertices a later step refers to (a sketch on a face, a fillet of edges).
4. `look` at the part from a few sides (`iso`, `front`, `top`, ...) to see that it is what was asked for, and `inspect` it: `mass_properties`, `measure` the sizes that matter.
Where writing a step's arguments is hard — a sketch to draw, a part placed to drag — `open_step` it and do what a user does: `step_event` clicks at points of the plane the step works in, keys pick its tools (a sketch's `l` draws lines), then `commit_step`. `undo` takes back an edit.";

impl Session {
    /// The resource `uri`, read: what it holds and its media type.
    pub fn read_resource(&mut self, uri: &str) -> GeopResult<(Contents, &'static str)> {
        let json = |value: Value| -> GeopResult<(Contents, &'static str)> {
            let text = serde_json::to_string_pretty(&value)
                .map_err(|e| GeopError::new(format!("writing {uri}: {e}")))?;
            Ok((Contents::Text(text), "application/json"))
        };
        let path = (uri.strip_prefix("geop://")).ok_or_else(|| {
            GeopError::new(format!("there is no resource {uri}: geop's are geop://…"))
        })?;
        match path.split_once('/') {
            None => match path {
                "program" => Ok((
                    Contents::Text(self.editor.program().to_json()?),
                    "application/json",
                )),
                "status" => json(self.status()?),
                "elements" => json(self.list_elements()?),
                "operations" => json(self.list_operations(None)?),
                "examples" => {
                    let program = self.shown()?;
                    json(json!({
                        "examples": program.examples,
                        "workspace_examples": program.workspace_examples,
                    }))
                }
                "picture" => {
                    let data = self.picture(None)?.0;
                    let media_type = "image/png";
                    Ok((Contents::Image(Image { media_type, data }), media_type))
                }
                _ => Err(self.no_resource(uri)),
            },
            Some(("operations", kind)) => json(self.list_operations(Some(kind))?),
            Some(("examples", name)) => {
                let registry = self.editor.registry();
                if self.shown()?.examples.contains(&name) {
                    let program = registry.example(name)?;
                    return Ok((Contents::Text(program.to_json()?), "application/json"));
                }
                let files = registry.workspace_example(name)?;
                let files = (files.into_iter())
                    .map(|(path, program)| Ok((path.to_string(), serde_json::to_value(&program)?)))
                    .collect::<Result<serde_json::Map<_, _>, serde_json::Error>>()
                    .map_err(|e| GeopError::new(format!("writing the example {name}: {e}")))?;
                json(Value::Object(files))
            }
            _ => Err(self.no_resource(uri)),
        }
    }

    fn no_resource(&self, uri: &str) -> GeopError {
        let uris: Vec<&str> = (resources().into_iter().chain(resource_templates()))
            .map(|r| r.uri)
            .collect();
        GeopError::new(format!(
            "there is no resource {uri}; there are {}",
            uris.join(", ")
        ))
    }

    /// The prompt `name` with `arguments` filled in: what it is for, and
    /// its text.
    pub fn prompt(&mut self, name: &str, arguments: &Value) -> GeopResult<(&'static str, String)> {
        let prompt = (prompts().into_iter())
            .find(|p| p.name == name)
            .ok_or_else(|| {
                let names: Vec<&str> = prompts().iter().map(|p| p.name).collect();
                GeopError::new(format!(
                    "there is no prompt {name}; there are {}",
                    names.join(", ")
                ))
            })?;
        let argument = |key: &str| {
            (arguments.get(key).and_then(Value::as_str))
                .ok_or_else(|| GeopError::new(format!("the prompt {name} needs `{key}`")))
        };
        let status = || -> GeopResult<String> {
            serde_json::to_string_pretty(&self.status()?)
                .map_err(|e| GeopError::new(format!("writing the status: {e}")))
        };
        let text = match name {
            "model_part" => format!(
                "Model this part in geop:\n\n{}\n\n{WORKFLOW}\n\nStart from `new_part`, unless the program open is the one to change.",
                argument("description")?
            ),
            "fix_failing_steps" => format!(
                "Some steps of the geop program open fail. Its status:\n\n{}\n\nFor each failing step, the first first: read its error — it names the step, the operation and the argument, with the kernel's context — look at what the steps before it built (`list_elements`, `look`), and fix the step's arguments with `edit_step`, or the step it refers to. Do not delete a step to make the status clean: the part has to stay what it was meant to be. Then `run`, and `look` at the part.\n\n{WORKFLOW}",
                status()?
            ),
            "review_part" => format!(
                "Review the geop part open. Its status:\n\n{}\n\n`look` at it from `iso`, `front`, `top` and `right` (and from below, if it has features there); `inspect` its `mass_properties` and its `interference`; `measure` the sizes that look wrong. Then say what is wrong with it, by the names of its steps and elements, and what to change. Change nothing yet.",
                status()?
            ),
            _ => return Err(GeopError::new(format!("the prompt {name} has no text"))),
        };
        Ok((prompt.description, text))
    }
}
