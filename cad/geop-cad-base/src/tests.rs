//! The composition's tests: the editor of everything as a front end
//! drives it, every operation offered and opened, and every example any
//! crate ships built, drawn, weighed and edited. A test of one operation
//! or one example lives in that operation's crate.

use std::collections::BTreeMap;

use geop_core_editor::testing::{dialog, handled, pointer};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{part::Part, program::Program, testing::args};

use crate::{build, workspace};

/// Building a program on its own, in an empty [`workspace`] (see
/// [`build`]).
trait Built {
    /// The part the program builds.
    fn built(&self) -> GeopResult<Part>;
}

impl Built for Program {
    fn built(&self) -> GeopResult<Part> {
        Ok(build(self, "", workspace(BTreeMap::new()))?.part)
    }
}

/// The editor's operations as one set: offered, and started from what the
/// steps before built.
mod set {
    use geop_ops_extrude_revolve::{ExtrudeArgs, Output};
    use std::collections::HashSet;

    use geop_ops_model::Combine;
    use geop_ops_model::sketch::whole_sketch;

    use super::{Built, args, handled};
    use crate::registry::{registry, runner};

    /// Every operation is offered once, registered under the kind its steps
    /// serialize with, and every step of every example is of one of them.
    #[test]
    fn every_operation_is_offered() {
        let registry = registry().unwrap();
        let mut infos = registry.infos();
        let operations = registry.operations();
        let kinds: Vec<&str> = infos.iter().map(|i| i.kind).collect();
        assert_eq!(kinds.iter().collect::<HashSet<_>>().len(), kinds.len());
        // Offered group by group, as the editor lists them: the sketch and
        // what it is drawn on first, the part's parameters after its shape.
        infos.sort_by_key(|listed| (listed.info.group, listed.info.tier));
        let kinds: Vec<&str> = infos.iter().map(|i| i.kind).collect();
        assert_eq!(kinds[0], "add_sketch");
        assert_eq!(infos[0].info.label, "Sketch");
        assert_eq!(kinds[1], "add_datum");
        let parameters = kinds.iter().position(|&k| k == "parameters").unwrap();
        let flange = kinds.iter().position(|&k| k == "base_flange").unwrap();
        assert!(flange < parameters, "{kinds:?}");
        let extrude = infos.iter().find(|i| i.kind == "extrude").unwrap();
        assert!(
            extrude.info.doc.starts_with("Sweep a sketch"),
            "{}",
            extrude.info.doc
        );
        for info in &infos {
            assert_eq!(operations.get(info.kind).unwrap().kind(), info.kind);
        }
        for (_, program) in crate::registry().unwrap().example_programs() {
            for step in &program().steps {
                assert!(
                    kinds.contains(&step.operation.as_str()),
                    "{}",
                    step.operation
                );
            }
        }
    }

    /// Every operation is shown by an icon: SVG markup, named with the
    /// operation that lacks one.
    #[test]
    fn every_operation_has_an_icon() {
        let registry = registry().unwrap();
        let missing: Vec<String> = registry
            .infos()
            .iter()
            .filter(|listed| !listed.info.icon.is_some_and(|svg| svg.starts_with('<')))
            .map(|listed| format!("{} ({:?})", listed.kind, listed.info.icon))
            .collect();
        assert!(
            missing.is_empty(),
            "operations without an icon: {missing:?}"
        );
    }

    /// A new step starts from what the steps before it built.
    #[test]
    fn new_steps_start_from_what_was_built() {
        let program = geop_ops_extrude_revolve::examples::box_with_drill_hole();
        let part = program.built().unwrap();
        let registry = registry().unwrap();
        let editors = registry.editors();
        let runner = runner();
        let new_step = |kind: &str| editors.new_step(&runner, &part, &program, kind, "new", None);
        let step = new_step("extrude").unwrap();
        let extrude: ExtrudeArgs = args(&step);
        assert_eq!(extrude.sketch, whole_sketch("hole_sketch"));
        assert_eq!(
            extrude.output,
            Output::Solid(Combine::join("extrude(hole)"))
        );
        for info in registry.infos() {
            let step = new_step(info.kind).unwrap();
            assert_eq!(step.operation, info.kind);
        }
        assert!(new_step("no_such_operation").is_err());
    }

    /// Every operation's dialog opens, the way the toolbar opens it, on an
    /// empty part and on a drilled box, and is cancelled again: none may fail
    /// for want of something to work on — in the browser a panic leaves the
    /// editor unusable — and each answers with a step to edit or says why not.
    #[test]
    fn every_operation_opens_on_any_part() {
        use crate::Command;
        for program in [
            None,
            Some(geop_ops_extrude_revolve::examples::box_with_drill_hole()),
        ] {
            let mut editor = crate::editor();
            if let Some(program) = program {
                handled(
                    &mut editor,
                    Command::Load {
                        program,
                        path: None,
                    },
                );
            }
            for info in crate::registry().unwrap().infos() {
                let update = editor.handle(Command::New {
                    kind: info.kind.to_string(),
                });
                assert!(
                    update.step.is_some() || update.error.is_some(),
                    "{}: neither a step nor a reason",
                    info.kind
                );
                editor.handle(Command::Cancel);
            }
        }
    }
}

/// The editor of everything, as the front end drives it: commands that
/// edit the program, and only what changed sent back; every operation
/// offered, group by group; every example loaded by name; a program of
/// steps no operation reads refused.
mod editor {
    use geop_core_editor::editing::{StepEditEvent, Value};
    use geop_core_program::program::{Program, ProgramStep};
    use geop_ops_extrude_revolve::ExtrudeArgs;

    use super::{args, dialog, handled, pointer};
    use crate::{Command, Editor, Update};

    fn editor() -> (Editor, Update) {
        let mut editor = crate::editor();
        let update = handled(
            &mut editor,
            Command::LoadExample {
                name: "box_with_drill_hole".into(),
            },
        );
        (editor, update)
    }

    fn distance(editor: &Editor, id: &str) -> f64 {
        let program = editor.program();
        let args: ExtrudeArgs = args(&program.steps[program.index_of(id).unwrap()]);
        match &args.extent.side1 {
            geop_ops_extrude_revolve::Extent::Blind(d) => d.plain().expect("a plain distance"),
            other => panic!("{other:?} is no distance"),
        }
    }

    /// A new step goes in where the program runs to, once it builds; an
    /// existing one is edited in place, moved and removed by id — and every
    /// change can be undone and redone.
    #[test]
    fn commands_edit_the_program() {
        let (mut editor, _) = editor();
        let before = editor.program().clone();

        let update = editor.handle(Command::New {
            kind: "extrude".into(),
        });
        let step = update.step.expect("a step is edited");
        assert_eq!(step.label, "Extrude");
        assert_eq!(step.id, None);
        let update = handled(&mut editor, Command::Commit);
        assert!(update.step.is_none());
        let steps = update.program.expect("the program changed").steps;
        assert_eq!(steps.last().unwrap().id, "extrude1");

        editor.handle(Command::Open {
            id: "extrude1".into(),
            placed: None,
        });
        editor.handle(dialog("distance", Value::Number(2.0)));
        // Not in the program until committed.
        assert_eq!(distance(&editor, "extrude1"), 1.0);
        editor.handle(Command::Commit);
        assert_eq!(distance(&editor, "extrude1"), 2.0);

        editor.handle(Command::Move {
            id: "extrude1".into(),
            index: 0,
        });
        assert_eq!(editor.program().steps[0].id, "extrude1");
        editor.handle(Command::Remove {
            id: "extrude1".into(),
        });
        assert_eq!(*editor.program(), before);

        for _ in 0..3 {
            editor.handle(Command::Undo);
        }
        assert_eq!(distance(&editor, "extrude1"), 1.0);
        let update = editor.handle(Command::Redo);
        assert!(update.program.unwrap().can_redo);
        assert_eq!(distance(&editor, "extrude1"), 2.0);
    }

    /// A command that would break the program is refused, saying why, and
    /// changes nothing.
    #[test]
    fn bad_commands_are_refused() {
        let (mut editor, _) = editor();
        let before = editor.program().clone();
        let mut twice = before.clone();
        twice.steps.push(twice.steps[0].clone());
        for bad in [
            Command::Remove { id: "nope".into() },
            Command::Move {
                id: "box".into(),
                index: before.steps.len(),
            },
            Command::Load {
                program: twice,
                path: None,
            },
            Command::Commit,
            Command::New {
                kind: "no_such_operation".into(),
            },
        ] {
            let update = editor.handle(bad);
            assert!(update.error.is_some());
            assert_eq!(*editor.program(), before);
        }

        // A step that does not build cannot go in: without the box, the hole
        // has no face to be sketched on.
        let update = editor.handle(Command::Remove { id: "box".into() });
        assert!(update.error.is_none());
        let update = editor.handle(Command::Open {
            id: "hole_sketch".into(),
            placed: None,
        });
        assert!(update.step.unwrap().error.is_some());
        let update = editor.handle(Command::Commit);
        assert!(update.error.is_some());
    }

    /// The part is sent again only when what is drawn changed, the program only
    /// when it or its run did — a hover sends neither.
    #[test]
    fn only_what_changed_is_sent() {
        let (mut editor, update) = editor();
        assert!(update.program.is_some() && update.scene.is_some());
        let update = editor.handle(Command::Open {
            id: "hole".into(),
            placed: None,
        });
        assert!(update.program.is_some(), "the step is marked as edited");
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Hover {
                pointer: pointer([5.0, 5.0, 5.0], [0.0, 0.0, -1.0]),
                shift: false,
            },
        });
        assert!(update.program.is_none() && update.scene.is_none());
        assert!(update.step.is_some());
        let update = editor.handle(Command::Preview { preview: false });
        assert!(update.scene.is_some(), "the part before the step is drawn");
        let update = editor.handle(Command::Show);
        assert!(update.program.is_some() && update.scene.is_some());
    }

    /// Every example loads by name, and builds.
    #[test]
    fn examples_load_by_name() {
        let mut editor = crate::editor();
        let names = editor.handle(Command::Show).program.unwrap().examples;
        assert_eq!(
            names.len(),
            crate::registry().unwrap().example_programs().len()
        );
        let update = editor.handle(Command::LoadExample {
            name: names[0].into(),
        });
        let steps = update.program.unwrap().steps;
        assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
        assert_ne!(*editor.program(), Program::new());
    }

    /// The toolbar's sections are the editor's: it sends the operations group
    /// by group, in the groups' order, every operation in one, and within a
    /// group the few used most (shown big) first, then the common ones (small),
    /// then those only in the group's menu.
    #[test]
    fn operations_are_offered_by_group() {
        use geop_core_editor::editable::OperationTier;
        let (_, update) = editor();
        let operations = update.program.expect("the program is sent").operations;
        assert_eq!(operations.len(), crate::registry().unwrap().infos().len());
        let order: Vec<_> = operations
            .iter()
            .map(|o| (o.info.group, o.info.tier))
            .collect();
        assert!(
            order.is_sorted(),
            "not group by group, the most used first: {order:?}"
        );
        let big: Vec<&str> = operations
            .iter()
            .filter(|o| o.info.tier == OperationTier::Big)
            .map(|o| o.kind)
            .collect();
        assert_eq!(
            big,
            [
                "add_sketch",
                "extrude",
                "revolve",
                "hole",
                "fillet",
                "boolean",
                "linear_pattern",
                "subd",
                "base_flange",
                "add_part",
                "drawing",
            ]
        );
        // The sketch comes first; the part's own parameters, tables and
        // properties after its shape, small buttons.
        let json = serde_json::to_value(&operations[0]).unwrap();
        assert_eq!(json["group"], "Sketch");
        assert_eq!(json["tier"], "Big");
        let parameters = operations.iter().find(|o| o.kind == "parameters").unwrap();
        let json = serde_json::to_value(parameters).unwrap();
        assert_eq!(json["group"], "Part");
        assert_eq!(json["tier"], "Small");
        let mirror = operations.iter().find(|o| o.kind == "mirror").unwrap();
        assert_eq!(
            serde_json::to_value(mirror).unwrap()["group"],
            "Pattern & bodies"
        );
    }

    /// A program the editor is given is checked against its operations before
    /// it is taken: one loaded or pasted with a step of an operation there is
    /// none of, or with arguments its operation does not read, is refused,
    /// naming the step, and the program edited stays as it was.
    #[test]
    fn a_program_of_unknown_steps_is_refused() {
        let mut editor = crate::editor();
        let mut unknown = Program::new();
        unknown.steps.push(ProgramStep {
            id: "mystery".into(),
            operation: "no_such_operation".into(),
            args: serde_json::json!({}),
        });
        let refused = editor.handle(Command::Load {
            program: unknown.clone(),
            path: None,
        });
        let error = refused.error.unwrap();
        assert!(
            error.contains("step `mystery`") && error.contains("no_such_operation"),
            "{error}"
        );
        assert!(editor.program().steps.is_empty());

        let refused = editor.handle(Command::Paste {
            text: unknown.to_json().unwrap(),
        });
        assert!(refused.error.unwrap().contains("no_such_operation"));
        assert!(editor.program().steps.is_empty());

        let mut unreadable = geop_ops_extrude_revolve::examples::box_with_drill_hole();
        unreadable.steps[0].args = serde_json::json!("not arguments");
        let refused = editor.handle(Command::Load {
            program: unreadable,
            path: None,
        });
        let error = refused.error.unwrap();
        assert!(error.contains("arguments of operation"), "{error}");
        assert!(editor.program().steps.is_empty());
    }
}

/// The examples of every operation, run incrementally: edited one after
/// another on one part, each builds what it builds from scratch.
mod incremental {
    use geop_core_program::{
        program::Program,
        testing::{assert_incremental, edit_sequence},
        workspace::Workspace,
    };
    use geop_ops_model::parameters::{ParameterKind, parameters_of, set_parameters};
    use std::{collections::BTreeMap, rc::Rc};

    /// A runner that takes steps from what it built before must build what
    /// building from scratch builds: after a step is dropped from the middle of
    /// a program and put back, and after a number a step reads is changed. A
    /// step skipped for reading what did not change would otherwise show here
    /// as a target that differs, or an error that is missing (see
    /// [`assert_incremental`]).
    #[test]
    #[ignore = "slow: 10 s — run with `cargo test -- --ignored`"]
    fn a_runner_builds_what_building_from_scratch_builds() {
        let examples = [
            "box_with_drill_hole",
            "bracket",
            "cross_drilled_shaft",
            "split_plate",
            "parametric_plate",
            "pin",
            "patterned_plate",
            "rounded_box",
            "hole_plate",
        ];
        for (name, make) in crate::registry().unwrap().example_programs() {
            if !examples.contains(&name) {
                continue;
            }
            let original = make();
            let steps = original.steps.len();
            let mut edits = Vec::new();
            for index in [0, steps / 2, steps - 1] {
                let mut dropped = original.clone();
                dropped.steps.remove(index);
                edits.push((format!("drop step {index}"), dropped));
                edits.push(("restore".to_string(), original.clone()));
            }
            let parameters = parameters_of(&original);
            for (k, parameter) in parameters.values.iter().enumerate() {
                if let ParameterKind::Number { expression, .. } = &parameter.kind
                    && let Ok(value) = expression.parse::<f64>()
                {
                    let mut scaled = original.clone();
                    let mut changed = parameters.clone();
                    if let ParameterKind::Number { expression, .. } = &mut changed.values[k].kind {
                        *expression = (value * 1.25).to_string();
                    }
                    set_parameters(&mut scaled, changed);
                    edits.push((format!("scale {}", parameter.name), scaled));
                    edits.push(("restore".to_string(), original.clone()));
                }
            }

            let programs: Vec<Program> = std::iter::once(original)
                .chain(edits.into_iter().map(|(_, program)| program))
                .collect();
            let workspace: Rc<dyn Workspace> = crate::registry::workspace(BTreeMap::new());
            let registry = Rc::new(crate::registry().unwrap().operations());
            assert_incremental(registry, Some((workspace, name)), &programs);
        }
    }

    /// Every example program named in `only` — all, for none — and every file
    /// of every example made of several, run through [`edit_sequence`],
    /// incrementally builds what it builds from scratch (see
    /// [`assert_incremental`]).
    fn assert_examples_incremental(only: Option<&[&str]>) {
        let offered = crate::registry().unwrap();
        let registry = Rc::new(offered.operations());
        let wanted = |name: &str| only.is_none_or(|only| only.contains(&name));
        for (name, make) in offered.example_programs() {
            if !wanted(name) {
                continue;
            }
            let workspace: Rc<dyn Workspace> = crate::registry::workspace(BTreeMap::new());
            let programs = edit_sequence(&make());
            assert_incremental(registry.clone(), Some((workspace, name)), &programs);
        }
        for name in offered
            .workspace_example_names()
            .filter(|name| wanted(name))
        {
            let files = offered.workspace_example(name).unwrap();
            let texts: BTreeMap<String, String> = (files.iter())
                .map(|(file, program)| (file.to_string(), program.to_json().unwrap()))
                .collect();
            let workspace: Rc<dyn Workspace> = crate::registry::workspace(texts);
            for (file, program) in &files {
                assert_incremental(
                    registry.clone(),
                    Some((workspace.clone(), file)),
                    &edit_sequence(program),
                );
            }
        }
    }

    /// [`assert_examples_incremental`] on a few cheap examples: a part with a
    /// table, an assembly placing parts.
    #[test]
    fn examples_build_incrementally_what_they_build_from_scratch() {
        assert_examples_incremental(Some(&[
            "box_with_drill_hole",
            "parametric_plate",
            "pin_in_plate",
        ]));
    }

    /// [`assert_examples_incremental`] on every example.
    #[test]
    #[ignore = "slow: every example built several times over — run with `cargo test -- --ignored`"]
    fn every_example_builds_incrementally_what_it_builds_from_scratch() {
        assert_examples_incremental(None);
    }
}

/// Every example the editor offers, whichever crate ships it: drawn in
/// every view, weighed and halved.
mod examples {
    use std::collections::BTreeMap;

    use geop_core_editor::output::Bounded;
    use geop_core_geometry::contains::curve::curve_could_contain;
    use geop_core_math::{geop_error::GeopError, scalars::Scalar, vector::Vector3};
    use geop_core_program::{part::Part, program::Program};
    use geop_lib_booleans::{RemeshParams, boolean};
    use geop_lib_sweep::shapes::cube_solid;
    use geop_ops_drawing::{
        DrawingArgs, ViewKind, ViewLine, ViewOptions, compose, drawing::drawn_faces, parts_list,
        project_view, scene::Scene,
    };
    use geop_ops_inspect::{
        interference::bounding_box,
        mass::{MassSummary, mass_report},
    };
    use geop_ops_model::placed::solids::{Shapes, placed_solids};
    use geop_ops_model::{BooleanOp, NamedModel, Namer, RefId};

    use super::Built;

    /// Every example program the editor offers, by name, made.
    fn examples() -> Vec<(&'static str, Program)> {
        let registry = crate::registry().unwrap();
        (registry.example_names())
            .map(|name| (name, registry.example(name).unwrap()))
            .collect()
    }

    /// `error` with every edge and face id it mentions followed by its name.
    fn named(part: &NamedModel, error: &impl std::fmt::Display) -> String {
        let mut text = error.to_string();
        for (id, name) in part.names().iter() {
            let tag = match id {
                RefId::Edge(e) => format!("{e:?}"),
                RefId::Face(f) => format!("{f:?}"),
                _ => continue,
            };
            text = text.replace(&tag, &format!("{tag} [{name}]"));
        }
        text
    }

    /// Views known not to build, and why.
    ///
    /// `revolved_cone_on_box` from the front and back: the boolean's vertex
    /// where the cone's intersection with the box's face `y = 1.18`
    /// (`combine(revolve1,extrude(extrude1,sketch1,c4),...)`, edge `a`) meets the
    /// box's right face lies at `x = 2.2456168`, 3.1e-5 off that face
    /// (`x = 2.2456482`). On the paper, `a` then ends 3e-5 short of where the
    /// cone's base circle (`combine(revolve1,revolve(revolve1,sketch2,p3,q0),...)`)
    /// ends, running into it tangentially, and the search for where the two
    /// cross cannot isolate a near-touch that close. The defect is the vertex's
    /// accuracy in the boolean, not the drawing.
    ///
    /// `subd_mug` from the front, back, top, bottom and left: a body of
    /// ninety bicubic patches, its inner wall 0.15 inside its outer one,
    /// a handle bridged to its side. `curve_curve_crossings` exhausts its
    /// 20 000 nodes with over a thousand pairs of projected edges pending,
    /// and from the top the silhouette trace of `subd(mug,f75)`, a patch of
    /// the lip, does not end within 4000 steps. The body validates, and is
    /// no different unmirrored: the defect is in the drawing's searches
    /// over that many close, smoothly joined curves.
    const KNOWN_TO_FAIL: [(&str, ViewKind); 7] = [
        ("revolved_cone_on_box", ViewKind::Front),
        ("revolved_cone_on_box", ViewKind::Back),
        ("subd_mug", ViewKind::Front),
        ("subd_mug", ViewKind::Back),
        ("subd_mug", ViewKind::Top),
        ("subd_mug", ViewKind::Bottom),
        ("subd_mug", ViewKind::Left),
    ];

    /// Every view of every example builds, and no piece of a line is drawn
    /// both visible and hidden.
    #[test]
    #[ignore = "slow: every view of every example — run with `cargo test -- --ignored`"]
    fn every_example_draws() {
        let mut failures = Vec::new();
        for (name, program) in examples() {
            let built = program.built().unwrap();
            // A part with no model of its own is drawn as an empty one.
            let part = &(geop_ops_model::read_model(&built.current()))
                .map(|model| model.into_owned())
                .unwrap_or_default();
            for kind in ViewKind::ALL {
                let model = part.topology();
                let v = match project_view(
                    model,
                    &drawn_faces(model),
                    &kind.frame().unwrap(),
                    &ViewOptions::default(),
                ) {
                    Ok(v) => v,
                    Err(_) if KNOWN_TO_FAIL.contains(&(name, kind)) => continue,
                    Err(e) => {
                        failures.push(format!("{name}, {} view: {}", kind.name(), named(part, &e)));
                        continue;
                    }
                };
                // A hidden line may cross or touch a visible one, never run
                // along it: no hidden piece has the points a third and two
                // thirds along it both on one visible line.
                for hidden in v.lines.iter().filter(|l| !l.visible) {
                    let (a, b) = hidden.curve.domain();
                    let at = |f: f64| {
                        let t = a.add(b.sub(a).mul(Scalar::from_f64(f))).sharpen();
                        hidden.curve.evaluate(t).unwrap()
                    };
                    let (p, q) = (at(1.0 / 3.0), at(2.0 / 3.0));
                    for visible in v.lines.iter().filter(|l| l.visible) {
                        let on = |x: &geop_core_math::vector::Vector2| {
                            curve_could_contain(&visible.curve, x, 20_000, Scalar::from_f64(1e-7))
                                .unwrap()
                                .is_some()
                        };
                        assert!(
                            !(on(&p) && on(&q)),
                            "{name}, {} view: the hidden {:?} of edge {:?} runs along the visible \
                             {:?} of edge {:?}",
                            kind.name(),
                            hidden.kind,
                            hidden.edge.and_then(|e| part.name_of(e)),
                            visible.kind,
                            visible.edge.and_then(|e| part.name_of(e)),
                        );
                    }
                }
            }
            // The whole default sheet, too, of every example whose views build.
            if !KNOWN_TO_FAIL.iter().any(|(n, _)| *n == name)
                && let Err(e) = compose(&built.current(), &DrawingArgs::default(), "", &[])
            {
                failures.push(format!("{name}, the sheet: {}", named(part, &e)));
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n\n"));
    }

    /// The example workspace `name` built: its first file, as that file's
    /// path, and the part.
    fn example_assembly(name: &str) -> (&'static str, Part) {
        let files = crate::registry().unwrap().workspace_example(name).unwrap();
        let (path, program) = files[0].clone();
        let files: BTreeMap<String, String> = files
            .into_iter()
            .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
            .collect();
        (
            path,
            crate::build(&program, path, crate::workspace(files))
                .unwrap()
                .part,
        )
    }

    /// Every example assembly draws: its default sheet with the bill of
    /// materials builds, with a balloon per line of the bill whose part has
    /// faces drawn, and in no view does a hidden piece run along a visible one.
    #[test]
    #[ignore = "slow: every example assembly drawn — run with `cargo test -- --ignored`"]
    fn every_example_assembly_draws() {
        use geop_lib_sheet::Shape;
        use geop_ops_drawing::layer;
        let mut failures = Vec::new();
        let registry = crate::registry().unwrap();
        for name in registry.workspace_example_names() {
            let (path, built) = example_assembly(name);
            let args = DrawingArgs {
                bom: true,
                ..Default::default()
            };
            let parts = parts_list(&built.current(), path, &args).unwrap();
            let scene = Scene::of(&built.current()).unwrap();
            let sheet = match compose(&built.current(), &args, "", &parts) {
                Ok(sheet) => sheet,
                Err(e) => {
                    failures.push(format!("{name}: {e}"));
                    continue;
                }
            };
            let drawn = parts
                .iter()
                .filter(|line| {
                    scene
                        .bodies
                        .iter()
                        .any(|b| line.placements.contains(&b.path))
                })
                .count();
            let balloons = sheet
                .strokes
                .iter()
                .filter(|s| {
                    s.layer == layer::DIMENSION
                        && matches!(s.shape, Shape::Circle { radius, .. } if radius == 4.0)
                })
                .count();
            if balloons != drawn {
                failures.push(format!(
                    "{name}: {balloons} balloons for {drawn} lines of the bill drawn"
                ));
            }
            for kind in ViewKind::ALL {
                let v = match scene.project(&kind.frame().unwrap(), &ViewOptions::default()) {
                    Ok(v) => v,
                    Err(e) => {
                        failures.push(format!("{name}, {} view: {e}", kind.name()));
                        continue;
                    }
                };
                for hidden in v.lines.iter().filter(|l| !l.visible) {
                    let (a, b) = hidden.curve.domain();
                    let at = |f: f64| {
                        let t = a.add(b.sub(a).mul(Scalar::from_f64(f))).sharpen();
                        hidden.curve.evaluate(t).unwrap()
                    };
                    let (p, q) = (at(1.0 / 3.0), at(2.0 / 3.0));
                    let on = |line: &ViewLine, x| {
                        curve_could_contain(&line.curve, x, 20_000, Scalar::from_f64(1e-7))
                            .unwrap()
                            .is_some()
                    };
                    if let Some(visible) = v
                        .lines
                        .iter()
                        .filter(|l| l.visible)
                        .find(|l| on(l, &p) && on(l, &q))
                    {
                        failures.push(format!(
                            "{name}, {} view: the hidden {:?} of {} runs along the visible {:?} of {}",
                            kind.name(),
                            hidden.kind,
                            scene.bodies[hidden.body].path,
                            visible.kind,
                            scene.bodies[visible.body].path,
                        ));
                    }
                }
            }
        }
        assert!(failures.is_empty(), "{}", failures.join("\n\n"));
    }

    /// The checks every solid's mass properties pass: converged, a positive
    /// volume, a symmetric inertia tensor whose principal moments are positive
    /// and satisfy the triangle inequality, as every body's do — the tensor
    /// `inertia` of its shape (of density one) and its `principal` moments,
    /// which it has whatever its material.
    #[allow(clippy::needless_range_loop, reason = "a tensor against its transpose")]
    fn check_mass(
        name: &str,
        summary: &MassSummary,
        inertia: [[Scalar; 3]; 3],
        principal: super::GeopResult<[Scalar; 3]>,
        failures: &mut Vec<String>,
    ) {
        let mut fail = |why: String| failures.push(format!("{name}: {why}"));
        if !summary.converged {
            fail("did not converge".into());
        }
        if summary.volume.value - summary.volume.error <= 0.0 {
            fail(format!("volume {:?}", summary.volume));
        }
        for a in 0..3 {
            for b in 0..3 {
                let (x, y) = (inertia[a][b], inertia[b][a]);
                let (x, y) = (Bounded::of(x), Bounded::of(y));
                if (x.value - y.value).abs() > x.error + y.error {
                    fail(format!("I[{a}][{b}] = {x:?} but I[{b}][{a}] = {y:?}"));
                }
            }
        }
        let principal = match principal {
            Ok(principal) => principal,
            Err(e) => return fail(format!("no principal moments: {e}")),
        };
        let [i1, i2, i3] = principal.map(Bounded::of);
        if i1.value - i1.error <= 0.0 {
            fail(format!("a principal moment {i1:?} is not positive"));
        }
        if i1.value + i2.value + i1.error + i2.error < i3.value - i3.error {
            fail(format!("principal moments {i1:?} + {i2:?} < {i3:?}"));
        }
    }

    /// Every example part: every solid's mass properties pass [`check_mass`],
    /// and its volume is that of its two pieces — cut across by a box, by
    /// booleans, each piece's volume integrated on its own.
    #[test]
    #[ignore = "slow: every example's mass properties, and booleans halving each solid — run with `cargo test -- --ignored`"]
    fn example_mass_properties_are_consistent() {
        let mut failures = Vec::new();
        for (example, program) in examples() {
            let part = program.built().unwrap();
            let reader = part.current();
            let report = mass_report(&reader).unwrap();
            let mut shapes = Shapes::default();
            for (body, placed) in report.bodies.iter().zip(placed_solids(&reader).unwrap()) {
                let name = format!("{example}/{}", body.name);
                let Some(summary) = &body.properties else {
                    failures.push(format!("{name}: {:?}", body.error));
                    continue;
                };
                let shape = placed.shape_properties(&mut shapes).unwrap();
                let principal = shape.principal().map(|p| p.moments);
                check_mass(&name, summary, shape.inertia, principal, &mut failures);
                // The part of its box below a plane across x, a little beyond
                // it elsewhere. Not through the centre of mass: of a symmetric
                // part, that is where its circles' seams are, and a cut placed
                // on an existing edge asks the boolean for a degenerate crossing
                // (see `AGENTS.md`) — so a fraction of the width no construction
                // chooses.
                assert!(placed.pose.is_none(), "{name}: an example places no part");
                let faces = placed.model.topology().solid_faces(placed.solid).unwrap();
                let bounds = bounding_box(&placed, &faces).unwrap();
                let lo = |k: usize| bounds[k].lower().to_f64() - 1.0;
                let hi = |k: usize| bounds[k].upper().to_f64() + 1.0;
                let cut = bounds[0].lower().to_f64() + 0.437_281 * bounds[0].width().to_f64();
                let halves = [BooleanOp::Intersection, BooleanOp::Difference].map(|op| {
                    let mut scratch = placed.model.clone().into_owned();
                    let knife = cube_solid(
                        &mut scratch,
                        "knife",
                        Vector3::from_array([lo(0), lo(1), lo(2)].map(Scalar::from_f64)),
                        Vector3::from_array([cut, hi(1), hi(2)].map(Scalar::from_f64)),
                    )?;
                    let namer = Namer::new("halve", "half")?;
                    let half = boolean(
                        &mut scratch,
                        &namer,
                        placed.solid,
                        knife,
                        op,
                        RemeshParams::default(),
                    )?;
                    let Some(half) = half else {
                        return Ok(Scalar::ZERO);
                    };
                    Ok(scratch.topology().mass_properties(half)?.volume)
                });
                match halves {
                    [Ok(a), Ok(b)] => {
                        let sum = Bounded::of(a.add(b));
                        let whole = summary.volume;
                        if (sum.value - whole.value).abs() > sum.error + whole.error {
                            failures.push(format!(
                                "{name}: halves {sum:?} do not add up to the whole {whole:?}"
                            ));
                        }
                    }
                    [a, b] => failures.push(format!(
                        "{name}: halving failed: {:?}",
                        [a.err(), b.err()]
                            .into_iter()
                            .flatten()
                            .map(|e: GeopError| e.to_string())
                            .collect::<Vec<_>>()
                    )),
                }
            }
        }
        assert!(failures.is_empty(), "{failures:#?}");
    }
}

/// The agent's tools ([`geop_core_editor::agent`]) on the editor of
/// everything: each tool the browser offers, run on the drilled box; and
/// those that take a path refused where there are no files.
mod agent {
    use geop_core_editor::agent::{Session, ToolAnswer, tools};
    use geop_ops_extrude_revolve::testing::drilled_box;
    use serde_json::{Value, json};

    /// Calls `name`, which must succeed: its answer as JSON, and how many
    /// updates it sent.
    fn ok(session: &mut Session, name: &str, input: Value) -> (Value, usize) {
        let ToolAnswer {
            text,
            is_error,
            updates,
        } = session.call(name, &input, None);
        assert!(!is_error, "{name} failed: {text}");
        let answer = serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("{name} answered no JSON ({e}): {text}"));
        (answer, updates.len())
    }

    #[test]
    fn the_browser_is_offered_every_tool_but_those_taking_a_path() {
        let offered: Vec<&str> = (tools().into_iter())
            .filter(|tool| !tool.files)
            .map(|tool| tool.name)
            .collect();
        assert_eq!(
            offered,
            [
                "new_part",
                "list_operations",
                "add_step",
                "edit_step",
                "delete_step",
                "run",
                "list_elements",
                "inspect"
            ]
        );
        let mut session = Session::new(crate::editor());
        for name in ["save", "load_program", "export"] {
            let answer = session.call(name, &json!({ "path": "part.geop" }), None);
            assert!(answer.is_error, "{name}: {}", answer.text);
            assert!(answer.text.contains("path"), "{}", answer.text);
            assert!(answer.updates.is_empty());
        }
    }

    /// Every tool the browser offers, on the drilled box: built step by
    /// step, run, listed, weighed, measured, edited and taken apart — each
    /// tool that changes the program sending the updates that show it.
    #[test]
    fn every_browser_tool_runs_on_the_drilled_box() {
        let mut session = Session::new(crate::editor());
        let (status, _) = ok(&mut session, "new_part", json!({}));
        assert_eq!(status["steps"], json!([]));

        let (operations, sent) = ok(&mut session, "list_operations", json!({}));
        assert_eq!(sent, 0, "listing the operations changes nothing");
        assert!(
            (operations["operations"].as_array().unwrap().iter()).any(|op| op["kind"] == "extrude")
        );
        let (extrude, _) = ok(
            &mut session,
            "list_operations",
            json!({ "kind": "extrude" }),
        );
        assert!(extrude["default_args"].is_object(), "{extrude}");
        assert!(
            session.editor().editing().is_none(),
            "the step opened is cancelled"
        );

        for step in drilled_box().steps {
            let (status, sent) = ok(
                &mut session,
                "add_step",
                json!({ "kind": step.operation, "args": step.args, "id": step.id }),
            );
            assert_eq!(status["failed"], 0, "{status}");
            assert!(sent > 0);
        }
        let (status, _) = ok(&mut session, "run", json!({}));
        assert_eq!(status["solids"], json!(["extrude(hole)"]), "{status}");

        let (elements, _) = ok(&mut session, "list_elements", json!({}));
        let faces = elements["faces"].as_array().unwrap();
        assert!(
            faces.iter().any(|f| f["name"] == "extrude(box,end)"),
            "{elements}"
        );

        let (mass, _) = ok(
            &mut session,
            "inspect",
            json!({ "inspector": "mass_properties" }),
        );
        let volume = mass["total"]["volume"]["value"].as_f64().unwrap();
        let expected = 4.0 - std::f64::consts::PI * 0.4 * 0.4 * 0.5;
        assert!((volume - expected).abs() < 1e-6, "volume {volume}");
        let (measured, _) = ok(
            &mut session,
            "inspect",
            json!({ "inspector": "measure", "elements": [{ "kind": "face", "name": "extrude(box,end)" }] }),
        );
        assert!(
            measured.to_string().contains("rea"),
            "no area in {measured}"
        );

        let distance = serde_json::to_value(geop_ops_extrude_revolve::Extents::blind(2.0)).unwrap();
        let (status, _) = ok(
            &mut session,
            "edit_step",
            json!({ "id": "box", "args": { "extent": distance } }),
        );
        assert_eq!(status["failed"], 0, "{status}");
        let (status, _) = ok(&mut session, "delete_step", json!({ "id": "hole" }));
        assert_eq!(status["solids"], json!(["extrude(box)"]), "{status}");
        assert_eq!(session.editor().program().steps.len(), 3);
    }
}
