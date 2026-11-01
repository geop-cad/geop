//! The colours of faces, in the part's properties, edited as a front end
//! edits them: a step that comes first, of faces later steps make. (An
//! integration test: its programs are written in `geop-lib-fixtures`, which
//! depends on this crate, and a unit test would hold a second copy of its
//! targets.)

use std::rc::Rc;

use geop_core_editor::{
    editing::{Control, Reference, Tone, Value},
    editor::{Command, Editor, Update},
    registry::Registry,
    testing::{click_at, dialog, editor_with, from_above, handled},
};
use geop_core_program::{
    operation_registry::OperationRegistry,
    program::Program,
    testing::{args, as_step, assert_incremental, assert_round_trips, edit_args, step},
};
use geop_lib_fixtures::{PrimitiveArgs, drilled_box};
use geop_ops_model::{
    element,
    parameters::{
        FaceColor, Material, Parameters, PartPropertiesArgs, parameters_of, read_properties,
        set_parameters,
    },
    read_model,
};

const FACES: &str = "face_color:0:faces";
const COLOUR: &str = "face_color:0:color";

/// The top of the block, as the fixtures name it.
const TOP: &str = "block(box,end)";

/// An editor of what the properties and the fixtures offer, on `program`.
fn editor(program: Program) -> Editor {
    let registry = geop_ops_model::register(Registry::default())
        .and_then(geop_lib_fixtures::register)
        .expect("one name each");
    editor_with(registry, program)
}

/// The operations of [`editor`], as a runner runs them.
fn operations() -> Rc<OperationRegistry> {
    let registry = geop_lib_fixtures::registry().expect("one name each");
    Rc::new(registry)
}

/// A block, its properties the first step, as most programs have them,
/// colouring `colors`.
fn block_with(colors: Vec<FaceColor>) -> Program {
    let mut program = Program::new();
    program.push(
        "properties",
        PartPropertiesArgs {
            title: Some("Block".into()),
            face_colors: colors,
            ..PartPropertiesArgs::default()
        },
    );
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [2.0, 2.0, 1.0]),
    );
    program
}

/// `color` on the faces named `faces`.
fn colored(faces: &[&str], color: &str) -> FaceColor {
    FaceColor {
        faces: faces.iter().map(|name| element(*name)).collect(),
        color: color.into(),
    }
}

/// The properties step opened, as clicking it in the step list does.
fn open(editor: &mut Editor) -> Update {
    handled(
        editor,
        Command::Open {
            id: "properties".into(),
            placed: None,
        },
    )
}

/// The reference field `key` of what `update` shows.
fn reference<'a>(update: &'a Update, key: &str) -> &'a Reference {
    let step = update.step.as_ref().expect("a step is edited");
    match step.presentation.dialog.get(key) {
        Some(Control::Reference(reference)) => reference,
        other => panic!("no reference {key}: {other:?}"),
    }
}

/// What `update` drew of the colours of faces.
fn drawn_colors(update: &Update) -> Vec<(String, String)> {
    let scene = update.scene.as_ref().expect("the part is drawn again");
    (scene.part.face_colors.iter())
        .map(|(face, color)| (face.clone(), color.clone()))
        .collect()
}

fn pair(face: &str, color: &str) -> (String, String) {
    (face.to_string(), color.to_string())
}

/// Opened as a user finds it, in the properties, which come before the
/// block: a colour of faces added waits for faces, a click on the block's
/// top picks it — which the step before it could not see — and the face is
/// drawn in the colour at once, not lit over it. The colour set and the
/// step committed, the part is drawn so; a second face picked goes with it,
/// and taken out, the entry takes its colour with it.
#[test]
fn a_face_a_later_step_makes_is_coloured_in_the_properties() {
    let mut editor = editor(block_with(Vec::new()));
    open(&mut editor);
    let added = handled(
        &mut editor,
        dialog("face_color_add", Value::Choice("add".into())),
    );
    let faces = reference(&added, FACES);
    assert!(
        faces.armed && faces.multiple && faces.value.is_empty(),
        "{faces:?}"
    );
    let refused = editor.handle(Command::Commit).error.expect("no faces yet");
    assert!(refused.contains("Faces"), "{refused}");

    // The top, seen from above.
    let picked = handled(&mut editor, click_at(from_above(1.0, 1.0)));
    let faces = reference(&picked, FACES);
    let held: Vec<_> = faces
        .value
        .iter()
        .map(|p| p.element.name.as_str())
        .collect();
    assert_eq!(held, [TOP]);
    assert_eq!(drawn_colors(&picked), [pair(TOP, "#d9483b")]);
    let step = picked.step.as_ref().unwrap();
    assert!(
        step.presentation.highlights.is_empty(),
        "lit over its colour"
    );

    let set = handled(&mut editor, dialog(COLOUR, Value::Text("#123456".into())));
    assert_eq!(drawn_colors(&set), [pair(TOP, "#123456")]);
    let committed = handled(&mut editor, Command::Commit);
    assert!(committed.step.is_none());
    assert_eq!(
        (
            editor.program().steps[0].operation.as_str(),
            drawn_face(&editor)
        ),
        ("properties", TOP.to_string())
    );

    // Opened again: it holds it, and taking the entry out takes the colour
    // off.
    let opened = open(&mut editor);
    assert_eq!(reference(&opened, FACES).value.len(), 1);
    let taken = handled(
        &mut editor,
        dialog("face_color:0:remove", Value::Choice("remove".into())),
    );
    assert!(drawn_colors(&taken).is_empty());
    handled(&mut editor, Command::Commit);
    assert!(
        read_properties(&editor.part())
            .unwrap()
            .face_colors
            .is_empty()
    );
    let saved = editor.program().to_json().unwrap();
    assert!(!saved.contains("face_colors"), "{saved}");
}

/// The one face the properties of `editor`'s part colour.
fn drawn_face(editor: &Editor) -> String {
    let properties = read_properties(&editor.part()).unwrap().into_owned();
    match &properties.face_colors[..] {
        [entry] => entry.faces[0].name().to_string(),
        other => panic!("not one entry: {other:?}"),
    }
}

/// A colour changed builds nothing again and meshes nothing: the very same
/// model and meshes are drawn, counted by what the runs made — the
/// properties' own target, and no more.
#[test]
fn a_colour_changed_builds_nothing_anew() {
    let mut editor = editor(block_with(vec![colored(&[TOP], "#ff0000")]));
    let model = |editor: &Editor| match read_model(&editor.part()) {
        Ok(std::borrow::Cow::Borrowed(built)) => built as *const _ as usize,
        other => panic!("no model built: {:?}", other.err()),
    };
    let before = model(&editor);
    let shown = handled(&mut editor, Command::Show);
    let meshes: Vec<_> = (shown.scene.as_ref().unwrap().part.faces.iter())
        .map(|face| face.mesh.clone())
        .collect();
    let makes = editor.counts().makes;

    open(&mut editor);
    let set = handled(&mut editor, dialog(COLOUR, Value::Text("#00ff00".into())));
    assert_eq!(drawn_colors(&set), [pair(TOP, "#00ff00")]);
    let scene = set.scene.as_ref().unwrap();
    assert_eq!(scene.part.faces.len(), meshes.len());
    assert!(
        scene.meshes.is_empty(),
        "no mesh sent again: {:?}",
        scene.meshes.keys()
    );
    assert!(
        (scene.part.faces.iter())
            .zip(&meshes)
            .all(|(face, mesh)| Rc::ptr_eq(&face.mesh, mesh)),
        "meshed again"
    );
    handled(&mut editor, Command::Commit);
    assert_eq!(model(&editor), before, "a colour rebuilt the model");
    assert_eq!(
        editor.counts().makes - makes,
        1,
        "only the properties are made again"
    );
}

/// A name no face has now fails nothing: the part builds, is drawn, and can
/// be saved; the name is told beside the faces picked, with the rest fine.
#[test]
fn a_name_no_face_has_is_told_in_the_form_and_fails_nothing() {
    let mut editor = editor(block_with(vec![colored(&[TOP, "gone"], "#ff0000")]));
    let shown = handled(&mut editor, Command::Show);
    assert!(editor.program().steps.len() == 2 && shown.error.is_none());
    let program = shown.program.as_ref().expect("the program is listed");
    assert!(program.steps.iter().all(|step| step.error.is_none()));

    let opened = open(&mut editor);
    let held = &reference(&opened, FACES).value;
    assert_eq!(held[0].tone, Tone::Normal, "{:?}", held[0].detail);
    assert_eq!(held[1].tone, Tone::Error);
    let told = held[1].detail.as_deref().expect("why");
    assert!(told.contains("no face"), "{told}");
    assert!(opened.step.as_ref().unwrap().error.is_none());
    // Nothing keeps it from being committed as it is.
    handled(&mut editor, Command::Commit);
}

/// Without a face of the name left — the step making it gone — the same.
#[test]
fn a_face_whose_step_is_taken_out_is_told_when_opened() {
    let mut program = block_with(vec![colored(&[TOP], "#ff0000")]);
    let mut editor = editor(program.clone());
    program.steps.truncate(1);
    handled(
        &mut editor,
        Command::Load {
            program,
            path: None,
        },
    );
    let opened = open(&mut editor);
    assert_eq!(reference(&opened, FACES).value[0].tone, Tone::Error);
}

/// A colour that is none fails the step, beside the field it was typed in.
#[test]
fn a_colour_that_is_none_is_told_beside_its_field() {
    let mut editor = editor(block_with(vec![colored(&[TOP], "#ff0000")]));
    open(&mut editor);
    let wrong = handled(&mut editor, dialog(COLOUR, Value::Text("red".into())));
    let step = wrong.step.as_ref().unwrap();
    let field = (step.presentation.dialog.0.iter())
        .find(|field| field.key == COLOUR)
        .expect("the field");
    assert!(
        field
            .failure
            .as_deref()
            .is_some_and(|f| f.contains("#rrggbb")),
        "{:?}",
        field.failure
    );
}

/// Saved and read back, they are the same; a program with none saves as it
/// did. A colour changed, the part reads as one built afresh.
#[test]
fn colours_of_faces_round_trip_and_build_incrementally() {
    let colors = vec![
        colored(&[TOP], "#ff0000"),
        colored(&["block(box,start)", "gone"], "#00ff00"),
    ];
    let program = block_with(colors.clone());
    let part = assert_round_trips(operations(), None, &program);
    let properties = read_properties(&part.current()).unwrap().into_owned();
    assert_eq!(properties.face_colors, colors);
    let json = program.to_json().unwrap();
    assert!(json.contains(TOP), "saved by the face's name: {json}");

    let plain = block_with(Vec::new()).to_json().unwrap();
    assert!(!plain.contains("face_colors"), "{plain}");

    let mut changed = program.clone();
    edit_args::<PartPropertiesArgs>(step(&mut changed, "properties"), |args| {
        args.face_colors[0].color = "#0000ff".into();
    });
    assert_incremental(operations(), None, &[program, changed]);
}

/// Setting a program's parameters and properties leaves its colours of
/// faces, which are neither: a material chosen must not take them away.
#[test]
fn setting_the_parameters_keeps_the_colours_of_faces() {
    let colors = vec![colored(&[TOP], "#ff0000")];
    let mut program = block_with(colors.clone());
    let material = Material::named("Steel");
    let parameters = Parameters {
        material: material.clone(),
        ..parameters_of(&program)
    };
    set_parameters(&mut program, parameters);
    assert_eq!(parameters_of(&program).material, material);
    let args: PartPropertiesArgs = args(step(&mut program, "properties"));
    assert_eq!(args.face_colors, colors);
}

/// The drilled box's faces are all pickable from the properties put first:
/// the hole's wall, a face of a cut, too.
#[test]
fn a_face_a_cut_leaves_is_picked_from_before_it() {
    let mut program = drilled_box();
    let properties = as_step("properties", PartPropertiesArgs::default());
    program.steps.insert(0, properties);
    let mut editor = editor(program);
    open(&mut editor);
    handled(
        &mut editor,
        dialog("face_color_add", Value::Choice("add".into())),
    );
    // Down the hole, on its floor.
    let picked = handled(&mut editor, click_at(from_above(1.0, 1.0)));
    let name = reference(&picked, FACES).value[0].element.name.clone();
    assert_ne!(name, "block(box,end)", "the floor, not the top");
    assert_eq!(drawn_colors(&picked), [pair(&name, "#d9483b")]);
}
