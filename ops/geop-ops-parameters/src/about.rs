//! What a run is given besides numbers, as targets the given step defines
//! (see [`geop_core_program::program_runner::Given`]): what the program
//! says of its part ([`About`]) — its file, its parameters, the rows and the
//! colour it is built with — and the other files it can read ([`Files`]).

use std::{borrow::Cow, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::{Input, Part, Reader, State},
    target::TargetReference,
};

use crate::parameters::{COLOR, Parameters, Setting, Settings};

/// What the program a run builds says of its part besides its steps: the
/// file it is in, as the library names files, the parameters it declares —
/// its colour, material, title and designation — and what they were set to
/// for this run: each table's row, each number, the colour.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct About {
    pub file: String,
    pub parameters: Parameters,
    pub chosen: Settings,
}

/// The run's [`About`], defined by the given step.
pub fn about() -> TargetReference<About> {
    TargetReference::new("about")
}

/// Read whole.
impl Kind for About {
    const NAME: &'static str = "about";
}

impl HasElements<About> for About {
    type Element<'a> = &'a About;
    const SINGLETON: Option<&'static str> = Some("about");

    fn element(&self, name: &str) -> GeopResult<&About> {
        whole(self, name)
    }
}

/// Defines what `about` says as the run's [`about`]: in the given step.
pub fn define_about<I: Input, S: State>(part: &mut Part<I, S>, about: &About) -> GeopResult<()> {
    part.define_target(&self::about(), about, |_, about| Ok(about.clone()))
}

/// What the program of the part `reader` reads says of it: nothing, for a
/// part built without a program — an operation's test.
pub fn read_about<'a, I: Input, S: State>(reader: &Reader<'a, I, S>) -> GeopResult<Cow<'a, About>> {
    if !reader.defines(&about()) {
        return Ok(Cow::Owned(About::default()));
    }
    Ok(Cow::Borrowed(reader.retrieve_element(&ElementReference::whole(about()))?))
}

/// What a part whose material is not given is weighed as: water, 1000
/// kg/m³.
const UNGIVEN_DENSITY: f64 = 1000.0;

/// What a part is made of, as it is weighed.
#[derive(Clone, Debug, PartialEq)]
pub struct MaterialOf {
    pub name: String,
    /// kg/m³.
    pub density: f64,
    /// Whether no material was given, and it is weighed as water.
    pub assumed: bool,
}

impl About {
    /// What its part is made of: its material — or, not given, water, and
    /// said so.
    pub fn material(&self) -> MaterialOf {
        match &self.parameters.material {
            Some(material) => MaterialOf {
                name: material.name.clone(),
                density: material.density,
                assumed: false,
            },
            None => MaterialOf {
                name: "Water (no material given)".into(),
                density: UNGIVEN_DENSITY,
                assumed: true,
            },
        }
    }

    /// Its part's colour, `#rrggbb`: none for the viewer's own.
    pub fn color(&self) -> Option<&str> {
        match self.chosen.get(COLOR) {
            Some(Setting::Text(color)) => Some(color),
            _ => None,
        }
    }

    /// The name of its file, without folders or `.geop`: what a file
    /// written from its part is named after — `part` if that leaves
    /// nothing.
    pub fn stem(&self) -> &str {
        let file = self.file.rsplit(['/', ':']).next().unwrap_or_default();
        match file.trim_end_matches(".geop") {
            "" => "part",
            stem => stem,
        }
    }

    /// What its part is called where it is listed: its title, else its
    /// file's [`About::stem`].
    pub fn name(&self) -> &str {
        self.parameters.title.as_deref().unwrap_or(self.stem())
    }

    /// What its part is ordered as, with what it was built with (see
    /// [`Parameters::designate`]): none for a part made rather than bought.
    pub fn designation(&self) -> Option<String> {
        self.parameters.designate(&self.chosen)
    }
}

/// The other files a program being built can read, as it names them:
/// what a step importing a file reads.
pub trait Files {
    /// The files it could read: every file but its own.
    fn files(&self) -> Vec<String>;

    /// The file `file`, relative to the program being built: its path, as
    /// the library names files, and its text.
    fn read(&self, file: &str) -> GeopResult<(String, String)>;
}

/// A value given to a run that is no data — the files around a program, the
/// parts it can place — shared by the run and its giver. Equal only to
/// itself, so a step reading it is rebuilt when it is given another.
pub struct Shared<T: ?Sized>(pub Rc<T>);

impl<T: ?Sized> Clone for Shared<T> {
    fn clone(&self) -> Self {
        Self(self.0.clone())
    }
}

impl<T: ?Sized> PartialEq for Shared<T> {
    fn eq(&self, other: &Self) -> bool {
        Rc::ptr_eq(&self.0, &other.0)
    }
}

/// The files a run can read, defined by the given step.
pub fn files() -> TargetReference<Shared<dyn Files>> {
    TargetReference::new("files")
}

/// Read whole.
impl Kind for Shared<dyn Files> {
    const NAME: &'static str = "files";
}

impl HasElements<Shared<dyn Files>> for Shared<dyn Files> {
    type Element<'a> = &'a dyn Files;
    const SINGLETON: Option<&'static str> = Some("files");

    fn element(&self, name: &str) -> GeopResult<&dyn Files> {
        Ok(whole(self, name)?.0.as_ref())
    }
}

/// Defines `files` as the run's [`files`]: in the given step.
pub fn define_files<I: Input, S: State>(
    part: &mut Part<I, S>,
    files: &Shared<dyn Files>,
) -> GeopResult<()> {
    part.define_target(&self::files(), files, |_, files| Ok(files.clone()))
}

/// The files a run of `reader` can read: an error, for a run given none.
pub fn read_files<'a, I: Input, S: State>(reader: &Reader<'a, I, S>) -> GeopResult<&'a dyn Files> {
    if !reader.defines(&files()) {
        return Err(GeopError::new(
            "this program is built without any other files: there is none to read",
        ));
    }
    reader.retrieve_element(&ElementReference::whole(files()))
}

/// What a field picking a file to read offers: the files of it `kind`
/// takes, sorted, and `current`, if a file is picked that is not among
/// them. None, without files.
pub fn file_choices<I: Input, S: State>(
    reader: &Reader<I, S>,
    kind: impl Fn(&str) -> bool,
    current: &str,
) -> Vec<String> {
    let mut choices: Vec<String> = (read_files(reader).map(|files| files.files()))
        .unwrap_or_default()
        .into_iter()
        .filter(|f| kind(f))
        .collect();
    choices.sort();
    if !current.is_empty() && !choices.iter().any(|f| f == current) {
        choices.push(current.to_string());
    }
    choices
}
