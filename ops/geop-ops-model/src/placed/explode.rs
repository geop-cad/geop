//! Exploded views: placed parts drawn moved off where they are assembled,
//! move by move. Each move ([`Explosion`]) is a step's ([`explosion`]),
//! read in the order the steps made them ([`explosions`]), and moves the
//! parts it names — and those placed in them — on from wherever the moves
//! before left them ([`exploded`]). What the scene draws and a drawing
//! shows of a placed part is where the moves put it; the state, the mates
//! and every other read of a part's pose stay as assembled.

use geop_core_math::{geop_error::GeopResult, primitives::Pose};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    placed::INSTANCE_SEPARATOR,
    reader::Reader,
    target::{Target, TargetReference},
};

/// One move of an explode sequence: the placed parts it moves, by their
/// paths — a part placed in one of them moves with it — and the motion it
/// moves them by, in the frame of the part they are placed in, after every
/// move before it.
#[derive(Clone, Debug, PartialEq)]
pub struct Explosion {
    pub parts: Vec<String>,
    pub motion: Pose,
}

impl Explosion {
    /// Whether it moves the part placed at `path`: that part, or one it is
    /// placed in.
    pub fn moves(&self, path: &str) -> bool {
        (self.parts.iter()).any(|part| {
            path.strip_prefix(part.as_str())
                .is_some_and(|rest| rest.is_empty() || rest.starts_with(INSTANCE_SEPARATOR))
        })
    }
}

/// Each explode step's own.
impl Target for Explosion {}

/// Read whole.
impl Kind for Explosion {
    const NAME: &'static str = "explosion";
}

/// What the target of an explosion is named, before the step's id.
const PREFIX: &str = "explosion:";

impl HasElements<Explosion> for Explosion {
    type Element<'a> = &'a Explosion;

    fn element(&self, name: &str) -> GeopResult<&Explosion> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}

/// The move the step `step` makes.
pub fn explosion(step: &str) -> TargetReference<Explosion> {
    TargetReference::new(format!("{PREFIX}{step}"))
}

/// Every move the part `reader` reads is exploded by, in the order the
/// steps made them: an explode sequence, as far as the steps run. Which
/// there are is recorded, so a drawing of them is made again when one
/// changes.
pub fn explosions<'a>(reader: &Reader<'a>) -> Vec<&'a Explosion> {
    (reader.targets_of::<Explosion>().into_iter())
        .filter_map(|target| {
            reader
                .retrieve_element(&ElementReference::whole(target))
                .ok()
        })
        .collect()
}

/// Where the placed part at `path`, assembled at `world`, is after the
/// moves `explosions`: each that moves it moving it on.
pub fn exploded(explosions: &[&Explosion], path: &str, world: &Pose) -> Pose {
    (explosions.iter())
        .filter(|explosion| explosion.moves(path))
        .fold(*world, |at, explosion| explosion.motion.compose(&at))
}
