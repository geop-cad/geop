//! What a part holds: targets, by name ([`TargetReference`]), each the value
//! of a [`Target`] type, made by the steps that produce it — one node per
//! step and target ([`Node`]) — and what a node read while it was made
//! ([`Read`]), which says when it can be reused.

use std::{
    any::{Any, TypeId},
    cell::Cell,
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::values::{Setting, StateValue};

/// A type a part holds targets of. A plain value: compared by `PartialEq`,
/// to tell a node made again into an equal value — whose readers are then
/// reused — from one that changed.
pub trait Target: PartialEq + 'static {
    /// Whether several steps may produce a target of this type, each from
    /// what the steps before left (see [`crate::reader::Reader::previous`]):
    /// the model, the datums, the parameters' values, the listings — what
    /// a program accumulates. A target that is not a chain — a sketch, a
    /// placed part — has one producer, and a second is an error naming
    /// both.
    const CHAIN: bool = false;
}

/// Plain values a target may hold: what a test produces, or a step that
/// produces a number or a text of its own.
macro_rules! plain_targets {
    ($($t:ty),*) => {$(impl Target for $t {})*};
}

plain_targets!(bool, u32, u64, usize, i32, i64, f64, String);

impl<T: PartialEq + 'static> Target for Vec<T> {}

impl<T: PartialEq + 'static> Target for Option<T> {}

/// The stable name of a target, as programs and arguments refer to it, and
/// the type of its value.
pub struct TargetReference<T>(String, std::marker::PhantomData<fn() -> T>);

// By hand, since a derive would ask the same of `T`, and a reference is only
// a name.
impl<T> Clone for TargetReference<T> {
    fn clone(&self) -> Self {
        Self::new(self.0.clone())
    }
}

impl<T> PartialEq for TargetReference<T> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T> Eq for TargetReference<T> {}

impl<T> std::fmt::Debug for TargetReference<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "TargetReference({:?})", self.0)
    }
}

impl<T> TargetReference<T> {
    pub fn new(name: impl Into<String>) -> Self {
        Self(name.into(), std::marker::PhantomData)
    }

    pub fn name(&self) -> &str {
        &self.0
    }
}

/// What a node read while it was made: what has to hold now for it to be
/// reused.
#[derive(Clone, Debug, PartialEq)]
pub enum Read {
    /// The target `name` as the step `producer` produced it, in `version`.
    Target {
        name: String,
        producer: String,
        version: u64,
    },
    /// That no step before produced the target `name`.
    Absent(String),
    /// The value of the run's input `name`, itself: none if none is given.
    Input(String, Option<Setting>),
    /// The value of the state `name`, itself: none if none is given.
    State(String, Option<StateValue>),
    /// The file at `path` of the workspace, in `revision`.
    File(String, u64),
    /// Which targets of one type there are, in order, and who produced
    /// each: what listing them read ([`crate::reader::Reader::targets_of`]).
    /// A target that is added or removed changes it.
    Listed {
        kind: TypeId,
        targets: Vec<(String, String)>,
    },
    /// `read`, in the part placed as `instance`.
    Inside { instance: String, read: Box<Read> },
}

impl Read {
    /// What tells two reads of one make apart: within one make the part
    /// does not change, so two reads of the same thing read the same value.
    pub(crate) fn key(&self) -> String {
        match self {
            Read::Target { name, .. } | Read::Absent(name) => format!("t:{name}"),
            Read::Input(name, _) => format!("i:{name}"),
            Read::State(name, _) => format!("s:{name}"),
            Read::File(path, _) => format!("f:{path}"),
            Read::Listed { kind, .. } => format!("l:{kind:?}"),
            Read::Inside { instance, read } => format!("{instance}/{}", read.key()),
        }
    }
}

thread_local! {
    /// The last version stamp given out.
    static VERSIONS: Cell<u64> = const { Cell::new(0) };
}

/// A version no node has had: what a node gets when its value changes. One
/// counter for every part, so that two parts' versions of one target — the
/// previous and the next build of a placed part — are equal only where one
/// carried the value over from the other.
pub(crate) fn next_version() -> u64 {
    VERSIONS.with(|versions| {
        let next = versions.get() + 1;
        versions.set(next);
        next
    })
}

/// What one step produced of one target: its value, the arguments it was
/// made from and what it read.
#[derive(Clone)]
pub(crate) struct Node {
    /// The step that produced it.
    pub(crate) step: String,
    /// The arguments it was made from: what a reuse compares.
    args: Rc<dyn Any>,
    pub(crate) reads: Rc<[Read]>,
    /// The value, or what made making it fail.
    pub(crate) value: Result<Rc<dyn Any>, String>,
    /// The type of value it is produced to hold, whether or not it failed.
    pub(crate) kind: TypeId,
    /// Whether two values of that type are equal, by its `PartialEq`.
    equal: fn(&dyn Any, &dyn Any) -> bool,
    /// Changes exactly when its value does (see [`next_version`]).
    pub(crate) version: u64,
    /// The run of its part that produced it last, made or reused: what
    /// tells a node of the step being run from one of an earlier run.
    pub(crate) produced_in: u64,
    /// A state it read, directly or through what it read, if it read any:
    /// what makes it move with the state.
    pub(crate) state_read: Option<String>,
}

impl Node {
    /// The node `step` made of `args`, having read `reads`: `made`, in a
    /// new version unless it equals `previous`'s value.
    pub(crate) fn new<Out: Target, Args: PartialEq + 'static>(
        step: &str,
        args: Args,
        reads: Vec<Read>,
        made: Result<Out, String>,
        previous: Option<&Node>,
        produced_in: u64,
        state_read: Option<String>,
    ) -> Self {
        let value = made.map(|value| Rc::new(value) as Rc<dyn Any>);
        let equal: fn(&dyn Any, &dyn Any) -> bool = |a, b| {
            (a.downcast_ref::<Out>()).is_some_and(|a| b.downcast_ref::<Out>() == Some(a))
        };
        let same = previous.is_some_and(|old| {
            old.kind == TypeId::of::<Out>()
                && match (&old.value, &value) {
                    (Ok(a), Ok(b)) => equal(a.as_ref(), b.as_ref()),
                    (Err(a), Err(b)) => a == b,
                    _ => false,
                }
        });
        Self {
            step: step.to_string(),
            args: Rc::new(args),
            reads: reads.into(),
            value,
            kind: TypeId::of::<Out>(),
            equal,
            version: match (same, previous) {
                (true, Some(old)) => old.version,
                _ => next_version(),
            },
            produced_in,
            state_read,
        }
    }

    /// Whether it was made, as an `Out`, from `args`, and holds a value: a
    /// failed node is made again.
    pub(crate) fn made_from<Out: 'static, Args: PartialEq + 'static>(&self, args: &Args) -> bool {
        self.value.is_ok()
            && self.kind == TypeId::of::<Out>()
            && self.args.downcast_ref::<Args>() == Some(args)
    }

    /// Its value, a `T` — or why there is none: it failed, or it is no `T`.
    /// `name` is the target's, for the error.
    pub(crate) fn get<T: 'static>(&self, name: &str) -> GeopResult<&T> {
        let value = self.value.as_ref().map_err(|e| self.failed(name, e))?;
        value.downcast_ref::<T>().ok_or_else(|| {
            GeopError::new(format!(
                "target `{name}` is not a `{}`",
                std::any::type_name::<T>()
            ))
        })
    }

    /// Why it failed, if it did. `name` is the target's, for the error.
    pub(crate) fn failure(&self, name: &str) -> Option<GeopError> {
        self.value.as_ref().err().map(|e| self.failed(name, e))
    }

    /// That it failed, for `e`.
    fn failed(&self, name: &str, e: &str) -> GeopError {
        GeopError::new(format!(
            "target `{name}` failed in step `{}`: {e}",
            self.step
        ))
    }

    /// Whether it holds what `other` holds — a node of another part, say
    /// — by the `PartialEq` of its type: equal values, or failures for the
    /// same reason.
    pub(crate) fn same_as(&self, other: &Node) -> bool {
        self.kind == other.kind
            && match (&self.value, &other.value) {
                (Ok(a), Ok(b)) => Rc::ptr_eq(a, b) || (self.equal)(a.as_ref(), b.as_ref()),
                (Err(a), Err(b)) => a == b,
                _ => false,
            }
    }
}
