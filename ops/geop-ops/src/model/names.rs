//! [`NameRegistry`]: the two-way mapping between an entity and its name that
//! [`crate::Part`] keeps in sync with its topology and sketches, and
//! [`Namer`], which builds those names.

use std::collections::HashMap;

use geop_core_math::geop_error::{GeopError, GeopResult};

use super::ids::RefId;

/// A two-way `RefId <-> String` mapping. Every entity a [`crate::Part`]
/// exposes — a vertex, edge, face, solid or sketch — has exactly one live
/// entry here for as long as it exists.
///
/// Names are chosen by whoever creates the entity, following the scheme in
/// the crate docs, never generated here from a counter: a counter depends on
/// creation order, which is exactly what a name must not depend on.
#[derive(Clone, Debug, Default)]
pub struct NameRegistry {
    id_to_name: HashMap<RefId, String>,
    name_to_id: HashMap<String, RefId>,
}

impl NameRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers `id` under `name`. Fails if `id` is already registered
    /// (under any name) or if `name` is already taken by a different id.
    pub fn insert(&mut self, id: impl Into<RefId>, name: impl Into<String>) -> GeopResult<()> {
        let id = id.into();
        let name = name.into();
        if let Some(existing) = self.id_to_name.get(&id) {
            return Err(GeopError::new(format!(
                "NameRegistry::insert: {id} is already named {existing:?}, cannot also name it {name:?}"
            )));
        }
        if let Some(&existing) = self.name_to_id.get(&name) {
            return Err(GeopError::new(format!(
                "NameRegistry::insert: name {name:?} is already used by {existing}"
            )));
        }
        self.id_to_name.insert(id, name.clone());
        self.name_to_id.insert(name, id);
        Ok(())
    }

    /// Gives the already named `id` the name `new_name` instead.
    ///
    /// For an operation that can only tell what an entity should be called
    /// once it has finished — a boolean numbers the crossings of two edges
    /// along one of them, which it knows only after finding all of them. It
    /// names such entities provisionally while it runs and settles every
    /// name before it returns; nothing outside the operation ever sees a
    /// provisional one.
    pub fn rename(&mut self, id: impl Into<RefId>, new_name: impl Into<String>) -> GeopResult<()> {
        let id = id.into();
        let new_name = new_name.into();
        let old = self.id_to_name.get(&id).cloned().ok_or_else(|| {
            GeopError::new(format!("NameRegistry::rename: {id} has no name to change"))
        })?;
        if old == new_name {
            return Ok(());
        }
        if let Some(&existing) = self.name_to_id.get(&new_name) {
            return Err(GeopError::new(format!(
                "NameRegistry::rename: cannot rename {id} from {old:?} to {new_name:?}, which is already used by {existing}"
            )));
        }
        self.name_to_id.remove(&old);
        self.id_to_name.insert(id, new_name.clone());
        self.name_to_id.insert(new_name, id);
        Ok(())
    }

    /// Forgets `id` and its name. A no-op if `id` was never registered.
    pub fn remove(&mut self, id: impl Into<RefId>) {
        if let Some(name) = self.id_to_name.remove(&id.into()) {
            self.name_to_id.remove(&name);
        }
    }

    /// Forgets every entry whose id `alive` rejects.
    pub fn retain(&mut self, mut alive: impl FnMut(RefId) -> bool) {
        self.id_to_name.retain(|&id, _| alive(id));
        self.name_to_id.retain(|_, id| alive(*id));
    }

    pub fn name_of(&self, id: impl Into<RefId>) -> Option<&str> {
        self.id_to_name.get(&id.into()).map(String::as_str)
    }

    pub fn id_of(&self, name: &str) -> Option<RefId> {
        self.name_to_id.get(name).copied()
    }

    /// Every `(id, name)`, in no particular order.
    pub fn iter(&self) -> impl Iterator<Item = (RefId, &str)> {
        self.id_to_name
            .iter()
            .map(|(&id, name)| (id, name.as_str()))
    }
}

/// Checks that `id` can be an operation id: non-empty, and only ASCII
/// letters, digits, `_`, `-` and `.`. Nothing that could be mistaken for the
/// `(`, `)` and `,` that structure a name, so the names built from it stay
/// unambiguous.
pub fn validate_operation_id(id: &str) -> GeopResult<()> {
    if !id.is_empty()
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.'))
    {
        Ok(())
    } else {
        Err(GeopError::new(format!(
            "{id:?} is not a valid operation id: use only ASCII letters, digits, '_', '-' and '.'"
        )))
    }
}

/// The id of the step that named the entity `name` (see [`Namer`]): `E`
/// for `extrude(E,end)`. None for a name no step built.
pub fn operation_of(name: &str) -> Option<&str> {
    let (_, rest) = name.split_once('(')?;
    let end = rest.find([',', ')'])?;
    Some(&rest[..end])
}

/// Builds the names one run of one operation gives to what it creates:
/// `kind(operation,arg,...)`, see the crate docs.
#[derive(Clone, Debug)]
pub struct Namer {
    kind: String,
    operation: String,
    /// The arguments every name starts with, see [`Namer::scoped`].
    scope: Vec<String>,
}

impl Namer {
    /// Names for operation `kind` run as the program step `operation`.
    pub fn new(kind: &str, operation: &str) -> GeopResult<Self> {
        validate_operation_id(operation)?;
        Ok(Self {
            kind: kind.to_string(),
            operation: operation.to_string(),
            scope: Vec::new(),
        })
    }

    /// The names for one of several parts of the operation's work that
    /// would otherwise name things alike — the second side of an extrude
    /// built on its own, say: `kind(operation,scope,arg,...)`.
    pub fn scoped(&self, scope: &str) -> Self {
        let mut scoped = self.clone();
        scoped.scope.push(scope.to_string());
        scoped
    }

    /// `kind(operation)`: the operation's own name, which is what the solid
    /// it builds is called — `kind(operation,scope)` for a scoped one.
    pub fn root(&self) -> String {
        self.name(&[])
    }

    /// `kind(operation,arg,...)`, an argument longer than
    /// [`LONGEST_ARGUMENT`] given by its digest instead (see [`digest`]).
    pub fn name(&self, args: &[&str]) -> String {
        let mut name = format!("{}({}", self.kind, self.operation);
        for arg in self
            .scope
            .iter()
            .map(String::as_str)
            .chain(args.iter().copied())
        {
            name.push(',');
            if arg.len() > LONGEST_ARGUMENT {
                name.push_str(&digest(arg));
            } else {
                name.push_str(arg);
            }
        }
        name.push(')');
        name
    }
}

/// The longest argument a name spells out (see [`Namer::name`]).
///
/// Names are built from names, and some of them repeat one: a boolean names
/// the piece of an edge it splits after the edge and the vertex it starts
/// at, and that vertex after the edge again. Cutting the same edge again
/// and again — the teeth of a gear, one gap at a time — so doubles its
/// name's length with every cut, and twenty of them made names of tens of
/// megabytes.
pub const LONGEST_ARGUMENT: usize = 160;

/// A long argument of a name, as the name spells it: `#` and 32 hex
/// digits, a 128-bit FNV-1a hash of it. A hash rather than a counter, so
/// that a name still depends only on what the entity was made from; FNV
/// written out here rather than the standard library's hasher, which may
/// change between Rust versions, so that a saved program's names keep
/// meaning the same entities.
pub fn digest(text: &str) -> String {
    const PRIME: u128 = 0x0000000001000000000000000000013B;
    let mut hash: u128 = 0x6c62272e07bb014262b821756295c58d;
    for byte in text.bytes() {
        hash ^= u128::from(byte);
        hash = hash.wrapping_mul(PRIME);
    }
    format!("#{hash:032x}")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A long argument is spelled by its digest: the name stays short, and
    /// the digest depends on the argument alone.
    #[test]
    fn long_arguments_are_digested() {
        let namer = Namer::new("combine", "teeth").unwrap();
        let long = "x".repeat(LONGEST_ARGUMENT + 1);
        let name = namer.name(&["e1", &long]);
        assert_eq!(name, format!("combine(teeth,e1,{})", digest(&long)));
        assert_eq!(digest(&long).len(), 33);
        assert_ne!(digest(&long), digest(&"x".repeat(LONGEST_ARGUMENT + 2)));
        // Fixed for good: saved programs refer to entities by these names.
        assert_eq!(digest(""), "#6c62272e07bb014262b821756295c58d");
        let short = "x".repeat(LONGEST_ARGUMENT);
        assert_eq!(namer.name(&[&short]), format!("combine(teeth,{short})"));
    }
}
