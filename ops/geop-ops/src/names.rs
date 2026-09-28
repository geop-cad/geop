//! [`NameRegistry`]: the two-way mapping between an entity and its name that
//! [`crate::Part`] keeps in sync with its topology and sketches, and
//! [`Namer`], which builds those names.

use std::collections::HashMap;

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::ids::RefId;

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

/// Builds the names one run of one operation gives to what it creates:
/// `kind(operation,arg,...)`, see the crate docs.
#[derive(Clone, Debug)]
pub struct Namer {
    kind: String,
    operation: String,
}

impl Namer {
    /// Names for operation `kind` run as the program step `operation`.
    pub fn new(kind: &str, operation: &str) -> GeopResult<Self> {
        validate_operation_id(operation)?;
        Ok(Self {
            kind: kind.to_string(),
            operation: operation.to_string(),
        })
    }

    /// `kind(operation)`: the operation's own name, which is what the solid
    /// it builds is called.
    pub fn root(&self) -> String {
        format!("{}({})", self.kind, self.operation)
    }

    /// `kind(operation,arg,...)`.
    pub fn name(&self, args: &[&str]) -> String {
        let mut name = format!("{}({}", self.kind, self.operation);
        for arg in args {
            name.push(',');
            name.push_str(arg);
        }
        name.push(')');
        name
    }
}
