//! [`NameRegistry`]: the two-way mapping between an entity and its name that
//! [`super::NamedModel`] keeps in sync with its topology, and [`Namer`],
//! which builds those names.

use imbl::HashMap;

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::program::validate_operation_id;

use super::ids::RefId;

/// A two-way `RefId <-> String` mapping. Every entity a
/// [`super::NamedModel`] exposes — a vertex, edge, face or solid — has
/// exactly one live entry here for as long as it exists. Its maps share
/// their structure with their copies, as the topology's do.
///
/// Names are chosen by whoever creates the entity, following the scheme in
/// the crate docs, never generated here from a counter: a counter depends on
/// creation order, which is exactly what a name must not depend on.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct NameRegistry {
    id_to_name: HashMap<RefId, String>,
    name_to_id: HashMap<String, RefId>,
    /// Names of entities made part of another, each with that other's
    /// name: they resolve to it (see [`NameRegistry::alias`]).
    aliases: HashMap<String, String>,
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
        self.aliases.remove(&name);
        self.id_to_name.insert(id, name.clone());
        self.name_to_id.insert(name, id);
        Ok(())
    }

    /// Lets `name`, the name of an entity made part of the entity named
    /// `into` — two faces on one surface made one, two edges joined —
    /// stand for `into` from now on, so what referred to either still
    /// finds it. Fails if `name` names an entity of its own.
    pub fn alias(&mut self, name: impl Into<String>, into: impl Into<String>) -> GeopResult<()> {
        let name = name.into();
        if let Some(&id) = self.name_to_id.get(&name) {
            return Err(GeopError::new(format!(
                "NameRegistry::alias: {name:?} names {id}, it cannot stand for another"
            )));
        }
        let into = into.into();
        // What stood for `name` stands for `into` now.
        let stood: Vec<String> = (self.aliases.iter())
            .filter(|(_, target)| **target == name)
            .map(|(alias, _)| alias.clone())
            .collect();
        for alias in stood {
            self.aliases.insert(alias, into.clone());
        }
        self.aliases.insert(name, into);
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

    pub fn name_of(&self, id: impl Into<RefId>) -> Option<&str> {
        self.id_to_name.get(&id.into()).map(String::as_str)
    }

    /// Whether `name` is an entity's own name — not merely an alias, which
    /// a new entity may take over.
    pub fn is_taken(&self, name: &str) -> bool {
        self.name_to_id.contains_key(name)
    }

    /// The entity named `name`, or that `name` stands for as an alias.
    pub fn id_of(&self, name: &str) -> Option<RefId> {
        (self.name_to_id.get(name).copied())
            .or_else(|| self.name_to_id.get(self.aliases.get(name)?).copied())
    }

    /// Every `(id, name)`, in no particular order.
    pub fn iter(&self) -> impl Iterator<Item = (RefId, &str)> {
        self.id_to_name
            .iter()
            .map(|(&id, name)| (id, name.as_str()))
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
#[derive(Clone, Debug, PartialEq, Eq)]
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

    /// `kind(operation,origin,at)`: a piece split off the entity named
    /// `origin` at — or along — the entity named `at`. A piece is named
    /// after what it was split off first of all, so that [`origin_of`]
    /// traces it back however often it was split since.
    pub fn piece(&self, origin: &str, at: &str) -> String {
        self.name(&[origin, at])
    }

    /// Whether `name` is one of the [`Namer::piece`]s this namer named of
    /// `origin`.
    pub fn is_piece_of(&self, name: &str, origin: &str) -> bool {
        // `kind(operation,origin,` — a piece's name without what it was
        // split at, and its `)`.
        let mut prefix = self.piece(origin, "");
        prefix.pop();
        name.starts_with(&prefix)
    }
}

/// What the entity named `name` was split off (see [`Namer::piece`]): the
/// first of the names among its arguments — an argument that is a name
/// itself, `kind(...)`, or a long one's [`digest`]. None, if it has none.
pub fn origin_of(name: &str) -> Option<&str> {
    let open = name.find('(')?;
    let inner = name[open + 1..].strip_suffix(')')?;
    let mut depth = 0usize;
    let mut start = 0usize;
    // The first argument is the step's id, never a name.
    let mut first = true;
    for (k, c) in inner
        .char_indices()
        .chain(std::iter::once((inner.len(), ',')))
    {
        match c {
            '(' => depth += 1,
            ')' => depth = depth.saturating_sub(1),
            ',' if depth == 0 => {
                let argument = &inner[start..k];
                if !first && (argument.contains('(') || argument.starts_with('#')) {
                    return Some(argument);
                }
                first = false;
                start = k + 1;
            }
            _ => {}
        }
    }
    None
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
/// digits, its digest ([`geop_core_program::cache::digest`]). A hash
/// rather than a counter, so that a name still depends only on what the
/// entity was made from, and one that does not change between Rust
/// versions, so that a saved program's names keep meaning the same
/// entities.
pub fn digest(text: &str) -> String {
    format!("#{:032x}", geop_core_program::cache::digest(text))
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

    #[test]
    fn origin_is_the_first_name_among_the_arguments() {
        assert_eq!(
            origin_of(
                "combine(holes,3,linear_pattern(holes,3,extrude(pin,c1)),extrude(plate,end))"
            ),
            Some("linear_pattern(holes,3,extrude(pin,c1))")
        );
        assert_eq!(origin_of("combine(e,#0123,x)"), Some("#0123"));
        assert_eq!(origin_of("extrude(plate,end)"), None);
        assert_eq!(origin_of("extrude(plate)"), None);
    }

    /// A piece is named after its origin, which [`origin_of`] finds again,
    /// and which [`Namer::is_piece_of`] tells, as the namer named it.
    #[test]
    fn a_piece_is_named_after_its_origin() {
        let namer = Namer::new("combine", "cut").unwrap().scoped("2");
        let piece = namer.piece("extrude(box,end)", "combine(cut,2,x(a),y(b),0,1)");
        assert_eq!(
            piece,
            "combine(cut,2,extrude(box,end),combine(cut,2,x(a),y(b),0,1))"
        );
        assert_eq!(origin_of(&piece), Some("extrude(box,end)"));
        assert!(namer.is_piece_of(&piece, "extrude(box,end)"));
        assert!(!namer.is_piece_of(&piece, "extrude(box)"));
        assert!(!namer.scoped("3").is_piece_of(&piece, "extrude(box,end)"));
    }
}
