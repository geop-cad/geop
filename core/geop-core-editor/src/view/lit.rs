//! [`Lit`]: what of a drawn part a set of elements lights — the faces,
//! edges, vertices, solids, sketches and datums a viewer draws lit for the
//! highlights of a [`crate::editing::Presentation`], or for what an answer
//! or an item of the structure points at — and [`Lighting`], what of the
//! part drawn and of each part placed in it is lit. The editor sends the
//! lighting ([`crate::editor::Update::lit`]); a front end only draws it,
//! so that every front end lights the same things.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_program::element::PickedElement;
use serde::Serialize;

use super::{PartView, drawn};

/// What a drawn part's elements are named in: the targets a lit element
/// is of, which tell what it lights.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Targets {
    /// The target the part's vertices, edges, faces and solids are of.
    pub model: String,
    /// The target its datums are of.
    pub datum_target: String,
    /// Each sketch's name, by the target it is.
    pub sketches: BTreeMap<String, String>,
}

/// What a set of elements lights of one drawn part (see the module docs).
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash, Serialize)]
pub struct Lit {
    /// Whether all of it is lit: a placed part hovered or selected whole.
    pub whole: bool,
    pub faces: BTreeSet<String>,
    pub edges: BTreeSet<String>,
    pub vertices: BTreeSet<String>,
    pub solids: BTreeSet<String>,
    /// Steps whose features' faces are lit, wherever they are.
    pub features: BTreeSet<String>,
    /// Whole sketches, by name.
    pub sketches: BTreeSet<String>,
    /// Curves of sketches, by the sketch's name and the curve's id.
    pub curves: BTreeSet<(String, String)>,
    /// Points of sketches, by the sketch's name and the point's id.
    pub sketch_points: BTreeSet<(String, String)>,
    /// Datums and their components, by name as drawn: `origin:x` an axis.
    pub datums: BTreeSet<String>,
}

impl Targets {
    /// The targets the elements of `view` are named in.
    pub fn of(view: &PartView) -> Self {
        Self {
            model: view.model.clone(),
            datum_target: view.datum_target.clone(),
            sketches: (view.sketches.iter())
                .map(|s| (s.target.clone(), s.name.clone()))
                .collect(),
        }
    }
}

impl Lit {
    /// What `elements` light of the part named in `targets`, drawn as the
    /// placed part `instance` — `""` for the part itself: those of the
    /// part's own model lighting its faces, edges, vertices and solids by
    /// name, whatever kind of thing drawn as them they are (the plane a
    /// face is, the line an edge runs along); those of a sketch, it, or
    /// its curves and points; those of its datums, them; a feature, the
    /// faces it made.
    pub fn of<'e>(
        targets: &Targets,
        instance: &str,
        elements: impl IntoIterator<Item = &'e PickedElement>,
    ) -> Self {
        let mut lit = Self::default();
        for e in elements {
            if e.target.path() != instance {
                continue;
            }
            let target = e.target.name();
            let name = e.name.clone();
            if e.kind == "feature" {
                lit.features.insert(name);
            } else if let Some(sketch) = targets.sketches.get(target) {
                match e.kind.as_str() {
                    drawn::SKETCH_CURVE => lit.curves.insert((sketch.clone(), name)),
                    drawn::SKETCH_POINT => lit.sketch_points.insert((sketch.clone(), name)),
                    _ => lit.sketches.insert(sketch.clone()),
                };
            } else if target == targets.datum_target {
                lit.datums.insert(name);
            } else if target == targets.model {
                match e.kind.as_str() {
                    drawn::FACE | "plane" | "round" => lit.faces.insert(name),
                    drawn::EDGE | "line" | "curve" | "circle" | "path" => lit.edges.insert(name),
                    drawn::VERTEX | "point" => lit.vertices.insert(name),
                    "solid" | "body" => lit.solids.insert(name),
                    _ => false,
                };
            }
        }
        lit
    }

    /// Whether the face `name`, of `solid`, made by `feature`, is lit.
    pub fn face(&self, name: &str, solid: Option<&str>, feature: Option<&str>) -> bool {
        self.whole
            || self.faces.contains(name)
            || solid.is_some_and(|s| self.solids.contains(s))
            || feature.is_some_and(|f| self.features.contains(f))
    }

    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }
}

/// What is lit of the part drawn — under `""` — and of each part placed in
/// it, by its name (`bolt`, `asm/bolt`): only those of which something is.
pub type Lighting = BTreeMap<String, Lit>;

/// What `elements` light of `part` and the parts placed in it, the placed
/// parts `whole` lit whole. Only the parts an element is of are looked at,
/// so lighting one face of an assembly of thousands of parts is cheap.
pub fn lighting(part: &PartView, elements: &[PickedElement], whole: &BTreeSet<&str>) -> Lighting {
    let mut by_part: BTreeMap<&str, Vec<&PickedElement>> = BTreeMap::new();
    for e in elements {
        by_part.entry(e.target.path()).or_default().push(e);
    }
    let mut lighting = Lighting::new();
    let mut light = |name: &str, view: &PartView, whole: bool| {
        let mut lit = Lit::of(
            &Targets::of(view),
            name,
            by_part.get(name).into_iter().flatten().copied(),
        );
        lit.whole = whole;
        if !lit.is_empty() {
            lighting.insert(name.to_string(), lit);
        }
    };
    if by_part.contains_key("") {
        light("", part, false);
    }
    if by_part.keys().any(|path| !path.is_empty()) || !whole.is_empty() {
        for instance in &part.instances {
            let name = instance.name.as_str();
            if by_part.contains_key(name) || whole.contains(name) {
                light(name, &instance.view, whole.contains(name));
            }
        }
    }
    lighting
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_program::placed::Located;

    fn element(path: &str, target: &str, kind: &str, name: &str) -> PickedElement {
        PickedElement {
            target: Located::in_instance(path, target),
            kind: kind.into(),
            name: name.into(),
        }
    }

    fn targets() -> Targets {
        Targets {
            model: "model".into(),
            datum_target: "datums".into(),
            sketches: BTreeMap::from([("sketch:s".into(), "s".into())]),
        }
    }

    #[test]
    fn elements_light_what_they_are_drawn_as_in_their_own_part() {
        let lit = Lit::of(
            &targets(),
            "",
            &[
                element("", "model", "face", "f"),
                element("", "model", "plane", "p"),
                element("", "model", "line", "l"),
                element("", "model", "solid", "body"),
                element("", "sketch:s", "sketch_curve", "c"),
                element("", "sketch:s", "sketch", ""),
                element("", "datums", "datum", "origin:x"),
                element("", "model", "feature", "hole"),
                element("bolt", "model", "face", "elsewhere"),
            ],
        );
        assert_eq!(lit.faces, BTreeSet::from(["f".into(), "p".into()]));
        assert_eq!(lit.edges, BTreeSet::from(["l".into()]));
        assert!(lit.face("other", Some("body"), None));
        assert!(lit.face("other", None, Some("hole")));
        assert!(!lit.face("elsewhere", None, None));
        assert!(lit.curves.contains(&("s".into(), "c".into())));
        assert!(lit.sketches.contains("s"));
        assert!(lit.datums.contains("origin:x"));
        let placed = Lit::of(&targets(), "bolt", &[element("bolt", "model", "face", "x")]);
        assert!(placed.faces.contains("x"));
    }
}
