//! [`Scope`]: which of the parts placed a question about a whole assembly
//! is asked of, so that one changed leg is checked without the rest.

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::placed::INSTANCE_SEPARATOR;
use geop_ops_model::placed::solids::PlacedSolid;
use serde::{Deserialize, Serialize};

/// Which solids a question is asked of: those of the parts placed `parts`
/// (`left_leg`, `left_leg/shin`), with the parts placed in them — all
/// solids, if it names none.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Scope {
    pub parts: Vec<String>,
    /// For a question about pairs: only those with both solids in `parts`,
    /// not each of them against everything.
    pub only_among: bool,
}

/// How far a question was asked before it was stopped (see
/// [`geop_core_program::watch`]): what was not looked at is unknown.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Stopped {
    pub checked: usize,
    pub of: usize,
}

impl Scope {
    /// Whether the part placed at `instance` is named, or placed in one
    /// that is.
    fn names(&self, instance: &str) -> bool {
        self.parts.iter().any(|part| {
            (instance.strip_prefix(part.as_str()))
                .is_some_and(|rest| rest.is_empty() || rest.starts_with(INSTANCE_SEPARATOR))
        })
    }

    /// Whether it asks about the solid of the part placed at `instance`.
    pub fn holds(&self, instance: &str) -> bool {
        self.parts.is_empty() || self.names(instance)
    }

    /// Whether it asks about the pair of solids of the parts placed at `a`
    /// and `b`.
    pub fn holds_pair(&self, a: &str, b: &str) -> bool {
        match self.only_among {
            true => self.holds(a) && self.holds(b),
            false => self.holds(a) || self.holds(b),
        }
    }

    /// Refuses a part that none of `solids` is of, saying which there are:
    /// a misspelt part would otherwise answer that nothing overlaps.
    pub fn check(&self, solids: &[PlacedSolid]) -> GeopResult<()> {
        for part in &self.parts {
            let one = Scope {
                parts: vec![part.clone()],
                only_among: false,
            };
            if !solids.iter().any(|solid| one.names(&solid.instance)) {
                let mut there: Vec<&str> = (solids.iter())
                    .map(|solid| solid.instance.as_str())
                    .filter(|instance| !instance.is_empty())
                    .collect();
                there.sort_unstable();
                there.dedup();
                return Err(GeopError::new(format!(
                    "no solid is of a part placed as {part:?}; the parts with solids are: {}",
                    there.join(", ")
                )));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn of(parts: &[&str], only_among: bool) -> Scope {
        Scope {
            parts: parts.iter().map(|p| p.to_string()).collect(),
            only_among,
        }
    }

    #[test]
    fn a_part_holds_what_is_placed_in_it_and_no_part_named_like_it() {
        let scope = of(&["leg"], false);
        assert!(scope.holds("leg") && scope.holds("leg/shin"));
        assert!(!scope.holds("leg2") && !scope.holds("arm") && !scope.holds(""));
        assert!(Scope::default().holds("") && Scope::default().holds("arm"));
    }

    #[test]
    fn a_pair_is_held_by_either_or_only_by_both() {
        assert!(of(&["leg"], false).holds_pair("leg/shin", "pelvis"));
        assert!(!of(&["leg"], false).holds_pair("arm", "pelvis"));
        assert!(!of(&["leg"], true).holds_pair("leg/shin", "pelvis"));
        assert!(of(&["leg"], true).holds_pair("leg/shin", "leg/foot"));
    }
}
