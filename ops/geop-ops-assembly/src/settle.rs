//! [`Settler`]: what settles a program's state after every edit (see
//! [`Settle`]) — moves its placed parts, and every other value a step says
//! a solve moves, so that its mates hold — and the constraints the last
//! settle found conflicting, for the joints panel to show
//! ([`crate::panel`]).

use std::{cell::RefCell, rc::Rc};

use geop_core_editor::{editing::Pull, editor::Rerun, registry::Settle};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{reader::Reader, values::StateValues};

use crate::solving::{self, check, conflicting, solved_states};

/// What the settler and the joints panel share: what the last settle
/// found.
#[derive(Default)]
pub(crate) struct Settled {
    /// A smallest set of the constraints that cannot all hold together, as
    /// the last settle found them: none where they all hold.
    pub(crate) conflicting: RefCell<Vec<String>>,
}

/// Settles a program's state by solving its mates (see [`solving`]).
pub struct Settler {
    pub(crate) settled: Rc<Settled>,
}

impl Settle for Settler {
    /// What the program pulls ([`Pull`]) pulls first, every part that is
    /// not fixed free to give way — a linkage follows the link dragged.
    /// Otherwise the parts move only if a mate does not hold: the step
    /// `own`'s part first, the others held — mating a new part moves it,
    /// not the part it is mated to — and every part that is not fixed only
    /// where that is not enough. Which constraints conflict is asked only
    /// where they do not all hold: a solve, and an enclosure of each group
    /// that fails.
    fn settle(
        &self,
        rerun: &mut Rerun,
        pulls: &[Pull],
        own: Option<&str>,
    ) -> GeopResult<StateValues> {
        let state = rerun.program.state.clone();
        let holds = check(&rerun.reader()).is_ok_and(|report| report.converged);
        let solved = if !pulls.is_empty() {
            solving::solve(rerun, &state, pulls, None, true).ok()
        } else if holds {
            None
        } else {
            let own_first = own
                .and_then(|own| solving::solve(rerun, &state, &[], Some(own), true).ok())
                .filter(|(_, report)| report.converged);
            own_first.or_else(|| solving::solve(rerun, &state, &[], None, true).ok())
        };
        *self.settled.conflicting.borrow_mut() = match &solved {
            Some((_, report)) if !report.converged => {
                conflicting(rerun, &state).unwrap_or_default()
            }
            _ => Vec::new(),
        };
        Ok(solved.map(|(moved, _)| moved).unwrap_or_default())
    }

    fn solvable(&self, reader: &Reader) -> GeopResult<Vec<String>> {
        solved_states(reader)
    }
}
