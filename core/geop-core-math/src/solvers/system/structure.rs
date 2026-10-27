//! [`System::structure`]: how a system falls apart for a solve — into
//! groups no term ties together, and within each, the blocks that only hang
//! off the others.
//!
//! A screw mated to a bracket, a bracket fastened to a link: the screw's
//! constraints read the screw and the bracket, and decide nothing about the
//! bracket — whatever the bracket does, the screw can follow, and how it
//! does is the screw's own business. Such a block is a **pendant** on the
//! blocks its terms read besides it: it is solved after them, alone, with
//! them where they ended up, and enclosed after them, over their boxes. An
//! assembly of a chain of links with hundreds of parts fastened and mated
//! to them is then one small system of the links, and hundreds of systems
//! of one part each, worked on in lockstep layer by layer — not one dense
//! system of every part, whose solve costs the cube of its size.
//!
//! A block `b` hangs off the blocks `S` its remaining terms read besides it
//! when, where the blocks are:
//!
//! - nothing pulls it, and no limit reads it: either could hold back what
//!   it hangs off, a limit only where it is reached;
//! - its terms hold: it is where they put it, given `S`;
//! - its terms' rows have the same rank by its own steps as by all of `S`'s
//!   and its own: they constrain nothing of `S`, to first order — whatever
//!   `S` does, it can follow. (A part mated to a link and to the ground is
//!   no pendant: its rows also hold the link.)
//! - held where it is ([`Mobility::Held`]), it can be carried along by one
//!   of `S` ([`Variable::carried`]): a solve starts it there. And every
//!   value of `S` that only follows ([`Mobility::Follows`]) is still read by
//!   some other term: it was decided by those, not by where `b` was held. A
//!   joint's coordinate only `b`'s joint reads would otherwise be held
//!   where it was, and `b` turned with the link it hangs off, where the
//!   whole solve held `b` and turned the joint.
//!
//! Blocks are taken off one after another, each pendant's terms with it, as
//! long as one qualifies: a screw comes off its bracket, then the bracket
//! off its link. What is left of each group is its core, solved as before.
//!
//! What changes is only what a block held where it was holds back: a
//! pendant moves with what it hangs off, and of its own freedom stays as
//! near where it was as its terms let it (see [`System::solve`]); but it no
//! longer holds back what it hangs off. A link carrying a hundred screws is
//! as easy to move as one carrying none.

use crate::{
    scalars::Scalar,
    solvers::{linear::eliminate, system::tolerance},
    union_find::UnionFind,
};

use super::{Bound, Equations, Group, Linearized, Mobility, System, Term, Variable, every_term};

/// The groups of a system's blocks a solve works on (see the module docs),
/// in layers: every group's [`Group::on`] blocks are in groups of earlier
/// layers.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Structure {
    pub groups: Vec<Group>,
    /// Per group no term ties to another — before any block was taken off
    /// it — its blocks and terms: what a solve falls back to solving as one
    /// where a pendant cannot follow what it hangs off after all.
    pub components: Vec<Group>,
}

impl Structure {
    /// How many layers its groups are in.
    pub fn layers(&self) -> usize {
        self.groups.iter().map(|g| g.layer + 1).max().unwrap_or(0)
    }

    /// The groups of layer `layer`, by index.
    pub fn layer(&self, layer: usize) -> Vec<usize> {
        (0..self.groups.len())
            .filter(|&g| self.groups[g].layer == layer)
            .collect()
    }
}

/// A term as the structure is found from: what it is, the blocks it reads,
/// and its rows where the blocks are.
struct Read {
    term: Term,
    blocks: Vec<usize>,
    made: Linearized,
}

/// The terms' rows over a box of steps around where the blocks are (see
/// [`System::structure`]), made the first time one is asked: one more
/// evaluation of every term, with every block moved — which a system
/// whose blocks hang off nothing never needs.
struct Over<'s, 'e, E: Equations> {
    system: &'s System<'e, E>,
    at: &'s [E::Value],
    terms: Vec<Term>,
    rows: std::cell::OnceCell<Vec<Option<Linearized>>>,
}

impl<E: Equations> Over<'_, '_, E> {
    /// The rows of the term `k` over the box, as slopes by the steps from
    /// where the blocks are; none where it cannot be made over it.
    fn get(&self, k: usize) -> Option<&Linearized> {
        let rows = self.rows.get_or_init(|| {
            let Some((boxed, differentials)) = self.system.around(self.at) else {
                return vec![None; self.terms.len()];
            };
            (self
                .system
                .equations
                .evaluate(&boxed, &self.terms)
                .into_iter())
            .map(|made| {
                let mut made = made?;
                for (block, rows) in &mut made.jacobian {
                    if let Some(Some(d)) = differentials.get(*block) {
                        for row in rows.iter_mut() {
                            *row = super::compose(row, d);
                        }
                    }
                }
                Some(made)
            })
            .collect()
        });
        rows.get(k)?.as_ref()
    }
}

impl<E: Equations> System<'_, E> {
    /// The structure of the system where the blocks are at `at`: its
    /// groups, the pendants taken off them, in layers (see the module
    /// docs). A term that reads no block, or cannot be made at `at`, is in
    /// none: nothing can be done about it.
    ///
    /// Whether a block's rows constrain what it hangs off is a question of
    /// rank, decided over a box of steps around `at` as wide as the
    /// solver's tolerance, as the enclosure decides its rows (see
    /// [`System::enclose`]): where the blocks are, rounding leaves a hair of
    /// a row that depends on the others, which no box holds apart from
    /// zero.
    pub fn structure(&self, at: &[E::Value]) -> Structure {
        let n = self.mobility.len();
        let terms = every_term(self.equations.count());
        let made = self.equations.evaluate(at, &terms);
        let mut reads: Vec<Read> = Vec::new();
        for (&term, made) in terms.iter().zip(made) {
            let Some(made) = made else {
                continue;
            };
            let mut blocks: Vec<usize> = (made.jacobian.iter())
                .map(|(block, _)| *block)
                .filter(|&b| b < n)
                .collect();
            blocks.sort_unstable();
            blocks.dedup();
            if !blocks.is_empty() {
                reads.push(Read { term, blocks, made });
            }
        }
        let components = grouped(n, reads.iter().map(|r| (r.term, r.blocks.as_slice())));
        let Ok(tolerance) = tolerance(self.scale) else {
            return Structure {
                groups: components.clone(),
                components,
            };
        };
        // Per block, the terms still on it, by index into `reads`.
        let mut on: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (k, read) in reads.iter().enumerate() {
            for &b in &read.blocks {
                on[b].push(k);
            }
        }
        let over = Over {
            system: self,
            at,
            terms: reads.iter().map(|r| r.term).collect(),
            rows: Default::default(),
        };
        let mut alive = vec![true; reads.len()];
        let mut peeled: Vec<Option<Pendant>> = vec![None; n];
        let mut order = Vec::new();
        let mut waiting: std::collections::BTreeSet<usize> = (0..n).collect();
        while let Some(b) = waiting.pop_first() {
            if peeled[b].is_some() {
                continue;
            }
            let own: Vec<usize> = on[b].iter().copied().filter(|&k| alive[k]).collect();
            let Some(pendant) = self.hangs_off(at, b, own, &reads, &over, &alive, &on, tolerance)
            else {
                continue;
            };
            for &k in &pendant.terms {
                alive[k] = false;
            }
            waiting.extend(pendant.on.iter().copied());
            peeled[b] = Some(pendant);
            order.push(b);
        }
        // The cores: what is left of each component.
        let cores = grouped(
            n,
            (reads.iter().zip(&alive))
                .filter(|(_, alive)| **alive)
                .map(|(r, _)| (r.term, r.blocks.as_slice())),
        );
        let component_of =
            |block: usize| (components.iter()).position(|g| g.blocks.binary_search(&block).is_ok());
        let mut groups: Vec<Group> = Vec::new();
        let mut group_of: Vec<Option<usize>> = vec![None; n];
        for mut core in cores {
            // A block taken off leaves a core of nothing.
            core.blocks.retain(|&b| peeled[b].is_none());
            if core.blocks.is_empty() {
                continue;
            }
            core.component = component_of(core.blocks[0]).unwrap_or(0);
            for &b in &core.blocks {
                group_of[b] = Some(groups.len());
            }
            groups.push(core);
        }
        // Pendants after what they hang off: the last taken off first.
        for &b in order.iter().rev() {
            let Some(pendant) = &peeled[b] else {
                continue;
            };
            let layer = (pendant.on.iter())
                .filter_map(|&s| group_of[s].map(|g| groups[g].layer + 1))
                .max()
                .unwrap_or(0);
            let mut group = Group {
                blocks: vec![b],
                on: pendant.on.clone(),
                layer,
                component: component_of(b).unwrap_or(0),
                own: pendant.own,
                ..Default::default()
            };
            for &k in &pendant.terms {
                match reads[k].term {
                    Term::Constraint(c) => group.constraints.push(c),
                    Term::Preference(p) => group.preferences.push(p),
                }
            }
            group_of[b] = Some(groups.len());
            groups.push(group);
        }
        groups.sort_by_key(|g| g.layer);
        Structure { groups, components }
    }

    /// The blocks over the box of steps as wide as the solver's tolerance
    /// around `at`, and per block the differential of its retraction there
    /// (see [`Variable::differential`]); none where one cannot be made.
    #[allow(clippy::type_complexity)]
    fn around(&self, at: &[E::Value]) -> Option<(Vec<E::Value>, Vec<Option<Vec<Vec<Scalar>>>>)> {
        let radius = tolerance(self.scale).ok()?;
        let mut boxed = Vec::with_capacity(at.len());
        let mut differentials = Vec::with_capacity(at.len());
        for (value, mobility) in at.iter().zip(&self.mobility) {
            let step = match mobility.free() {
                true => vec![radius.neg().union(radius); value.dimension()],
                false => vec![Scalar::ZERO; value.dimension()],
            };
            boxed.push(value.retract_over(&step).ok()?);
            differentials.push(value.differential(&step).ok()?);
        }
        Some((boxed, differentials))
    }

    /// What the block `b` hangs off, if it is a pendant (see the module
    /// docs): `own` the terms still on it, by index into `reads`.
    #[allow(clippy::too_many_arguments)]
    fn hangs_off(
        &self,
        at: &[E::Value],
        b: usize,
        own: Vec<usize>,
        reads: &[Read],
        over: &Over<E>,
        alive: &[bool],
        on: &[Vec<usize>],
        tolerance: Scalar,
    ) -> Option<Pendant> {
        let mobility = self.mobility[b];
        if own.is_empty() || !mobility.free() {
            return None;
        }
        let mut hangs: Vec<usize> = Vec::new();
        for &k in &own {
            let read = &reads[k];
            let Term::Constraint(c) = read.term else {
                return None;
            };
            let bound = self.equations.bound(c);
            if bound != Bound::Zero
                || !(read.made.values.iter()).all(|&r| bound.holds(r, tolerance))
            {
                return None;
            }
            hangs.extend(read.blocks.iter().copied().filter(|&o| o != b));
        }
        hangs.sort_unstable();
        hangs.dedup();
        // The rows constrain nothing of what it hangs off.
        let mut columns = vec![b];
        columns.extend(&hangs);
        let mut offsets = Vec::with_capacity(columns.len());
        let mut width = 0;
        for &c in &columns {
            offsets.push(width);
            width += at[c].dimension();
        }
        let own_width = at[b].dimension();
        let mut rows: Vec<Vec<Scalar>> = Vec::new();
        for &k in &own {
            // Hanging off nothing, it constrains nothing else: its rows
            // only count its own freedom, which they do where it is.
            let made = match hangs.is_empty() {
                true => &reads[k].made,
                false => over.get(k)?,
            };
            for r in 0..made.values.len() {
                let mut row = vec![Scalar::ZERO; width];
                for (block, derivatives) in &made.jacobian {
                    let Some(at) = columns.iter().position(|c| c == block) else {
                        continue;
                    };
                    let Some(local) = derivatives.get(r) else {
                        continue;
                    };
                    for (slot, &d) in row.iter_mut().skip(offsets[at]).zip(local) {
                        *slot = slot.add(d);
                    }
                }
                rows.push(row);
            }
        }
        // Eliminated by its own columns only — the others go along — every
        // row left over could be zero: it says nothing its own steps do not.
        // As many pivots as its own columns at most, so the cost is the
        // rows' size, not its square.
        let reduced = eliminate(rows, own_width);
        let constrained = reduced.pivots.len();
        let rest = (reduced.rows.iter()).skip(constrained);
        if !rest.flatten().all(|v| v.could_be_equal(Scalar::ZERO)) {
            return None;
        }
        if mobility == Mobility::Held {
            let carried = (hangs.iter()).any(|&s| at[b].carried(&at[s], &at[s]).is_some());
            if !hangs.is_empty() && !carried {
                return None;
            }
            let decided = |f: usize| {
                (on[f].iter()).any(|&k| {
                    alive[k]
                        && !own.contains(&k)
                        && matches!(reads[k].term, Term::Constraint(c)
                            if self.equations.bound(c) == Bound::Zero)
                })
            };
            if (hangs.iter()).any(|&f| self.mobility[f] == Mobility::Follows && !decided(f)) {
                return None;
            }
        }
        Some(Pendant {
            on: hangs,
            own: own_width - constrained,
            terms: own,
        })
    }
}

/// A block taken off as a pendant: what it hangs off, how many ways its
/// terms leave it free to move with that held, and its terms, by index.
#[derive(Clone)]
struct Pendant {
    on: Vec<usize>,
    own: usize,
    terms: Vec<usize>,
}

/// The groups of `n` blocks the terms `reads` — each with the blocks it
/// reads — tie together, each with its terms, in order of their first
/// blocks.
fn grouped<'r>(n: usize, reads: impl Iterator<Item = (Term, &'r [usize])> + Clone) -> Vec<Group> {
    let mut sets = UnionFind::new(n);
    for (_, blocks) in reads.clone() {
        for pair in blocks.windows(2) {
            sets.union(pair[0], pair[1]);
        }
    }
    let mut by_root: std::collections::BTreeMap<usize, Group> = Default::default();
    for block in 0..n {
        by_root
            .entry(sets.find(block))
            .or_default()
            .blocks
            .push(block);
    }
    for (term, blocks) in reads {
        let Some(group) = by_root.get_mut(&sets.find(blocks[0])) else {
            continue;
        };
        match term {
            Term::Constraint(c) => group.constraints.push(c),
            Term::Preference(p) => group.preferences.push(p),
        }
    }
    by_root.into_values().collect()
}
