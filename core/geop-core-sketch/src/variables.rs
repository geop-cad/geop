//! What the planar and the 3-D sketch's solves share: the sketch's
//! variables, some free, and its constraints, each residuals of a few of
//! them computed in dual numbers, as a system of the generic solver
//! ([`geop_core_math::solvers::system`]): every free variable a block of
//! one number, every constraint a term over the variables it reads, every
//! dragged variable a preference.

use std::collections::BTreeMap;

use crate::sketch::PointId;
use geop_core_math::{
    dual::Dual,
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::system::{
        Bound, EncloseError, Enclosure, Equations, Held, Linearized, Mobility, Solution, System,
        Term,
    },
    vector::Vector,
};

/// A point of a sketch as its solve reads and writes it: `D` coordinates,
/// and whether it is given rather than solved for.
pub(crate) trait SketchPoint<const D: usize> {
    fn fixed(&self) -> bool;
    fn coordinates(&self) -> [Scalar; D];
    fn set_coordinates(&mut self, at: [Scalar; D]);
}

/// Where a sketch's points are among its variables: one block of `D`
/// coordinates for each class of coincident points (see
/// [`crate::sketch::Sketch::point_classes`]) — coincidence is no residual,
/// it removes the coordinates by construction — none of them free for a
/// class with a fixed point. Whatever else a sketch solves for follows
/// ([`PointLayout::push`]).
pub(crate) struct PointLayout<const D: usize> {
    /// The index of each point's first coordinate (the others follow).
    pub point_var: BTreeMap<PointId, usize>,
    /// Per variable: whether the solver may change it.
    pub free: Vec<bool>,
}

impl<const D: usize> PointLayout<D> {
    /// The blocks of `points`, coincident by `classes`.
    pub fn new<P: SketchPoint<D>>(
        points: &BTreeMap<PointId, P>,
        classes: &BTreeMap<PointId, PointId>,
    ) -> Self {
        let mut free = Vec::new();
        let mut class_var = BTreeMap::new();
        for rep in classes.values() {
            class_var.entry(*rep).or_insert_with(|| {
                free.extend([true; D]);
                free.len() - D
            });
        }
        let point_var: BTreeMap<PointId, usize> = classes
            .iter()
            .map(|(&p, rep)| (p, class_var[rep]))
            .collect();
        for (id, p) in points {
            if p.fixed() {
                free[point_var[id]..point_var[id] + D].fill(false);
            }
        }
        PointLayout { point_var, free }
    }

    /// One more variable, free or not: its index.
    pub fn push(&mut self, free: bool) -> usize {
        self.free.push(free);
        self.free.len() - 1
    }

    /// How many variables there are.
    pub fn len(&self) -> usize {
        self.free.len()
    }

    /// Per variable, its index among the free ones: how the solver's own
    /// vectors are laid out.
    pub fn offsets(&self) -> Vec<Option<usize>> {
        let mut n = 0;
        self.free
            .iter()
            .map(|&free| {
                free.then(|| {
                    n += 1;
                    n - 1
                })
            })
            .collect()
    }

    /// The variables of point `p`, added to `out`.
    pub fn point_vars(&self, p: PointId, out: &mut Vec<usize>) {
        let i = self.point_var[&p];
        out.extend(i..i + D);
    }

    /// The point `p` of the variables `x`.
    pub fn point<T: Copy>(&self, x: &[T], p: PointId) -> [T; D] {
        let i = self.point_var[&p];
        std::array::from_fn(|k| x[i + k])
    }

    /// The points' variables, read from `points` into `x`: a class where
    /// its fixed point is, else where its representative (lowest id) is.
    /// Where things are is the sketch's state, a free choice, sharp.
    pub fn read<P: SketchPoint<D>>(&self, points: &BTreeMap<PointId, P>, x: &mut [Scalar]) {
        // In reverse, so a class takes its representative's position — or,
        // after, its fixed point's.
        let points = points.iter().rev();
        for (id, p) in points
            .clone()
            .filter(|(_, p)| !p.fixed())
            .chain(points.filter(|(_, p)| p.fixed()))
        {
            let i = self.point_var[id];
            x[i..i + D].copy_from_slice(&p.coordinates());
        }
    }

    /// Writes the points of `x` back into `points`, all but the fixed
    /// ones, which stay as given.
    pub fn write<P: SketchPoint<D>>(&self, points: &mut BTreeMap<PointId, P>, x: &[Scalar]) {
        for (id, p) in points.iter_mut().filter(|(_, p)| !p.fixed()) {
            p.set_coordinates(self.point(x, *id));
        }
    }
}

/// A sketch's constraints over its variables: what [`Variables`] needs of
/// the planar and the 3-D sketch alike. `N` is the most variables one
/// constraint reads.
pub(crate) trait Residuals<const N: usize> {
    /// Per variable, its block if it is free.
    fn offsets(&self) -> Vec<Option<usize>>;

    /// How many constraints there are.
    fn constraints(&self) -> usize;

    /// The variables the constraint `c` reads, at most `N`.
    fn reads(&self, c: usize) -> &[usize];

    /// The residuals of the constraint `c`, each a length, the variables it
    /// reads at `values`, in the order of [`Residuals::reads`]. `Err` where
    /// one is undecidable.
    fn residuals(&self, c: usize, values: &[Dual<N>], out: &mut Vec<Dual<N>>) -> GeopResult<()>;

    /// The constraint `c`, as an error names it.
    fn name(&self, c: usize) -> String;

    /// The sketch's size (see [`System::scale`]).
    fn scale(&self) -> Scalar;
}

/// A sketch's constraints, and the variables a drag pulls, as a system of
/// its free variables.
pub(crate) struct Variables<'p, P, const N: usize> {
    pub problem: &'p P,
    /// Every variable's value: where the fixed ones stay.
    params: Vec<Scalar>,
    /// Per variable, its block if it is free.
    offsets: Vec<Option<usize>>,
    /// The variables pulled, and where to.
    pulls: Vec<(usize, Scalar)>,
}

impl<'p, const N: usize, P: Residuals<N>> Variables<'p, P, N> {
    pub fn new(problem: &'p P, params: Vec<Scalar>, pulls: Vec<(usize, Scalar)>) -> Self {
        Variables {
            offsets: problem.offsets(),
            problem,
            params,
            pulls,
        }
    }

    /// The system solved from `params` with each `(point, target)` pulled
    /// towards its target as far as the constraints allow — dragging;
    /// `point_var` names each point's first coordinate, the others follow.
    /// The variables, the solution, and every variable where the solve put
    /// it.
    pub fn dragged<Id: Ord, const D: usize>(
        problem: &'p P,
        params: Vec<Scalar>,
        point_var: &BTreeMap<Id, usize>,
        drags: &[(Id, Vector<D>)],
    ) -> GeopResult<(Self, Solution<Scalar>, Vec<Scalar>)> {
        // A dragged point is pulled towards the cursor among the
        // configurations that meet the constraints: it follows the cursor
        // exactly where it is free to.
        let pulls = (drags.iter())
            .flat_map(|(point, target)| {
                let i = point_var[point];
                (0..D).map(move |k| (i + k, target[k]))
            })
            .collect();
        let variables = Variables::new(problem, params, pulls);
        let solved = variables.system().solve(&variables.start())?;
        let all = variables.all(&solved.values);
        Ok((variables, solved, all))
    }

    /// The free variables' values: the blocks where a solve starts.
    pub fn start(&self) -> Vec<Scalar> {
        (self.params.iter().zip(&self.offsets))
            .filter(|(_, offset)| offset.is_some())
            .map(|(&v, _)| v)
            .collect()
    }

    /// Every variable's value, the free ones at `blocks`.
    pub fn all(&self, blocks: &[Scalar]) -> Vec<Scalar> {
        (self.params.iter().zip(&self.offsets))
            .map(|(&given, offset)| offset.map_or(given, |o| blocks[o]))
            .collect()
    }

    /// The system: every free variable held where it was, as far as the
    /// constraints allow.
    pub fn system(&self) -> System<'_, Self> {
        System {
            equations: self,
            mobility: vec![Mobility::Held; self.start().len()],
            scale: self.problem.scale(),
        }
    }

    /// Why `error` proves no solution, each constraint by its name.
    pub fn named(&self, error: EncloseError) -> GeopError {
        error.named(|c| self.problem.name(c))
    }

    /// The exact solution near `blocks`, a solution to the solver's
    /// tolerance: every variable, the free ones enclosed (see
    /// [`System::enclose`]); fails, naming the constraints, where it is
    /// not proven.
    pub fn enclosed(&self, blocks: &[Scalar]) -> GeopResult<Vec<Scalar>> {
        let Enclosure {
            boxes, failures, ..
        } = self.system().enclose(blocks)?;
        if let Some((_, error)) = failures.into_iter().next() {
            return Err(self.named(error));
        }
        let enclosed: Vec<Scalar> = (boxes.into_iter().zip(blocks))
            .map(|(boxed, &given)| boxed.unwrap_or(given))
            .collect();
        Ok(self.all(&enclosed))
    }

    /// How the sketch stands with the free variables at `blocks`: which
    /// constraints hold, and what can still move — from the enclosure there
    /// of the groups a solve that ended there solved, if given, and of the
    /// others.
    pub fn standing(
        &self,
        blocks: &[Scalar],
        solved: Option<Enclosure<Scalar>>,
    ) -> GeopResult<Standing> {
        let system = self.system();
        let held = system.held(blocks)?;
        let enclosure = match solved {
            Some(solved) => system.enclose_rest(blocks, solved)?,
            None => system.enclose(blocks)?,
        };
        Ok(Standing {
            held,
            dof: enclosure.total,
            free: (self.offsets.iter())
                .map(|o| o.is_some_and(|o| enclosure.freedom[o] > 0))
                .collect(),
            unproven: (enclosure.failures.into_iter())
                .map(|(_, error)| self.named(error).root_message().to_string())
                .collect(),
        })
    }

    /// The constraint `c` with the free variables at `blocks`: its
    /// residuals, and their derivatives by each free variable it reads —
    /// each seeded at a slot of its dual numbers' gradient of its own.
    fn constraint(&self, blocks: &[Scalar], c: usize) -> GeopResult<Linearized> {
        let mut values = Vec::new();
        let mut slots = Vec::new();
        for &v in self.problem.reads(c) {
            values.push(match self.offsets[v] {
                Some(block) => {
                    slots.push(block);
                    Dual::var(blocks[block], slots.len() - 1)
                }
                None => Dual::cst(self.params[v]),
            });
        }
        let mut out = Vec::new();
        self.problem.residuals(c, &values, &mut out)?;
        Ok(Linearized {
            values: out.iter().map(|d| d.v).collect(),
            jacobian: (slots.iter().enumerate())
                .map(|(slot, &block)| (block, out.iter().map(|d| vec![d.d[slot]]).collect()))
                .collect(),
        })
    }
}

/// How a sketch stands (see [`Variables::standing`]).
pub(crate) struct Standing {
    /// Which constraints hold, by index.
    pub held: Held,
    /// The degrees of freedom left: variables minus independent
    /// constraints, which are independent decided over the box an
    /// enclosure of the solution chose them over (see [`System::enclose`]).
    pub dof: usize,
    /// Per variable, whether it can still move.
    pub free: Vec<bool>,
    /// Why no true solution near the blocks is proven, if none is.
    pub unproven: Vec<String>,
}

impl<const N: usize, P: Residuals<N>> Equations for Variables<'_, P, N> {
    type Value = Scalar;

    fn count(&self) -> (usize, usize) {
        (self.problem.constraints(), self.pulls.len())
    }

    fn bound(&self, _: usize) -> Bound {
        Bound::Zero
    }

    fn evaluate(&self, blocks: &[Scalar], which: &[Term]) -> Vec<Option<Linearized>> {
        (which.iter())
            .map(|term| match *term {
                Term::Constraint(c) => self.constraint(blocks, c).ok(),
                // A pulled variable towards its target.
                Term::Preference(p) => {
                    let &(v, target) = self.pulls.get(p)?;
                    Some(match self.offsets[v] {
                        Some(block) => Linearized {
                            values: vec![blocks[block].sub(target)],
                            jacobian: vec![(block, vec![vec![Scalar::ONE]])],
                        },
                        None => Linearized {
                            values: vec![self.params[v].sub(target)],
                            jacobian: Vec::new(),
                        },
                    })
                }
            })
            .collect()
    }
}
