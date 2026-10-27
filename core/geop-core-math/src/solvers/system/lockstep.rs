//! Groups worked on in lockstep. Each group's work — a solve, an
//! enclosure — is written as if it were alone, and asks its terms of a
//! [`Batch`] where it would make them. Every group waiting is answered by
//! one [`Equations::evaluate`], each at its own values: no term ties two
//! groups together, so one evaluation with every group where it is gives
//! each what it asked for. An adapter whose evaluation costs as much for
//! one group as for all — a program run again at every state tried — then
//! pays per step of the slowest group, not per step of every group.
//!
//! A group that is done keeps its values; one not asking stays where the
//! batch's base has it.
//!
//! A term that cannot be made where every group is is asked again of its
//! group alone: an adapter may fail a whole evaluation for one term it
//! cannot make (a program's constraints, gathered all at once), and a group
//! must not see another's failure as its own.

use std::{
    cell::{Cell, RefCell},
    future::Future,
    pin::Pin,
    rc::Rc,
    task::{Context, Poll, Waker},
};

use crate::geop_error::{GeopError, GeopResult};

use super::{Equations, Linearized, System, Term};

/// What an evaluation answers: per term asked, its residuals and
/// derivatives, or `None` (see [`Equations::evaluate`]).
pub(super) type Made = Vec<Option<Linearized>>;

/// The terms one group asks, with its blocks at its values, and where the
/// answer goes.
struct Request<E: Equations> {
    blocks: Vec<usize>,
    values: Vec<E::Value>,
    which: Vec<Term>,
    answer: Rc<RefCell<Option<Made>>>,
}

/// The terms the groups worked on together ask, answered all at once, and
/// how many evaluations that took.
pub(super) struct Batch<'s, 'e, E: Equations> {
    system: &'s System<'e, E>,
    /// Every block's value where no group waiting says otherwise.
    base: &'s [E::Value],
    asked: RefCell<Vec<Request<E>>>,
    evaluations: Cell<usize>,
}

impl<'s, 'e, E: Equations> Batch<'s, 'e, E> {
    pub(super) fn new(system: &'s System<'e, E>, base: &'s [E::Value]) -> Self {
        Self {
            system,
            base,
            asked: Default::default(),
            evaluations: Default::default(),
        }
    }

    /// How many times the equations were made: the work done.
    pub(super) fn evaluations(&self) -> usize {
        self.evaluations.get()
    }

    /// The terms `which` with the blocks `blocks` — a group's — at `values`
    /// and every other where the base has it, once the batch is answered.
    pub(super) async fn ask(&self, blocks: &[usize], values: &[E::Value], which: &[Term]) -> Made {
        // Nothing asked is answered without making anything: a group of no
        // limits asked which of them it is beyond.
        if which.is_empty() {
            return Made::new();
        }
        let answer = Rc::new(RefCell::new(None));
        self.asked.borrow_mut().push(Request {
            blocks: blocks.to_vec(),
            values: values.to_vec(),
            which: which.to_vec(),
            answer: answer.clone(),
        });
        std::future::poll_fn(|_| match answer.borrow_mut().take() {
            Some(made) => Poll::Ready(made),
            None => Poll::Pending,
        })
        .await
    }

    /// Runs `work` — per group, what is done with it, asking its terms of
    /// this batch — in lockstep, until each is done: their results, in
    /// order. The first that fails fails them all.
    pub(super) fn run<T, F: Future<Output = GeopResult<T>>>(
        &self,
        work: impl IntoIterator<Item = F>,
    ) -> GeopResult<Vec<T>> {
        let mut work: Vec<Option<Pin<Box<F>>>> =
            work.into_iter().map(|w| Some(Box::pin(w))).collect();
        let mut done: Vec<Option<T>> = work.iter().map(|_| None).collect();
        let mut context = Context::from_waker(Waker::noop());
        loop {
            for (slot, result) in work.iter_mut().zip(&mut done) {
                let Some(going) = slot else {
                    continue;
                };
                if let Poll::Ready(ended) = going.as_mut().poll(&mut context) {
                    *result = Some(ended?);
                    *slot = None;
                }
            }
            let asked = self.asked.take();
            if asked.is_empty() {
                break;
            }
            self.answer(asked);
        }
        (done.into_iter())
            .map(|result| {
                result.ok_or_else(|| {
                    GeopError::new("a group's work is pending without asking anything")
                })
            })
            .collect()
    }

    /// Answers every request of `asked` by one evaluation, each group at
    /// its values — and a request one of whose terms could not be made
    /// there again alone (see the module docs).
    fn answer(&self, asked: Vec<Request<E>>) {
        let mut values = self.base.to_vec();
        let mut terms = Vec::new();
        for request in &asked {
            for (&block, value) in request.blocks.iter().zip(&request.values) {
                values[block] = value.clone();
            }
            terms.extend(request.which.iter().copied());
        }
        terms.sort_unstable();
        terms.dedup();
        let made = self.evaluate(&values, &terms);
        let together = asked.len() > 1;
        for request in asked {
            let mut answer: Made = (request.which.iter())
                .map(|term| {
                    let k = terms.binary_search(term).ok()?;
                    made.get(k)?.clone()
                })
                .collect();
            if together && answer.iter().any(Option::is_none) {
                let mut alone = self.base.to_vec();
                for (&block, value) in request.blocks.iter().zip(&request.values) {
                    alone[block] = value.clone();
                }
                answer = self.evaluate(&alone, &request.which);
            }
            *request.answer.borrow_mut() = Some(answer);
        }
    }

    /// The terms `which` at `values`, counted.
    fn evaluate(&self, values: &[E::Value], which: &[Term]) -> Made {
        self.evaluations.set(self.evaluations.get() + 1);
        self.system.equations.evaluate(values, which)
    }
}
