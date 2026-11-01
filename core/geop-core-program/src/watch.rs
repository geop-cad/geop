//! What a long computation has done, for whoever waits for it, and whether
//! they still want it.
//!
//! A front end that can wait for a kernel on a thread of its own (`geop
//! mcp`) sets a watcher around what it asks ([`watching`]). The loops that
//! take long tell it how far they are at each turn: a loop that can stop
//! part way and say so asks [`go_on`]; one whose answer is nothing unless
//! whole, [`checkpoint`], which fails; one that must finish, a build,
//! [`report`]. A watcher decides how much of a question is asked, never
//! what the answer means: a report stopped part way says how far it got.
//! Between two turns nothing is looked at, so a boolean running stops once
//! it is done.

use std::cell::RefCell;

use geop_core_math::geop_error::{GeopError, GeopResult};

/// How far a computation is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Progress<'a> {
    /// What it is doing.
    pub what: &'a str,
    pub done: usize,
    pub total: usize,
}

type Watcher = Box<dyn FnMut(&Progress) -> bool>;

thread_local! {
    static WATCHER: RefCell<Option<Watcher>> = const { RefCell::new(None) };
}

/// Runs `f` with `watcher` told how far it is, and asked whether to go on.
pub fn watching<T>(watcher: impl FnMut(&Progress) -> bool + 'static, f: impl FnOnce() -> T) -> T {
    /// Takes the watcher away again, even from a kernel that panicked.
    struct Unset(Option<Watcher>);
    impl Drop for Unset {
        fn drop(&mut self) {
            WATCHER.with(|w| *w.borrow_mut() = self.0.take());
        }
    }
    let _unset = Unset(WATCHER.with(|w| w.borrow_mut().replace(Box::new(watcher))));
    f()
}

/// Tells the watcher how far a loop is: whether it should go on, as it
/// should with none.
pub fn go_on(what: &str, done: usize, total: usize) -> bool {
    let progress = Progress { what, done, total };
    WATCHER.with(|w| (w.borrow_mut().as_mut()).is_none_or(|watcher| watcher(&progress)))
}

/// Tells the watcher how far a computation that must finish is.
pub fn report(what: &str, done: usize, total: usize) {
    go_on(what, done, total);
}

/// [`go_on`] for an answer that is nothing unless whole: fails, saying how
/// far it got, if it should not go on.
pub fn checkpoint(what: &str, done: usize, total: usize) -> GeopResult<()> {
    match go_on(what, done, total) {
        true => Ok(()),
        false => Err(GeopError::new(format!(
            "stopped at {what}, {done} of {total} done: cancelled, or past its time limit"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use std::{cell::Cell, rc::Rc};

    use super::*;

    #[test]
    fn with_no_watcher_a_loop_goes_on() {
        assert!(go_on("anything", 0, 1));
        assert!(checkpoint("anything", 0, 1).is_ok());
    }

    #[test]
    fn a_watcher_hears_how_far_it_is_and_stops_it() {
        let heard = Rc::new(Cell::new(0));
        let counted = heard.clone();
        let done = watching(
            move |progress: &Progress| {
                counted.set(counted.get() + 1);
                progress.done < 2
            },
            || (0..5).take_while(|&done| go_on("pairs", done, 5)).count(),
        );
        assert_eq!((done, heard.get()), (2, 3));
        let stopped = watching(|_: &Progress| false, || checkpoint("solids", 3, 7));
        let said = stopped.unwrap_err().to_string();
        assert!(said.contains("solids, 3 of 7"), "{said}");
        assert!(go_on("after it", 0, 1), "the watcher is gone afterwards");
    }

    #[test]
    fn a_panic_takes_the_watcher_away() {
        let panicked = std::panic::catch_unwind(|| {
            watching(|_: &Progress| false, || panic!("the kernel crashed"))
        });
        assert!(panicked.is_err());
        assert!(go_on("after it", 0, 1));
    }
}
