//! [`FaceTree`]: a bounding volume hierarchy over the triangles of a view's
//! faces, so that a pick tests the triangles near the ray, not every one —
//! a few boxes per level, and the triangles of the boxes the ray passes
//! through, for a part of any size. A component's view is shared by every
//! part placed from it, and so is its tree: built once, when it is first
//! picked in.
//!
//! A box is passed by only where the ray definitely misses it, its
//! crossing of each slab enclosed by interval arithmetic: every triangle
//! the ray could meet is tested, as testing every one would.

use std::cell::Cell;

use geop_core_math::{primitives::Ray, scalars::Scalar, vector::Vector3};

use super::ViewFace;

/// How many triangles a leaf of the tree holds at most.
const LEAF: usize = 4;

thread_local! {
    static TRIANGLES_TESTED: Cell<usize> = const { Cell::new(0) };
}

/// How many triangles this thread has tested a pick's ray against ever:
/// what a hover must keep few, however many triangles the parts under the
/// pointer have — so a test bounds it by counting these, not by timing it.
pub fn triangles_tested() -> usize {
    TRIANGLES_TESTED.get()
}

/// A box of the tree, and what is in it: two boxes, or a run of triangles.
struct Node {
    lo: [f64; 3],
    hi: [f64; 3],
    content: Content,
}

enum Content {
    Boxes(usize, usize),
    /// The triangles `start..end` of [`FaceTree::triangles`].
    Triangles(usize, usize),
}

/// The tree of the triangles of some faces: see the [module](self).
pub(crate) struct FaceTree {
    nodes: Vec<Node>,
    /// Each triangle, by its face's index and its own in the face's mesh,
    /// in the order the leaves hold them.
    triangles: Vec<(usize, usize)>,
    /// The meshes it was built of, to tell it from the tree of other faces.
    meshes: Vec<*const super::FaceMesh>,
}

impl std::fmt::Debug for FaceTree {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "FaceTree({} triangles)", self.triangles.len())
    }
}

/// A triangle while the tree is built: where it is, its box, and its
/// middle.
struct Entry {
    at: (usize, usize),
    lo: [f64; 3],
    hi: [f64; 3],
    middle: [f64; 3],
}

impl FaceTree {
    /// The tree of the triangles of `faces`.
    pub(crate) fn of(faces: &[ViewFace]) -> Self {
        let mut entries = Vec::new();
        for (f, face) in faces.iter().enumerate() {
            for (t, corners) in face.mesh.triangles.iter().enumerate() {
                let mut lo = [f64::INFINITY; 3];
                let mut hi = [f64::NEG_INFINITY; 3];
                for p in corners {
                    widen(&mut lo, &mut hi, p.to_array().map(|x| (x.lo, x.hi)));
                }
                let middle = std::array::from_fn(|k| {
                    let (a, b) = (lo.get(k).copied(), hi.get(k).copied());
                    0.5 * (a.unwrap_or_default() + b.unwrap_or_default())
                });
                entries.push(Entry {
                    at: (f, t),
                    lo,
                    hi,
                    middle,
                });
            }
        }
        let mut tree = Self {
            nodes: Vec::new(),
            triangles: Vec::with_capacity(entries.len()),
            meshes: faces.iter().map(|f| std::rc::Rc::as_ptr(&f.mesh)).collect(),
        };
        if !entries.is_empty() {
            tree.build(&mut entries);
        }
        tree
    }

    /// Whether it is the tree of `faces`.
    pub(crate) fn is_of(&self, faces: &[ViewFace]) -> bool {
        self.meshes.len() == faces.len()
            && (self.meshes.iter().zip(faces)).all(|(m, f)| *m == std::rc::Rc::as_ptr(&f.mesh))
    }

    /// Adds the box of `entries`, and those in it, and gives its index.
    fn build(&mut self, entries: &mut [Entry]) -> usize {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for e in entries.iter() {
            widen(
                &mut lo,
                &mut hi,
                std::array::from_fn(|k| {
                    (
                        e.lo.get(k).copied().unwrap_or_default(),
                        e.hi.get(k).copied().unwrap_or_default(),
                    )
                }),
            );
        }
        let at = self.nodes.len();
        self.nodes.push(Node {
            lo,
            hi,
            content: Content::Triangles(0, 0),
        });
        let content = if entries.len() <= LEAF {
            let start = self.triangles.len();
            self.triangles.extend(entries.iter().map(|e| e.at));
            Content::Triangles(start, self.triangles.len())
        } else {
            // Halved at the middle triangle along the axis the middles
            // spread furthest along.
            let spread = |k: usize| {
                let (mut a, mut b) = (f64::INFINITY, f64::NEG_INFINITY);
                for x in entries.iter().filter_map(|e| e.middle.get(k)) {
                    a = a.min(*x);
                    b = b.max(*x);
                }
                b - a
            };
            let axis = (0..3)
                .max_by(|&a, &b| spread(a).total_cmp(&spread(b)))
                .unwrap_or_default();
            let half = entries.len() / 2;
            let along = |e: &Entry| e.middle.get(axis).copied().unwrap_or_default();
            entries.select_nth_unstable_by(half, |a, b| along(a).total_cmp(&along(b)));
            let (left, right) = entries.split_at_mut(half);
            Content::Boxes(self.build(left), self.build(right))
        };
        if let Some(node) = self.nodes.get_mut(at) {
            node.content = content;
        }
        at
    }

    /// Calls `test` with each triangle, by its face's index and its own,
    /// in a box `ray` could pass through — and counts them.
    pub(crate) fn near(&self, ray: &Ray, mut test: impl FnMut(usize, usize)) {
        let mut stack = vec![0];
        while let Some(at) = stack.pop() {
            let Some(node) = self.nodes.get(at) else {
                continue;
            };
            if !could_pass(ray, &node.lo, &node.hi) {
                continue;
            }
            match node.content {
                Content::Boxes(a, b) => stack.extend([a, b]),
                Content::Triangles(start, end) => {
                    for &(face, triangle) in self.triangles.get(start..end).unwrap_or_default() {
                        TRIANGLES_TESTED.set(TRIANGLES_TESTED.get() + 1);
                        test(face, triangle);
                    }
                }
            }
        }
    }
}

/// Whether `ray` could pass through the box from `lo` to `hi`, ahead of
/// its origin: false only where it definitely does not.
fn could_pass(ray: &Ray, lo: &[f64; 3], hi: &[f64; 3]) -> bool {
    let (origin, dir): (&Vector3, &Vector3) = (ray.origin(), ray.dir());
    // The ray is in the box from the latest it could enter a slab to the
    // earliest it could leave one, and never behind its origin.
    let mut enter = 0.0f64;
    let mut leave = f64::INFINITY;
    let slabs = (lo.iter().zip(hi)).zip(origin.to_array().into_iter().zip(dir.to_array()));
    for ((&a, &b), (o, d)) in slabs {
        let (a, b) = (Scalar::from_f64(a), Scalar::from_f64(b));
        if d.could_be_equal(Scalar::ZERO) {
            // Running along the slab: it misses it only from outside.
            if o.definitely_less(a) || o.definitely_greater(b) {
                return false;
            }
            continue;
        }
        let (Ok(ta), Ok(tb)) = (a.sub(o).div(d), b.sub(o).div(d)) else {
            continue;
        };
        enter = enter.max(ta.lo.min(tb.lo));
        leave = leave.min(ta.hi.max(tb.hi));
    }
    enter <= leave
}

/// Widens the box from `lo` to `hi` to take in `bounds`, an interval each.
fn widen(lo: &mut [f64; 3], hi: &mut [f64; 3], bounds: [(f64, f64); 3]) {
    for ((l, h), (a, b)) in lo.iter_mut().zip(hi.iter_mut()).zip(bounds) {
        *l = l.min(a);
        *h = h.max(b);
    }
}
