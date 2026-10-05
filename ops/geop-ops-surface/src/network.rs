//! [`NetworkSurface`]: a face standing on its own through a network of
//! curves — curves in one direction, `u`, crossing curves in the other,
//! `v`, each `u` curve every `v` curve once — interpolating all of them
//! (see [`NurbSurface3D::gordon`]).
//!
//! The curves are edges, of solids, sheets or wires — a 3-D sketch's
//! lines, arcs and splines — or a planar sketch's curves, picked in any
//! order and running either way. They are put in order along the other
//! direction, by where they cross the first curve of it, and turned to run
//! the way the curves they cross are ordered.
//!
//! Where two curves cross is found by the kernel's curve–curve intersection,
//! refined by Newton (see [`refine_curve_curve_crossing`]). Two curves
//! sharing an end — an edge's vertex, a sketch's point — are declared to
//! meet there, and are taken to, without a search. A pair that does not
//! cross, crosses more than once, or runs along each other is refused,
//! naming both curves, and a pair that misses says by how much.
//!
//! The network's outer curves bound the face: each curve is cut to the
//! stretch between its first and last crossing, so curves may run on past
//! the network's border. Closed curves are refused, as are tangency
//! conditions: the face meets the faces along its border as the curves
//! make it.

use geop_core_geometry::{
    intersection::{Intersections, curve_curve_intersect, refine_curve_curve_crossing},
    nurb_curve::NurbCurve3D,
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::{
    Sense,
    build::{BodySpec, BuiltBody, EdgeSpec},
};
use geop_ops::{
    BodyNames, Context, Library, Namer, Part,
    operation::{EntityRef, Operation, Role},
    ui::Form,
};
use serde::{Deserialize, Serialize};

use crate::{MAX_NODES, boundary::patch_face, min_subdivision_size};

/// Spans a face standing on its own through a network of curves, `u_curves`
/// crossing `v_curves`, for the operation `N` — see the module docs. The
/// curves are copied, the originals left as they are.
///
/// The face is named `network(N)`. The copy of each outer curve `X` along
/// its border is `network(N,X)` — for a sketch's curve `X` is `K,c3` (see
/// [`EntityRef::resolve_curve`]) — and the corner where the `u` curve `X`
/// crosses the `v` curve `Y` is `network(N,X,Y)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NetworkSurface;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NetworkSurfaceArgs {
    /// The curves in one direction: two at least, in any order.
    pub u_curves: Vec<EntityRef>,
    /// The curves crossing them: two at least, in any order.
    pub v_curves: Vec<EntityRef>,
}

impl Operation for NetworkSurface {
    type Args = NetworkSurfaceArgs;
    type Session = ();

    /// Nothing picked yet.
    fn new_args<S: Scalar>(&self, _before: &Part<S>) -> NetworkSurfaceArgs {
        NetworkSurfaceArgs::default()
    }

    /// The curves of each direction, picked.
    fn form<'a, S: Scalar>(
        &self,
        _: Context<'a, S>,
        args: &NetworkSurfaceArgs,
        _: &(),
        _: &[String],
    ) -> Form<'a, S, NetworkSurfaceArgs> {
        let mut f = Form::<S, NetworkSurfaceArgs>::new();
        f.reference(
            "u_curves",
            "u curves",
            args.u_curves.clone(),
            &[Role::Curve],
            None,
            true,
            |e, picked| e.args.u_curves = picked,
        );
        f.reference(
            "v_curves",
            "v curves",
            args.v_curves.clone(),
            &[Role::Curve],
            None,
            true,
            |e, picked| e.args.v_curves = picked,
        );
        f
    }

    fn apply<S: Scalar>(
        &self,
        mut part: Part<S>,
        operation_id: &str,
        args: &NetworkSurfaceArgs,
        _library: &dyn Library<S>,
    ) -> GeopResult<Part<S>> {
        let ctx = with_context!("network_surface({operation_id}, {args:?})");
        let namer = Namer::new("network", operation_id)?;
        network_surface(&mut part, &namer, &args.u_curves, &args.v_curves).with_context(ctx)?;
        Ok(part)
    }
}

/// A curve of the network, as picked: its name, where it runs, and the
/// names of its ends.
#[derive(Clone, Debug)]
struct Strand<S: Scalar> {
    /// `u curve X` or `v curve X`, as errors name it.
    label: String,
    name: String,
    curve: NurbCurve3D<S>,
    start: String,
    end: String,
}

impl<S: Scalar> Strand<S> {
    fn of(part: &Part<S>, entity: &EntityRef, direction: &str) -> GeopResult<Self> {
        let named = entity.resolve_curve(part)?;
        let label = format!("{direction} curve {}", named.name);
        if named.start.0 == named.end.0 {
            return Err(GeopError::new(format!(
                "{label} is closed: a network surface spans curves with two ends"
            )));
        }
        Ok(Self {
            label,
            name: named.name,
            curve: named.curve,
            start: named.start.0,
            end: named.end.0,
        })
    }

    /// The same curve run the other way, and where a parameter of it is
    /// then.
    fn turned(&self) -> (Self, impl Fn(S) -> S) {
        let (t0, t1) = self.curve.domain();
        let turned = Self {
            curve: self.curve.reverse(),
            start: self.end.clone(),
            end: self.start.clone(),
            ..self.clone()
        };
        (turned, move |t: S| t0.add(t1).sub(t))
    }
}

/// How far apart `a` and `b` come nearest, as a plain number for an error
/// message: from the nearest of samples along `a`, the nearest points of
/// the two curves found by turns.
fn gap<S: Scalar>(a: &NurbCurve3D<S>, b: &NurbCurve3D<S>) -> GeopResult<f64> {
    let (t0, t1) = a.domain();
    let mut best: Option<(f64, Vector3<S>)> = None;
    let mut samples = (0..32)
        .map(|k| Ok(S::interpolate(t0, t1, S::from_ratio(k, 32)?).sharpen()))
        .collect::<GeopResult<Vec<_>>>()?;
    samples.push(t1);
    for t in samples {
        let p = a.evaluate(t)?;
        let (_, q) = b.closest_point(&p)?;
        let d = p.sub(&q).norm().to_f64();
        if best.as_ref().is_none_or(|(e, _)| d < *e) {
            best = Some((d, q));
        }
    }
    let (mut distance, mut q) = best.expect("samples");
    for _ in 0..8 {
        let (_, p) = a.closest_point(&q)?;
        let (_, next) = b.closest_point(&p)?;
        distance = distance.min(p.sub(&next).norm().to_f64());
        q = next;
    }
    Ok(distance)
}

/// Where `a` crosses `b`: their parameters there. At an end the two share,
/// that end; otherwise the one crossing the intersection search finds,
/// refined — an error naming both if there is none, or more than one.
fn crossing<S: Scalar>(a: &Strand<S>, b: &Strand<S>) -> GeopResult<(S, S)> {
    let ends = |s: &Strand<S>| {
        let (t0, t1) = s.curve.domain();
        [(s.start.clone(), t0), (s.end.clone(), t1)]
    };
    let shared: Vec<(S, S)> = ends(a)
        .into_iter()
        .flat_map(|(na, ta)| {
            ends(b)
                .into_iter()
                .filter(move |(nb, _)| *nb == na)
                .map(move |(_, tb)| (ta, tb))
        })
        .collect();
    let (la, lb) = (&a.label, &b.label);
    match shared.as_slice() {
        [one] => return Ok(*one),
        [] => {}
        _ => {
            return Err(GeopError::new(format!(
                "{la} and {lb} meet at both their ends: each u curve must cross each v curve once"
            )));
        }
    }
    let found = curve_curve_intersect(&a.curve, &b.curve, 3, MAX_NODES, min_subdivision_size())?;
    match found {
        Intersections::Coincident(_) => Err(GeopError::new(format!(
            "{la} and {lb} run along each other: each u curve must cross each v curve once"
        ))),
        Intersections::Found(found) => match found.as_slice() {
            [] => Err(GeopError::new(format!(
                "{la} and {lb} do not cross: they pass {:.3e} apart",
                gap(&a.curve, &b.curve)?
            ))),
            [(ta, tb)] => Ok(refine_curve_curve_crossing(&a.curve, &b.curve, *ta, *tb)),
            more => Err(GeopError::new(format!(
                "{la} and {lb} cross {} times: each u curve must cross each v curve once",
                more.len()
            ))),
        },
    }
}

/// The order `strands` go in along a curve they all cross at `params`: by
/// where they cross it. Which way round is a free choice.
fn order<S: Scalar>(params: &[S]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..params.len()).collect();
    order.sort_by(|&i, &j| params[i].to_f64().total_cmp(&params[j].to_f64()));
    order
}

/// `strands`, each crossing the curves of the other direction at
/// `params[k]` in the order `across`, turned where those run backwards, and
/// the crossings in that order — or an error naming a strand along which
/// they do not run in that order, either way.
fn aligned<S: Scalar>(
    strands: Vec<Strand<S>>,
    params: Vec<Vec<S>>,
    across: &[usize],
    first: &str,
) -> GeopResult<(Vec<Strand<S>>, Vec<Vec<S>>)> {
    let mut out = (Vec::new(), Vec::new());
    for (strand, params) in strands.into_iter().zip(params) {
        let along: Vec<S> = across.iter().map(|&i| params[i]).collect();
        let rising = along.windows(2).all(|w| w[0].definitely_less(w[1]));
        let falling = along.windows(2).all(|w| w[0].definitely_greater(w[1]));
        let (strand, along) = if rising {
            (strand, along)
        } else if falling {
            let (turned, at) = strand.turned();
            (turned, along.into_iter().map(at).collect())
        } else {
            return Err(GeopError::new(format!(
                "the curves crossing {} cross it in another order than they cross {first}: the curves must make a grid",
                strand.label
            )));
        };
        out.0.push(strand);
        out.1.push(along);
    }
    Ok(out)
}

/// Spans a sheet through the network of `u_curves` and `v_curves` (see the
/// module docs), and returns it. What it builds is named after `namer`, as
/// [`NetworkSurface`] says.
pub fn network_surface<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    u_curves: &[EntityRef],
    v_curves: &[EntityRef],
) -> GeopResult<BuiltBody> {
    if u_curves.len() < 2 || v_curves.len() < 2 {
        return Err(GeopError::new(format!(
            "a network surface needs two curves at least in each direction, not {} u curves and {} v curves",
            u_curves.len(),
            v_curves.len()
        )));
    }
    let picked: Vec<&EntityRef> = u_curves.iter().chain(v_curves).collect();
    if let Some((_, twice)) = picked
        .iter()
        .enumerate()
        .find(|(k, e)| picked[..*k].contains(e))
    {
        return Err(GeopError::new(format!(
            "{twice} is picked twice: each curve of a network is in one direction, once"
        )));
    }
    let u = u_curves
        .iter()
        .map(|e| Strand::of(part, e, "u"))
        .collect::<GeopResult<Vec<_>>>()?;
    let v = v_curves
        .iter()
        .map(|e| Strand::of(part, e, "v"))
        .collect::<GeopResult<Vec<_>>>()?;
    let (m, n) = (u.len(), v.len());
    // `on_u[j][i]` and `on_v[i][j]`: where `u[j]` and `v[i]` cross, on each.
    let mut on_u = vec![Vec::with_capacity(n); m];
    let mut on_v = vec![Vec::with_capacity(m); n];
    for (j, uj) in u.iter().enumerate() {
        for (i, vi) in v.iter().enumerate() {
            let (tu, tv) = crossing(uj, vi)?;
            on_u[j].push(tu);
            on_v[i].push(tv);
        }
    }
    // The `v` curves in order along the first `u` curve, the `u` curves
    // along the first `v` curve; every curve turned to run that way.
    let v_order = order(&on_u[0]);
    let u_order = order(&on_v[0]);
    let (u_label, v_label) = (u[0].label.clone(), v[0].label.clone());
    let (u, on_u) = aligned(u, on_u, &v_order, &u_label)?;
    let (v, on_v) = aligned(v, on_v, &u_order, &v_label)?;
    let pick = |items: &[Strand<S>], params: &[Vec<S>], order: &[usize]| {
        order
            .iter()
            .map(|&k| (items[k].clone(), params[k].clone()))
            .unzip::<_, _, Vec<_>, Vec<_>>()
    };
    let (u, on_u): (Vec<Strand<S>>, Vec<Vec<S>>) = pick(&u, &on_u, &u_order);
    let (v, on_v): (Vec<Strand<S>>, Vec<Vec<S>>) = pick(&v, &on_v, &v_order);

    let curves = |s: &[Strand<S>]| s.iter().map(|s| s.curve.clone()).collect::<Vec<_>>();
    let surface = NurbSurface3D::gordon(&curves(&u), &curves(&v), &on_u, &on_v)?;

    // Corners `(u, v)` counter-clockwise from `(0, 0)`, as `(j, i)`: where
    // the `j`-th `u` curve crosses the `i`-th `v` curve.
    let corners = [(0, 0), (0, n - 1), (m - 1, n - 1), (m - 1, 0)];
    let vertices = corners
        .iter()
        .map(|&(j, i)| {
            let a = u[j].curve.evaluate(on_u[j][i])?;
            let b = v[i].curve.evaluate(on_v[i][j])?;
            Ok(a.union(&b))
        })
        .collect::<GeopResult<Vec<_>>>()?;
    let cut = |s: &Strand<S>, params: &[S]| s.curve.sub_curve(params[0], params[params.len() - 1]);
    let edges = vec![
        EdgeSpec {
            curve: cut(&u[0], &on_u[0])?,
            start: 0,
            end: 1,
        },
        EdgeSpec {
            curve: cut(&v[n - 1], &on_v[n - 1])?,
            start: 1,
            end: 2,
        },
        EdgeSpec {
            curve: cut(&u[m - 1], &on_u[m - 1])?,
            start: 3,
            end: 2,
        },
        EdgeSpec {
            curve: cut(&v[0], &on_v[0])?,
            start: 0,
            end: 3,
        },
    ];
    let face = patch_face(
        surface,
        [
            (0, Sense::Forward),
            (1, Sense::Forward),
            (2, Sense::Reversed),
            (3, Sense::Reversed),
        ],
    )?;
    let names = BodyNames {
        vertices: corners
            .iter()
            .map(|&(j, i)| namer.name(&[&u[j].name, &v[i].name]))
            .collect(),
        edges: [&u[0], &v[n - 1], &u[m - 1], &v[0]]
            .iter()
            .map(|s| namer.name(&[&s.name]))
            .collect(),
        faces: vec![namer.root()],
        solid: None,
    };
    let spec = BodySpec {
        vertices,
        edges,
        faces: vec![face],
        shells: vec![vec![0]],
        solid: false,
    };
    part.build_body(spec, names)
}

#[cfg(test)]
mod tests;
