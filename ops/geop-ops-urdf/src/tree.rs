//! [`tree`]: a part's mechanism as a URDF robot's tree of links — which
//! placed parts move as one link, and which joint carries each link on the
//! one before it — or why it is none.
//!
//! Parts held together by mates alone — constraints and fastened joints —
//! move as one, so they make one link; so do the parts the mates hold to
//! the ground (the fixed parts, and a pattern's copies), which make the
//! root. That only holds if those mates hold them rigidly: where they leave
//! a part free to move, the motion is one no URDF joint names, and the
//! export is refused. The revolute and slider joints between links must
//! then form a tree hanging from the root: a joint closing a loop, or a
//! link no joint reaches, is refused too, by name.

use std::collections::{BTreeMap, HashMap, HashSet};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    solvers::walk::{Take, flood},
    union_find::UnionFind,
};
use geop_ops_assembly::JointKind;

use crate::Mechanism;

/// The links of a mechanism and the joints between them.
#[derive(Clone, Debug, PartialEq)]
pub struct Tree {
    /// The root first — the ground, and every body held to it — then each
    /// link after the one its joint carries it on.
    pub links: Vec<LinkBodies>,
    /// The joint carrying each link but the root, in the order of
    /// [`Tree::links`].
    pub joints: Vec<TreeJoint>,
}

/// The bodies of one link, by index in the mechanism, and its name: the
/// first of its parts — for the root, the first fixed one, or `base`.
#[derive(Clone, Debug, PartialEq)]
pub struct LinkBodies {
    pub name: String,
    pub bodies: Vec<usize>,
}

/// A joint of the mechanism (by index) carrying the link `child` on the
/// link `parent` (by index in [`Tree::links`]); `forward` if its first end
/// is on the parent.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeJoint {
    pub joint: usize,
    pub parent: usize,
    pub child: usize,
    pub forward: bool,
}

/// The groups of `mechanism`'s bodies that move as one: those held to the
/// ground, and those its constraints and fastened joints hold together —
/// of the ground (`0`) and of every body (`b + 1`), the smallest of its
/// group, so the ground's group is `0`.
fn groups(mechanism: &Mechanism) -> Vec<usize> {
    let count = mechanism.bodies.len();
    let mut groups = UnionFind::new(count + 1);
    for (b, body) in mechanism.bodies.iter().enumerate() {
        if !body.free {
            groups.union(0, b + 1);
        }
    }
    for (_, [a, b]) in &mechanism.constraints {
        groups.union(node(*a), node(*b));
    }
    for joint in &mechanism.joints {
        if matches!(joint.kind, JointKind::Fastened) {
            groups.union(node(joint.a.body), node(joint.b.body));
        }
    }
    (0..=count).map(|node| groups.find(node)).collect()
}

/// The node of an end attached to `body`, or to the ground.
fn node(body: Option<usize>) -> usize {
    body.map_or(0, |b| b + 1)
}

/// The tree of links `mechanism` makes (see the module docs) — not yet
/// checked for links mates hold loosely (see [`check_rigid`]).
pub fn tree(mechanism: &Mechanism) -> GeopResult<Tree> {
    let count = mechanism.bodies.len();
    let groups = groups(mechanism);
    for joint in &mechanism.joints {
        check_joint(&joint.name, &joint.kind)?;
    }

    let part = |b: usize| mechanism.bodies[b].name.as_str();
    let members =
        |group: usize| -> Vec<usize> { (0..count).filter(|&b| groups[b + 1] == group).collect() };
    let link_named = |group: usize| -> LinkBodies {
        let bodies = members(group);
        let fixed = bodies.iter().find(|&&b| !mechanism.bodies[b].free);
        let name = match fixed.or(bodies.first()) {
            Some(&b) => part(b).to_string(),
            None => "base".to_string(),
        };
        LinkBodies { name, bodies }
    };

    let moving: Vec<usize> = (0..mechanism.joints.len())
        .filter(|&j| !matches!(mechanism.joints[j].kind, JointKind::Fastened))
        .collect();
    let ends = |j: usize| {
        let joint = &mechanism.joints[j];
        (groups[node(joint.a.body)], groups[node(joint.b.body)])
    };
    let mut tree = Tree {
        links: vec![link_named(0)],
        joints: Vec::new(),
    };
    let mut link_of_group = HashMap::from([(0, 0)]);
    let mut group_of_link = vec![0];
    // Per link, the joint carrying it, by index into `tree.joints`.
    let mut carried_by: Vec<Option<usize>> = vec![None];
    let mut used = HashSet::new();
    // Out from the root, level by level: each link carries the links its
    // joints not yet used lead to.
    flood([0], Take::Oldest, |&link| {
        let group = group_of_link[link];
        let mut children = Vec::new();
        for &j in &moving {
            let (a, b) = ends(j);
            if used.contains(&j) || (a != group && b != group) {
                continue;
            }
            used.insert(j);
            if a == b {
                return Err(GeopError::new(format!(
                    "the joint {:?} joins {}, which mates already hold rigidly together: a closed loop, which URDF — a tree of links — cannot express; remove the joint or the mates",
                    mechanism.joints[j].name, tree.links[link].name
                )));
            }
            let (other, forward) = if a == group { (b, true) } else { (a, false) };
            if let Some(&reached) = link_of_group.get(&other) {
                return Err(loop_error(mechanism, &tree, &carried_by, link, reached, j));
            }
            let child = tree.links.len();
            link_of_group.insert(other, child);
            group_of_link.push(other);
            tree.links.push(link_named(other));
            carried_by.push(Some(tree.joints.len()));
            tree.joints.push(TreeJoint {
                joint: j,
                parent: link,
                child,
                forward,
            });
            children.push(child);
        }
        Ok(children)
    })?;

    let unreached: Vec<&str> = (0..count)
        .filter(|&b| !link_of_group.contains_key(&groups[b + 1]))
        .map(part)
        .collect();
    if !unreached.is_empty() {
        let root = &tree.links[0];
        let one = unreached.len() == 1;
        let parts = unreached.join(", ");
        return Err(GeopError::new(if root.bodies.is_empty() {
            format!(
                "no part is fixed, so {parts} {} from nothing: a URDF robot is a tree of links from one root, the fixed part — fix the part the robot stands on",
                if one { "hangs" } else { "hang" }
            )
        } else {
            format!(
                "{parts} {} joined to the fixed part {} by neither joints nor mates: a URDF robot is one tree of links from one root — join {} to it, or fix {}",
                if one { "is" } else { "are" },
                root.name,
                if one { "it" } else { "them" },
                if one { "it" } else { "them" },
            )
        }));
    }
    Ok(tree)
}

/// Refuses a joint no URDF joint is: a cylindrical one, or limits URDF
/// cannot write — a revolute joint limited one way only, a slider not
/// limited both ways.
fn check_joint(name: &str, kind: &JointKind) -> GeopResult<()> {
    let problem = match kind {
        JointKind::Cylindrical => Some(
            "is cylindrical, and URDF has no joint that both turns and slides: use a revolute or a slider joint"
                .to_string(),
        ),
        JointKind::Revolute { min, max } if min.is_some() != max.is_some() => Some(format!(
            "is limited one way only ({}), and a URDF revolute joint is limited both ways or not at all (continuous): give it both limits or neither",
            if min.is_some() { "a min" } else { "a max" }
        )),
        JointKind::Slider { min, max } if min.is_none() || max.is_none() => Some(
            "slides without a min and a max, and a URDF prismatic joint needs both: give the joint its travel"
                .to_string(),
        ),
        _ => None,
    };
    match problem {
        Some(problem) => Err(GeopError::new(format!("the joint {name:?} {problem}"))),
        None => Ok(()),
    }
}

/// Refuses parts that mates alone hold together but not rigidly: those of
/// `mechanism` that `freedom` — how free each placed part is, by name,
/// with every joint held at its coordinates — says can still move. Of the
/// links of `tree`, which hold every part.
pub fn check_rigid(
    mechanism: &Mechanism,
    tree: &Tree,
    freedom: &BTreeMap<String, usize>,
) -> GeopResult<()> {
    let loose: Vec<usize> = (0..mechanism.bodies.len())
        .filter(|&b| {
            freedom
                .get(&mechanism.bodies[b].name)
                .is_some_and(|&dof| dof > 0)
        })
        .collect();
    if loose.is_empty() {
        return Ok(());
    }
    // The links of the loose parts, and the mates holding those together.
    // The assembly itself is the root's.
    let link_of = |body: Option<usize>| match body {
        Some(b) => tree.links.iter().position(|l| l.bodies.contains(&b)),
        None => Some(0),
    };
    let links: HashSet<Option<usize>> = loose.iter().map(|&b| link_of(Some(b))).collect();
    let in_links = |ends: [Option<usize>; 2]| ends.iter().any(|&e| links.contains(&link_of(e)));
    let parts: Vec<&str> = (loose.iter())
        .map(|&b| mechanism.bodies[b].name.as_str())
        .collect();
    let mut mates: Vec<&str> = (mechanism.constraints.iter())
        .filter(|(_, ends)| in_links(*ends))
        .map(|(name, _)| name.as_str())
        .collect();
    mates.extend(
        (mechanism.joints.iter())
            .filter(|j| matches!(j.kind, JointKind::Fastened) && in_links([j.a.body, j.b.body]))
            .map(|j| j.name.as_str()),
    );
    Err(GeopError::new(format!(
        "{} {} held by the mates {} alone, which leave {} free to move: a URDF robot moves only at its joints — join the parts by revolute or slider joints, or hold them fully",
        parts.join(", "),
        if parts.len() == 1 { "is" } else { "are" },
        mates.join(", "),
        if parts.len() == 1 { "it" } else { "them" },
    )))
}

/// The error for the joint `closing`, which joins the link `from` to the
/// link `reached`, already in the tree: the loop it closes, by its joints
/// and its links.
fn loop_error(
    mechanism: &Mechanism,
    tree: &Tree,
    carried_by: &[Option<usize>],
    from: usize,
    reached: usize,
    closing: usize,
) -> GeopError {
    // Each link's way back to the root: the links, from it.
    let up = |link: usize| -> Vec<usize> {
        std::iter::successors(Some(link), |&l| {
            carried_by[l].map(|j| tree.joints[j].parent)
        })
        .collect()
    };
    let (a, b) = (up(from), up(reached));
    let common = a.iter().find(|l| b.contains(l)).copied().unwrap_or(0);
    let before = |path: &[usize]| -> Vec<usize> {
        path.iter().copied().take_while(|&l| l != common).collect()
    };
    let (a, b) = (before(&a), before(&b));
    let mut links: Vec<usize> = a.clone();
    links.push(common);
    links.extend(b.iter().rev());
    let mut joints: Vec<&str> = a
        .iter()
        .chain(&b)
        .filter_map(|&l| carried_by[l])
        .map(|j| mechanism.joints[tree.joints[j].joint].name.as_str())
        .collect();
    joints.push(mechanism.joints[closing].name.as_str());
    joints.sort();
    let links: Vec<&str> = links.iter().map(|&l| tree.links[l].name.as_str()).collect();
    GeopError::new(format!(
        "the joints {} close a loop through {}: URDF describes a tree of links, and cannot express a closed kinematic chain — remove one of these joints to export the rest",
        joints.join(", "),
        links.join(", ")
    ))
}
