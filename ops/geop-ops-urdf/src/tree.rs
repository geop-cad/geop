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

use std::collections::{HashMap, HashSet, VecDeque};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_solve::mates::{Assembly, JointKind};
use geop_ops::assembly::Mechanism;

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

/// Which bodies move as one: a union-find over the ground (`0`) and every
/// body (`b + 1`).
struct Groups(Vec<usize>);

impl Groups {
    fn find(&self, mut node: usize) -> usize {
        while self.0[node] != node {
            node = self.0[node];
        }
        node
    }

    fn union(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        // The smaller node is the root: the ground roots its own group.
        self.0[a.max(b)] = a.min(b);
    }
}

/// The node of an end attached to `body`, or to the ground.
fn node(body: Option<usize>) -> usize {
    body.map_or(0, |b| b + 1)
}

/// The tree of links `mechanism` makes (see the module docs).
pub fn tree<S: Scalar>(mechanism: &Mechanism<'_, S>) -> GeopResult<Tree> {
    let assembly = &mechanism.assembly;
    let count = assembly.bodies.len();
    let mut groups = Groups((0..=count).collect());
    for (b, body) in assembly.bodies.iter().enumerate() {
        if !body.free {
            groups.union(0, b + 1);
        }
    }
    for constraint in &assembly.constraints {
        groups.union(node(constraint.a.body), node(constraint.b.body));
    }
    for joint in &assembly.joints {
        if matches!(joint.kind, JointKind::Fastened) {
            groups.union(node(joint.a.body), node(joint.b.body));
        }
    }
    check_rigid(mechanism, &groups)?;
    for (j, joint) in assembly.joints.iter().enumerate() {
        check_joint(mechanism, j, &joint.kind)?;
    }

    let part = |b: usize| mechanism.bodies[b].name.as_str();
    let members = |group: usize| -> Vec<usize> {
        (0..count)
            .filter(|&b| groups.find(b + 1) == group)
            .collect()
    };
    let link_named = |group: usize| -> LinkBodies {
        let bodies = members(group);
        let fixed = bodies.iter().find(|&&b| !assembly.bodies[b].free);
        let name = match fixed.or(bodies.first()) {
            Some(&b) => part(b).to_string(),
            None => "base".to_string(),
        };
        LinkBodies { name, bodies }
    };

    let moving: Vec<usize> = (0..assembly.joints.len())
        .filter(|&j| !matches!(assembly.joints[j].kind, JointKind::Fastened))
        .collect();
    let ends = |j: usize| {
        let joint = &assembly.joints[j];
        (
            groups.find(node(joint.a.body)),
            groups.find(node(joint.b.body)),
        )
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
    let mut queue = VecDeque::from([0]);
    while let Some(link) = queue.pop_front() {
        let group = group_of_link[link];
        for &j in &moving {
            let (a, b) = ends(j);
            if used.contains(&j) || (a != group && b != group) {
                continue;
            }
            used.insert(j);
            if a == b {
                return Err(GeopError::new(format!(
                    "the joint {:?} joins {}, which mates already hold rigidly together: a closed loop, which URDF — a tree of links — cannot express; remove the joint or the mates",
                    mechanism.joint_name(j),
                    tree.links[link].name
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
            queue.push_back(child);
        }
    }

    let unreached: Vec<&str> = (0..count)
        .filter(|&b| !link_of_group.contains_key(&groups.find(b + 1)))
        .map(part)
        .collect();
    if !unreached.is_empty() {
        let root = &tree.links[0];
        return Err(GeopError::new(if root.bodies.is_empty() {
            format!(
                "no part is fixed, so {} hang from nothing: a URDF robot is a tree of links from one root, the fixed part — fix the part the robot stands on",
                unreached.join(", ")
            )
        } else {
            format!(
                "{} are joined to the fixed part {} by neither joints nor mates: a URDF robot is one tree of links from one root — join them to it, or fix them",
                unreached.join(", "),
                root.name
            )
        }));
    }
    Ok(tree)
}

/// Refuses a joint no URDF joint is: a cylindrical one, or limits URDF
/// cannot write — a revolute joint limited one way only, a slider not
/// limited both ways.
fn check_joint<S: Scalar>(
    mechanism: &Mechanism<'_, S>,
    j: usize,
    kind: &JointKind<S>,
) -> GeopResult<()> {
    let name = mechanism.joint_name(j);
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

/// Refuses parts that mates alone hold together but not rigidly: solved
/// with those mates only — every link's first body held where it is, the
/// ground's group held by the ground — any body that can still move.
fn check_rigid<S: Scalar>(mechanism: &Mechanism<'_, S>, groups: &Groups) -> GeopResult<()> {
    let assembly = &mechanism.assembly;
    let fastened: Vec<usize> = (0..assembly.joints.len())
        .filter(|&j| matches!(assembly.joints[j].kind, JointKind::Fastened))
        .collect();
    if assembly.constraints.is_empty() && fastened.is_empty() {
        return Ok(());
    }
    let mut bodies = assembly.bodies.clone();
    let mut anchored = HashSet::from([0]);
    for (b, body) in bodies.iter_mut().enumerate() {
        if anchored.insert(groups.find(b + 1)) {
            body.free = false;
        }
    }
    let mut rigid = Assembly::new(bodies, assembly.constraints.clone(), assembly.scale);
    rigid.joints = fastened.iter().map(|&j| assembly.joints[j]).collect();
    let freedom = rigid.freedom()?;
    let Some(loose) = (0..assembly.bodies.len()).find(|&b| freedom.bodies[b] > 0) else {
        return Ok(());
    };
    let group = groups.find(loose + 1);
    let in_group = |body: Option<usize>| groups.find(node(body)) == group;
    let parts: Vec<&str> = (0..assembly.bodies.len())
        .filter(|&b| in_group(Some(b)) && freedom.bodies[b] > 0)
        .map(|b| mechanism.bodies[b].name.as_str())
        .collect();
    let mut mates: Vec<&str> = (0..assembly.constraints.len())
        .filter(|&c| in_group(assembly.constraints[c].a.body))
        .map(|c| mechanism.constraint_name(c))
        .collect();
    mates.extend(
        fastened
            .iter()
            .filter(|&&j| in_group(assembly.joints[j].a.body))
            .map(|&j| mechanism.joint_name(j)),
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
fn loop_error<S: Scalar>(
    mechanism: &Mechanism<'_, S>,
    tree: &Tree,
    carried_by: &[Option<usize>],
    from: usize,
    reached: usize,
    closing: usize,
) -> GeopError {
    // Each link's way back to the root: the links, from it.
    let up = |mut link: usize| {
        let mut path = vec![link];
        while let Some(j) = carried_by[link] {
            link = tree.joints[j].parent;
            path.push(link);
        }
        path
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
        .map(|j| mechanism.joint_name(tree.joints[j].joint))
        .collect();
    joints.push(mechanism.joint_name(closing));
    joints.sort();
    let links: Vec<&str> = links.iter().map(|&l| tree.links[l].name.as_str()).collect();
    GeopError::new(format!(
        "the joints {} close a loop through {}: URDF describes a tree of links, and cannot express a closed kinematic chain — remove one of these joints to export the rest",
        joints.join(", "),
        links.join(", ")
    ))
}
