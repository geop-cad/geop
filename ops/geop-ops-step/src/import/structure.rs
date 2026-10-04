//! Where a file's bodies are: the shape representations holding them, the
//! units and uncertainty each is in, and — for an assembly — where its
//! parts are placed, flattened into one placement per body.
//!
//! A product's shape is a representation of items — solids, sheets,
//! placements — in a context that gives its units. Representations related
//! without a transformation (a product's shape and the B-rep representation
//! holding its solid, say) share their coordinates. An assembly places the
//! shape of each component in its own through a relationship with an
//! `ITEM_DEFINED_TRANSFORMATION`, tied to the `NEXT_ASSEMBLY_USAGE_OCCURRENCE`
//! saying which product is placed in which; a `MAPPED_ITEM` places one
//! representation within another the same way. Every body is found by
//! walking down from the products nothing places, composing placements on
//! the way.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    union_find::UnionFind,
};

use super::{
    geometry::{Frame, Placement, Scope},
    reader::{Args, Reader, unsupported},
};
use crate::part21::{Instance, Value};

/// A body to import: the solid or sheet `#id`, read in `scope`, and what it
/// is called — its own name, or else the product it is the shape of.
#[derive(Clone, Debug)]
pub struct Item {
    pub id: u64,
    pub scope: Scope,
    pub label: String,
}

/// The uncertainty of a file that states none, in millimetres.
const DEFAULT_UNCERTAINTY: f64 = 1e-6;

/// How deeply placements may nest: deeper than any real assembly, and a
/// bound on a file whose placements form a cycle.
const MAX_DEPTH: usize = 64;

/// The kinds of instance that are bodies.
const BODY_TYPES: [&str; 3] = [
    "MANIFOLD_SOLID_BREP",
    "BREP_WITH_VOIDS",
    "SHELL_BASED_SURFACE_MODEL",
];

/// A body type the importer does not read, and why.
const REFUSED_BODY_TYPES: [(&str, &str); 1] = [(
    "FACETED_BREP",
    "a solid of flat facets bounded by polygons rather than edges",
)];

/// The representation record of `instance`: one of the types whose
/// parameters are a name, a list of items and a context.
fn representation(instance: &Instance) -> Option<&crate::part21::Record> {
    let records: Vec<&crate::part21::Record> = match instance {
        Instance::Simple(r) => vec![r],
        Instance::Complex(rs) => rs.iter().collect(),
    };
    records.into_iter().find(|r| {
        r.name.ends_with("REPRESENTATION")
            && r.args.len() == 3
            && matches!(r.args[1], Value::List(_))
            && matches!(r.args[2], Value::Ref(_))
    })
}

impl Reader<'_> {
    /// The scope of a representation's context: its length and angle units
    /// and its uncertainty, placed nowhere yet.
    fn context_scope(&self, context: u64) -> GeopResult<Scope> {
        let instance = self.instance(context)?;
        let mut scope = Scope {
            length: 1.0,
            angle: 1.0,
            uncertainty: f64::NAN,
            place: Placement::IDENTITY,
        };
        if let Some(record) = instance.record("GLOBAL_UNIT_ASSIGNED_CONTEXT") {
            let args = Args {
                id: context,
                record,
            };
            for unit in args.references(0)? {
                let u = self.instance(unit)?;
                if u.is("LENGTH_UNIT") {
                    scope.length = self.length_unit(unit)?;
                } else if u.is("PLANE_ANGLE_UNIT") {
                    scope.angle = self.angle_unit(unit)?;
                }
            }
        }
        if let Some(record) = instance.record("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT") {
            let args = Args {
                id: context,
                record,
            };
            for uncertainty in args.references(0)? {
                let u = self.args(uncertainty, "UNCERTAINTY_MEASURE_WITH_UNIT")?;
                let value = u.real(0)?;
                let unit = u.reference(1)?;
                if self.instance(unit)?.is("LENGTH_UNIT") {
                    let mm = value * self.length_unit(unit)?;
                    if mm > 0.0 && !(scope.uncertainty <= mm) {
                        scope.uncertainty = mm;
                    }
                }
            }
        }
        if !(scope.uncertainty > 0.0) {
            scope.uncertainty = DEFAULT_UNCERTAINTY;
        }
        Ok(scope)
    }

    /// The factor `value` with a unit gives, as `(value, unit)`: a
    /// `*_MEASURE_WITH_UNIT`, simple or complex.
    fn measure(&self, id: u64) -> GeopResult<(f64, u64)> {
        let instance = self.instance(id)?;
        let records: Vec<&crate::part21::Record> = match instance {
            Instance::Simple(r) => vec![r],
            Instance::Complex(rs) => rs.iter().collect(),
        };
        let record = records
            .into_iter()
            .find(|r| r.name.ends_with("MEASURE_WITH_UNIT") && r.args.len() == 2)
            .ok_or_else(|| {
                unsupported(
                    id,
                    instance,
                    "a conversion factor that is not a measure with a unit",
                )
            })?;
        let args = Args { id, record };
        Ok((args.real(0)?, args.reference(1)?))
    }

    /// Millimetres per unit of the length unit `#id`.
    fn length_unit(&self, id: u64) -> GeopResult<f64> {
        let instance = self.instance(id)?;
        if let Some(record) = instance.record("SI_UNIT") {
            let args = Args { id, record };
            if args.enumeration(1)? != "METRE" {
                return Err(unsupported(
                    id,
                    instance,
                    "a length unit that is not a metre",
                ));
            }
            return Ok(1000.0 * prefix(id, instance, &args)?);
        }
        if let Some(record) = instance.record("CONVERSION_BASED_UNIT") {
            let (value, unit) = self.measure(Args { id, record }.reference(1)?)?;
            return Ok(value * self.length_unit(unit)?);
        }
        Err(unsupported(
            id,
            instance,
            "a length unit neither SI nor converted from one",
        ))
    }

    /// Radians per unit of the angle unit `#id`.
    fn angle_unit(&self, id: u64) -> GeopResult<f64> {
        let instance = self.instance(id)?;
        if let Some(record) = instance.record("SI_UNIT") {
            let args = Args { id, record };
            if args.enumeration(1)? != "RADIAN" {
                return Err(unsupported(
                    id,
                    instance,
                    "an angle unit that is not a radian",
                ));
            }
            return prefix(id, instance, &args);
        }
        if let Some(record) = instance.record("CONVERSION_BASED_UNIT") {
            let (value, unit) = self.measure(Args { id, record }.reference(1)?)?;
            return Ok(value * self.angle_unit(unit)?);
        }
        Err(unsupported(
            id,
            instance,
            "an angle unit neither SI nor converted from one",
        ))
    }

    /// The placement `#id` as a frame of the representation read in
    /// `scope`.
    fn placement_frame(&self, scope: &Scope, id: u64) -> GeopResult<Frame> {
        self.frame(scope, id)
    }

    /// Every body of the file, each with where it is placed.
    pub fn bodies(&self) -> GeopResult<Vec<Item>> {
        let exchange = self.exchange;
        // The representations, their items and the scope of each.
        let mut reps: BTreeMap<u64, (Vec<u64>, Scope, String)> = BTreeMap::new();
        for (&id, instance) in &exchange.instances {
            let Some(record) = representation(instance) else {
                continue;
            };
            let args = Args { id, record };
            let items = args.references(1)?;
            let scope = self.context_scope(args.reference(2)?)?;
            reps.insert(id, (items, scope, args.string(0).unwrap_or("").to_string()));
        }
        let rep_ids: Vec<u64> = reps.keys().copied().collect();
        let index: HashMap<u64, usize> =
            rep_ids.iter().enumerate().map(|(i, &id)| (id, i)).collect();

        // Representations related without a transformation share their
        // coordinates: one group.
        let mut groups = UnionFind::new(rep_ids.len());
        // Placements of one group's representations in another's:
        // (parent rep, child rep, parent item, child item).
        let mut placed: Vec<(u64, u64, u64, u64)> = Vec::new();
        for (&id, instance) in &exchange.instances {
            let Some(record) = instance.record("REPRESENTATION_RELATIONSHIP").or_else(|| {
                instance
                    .record("SHAPE_REPRESENTATION_RELATIONSHIP")
                    .filter(|r| r.args.len() >= 4)
            }) else {
                continue;
            };
            let args = Args { id, record };
            let (Ok(rep_1), Ok(rep_2)) = (args.reference(2), args.reference(3)) else {
                continue;
            };
            let (Some(&a), Some(&b)) = (index.get(&rep_1), index.get(&rep_2)) else {
                continue;
            };
            match instance.record("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
                None => groups.union(a, b),
                Some(record) => {
                    let transformation = Args { id, record }.reference(0)?;
                    let t = self.instance(transformation)?;
                    let Some(record) = t.record("ITEM_DEFINED_TRANSFORMATION") else {
                        return Err(unsupported(
                            transformation,
                            t,
                            "a placement of a part not given by two placements",
                        ));
                    };
                    let t_args = Args {
                        id: transformation,
                        record,
                    };
                    // The component is the first representation, the
                    // assembly the second, each placement in its own
                    // (the recommended practice); the products related
                    // decide below where a writer did otherwise.
                    placed.push((rep_2, rep_1, t_args.reference(3)?, t_args.reference(2)?));
                    let _ = id;
                }
            }
        }
        let group_of = |groups: &mut UnionFind, rep: u64| index.get(&rep).map(|&i| groups.find(i));

        // Products and the representations of their shapes.
        let mut shape_of_definition: HashMap<u64, Vec<u64>> = HashMap::new();
        let mut definition_name: HashMap<u64, String> = HashMap::new();
        for id in exchange.all_of("SHAPE_DEFINITION_REPRESENTATION") {
            let args = self.args(id, "SHAPE_DEFINITION_REPRESENTATION")?;
            let (Ok(pds), Ok(rep)) = (args.reference(0), args.reference(1)) else {
                continue;
            };
            let Ok(pds) = self.args(pds, "PRODUCT_DEFINITION_SHAPE") else {
                continue;
            };
            let Ok(definition) = pds.reference(2) else {
                continue;
            };
            shape_of_definition.entry(definition).or_default().push(rep);
            if let Some(name) = self.product_name(definition) {
                definition_name.insert(definition, name);
            }
        }
        let mut children: BTreeSet<u64> = BTreeSet::new();
        // The parent and child product definitions of each occurrence.
        let mut occurrences: HashMap<u64, (u64, u64)> = HashMap::new();
        for id in exchange.all_of("NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
            let args = self.args(id, "NEXT_ASSEMBLY_USAGE_OCCURRENCE")?;
            let (parent, child) = (args.reference(3)?, args.reference(4)?);
            children.insert(child);
            occurrences.insert(id, (parent, child));
        }
        // Turn each placement round where the occurrence it belongs to says
        // its first representation is the assembly's.
        for id in exchange.all_of("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") {
            let args = self.args(id, "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")?;
            let (Ok(relation), Ok(pds)) = (args.reference(0), args.reference(1)) else {
                continue;
            };
            let Ok(pds) = self.args(pds, "PRODUCT_DEFINITION_SHAPE") else {
                continue;
            };
            let Some(&(_, child)) = pds.reference(2).ok().and_then(|o| occurrences.get(&o)) else {
                continue;
            };
            let rel = self.instance(relation)?;
            let Some(record) = rel.record("REPRESENTATION_RELATIONSHIP") else {
                continue;
            };
            let rel_args = Args {
                id: relation,
                record,
            };
            let rep_1 = rel_args.reference(2)?;
            let child_groups: BTreeSet<usize> = shape_of_definition
                .get(&child)
                .into_iter()
                .flatten()
                .filter_map(|&rep| group_of(&mut groups, rep))
                .collect();
            if child_groups.is_empty()
                || group_of(&mut groups, rep_1).is_some_and(|g| child_groups.contains(&g))
            {
                continue;
            }
            // The first representation is the assembly's: swap.
            for p in placed.iter_mut() {
                if p.1 == rep_1 {
                    *p = (p.1, p.0, p.3, p.2);
                }
            }
        }

        // Each group's items, scopes and placements of others.
        let mut members: BTreeMap<usize, Vec<u64>> = BTreeMap::new();
        for &rep in &rep_ids {
            let g = group_of(&mut groups, rep).expect("a representation");
            members.entry(g).or_default().push(rep);
        }
        let mut child_groups: BTreeSet<usize> = BTreeSet::new();
        let mut edges: BTreeMap<usize, Vec<(u64, u64, u64, u64)>> = BTreeMap::new();
        for &(parent, child, parent_item, child_item) in &placed {
            let (Some(pg), Some(cg)) =
                (group_of(&mut groups, parent), group_of(&mut groups, child))
            else {
                continue;
            };
            child_groups.insert(cg);
            edges
                .entry(pg)
                .or_default()
                .push((parent, child, parent_item, child_item));
        }

        // The roots: the shapes of products nothing places, or, in a file
        // without products, every group nothing places.
        let mut roots: Vec<(usize, String)> = Vec::new();
        let mut seen = BTreeSet::new();
        for (&definition, shapes) in &shape_of_definition {
            if children.contains(&definition) {
                continue;
            }
            for &rep in shapes {
                if let Some(g) = group_of(&mut groups, rep)
                    && !child_groups.contains(&g)
                    && seen.insert(g)
                {
                    roots.push((
                        g,
                        definition_name
                            .get(&definition)
                            .cloned()
                            .unwrap_or_default(),
                    ));
                }
            }
        }
        for &g in members.keys() {
            if !child_groups.contains(&g) && seen.insert(g) && shape_of_definition.is_empty() {
                roots.push((g, String::new()));
            }
        }
        roots.sort();

        let mut items = Vec::new();
        for (root, label) in roots {
            self.collect(
                &reps,
                &members,
                &edges,
                &mut groups,
                &index,
                root,
                Placement::IDENTITY,
                &label,
                0,
                &mut items,
            )?;
        }
        Ok(items)
    }

    /// The name of the product the product definition `#definition` is
    /// of.
    fn product_name(&self, definition: u64) -> Option<String> {
        let formation = self
            .args(definition, "PRODUCT_DEFINITION")
            .ok()?
            .reference(2)
            .ok()?;
        let instance = self.instance(formation).ok()?;
        let record = match instance {
            Instance::Simple(r) => r,
            Instance::Complex(rs) => rs.first()?,
        };
        let product = Args {
            id: formation,
            record,
        }
        .reference(2)
        .ok()?;
        let product = self.args(product, "PRODUCT").ok()?;
        let name = product
            .string(1)
            .ok()
            .filter(|s| !s.is_empty())
            .or_else(|| product.string(0).ok())?;
        Some(name.to_string())
    }

    /// The bodies of the group `group` and of everything placed in it, at
    /// `place`.
    #[allow(clippy::too_many_arguments)]
    fn collect(
        &self,
        reps: &BTreeMap<u64, (Vec<u64>, Scope, String)>,
        members: &BTreeMap<usize, Vec<u64>>,
        edges: &BTreeMap<usize, Vec<(u64, u64, u64, u64)>>,
        groups: &mut UnionFind,
        index: &HashMap<u64, usize>,
        group: usize,
        place: Placement,
        label: &str,
        depth: usize,
        out: &mut Vec<Item>,
    ) -> GeopResult<()> {
        if depth > MAX_DEPTH {
            return Err(GeopError::new(format!(
                "the file places parts within parts more than {MAX_DEPTH} deep: its placements form a cycle"
            )));
        }
        for &rep in members.get(&group).into_iter().flatten() {
            let (items, scope, rep_name) = &reps[&rep];
            let label = if label.is_empty() {
                rep_name.as_str()
            } else {
                label
            };
            for &item in items {
                let instance = self.instance(item)?;
                if let Some(name) = BODY_TYPES.iter().find(|t| instance.is(t)) {
                    let own = self
                        .args(item, name)
                        .ok()
                        .and_then(|a| a.string(0).ok())
                        .unwrap_or("");
                    out.push(Item {
                        id: item,
                        scope: Scope { place, ..*scope },
                        label: if own.is_empty() {
                            label.to_string()
                        } else {
                            own.to_string()
                        },
                    });
                } else if let Some((name, why)) =
                    REFUSED_BODY_TYPES.iter().find(|(t, _)| instance.is(t))
                {
                    let _ = name;
                    return Err(unsupported(item, instance, why));
                } else if instance.is("MAPPED_ITEM") {
                    let args = self.args(item, "MAPPED_ITEM")?;
                    let map = self.args(args.reference(1)?, "REPRESENTATION_MAP")?;
                    let mapped = map.reference(1)?;
                    let Some((_, child_scope, _)) = reps.get(&mapped) else {
                        continue;
                    };
                    let origin = self.placement_frame(child_scope, map.reference(0)?)?;
                    let target = self.placement_frame(scope, args.reference(2)?)?;
                    let motion = Placement::of(&origin)
                        .inverse()
                        .then(&Placement::of(&target))
                        .then(&place);
                    let Some(&i) = index.get(&mapped) else {
                        continue;
                    };
                    let g = groups.find(i);
                    self.collect(
                        reps,
                        members,
                        edges,
                        groups,
                        index,
                        g,
                        motion,
                        label,
                        depth + 1,
                        out,
                    )?;
                }
            }
        }
        for &(parent, child, parent_item, child_item) in edges.get(&group).into_iter().flatten() {
            let parent_scope = reps[&parent].1;
            let child_scope = reps[&child].1;
            let target = self.placement_frame(&parent_scope, parent_item)?;
            let origin = self.placement_frame(&child_scope, child_item)?;
            let motion = Placement::of(&origin)
                .inverse()
                .then(&Placement::of(&target))
                .then(&place);
            let g = groups.find(index[&child]);
            let child_label = reps[&child].2.clone();
            self.collect(
                reps,
                members,
                edges,
                groups,
                index,
                g,
                motion,
                &child_label,
                depth + 1,
                out,
            )?;
        }
        Ok(())
    }
}

/// The factor of an SI unit's prefix.
fn prefix(id: u64, instance: &Instance, args: &Args) -> GeopResult<f64> {
    if args.is_null(0) {
        return Ok(1.0);
    }
    Ok(match args.enumeration(0)? {
        "EXA" => 1e18,
        "PETA" => 1e15,
        "TERA" => 1e12,
        "GIGA" => 1e9,
        "MEGA" => 1e6,
        "KILO" => 1e3,
        "HECTO" => 1e2,
        "DECA" => 1e1,
        "DECI" => 1e-1,
        "CENTI" => 1e-2,
        "MILLI" => 1e-3,
        "MICRO" => 1e-6,
        "NANO" => 1e-9,
        "PICO" => 1e-12,
        "FEMTO" => 1e-15,
        "ATTO" => 1e-18,
        other => {
            return Err(unsupported(
                id,
                instance,
                &format!("the unit prefix {other}"),
            ));
        }
    })
}
