//! Writing a part as a STEP file (AP214, the geometry every CAD system
//! reads): each solid a `MANIFOLD_SOLID_BREP` — a `BREP_WITH_VOIDS` if it
//! has voids — and each sheet of faces standing on their own a
//! `SHELL_BASED_SURFACE_MODEL`, with every surface and curve written exactly
//! as the kernel holds it: a `B_SPLINE_*_WITH_KNOTS`, rational where a
//! weight is not one. Lengths are millimetres: a length of one in the
//! kernel is one millimetre.
//!
//! The parts placed in the part are written flattened: their bodies moved
//! to where they are placed, as bodies of the one product, not as an
//! assembly of products.
//!
//! Every value is written as its interval's midpoint. Entities are named
//! after the part's names for them, so a face picked as `extrude(E,end)`
//! here is called that in the file.

use std::collections::HashMap;

use geop_core_geometry::{nurb_curve::NurbCurve3D, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Motion, Pose},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{
    CoedgeGeometry, EdgeId, FaceId, Model, Sense, ShellId, VertexId, boundary::BoundaryType,
};
use geop_ops::{Part, RefId, operation::INSTANCE_SEPARATOR};

use crate::part21::{Exchange, Instance, Record, Value};

/// The uncertainty the file declares for its lengths, in millimetres: how
/// far apart two points may be and still be read as one. Far looser than
/// anything the kernel holds, which is what a reader needs to rebuild the
/// topology — far tighter than any feature.
const UNCERTAINTY: f64 = 1e-7;

/// The part `part` — its bodies and those of the parts placed in it — as
/// the text of a STEP file whose product is called `name`.
pub fn write_step<S: Scalar>(part: &Part<S>, name: &str) -> GeopResult<String> {
    let mut writer = Writer::default();
    let context = writer.context();
    let mut solids = Vec::new();
    let mut sheets = Vec::new();
    writer.part(part, &Pose::identity(), "", &mut solids, &mut sheets)?;

    let origin = writer.placement([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let product = writer.product(name, context.application);
    let mut items = vec![Value::Ref(origin)];
    items.extend(solids.iter().map(|&id| Value::Ref(id)));
    let shape = writer.add(
        "ADVANCED_BREP_SHAPE_REPRESENTATION",
        vec![
            string(name),
            Value::List(items),
            Value::Ref(context.representation),
        ],
    );
    writer.add(
        "SHAPE_DEFINITION_REPRESENTATION",
        vec![Value::Ref(product), Value::Ref(shape)],
    );
    if !sheets.is_empty() {
        let mut items = vec![Value::Ref(origin)];
        items.extend(sheets.iter().map(|&id| Value::Ref(id)));
        let surfaces = writer.add(
            "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
            vec![
                string(name),
                Value::List(items),
                Value::Ref(context.representation),
            ],
        );
        writer.add(
            "SHAPE_REPRESENTATION_RELATIONSHIP",
            vec![
                string(""),
                string(""),
                Value::Ref(shape),
                Value::Ref(surfaces),
            ],
        );
    }

    let file_name = format!("{name}.step");
    Ok(Exchange {
        header: vec![
            Record {
                name: "FILE_DESCRIPTION".into(),
                args: vec![Value::List(vec![string(name)]), string("2;1")],
            },
            Record {
                name: "FILE_NAME".into(),
                args: vec![
                    string(&file_name),
                    string(""),
                    Value::List(vec![string("")]),
                    Value::List(vec![string("")]),
                    string("geop"),
                    string("geop"),
                    string(""),
                ],
            },
            Record {
                name: "FILE_SCHEMA".into(),
                args: vec![Value::List(vec![string(
                    "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }",
                )])],
            },
        ],
        instances: writer.instances,
    }
    .write())
}

fn string(s: &str) -> Value {
    Value::String(s.to_string())
}

fn logical(b: bool) -> Value {
    Value::Enum(if b { "T" } else { "F" }.into())
}

fn reals(xs: impl IntoIterator<Item = f64>) -> Value {
    Value::List(xs.into_iter().map(Value::Real).collect())
}

fn refs(ids: impl IntoIterator<Item = u64>) -> Value {
    Value::List(ids.into_iter().map(Value::Ref).collect())
}

/// The distinct knots of `knots` and how often each is repeated, as STEP
/// writes a knot vector. Knots are repeated when the kernel built them as
/// the same value, so they are grouped by equal midpoints.
fn knots_and_multiplicities<S: Scalar>(knots: &[S]) -> (Value, Value) {
    let mut distinct: Vec<f64> = Vec::new();
    let mut counts: Vec<i64> = Vec::new();
    for k in knots {
        let k = k.to_f64();
        if distinct.last() == Some(&k) {
            *counts.last_mut().expect("a knot was pushed with it") += 1;
        } else {
            distinct.push(k);
            counts.push(1);
        }
    }
    (
        Value::List(counts.into_iter().map(Value::Integer).collect()),
        reals(distinct),
    )
}

/// The shared entities every representation refers to.
struct Context {
    application: u64,
    representation: u64,
}

#[derive(Default)]
struct Writer {
    instances: std::collections::BTreeMap<u64, Instance>,
}

impl Writer {
    fn insert(&mut self, instance: Instance) -> u64 {
        let id = self.instances.len() as u64 + 1;
        self.instances.insert(id, instance);
        id
    }

    fn add(&mut self, name: &str, args: Vec<Value>) -> u64 {
        self.insert(Instance::Simple(Record {
            name: name.into(),
            args,
        }))
    }

    /// A complex instance of the records `records`, written in alphabetical
    /// order as Part 21 requires.
    fn add_complex(&mut self, mut records: Vec<Record>) -> u64 {
        records.sort_by(|a, b| a.name.cmp(&b.name));
        self.insert(Instance::Complex(records))
    }

    fn point(&mut self, p: [f64; 3]) -> u64 {
        self.add("CARTESIAN_POINT", vec![string(""), reals(p)])
    }

    fn placement(&mut self, origin: [f64; 3], axis: [f64; 3], reference: [f64; 3]) -> u64 {
        let origin = self.point(origin);
        let axis = self.add("DIRECTION", vec![string(""), reals(axis)]);
        let reference = self.add("DIRECTION", vec![string(""), reals(reference)]);
        self.add(
            "AXIS2_PLACEMENT_3D",
            vec![
                string(""),
                Value::Ref(origin),
                Value::Ref(axis),
                Value::Ref(reference),
            ],
        )
    }

    /// The application context, the units — millimetres, radians — and the
    /// geometric context with the file's uncertainty.
    fn context(&mut self) -> Context {
        let application = self.add(
            "APPLICATION_CONTEXT",
            vec![string(
                "core data for automotive mechanical design processes",
            )],
        );
        self.add(
            "APPLICATION_PROTOCOL_DEFINITION",
            vec![
                string("international standard"),
                string("automotive_design"),
                Value::Integer(2000),
                Value::Ref(application),
            ],
        );
        let record = |name: &str, args: Vec<Value>| Record {
            name: name.into(),
            args,
        };
        let length_unit = self.add_complex(vec![
            record("LENGTH_UNIT", vec![]),
            record("NAMED_UNIT", vec![Value::Derived]),
            record(
                "SI_UNIT",
                vec![Value::Enum("MILLI".into()), Value::Enum("METRE".into())],
            ),
        ]);
        let angle_unit = self.add_complex(vec![
            record("NAMED_UNIT", vec![Value::Derived]),
            record("PLANE_ANGLE_UNIT", vec![]),
            record("SI_UNIT", vec![Value::Null, Value::Enum("RADIAN".into())]),
        ]);
        let solid_angle_unit = self.add_complex(vec![
            record("NAMED_UNIT", vec![Value::Derived]),
            record(
                "SI_UNIT",
                vec![Value::Null, Value::Enum("STERADIAN".into())],
            ),
            record("SOLID_ANGLE_UNIT", vec![]),
        ]);
        let uncertainty = self.add(
            "UNCERTAINTY_MEASURE_WITH_UNIT",
            vec![
                Value::Typed("LENGTH_MEASURE".into(), Box::new(Value::Real(UNCERTAINTY))),
                Value::Ref(length_unit),
                string("distance_accuracy_value"),
                string("confusion accuracy"),
            ],
        );
        let representation = self.add_complex(vec![
            record("GEOMETRIC_REPRESENTATION_CONTEXT", vec![Value::Integer(3)]),
            record(
                "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT",
                vec![refs([uncertainty])],
            ),
            record(
                "GLOBAL_UNIT_ASSIGNED_CONTEXT",
                vec![refs([length_unit, angle_unit, solid_angle_unit])],
            ),
            record("REPRESENTATION_CONTEXT", vec![string("geop"), string("3D")]),
        ]);
        Context {
            application,
            representation,
        }
    }

    /// The product `name` and its definition, down to the
    /// `PRODUCT_DEFINITION_SHAPE` its shape representation is attached to.
    fn product(&mut self, name: &str, application: u64) -> u64 {
        let context = self.add(
            "PRODUCT_CONTEXT",
            vec![string(""), Value::Ref(application), string("mechanical")],
        );
        let product = self.add(
            "PRODUCT",
            vec![string(name), string(name), string(""), refs([context])],
        );
        self.add(
            "PRODUCT_RELATED_PRODUCT_CATEGORY",
            vec![string("part"), Value::Null, refs([product])],
        );
        let formation = self.add(
            "PRODUCT_DEFINITION_FORMATION",
            vec![string(""), string(""), Value::Ref(product)],
        );
        let definition_context = self.add(
            "PRODUCT_DEFINITION_CONTEXT",
            vec![
                string("part definition"),
                Value::Ref(application),
                string("design"),
            ],
        );
        let definition = self.add(
            "PRODUCT_DEFINITION",
            vec![
                string("design"),
                string(""),
                Value::Ref(formation),
                Value::Ref(definition_context),
            ],
        );
        self.add(
            "PRODUCT_DEFINITION_SHAPE",
            vec![string(""), string(""), Value::Ref(definition)],
        )
    }

    /// Every body of `part`, moved by `pose`, and those of the parts placed
    /// in it: solids onto `solids`, sheets onto `sheets`. Entities are named
    /// `prefix` and then their names in `part`.
    fn part<S: Scalar>(
        &mut self,
        part: &Part<S>,
        pose: &Pose<S>,
        prefix: &str,
        solids: &mut Vec<u64>,
        sheets: &mut Vec<u64>,
    ) -> GeopResult<()> {
        let model = part.topology();
        let mut body = BodyWriter {
            writer: self,
            model,
            motion: pose.motion(),
            name: |id: RefId| format!("{prefix}{}", part.name_of(id).unwrap_or_default()),
            vertices: HashMap::new(),
            edges: HashMap::new(),
        };
        let mut solid_ids: Vec<_> = model.solids.keys().copied().collect();
        solid_ids.sort_by_key(|id| id.0);
        for solid_id in solid_ids {
            let solid = model.get_solid(solid_id)?;
            let name = (body.name)(solid_id.into());
            let [outer, voids @ ..] = solid.shells.as_slice() else {
                return Err(GeopError::new(format!(
                    "write_step: solid {name} has no shell"
                )));
            };
            let outer = body.shell(*outer, "CLOSED_SHELL", false)?;
            let id = if voids.is_empty() {
                body.writer.add(
                    "MANIFOLD_SOLID_BREP",
                    vec![string(&name), Value::Ref(outer)],
                )
            } else {
                // A void's faces point into it, out of the material: the
                // closed shell around the void is them turned around, and
                // the void is that shell, oriented back.
                let mut oriented = Vec::new();
                for &void in voids {
                    let shell = body.shell(void, "CLOSED_SHELL", true)?;
                    oriented.push(body.writer.add(
                        "ORIENTED_CLOSED_SHELL",
                        vec![
                            string(""),
                            Value::Derived,
                            Value::Ref(shell),
                            logical(false),
                        ],
                    ));
                }
                body.writer.add(
                    "BREP_WITH_VOIDS",
                    vec![string(&name), Value::Ref(outer), refs(oriented)],
                )
            };
            solids.push(id);
        }
        let mut sheet_ids: Vec<ShellId> = model
            .shells
            .iter()
            .filter(|(_, shell)| shell.solid.is_none())
            .map(|(&id, _)| id)
            .collect();
        sheet_ids.sort_by_key(|id| id.0);
        for sheet in sheet_ids {
            let shell = body.shell(sheet, "OPEN_SHELL", false)?;
            sheets.push(
                body.writer
                    .add("SHELL_BASED_SURFACE_MODEL", vec![string(""), refs([shell])]),
            );
        }
        for (id, instance) in part.instances() {
            let prefix = format!(
                "{prefix}{}{INSTANCE_SEPARATOR}",
                part.name_of(id).unwrap_or_default()
            );
            self.part(
                instance.part(),
                &pose.compose(&instance.pose),
                &prefix,
                solids,
                sheets,
            )?;
        }
        Ok(())
    }
}

/// Writes the bodies of one model, sharing each vertex and edge between
/// the faces that use it.
struct BodyWriter<'w, 'm, S: Scalar, N> {
    writer: &'w mut Writer,
    model: &'m Model<S>,
    motion: Motion<S>,
    name: N,
    vertices: HashMap<VertexId, u64>,
    edges: HashMap<EdgeId, u64>,
}

impl<S: Scalar, N: Fn(RefId) -> String> BodyWriter<'_, '_, S, N> {
    fn moved(&self, p: &Vector3<S>) -> [f64; 3] {
        let p = self.motion.apply(p);
        [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]
    }

    /// The homogeneous control point `cp` as STEP writes it: the point,
    /// moved, and its weight.
    fn control_point(&mut self, cp: &geop_core_math::vector::Vector4<S>) -> GeopResult<(u64, f64)> {
        let w = cp[3];
        let p = Vector3::from_array([cp[0].div(w)?, cp[1].div(w)?, cp[2].div(w)?]);
        let p = self.moved(&p);
        Ok((self.writer.point(p), w.to_f64()))
    }

    fn vertex(&mut self, id: VertexId) -> GeopResult<u64> {
        if let Some(&step) = self.vertices.get(&id) {
            return Ok(step);
        }
        let p = self.moved(&self.model.get_vertex(id)?.point);
        let point = self.writer.point(p);
        let step = self.writer.add(
            "VERTEX_POINT",
            vec![string(&(self.name)(id.into())), Value::Ref(point)],
        );
        self.vertices.insert(id, step);
        Ok(step)
    }

    fn curve(&mut self, curve: &NurbCurve3D<S>) -> GeopResult<u64> {
        let mut points = Vec::new();
        let mut weights = Vec::new();
        for cp in &curve.control_points {
            let (point, weight) = self.control_point(cp)?;
            points.push(point);
            weights.push(weight);
        }
        let (multiplicities, knots) = knots_and_multiplicities(&curve.knot_vector);
        let degree = Value::Integer(curve.degree as i64);
        let form = Value::Enum("UNSPECIFIED".into());
        if weights.iter().all(|&w| w == 1.0) {
            return Ok(self.writer.add(
                "B_SPLINE_CURVE_WITH_KNOTS",
                vec![
                    string(""),
                    degree,
                    refs(points),
                    form.clone(),
                    logical(false),
                    logical(false),
                    multiplicities,
                    knots,
                    form,
                ],
            ));
        }
        let record = |name: &str, args: Vec<Value>| Record {
            name: name.into(),
            args,
        };
        Ok(self.writer.add_complex(vec![
            record("BOUNDED_CURVE", vec![]),
            record(
                "B_SPLINE_CURVE",
                vec![
                    degree,
                    refs(points),
                    form.clone(),
                    logical(false),
                    logical(false),
                ],
            ),
            record(
                "B_SPLINE_CURVE_WITH_KNOTS",
                vec![multiplicities, knots, form],
            ),
            record("CURVE", vec![]),
            record("GEOMETRIC_REPRESENTATION_ITEM", vec![]),
            record("RATIONAL_B_SPLINE_CURVE", vec![reals(weights)]),
            record("REPRESENTATION_ITEM", vec![string("")]),
        ]))
    }

    fn surface(&mut self, surface: &NurbSurface3D<S>) -> GeopResult<u64> {
        let mut rows = Vec::new();
        let mut weight_rows = Vec::new();
        for i in 0..surface.num_u {
            let mut row = Vec::new();
            let mut weights = Vec::new();
            for j in 0..surface.num_v {
                let (point, weight) =
                    self.control_point(&surface.control_points[i * surface.num_v + j])?;
                row.push(point);
                weights.push(weight);
            }
            rows.push(refs(row));
            weight_rows.push(weights);
        }
        let (u_multiplicities, u_knots) = knots_and_multiplicities(&surface.knot_vector_u);
        let (v_multiplicities, v_knots) = knots_and_multiplicities(&surface.knot_vector_v);
        let degree_u = Value::Integer(surface.degree_u as i64);
        let degree_v = Value::Integer(surface.degree_v as i64);
        let form = Value::Enum("UNSPECIFIED".into());
        if weight_rows.iter().flatten().all(|&w| w == 1.0) {
            return Ok(self.writer.add(
                "B_SPLINE_SURFACE_WITH_KNOTS",
                vec![
                    string(""),
                    degree_u,
                    degree_v,
                    Value::List(rows),
                    form.clone(),
                    logical(false),
                    logical(false),
                    logical(false),
                    u_multiplicities,
                    v_multiplicities,
                    u_knots,
                    v_knots,
                    form,
                ],
            ));
        }
        let record = |name: &str, args: Vec<Value>| Record {
            name: name.into(),
            args,
        };
        Ok(self.writer.add_complex(vec![
            record("BOUNDED_SURFACE", vec![]),
            record(
                "B_SPLINE_SURFACE",
                vec![
                    degree_u,
                    degree_v,
                    Value::List(rows),
                    form.clone(),
                    logical(false),
                    logical(false),
                    logical(false),
                ],
            ),
            record(
                "B_SPLINE_SURFACE_WITH_KNOTS",
                vec![u_multiplicities, v_multiplicities, u_knots, v_knots, form],
            ),
            record("GEOMETRIC_REPRESENTATION_ITEM", vec![]),
            record(
                "RATIONAL_B_SPLINE_SURFACE",
                vec![Value::List(weight_rows.into_iter().map(reals).collect())],
            ),
            record("REPRESENTATION_ITEM", vec![string("")]),
            record("SURFACE", vec![]),
        ]))
    }

    fn edge(&mut self, id: EdgeId) -> GeopResult<u64> {
        if let Some(&step) = self.edges.get(&id) {
            return Ok(step);
        }
        let edge = self.model.get_edge(id)?;
        let start = self.vertex(edge.start_vertex)?;
        let end = self.vertex(edge.end_vertex)?;
        let curve = self.curve(&edge.curve)?;
        let step = self.writer.add(
            "EDGE_CURVE",
            vec![
                string(&(self.name)(id.into())),
                Value::Ref(start),
                Value::Ref(end),
                Value::Ref(curve),
                logical(true),
            ],
        );
        self.edges.insert(id, step);
        Ok(step)
    }

    /// A face's boundary: a `FACE_OUTER_BOUND` for its `outer` one, a
    /// `FACE_BOUND` for a hole, turned around if `reversed`.
    fn bound(&mut self, boundary: BoundaryType, outer: bool, reversed: bool) -> GeopResult<u64> {
        let lp = match boundary {
            BoundaryType::Vertex(v) => {
                let vertex = self.vertex(v)?;
                self.writer
                    .add("VERTEX_LOOP", vec![string(""), Value::Ref(vertex)])
            }
            BoundaryType::Loop(anchor) => {
                let mut oriented = Vec::new();
                let coedges: Vec<_> = self.model.iterate_loop_coedges(anchor).collect();
                for coedge_id in coedges {
                    let coedge = self.model.get_coedge(coedge_id)?;
                    // A coedge sitting at a pole joins the edges either side
                    // of it at the pole's vertex: STEP needs nothing for it.
                    let CoedgeGeometry::Edge(edge) = coedge.geometry else {
                        continue;
                    };
                    let forward = coedge.sense == Sense::Forward;
                    let edge = self.edge(edge)?;
                    oriented.push(self.writer.add(
                        "ORIENTED_EDGE",
                        vec![
                            string(""),
                            Value::Derived,
                            Value::Derived,
                            Value::Ref(edge),
                            logical(forward),
                        ],
                    ));
                }
                self.writer
                    .add("EDGE_LOOP", vec![string(""), refs(oriented)])
            }
        };
        let kind = if outer {
            "FACE_OUTER_BOUND"
        } else {
            "FACE_BOUND"
        };
        Ok(self
            .writer
            .add(kind, vec![string(""), Value::Ref(lp), logical(!reversed)]))
    }

    fn face(&mut self, id: FaceId, reversed: bool) -> GeopResult<u64> {
        let face = self.model.get_face(id)?;
        let surface = self.surface(&face.surface)?;
        let mut bounds = vec![self.bound(face.outer, true, reversed)?];
        for &hole in &face.holes {
            bounds.push(self.bound(hole, false, reversed)?);
        }
        Ok(self.writer.add(
            "ADVANCED_FACE",
            vec![
                string(&(self.name)(id.into())),
                refs(bounds),
                Value::Ref(surface),
                logical(!reversed),
            ],
        ))
    }

    /// The shell `id` as a `kind` (`CLOSED_SHELL` or `OPEN_SHELL`), its
    /// faces turned around if `reversed`.
    fn shell(&mut self, id: ShellId, kind: &str, reversed: bool) -> GeopResult<u64> {
        let mut faces = self.model.get_shell(id)?.faces.clone();
        faces.sort_by_key(|id| id.0);
        let mut steps = Vec::new();
        for face in faces {
            steps.push(self.face(face, reversed)?);
        }
        Ok(self.writer.add(kind, vec![string(""), refs(steps)]))
    }
}
