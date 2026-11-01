//! The edit operations run as steps. Deleting bodies and extracting faces,
//! and splitting with those, are run by `geop-ops-boolean`'s tests.

// ── Direct edits ────────────────────────────────────────────────────────────

mod direct {
    use geop_core_program::{operation_registry::OperationRegistry, program::Program};
    use geop_lib_fixtures::{PrimitiveArgs, bodies::cylinder, drilled_box};
    use geop_ops_model::{
        element,
        testing::{Built, assert_valid, assert_volume},
    };

    use crate::{MoveFace, MoveFaceArgs, OffsetFace, OffsetFaceArgs, ReplaceFace, ReplaceFaceArgs};

    fn registry() -> geop_core_math::geop_error::GeopResult<OperationRegistry> {
        geop_lib_fixtures::registry()?
            .with(MoveFace)?
            .with(OffsetFace)?
            .with(ReplaceFace)
    }

    /// The top of a box drilled from the top, pulled up a quarter: the
    /// hole's wall grows with it.
    #[test]
    fn drilled_box_top_moved_up() {
        let mut program = drilled_box();
        program.push(
            "m",
            MoveFaceArgs {
                faces: vec![element("block(box,end)")],
                translation: [0.0, 0.0, 0.25],
            },
        );
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        let hole = std::f64::consts::PI * 0.4 * 0.4;
        assert_volume(&part, "move_face(m)", 1.25 * 4.0 - 0.75 * hole, 2e-2);
        assert!(part.face_id("block(box,end)").is_ok());
    }

    /// A cylinder's wall offset out by a half: a cylinder of radius 1.5.
    #[test]
    fn cylinder_wall_offset() {
        let mut program = cylinder();
        let part = program.build_model(registry).unwrap();
        let walls: Vec<_> = part
            .topology()
            .faces
            .iter()
            .filter(|(_, f)| f.surface.as_plane().unwrap().is_none())
            .map(|(&id, _)| element(part.name_of(id).unwrap()))
            .collect();
        program.push(
            "o",
            OffsetFaceArgs {
                faces: walls,
                distance: 0.5,
            },
        );
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        let volume = std::f64::consts::PI * 1.5 * 1.5 * 1.5;
        assert_volume(&part, "offset_face(o)", volume, 5e-2);
    }

    /// A box's top put on the top of a taller box beside it: as tall.
    #[test]
    fn box_top_replaced_by_a_higher_one() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        program.push("b", PrimitiveArgs::block([3.0, 0.0, 0.0], [4.0, 1.0, 2.0]));
        program.push(
            "r",
            ReplaceFaceArgs {
                face: element("block(a,end)"),
                with: element("block(b,end)"),
            },
        );
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        assert_volume(&part, "replace_face(r)", 2.0, 1e-5);
    }

    /// The top pushed down through the bottom is refused: the sides would
    /// vanish.
    #[test]
    fn box_top_pushed_through_is_refused() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        program.push(
            "o",
            OffsetFaceArgs {
                faces: vec![element("block(a,end)")],
                distance: -1.5,
            },
        );
        let error = program.build_model(registry).unwrap_err();
        assert!(format!("{error}").contains("turns around"), "{error}");
    }
}

// ── Deleting faces ──────────────────────────────────────────────────────────

mod delete_face {
    use geop_core_program::{operation_registry::OperationRegistry, program::Program};
    use geop_lib_fixtures::{
        PrimitiveArgs,
        bodies::{block, block_drilled, block_with_boss},
        drilled_box,
    };
    use geop_ops_fillet::{ChamferArgs, Fillet, FilletArgs};
    use geop_ops_model::{
        NamedModel, element,
        testing::{Built, assert_valid, assert_valid_sheets, assert_volume, edge_at},
    };

    use crate::{DeleteFace, DeleteFaceArgs, direct::HealGap};

    fn registry() -> geop_core_math::geop_error::GeopResult<OperationRegistry> {
        geop_lib_fixtures::registry()?
            .with(DeleteFace)?
            .with(Fillet)?
            .with(geop_ops_fillet::Chamfer)
    }

    /// The names of the faces of `part` whose names start with `prefix`.
    fn faces_named(part: &NamedModel, prefix: &str) -> Vec<String> {
        let mut names: Vec<String> = (part.topology().faces.keys())
            .map(|&f| part.name_of(f).unwrap().to_string())
            .filter(|n| n.starts_with(prefix))
            .collect();
        names.sort();
        names
    }

    /// `program` with the faces `faces` deleted, healed or not, as the step
    /// `d`.
    fn deleted(mut program: Program, faces: Vec<String>, heal: HealGap) -> NamedModel {
        program.push(
            "d",
            DeleteFaceArgs {
                faces: faces.into_iter().map(element).collect(),
                heal,
                whole_feature: false,
            },
        );
        program.build_model(registry).unwrap()
    }

    /// The blind hole of a box deleted, wall and bottom: the box whole.
    #[test]
    fn blind_hole_filled() {
        let program = drilled_box();
        let part = program.build_model(registry).unwrap();
        let hole = faces_named(&part, "cylinder(hole");
        let part = deleted(program, hole, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
        assert_eq!(part.topology().faces.len(), 6);
    }

    /// A hole through a box deleted: its wall alone closes two holes.
    #[test]
    fn through_hole_filled() {
        let program = block_drilled();
        let part = program.build_model(registry).unwrap();
        let hole = faces_named(&part, "cylinder(hole");
        let part = deleted(program, hole, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
    }

    /// A boss on a box deleted, its wall and its top.
    #[test]
    fn boss_removed() {
        let program = block_with_boss();
        let part = program.build_model(registry).unwrap();
        let boss = faces_named(&part, "cylinder(boss");
        let part = deleted(program, boss, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
    }

    /// A cut-off vertical edge of a box: its cut face deleted, the two
    /// sides grown back to their edge.
    #[test]
    fn cut_off_edge_restored() {
        let mut program = block();
        program.push(
            "corner",
            PrimitiveArgs::prism(&[[1.0, 2.5], [2.5, 1.0], [2.5, 2.5]], -0.5, 1.5)
                .cut("block(body)"),
        );
        let part = program.build_model(registry).unwrap();
        assert_volume(&part, &part.solid_names()[0], 4.0 - 0.125, 1e-5);
        let cut: Vec<String> = faces_named(&part, "")
            .into_iter()
            .filter(|n| n.starts_with("prism(corner,"))
            .collect();
        assert_eq!(cut.len(), 1, "{cut:?}");
        let part = deleted(program, cut, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
        assert_eq!(part.topology().faces.len(), 6);
        assert_eq!(part.topology().edges.len(), 12);
    }

    /// A chamfer along a box's top edge deleted: the sharp edge back.
    #[test]
    fn chamfer_removed() {
        let mut program = block();
        let part = program.build_model(registry).unwrap();
        let edge = edge_at(&part, [1.0, 0.0, 1.0]);
        program.push("c", ChamferArgs::even(vec![edge], 0.2));
        let part = program.build_model(registry).unwrap();
        let chamfer: Vec<String> = faces_named(&part, "chamfer(c")
            .into_iter()
            .filter(|n| n.ends_with(",chamfer)"))
            .collect();
        assert_eq!(chamfer.len(), 1, "{chamfer:?}");
        let part = deleted(program, chamfer, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
    }

    /// A ball cut out of a box's corner leaves a spherical triangle;
    /// deleted, the three sides grow back to the corner.
    #[test]
    fn corner_scoop_closed() {
        let mut program = block();
        program.push(
            "scoop",
            PrimitiveArgs::sphere([2.0, 2.0, 1.0], 0.5).cut("block(body)"),
        );
        let part = program.build_model(registry).unwrap();
        let scoop: Vec<String> = faces_named(&part, "")
            .into_iter()
            .filter(|n| {
                let face = part.face_id(n).unwrap();
                (part.topology().get_face(face).unwrap().surface)
                    .as_plane()
                    .unwrap()
                    .is_none()
            })
            .collect();
        assert_eq!(scoop.len(), 1, "{scoop:?}");
        let part = deleted(program, scoop, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
        assert_eq!(part.topology().vertices.len(), 8);
    }

    /// A fillet along a box's top edge deleted: the sharp edge back.
    #[test]
    fn fillet_removed() {
        let mut program = block();
        let part = program.build_model(registry).unwrap();
        let edge = edge_at(&part, [1.0, 0.0, 1.0]);
        program.push("f", FilletArgs::constant(vec![edge], 0.2));
        let part = program.build_model(registry).unwrap();
        let fillet: Vec<String> = faces_named(&part, "fillet(f")
            .into_iter()
            .filter(|n| {
                let face = part.face_id(n).unwrap();
                (part.topology().get_face(face).unwrap().surface)
                    .as_plane()
                    .unwrap()
                    .is_none()
            })
            .collect();
        assert_eq!(fillet.len(), 1, "{fillet:?}");
        let part = deleted(program, fillet, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
    }

    /// Fillets round two top edges of a box meeting at a corner, deleted
    /// together: a gap of more than four sides, closed by the faces
    /// around it meeting as they did, the sharp corner back.
    #[test]
    fn fillet_chain_removed() {
        let mut program = block();
        let part = program.build_model(registry).unwrap();
        let edges = vec![
            edge_at(&part, [1.0, 0.0, 1.0]),
            edge_at(&part, [2.0, 1.0, 1.0]),
        ];
        program.push("f", FilletArgs::constant(edges, 0.2));
        let part = program.build_model(registry).unwrap();
        let fillets: Vec<String> = faces_named(&part, "")
            .into_iter()
            .filter(|n| {
                let face = part.face_id(n).unwrap();
                (part.topology().get_face(face).unwrap().surface)
                    .as_plane()
                    .unwrap()
                    .is_none()
            })
            .collect();
        assert!(fillets.len() >= 2, "{fillets:?}");
        let part = deleted(program, fillets, HealGap::Heal);
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
        assert_eq!(part.topology().faces.len(), 6);
    }

    /// A box's top deleted and left open: a sheet of the other five.
    #[test]
    fn top_left_open() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        let part = deleted(program, vec!["block(a,end)".into()], HealGap::Open);
        assert_valid_sheets(&part);
        assert!(part.solid_names().is_empty());
        assert_eq!(part.sheet_face_names().len(), 5);
    }

    /// A box's top healed over cannot be: its sides are parallel and never
    /// meet. Refused, saying so.
    #[test]
    fn top_of_a_box_cannot_heal() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        program.push(
            "d",
            DeleteFaceArgs {
                faces: vec![element("block(a,end)")],
                heal: HealGap::Heal,
                whole_feature: false,
            },
        );
        let error = program.build_model(registry).unwrap_err();
        assert!(format!("{error}").contains("meet"), "{error}");
    }
}

// ── Merging faces on one surface ────────────────────────────────────────────

mod merge_faces {
    use geop_core_program::element::ElementReference;
    use geop_core_program::{operation_registry::OperationRegistry, program::Program};
    use geop_lib_fixtures::PrimitiveArgs;
    use geop_ops_model::{
        Combine, NamedModel, model,
        testing::{Built, assert_valid, assert_volume},
    };

    use crate::{MergeFaces, MergeFacesArgs};

    fn registry() -> geop_core_math::geop_error::GeopResult<OperationRegistry> {
        geop_lib_fixtures::registry()?.with(MergeFaces)
    }

    /// `program` and the step `m` merging the faces of its solid `solid`.
    fn merged(mut program: Program, solid: &str) -> NamedModel {
        program.push(
            "m",
            MergeFacesArgs {
                bodies: vec![ElementReference::new(model(), solid)],
            },
        );
        program.build_model(registry).unwrap()
    }

    /// Two blocks united flush are one box already, six faces, twelve
    /// edges, eight corners: the union made them one, and the step finds
    /// nothing more. The names of the faces made part of others still find
    /// them.
    #[test]
    fn blocks_united_flush_are_one_box() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        program.push(
            "b",
            PrimitiveArgs::block([1.0, 0.0, 0.0], [2.0, 1.0, 1.0])
                .combined(Combine::join("block(a)")),
        );
        let joined = program.build_model(registry).unwrap();
        let solid = joined.solid_names()[0].clone();
        assert_eq!(joined.topology().faces.len(), 6);
        let part = merged(program, &solid);
        assert_valid(&part);
        let model = part.topology();
        assert_eq!(model.faces.len(), 6);
        assert_eq!(model.edges.len(), 12);
        assert_eq!(model.vertices.len(), 8);
        assert_volume(&part, &solid, 2.0, 1e-5);
        let top_a = part.face_id("block(a,end)").unwrap();
        let top_b = part.face_id("block(b,end)").unwrap();
        assert_eq!(top_a, top_b);
    }

    /// A row of four overlapping blocks joined one by one: one long box.
    #[test]
    fn a_row_of_overlapping_blocks_is_one_box() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [1.0, 1.0, 1.0]));
        let mut last = "block(a)".to_string();
        for (k, step) in ["b", "c", "d"].iter().enumerate() {
            let x = 0.6 * (k + 1) as f64;
            program.push(
                *step,
                PrimitiveArgs::block([x, 0.0, 0.0], [x + 1.0, 1.0, 1.0])
                    .combined(Combine::join(&last)),
            );
            last = format!("block({step})");
        }
        let joined = program.build_model(registry).unwrap();
        let solid = joined.solid_names()[0].clone();
        let part = merged(program, &solid);
        assert_valid(&part);
        assert_eq!(part.topology().faces.len(), 6);
        assert_volume(&part, &solid, 2.8, 1e-5);
    }

    /// A box with a block standing on its top, not flush: nothing lies on
    /// one surface across an edge, and nothing changes.
    #[test]
    fn a_step_stays_a_step() {
        let mut program = Program::new();
        program.push("a", PrimitiveArgs::block([0.0; 3], [2.0, 2.0, 1.0]));
        program.push(
            "b",
            PrimitiveArgs::block([0.5, 0.5, 1.0], [1.5, 1.5, 2.0])
                .combined(Combine::join("block(a)")),
        );
        let joined = program.build_model(registry).unwrap();
        let solid = joined.solid_names()[0].clone();
        let part = merged(program, &solid);
        assert_valid(&part);
        assert_eq!(part.topology().faces.len(), joined.topology().faces.len());
    }
}

// ── Taking out whole features ───────────────────────────────────────────────

mod whole_feature {
    use geop_core_program::{operation_registry::OperationRegistry, program::Program};
    use geop_lib_fixtures::{bodies::block_with_boss, drilled_box};
    use geop_ops_model::{
        NamedModel, element,
        testing::{Built, assert_valid, assert_volume},
    };

    use crate::{DeleteFace, DeleteFaceArgs, direct::HealGap};

    fn registry() -> geop_core_math::geop_error::GeopResult<OperationRegistry> {
        geop_lib_fixtures::registry()?.with(DeleteFace)
    }

    /// `program` with the whole feature `face` is part of taken out, healed
    /// over.
    fn defeatured(
        mut program: Program,
        face: &str,
    ) -> geop_core_math::geop_error::GeopResult<NamedModel> {
        program.push(
            "d",
            DeleteFaceArgs {
                faces: vec![element(face)],
                heal: HealGap::Heal,
                whole_feature: true,
            },
        );
        program.build_model(registry)
    }

    /// One piece of a blind hole's wall picked: the whole hole, its other
    /// pieces and its bottom, taken out — the box whole again.
    #[test]
    fn a_hole_from_one_face_of_it() {
        let program = drilled_box();
        let part = program.build_model(registry).unwrap();
        let mut walls: Vec<String> = (part.topology().faces.keys())
            .map(|&f| part.name_of(f).unwrap().to_string())
            .filter(|n| n.starts_with("cylinder(hole"))
            .collect();
        walls.sort();
        let part = defeatured(program, &walls[0]).unwrap();
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
        assert_eq!(part.topology().faces.len(), 6);
    }

    /// A boss's top picked: the boss, wall and top, taken out.
    #[test]
    fn a_boss_from_its_top() {
        let program = block_with_boss();
        let part = program.build_model(registry).unwrap();
        let top = (part.topology().faces.keys())
            .map(|&f| part.name_of(f).unwrap().to_string())
            .find(|n| n.starts_with("cylinder(boss") && n.contains("end"))
            .expect("the boss's top");
        let part = defeatured(program, &top).unwrap();
        assert_valid(&part);
        assert_volume(&part, "delete_face(d)", 4.0, 1e-5);
    }

    /// The face a feature stands on is part of no feature: refused, naming
    /// it.
    #[test]
    fn the_face_a_hole_is_in_is_no_feature() {
        let error = defeatured(drilled_box(), "block(box,end)").unwrap_err();
        let message = format!("{error}");
        assert!(message.contains("part of no feature"), "{message}");
    }
}
