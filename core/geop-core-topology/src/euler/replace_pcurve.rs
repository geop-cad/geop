use crate::{CoedgeId, Model, argument_validation::validate_pcurve_start_and_end};
use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Swap `coedge_id`'s pcurve for `pcurve` — e.g. fixing up a coedge that
    // `mer` just moved onto a different face, whose old pcurve (valid for
    // whatever face it used to sit on) has no reason to still be valid for
    // its new one. `pcurve` must still land on the coedge's actual edge's
    // 3-D endpoints under its (current) face's surface, checked before
    // anything is mutated, so a rejected swap leaves the model untouched.
    pub fn replace_pcurve(
        self: &mut Model<S>,
        coedge_id: CoedgeId,
        pcurve: NurbCurve2D<S>,
    ) -> GeopResult<()> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::replace_pcurve(coedge_id={coedge_id}, pcurve={pcurve})"
            ))
        };

        let coedge = self.get_coedge(coedge_id)?.clone();
        let face = self.get_face(coedge.face)?.clone();
        let start = self.coedge_start_vertex(coedge_id)?.clone();
        let end = self.coedge_end_vertex(coedge_id)?.clone();
        validate_pcurve_start_and_end(&face.surface, &pcurve, &start.point, &end.point)
            .with_context(&ctx)?;

        self.get_coedge_mut(coedge_id)?.pcurve = pcurve;
        Ok(())
    }
}
