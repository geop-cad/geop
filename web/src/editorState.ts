import type { Args, EntityRef, Frame, OperationSchema, ProgramEdit, Sketch } from "./geop";
import type { SketchView } from "./camera";

/** A step being written: a new one to insert at `index`, or the existing step `stepId` there. */
export interface FormState {
  schema: OperationSchema;
  args: Args;
  index: number;
  stepId: string | null;
  /** The arguments the user set themselves, which no other argument's change may overrule. */
  touched: string[];
}

/** A drawing argument of the open form, being drawn in the sketch editor. */
export interface SketchSession {
  arg: string;
  plane: EntityRef;
  frame: Frame;
  sketch: Sketch;
  /** What the 3-D view showed when the camera landed facing the plane: where the editor opens. */
  view: SketchView | null;
}

/** The edit that writes `form` into the program. */
export function formEdit(form: FormState): ProgramEdit {
  const operation = { operation: form.schema.kind, args: form.args };
  return form.stepId == null
    ? { edit: "insert", index: form.index, ...operation }
    : { edit: "update", id: form.stepId, ...operation };
}
