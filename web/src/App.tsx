import { useEffect, useMemo, useRef, useState } from "react";
import "./App.css";
import {
  examplePrograms,
  currentProgram,
  entities,
  loadGeop,
  operationSchemas,
  previewProgram,
  runProgram,
  sketchPlane,
  updateProgram,
  sameEntity,
  type ArgSchema,
  type EntityRef,
  type Highlight,
  type OperationSchema,
  type Program,
  type ProgramEdit,
  type RunResult,
  type Sketch,
  type StepHandle,
  type Vec3,
} from "./geop";
import { DEFAULT_POSE, headOnPose, poseForSketchView, type CameraPose, type Projection } from "./camera";
import { OperationForm } from "./OperationForm";
import {
  acceptedDatums,
  argsSummary,
  defaultArgs,
  isComplete,
  pickedValue,
  withArg,
  withPath,
} from "./operationArgs";
import type { HandleEdit } from "./handles3d";
import { SceneViewer, type Marker, type ViewRay } from "./SceneViewer";
import { SketchEditor } from "./SketchEditor";
import { Timeline, type TimelineStep } from "./Timeline";
import { toSketch } from "./sketchGeometry";
import { formEdit, type FormState, type SketchSession } from "./editorState";
import { pickAt } from "./picking";
import { Toolbar } from "./Toolbar";
import { MobileBottom, type MobileTab } from "./MobileBottom";
import { useIsMobile } from "./useIsMobile";

const MARKER_COLOR = 0xffa040;

/**
 * The editor: a tool for writing part programs. The program lives in the
 * kernel and changes only through `updateProgram` — every button here just
 * composes one `ProgramEdit` — so editing works the same as in any other
 * editor of these programs.
 */
function App() {
  const [wasmReady, setWasmReady] = useState(false);
  const [wasmError, setWasmError] = useState<string | null>(null);
  const [schemas, setSchemas] = useState<OperationSchema[]>([]);

  const [program, setProgram] = useState<Program>({ steps: [] });
  /** Earlier programs, for undo — restored through `updateProgram` like any edit. */
  const [history, setHistory] = useState<Program[]>([]);
  /** Programs undone, most recent last, for redo — gone with the next edit. */
  const [future, setFuture] = useState<Program[]>([]);
  /** Rolled back: only the first `marker` steps run, and new steps go there. `null`: the end. */
  const [marker, setMarker] = useState<number | null>(null);
  const [committed, setCommitted] = useState<RunResult | null>(null);
  const [committedError, setCommittedError] = useState<string | null>(null);

  const [form, setForm] = useState<FormState | null>(null);
  const [pickArg, setPickArg] = useState<string | null>(null);
  const [pickPoints, setPickPoints] = useState<Record<string, Vec3>>({});
  /** What a click would pick right now, drawn highlighted. */
  const [hover, setHover] = useState<Highlight | null>(null);
  const [preview, setPreview] = useState(true);
  const [formError, setFormError] = useState<string | null>(null);

  /** Which panel the bottom half shows on a narrow (mobile) screen — irrelevant on desktop, where all three show at once. */
  const [mobileTab, setMobileTab] = useState<MobileTab>("buttons");
  const isMobile = useIsMobile();
  /** The mobile "Draw" tab's pane, once mounted — where the sketch editor's Draw/Constrain/Status panel portals to. */
  const [sketchPanelHost, setSketchPanelHost] = useState<HTMLDivElement | null>(null);
  /** The mobile "Bug" tab's pane, once mounted — where the bug-report form portals to when open. */
  const [bugReportHost, setBugReportHost] = useState<HTMLDivElement | null>(null);
  const [bugReportOpen, setBugReportOpen] = useState(false);

  const [session, setSession] = useState<SketchSession | null>(null);
  // Entering a sketch: the camera glides to face its plane first, and the
  // editor, drawn over the 3-D view, takes over once it lands — from then
  // on it holds the camera where its view is.
  const [arriving, setArriving] = useState<SketchSession | null>(null);
  const [focus, setFocus] = useState<CameraPose | null>(null);
  const [heldPose, setHeldPose] = useState<CameraPose | null>(null);
  const [projection, setProjection] = useState<Projection>("perspective");
  const poseRef = useRef<CameraPose>(DEFAULT_POSE);
  /** Where the camera was before sketching, to come back to. */
  const beforeSketchRef = useRef<CameraPose>(DEFAULT_POSE);

  useEffect(() => {
    loadGeop()
      .then(() => {
        setSchemas(operationSchemas());
        setProgram(currentProgram());
        setWasmReady(true);
      })
      .catch((e) => setWasmError(String(e)));
  }, []);

  // While a step is being written, the program runs only up to it: what it
  // can refer to is what the steps before it built, and the steps after it
  // needn't run on every change.
  const runStop = form ? form.index : marker;
  useEffect(() => {
    if (!wasmReady) return;
    try {
      setCommitted(runProgram(runStop));
      setCommittedError(null);
    } catch (e) {
      setCommittedError(String(e));
    }
  }, [wasmReady, program, runStop]);

  const complete = form != null && isComplete(form.schema, form.args);
  const pendingEdit = form && complete ? formEdit(form) : null;
  const pendingKey = JSON.stringify(pendingEdit);

  const [previewResult, setPreviewResult] = useState<RunResult | null>(null);
  const [previewError, setPreviewError] = useState<string | null>(null);
  useEffect(() => {
    if (!wasmReady || !form || !preview || !pendingEdit) {
      setPreviewResult(null);
      setPreviewError(null);
      return;
    }
    try {
      // Up to and including the step being written.
      const result = previewProgram(pendingEdit, form.index + 1);
      setPreviewResult(result);
      setPreviewError(result.results.find((r) => r.error)?.error ?? null);
    } catch (e) {
      setPreviewResult(null);
      setPreviewError(String(e));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [wasmReady, preview, pendingKey, program]);

  // ── handles ────────────────────────────────────────────────────────────

  /**
   * A drag of one of the form's handles (only its step's are shown): it
   * sets the arguments like typing would, top-level ones through `withArg`
   * so what follows them follows. It does not make them the user's: a
   * dragged distance may still flip an untouched combine between join and
   * cut.
   */
  function handleDragged(_handle: StepHandle, edits: HandleEdit[]) {
    if (!form) return;
    let args = form.args;
    for (const e of edits) {
      args = e.path.length === 1 ? withArg(form.schema, args, e.path[0], e.value, committed, form.touched) : withPath(args, e.path, e.value);
    }
    setForm({ ...form, args });
  }

  /**
   * The handles to offer: only while a step is being created or edited,
   * only that step's (as previewed), and a feature's, never a sketch's —
   * sketches are edited in the sketcher.
   */
  function shownHandles(): StepHandle[] {
    if (!form || session || arriving) return [];
    return (previewResult?.handles ?? []).filter((h) => h.group === "feature" && h.step === pendingId);
  }

  /** The id of the step the open form writes, as the preview runs it. */
  const pendingId = form ? (form.stepId ?? previewResult?.results[form.index]?.id ?? null) : null;

  // While sketching, the model as the steps before the sketch built it: a
  // preview would draw the drawing being edited a second time.
  const displayResult = session || arriving ? committed : (previewResult ?? committed);
  const schemaOf = (kind: string) => schemas.find((s) => s.kind === kind);

  /**
   * The sketches and datums a shown step has used — extruded a sketch,
   * sketched on a datum plane, built a datum from another: the viewer hides
   * them, since what was made from them shows them now. A step that failed
   * used nothing. None of a kind while an argument that takes that kind is
   * being picked: then any can be chosen, used or not.
   */
  function usedReferences(): string[] {
    if (!displayResult) return [];
    const pickingSketch = pickSchema?.kind.type === "sketch";
    const pickingDatum = pickSchema != null && acceptedDatums(pickSchema.kind).length > 0;
    const datumNames = (value: unknown) =>
      entities(value).flatMap((e) => (e.type === "Datum" ? [e.name] : []));
    const used = (arg: ArgSchema, value: unknown): string[] => {
      switch (arg.kind.type) {
        case "sketch":
          return pickingSketch ? [] : [value as string];
        case "plane":
          return pickingDatum ? [] : datumNames([value]);
        case "selection":
          return pickingDatum ? [] : datumNames(value);
        default:
          return [];
      }
    };
    return displayResult.results.flatMap((r) => {
      if (r.error) return [];
      const step =
        form && r.id === pendingId ? { operation: form.schema.kind, args: form.args } : program.steps.find((s) => s.id === r.id);
      const schema = step && schemaOf(step.operation);
      if (!step || !schema) return [];
      return schema.args.flatMap((a) => used(a, step.args[a.name]));
    });
  }

  /** Apply `edit` to the program, remembering the program before it for undo. Throws if it is rejected. */
  function edit(e: ProgramEdit): string | null {
    const before = program;
    const id = updateProgram(e);
    setHistory((h) => [...h, before]);
    setFuture([]);
    setProgram(currentProgram());
    return id;
  }

  function undo() {
    const previous = history[history.length - 1];
    if (!previous) return;
    updateProgram({ edit: "replace", program: previous });
    setHistory(history.slice(0, -1));
    setFuture([...future, program]);
    setProgram(currentProgram());
    setMarker(null);
  }

  function redo() {
    const next = future[future.length - 1];
    if (!next) return;
    updateProgram({ edit: "replace", program: next });
    setFuture(future.slice(0, -1));
    setHistory([...history, program]);
    setProgram(currentProgram());
    setMarker(null);
  }

  function openForm(next: FormState) {
    setForm(next);
    setFormError(null);
    setPickPoints({});
    // A new step's plane or selection is picked more often than not: arm
    // that pick right away.
    const first = next.schema.args.find((a) => a.kind.type === "plane" || a.kind.type === "selection");
    setPickArg(next.stepId == null ? (first?.name ?? null) : null);
    setMobileTab("detail");
  }

  function newStep(schema: OperationSchema) {
    if (form?.schema === schema) {
      closeForm();
      return;
    }
    const index = marker ?? program.steps.length;
    // Defaults come from what the steps before the new one built.
    openForm({ schema, args: defaultArgs(schema, runProgram(index)), index, stepId: null, touched: [] });
  }

  function editStep(index: number) {
    const step = program.steps[index];
    const schema = schemaOf(step.operation);
    if (!schema) return;
    // An existing step's arguments were all chosen already.
    const next: FormState = { schema, args: step.args, index, stepId: step.id, touched: schema.args.map((a) => a.name) };
    openForm(next);
    // A drawing is what is edited in such a step: straight to it. Its
    // form, e.g. to change the plane, is one button away in the editor.
    const drawing = schema.args.find((a) => a.kind.type === "drawing");
    if (drawing) draw(next, drawing.name);
  }

  function closeForm() {
    setForm(null);
    setPickArg(null);
    setPickPoints({});
    setFormError(null);
    setMobileTab((tab) => (tab === "detail" ? "buttons" : tab));
  }

  function commit(f: FormState) {
    try {
      edit(formEdit(f));
      if (f.stepId == null && marker != null) setMarker(marker + 1);
      closeForm();
    } catch (e) {
      setFormError(String(e));
    }
  }

  /**
   * The user set `name` to `value`: it is theirs now, unless `touch` is
   * false — picking a combine's target is not choosing its mode.
   */
  function setArg(name: string, value: unknown, touch = true) {
    if (!form) return;
    const touched = touch && !form.touched.includes(name) ? [...form.touched, name] : form.touched;
    setForm({ ...form, args: withArg(form.schema, form.args, name, value, committed, touched), touched });
  }

  function tryEdit(e: ProgramEdit) {
    try {
      edit(e);
      setCommittedError(null);
    } catch (err) {
      setCommittedError(String(err));
    }
  }

  function removeStep(index: number) {
    tryEdit({ edit: "remove", id: program.steps[index].id });
    if (marker != null && index < marker) setMarker(marker - 1);
  }

  function moveStep(index: number, to: number) {
    tryEdit({ edit: "move", id: program.steps[index].id, index: to });
  }

  function loadProgram(p: Program) {
    tryEdit({ edit: "replace", program: p });
    closeForm();
    setSession(null);
    setMarker(null);
  }

  function saveProgram() {
    const json = JSON.stringify(program, null, 2);
    const url = URL.createObjectURL(new Blob([json], { type: "application/json" }));
    const a = document.createElement("a");
    a.href = url;
    a.download = "part.program.json";
    a.click();
    URL.revokeObjectURL(url);
  }

  function loadFile(file: File) {
    file
      .text()
      .then((text) => loadProgram(JSON.parse(text) as Program))
      .catch((e) => setCommittedError(String(e)));
  }

  // ── picking ────────────────────────────────────────────────────────────

  /** The argument waiting for a pick, if any. */
  const pickSchema = form?.schema.args.find((a) => a.name === pickArg);

  /** The pointer moved: highlight what a click there would pick. */
  function handleHover(ray: ViewRay | null) {
    const next = ray && pickSchema ? (pickAt(pickSchema.kind, ray)?.hit ?? null) : null;
    setHover((current) => (current === next || (current && next && sameEntity(current, next)) ? current : next));
  }

  // A pick armed, made or dropped: whatever was lit no longer applies.
  useEffect(() => setHover(null), [pickArg]);

  /**
   * `hit` was picked, at `point`, for the argument waiting for a pick. A
   * selection goes on picking; any other argument has its value.
   *
   * Picking the plane a drawing argument draws on goes straight into the
   * sketch editor — the same jump `editStep` makes for an existing step —
   * rather than leaving the user to find and click "Draw sketch…" next.
   */
  function picked(hit: Highlight | null, point: Vec3 | null) {
    if (!form || !pickArg) return;
    const arg = form.schema.args.find((a) => a.name === pickArg);
    if (!arg) return;
    const selecting = arg.kind.type === "selection";
    if (!selecting) setPickArg(null);
    const value = hit && pickedValue(arg.kind, form.args[arg.name], hit);
    if (value == null) return;
    const touch = arg.kind.type !== "combine";
    const touched = touch && !form.touched.includes(arg.name) ? [...form.touched, arg.name] : form.touched;
    const args = withArg(form.schema, form.args, arg.name, value, committed, touched);
    const next = { ...form, args, touched };
    setForm(next);
    if (!selecting && point) setPickPoints((points) => ({ ...points, [arg.name]: point }));

    const drawing = form.schema.args.find((a) => a.kind.type === "drawing" && a.kind.plane === arg.name);
    if (drawing) draw(next, drawing.name);
  }

  function handlePick(ray: ViewRay) {
    const found = pickSchema ? pickAt(pickSchema.kind, ray) : null;
    picked(found?.hit ?? null, found?.point ?? null);
  }

  /** A click on the origin gizmo: its origin, axis or base plane. */
  function handleEntity(entity: EntityRef, point: Vec3) {
    picked(entity, point);
  }

  const pickable = pickSchema ? acceptedDatums(pickSchema.kind) : [];

  /** What to draw lit: what a click would pick, and what the open form has selected. */
  const highlights: Highlight[] = [
    ...(hover ? [hover] : []),
    ...(form ? form.schema.args.flatMap((a) => (a.kind.type === "selection" ? entities(form.args[a.name]) : [])) : []),
  ];

  const markers: Marker[] = Object.values(pickPoints).map((point) => ({ point, color: MARKER_COLOR }));

  // ── sketching ──────────────────────────────────────────────────────────

  /** Draw the drawing argument `arg` of the form `f`, on the plane its schema names. */
  function draw(f: FormState, arg: string) {
    const kind = f.schema.args.find((a) => a.name === arg)?.kind;
    if (kind?.type !== "drawing") return;
    const plane = f.args[kind.plane] as EntityRef;
    try {
      const frame = sketchPlane(plane);
      beforeSketchRef.current = poseRef.current;
      setArriving({ arg, plane, frame, sketch: f.args[arg] as Sketch, view: null });
      setFocus(headOnPose(frame, poseRef.current));
    } catch (e) {
      setFormError(String(e));
    }
  }

  /** The camera landed: open the editor on exactly the view it shows. */
  function onFocusReached({ pose, pixelsPerUnit, height }: { pose: CameraPose; pixelsPerUnit: number; height: number }) {
    poseRef.current = pose;
    setFocus(null);
    if (!arriving) return;
    setSession({ ...arriving, view: { center: toSketch(arriving.frame, pose.target).xy, scale: pixelsPerUnit, height } });
    setArriving(null);
    setMobileTab("draw");
  }

  /** Leave the editor: the camera, let go, glides back to where it was before. */
  function leaveSketch() {
    setSession(null);
    setHeldPose(null);
    setFocus({ ...beforeSketchRef.current });
    // Back to the form that was open (closeForm downgrades this further if
    // it turns out to have been closed too, e.g. cancelling the sketch).
    setMobileTab((tab) => (tab === "draw" ? "detail" : tab));
  }

  /** The form with the drawing in progress written in. */
  function withDrawing(sketch: Sketch): FormState | null {
    if (!session || !form) return null;
    return { ...form, args: withArg(form.schema, form.args, session.arg, sketch, committed, form.touched) };
  }

  /** The drawing is done: into the form — and, if that completes it, into the program. */
  function finishSketch(sketch: Sketch) {
    const next = withDrawing(sketch);
    if (!next) return;
    leaveSketch();
    if (isComplete(next.schema, next.args)) commit(next);
    else setForm(next);
  }

  /** Back to the form, drawing kept: its other arguments — the plane — can change, and Draw comes back here. */
  function sketchSetup(sketch: Sketch) {
    const next = withDrawing(sketch);
    if (!next) return;
    leaveSketch();
    setForm(next);
  }

  /** Drop the drawing, and with it the step being written. */
  function cancelSketch() {
    leaveSketch();
    closeForm();
  }

  const busy = !wasmReady || session != null || arriving != null;
  const examples = useMemo(() => (wasmReady ? examplePrograms() : []), [wasmReady]);

  // A shared link — app.geop-cad.dev/?example=<name> — loads that example
  // once it's available, in place of the (still-empty) starting program.
  // Once, not on every re-render: a ref, not state, records it happened.
  const loadedFromUrl = useRef(false);
  useEffect(() => {
    if (loadedFromUrl.current || examples.length === 0) return;
    loadedFromUrl.current = true;
    const name = new URLSearchParams(window.location.search).get("example");
    const example = name && examples.find((x) => x.name === name);
    if (example) loadProgram(example.program);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [examples]);

  const ran = committed?.results.length ?? 0;

  const stepCount = program.steps.length;
  /** Move the seeker to just after the first `n` steps — the end means "all of them". */
  const seek = (n: number) => setMarker(n >= stepCount ? null : n);
  const timelineSteps: TimelineStep[] = program.steps.map((step, i) => {
    const schema = schemaOf(step.operation);
    const editing = form?.stepId === step.id;
    return {
      id: step.id,
      title: `${schema?.label ?? step.operation}: ${argsSummary(schema, step.args)}`,
      error: (i < ran ? committed?.results[i]?.error : null) ?? null,
      dim: !editing && runStop != null && i >= runStop,
      editing,
    };
  });

  // The operation-buttons row, the program panel and the step-detail form
  // are each rendered once here and placed in two spots in the tree below:
  // inline in the desktop layout, and again in the mobile bottom-tab
  // region, shown or hidden per breakpoint by CSS alone. They hold no
  // state of their own (Timeline's drag state lives in its own instance
  // either way), so reusing the same JSX in two places is safe.
  const operationButtonsRow = (
    <div className="tools operation-tools">
      {schemas.map((schema) => (
        <button
          key={schema.kind}
          title={schema.doc}
          className={form?.schema === schema && form.stepId == null ? "active" : ""}
          disabled={busy || (form != null && form.schema !== schema)}
          onClick={() => newStep(schema)}
        >
          {schema.label}
        </button>
      ))}
    </div>
  );

  const programPanel = (
    <section className="panel timeline">
      <h2>Program</h2>
      <Timeline
        steps={timelineSteps}
        seeker={marker ?? stepCount}
        enabled={!busy && form == null}
        onEdit={editStep}
        onRemove={removeStep}
        onMove={moveStep}
        onSeek={seek}
      />
    </section>
  );

  const sketchEditor = session?.view ? (
    <SketchEditor
      key={`${form?.stepId ?? "new"}-${session.arg}`}
      initial={session.sketch}
      initialView={session.view}
      onView={(view) => setHeldPose(poseForSketchView(session.frame, view))}
      panelHost={isMobile ? sketchPanelHost : null}
      onFinish={finishSketch}
      onSetup={sketchSetup}
      onCancel={cancelSketch}
    />
  ) : null;

  const detailAvailable = form != null && !session && !arriving;
  const detailPanel = detailAvailable && (
    <OperationForm
      schema={form.schema}
      args={form.args}
      setArg={setArg}
      stepId={form.stepId}
      before={committed}
      pickArg={pickArg}
      setPickArg={setPickArg}
      onDraw={(arg) => draw(form, arg)}
      preview={preview}
      setPreview={setPreview}
      previewError={previewError}
      error={formError}
      complete={complete}
      onCommit={() => commit(form)}
      onCancel={closeForm}
    />
  );

  return (
    <div className="app">
      <Toolbar
        busy={busy}
        hasSteps={program.steps.length > 0}
        onSave={saveProgram}
        onLoadFile={loadFile}
        exampleNames={examples.map((x) => x.name)}
        onLoadExample={(name) => {
          const example = examples.find((x) => x.name === name);
          if (example) loadProgram(example.program);
        }}
        canUndo={history.length > 0 && form == null}
        onUndo={undo}
        canRedo={future.length > 0 && form == null}
        onRedo={redo}
        operationButtons={operationButtonsRow}
        sketchingOn={(session ?? arriving)?.plane ?? null}
        committed={committed}
        committedError={committedError}
        program={program}
        stepCount={program.steps.length}
        triangleCount={displayResult?.scene.triangles.length ?? 0}
        bugReportHost={isMobile ? bugReportHost : null}
        onBugReportOpen={() => {
          setBugReportOpen(true);
          setMobileTab("bug");
        }}
        onBugReportClose={() => {
          setBugReportOpen(false);
          setMobileTab((tab) => (tab === "bug" ? "buttons" : tab));
        }}
      />
      <div className="body">
        <aside className="sidebar desktop-only">{programPanel}</aside>
        <main className="viewport">
          {wasmError && <p className="error">Failed to load wasm: {wasmError}</p>}
          {!wasmError && !wasmReady && <p className="status">Loading geop wasm module…</p>}
          {committedError && <p className="error">{committedError}</p>}
          {wasmReady && displayResult && (
              <SceneViewer
                scene={displayResult.scene}
                markers={markers}
                onPick={handlePick}
                pickable={pickable}
                onPickEntity={handleEntity}
                onHover={handleHover}
                highlights={highlights}
                datums={displayResult.datums}
                hidden={usedReferences()}
                handles={shownHandles()}
                onHandleDrag={handleDragged}
                projection={projection}
                heldPose={heldPose}
                focus={focus}
                onFocusReached={onFocusReached}
                onPose={(pose) => (poseRef.current = pose)}
              />
          )}
          {sketchEditor}
          <button
            className="projection-toggle"
            title="Switch between a perspective and an orthographic view"
            onClick={() => setProjection(projection === "perspective" ? "orthographic" : "perspective")}
          >
            {projection === "perspective" ? "Perspective" : "Orthographic"}
          </button>
          <div className="desktop-only">{detailPanel}</div>
        </main>
      </div>
      <MobileBottom
        tab={mobileTab}
        onTab={setMobileTab}
        detailAvailable={detailAvailable}
        sketchAvailable={session?.view != null}
        bugReportOpen={bugReportOpen}
        operationButtons={operationButtonsRow}
        programPanel={programPanel}
        detailPanel={detailPanel}
        onSketchPanelHost={setSketchPanelHost}
        onBugReportHost={setBugReportHost}
      />
    </div>
  );
}

export default App;
