import { useEffect, useMemo, useRef, useState } from "react";
import "./App.css";
import {
  currentProgram,
  describeProgram,
  editStep,
  examplePrograms,
  loadGeop,
  operationInfos,
  previewProgram,
  runProgram,
  updateProgram,
  type DialogValue,
  type EditEvent,
  type Edited,
  type OperationInfo,
  type Program,
  type ProgramEdit,
  type RunResult,
} from "./geop";
import { DEFAULT_POSE, headOnPose, type CameraPose, type Projection } from "./camera";
import { DialogView } from "./DialogView";
import { SceneViewer } from "./SceneViewer";
import { Timeline, type TimelineStep } from "./Timeline";
import { Toolbar } from "./Toolbar";
import { trackEdit, trackExample, trackFailures, trackFile } from "./analytics";
import { MobileBottom, type MobileTab } from "./MobileBottom";
import { useIsMobile } from "./useIsMobile";

/** A step being edited: a new one to insert at `index`, or the existing step `stepId` there. */
interface FormState {
  info: OperationInfo;
  index: number;
  stepId: string | null;
  /** The step as it now is, the session to send with the next event, and what to show. */
  edited: Edited;
}

/** The edit that writes `form` into the program. */
function formEdit(form: FormState): ProgramEdit {
  const operation = form.edited.operation;
  return form.stepId == null
    ? { edit: "insert", index: form.index, ...operation }
    : { edit: "update", id: form.stepId, ...operation };
}

/** Whether a key press belongs to a field being typed into rather than to the step. */
function typing(e: KeyboardEvent): boolean {
  const t = e.target;
  return t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement || t instanceof HTMLSelectElement;
}

/**
 * The editor: a tool for writing part programs. The program lives in the
 * kernel and changes only through `updateProgram` — every button here just
 * composes one `ProgramEdit` — and a step is edited only through
 * `editStep`: what the user does goes there, and what comes back is shown.
 * So editing works the same as in any other editor of these programs, and
 * nothing here knows any operation.
 */
function App() {
  const [wasmReady, setWasmReady] = useState(false);
  const [wasmError, setWasmError] = useState<string | null>(null);
  const [infos, setInfos] = useState<OperationInfo[]>([]);

  const [program, setProgram] = useState<Program>({ steps: [] });
  /** Earlier programs, for undo — restored through `updateProgram` like any edit. */
  const [history, setHistory] = useState<Program[]>([]);
  /** Programs undone, most recent last, for redo — gone with the next edit. */
  const [future, setFuture] = useState<Program[]>([]);
  /** Rolled back: only the first `marker` steps run, and new steps go there. `null`: the end. */
  const [marker, setMarker] = useState<number | null>(null);
  const [committed, setCommitted] = useState<RunResult | null>(null);
  const [committedError, setCommittedError] = useState<string | null>(null);

  const [form, setFormState] = useState<FormState | null>(null);
  /**
   * The step being edited as of the last event: events arrive faster than
   * React renders (a hover every frame), and each must build on the last.
   */
  const formRef = useRef<FormState | null>(null);
  const setForm = (next: FormState | null) => {
    formRef.current = next;
    setFormState(next);
  };
  const [preview, setPreview] = useState(true);
  const [formError, setFormError] = useState<string | null>(null);

  /** Which panel the bottom half shows on a narrow (mobile) screen — irrelevant on desktop, where all three show at once. */
  const [mobileTab, setMobileTab] = useState<MobileTab>("buttons");
  const isMobile = useIsMobile();
  /** The mobile "Bug" tab's pane, once mounted — where the bug-report form portals to when open. */
  const [bugReportHost, setBugReportHost] = useState<HTMLDivElement | null>(null);
  const [bugReportOpen, setBugReportOpen] = useState(false);

  const [focus, setFocus] = useState<CameraPose | null>(null);
  const [projection, setProjection] = useState<Projection>("perspective");
  const poseRef = useRef<CameraPose>(DEFAULT_POSE);
  /** Where the camera was before it turned to face the plane being worked in, to come back to. */
  const beforePlaneRef = useRef<CameraPose | null>(null);

  useEffect(() => {
    loadGeop()
      .then(() => {
        setInfos(operationInfos());
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
      const result = runProgram(runStop);
      trackFailures(
        result,
        program.steps.map((s) => s.operation),
      );
      setCommitted(result);
      setCommittedError(null);
    } catch (e) {
      setCommittedError(String(e));
    }
  }, [wasmReady, program, runStop]);

  // The kernel describes the program it holds, which `program` mirrors.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  const steps = useMemo(() => (wasmReady ? describeProgram() : []), [wasmReady, program]);

  // The step being edited, run: what it builds — and whether it builds,
  // which is what lets it into the program.
  const pendingKey = form ? JSON.stringify(formEdit(form)) : null;
  const [previewResult, setPreviewResult] = useState<RunResult | null>(null);
  const [previewError, setPreviewError] = useState<string | null>(null);
  useEffect(() => {
    const f = formRef.current;
    if (!wasmReady || !f) {
      setPreviewResult(null);
      setPreviewError(null);
      return;
    }
    try {
      // Up to and including the step being written.
      const result = previewProgram(formEdit(f), f.index + 1);
      setPreviewResult(result);
      setPreviewError(result.results.find((r) => r.error)?.error ?? null);
    } catch (e) {
      setPreviewResult(null);
      setPreviewError(String(e));
    }
  }, [wasmReady, pendingKey, program]);

  const presentation = form?.edited.presentation ?? null;
  const plane = presentation?.focus ?? null;

  // Working in a plane: the camera turns to face it, and comes back to
  // where it was once done.
  const planeKey = JSON.stringify(plane);
  useEffect(() => {
    if (plane) {
      beforePlaneRef.current ??= poseRef.current;
      setFocus(headOnPose(plane, poseRef.current));
    } else if (beforePlaneRef.current) {
      setFocus({ ...beforePlaneRef.current });
      beforePlaneRef.current = null;
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [planeKey]);

  // ── editing a step ─────────────────────────────────────────────────────

  /** Start editing: a new step of `info` at `index`, or the existing step `stepId` there. */
  function openForm(info: OperationInfo, index: number, stepId: string | null) {
    try {
      // What the step is edited against: the part the steps before it build.
      setCommitted(runProgram(index));
      const step = stepId == null ? null : program.steps.find((s) => s.id === stepId);
      const edited = editStep(step ? { operation: step, session: null } : { kind: info.kind });
      setForm({ info, index, stepId, edited });
      setFormError(null);
      setMobileTab("detail");
    } catch (e) {
      setCommittedError(String(e));
    }
  }

  function closeForm() {
    setForm(null);
    setFormError(null);
    setMobileTab((tab) => (tab === "detail" ? "buttons" : tab));
  }

  /** What the user did to the step being edited. */
  function send(event: EditEvent) {
    const f = formRef.current;
    if (!f) return;
    try {
      const edited = editStep({ operation: f.edited.operation, session: f.edited.session, event });
      setForm({ ...f, edited });
      setFormError(null);
    } catch (e) {
      setFormError(String(e));
    }
  }

  // Keys go to the step being edited — tools, Escape, Delete — unless
  // typed into a field.
  useEffect(() => {
    if (!form) return;
    const onKey = (e: KeyboardEvent) => {
      if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
      send({ type: "key", key: e.key });
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [form != null]);

  function newStep(info: OperationInfo) {
    if (form?.info.kind === info.kind && form.stepId == null) {
      closeForm();
      return;
    }
    openForm(info, marker ?? program.steps.length, null);
  }

  function editExisting(index: number) {
    const info = infos.find((i) => i.kind === program.steps[index].operation);
    if (info) openForm(info, index, program.steps[index].id);
  }

  function commit() {
    const f = formRef.current;
    if (!f) return;
    try {
      edit(formEdit(f));
      if (f.stepId == null && marker != null) setMarker(marker + 1);
      closeForm();
    } catch (e) {
      setFormError(String(e));
    }
  }

  // ── the program ────────────────────────────────────────────────────────

  /** Apply `edit` to the program, remembering the program before it for undo. Throws if it is rejected. */
  function edit(e: ProgramEdit): string | null {
    const before = program;
    const id = updateProgram(e);
    trackEdit(e);
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
    trackFile("saved");
  }

  function loadFile(file: File) {
    file
      .text()
      .then((text) => {
        loadProgram(JSON.parse(text) as Program);
        trackFile("loaded");
      })
      .catch((e) => setCommittedError(String(e)));
  }

  // ── what is shown ──────────────────────────────────────────────────────

  // Working in a plane, the model as the steps before the step built it: a
  // preview would draw what is being drawn a second time.
  const displayResult = !plane && preview && previewResult ? previewResult : committed;

  /**
   * The sketches and datums a shown step has used: the viewer hides them,
   * since what was made from them shows them now. None of a kind while that
   * kind is being picked: then any can be chosen, used or not.
   */
  function hidden(): string[] {
    const pickable = presentation?.pickable ?? [];
    const pickingSketch = pickable.includes("sketch");
    const pickingDatum = pickable.some((t) => typeof t === "object");
    return (displayResult?.references ?? []).flatMap((r) =>
      (r.type === "Sketch" && !pickingSketch) || (r.type === "Datum" && !pickingDatum) ? [r.name] : [],
    );
  }

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
    if (example) {
      loadProgram(example.program);
      trackExample(example.name);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [examples]);

  const ran = committed?.results.length ?? 0;

  const stepCount = program.steps.length;
  /** Move the seeker to just after the first `n` steps — the end means "all of them". */
  const seek = (n: number) => setMarker(n >= stepCount ? null : n);
  const timelineSteps: TimelineStep[] = steps.map((step, i) => {
    const editing = form?.stepId === step.id;
    return {
      id: step.id,
      title: `${step.label}: ${step.summary}`,
      error: (i < ran ? committed?.results[i]?.error : null) ?? null,
      dim: !editing && runStop != null && i >= runStop,
      editing,
    };
  });

  // The operation-buttons row, the program panel and the step's dialog are
  // each rendered once here and placed in two spots in the tree below:
  // inline in the desktop layout, and again in the mobile bottom-tab
  // region, shown or hidden per breakpoint by CSS alone. They hold no
  // state of their own (Timeline's drag state lives in its own instance
  // either way), so reusing the same JSX in two places is safe.
  const operationButtonsRow = (
    <div className="tools operation-tools">
      {infos.map((info) => (
        <button
          key={info.kind}
          title={info.doc}
          className={form?.info.kind === info.kind && form.stepId == null ? "active" : ""}
          disabled={!wasmReady || (form != null && form.info.kind !== info.kind)}
          onClick={() => newStep(info)}
        >
          {info.label}
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
        enabled={wasmReady && form == null}
        onEdit={editExisting}
        onRemove={removeStep}
        onMove={moveStep}
        onSeek={seek}
      />
    </section>
  );

  const detailPanel = form && presentation && (
    <DialogView
      label={form.info.label}
      stepId={form.stepId}
      doc={form.info.doc}
      dialog={presentation.dialog}
      onDialog={(key: string, value: DialogValue) => send({ type: "dialog", key, value })}
      preview={preview}
      setPreview={setPreview}
      previewError={previewError}
      error={formError}
      canCommit={previewResult != null && previewError == null}
      onCommit={commit}
      onCancel={closeForm}
    />
  );

  return (
    <div className="app">
      <Toolbar
        busy={!wasmReady}
        hasSteps={program.steps.length > 0}
        onSave={saveProgram}
        onLoadFile={loadFile}
        exampleNames={examples.map((x) => x.name)}
        onLoadExample={(name) => {
          const example = examples.find((x) => x.name === name);
          if (example) {
            loadProgram(example.program);
            trackExample(example.name);
          }
        }}
        canUndo={history.length > 0 && form == null}
        onUndo={undo}
        canRedo={future.length > 0 && form == null}
        onRedo={redo}
        operationButtons={operationButtonsRow}
        badge={form && plane ? `${form.info.label}: working in its plane` : null}
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
              visuals={presentation?.visuals}
              highlights={presentation?.highlights}
              pickable={presentation?.pickable}
              datums={displayResult.datums}
              extent={displayResult.extent}
              hidden={hidden()}
              plane={plane}
              grab={presentation?.grab ?? false}
              onPointer={send}
              projection={projection}
              focus={focus}
              onFocusReached={(pose) => {
                poseRef.current = pose;
                setFocus(null);
              }}
              onPose={(pose) => (poseRef.current = pose)}
            />
          )}
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
        detailAvailable={form != null}
        bugReportOpen={bugReportOpen}
        operationButtons={operationButtonsRow}
        programPanel={programPanel}
        detailPanel={detailPanel}
        onBugReportHost={setBugReportHost}
      />
    </div>
  );
}

export default App;
