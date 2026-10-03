import { useEffect, useRef, useState } from "react";
import "./App.css";
import {
  loadGeop,
  send,
  type Command,
  type EditEvent,
  type OperationInfo,
  type Program,
  type ProgramState,
  type SceneState,
  type StepState,
  type Update,
  type Value,
} from "./geop";
import { host } from "./backend";
import { DEFAULT_POSE, headOnPose, type CameraPose, type Projection } from "./camera";
import { DialogView } from "./DialogView";
import { SceneViewer } from "./SceneViewer";
import { Timeline, type TimelineStep } from "./Timeline";
import { Toolbar } from "./Toolbar";
import { trackEdit, trackExample, trackFailures, trackFile } from "./analytics";
import { MobileBottom, type MobileTab } from "./MobileBottom";
import { useIsMobile } from "./useIsMobile";

/** Whether a key press belongs to a field being typed into rather than to the step. */
function typing(e: KeyboardEvent): boolean {
  const t = e.target;
  return t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement || t instanceof HTMLSelectElement;
}

/** The commands that change the program, for statistics. */
const EDITS: Command["command"][] = ["commit", "remove", "move", "load", "load_example", "undo", "redo"];

/**
 * The editor: a view of the one the kernel runs (see `geop_cad_base::editor`).
 * Every button, click and key is a command sent to it, and what comes back
 * — the program, the part to draw, the step being edited — is shown; so
 * nothing here knows any operation, and the only state kept here is how
 * things are laid out and where the camera is.
 */
function App() {
  const [wasmReady, setWasmReady] = useState(false);
  const [wasmError, setWasmError] = useState<string | null>(null);

  const [program, setProgram] = useState<ProgramState | null>(null);
  const [scene, setScene] = useState<SceneState | null>(null);
  const [step, setStep] = useState<StepState | null>(null);
  /** Why the last command was refused, if it was. */
  const [error, setError] = useState<string | null>(null);

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

  /** Send `command` to the kernel, and show what comes back. */
  async function dispatch(command: Command): Promise<Update | null> {
    try {
      const update = await send(command);
      if (update.program) {
        setProgram(update.program);
        trackFailures(update.program.steps);
      }
      if (update.scene) setScene(update.scene);
      setStep(update.step);
      setError(update.error);
      if (!update.error && EDITS.includes(command.command)) {
        trackEdit(command, command.command === "commit" ? (step?.kind ?? undefined) : undefined);
      }
      return update;
    } catch (e) {
      setError(String(e));
      return null;
    }
  }

  useEffect(() => {
    loadGeop()
      .then(() => {
        dispatch({ command: "show" });
        setWasmReady(true);
      })
      .catch((e) => setWasmError(String(e)));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const presentation = step?.presentation ?? null;
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

  async function open(command: Command) {
    if ((await dispatch(command))?.step) setMobileTab("detail");
  }

  async function close(command: Command) {
    const update = await dispatch(command);
    if (update && !update.step) setMobileTab((tab) => (tab === "detail" ? "buttons" : tab));
  }

  /** What the user did to the step being edited. */
  function event(event: EditEvent) {
    dispatch({ command: "event", event });
  }

  // Keys go to the step being edited — tools, Escape, Delete — unless
  // typed into a field.
  const editing = step != null;
  useEffect(() => {
    if (!editing) return;
    const onKey = (e: KeyboardEvent) => {
      if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
      event({ type: "key", key: e.key });
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [editing]);

  function newStep(info: OperationInfo) {
    if (step?.kind === info.kind && step.id == null) void close({ command: "cancel" });
    else void open({ command: "new", kind: info.kind });
  }

  // ── the program ────────────────────────────────────────────────────────

  const steps = program?.steps ?? [];

  function loadProgram(p: Program) {
    return dispatch({ command: "load", program: p });
  }

  function saveProgram() {
    if (!program) return;
    const json = JSON.stringify(program.program, null, 2);
    const url = URL.createObjectURL(new Blob([json], { type: "application/json" }));
    const a = document.createElement("a");
    a.href = url;
    a.download = "part.geop";
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
      .catch((e) => setError(String(e)));
  }

  async function loadExample(name: string) {
    if ((await dispatch({ command: "load_example", name }))?.error == null) trackExample(name);
  }

  // A shared link — app.geop-cad.dev/?example=<name> — loads that example
  // once the kernel is up, in place of the (still-empty) starting program.
  useEffect(() => {
    if (!wasmReady) return;
    const name = new URLSearchParams(window.location.search).get("example");
    if (name && program?.examples.includes(name)) void loadExample(name);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [wasmReady]);

  // Hosted in VS Code, the program is the document's text: it is loaded
  // from it, and every change to the program is written back. Until the
  // document has been loaded, the starting program must not be written
  // over it — and one that cannot be read is never written over either.
  const documentLoaded = useRef(false);
  useEffect(() => {
    if (!wasmReady || !host) return;
    host.onDocument((text) => {
      let loaded: Program;
      try {
        loaded = text.trim() === "" ? { steps: [] } : (JSON.parse(text) as Program);
      } catch (e) {
        setError(`Not a geop program: ${e}`);
        return;
      }
      void loadProgram(loaded).then((update) => {
        if (update && !update.error) documentLoaded.current = true;
      });
    });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [wasmReady]);
  const programValue = program?.program;
  useEffect(() => {
    if (host && documentLoaded.current && programValue) host.programChanged(programValue);
  }, [programValue]);

  const infos = program?.operations ?? [];
  const stepCount = steps.length;
  const triangleCount = scene?.part.faces.reduce((n, f) => n + f.triangles.length, 0) ?? 0;
  const timelineSteps: TimelineStep[] = steps.map((s) => ({
    id: s.id,
    title: `${s.label}: ${s.summary}`,
    error: s.error,
    dim: !s.editing && !s.runs,
    editing: s.editing,
  }));

  const operationButtonsRow = (
    <div className="tools operation-tools">
      {infos.map((info) => (
        <button
          key={info.kind}
          title={info.doc}
          className={step?.kind === info.kind && step.id == null ? "active" : ""}
          disabled={!wasmReady || (step != null && step.kind !== info.kind)}
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
        seeker={program?.marker ?? stepCount}
        enabled={wasmReady && step == null}
        onEdit={(i) => void open({ command: "open", id: steps[i].id })}
        onRemove={(i) => dispatch({ command: "remove", id: steps[i].id })}
        onMove={(i, to) => dispatch({ command: "move", id: steps[i].id, index: to })}
        onSeek={(slot) => dispatch({ command: "seek", marker: slot >= stepCount ? null : slot })}
      />
    </section>
  );

  const detailPanel = step && (
    <DialogView
      step={step}
      onDialog={(key: string, value: Value) => event({ type: "dialog", key, value })}
      setPreview={(preview) => dispatch({ command: "preview", preview })}
      error={error}
      onCommit={() => void close({ command: "commit" })}
      onCancel={() => void close({ command: "cancel" })}
    />
  );

  return (
    <div className="app">
      <Toolbar
        busy={!wasmReady}
        hosted={host != null}
        hasSteps={stepCount > 0}
        onSave={saveProgram}
        onLoadFile={loadFile}
        exampleNames={program?.examples ?? []}
        onLoadExample={(name) => void loadExample(name)}
        canUndo={(program?.can_undo ?? false) && step == null}
        onUndo={() => dispatch({ command: "undo" })}
        canRedo={(program?.can_redo ?? false) && step == null}
        onRedo={() => dispatch({ command: "redo" })}
        operationButtons={operationButtonsRow}
        badge={step && plane ? `${step.label}: working in its plane` : null}
        error={error}
        program={program?.program ?? { steps: [] }}
        stepCount={stepCount}
        triangleCount={triangleCount}
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
          {error && !step && <p className="error">{error}</p>}
          {wasmReady && scene && (
            <SceneViewer
              part={scene.part}
              visuals={presentation?.visuals}
              highlights={presentation?.highlights}
              pickable={presentation?.pickable}
              hidden={scene.hidden}
              plane={plane}
              grab={presentation?.grab ?? false}
              onPointer={event}
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
        detailAvailable={step != null}
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
