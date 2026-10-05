import { useEffect, useMemo, useRef, useState } from "react";
import "./App.css";
import {
  applyScene,
  loadGeop,
  send,
  type Bom,
  type BomStructure,
  type Command,
  type EditEvent,
  type Placed,
  type EntityRef,
  type InterferenceReport,
  type MassReport,
  type Measurement,
  type OperationInfo,
  type PartView,
  type Presentation,
  type Program,
  type ProgramState,
  type Query,
  type SceneState,
  type StepState,
  type Update,
  type Value,
} from "./geop";
import { host, onRestart } from "./backend";
import { KernelCrashed } from "./crash";
import { DEFAULT_POSE, headOnPose, wantsFraming, type CameraPose, type Drawn, type Projection } from "./camera";
import { DialogView } from "./DialogView";
import { Explorer } from "./Explorer";
import {
  freePath,
  loadWorkspace,
  parseProgram,
  programText,
  saveWorkspace,
  type Workspace,
  isStepFile,
} from "./files";
import { InspectPanel } from "./InspectPanel";
import { sectionPlane, type Section } from "./section";
import { ParametersPanel } from "./ParametersPanel";
import { JointsPanel } from "./JointsPanel";
import { SceneViewer } from "./SceneViewer";
import { StructurePanel } from "./StructurePanel";
import { OperationGrid, OperationRibbon, type Tool } from "./OperationRibbon";
import { Timeline, type TimelineStep } from "./Timeline";
import { Toolbar } from "./Toolbar";
import { trackEdit, trackExample, trackFailures, trackFile } from "./analytics";
import { MobileBottom, type MobileTab } from "./MobileBottom";
import { useIsMobile } from "./useIsMobile";
import { scopeTextSelection } from "./selection";

/** Whether a key press belongs to a field being typed into rather than to the step. */
function typing(e: KeyboardEvent): boolean {
  const t = e.target;
  return t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement || t instanceof HTMLSelectElement;
}

/** The commands that change the program, for statistics. */
const EDITS: Command["command"][] = ["commit", "remove", "move", "load", "load_example", "undo", "redo"];

/** Download `data` as the file `name`, of the media type `type`. */
function download(name: string, data: string | Uint8Array<ArrayBuffer>, type = "application/json") {
  const url = URL.createObjectURL(new Blob([data], { type }));
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  URL.revokeObjectURL(url);
}

/** The last segment of `path`. */
function baseName(path: string): string {
  return path.slice(path.lastIndexOf("/") + 1);
}

/**
 * The editor: a view of the one the kernel runs (see `geop_cad_base::editor`).
 * Every button, click and key is a command sent to it, and what comes back
 * — the program, the part to draw, the step being edited — is shown; so
 * nothing here knows any operation, and the only state kept here is how
 * things are laid out, where the camera is — and, in the browser, the
 * program files (see `files.ts`), which VS Code keeps for the extension.
 */
function App() {
  const [wasmReady, setWasmReady] = useState(false);
  const [wasmError, setWasmError] = useState<string | null>(null);

  const [program, setProgram] = useState<ProgramState | null>(null);
  const [scene, setScene] = useState<SceneState | null>(null);
  /**
   * The placed parts drawn, and the views of their components: the kernel
   * sends what changed (see [[applyScene]]). Kept in a ref too, so that
   * updates answered one after another each apply to the one before.
   */
  const [placed, setPlaced] = useState<Placed>({ instances: new Map(), components: {} });
  const placedRef = useRef(placed);
  const instances = useMemo(() => [...placed.instances.values()], [placed]);
  const components = placed.components;
  const [step, setStep] = useState<StepState | null>(null);
  /** What the drag tool shows, while it is in hand and no step is edited. */
  const [tool, setTool] = useState<Presentation | null>(null);
  /** Why the last command was refused, if it was. */
  const [error, setError] = useState<string | null>(null);
  /** What the measure tool's picks measure, while it is in hand. */
  const [measurement, setMeasurement] = useState<Measurement | null>(null);
  /** The last answers to the inspect questions — until the part changes. */
  const [mass, setMass] = useState<MassReport | null>(null);
  const [interference, setInterference] = useState<InterferenceReport | null>(null);
  const [bom, setBom] = useState<Bom | null>(null);
  const [inspecting, setInspecting] = useState(false);
  /** The section view, if one is on: the view only, never the model. */
  const [section, setSection] = useState<Section | null>(null);
  /** What the inspect panel lights, beyond what the kernel does. */
  const [lights, setLights] = useState<EntityRef[]>([]);

  // A drag selects text only within the dialog or panel it starts in (see selection.ts).
  const appRef = useRef<HTMLDivElement>(null);
  useEffect(() => (appRef.current ? scopeTextSelection(appRef.current) : undefined), []);
  /** Which panel the bottom half shows on a narrow (mobile) screen — irrelevant on desktop, where all three show at once. */
  const [mobileTab, setMobileTab] = useState<MobileTab>("buttons");
  const isMobile = useIsMobile();
  /** The mobile "Bug" tab's pane, once mounted — where the bug-report form portals to when open. */
  const [bugReportHost, setBugReportHost] = useState<HTMLDivElement | null>(null);
  const [bugReportOpen, setBugReportOpen] = useState(false);

  /** The browser's program files; unused in VS Code, whose workspace they are. */
  const [workspace, setWorkspace] = useState<Workspace>(loadWorkspace);
  const workspaceRef = useRef(workspace);
  workspaceRef.current = workspace;
  /**
   * Whether the kernel is editing one of the files yet. Until it is, what
   * it shows is its own empty starting program, which must not be written
   * over any file.
   */
  const editingFile = useRef(false);
  useEffect(() => {
    if (!host) saveWorkspace(workspace);
  }, [workspace]);

  const [focus, setFocus] = useState<CameraPose | null>(null);
  /**
   * Set afresh to frame the whole drawing (see `SceneViewer`'s `fit`): on
   * opening another file or an example, on F, by the button — and when the
   * drawing wants it (see [[wantsFraming]]).
   */
  const [fit, setFit] = useState<{ from: CameraPose | null } | null>(null);
  /** Frame the drawing, looking from `from`'s direction, else the camera's. */
  const fitView = (from: CameraPose | null = null) => setFit({ from });
  /** The drawing last shown, and whether the one shown now is to be framed (see [[wantsFraming]]). */
  const drawnRef = useRef<Drawn | null>(null);
  const frameRef = useRef(false);
  const [projection, setProjection] = useState<Projection>("perspective");
  const poseRef = useRef<CameraPose>(DEFAULT_POSE);
  /** How many commands wait for the kernel's answer: the app is busy (`aria-busy`) while any do. */
  const [pending, setPending] = useState(0);
  /** Where the camera was before it turned to face the plane being worked in, to come back to. */
  const beforePlaneRef = useRef<CameraPose | null>(null);

  /** Send `command` to the kernel, and show what comes back. */
  async function dispatch(command: Command): Promise<Update | null> {
    setPending((n) => n + 1);
    try {
      const update = await send(command);
      if (update.program) {
        setProgram(update.program);
        trackFailures(update.program.steps);
      }
      if (!host && update.files) {
        const added = Object.fromEntries(update.files.map((f) => [f.path, programText(f.program)]));
        setWorkspace((w) => ({ ...w, files: { ...w.files, ...added } }));
      }
      // The program edited is the file the kernel says it is in: kept as
      // the kernel has it, with every change.
      const edited = update.program;
      if (!host && edited?.path != null && editingFile.current) {
        const { path } = edited;
        const text = programText(edited.program);
        setWorkspace((w) =>
          w.files[path] === text && w.active === path ? w : { files: { ...w.files, [path]: text }, active: path },
        );
      }
      if (update.scene) {
        setScene(update.scene);
        // The answers were about the part as it was.
        setMass(null);
        setInterference(null);
        setBom(null);
        placedRef.current = applyScene(placedRef.current, update.scene);
        setPlaced(placedRef.current);
        const drawn = {
          extent: update.scene.part.extent,
          solids: update.scene.part.solids.length + placedRef.current.instances.size,
        };
        if (drawnRef.current && wantsFraming(drawnRef.current, drawn, poseRef.current)) frameRef.current = true;
        drawnRef.current = drawn;
      }
      setStep(update.step);
      setTool(update.tool);
      const inspection = update.inspection;
      setMeasurement(inspection?.kind === "measure" ? inspection : null);
      if (inspection?.kind === "mass_properties") setMass(inspection);
      if (inspection?.kind === "interference") setInterference(inspection);
      if (inspection?.kind === "bom") setBom(inspection);
      setError(update.error);
      if (!update.error && EDITS.includes(command.command)) {
        trackEdit(command, command.command === "commit" ? (step?.kind ?? undefined) : undefined);
      }
      return update;
    } catch (e) {
      if (e instanceof KernelCrashed) {
        // A fresh kernel has the program as it was before: show it.
        const shown = await dispatch({ command: "show" });
        if (shown) {
          setError(
            `${e.message}. This is a bug in geop: please report it. The kernel was restarted with your program as it was before.`,
          );
        }
        return null;
      }
      setError(String(e));
      return null;
    } finally {
      setPending((n) => n - 1);
    }
  }

  // Any command, run as the app runs it: for the end-to-end checks
  // (`e2e/`) and the browser's console.
  useEffect(() => {
    (window as unknown as { geopCommand: typeof dispatch }).geopCommand = dispatch;
  });

  useEffect(() => {
    // In the browser, a kernel that crashed is restarted with the files,
    // and the one edited as the kernel last had it (see `backend.ts`).
    if (!host) {
      onRestart(() => {
        const { files, active } = workspaceRef.current;
        return [
          JSON.stringify({ command: "files", files } satisfies Command),
          JSON.stringify({ command: "load", program: parseProgram(files[active]), path: active } satisfies Command),
        ];
      });
    }
    loadGeop()
      .then(async () => {
        if (host) {
          await dispatch({ command: "show" });
          fitView();
        } else {
          // Every file, then the one edited — as it was left.
          const { files, active } = workspaceRef.current;
          await dispatch({ command: "files", files });
          editingFile.current = true;
          await openFile(active, files[active]);
        }
        setWasmReady(true);
      })
      .catch((e) => setWasmError(String(e)));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const presentation = step?.presentation ?? tool;
  const plane = presentation?.focus ?? null;

  // Working in a plane: the camera turns to face it, and comes back to
  // where it was once done — framing the drawing from there, if it wants
  // framing now: a sketch drawn much larger than the view.
  const planeKey = JSON.stringify(plane);
  useEffect(() => {
    const frame = frameRef.current;
    frameRef.current = false;
    if (plane) {
      beforePlaneRef.current ??= poseRef.current;
      const headOn = headOnPose(plane, poseRef.current);
      // A sheet of its own is framed whole: it is drawn instead of the part.
      if (frame || presentation?.sheet) fitView(headOn);
      else setFocus(headOn);
    } else if (beforePlaneRef.current) {
      const before = beforePlaneRef.current;
      beforePlaneRef.current = null;
      if (frame) fitView(before);
      else setFocus({ ...before });
    } else if (frame) {
      fitView();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [planeKey]);
  // The drawing changed with the camera where it is: framed, if it wants it.
  useEffect(() => {
    if (!frameRef.current) return;
    frameRef.current = false;
    fitView();
  }, [scene]);

  // Ctrl+Z undoes, Ctrl+Shift+Z or Ctrl+Y redoes — a step's own edits
  // while one is edited — unless typed into a field.
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (typing(e) || !(e.ctrlKey || e.metaKey)) return;
      const key = e.key.toLowerCase();
      if (key === "z" && !e.shiftKey) {
        e.preventDefault();
        void dispatch({ command: "undo" });
      } else if ((key === "z" && e.shiftKey) || key === "y") {
        e.preventDefault();
        void dispatch({ command: "redo" });
      }
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

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
    return dispatch({ command: "event", event });
  }

  // Keys go to the step being edited — tools, Escape, Delete — unless
  // typed into a field. With none edited, F frames the drawing.
  const editing = step != null;
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
      if (editing) event({ type: "key", key: e.key });
      else if (e.key === "f" || e.key === "F") fitView();
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

  // ── the files (in the browser) ─────────────────────────────────────────

  /** Edit the file `path`, whose program is `text` — framed in the view, unless `keepView`. A STEP file is data, not opened. */
  async function openFile(path: string, text = workspaceRef.current.files[path], keepView = false) {
    if (isStepFile(path)) {
      setError(`${path} is a STEP file: import it into a program with "Import STEP"`);
      return null;
    }
    const update = await dispatch({ command: "load", program: parseProgram(text), path });
    if (!keepView) fitView();
    return update;
  }

  /** Tell the kernel that the files `files` changed — `null` for one gone — and keep them. */
  async function changeFiles(files: Record<string, string | null>) {
    setWorkspace((w) => {
      const next = { ...w.files };
      for (const [path, text] of Object.entries(files)) {
        if (text == null) delete next[path];
        else next[path] = text;
      }
      return { ...w, files: next };
    });
    await dispatch({ command: "files", files });
  }

  async function createFile(path: string, program: Program = { steps: [] }) {
    const text = programText(program);
    // Opened first, so the kernel rebuilds the empty new file, not the
    // one left, when told of it.
    await openFile(path, text);
    await changeFiles({ [path]: text });
  }

  async function renameFile(from: string, to: string) {
    const text = workspaceRef.current.files[from];
    if (from === workspaceRef.current.active) await openFile(to, text, true);
    await changeFiles({ [from]: null, [to]: text });
  }

  /** Delete the files `paths` — a file, or every file of a folder — opening another if the one edited goes. */
  async function deleteFiles(paths: string[]) {
    const { files, active } = workspaceRef.current;
    const gone = new Set(paths);
    if (gone.has(active)) {
      const other = Object.keys(files)
        .filter((p) => !gone.has(p))
        .sort()[0];
      if (other) await openFile(other);
      else await createFile(freePath({}, "part.geop"));
    }
    await changeFiles(Object.fromEntries(paths.map((p) => [p, null])));
  }

  async function uploadFiles(uploaded: File[]) {
    const added: Record<string, string> = {};
    const taken = { ...workspaceRef.current.files };
    for (const file of uploaded) {
      if (isStepFile(file.name)) {
        // Data for an import: kept as it is, next to the programs.
        const text = await file.text();
        const path = freePath(taken, file.name);
        taken[path] = text;
        added[path] = text;
        continue;
      }
      try {
        const text = await file.text();
        parseProgram(text);
        const name = file.name.endsWith(".geop") ? file.name : `${file.name.replace(/\.json$/, "")}.geop`;
        const path = freePath(taken, name);
        taken[path] = text;
        added[path] = text;
      } catch (e) {
        setError(`${file.name} is not a geop program: ${e}`);
      }
    }
    const paths = Object.keys(added);
    if (paths.length === 0) return;
    await changeFiles(added);
    const program = paths.find((p) => !isStepFile(p));
    if (program) await openFile(program, added[program]);
    trackFile("loaded");
  }

  /** Write the part shown as a STEP file, and save it. */
  async function exportStep() {
    const update = await dispatch({ command: "export_step" });
    const file = update?.export;
    if (file?.text == null) return;
    if (host) host.saveFile(file);
    else download(file.name, file.text, "application/step");
    trackFile("saved");
  }

  /** Write every solid of the part shown as an STL mesh, and save it. */
  async function exportStl() {
    const update = await dispatch({ command: "export_stl" });
    const file = update?.export;
    if (!file?.bytes) return;
    if (host) host.saveFile(file);
    else download(file.name, Uint8Array.from(atob(file.bytes), (c) => c.charCodeAt(0)), "model/stl");
    trackFile("saved");
  }

  function downloadFile(path: string) {
    const text = workspaceRef.current.files[path];
    if (text == null) return;
    download(baseName(path), text, isStepFile(path) ? "application/step" : undefined);
    trackFile("saved");
  }

  /** The drawing being edited, written as `format` — dated today — and saved: downloaded, or through VS Code. */
  async function exportDrawing(format: string) {
    const date = new Date().toISOString().slice(0, 10);
    const update = await dispatch({ command: "export_drawing", format: format === "dxf" ? "dxf" : "svg", date });
    const file = update?.export;
    if (!file) return;
    if (host) host.saveFile(file);
    else download(file.name, file.text ?? "", format === "dxf" ? "application/dxf" : "image/svg+xml");
    trackFile("saved");
  }

  /** Write the bill of materials of the part shown as a CSV file, and save it. */
  async function exportBom(structure: BomStructure) {
    const update = await dispatch({ command: "export_bom", structure });
    const file = update?.export;
    if (file?.text == null) return;
    if (host) host.saveFile(file);
    else download(file.name, file.text, "text/csv");
    trackFile("saved");
  }

  /** Write the flat pattern of the newest sheet-metal body as a DXF file for laser cutting, and save it. */
  async function exportFlatPattern() {
    const update = await dispatch({ command: "export_flat_pattern" });
    const file = update?.export;
    if (file?.text == null) return;
    if (host) host.saveFile(file);
    else download(file.name, file.text, "application/dxf");
    trackFile("saved");
  }

  /** Export the assembly as a URDF robot, a ZIP archive, and save it. */
  async function exportUrdf() {
    const update = await dispatch({ command: "export_urdf" });
    const file = update?.export;
    if (!file?.bytes) return;
    if (host) host.saveFile(file);
    else download(file.name, Uint8Array.from(atob(file.bytes), (c) => c.charCodeAt(0)), "application/zip");
  }

  async function loadExample(name: string) {
    // In the browser an example gets a file of its own, rather than
    // replacing the one edited.
    if (!host) await createFile(freePath(workspaceRef.current.files, `${name}.geop`));
    if ((await dispatch({ command: "load_example", name }))?.error == null) trackExample(name);
    fitView();
  }

  async function loadWorkspaceExample(name: string) {
    const folder = Object.keys(workspaceRef.current.files).some((p) => p.startsWith(`examples/${name}/`))
      ? `examples/${name} ${Date.now()}`
      : `examples/${name}`;
    if ((await dispatch({ command: "load_workspace_example", name, folder }))?.error == null) trackExample(name);
    fitView();
  }

  // A shared link — app.geop-cad.dev/?example=<name> — opens that example
  // once the kernel is up.
  useEffect(() => {
    if (!wasmReady) return;
    const name = new URLSearchParams(window.location.search).get("example");
    if (name && program?.examples.includes(name)) void loadExample(name);
    else if (name && program?.workspace_examples.includes(name)) void loadWorkspaceExample(name);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [wasmReady]);

  // Hosted in VS Code, the program is the document's text: it is loaded
  // from it, and every change to the program is written back. Until the
  // document has been loaded, the starting program must not be written
  // over it — and one that cannot be read is never written over either.
  // The other program files of the workspace are sent along, and again
  // whenever they change.
  const documentLoaded = useRef(false);
  useEffect(() => {
    if (!wasmReady || !host) return;
    host.onDocument((text, path) => {
      let loaded: Program;
      try {
        loaded = parseProgram(text);
      } catch (e) {
        setError(`Not a geop program: ${e}`);
        return;
      }
      void dispatch({ command: "load", program: loaded, path }).then((update) => {
        if (update && !update.error) {
          // Framed when it opens; the host sends it again on every outside
          // edit, which keeps the view.
          if (!documentLoaded.current) fitView();
          documentLoaded.current = true;
        }
      });
    });
    host.onFiles((files) => void dispatch({ command: "files", files }));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [wasmReady]);
  const programValue = program?.program;
  useEffect(() => {
    if (host && documentLoaded.current && programValue) host.programChanged(programValue);
  }, [programValue]);

  const infos = program?.operations ?? [];
  const stepCount = steps.length;
  const triangles = (view: PartView | undefined) => view?.faces.reduce((n, f) => n + f.triangles.length, 0) ?? 0;
  const triangleCount =
    triangles(scene?.part) +
    instances.reduce((n, i) => n + triangles(components[i.component]), 0);
  const timelineSteps: TimelineStep[] = steps.map((s) => ({
    id: s.id,
    title: `${s.label}: ${s.summary}`,
    error: s.error,
    dim: !s.editing && !s.runs,
    editing: s.editing,
  }));

  const dragTool = program?.drag_tool ?? false;
  const tools: Tool[] = [
    ...infos.map((info) => ({
      ...info,
      active: step?.kind === info.kind && step.id == null,
      disabled: !wasmReady || (step != null && step.kind !== info.kind),
      onSelect: () => newStep(info),
    })),
    {
      kind: "drag",
      label: "Drag",
      doc: "Drag placed parts as far as their mates let them",
      group: "Assembly",
      primary: false,
      active: dragTool,
      disabled: !wasmReady || step != null,
      onSelect: () => void dispatch({ command: "drag_tool", on: !dragTool }),
    },
  ];

  const programPanel = (
    <section className="panel timeline">
      <h2>Parameters</h2>
      <ParametersPanel
        parameters={program?.program.parameters ?? {}}
        resolved={program?.parameters ?? { values: {}, errors: {} }}
        enabled={wasmReady}
        materials={program?.materials ?? []}
        uses={program?.parameter_uses ?? {}}
        onChange={(parameters) => dispatch({ command: "parameters", parameters })}
        onRename={(from, to) => dispatch({ command: "rename_parameter", from, to })}
      />
      {(program?.joints.length ?? 0) > 0 && (
        <>
          <h2>Joints</h2>
          <JointsPanel
            joints={program?.joints ?? []}
            freedom={program?.freedom ?? null}
            enabled={wasmReady && step == null}
            onSet={(parameter, value) => dispatch({ command: "joint", parameter, value })}
          />
        </>
      )}
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

  /** Ask a question of the part as drawn; the answer replaces the last. */
  async function inspect(query: Query) {
    setInspecting(true);
    await dispatch({ command: "inspect", query });
    setInspecting(false);
  }

  const structurePanel = (
    <>
      <section className="panel structure-panel">
        <h2>Part</h2>
        <StructurePanel
          items={scene?.structure ?? []}
          enabled={wasmReady}
          onVisibility={(name, visible) => dispatch({ command: "visibility", name, visible })}
        />
      </section>
      <section className="panel inspect-panel">
        <h2>Inspect</h2>
        <InspectPanel
          enabled={wasmReady && step == null}
          measuring={program?.measure_tool ?? false}
          onMeasure={(on) => dispatch({ command: "measure_tool", on })}
          measurement={measurement}
          mass={mass}
          interference={interference}
          bom={bom}
          busy={inspecting}
          onQuery={(query) => void inspect(query)}
          onExportBom={(structure) => void exportBom(structure)}
          section={section}
          onSection={setSection}
          size={scene?.part.extent.size ?? 1}
          onLight={setLights}
        />
      </section>
    </>
  );

  const explorer = !host && (
    <Explorer
      workspace={workspace}
      enabled={wasmReady && step == null}
      onOpen={(path) => void openFile(path)}
      onCreate={(path) => void createFile(path)}
      onRename={(from, to) => void renameFile(from, to)}
      onDelete={(paths) => void deleteFiles(paths)}
      onUpload={(files) => void uploadFiles(files)}
      onDownload={downloadFile}
    />
  );

  const detailPanel = step && (
    <DialogView
      step={step}
      onDialog={(key: string, value: Value) => event({ type: "dialog", key, value })}
      setPreview={(preview) => dispatch({ command: "preview", preview })}
      error={error}
      onCommit={() => void close({ command: "commit" })}
      onCancel={() => void close({ command: "cancel" })}
      onDownload={(format) => void exportDrawing(format)}
    />
  );

  return (
    <div ref={appRef} className="app" aria-busy={!wasmReady || pending > 0}>
      <Toolbar
        busy={!wasmReady}
        hosted={host != null}
        hasSteps={stepCount > 0}
        onSave={() => downloadFile(workspace.active)}
        onExportUrdf={() => void exportUrdf()}
        onExportStep={() => void exportStep()}
        onExportFlatPattern={() => void exportFlatPattern()}
        onExportStl={() => void exportStl()}
        onLoadFile={(file) => void uploadFiles([file])}
        exampleNames={program?.examples ?? []}
        onLoadExample={(name) => void loadExample(name)}
        workspaceExampleNames={host ? [] : (program?.workspace_examples ?? [])}
        onLoadWorkspaceExample={(name) => void loadWorkspaceExample(name)}
        canUndo={step ? step.can_undo : (program?.can_undo ?? false)}
        onUndo={() => dispatch({ command: "undo" })}
        canRedo={step ? step.can_redo : (program?.can_redo ?? false)}
        onRedo={() => dispatch({ command: "redo" })}
        operationRibbon={<OperationRibbon tools={tools} />}
        badge={step && plane ? `${step.label}: working in its plane` : null}
        error={error}
        program={program?.program ?? { steps: [] }}
        stepCount={stepCount}
        triangleCount={triangleCount}
        bugReportOpen={bugReportOpen}
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
        {explorer && <aside className="explorer-bar desktop-only">{explorer}</aside>}
        <aside className="sidebar desktop-only">
          {programPanel}
          {structurePanel}
        </aside>
        <div className="editor-area">
          {!host && (
            <div className="editor-tabs desktop-only">
              <div className="editor-tab active" title={workspace.active}>
                <span className="editor-tab-name">{baseName(workspace.active)}</span>
                {workspace.active.includes("/") && (
                  <span className="editor-tab-folder">{workspace.active.slice(0, workspace.active.lastIndexOf("/"))}</span>
                )}
              </div>
            </div>
          )}
          <main className="viewport">
            {wasmError && <p className="error">Failed to load wasm: {wasmError}</p>}
            {!wasmError && !wasmReady && <p className="status">Loading geop wasm module…</p>}
            {error && !step && <p className="error">{error}</p>}
            {wasmReady && scene && (
              <SceneViewer
                part={scene.part}
                instances={instances}
                components={components}
                visuals={presentation?.visuals}
                gizmo={presentation?.gizmo ?? null}
                highlights={[...(presentation?.highlights ?? []), ...lights]}
                section={section && sectionPlane(section, scene.part.extent.center)}
                pickable={presentation?.pickable}
                hidden={scene.hidden}
                plane={plane}
                grid={presentation?.sheet == null}
                grab={presentation?.grab ?? false}
                prompt={presentation?.prompt ?? null}
                onPrompt={(key, text) => event({ type: "dialog", key, value: { type: "text", value: text } })}
                onPromptCancel={() => event({ type: "key", key: "Escape" })}
                onPointer={async (e) => {
                  const update = await dispatch({ command: "event", event: e });
                  return (update?.step?.presentation ?? update?.tool)?.grab ?? false;
                }}
                projection={projection}
                focus={focus}
                fit={fit}
                onFocusReached={(pose) => {
                  poseRef.current = pose;
                  setFocus(null);
                }}
                onPose={(pose) => (poseRef.current = pose)}
              />
            )}
            <button className="fit-view" title="Frame the whole part in the view (F)" onClick={() => fitView()}>
              Fit
            </button>
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
      </div>
      <MobileBottom
        tab={mobileTab}
        onTab={setMobileTab}
        detailAvailable={step != null}
        bugReportOpen={bugReportOpen}
        operationButtons={<OperationGrid tools={tools} />}
        programPanel={programPanel}
        structurePanel={structurePanel}
        detailPanel={detailPanel}
        filesPanel={explorer || null}
        onBugReportHost={setBugReportHost}
      />
    </div>
  );
}

export default App;
