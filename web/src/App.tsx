import { useEffect, useMemo, useRef, useState } from "react";
import "./App.css";
import {
  loadGeop,
  runTool,
  send,
  type Command,
  type EditEvent,
  type PickedElement,
  type Lighting,
  type Inspection,
  type OperationInfo,
  type PartView,
  type Presentation,
  type Program,
  type ProgramState,
  type StepState,
  type Update,
  type Value,
} from "./geop";
import { host, onRestart, openFolder, store } from "./backend";
import { applyScene, NOTHING_PLACED, type DrawnScene, type Placed } from "./kept";
import { KernelCrashed } from "./crash";
import { DEFAULT_POSE, headOnPose, wantsFraming, type CameraPose, type Drawn, type Projection } from "./camera";
import { DialogView } from "./DialogView";
import { Explorer } from "./Explorer";
import { Welcome } from "./Welcome";
import {
  freeFolder,
  freePath,
  loadWorkspace,
  parseProgram,
  programText,
  type Workspace,
  dataExtensions,
  fileContent,
  isData,
  isProgram,
  programName,
} from "./files";
import { InspectPanel } from "./InspectPanel";
import { sectionPlane, type Section } from "./section";
import { ProgramPanels } from "./ProgramPanels";
import { suppressStep } from "./panels";
import { SceneViewer } from "./SceneViewer";
import { StructurePanel } from "./StructurePanel";
import { OperationGrid, OperationRibbon, type Tool } from "./OperationRibbon";
import { Timeline, type TimelineStep } from "./Timeline";
import { Toolbar } from "./Toolbar";
import { trackEdit, trackExample, trackFailures, trackFile } from "./analytics";
import { MobileBottom, type MobileTab } from "./MobileBottom";
import { useIsMobile } from "./useIsMobile";
import { assistantFits, docked, useWindowWidth } from "./layout";
import { scopeTextSelection } from "./selection";
import { AssistantPanel } from "./Assistant";
import { ASSISTANT_CONFIGURED, useAssistant } from "./useAssistant";
import type { ToolResult } from "./agent";
import { CommandPalette, ShortcutsOverlay } from "./CommandPalette";
import { matches, parseChord, pressed, withShortcut, type PaletteEntry } from "./shortcuts";

/** What the pointer sends as it moves: answered as often as it moves, never "busy". */
function following(command: Command): boolean {
  return (
    command.command === "light" ||
    (command.command === "event" && ["hover", "leave", "drag", "view"].includes(command.event.type))
  );
}

/** Whether a key press belongs to a field being typed into rather than to the step. */
function typing(e: KeyboardEvent): boolean {
  const t = e.target;
  return t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement || t instanceof HTMLSelectElement;
}

/** The commands that change the program, for statistics. */
/** Where it is kept that the assistant was closed, to stay closed. */
const ASSISTANT_CLOSED = "geop.assistantClosed";

const EDITS: Command["command"][] = ["commit", "remove", "move", "paste", "load", "load_example", "undo", "redo"];

/** Download `data` as the file `name`, of the media type `type`. */
function download(name: string, data: string | Uint8Array<ArrayBuffer>, type = "application/json") {
  const url = URL.createObjectURL(new Blob([data], { type }));
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  URL.revokeObjectURL(url);
}

/** `next`, unless it is what `shown` is already: then `shown`, so that what draws it does not again. */
function same<T>(shown: T, next: T): T {
  return shown === next || JSON.stringify(shown) === JSON.stringify(next) ? shown : next;
}

/** The last segment of `path`. */
function baseName(path: string): string {
  return path.slice(path.lastIndexOf("/") + 1);
}

/**
 * The editor: a view of the one the kernel runs (see `geop_core_editor::editor`).
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
  const [scene, setScene] = useState<DrawnScene | null>(null);
  /**
   * The placed parts drawn, and the views of their components: the kernel
   * sends what changed (see [[applyScene]]). Kept in a ref too, so that
   * updates answered one after another each apply to the one before.
   */
  const [placed, setPlaced] = useState<Placed>(NOTHING_PLACED);
  const placedRef = useRef(placed);
  const instances = useMemo(() => [...placed.instances.values()], [placed]);
  const components = placed.components;
  const [step, setStep] = useState<StepState | null>(null);
  /** What the drag tool shows, while it is in hand and no step is edited. */
  const [tool, setTool] = useState<Presentation | null>(null);
  /** Why the last command was refused, if it was. */
  const [error, setError] = useState<string | null>(null);
  /** The pick tool's answer about its picks, while it is in hand. */
  const [picked, setPicked] = useState<Inspection | null>(null);
  /** The last answers of the inspectors asked of the whole part, by kind — until the part changes. */
  const [answers, setAnswers] = useState<Record<string, Inspection>>({});
  const [inspecting, setInspecting] = useState(false);
  /** Whether the welcome was closed: it stays closed for this page. */
  const [welcomeClosed, setWelcomeClosed] = useState(false);
  /** How much of the view's bottom the welcome covers, in pixels: the view is centred above it. */
  const [covered, setCovered] = useState(0);
  /** The section view, if one is on: the view only, never the model. */
  const [section, setSection] = useState<Section | null>(null);
  /** What the kernel says is lit: kept until it sends it again. */
  const [lit, setLit] = useState<Lighting>({});

  // A drag selects text only within the dialog or panel it starts in (see selection.ts).
  const appRef = useRef<HTMLDivElement>(null);
  useEffect(() => (appRef.current ? scopeTextSelection(appRef.current) : undefined), []);
  /** Which panel the bottom half shows on a narrow (mobile) screen — irrelevant on desktop, where all three show at once. */
  const [mobileTab, setMobileTab] = useState<MobileTab>("buttons");
  const isMobile = useIsMobile();
  /** The mobile "Bug" tab's pane, once mounted — where the bug-report form portals to when open. */
  const [bugReportHost, setBugReportHost] = useState<HTMLDivElement | null>(null);
  const [bugReportOpen, setBugReportOpen] = useState(false);

  /** The program files, kept where the backend keeps them (`store`); unused in VS Code, whose workspace they are. */
  const [workspace, setWorkspace] = useState<Workspace>(() => loadWorkspace(store));
  const workspaceRef = useRef(workspace);
  workspaceRef.current = workspace;
  /**
   * Whether the kernel is editing one of the files yet. Until it is, what
   * it shows is its own empty starting program, which must not be written
   * over any file.
   */
  const editingFile = useRef(false);
  useEffect(() => {
    if (!host) store.save(workspace);
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
  // Orthographic, as drawings are: what is parallel stays parallel, and a
  // size reads the same near and far.
  const [projection, setProjection] = useState<Projection>("orthographic");
  const poseRef = useRef<CameraPose>(DEFAULT_POSE);
  /** How many commands wait for the kernel's answer: the app is busy (`aria-busy`) while any do. */
  const [pending, setPending] = useState(0);
  /** Where the camera was before it turned to face the plane being worked in, to come back to. */
  const beforePlaneRef = useRef<CameraPose | null>(null);

  /**
   * Show `update`, the kernel's answer to a command of the kind `kind` — or
   * to one an agent's tool sent (`agent`), whose errors and answers are the
   * assistant's to show; `moved`, to the pointer moving, which leaves an
   * error shown before as it is.
   */
  function show(update: Update, kind: Command["command"] | "agent", moved = false) {
    if (update.program) {
      setProgram(update.program);
      trackFailures(update.program.steps);
    }
    // A placed part's step committed: its file is kept where files are.
    for (const file of update.written ?? []) {
      if (host) host.fileChanged(file.path, file.program);
      else setWorkspace((w) => ({ ...w, files: { ...w.files, [file.path]: programText(file.program) } }));
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
      const applied = applyScene(placedRef.current, update.scene);
      setScene(applied.scene);
      // The answers were about the part as it was.
      setAnswers({});
      placedRef.current = applied.placed;
      setPlaced(placedRef.current);
      const drawn = {
        extent: applied.scene.part.extent,
        solids: applied.scene.part.solids.length + placedRef.current.instances.size,
      };
      if (drawnRef.current && wantsFraming(drawnRef.current, drawn, poseRef.current)) frameRef.current = true;
      drawnRef.current = drawn;
    }
    // The same step or tool shown as before — a hover over nothing — is
    // kept as it was: nothing it is drawn by renders again.
    setStep((shown) => same(shown, update.step));
    setTool((shown) => same(shown, update.tool));
    if (update.lit) setLit(update.lit);
    // How many updates were shown: what the latency checks (`e2e/perf.mjs`) wait for.
    const counted = window as unknown as { geopUpdates?: number };
    counted.geopUpdates = (counted.geopUpdates ?? 0) + 1;
    if (kind === "agent") return;
    // An answer asked for is kept; the pick tool's comes with every update.
    const inspection = update.inspection;
    if (kind === "inspect") {
      if (inspection) setAnswers((answers) => ({ ...answers, [inspection.kind]: inspection }));
    } else setPicked(inspection);
    // The pointer moving clears no error the user has not read yet.
    if (!moved || update.error) setError(update.error);
  }

  /** Send `command` to the kernel, and show what comes back. */
  async function dispatch(command: Command): Promise<Update | null> {
    // The pointer's moves are answered within a frame or two: shown as
    // busy, every hover would flash the spinner.
    const busy = !following(command);
    if (busy) setPending((n) => n + 1);
    try {
      const update = await send(command);
      show(update, command.command, !busy);
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
      if (busy) setPending((n) => n - 1);
    }
  }

  /** Run the assistant's tool `name` on the kernel, showing every update it causes as they come. */
  async function agentTool(name: string, input: unknown): Promise<ToolResult> {
    setPending((n) => n + 1);
    try {
      const answer = await runTool(name, input);
      for (const update of answer.updates) show(update, "agent");
      return { text: answer.text, isError: answer.is_error };
    } catch (e) {
      if (e instanceof KernelCrashed) {
        await dispatch({ command: "show" });
        return {
          text: `${name} crashed the kernel (${e.message}); it was restarted with the program as it was before the call`,
          isError: true,
        };
      }
      return { text: `${name} failed: ${String(e)}`, isError: true };
    } finally {
      setPending((n) => n - 1);
    }
  }

  // Shown by default where the assistant is offered and the window has room
  // to dock it beside the view (see `layout.ts`), until closed: the session
  // (and the check that one is human) starts with the first message, so a
  // visitor who never asks costs nothing.
  const [assistantOpen, setAssistantOpenState] = useState(() => {
    if (!ASSISTANT_CONFIGURED || !assistantFits(window.innerWidth)) return false;
    try {
      return localStorage.getItem(ASSISTANT_CLOSED) !== "1";
    } catch {
      return true;
    }
  });
  const setAssistantOpen = (update: (open: boolean) => boolean) =>
    setAssistantOpenState((open) => {
      const next = update(open);
      try {
        if (next) localStorage.removeItem(ASSISTANT_CLOSED);
        else localStorage.setItem(ASSISTANT_CLOSED, "1");
      } catch {
        // Without storage it is shown again next time.
      }
      return next;
    });
  const dock = docked(useWindowWidth(), assistantOpen);
  const assistant = useAssistant({
    program: program?.program ?? null,
    idle: wasmReady && step == null,
    runTool: agentTool,
    restore: async (before) => {
      await dispatch({ command: "load", program: before });
    },
  });

  // Any command, run as the app runs it, and what is lit now: for the
  // end-to-end checks (`e2e/`) and the browser's console.
  useEffect(() => {
    (window as unknown as { geopCommand: typeof dispatch }).geopCommand = dispatch;
    (window as unknown as { geopLit: Lighting }).geopLit = lit;
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


  /** Copy the step `id` to the clipboard, as text a paste inserts. */
  async function copyStep(id: string) {
    const text = (await dispatch({ command: "copy", id }))?.export?.text;
    if (text == null) return;
    try {
      await navigator.clipboard.writeText(text);
    } catch (e) {
      // Where the page may not write the clipboard itself (some webviews),
      // the browser's own copy of a selection still can.
      const field = document.createElement("textarea");
      field.value = text;
      field.style.position = "fixed";
      field.style.opacity = "0";
      document.body.append(field);
      field.select();
      const copied = document.execCommand("copy");
      field.remove();
      if (!copied) setError(`copying the step ${id}: the clipboard cannot be written (${String(e)})`);
    }
  }

  // Ctrl+C copies the step being edited, unless text is selected or typed
  // into a field; Ctrl+V pastes steps copied — here, or in another file —
  // where new steps go. Only a paste the keys asked for: a middle click
  // pastes the primary selection on Linux, and that is never steps.
  const copyable = step?.id ?? null;
  useEffect(() => {
    let keyedPaste = false;
    const onKey = (e: KeyboardEvent) => {
      if (typing(e) || !(e.ctrlKey || e.metaKey)) return;
      const key = e.key.toLowerCase();
      if (key === "v") keyedPaste = true;
      if (key !== "c") return;
      if (copyable == null || !(window.getSelection()?.isCollapsed ?? true)) return;
      e.preventDefault();
      void copyStep(copyable);
    };
    const onPaste = (e: ClipboardEvent) => {
      const keyed = keyedPaste;
      keyedPaste = false;
      const t = e.target;
      if (!keyed || t instanceof HTMLInputElement || t instanceof HTMLTextAreaElement) return;
      const text = e.clipboardData?.getData("text/plain");
      if (!text) return;
      e.preventDefault();
      void dispatch({ command: "paste", text });
    };
    window.addEventListener("keydown", onKey);
    window.addEventListener("paste", onPaste);
    return () => {
      window.removeEventListener("keydown", onKey);
      window.removeEventListener("paste", onPaste);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [copyable]);

  /**
   * The part placed in the program that new steps go into, built in context
   * against the parts placed beside it — none, for the program itself. Gone
   * with the file, or the part.
   */
  const [context, setContext] = useState<string | null>(null);
  const placedNames = (program?.steps ?? []).flatMap((s) => s.placed.map((p) => p.instance)).join("\n");
  useEffect(() => {
    if (context != null && !placedNames.split("\n").includes(context)) setContext(null);
  }, [context, placedNames]);

  /** Suppress the step `id` in the configuration built, or run it again. */
  function suppress(id: string) {
    const { key, payload } = suppressStep(id);
    return dispatch({ command: "panel", key, payload });
  }

  /** The step under the pointer in the timeline, which Ctrl+E suppresses with no step edited. */
  const hoveredRef = useRef<string | null>(null);

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

  const editing = step != null;
  /** Whether the command palette, or the list of keys, is open. */
  const [palette, setPalette] = useState<"palette" | "keys" | null>(null);

  function newStep(info: OperationInfo) {
    if (step?.kind === info.kind && step.id == null) void close({ command: "cancel" });
    else if (context != null) void open({ command: "new_in", kind: info.kind, placed: context });
    else void open({ command: "new", kind: info.kind });
  }

  // ── the program ────────────────────────────────────────────────────────

  const steps = useMemo(() => program?.steps ?? [], [program?.steps]);
  /** The extensions of the data files programs read beside them (see `OperationInfo.reads`). */
  const reads = useMemo(() => dataExtensions(program?.operations ?? []), [program?.operations]);

  // ── the files (in the browser) ─────────────────────────────────────────

  /** Edit the file `path`, whose program is `text` — framed in the view, unless `keepView`. A STEP file is data, not opened. */
  async function openFile(path: string, text = workspaceRef.current.files[path], keepView = false) {
    if (!isProgram(path)) {
      const reader = program?.operations.find((o) => isData(path, o.reads));
      setError(`${path} is no program${reader ? `: read it into one with "${reader.label}"` : ""}`);
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
      if (isData(file.name, reads)) {
        // Data for an import: kept as it is, next to the programs — bytes
        // that are not text as a `data:` URL.
        const text = fileContent(new Uint8Array(await file.arrayBuffer()));
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
    const opened = paths.find(isProgram);
    if (opened) await openFile(opened, added[opened]);
    trackFile("loaded");
  }

  /**
   * Store `file`, chosen from disk for a file field, next to the program
   * edited — under another name if one of its name there holds something
   * else — and give the kernel it: its path relative to the program.
   */
  async function chooseFile(file: File): Promise<string | null> {
    try {
      const text = fileContent(new Uint8Array(await file.arrayBuffer()));
      if (host) return await host.addFile(file.name, text);
      const { files, active } = workspaceRef.current;
      const folder = active.slice(0, active.lastIndexOf("/") + 1);
      const wanted = folder + file.name;
      const path = files[wanted] == null || files[wanted] === text ? wanted : freePath(files, wanted);
      await changeFiles({ [path]: text });
      trackFile("loaded");
      return path.slice(folder.length);
    } catch (e) {
      setError(`${file.name} could not be added: ${e instanceof Error ? e.message : e}`);
      return null;
    }
  }

  /**
   * Write the part shown with the exporter `exporter` — in `format`, else
   * its first; told `options` and dated today — and save it: downloaded,
   * or through VS Code.
   */
  async function exportWith(exporter: string, format?: string, options?: unknown) {
    const date = new Date().toISOString().slice(0, 10);
    const update = await dispatch({ command: "export", exporter, format, options, date });
    const file = update?.export;
    if (!file) return;
    if (host) host.saveFile(file);
    else if (file.bytes != null) download(file.name, Uint8Array.from(atob(file.bytes), (c) => c.charCodeAt(0)), file.media_type);
    else download(file.name, file.text ?? "", file.media_type);
    trackFile("saved");
  }

  function downloadFile(path: string) {
    const text = workspaceRef.current.files[path];
    if (text == null) return;
    download(baseName(path), text, isProgram(path) ? undefined : "application/octet-stream");
    trackFile("saved");
  }

  async function loadExample(name: string) {
    // In the browser an example gets a file of its own, rather than
    // replacing the one edited.
    if (!host) await createFile(freePath(workspaceRef.current.files, `${name}.geop`));
    if ((await dispatch({ command: "load_example", name }))?.error == null) trackExample(name);
    fitView();
  }

  async function loadWorkspaceExample(name: string) {
    const folder = freeFolder(workspaceRef.current.files, `examples/${name}`);
    if ((await dispatch({ command: "load_workspace_example", name, folder }))?.error == null) trackExample(name);
    fitView();
  }

  // A shared link — app.geop-cad.dev/?example=<name> — opens that example
  // once the kernel is up.
  useEffect(() => {
    if (!wasmReady) return;
    const params = new URLSearchParams(window.location.search);
    const name = params.get("example");
    // Opened once: a reload keeps the files made, rather than making them again.
    if (name) {
      params.delete("example");
      const query = params.toString();
      window.history.replaceState(null, "", `${window.location.pathname}${query ? `?${query}` : ""}${window.location.hash}`);
    }
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

  const infos = useMemo(() => program?.operations ?? [], [program?.operations]);
  /** Reduced mode, on unless turned off in the Help menu: only the
   *  operations that are robust today, the experimental ones left out.
   *  Remembered per browser. */
  const [reducedMode, setReducedModeState] = useState(() => {
    try {
      return localStorage.getItem("geop.reducedMode") !== "off";
    } catch {
      return true;
    }
  });
  const setReducedMode = (on: boolean) => {
    setReducedModeState(on);
    try {
      localStorage.setItem("geop.reducedMode", on ? "on" : "off");
    } catch {
      // Not remembered, then: private windows may refuse storage.
    }
  };
  const stepCount = steps.length;
  const triangles = (view: PartView | undefined) => view?.faces.reduce((n, f) => n + f.triangles.length, 0) ?? 0;
  const triangleCount =
    triangles(scene?.part) +
    instances.reduce((n, i) => n + triangles(components[i.component]), 0);
  // Made again only when the program is, not at every hover.
  const timelineSteps: TimelineStep[] = useMemo(() => {
    const iconOf = (kind: string) => infos.find((info) => info.kind === kind)?.icon ?? null;
    return steps.map((s) => ({
      id: s.id,
      name: s.id,
      icon: iconOf(s.kind),
      // A step placing one part says which file it places.
      title:
        s.placed.length === 1
          ? `${s.label} of ${programName(s.placed[0].file)}: ${s.summary}`
          : `${s.label}: ${s.summary}`,
      error: s.error,
      dim: !s.editing && !s.runs,
      suppressed: s.suppressed,
      editing: s.editing,
      placed: s.placed.map((p) => ({
        instance: p.instance,
        steps: p.steps.map((inner) => ({
          id: inner.id,
          name: inner.id,
          icon: iconOf(inner.kind),
          title: `${programName(p.file)}: ${inner.label}: ${inner.summary}`,
          error: inner.error,
          suppressed: inner.suppressed,
          editing: inner.editing,
        })),
      })),
    }));
  }, [steps, infos]);

  const inHand = program?.tool ?? null;
  // The ribbon offers the operations alone: the editor's tools — the drag
  // tool — are no operations, and the inspect panel offers them.
  const tools: Tool[] = infos
    .filter((info) => !reducedMode || !info.experimental)
    .map((info) => ({
      ...info,
      active: step?.kind === info.kind && step.id == null,
      disabled: !wasmReady || (step != null && step.kind !== info.kind),
      onSelect: () => newStep(info),
    }));
  /** The key of an inspector's pick tool: `pick:measure`. */
  const PICK = "pick:";

  // Everything the editor offers, as the palette lists it and the keys
  // reach it: the operations by the kernel's keys, the steps, examples,
  // tools, inspectors and exports, and the app's own actions.
  const idle = wasmReady && step == null;
  /** The key the kernel gives the app's `action` (see `geop_core_editor::keys`). */
  const appKey = (action: string) => program?.keys.find((k) => k.action === action)?.key ?? null;
  const firstSentence = (doc: string) => doc.split(/(?<=\.)\s/)[0];
  const paletteEntries: PaletteEntry[] = [
    ...tools.map((t) => ({
      id: `op:${t.kind}`,
      label: t.label,
      group: "Operation",
      detail: firstSentence(t.doc),
      shortcut: t.shortcut,
      disabled: t.disabled,
      run: t.onSelect,
    })),
    {
      id: "edit:commit",
      label: "OK: put the step into the program",
      group: "Edit",
      shortcut: appKey("commit"),
      disabled: step == null || step.missing.length > 0 || step.error != null,
      run: () => void close({ command: "commit" }),
    },
    {
      id: "edit:escape",
      label: "Stop what is in progress — a tool, a pick, a drag — else cancel the step (undo opens it again)",
      group: "Edit",
      // The step's: it decides what is in progress (see the key handler).
      shortcut: appKey("escape"),
      disabled: step == null,
      run: () => void event({ type: "key", key: "Escape" }),
    },
    {
      id: "edit:cancel",
      label: "Cancel the step, whatever is in progress",
      group: "Edit",
      shortcut: appKey("cancel"),
      disabled: step == null,
      run: () => void close({ command: "cancel" }),
    },
    {
      id: "edit:undo",
      label: "Undo",
      group: "Edit",
      shortcut: appKey("undo"),
      disabled: !(step ? step.can_undo : (program?.can_undo ?? false)),
      run: () => void dispatch({ command: "undo" }),
    },
    {
      id: "edit:redo",
      label: "Redo",
      group: "Edit",
      shortcut: appKey("redo"),
      disabled: !(step ? step.can_redo : (program?.can_redo ?? false)),
      run: () => void dispatch({ command: "redo" }),
    },
    {
      id: "edit:suppress",
      label: "Suppress the step, or run it again",
      group: "Edit",
      detail: "the step edited, or the one under the pointer in the timeline, in the configuration built",
      shortcut: "Mod+E",
      disabled: !wasmReady || (step != null && step.id == null),
      run: () => {
        const id = step?.id ?? hoveredRef.current;
        if (id != null) void suppress(id);
      },
    },
    {
      id: "edit:preview",
      label: step?.preview === false ? "Preview the step" : "Stop previewing the step",
      group: "Edit",
      disabled: step == null,
      run: () => void dispatch({ command: "preview", preview: !(step?.preview ?? true) }),
    },
    { id: "view:fit", label: "Fit the part in the view", group: "View", shortcut: appKey("fit"), disabled: !idle, run: () => fitView() },
    {
      id: "view:home",
      label: "Reset the view",
      group: "View",
      shortcut: appKey("home"),
      disabled: !idle,
      run: () => fitView(DEFAULT_POSE),
    },
    {
      id: "view:projection",
      label: projection === "perspective" ? "Orthographic view" : "Perspective view",
      group: "View",
      shortcut: appKey("projection"),
      disabled: !idle,
      run: () => setProjection(projection === "perspective" ? "orthographic" : "perspective"),
    },
    ...(program?.tools ?? []).map((t) => ({
      id: `tool:${t.key}`,
      label: inHand === t.key ? `Put down: ${t.label}` : t.label,
      group: "Tool",
      detail: firstSentence(t.doc),
      disabled: !idle,
      run: () => void dispatch({ command: "tool", key: inHand === t.key ? null : t.key }),
    })),
    ...(program?.inspectors ?? []).map((i) => ({
      id: `inspect:${i.kind}`,
      label: i.label,
      group: "Inspect",
      detail: firstSentence(i.doc),
      disabled: !idle,
      run: () =>
        void (i.picks
          ? dispatch({ command: "tool", key: PICK + i.kind })
          : inspect(i.kind).then(() => setMobileTab("part"))),
    })),
    ...steps.map((s, i) => ({
      id: `step:${s.id}`,
      label: s.id,
      group: "Step",
      detail: `${s.label}: ${s.summary}`,
      disabled: !idle,
      run: () => void open({ command: "open", id: steps[i].id }),
    })),
    ...(program?.exporters ?? []).map((x) => ({
      id: `export:${x.kind}`,
      label: `Export ${x.label}`,
      group: "Export",
      detail: x.doc,
      disabled: !idle,
      run: () => void exportWith(x.kind),
    })),
    ...(program?.examples ?? []).map((name) => ({
      id: `example:${name}`,
      label: name,
      group: "Example",
      disabled: !idle,
      run: () => void loadExample(name),
    })),
    ...(host ? [] : (program?.workspace_examples ?? [])).map((name) => ({
      id: `workspace:${name}`,
      label: name,
      group: "Example",
      detail: "several files",
      disabled: !idle,
      run: () => void loadWorkspaceExample(name),
    })),
    { id: "help:palette", label: "Command palette", group: "Help", shortcut: appKey("palette"), run: () => setPalette("palette") },
    { id: "help:keys", label: "Keyboard shortcuts", group: "Help", shortcut: appKey("keys"), run: () => setPalette("keys") },
  ];
  const entriesRef = useRef(paletteEntries);
  entriesRef.current = paletteEntries;

  // The keys: Ctrl+K (⌘K) opens the palette anywhere; then, unless typed
  // into a field, `?` lists the keys, Ctrl+Enter commits, and the
  // palette's keys run what they name — an operation's only while no
  // step is edited. Any other key goes to the step being edited (its
  // tools, Escape, Enter, Delete).
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const palettes = entriesRef.current.find((entry) => entry.id === "help:palette");
      if (palettes?.shortcut && matches(parseChord(palettes.shortcut), e)) {
        e.preventDefault();
        setPalette((open) => (open === "palette" ? null : "palette"));
        return;
      }
      if (palette != null || e.defaultPrevented) return;
      // Ctrl+Enter commits from a field too, after the field took its Enter.
      const commit = entriesRef.current.find((entry) => entry.id === "edit:commit");
      if (commit?.shortcut && !commit.disabled && matches(parseChord(commit.shortcut), e)) {
        e.preventDefault();
        commit.run();
        return;
      }
      if (typing(e)) return;
      // While a step is edited, its tools take the plain keys.
      const usable = entriesRef.current.filter(
        (entry) => !editing || entry.shortcut?.startsWith("Mod+") || entry.id === "help:keys",
      );
      const entry = pressed(usable, e);
      if (entry) {
        e.preventDefault();
        entry.run();
        return;
      }
      if (matches(parseChord("Mod+Y"), e)) {
        e.preventDefault();
        void dispatch({ command: "redo" });
        return;
      }
      if (editing && !e.ctrlKey && !e.metaKey && !e.altKey) void event({ type: "key", key: e.key });
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [editing, palette]);

  const programPanel = (
    <section className="panel timeline">
      <ProgramPanels
        panels={program?.panels ?? {}}
        enabled={wasmReady}
        idle={step == null}
        onCommand={(key, payload) => dispatch({ command: "panel", key, payload })}
        onOpenStep={(id) => void open({ command: "open", id })}
      />
      <h2>Program</h2>
      <Timeline
        steps={timelineSteps}
        seeker={program?.marker ?? stepCount}
        enabled={wasmReady && step == null}
        onEdit={(i) => void open({ command: "open", id: steps[i].id })}
        onEditPlaced={(placed, id) => void open({ command: "open", id, placed })}
        context={context}
        onContext={setContext}
        onRemove={(i) => dispatch({ command: "remove", id: steps[i].id })}
        onCopy={(i) => void copyStep(steps[i].id)}
        onSuppress={(i) => void suppress(steps[i].id)}
        onHover={(i) => (hoveredRef.current = i == null ? null : steps[i].id)}
        onMove={(i, to) => dispatch({ command: "move", id: steps[i].id, index: to })}
        onSeek={(slot) => dispatch({ command: "seek", marker: slot >= stepCount ? null : slot })}
      />
    </section>
  );

  /** Ask `inspector` about the part as drawn, told `options`; the answer replaces its last. */
  async function inspect(inspector: string, options?: unknown) {
    setInspecting(true);
    await dispatch({ command: "inspect", inspector, options });
    setInspecting(false);
  }

  /** Light what a list item or an answer's row points at, while it is pointed at: the kernel says what that lights. */
  const light = (elements: PickedElement[]) => void dispatch({ command: "light", elements });

  const structurePanel = (
    <>
      <section className="panel structure-panel">
        <h2>Part</h2>
        <StructurePanel
          items={scene?.structure ?? []}
          details={program?.details ?? {}}
          enabled={wasmReady}
          onVisibility={(name, visible) => dispatch({ command: "visibility", name, visible })}
          onLight={light}
        />
      </section>
      <section className="panel inspect-panel">
        <h2>Inspect</h2>
        <InspectPanel
          enabled={wasmReady && step == null}
          tools={program?.tools ?? []}
          inHand={inHand}
          onTool={(key) => void dispatch({ command: "tool", key })}
          inspectors={program?.inspectors ?? []}
          picking={inHand?.startsWith(PICK) ? inHand.slice(PICK.length) : null}
          onPicking={(inspector) => dispatch({ command: "tool", key: inspector == null ? null : PICK + inspector })}
          picked={picked}
          answers={answers}
          busy={inspecting}
          onInspect={(inspector, options) => void inspect(inspector, options)}
          onExport={(exporter, options) => void exportWith(exporter, undefined, options)}
          section={section}
          onSection={setSection}
          size={scene?.part.extent.size ?? 1}
          onLight={light}
        />
      </section>
    </>
  );

  const explorer = !host && (
    <Explorer
      workspace={workspace}
      reads={reads}
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
      onChooseFile={chooseFile}
      setPreview={(preview) => dispatch({ command: "preview", preview })}
      error={error}
      onCommit={() => void close({ command: "commit" })}
      onCancel={() => void close({ command: "cancel" })}
      onDownload={(exporter, format) => void exportWith(exporter, format)}
      onEnter={() => void event({ type: "key", key: "Enter" })}
    />
  );

  return (
    <div ref={appRef} className="app" aria-busy={!wasmReady || pending > 0}>
      <Toolbar
        busy={!wasmReady}
        hosted={host != null}
        hasSteps={stepCount > 0}
        onNewPart={() => void createFile(freePath(workspaceRef.current.files, "part.geop"))}
        onSave={() => downloadFile(workspace.active)}
        exporters={program?.exporters ?? []}
        reads={reads}
        onExport={(exporter) => void exportWith(exporter)}
        onLoadFile={(file) => void uploadFiles([file])}
        onOpenFolder={openFolder}
        exampleNames={program?.examples ?? []}
        onLoadExample={(name) => void loadExample(name)}
        workspaceExampleNames={host ? [] : (program?.workspace_examples ?? [])}
        onLoadWorkspaceExample={(name) => void loadWorkspaceExample(name)}
        undoKey={appKey("undo")}
        redoKey={appKey("redo")}
        canUndo={step ? step.can_undo : (program?.can_undo ?? false)}
        onUndo={() => dispatch({ command: "undo" })}
        canRedo={step ? step.can_redo : (program?.can_redo ?? false)}
        onRedo={() => dispatch({ command: "redo" })}
        operationRibbon={<OperationRibbon tools={tools} />}
        badge={step && plane ? `${step.label}: working in its plane` : null}
        error={error}
        program={program?.program ?? { steps: [] }}
        bugReportOpen={bugReportOpen}
        bugReportHost={isMobile ? bugReportHost : null}
        reducedMode={reducedMode}
        onReducedMode={setReducedMode}
        onBugReportOpen={() => {
          setBugReportOpen(true);
          setMobileTab("bug");
        }}
        onBugReportCollect={async () => (await dispatch({ command: "report" }))?.report ?? null}
        assistantOpen={host ? null : assistantOpen}
        onAssistant={() => setAssistantOpen((open) => !open)}
        onBugReportClose={() => {
          setBugReportOpen(false);
          setMobileTab((tab) => (tab === "bug" ? "buttons" : tab));
        }}
      />
      <div className="body">
        {explorer && dock.explorer && <aside className="explorer-bar desktop-only">{explorer}</aside>}
        <aside className="sidebar desktop-only">
          {explorer && !dock.explorer && <div className="sidebar-explorer">{explorer}</div>}
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
            {error && !step && <p className="error">{error}</p>}
            {infos.some((info) => info.tier === undefined) && (
              <p className="error">
                The kernel is older than this page: its operations do not say how the toolbar shows them. Rebuild it
                (npm run build:wasm, or the VS Code extension's npm run build) and reload.
              </p>
            )}
            {wasmReady && scene && (
              <SceneViewer
                part={scene.part}
                instances={instances}
                components={components}
                visuals={presentation?.visuals}
                gizmo={presentation?.gizmo ?? null}
                lit={lit}
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
                cycle={presentation?.cycle ?? null}
                onHome={() => fitView(DEFAULT_POSE)}
                homeKey={appKey("home")}
                covered={covered}
              />
            )}
            {wasmReady && program && stepCount === 0 && step == null && pending === 0 && !host && !welcomeClosed && (
              <Welcome
                examples={program.examples}
                workspaceExamples={program.workspace_examples}
                onExample={(name) => void loadExample(name)}
                onWorkspaceExample={(name) => void loadWorkspaceExample(name)}
                keyOf={(kind) =>
                  program.operations.find((o) => o.kind === kind)?.shortcut ?? appKey(kind)
                }
                onClose={() => setWelcomeClosed(true)}
                onCover={setCovered}
              />
            )}
            {(!wasmReady || pending > 0) && !wasmError && (
              <div className="loading" role="status" aria-label={wasmReady ? "Working" : "Loading"}>
                <span className="spinner" />
                <span>{wasmReady ? "Building…" : "Loading geop…"}</span>
              </div>
            )}
            {/* What is built, in the view's corner: the top bar's room is the operations'. */}
            {wasmReady && (
              <span className="stats" title="Steps in the program, and triangles drawn">
                <span>
                  {stepCount} step{stepCount === 1 ? "" : "s"}
                </span>
                <span className="stats-separator"> · </span>
                <span>{triangleCount} tris</span>
              </span>
            )}
            <div className="view-buttons">
              <button title={withShortcut("Frame the whole part in the view", appKey("fit"))} onClick={() => fitView()}>
                Fit
              </button>
              <button
                title={withShortcut("Switch between a perspective and an orthographic view", appKey("projection"))}
                onClick={() => setProjection(projection === "perspective" ? "orthographic" : "perspective")}
              >
                {projection === "perspective" ? "Perspective" : "Orthographic"}
              </button>
            </div>
            <div className="desktop-only">{detailPanel}</div>
          </main>
        </div>
        {!host && assistantOpen && (
          <aside className={`assistant-bar desktop-only${dock.assistant ? "" : " assistant-over"}`}>
            <AssistantPanel assistant={assistant} onClose={() => setAssistantOpen(() => false)} />
          </aside>
        )}
      </div>
      {palette === "palette" && <CommandPalette entries={paletteEntries} onClose={() => setPalette(null)} />}
      {palette === "keys" && <ShortcutsOverlay entries={paletteEntries} onClose={() => setPalette(null)} />}
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
        assistantPanel={host ? null : <AssistantPanel assistant={assistant} />}
        onBugReportHost={setBugReportHost}
      />
    </div>
  );
}

export default App;
