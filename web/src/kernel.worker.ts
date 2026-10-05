// The kernel, in the browser: the wasm module, in a worker of its own (see
// `backend.ts`). It says once whether the module loaded, then runs one
// command at a time, as they come, and answers each with the update, or
// with why the command could not be read — or, if the kernel panicked,
// with what panicked. A panic leaves the module unusable, so the page then
// ends this worker and starts another.
import init, { handle, init_panic_hook } from "./wasm/pkg/geop.js";

/** What the page sends: a command (`geop_cad_base::Command` as JSON). */
export interface ToKernel {
  id: number;
  command: string;
}

/** What the worker says: whether it loaded, then an answer per command — the update, a command it could not read, or a crash. */
export type FromKernel =
  | { loaded: true }
  | { loaded: false; error: string }
  | { id: number; update: string }
  | { id: number; failure: string }
  | { id: number; crashed: string };

const scope = self as unknown as {
  postMessage(message: FromKernel): void;
  onmessage: ((e: MessageEvent<ToKernel>) => void) | null;
  /** Told what panicked by the module's panic hook (see `geop-cad-web`). */
  geopPanicked: (message: string) => void;
};

/** What the last panic said. */
let panicked = "";
scope.geopPanicked = (message) => {
  panicked = message;
};

const ready = init().then(() => init_panic_hook());
ready.then(
  () => scope.postMessage({ loaded: true }),
  (e) => scope.postMessage({ loaded: false, error: String(e) }),
);

// Commands that come before the module has loaded wait for it, in order.
scope.onmessage = async (e) => {
  const { id, command } = e.data;
  try {
    await ready;
    scope.postMessage({ id, update: handle(command) });
  } catch (error) {
    // `handle` throws the text of a command it could not read; anything
    // else is the module trapping — a panic, or memory run out.
    scope.postMessage(typeof error === "string" ? { id, failure: error } : { id, crashed: panicked || String(error) });
  }
};
