// Which text a drag may select. The app's chrome — buttons, labels, tab
// bars — is never selectable (`user-select: none` in App.css). Beyond that,
// a drag selects text only inside the dialog or panel it started in: one
// that starts on the 3-D view, where it turns the camera or drags a handle,
// selects nothing at all, wherever it goes; one that starts in a dialog's
// field or the program panel selects there, and not in what it crosses on
// the way.

/** What a drag that starts inside it may select text within. */
const SCOPES = "input, textarea, .popup, .modal, .panel";

/**
 * Scope every drag's text selection in `app`, as above: while a pointer is
 * down, `app` is `.selecting` — nothing selectable — but for the scope it
 * went down in, `.select-scope`. Returns what undoes it.
 */
export function scopeTextSelection(app: HTMLElement): () => void {
  let scope: Element | null = null;
  const end = () => {
    app.classList.remove("selecting");
    scope?.classList.remove("select-scope");
    scope = null;
  };
  const start = (e: PointerEvent) => {
    end();
    const target = e.target instanceof Element ? e.target : null;
    scope = target?.closest(SCOPES) ?? null;
    scope?.classList.add("select-scope");
    app.classList.add("selecting");
  };
  // In the capture phase, so it holds before anything handles the press —
  // and the release, even where a handler stops it.
  window.addEventListener("pointerdown", start, true);
  window.addEventListener("pointerup", end, true);
  window.addEventListener("pointercancel", end, true);
  return () => {
    end();
    window.removeEventListener("pointerdown", start, true);
    window.removeEventListener("pointerup", end, true);
    window.removeEventListener("pointercancel", end, true);
  };
}
