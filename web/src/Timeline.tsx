// The program as a list of step boxes, with the seeker: the line after
// which nothing runs. Everything is done by pointer: click a box to edit its
// step, drag it to move the step, drag the seeker to go back in time, copy
// it to paste elsewhere.
//
// Both drags work the same way: what is being dragged, and the *slot* the
// pointer is over — a position between steps, 0 (before the first) to
// `steps.length` (after the last). The drop only happens on release; until
// then the slot is only drawn.

import { useRef, useState } from "react";

/** One step as the timeline shows it. */
export interface TimelineStep {
  id: string;
  /** Shown on hover: what the step is and does. */
  title: string;
  error: string | null;
  /** Past where the program currently runs to. */
  dim: boolean;
  /** Being written by the open form. */
  editing: boolean;
}

interface Props {
  steps: TimelineStep[];
  /** The seeker's slot: how many steps run. */
  seeker: number;
  /** Whether anything can be clicked or dragged. */
  enabled: boolean;
  onEdit: (index: number) => void;
  onRemove: (index: number) => void;
  /** Copy step `index` to the clipboard, to paste with Ctrl+V. */
  onCopy: (index: number) => void;
  /** Move step `index` to `to` among the other steps. */
  onMove: (index: number, to: number) => void;
  onSeek: (slot: number) => void;
}

/** How far the pointer has to travel before a press on a box is a drag, not a click, in pixels. */
const DRAG_THRESHOLD = 4;

type Dragging = { what: "step"; index: number } | { what: "seeker" };

interface Press {
  what: Dragging;
  x: number;
  y: number;
  /** Past the threshold: a drag. */
  moved: boolean;
}

export function Timeline({ steps, seeker, enabled, onEdit, onRemove, onCopy, onMove, onSeek }: Props) {
  const listRef = useRef<HTMLOListElement>(null);
  const pressRef = useRef<Press | null>(null);
  /** The drag being drawn, and the slot it would drop into. */
  const [drag, setDrag] = useState<{ what: Dragging; slot: number } | null>(null);

  /** The slot at height `y`: after every step whose box's middle is above it. */
  function slotAt(y: number): number {
    const boxes = listRef.current?.querySelectorAll<HTMLElement>("[data-step]") ?? [];
    let slot = 0;
    for (const box of boxes) {
      const r = box.getBoundingClientRect();
      if (y > r.top + r.height / 2) slot += 1;
    }
    return slot;
  }

  function onPointerDown(e: React.PointerEvent, what: Dragging) {
    if (!enabled || e.button !== 0) return;
    e.preventDefault();
    // Captured by the list, not the row: a dragged seeker's row moves in
    // the DOM as it goes, and moving an element drops its capture.
    listRef.current?.setPointerCapture(e.pointerId);
    pressRef.current = { what, x: e.clientX, y: e.clientY, moved: what.what === "seeker" };
    if (what.what === "seeker") setDrag({ what, slot: seeker });
  }

  function onPointerMove(e: React.PointerEvent) {
    const press = pressRef.current;
    if (!press) return;
    if (!press.moved && Math.hypot(e.clientX - press.x, e.clientY - press.y) < DRAG_THRESHOLD) return;
    press.moved = true;
    setDrag({ what: press.what, slot: slotAt(e.clientY) });
  }

  function onPointerUp(e: React.PointerEvent) {
    const press = pressRef.current;
    pressRef.current = null;
    setDrag(null);
    if (!press) return;
    const what = press.what;
    if (!press.moved) {
      if (what.what === "step") onEdit(what.index);
      return;
    }
    const slot = slotAt(e.clientY);
    if (what.what === "seeker") {
      if (slot !== seeker) onSeek(slot);
      return;
    }
    // Among the other steps, the ones before the slot keep their place.
    const to = slot > what.index ? slot - 1 : slot;
    if (to !== what.index) onMove(what.index, to);
  }

  function onPointerCancel() {
    pressRef.current = null;
    setDrag(null);
  }

  const seekerSlot = drag?.what.what === "seeker" ? drag.slot : seeker;
  const dropSlot = drag?.what.what === "step" ? drag.slot : null;

  const seekerRow = (
    <li
      key="seeker"
      className={`seeker${seekerSlot < steps.length ? " rolled-back" : ""}${drag?.what.what === "seeker" ? " dragging" : ""}`}
      title="Drag to go back in time: only the steps above run, and new steps go here."
      onPointerDown={(e) => onPointerDown(e, { what: "seeker" })}
    >
      <span className="seeker-head" />
      <span className="seeker-line" />
    </li>
  );

  const rows: React.ReactNode[] = [];
  steps.forEach((step, i) => {
    if (seekerSlot === i) rows.push(seekerRow);
    if (dropSlot === i) rows.push(<li key="drop" className="drop-indicator" />);
    const dragged = drag?.what.what === "step" && drag.what.index === i;
    rows.push(
      <li
        key={step.id}
        data-step={i}
        className={[
          "step-box",
          step.error ? "op-error" : "",
          step.dim ? "step-dim" : "",
          step.editing ? "step-editing" : "",
          dragged ? "dragging" : "",
          enabled ? "" : "disabled",
        ].join(" ")}
        title={step.error ? `${step.title}\n\n${step.error}` : step.title}
        onPointerDown={(e) => onPointerDown(e, { what: "step", index: i })}
      >
        <span className="step-name">{step.id}</span>
        <button
          className="step-copy"
          disabled={!enabled}
          title="Copy this step — paste it with Ctrl+V, here or in another file"
          aria-label="Copy this step"
          onPointerDown={(e) => e.stopPropagation()}
          onClick={() => onCopy(i)}
        >
          ⧉
        </button>
        <button
          className="step-remove"
          disabled={!enabled}
          title="Delete this step"
          // Not a press on the box: no click, no drag.
          onPointerDown={(e) => e.stopPropagation()}
          onClick={() => onRemove(i)}
        >
          ✕
        </button>
      </li>,
    );
  });
  if (dropSlot === steps.length) rows.push(<li key="drop" className="drop-indicator" />);
  if (seekerSlot === steps.length) rows.push(seekerRow);

  return (
    <ol
      ref={listRef}
      className={drag ? "dragging" : ""}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
      onPointerCancel={onPointerCancel}
    >
      {rows}
      {steps.length === 0 && <li className="empty">No steps yet.</li>}
    </ol>
  );
}
