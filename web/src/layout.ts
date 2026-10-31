// What the desktop layout docks beside the view, by the window's width (see
// docs/ux.md, Layout and words). The view keeps `VIEW_MIN` pixels: a panel
// docks only where it leaves the view that much, and otherwise moves out of
// the view's way — the explorer to the top of the program panel, the
// assistant over the view's right side, where the visitor opened it.
import { useEffect, useState } from "react";

/** The narrowest the view becomes beside docked panels, in pixels. */
export const VIEW_MIN = 480;
/** The panels' widths with their borders, as App.css sets them. */
const SIDEBAR = 301;
const EXPLORER = 241;
const ASSISTANT = 341;

/** What docks beside the view in a window `width` wide, the assistant open or not. */
export function docked(width: number, assistantOpen: boolean): { explorer: boolean; assistant: boolean } {
  const room = width - SIDEBAR - VIEW_MIN;
  // The assistant, opened, docks before the explorer: it is what was asked for.
  const assistant = assistantOpen && room >= ASSISTANT;
  return { explorer: room - (assistant ? ASSISTANT : 0) >= EXPLORER, assistant };
}

/** Whether the assistant starts open in a window `width` wide: only where it docks beside every other panel. */
export function assistantFits(width: number): boolean {
  const both = docked(width, true);
  return both.assistant && both.explorer;
}

/** The window's width, as it changes. */
export function useWindowWidth(): number {
  const [width, setWidth] = useState(() => window.innerWidth);
  useEffect(() => {
    const onResize = () => setWidth(window.innerWidth);
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);
  return width;
}
