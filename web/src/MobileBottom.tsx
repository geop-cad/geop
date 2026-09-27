import type { ReactNode } from "react";

export type MobileTab = "buttons" | "program" | "detail" | "draw" | "bug";

interface Props {
  tab: MobileTab;
  onTab: (tab: MobileTab) => void;
  detailAvailable: boolean;
  sketchAvailable: boolean;
  bugReportOpen: boolean;
  operationButtons: ReactNode;
  programPanel: ReactNode;
  detailPanel: ReactNode;
  /**
   * The "Draw" pane's DOM node, once mounted (or null once unmounted) —
   * the sketch editor's Draw/Constrain/Status/Constraints panel portals
   * into it from wherever the 3-D view (and its canvas) actually is. Kept
   * mounted for as long as sketching is possible, not just while this tab
   * is the active one, so the portal target stays stable across tab
   * switches — only hidden, via the `hidden` attribute, when inactive.
   */
  onSketchPanelHost: (el: HTMLDivElement | null) => void;
  /** Same idea as onSketchPanelHost, for the bug-report form (opened from the toolbar's "Report bug" button). */
  onBugReportHost: (el: HTMLDivElement | null) => void;
}

/**
 * The bottom half of the mobile layout: a tab bar switching between the
 * operation buttons, the program timeline, the open step's detail form,
 * (while sketching) the sketch editor's control panel, and (while open)
 * the bug-report form — the drawing surface itself stays in the 3-D view
 * up top; only these panels move down here. Hidden entirely above the
 * mobile breakpoint (see .mobile-bottom in App.css).
 */
export function MobileBottom({
  tab,
  onTab,
  detailAvailable,
  sketchAvailable,
  bugReportOpen,
  operationButtons,
  programPanel,
  detailPanel,
  onSketchPanelHost,
  onBugReportHost,
}: Props) {
  return (
    <div className="mobile-bottom">
      <nav className="mobile-tabbar">
        <button className={tab === "buttons" ? "active" : ""} onClick={() => onTab("buttons")}>
          Operations
        </button>
        <button className={tab === "program" ? "active" : ""} onClick={() => onTab("program")}>
          Program
        </button>
        {detailAvailable && (
          <button className={tab === "detail" ? "active" : ""} onClick={() => onTab("detail")}>
            Edit
          </button>
        )}
        {sketchAvailable && (
          <button className={tab === "draw" ? "active" : ""} onClick={() => onTab("draw")}>
            Draw
          </button>
        )}
        {bugReportOpen && (
          <button className={tab === "bug" ? "active" : ""} onClick={() => onTab("bug")}>
            Bug
          </button>
        )}
      </nav>
      <div className="mobile-tab-content">
        {tab === "buttons" && operationButtons}
        {tab === "program" && programPanel}
        {tab === "detail" && detailPanel}
        {sketchAvailable && <div className="sketch-panel-host" hidden={tab !== "draw"} ref={onSketchPanelHost} />}
        {bugReportOpen && <div className="sketch-panel-host" hidden={tab !== "bug"} ref={onBugReportHost} />}
      </div>
    </div>
  );
}
