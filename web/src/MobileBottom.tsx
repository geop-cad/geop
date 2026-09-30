import type { ReactNode } from "react";

export type MobileTab = "buttons" | "program" | "detail" | "bug";

interface Props {
  tab: MobileTab;
  onTab: (tab: MobileTab) => void;
  detailAvailable: boolean;
  bugReportOpen: boolean;
  operationButtons: ReactNode;
  programPanel: ReactNode;
  detailPanel: ReactNode;
  /**
   * The "Bug" pane's DOM node, once mounted (or null once unmounted) — the
   * bug-report form (opened from the toolbar's "Report bug" button)
   * portals into it. Kept mounted while the form is open, not just while
   * this tab is the active one, so the portal target stays stable across
   * tab switches — only hidden, via the `hidden` attribute, when inactive.
   */
  onBugReportHost: (el: HTMLDivElement | null) => void;
}

/**
 * The bottom half of the mobile layout: a tab bar switching between the
 * operation buttons, the program timeline, the dialog of the step being
 * edited, and (while open) the bug-report form — the 3-D view stays up
 * top; only these panels move down here. Hidden entirely above the
 * mobile breakpoint (see .mobile-bottom in App.css).
 */
export function MobileBottom({
  tab,
  onTab,
  detailAvailable,
  bugReportOpen,
  operationButtons,
  programPanel,
  detailPanel,
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
        {bugReportOpen && <div className="panel-host" hidden={tab !== "bug"} ref={onBugReportHost} />}
      </div>
    </div>
  );
}
