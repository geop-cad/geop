import { useEffect, useState } from "react";

/** Whether the viewport is at or below the mobile layout breakpoint (see .mobile-bottom in App.css). */
export function useIsMobile(breakpointPx = 860): boolean {
  const query = `(max-width: ${breakpointPx}px)`;
  const [isMobile, setIsMobile] = useState(() => window.matchMedia(query).matches);
  useEffect(() => {
    const mql = window.matchMedia(query);
    const onChange = () => setIsMobile(mql.matches);
    mql.addEventListener("change", onChange);
    return () => mql.removeEventListener("change", onChange);
  }, [query]);
  return isMobile;
}
