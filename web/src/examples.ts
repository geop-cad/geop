// How the examples the kernel has are offered.

/** `name`, as an example is offered: `box_with_drill_hole` reads "Box with drill hole". */
export function title(name: string): string {
  const words = name.replace(/_/g, " ");
  return words.charAt(0).toUpperCase() + words.slice(1);
}

/** The examples offered first, where the kernel has them: those the homepage shows. */
export const FEATURED = ["turbine_blade", "cool_flange", "press", "edge_part"];
