// How the examples the kernel has are offered.

/** `name`, as an example is offered: `box_with_drill_hole` reads "Box with drill hole". */
export function title(name: string): string {
  const words = name.replace(/_/g, " ");
  return words.charAt(0).toUpperCase() + words.slice(1);
}

/**
 * The examples offered first, where the kernel has them — those the
 * homepage shows — each with a line on what it is. Each has a preview
 * rendered from what it builds when the app is built (`npm run
 * build:previews`, see `e2e/previews.mjs`).
 */
export const FEATURED: { name: string; about: string }[] = [
  { name: "turbine_blade", about: "A blade lofted through three sections, patterned round a hub" },
  { name: "cool_flange", about: "A revolved flange, a slot cut through it sixteen times" },
  { name: "press", about: "A base, handle, link and slider, each a file, mated to move" },
  { name: "edge_part", about: "Three outlines on the origin's planes, extruded and joined" },
];

/** Where the preview of the example `name` is served: `dist/examples/<name>.webp`. */
export function preview(name: string): string {
  return `${import.meta.env.BASE_URL}examples/${name}.webp`;
}
