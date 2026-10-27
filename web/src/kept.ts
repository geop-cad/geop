// What a viewer keeps of the scenes the kernel sent it: the placed parts
// drawn, the views of their components and the meshes of the faces drawn,
// each sent once by key and kept until the kernel says to forget it — the
// viewer's side of `geop_core_editor::editor::kept`.
import type { FaceMesh, PartView, SceneState, SentPartView, ViewInstance } from "./geop";

/** A scene as it is drawn: its part with its faces' meshes (see [[applyScene]]). */
export type DrawnScene = Omit<SceneState, "part"> & { part: PartView };

/** The placed parts drawn, by name, the views of the components they are drawn from, by key, and the meshes of the faces drawn, by key. */
export interface Placed {
  instances: Map<string, ViewInstance>;
  components: Record<string, PartView>;
  meshes: Record<string, FaceMesh>;
}

/** Nothing placed, nothing kept. */
export const NOTHING_PLACED: Placed = { instances: new Map(), components: {}, meshes: {} };

/** `view` with each face's mesh, of `meshes`: the same mesh, not a copy, for a face drawn alike in two views. */
export function withMeshes(view: SentPartView, meshes: Record<string, FaceMesh>): PartView {
  const faces = view.faces.map(({ mesh: key, ...face }) => {
    const mesh = meshes[key];
    if (!mesh) throw new Error(`the kernel drew the face ${face.name} with the mesh ${key}, which it never sent`);
    return { ...face, triangles: mesh.triangles, normals: mesh.normals };
  });
  return { ...view, faces };
}

/**
 * `placed` with `scene`'s changes — its placed parts added, moved and
 * removed, and the components and meshes it sent kept: those of a part
 * that left the scene too, until the kernel says to forget them, so a part
 * that shows again is not sent again — and the scene as drawn.
 */
export function applyScene(placed: Placed, scene: SceneState): { placed: Placed; scene: DrawnScene } {
  const instances = scene.all ? new Map<string, ViewInstance>() : new Map(placed.instances);
  for (const name of scene.removed) instances.delete(name);
  for (const instance of scene.instances) instances.set(instance.name, instance);
  const meshes = { ...(scene.all ? {} : placed.meshes), ...scene.meshes };
  const components = { ...(scene.all ? {} : placed.components) };
  for (const [key, view] of Object.entries(scene.components)) components[key] = withMeshes(view, meshes);
  for (const key of scene.forgotten) delete components[key];
  const part = withMeshes(scene.part, meshes);
  for (const key of scene.forgotten_meshes) delete meshes[key];
  return { placed: { instances, components, meshes }, scene: { ...scene, part } };
}
