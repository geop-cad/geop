import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { CAMERA_FOV, DEFAULT_POSE, REACH_PX, type CameraPose, type Projection } from "./camera";
import { DatumLayer } from "./datums3d";
import {
  sameEntity,
  type DatumInfo,
  type DatumKind,
  type EntityRef,
  type Extent,
  type Frame,
  type Pointer,
  type PointerEvent_,
  type Reach,
  type Scene,
  type Target,
  type Visual,
} from "./geop";
import { PlaneGrid } from "./planeGrid";
import { VisualLayer } from "./visuals3d";

/** How far, in pixels, a press may move and still be a click. */
const CLICK_PX = 4;
/** How soon, in milliseconds, a second click makes a double click. */
const DOUBLE_MS = 350;

/** The kinds of datum a click can pick, among `targets`. */
function datumKinds(targets: Target[]): DatumKind[] {
  return targets.flatMap((t) => (typeof t === "object" ? [t.datum] : []));
}

/** How long a [[Props.focus]] move takes, in milliseconds. */
const FOCUS_MS = 550;

interface Props {
  scene: Scene;
  /** What the step being edited shows, drawn over the model. */
  visuals?: Visual[];
  /** What to draw highlighted — what is picked, what a click would pick. */
  highlights?: EntityRef[];
  /** What a click picks right now: reference geometry of these kinds stands out, the rest fades. */
  pickable?: Target[];
  /** The part's datums, drawn as reference geometry. */
  datums?: DatumInfo[];
  /** Where the drawing is and how big: how big datum planes and axes are drawn. */
  extent?: Extent;
  /** Sketches and datums, by name, not to draw. */
  hidden?: string[];
  /**
   * A plane to work in: no orbiting — dragging pans — and a grid on it.
   * Facing it is the caller's, through [[Props.focus]].
   */
  plane?: Frame | null;
  /** Whether a press where the pointer hovers starts a drag, sent to [[Props.onPointer]], rather than moving the camera. */
  grab?: boolean;
  /** What the user does with the pointer: hovers, clicks, drags — as rays. */
  onPointer?: (event: PointerEvent_) => void;
  /** How the view projects; switching keeps the view (see [[Projection]]). */
  projection: Projection;
  /** A pose to glide to; set it to move the camera, `null` to leave it alone. */
  focus?: CameraPose | null;
  /** Fired when a [[Props.focus]] move finishes, with the pose reached. */
  onFocusReached?: (pose: CameraPose) => void;
  /** Fired whenever the user finishes moving the camera, so a caller can come back to it later. */
  onPose?: (pose: CameraPose) => void;
}

const vec = (v: [number, number, number]) => new THREE.Vector3(v[0], v[1], v[2]);
const arr = (v: THREE.Vector3): [number, number, number] => [v.x, v.y, v.z];

/** Smoothstep: starts and ends at rest, so the move has no visible kick. */
const ease = (t: number) => t * t * (3 - 2 * t);

/**
 * A pose's orientation, as a single rotation — so a focus move can slerp
 * position, target and up together (below) instead of lerping `up` on its
 * own. Lerping `up` independently, then normalizing, does not take the
 * shortest rotation from one orientation to the other: near antiparallel
 * `up`s it passes close to the zero vector, where normalizing amplifies
 * whatever direction floating-point noise happens to leave it pointing —
 * the camera visibly spins around its own view axis while gliding in.
 */
function cameraOrientation(pose: CameraPose): THREE.Quaternion {
  const m = new THREE.Matrix4().lookAt(vec(pose.position), vec(pose.target), vec(pose.up));
  return new THREE.Quaternion().setFromRotationMatrix(m);
}

/** The color a highlighted entity is drawn in. */
const HIGHLIGHT = new THREE.Color(0xffc94a);

/** What a triangle mesh needs to highlight faces by name: its base colors and each triangle's face. */
interface TriangleTags {
  mesh: THREE.Mesh;
  base: Float32Array;
  triangleFaces: number[];
  faces: Scene["faces"];
}

/** Build the meshes/lines/points of one [[Scene]] into a group. */
function buildSceneGroup(scene: Scene): THREE.Group {
  const group = new THREE.Group();

  // Triangles: one mesh, vertex colors so per-triangle color still works.
  if (scene.triangles.length > 0) {
    const positions = new Float32Array(scene.triangles.length * 9);
    const colors = new Float32Array(scene.triangles.length * 9);
    const hasNormals = scene.normals?.length === scene.triangles.length;
    const normals = hasNormals ? new Float32Array(scene.triangles.length * 9) : null;
    scene.triangles.forEach(([ax, ay, az, bx, by, bz, cx, cy, cz, hex], i) => {
      const o = i * 9;
      positions.set([ax, ay, az, bx, by, bz, cx, cy, cz], o);
      normals?.set(scene.normals[i], o);
      const color = new THREE.Color(hex);
      for (let v = 0; v < 3; v++) {
        colors.set([color.r, color.g, color.b], o + v * 3);
      }
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    // The kernel's own normals where it has them: averaging the mesh's
    // facet normals (computeVertexNormals) cannot, since every triangle
    // here has its own three vertices — that is what made curved faces
    // look faceted however finely they were tessellated.
    if (normals) geometry.setAttribute("normal", new THREE.BufferAttribute(normals, 3));
    else geometry.computeVertexNormals();
    const material = new THREE.MeshStandardMaterial({
      vertexColors: true,
      side: THREE.DoubleSide,
      roughness: 0.6,
    });
    const mesh = new THREE.Mesh(geometry, material);
    group.add(mesh);
    group.userData.triangles = {
      mesh,
      base: colors.slice(),
      triangleFaces: scene.triangle_faces,
      faces: scene.faces,
    } satisfies TriangleTags;
  }

  // Lines: one LineSegments per sketch and color (and one per color for
  // the model's edges), so a sketch can be highlighted on its own.
  const byOwner = new Map<string, { hex: number; sketch: string | null; coords: number[] }>();
  scene.lines.forEach(([x0, y0, z0, x1, y1, z1, hex], i) => {
    const owner = scene.line_sketches[i] ?? -1;
    const key = `${owner}:${hex}`;
    const entry = byOwner.get(key) ?? { hex, sketch: owner >= 0 ? scene.sketch_names[owner] : null, coords: [] };
    entry.coords.push(x0, y0, z0, x1, y1, z1);
    byOwner.set(key, entry);
  });
  for (const { hex, sketch, coords } of byOwner.values()) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(coords), 3));
    const lines = new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: hex }));
    lines.userData = { sketch, color: hex };
    group.add(lines);
  }

  // Points.
  if (scene.points.length > 0) {
    const positions = new Float32Array(scene.points.length * 3);
    const colors = new Float32Array(scene.points.length * 3);
    scene.points.forEach(([x, y, z, hex], i) => {
      positions.set([x, y, z], i * 3);
      const color = new THREE.Color(hex);
      colors.set([color.r, color.g, color.b], i * 3);
    });
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
    group.add(new THREE.Points(geometry, new THREE.PointsMaterial({ size: 0.02, vertexColors: true })));
  }

  return group;
}

/** Blend `colors[k..k+3]` most of the way to the highlight color. */
function tint(colors: Float32Array, k: number) {
  colors[k] += (HIGHLIGHT.r - colors[k]) * 0.7;
  colors[k + 1] += (HIGHLIGHT.g - colors[k + 1]) * 0.7;
  colors[k + 2] += (HIGHLIGHT.b - colors[k + 2]) * 0.7;
}

/**
 * Draw `highlights` highlighted in `group` (built by [[buildSceneGroup]]
 * from `scene`), the sketches in `hidden` not at all — unless lit — and
 * everything else as built. Lit edges and vertices are drawn again on top,
 * so they can be seen wherever they are.
 */
function applyHighlight(group: THREE.Group, scene: Scene, highlights: EntityRef[], hidden: string[]) {
  const named = (type: EntityRef["type"]) =>
    new Set(highlights.flatMap((h) => (h.type === type && "name" in h ? [h.name] : [])));
  const [faces, solids, sketches, edges, vertices] = (["Face", "Solid", "Sketch", "Edge", "Vertex"] as const).map(named);
  const tags = group.userData.triangles as TriangleTags | undefined;
  if (tags) {
    const attribute = tags.mesh.geometry.getAttribute("color") as THREE.BufferAttribute;
    const colors = attribute.array as Float32Array;
    colors.set(tags.base);
    tags.triangleFaces.forEach((f, i) => {
      const face = tags.faces[f];
      if (!face || !(faces.has(face.name) || (face.solid != null && solids.has(face.solid)))) return;
      for (let k = i * 9; k < i * 9 + 9; k += 3) tint(colors, k);
    });
    attribute.needsUpdate = true;
  }
  for (const child of group.children) {
    if (!(child instanceof THREE.LineSegments) || child.userData.sketch == null) continue;
    const lit = sketches.has(child.userData.sketch);
    child.visible = lit || !hidden.includes(child.userData.sketch);
    const material = child.material as THREE.LineBasicMaterial;
    material.color.set(lit ? HIGHLIGHT : child.userData.color);
    // A sketch often lies on or behind the model (on a face, or under the
    // solid made from it): lit, it is drawn on top, so it can be seen.
    material.depthTest = !lit;
    child.renderOrder = lit ? 999 : 0;
  }

  const old = group.children.find((c) => c.userData.overlay);
  if (old) {
    group.remove(old);
    disposeGroup(old as THREE.Group);
  }
  if (edges.size === 0 && vertices.size === 0) return;
  const overlay = new THREE.Group();
  overlay.userData.overlay = true;
  const coords: number[] = [];
  scene.lines.forEach(([x0, y0, z0, x1, y1, z1], i) => {
    const edge = scene.line_edges[i] ?? -1;
    if (edge >= 0 && edges.has(scene.edge_names[edge])) coords.push(x0, y0, z0, x1, y1, z1);
  });
  if (coords.length) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(coords), 3));
    overlay.add(new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: HIGHLIGHT, depthTest: false })));
  }
  const points = scene.points.filter((_, i) => vertices.has(scene.point_names[i]));
  if (points.length) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(points.flatMap(([x, y, z]) => [x, y, z])), 3));
    overlay.add(new THREE.Points(geometry, new THREE.PointsMaterial({ color: HIGHLIGHT, size: 9, sizeAttenuation: false, depthTest: false })));
  }
  overlay.traverse((o) => (o.renderOrder = 999));
  group.add(overlay);
}

/** Free every geometry and material a group owns. */
function disposeGroup(group: THREE.Object3D) {
  group.traverse((o) => {
    const any = o as THREE.Mesh;
    any.geometry?.dispose();
    const material = any.material;
    if (Array.isArray(material)) material.forEach((m) => m.dispose());
    else material?.dispose();
  });
  group.clear();
}

/**
 * Renders a [[Scene]] (points/lines/triangles from the wasm crate) with
 * three.js, and what the step being edited shows over it.
 *
 * The renderer, camera and controls are created once and kept: a new scene
 * (a committed op, or a preview updating under a slider) only swaps the
 * geometry group, so the view the user has orbited to stays exactly where
 * it was.
 *
 * It picks nothing itself: hovers, clicks and drags go to
 * [[Props.onPointer]] as rays, with what a screen pixel measures along
 * them, and the kernel decides what they hit.
 */
export function SceneViewer({
  scene,
  visuals,
  highlights,
  pickable,
  datums,
  extent,
  hidden,
  plane,
  grab,
  onPointer,
  projection,
  focus,
  onFocusReached,
  onPose,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  // Read inside the effects via refs so a prop change doesn't tear down
  // anything.
  const visualsRef = useRef(visuals ?? []);
  visualsRef.current = visuals ?? [];
  const highlightsRef = useRef(highlights ?? []);
  highlightsRef.current = highlights ?? [];
  const pickableRef = useRef(pickable ?? []);
  pickableRef.current = pickable ?? [];
  const datumsRef = useRef(datums ?? []);
  datumsRef.current = datums ?? [];
  const extentRef = useRef<Extent>(extent ?? { center: [0, 0, 0], size: 1 });
  extentRef.current = extent ?? { center: [0, 0, 0], size: 1 };
  const hiddenRef = useRef(hidden ?? []);
  hiddenRef.current = hidden ?? [];
  const planeRef = useRef(plane ?? null);
  planeRef.current = plane ?? null;
  const grabRef = useRef(grab ?? false);
  grabRef.current = grab ?? false;
  const onPointerRef = useRef(onPointer);
  onPointerRef.current = onPointer;
  const sceneRef = useRef<THREE.Scene | null>(null);
  const groupRef = useRef<THREE.Group | null>(null);
  const cameraRef = useRef<THREE.Camera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  // The move in progress, if any: where it started, where it is going, when.
  const moveRef = useRef<{
    from: CameraPose;
    to: CameraPose;
    fromQuat: THREE.Quaternion;
    toQuat: THREE.Quaternion;
    start: number;
  } | null>(null);
  const onFocusReachedRef = useRef(onFocusReached);
  onFocusReachedRef.current = onFocusReached;
  const onPoseRef = useRef(onPose);
  onPoseRef.current = onPose;
  const projectionRef = useRef(projection);
  projectionRef.current = projection;

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);
    // Labels are HTML over the canvas: crisp text, and they never catch
    // the pointer.
    const labelRenderer = new CSS2DRenderer();
    Object.assign(labelRenderer.domElement.style, { position: "absolute", top: "0", left: "0", pointerEvents: "none" });
    container.appendChild(labelRenderer.domElement);

    const threeScene = new THREE.Scene();
    threeScene.background = new THREE.Color(0x1a1a1a);
    sceneRef.current = threeScene;

    const perspective = new THREE.PerspectiveCamera(CAMERA_FOV, 1, 0.01, 1000);
    // Sized every frame from its distance to the target (see `fitOrthographic`);
    // it sees what is behind it too, since only its direction matters.
    const orthographic = new THREE.OrthographicCamera(-1, 1, 1, -1, -1000, 1000);
    let camera: THREE.PerspectiveCamera | THREE.OrthographicCamera =
      projectionRef.current === "orthographic" ? orthographic : perspective;
    camera.position.set(...DEFAULT_POSE.position);
    camera.up.set(...DEFAULT_POSE.up);
    cameraRef.current = camera;

    // `OrbitControls` reads `camera.up` once, when it is made, and orbits
    // about that axis from then on. A focus move can turn the camera's up
    // (to face a plane head on), so the controls are made afresh whenever
    // one lands: orbiting about a stale up is what turned a vertical drag
    // sideways. Making them afresh also drops any momentum left over from
    // before the move. The same holds for the camera itself: the controls
    // are made for one.
    const makeControls = (target: THREE.Vector3): OrbitControls => {
      const made = new OrbitControls(camera, renderer.domElement);
      made.enableDamping = true;
      made.target.copy(target);
      made.addEventListener("end", () => onPoseRef.current?.(pose()));
      controlsRef.current = made;
      return made;
    };
    let controls = makeControls(vec(DEFAULT_POSE.target));
    const pose = (): CameraPose => ({
      position: arr(camera.position),
      target: arr(controls.target),
      up: arr(camera.up),
    });

    threeScene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(5, 8, 6);
    threeScene.add(dirLight);
    const datumLayer = new DatumLayer();
    threeScene.add(datumLayer.group);
    const visualLayer = new VisualLayer();
    threeScene.add(visualLayer.group);
    const grid = new PlaneGrid();
    threeScene.add(grid.group);

    const resize = () => {
      const { clientWidth, clientHeight } = container;
      renderer.setSize(clientWidth, clientHeight);
      labelRenderer.setSize(clientWidth, clientHeight);
      perspective.aspect = clientWidth / clientHeight;
      perspective.updateProjectionMatrix();
    };
    /**
     * Size the orthographic camera like a perspective one at its distance
     * to the target — after taking in any zoom the controls applied, as
     * distance: zooming an orthographic camera is moving it closer.
     */
    const fitOrthographic = () => {
      const target = controls.target;
      if (orthographic.zoom !== 1) {
        orthographic.position.sub(target).divideScalar(orthographic.zoom).add(target);
        orthographic.zoom = 1;
      }
      const half = orthographic.position.distanceTo(target) * Math.tan(THREE.MathUtils.degToRad(CAMERA_FOV) / 2);
      const aspect = container.clientWidth / Math.max(container.clientHeight, 1);
      orthographic.top = half;
      orthographic.bottom = -half;
      orthographic.left = -half * aspect;
      orthographic.right = half * aspect;
      orthographic.updateProjectionMatrix();
    };
    /** Switch to the camera `projectionRef` asks for, where the current one is. */
    const applyProjection = () => {
      const next = projectionRef.current === "orthographic" ? orthographic : perspective;
      if (next === camera) return;
      next.position.copy(camera.position);
      next.up.copy(camera.up);
      next.quaternion.copy(camera.quaternion);
      camera = next;
      cameraRef.current = camera;
      const target = controls.target.clone();
      const enabled = controls.enabled;
      controls.dispose();
      controls = makeControls(target);
      controls.enabled = enabled;
    };
    resize();
    const resizeObserver = new ResizeObserver(resize);
    resizeObserver.observe(container);

    /**
     * The pointer at `(x, y)`, in client coordinates: the ray through it,
     * reaching [[REACH_PX]] pixels — a cone from the eye in perspective, a
     * tube in an orthographic view.
     */
    const pointerAt = (x: number, y: number): Pointer => {
      const rect = container.getBoundingClientRect();
      const ndc = new THREE.Vector2(((x - rect.left) / rect.width) * 2 - 1, -((y - rect.top) / rect.height) * 2 + 1);
      const raycaster = new THREE.Raycaster();
      raycaster.setFromCamera(ndc, camera);
      const height = Math.max(container.clientHeight, 1);
      const reach: Reach =
        camera instanceof THREE.OrthographicCamera
          ? { type: "tube", radius: (REACH_PX * (camera.top - camera.bottom)) / camera.zoom / height }
          : { type: "cone", slope: (REACH_PX * 2 * Math.tan(THREE.MathUtils.degToRad(CAMERA_FOV) / 2)) / height };
      return { ray: { origin: arr(raycaster.ray.origin), dir: arr(raycaster.ray.direction) }, reach };
    };
    const send = (event: PointerEvent_) => onPointerRef.current?.(event);

    // A press: a click if it does not move, else the camera's — or, where
    // the hover offered a grab, a drag of what is under it.
    let press: { x: number; y: number; button: number; grabbed: boolean; from: Pointer; moved: boolean } | null = null;
    let lastClick: { x: number; y: number; time: number } | null = null;
    // The latest pointer position, handled once per frame.
    let pendingHover: { x: number; y: number } | "leave" | null = null;
    let pendingDrag: { x: number; y: number } | null = null;

    const onPointerDown = (e: PointerEvent) => {
      if (moveRef.current) return;
      const grabbed = e.button === 0 && grabRef.current;
      press = { x: e.clientX, y: e.clientY, button: e.button, grabbed, from: pointerAt(e.clientX, e.clientY), moved: false };
      if (grabbed) {
        // The drag is the step's: neither the controls nor a click see it.
        e.stopPropagation();
        controls.enabled = false;
        container.setPointerCapture(e.pointerId);
      }
    };
    const onPointerMove = (e: PointerEvent) => {
      if (press) {
        if (Math.hypot(e.clientX - press.x, e.clientY - press.y) > CLICK_PX) press.moved = true;
        if (press.grabbed && press.moved) pendingDrag = { x: e.clientX, y: e.clientY };
        return;
      }
      if (e.buttons === 0) pendingHover = { x: e.clientX, y: e.clientY };
    };
    const onPointerUp = (e: PointerEvent) => {
      const done = press;
      press = null;
      if (!done) return;
      if (done.grabbed) {
        controls.enabled = !moveRef.current;
        pendingDrag = null;
        if (done.moved) {
          send({ type: "drag", from: done.from, to: pointerAt(e.clientX, e.clientY), done: true });
          pendingHover = { x: e.clientX, y: e.clientY };
          return;
        }
      }
      if (done.moved || (done.button !== 0 && done.button !== 2)) return;
      const now = performance.now();
      const double =
        lastClick != null &&
        now - lastClick.time < DOUBLE_MS &&
        Math.hypot(e.clientX - lastClick.x, e.clientY - lastClick.y) <= CLICK_PX;
      lastClick = double ? null : { x: e.clientX, y: e.clientY, time: now };
      send({
        type: "click",
        pointer: pointerAt(e.clientX, e.clientY),
        button: done.button === 2 ? "secondary" : "primary",
        double,
        shift: e.shiftKey,
      });
    };
    const onPointerLeave = () => {
      pendingHover = "leave";
    };
    const onContextMenu = (e: Event) => e.preventDefault();
    // In the capture phase: a grab must be taken before the controls see
    // the press.
    container.addEventListener("pointerdown", onPointerDown, { capture: true });
    container.addEventListener("pointermove", onPointerMove);
    container.addEventListener("pointerup", onPointerUp);
    container.addEventListener("pointerleave", onPointerLeave);
    container.addEventListener("contextmenu", onContextMenu);

    let frame = requestAnimationFrame(function animate() {
      applyProjection();
      // A focus move drives the camera itself; the controls take over again
      // the moment it lands.
      const move = moveRef.current;
      if (move) {
        const t = Math.min(1, (performance.now() - move.start) / FOCUS_MS);
        const k = ease(t);
        camera.position.lerpVectors(vec(move.from.position), vec(move.to.position), k);
        controls.target.lerpVectors(vec(move.from.target), vec(move.to.target), k);
        const qK = move.fromQuat.clone().slerp(move.toQuat, k);
        camera.up.set(0, 1, 0).applyQuaternion(qK).normalize();
        camera.lookAt(controls.target);
        if (t >= 1) {
          moveRef.current = null;
          controls.dispose();
          controls = makeControls(vec(move.to.target));
          onFocusReachedRef.current?.(pose());
          onPoseRef.current?.(pose());
        }
      } else {
        // Working in a plane, the view stays head on to it: dragging pans.
        const inPlane = planeRef.current != null;
        controls.enableRotate = !inPlane;
        controls.mouseButtons.LEFT = inPlane ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
        controls.update();
      }
      if (camera === orthographic) fitOrthographic();
      const height = container.clientHeight;

      const hover = pendingHover;
      pendingHover = null;
      if (hover === "leave") send({ type: "leave" });
      else if (hover) send({ type: "hover", pointer: pointerAt(hover.x, hover.y) });
      const drag = pendingDrag;
      pendingDrag = null;
      if (drag && press) send({ type: "drag", from: press.from, to: pointerAt(drag.x, drag.y), done: false });
      renderer.domElement.style.cursor = press?.grabbed ? "grabbing" : grabRef.current ? "grab" : "";

      const lit = highlightsRef.current;
      const hidden = hiddenRef.current.filter((name) => !lit.some((l) => sameEntity(l, { type: "Datum", name })));
      datumLayer.sync(datumsRef.current, hidden, extentRef.current.size, vec(extentRef.current.center));
      datumLayer.update(camera, height, datumKinds(pickableRef.current), lit);
      visualLayer.sync(visualsRef.current);
      visualLayer.update(camera, height, controls.target);
      grid.sync(planeRef.current);
      grid.update(camera, height, controls.target);

      renderer.render(threeScene, camera);
      labelRenderer.render(threeScene, camera);
      frame = requestAnimationFrame(animate);
    });

    return () => {
      cancelAnimationFrame(frame);
      resizeObserver.disconnect();
      container.removeEventListener("pointerdown", onPointerDown, { capture: true });
      container.removeEventListener("pointermove", onPointerMove);
      container.removeEventListener("pointerup", onPointerUp);
      container.removeEventListener("pointerleave", onPointerLeave);
      container.removeEventListener("contextmenu", onContextMenu);
      controls.dispose();
      renderer.dispose();
      container.removeChild(renderer.domElement);
      container.removeChild(labelRenderer.domElement);
      sceneRef.current = null;
      cameraRef.current = null;
      controlsRef.current = null;
    };
  }, []);

  // Start gliding whenever a new focus arrives.
  useEffect(() => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!focus || !camera || !controls) return;
    controls.enabled = false;
    const from: CameraPose = { position: arr(camera.position), target: arr(controls.target), up: arr(camera.up) };
    moveRef.current = {
      from,
      to: focus,
      fromQuat: cameraOrientation(from),
      toQuat: cameraOrientation(focus),
      start: performance.now(),
    };
  }, [focus]);

  // Swap in the current scene's geometry, leaving camera and controls alone.
  useEffect(() => {
    const threeScene = sceneRef.current;
    if (!threeScene) return;
    if (groupRef.current) {
      threeScene.remove(groupRef.current);
      disposeGroup(groupRef.current);
    }
    const group = buildSceneGroup(scene);
    groupRef.current = group;
    threeScene.add(group);
    applyHighlight(group, scene, highlightsRef.current, hiddenRef.current);
  }, [scene]);

  // Re-tint in place when the highlights or what is hidden change: nothing is rebuilt.
  const appearanceKey = JSON.stringify([highlights ?? [], hidden ?? []]);
  useEffect(() => {
    if (groupRef.current) applyHighlight(groupRef.current, scene, highlightsRef.current, hiddenRef.current);
    // `scene` is the one the group was built from: a new one rebuilds it above.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [appearanceKey]);

  return <div ref={containerRef} style={{ position: "relative", width: "100%", height: "100%", minHeight: 0 }} />;
}
