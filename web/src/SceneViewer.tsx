import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CAMERA_FOV, DEFAULT_POSE, scaleForDistance, worldPerPixel, type CameraPose, type Projection } from "./camera";
import { DatumLayer, type DatumHit } from "./datums3d";
import {
  sameEntity,
  type DatumInfo,
  type DatumKind,
  type EntityRef,
  type Highlight,
  type Scene,
  type StepHandle,
  type Vec3,
} from "./geop";
import { HandleLayer, dragEdits, startDrag, type HandleDrag, type HandleEdit } from "./handles3d";
import { buildGizmo, pickGizmo, updateGizmo } from "./originGizmo";

export interface Marker {
  point: Vec3;
  color: number;
}

/** A ray from the camera through the cursor, and what a pick along it needs to know. */
export interface ViewRay {
  origin: Vec3;
  /** Unit length: a distance along the ray is a distance in the world. */
  dir: Vec3;
  /** What one screen pixel measures, in world units, around where the camera looks. */
  pixel: number;
  /** The nearest datum of a pickable kind the ray hits, if any. */
  datum: DatumHit | null;
}

/** How long a [[Props.focus]] move takes, in milliseconds. */
const FOCUS_MS = 550;

interface Props {
  scene: Scene;
  /** Small colored spheres overlaid on the scene, e.g. current picks. */
  markers?: Marker[];
  /** Fired on a plain click (not a drag-to-orbit) with the ray under the cursor. */
  onPick?: (ray: ViewRay) => void;
  /** Which kinds of datum a click can pick right now — on the origin gizmo, or among [[Props.datums]]; none by default. */
  pickable?: DatumKind[];
  /** Fired instead of [[Props.onPick]] when a click hits a pickable part of the origin gizmo. */
  onPickEntity?: (entity: EntityRef, point: Vec3) => void;
  /**
   * Fired as the pointer moves (at most once a frame, never while dragging)
   * with the ray under it — or `null` when there is nothing to show: the
   * pointer left, is dragging, or is over the origin gizmo, which shows its
   * own hover.
   */
  onHover?: (ray: ViewRay | null) => void;
  /** What to draw highlighted — what a click would pick, what is picked already. */
  highlights?: Highlight[];
  /** The part's datums, drawn as reference geometry. */
  datums?: DatumInfo[];
  /** Sketches and datums, by name, not to draw. */
  hidden?: string[];
  /** The handles to offer: drawn on top, and draggable. */
  handles?: StepHandle[];
  /** A handle is being dragged: `edits` are its new values. Called as the drag goes, at most once a frame. */
  onHandleDrag?: (handle: StepHandle, edits: HandleEdit[]) => void;
  /** How the view projects; switching keeps the view (see [[Projection]]). */
  projection: Projection;
  /** While set (and no [[Props.focus]] move runs), the camera is held here: a sketch editor drawn over the view drives it. */
  heldPose?: CameraPose | null;
  /** A pose to glide to; set it to move the camera, `null` to leave it alone. */
  focus?: CameraPose | null;
  /** Fired when a [[Props.focus]] move finishes, with the pose reached and the viewport's scale there. */
  onFocusReached?: (reached: { pose: CameraPose; pixelsPerUnit: number; height: number }) => void;
  /** Fired whenever the user finishes moving the camera, so a caller can come back to it later. */
  onPose?: (pose: CameraPose) => void;
}

const vec = (v: Vec3) => new THREE.Vector3(v[0], v[1], v[2]);
const arr = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];

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
function applyHighlight(group: THREE.Group, scene: Scene, highlights: Highlight[], hidden: string[]) {
  const named = (type: Highlight["type"]) =>
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

/** Where the scene is and how big: the center and diagonal (at least 1) of the box around it. */
function sceneExtent(group: THREE.Group): { center: THREE.Vector3; size: number } {
  const box = new THREE.Box3().setFromObject(group);
  if (box.isEmpty()) return { center: new THREE.Vector3(), size: 1 };
  return { center: box.getCenter(new THREE.Vector3()), size: Math.max(1, box.getSize(new THREE.Vector3()).length()) };
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
 * Renders a [[Scene]] (points/lines/triangles from the wasm crate) with three.js.
 *
 * The renderer, camera and controls are created once and kept: a new scene
 * (a committed op, or a preview updating under a slider) only swaps the
 * geometry group, so the view the user has orbited to stays exactly where
 * it was.
 */
export function SceneViewer({
  scene,
  markers,
  onPick,
  pickable,
  onPickEntity,
  onHover,
  highlights,
  datums,
  hidden,
  handles,
  onHandleDrag,
  projection,
  heldPose,
  focus,
  onFocusReached,
  onPose,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  // Read inside the effects via refs so a marker/callback change doesn't
  // tear down anything.
  const onPickRef = useRef(onPick);
  onPickRef.current = onPick;
  const pickableRef = useRef(pickable ?? []);
  pickableRef.current = pickable ?? [];
  const onPickEntityRef = useRef(onPickEntity);
  onPickEntityRef.current = onPickEntity;
  const onHoverRef = useRef(onHover);
  onHoverRef.current = onHover;
  const handlesRef = useRef(handles ?? []);
  handlesRef.current = handles ?? [];
  const onHandleDragRef = useRef(onHandleDrag);
  onHandleDragRef.current = onHandleDrag;
  const highlightsRef = useRef(highlights ?? []);
  highlightsRef.current = highlights ?? [];
  const hiddenRef = useRef(hidden ?? []);
  hiddenRef.current = hidden ?? [];
  const datumsRef = useRef(datums ?? []);
  datumsRef.current = datums ?? [];
  /** Where the scene is and how big: where and how big datum planes and axes are drawn. */
  const extentRef = useRef({ center: new THREE.Vector3(), size: 1 });
  const markersRef = useRef(markers);
  markersRef.current = markers;
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
  const heldPoseRef = useRef(heldPose ?? null);
  heldPoseRef.current = heldPose ?? null;

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

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
    // (to face a sketch plane head on), so the controls are made afresh
    // whenever one lands: orbiting about a stale up is what turned a
    // vertical drag sideways. Making them afresh also drops any momentum
    // left over from before the move.
    // The same holds for the camera itself: the controls are made for one.
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
    const gizmo = buildGizmo();
    threeScene.add(gizmo);
    const handleLayer = new HandleLayer();
    threeScene.add(handleLayer.group);
    const datumLayer = new DatumLayer();
    threeScene.add(datumLayer.group);

    // Markers: re-synced every frame from `markersRef`, so they track
    // selection changes without rebuilding the rest of the scene.
    const markerGroup = new THREE.Group();
    threeScene.add(markerGroup);
    const markerGeometry = new THREE.SphereGeometry(0.035, 12, 12);
    let markerSignature = "";

    const resize = () => {
      const { clientWidth, clientHeight } = container;
      renderer.setSize(clientWidth, clientHeight);
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
    /** Whether the camera was held last frame: the controls take over afresh once it is let go. */
    let wasHeld = false;
    resize();
    const resizeObserver = new ResizeObserver(resize);
    resizeObserver.observe(container);

    // Click-to-pick: a plain click (down and up within a small pixel
    // radius) casts a ray from the camera through the cursor; a drag (an
    // orbit-control gesture) does not.
    let downPos: { x: number; y: number } | null = null;
    /** A ray from the camera through the viewport point `(x, y)`, in client coordinates. */
    const rayAt = (x: number, y: number): THREE.Raycaster => {
      const rect = container.getBoundingClientRect();
      const ndc = new THREE.Vector2(((x - rect.left) / rect.width) * 2 - 1, -((y - rect.top) / rect.height) * 2 + 1);
      const raycaster = new THREE.Raycaster();
      raycaster.setFromCamera(ndc, camera);
      return raycaster;
    };
    /** What one pixel measures in world units, around where the camera looks. */
    const pixel = () => worldPerPixel(camera, controls.target, container.clientHeight);
    /** `raycaster`'s ray as a pick needs it. */
    const viewRay = (raycaster: THREE.Raycaster): ViewRay => {
      const { origin: o, direction: d } = raycaster.ray;
      return {
        origin: [o.x, o.y, o.z],
        dir: [d.x, d.y, d.z],
        pixel: pixel(),
        datum: datumLayer.pick(raycaster, pickableRef.current, pixel()),
      };
    };
    // Hover: the latest pointer position, handled once per frame.
    let pendingHover: THREE.Raycaster | "clear" | null = null;
    let hoveredEntity: EntityRef | null = null;
    let hoveredHandle: StepHandle | null = null;
    // A handle drag: the handle as grabbed, and the latest pointer ray.
    let drag: HandleDrag | null = null;
    let dragRay: THREE.Ray | null = null;
    let lastEdits: HandleEdit[] = [];
    const onPointerMove = (e: PointerEvent) => {
      if (drag) {
        dragRay = rayAt(e.clientX, e.clientY).ray;
        return;
      }
      pendingHover = e.buttons !== 0 ? "clear" : rayAt(e.clientX, e.clientY);
    };
    // Grabbing a handle, in the capture phase: the drag is the handle's,
    // so neither the orbit controls nor a pick ever see it.
    const onHandleDown = (e: PointerEvent) => {
      if (e.button !== 0 || moveRef.current) return;
      const ray = rayAt(e.clientX, e.clientY);
      const handle = handleLayer.pick(ray);
      const grabbed = handle && startDrag(handle, ray.ray);
      if (!grabbed) return;
      e.stopPropagation();
      drag = grabbed;
      dragRay = null;
      lastEdits = [];
      controls.enabled = false;
      renderer.domElement.setPointerCapture(e.pointerId);
    };
    /** Deliver where the drag is now, unless that was already delivered. */
    const flushDrag = () => {
      if (!drag || !dragRay) return;
      const edits = dragEdits(drag, dragRay);
      dragRay = null;
      if (edits.length && JSON.stringify(edits) !== JSON.stringify(lastEdits)) {
        lastEdits = edits;
        onHandleDragRef.current?.(drag.handle, edits);
      }
    };
    const endDrag = () => {
      flushDrag();
      drag = null;
      dragRay = null;
      controls.enabled = !moveRef.current;
    };
    const onPointerLeave = () => {
      pendingHover = "clear";
    };
    const onPointerDown = (e: PointerEvent) => {
      downPos = { x: e.clientX, y: e.clientY };
    };
    const onPointerUp = (e: PointerEvent) => {
      if (drag) {
        endDrag();
        return;
      }
      const start = downPos;
      downPos = null;
      if (!start) return;
      if (Math.hypot(e.clientX - start.x, e.clientY - start.y) > 4) return;
      const raycaster = rayAt(e.clientX, e.clientY);
      // The gizmo is drawn on top, so it is picked first.
      const gizmoHit = pickGizmo(gizmo, raycaster, pickableRef.current);
      if (gizmoHit && onPickEntityRef.current) {
        onPickEntityRef.current(gizmoHit.entity, gizmoHit.point);
        return;
      }
      onPickRef.current?.(viewRay(raycaster));
    };
    container.addEventListener("pointerdown", onHandleDown, { capture: true });
    renderer.domElement.addEventListener("pointerdown", onPointerDown);
    renderer.domElement.addEventListener("pointerup", onPointerUp);
    renderer.domElement.addEventListener("pointermove", onPointerMove);
    renderer.domElement.addEventListener("pointerleave", onPointerLeave);

    let frame = requestAnimationFrame(function animate() {
      applyProjection();
      // A focus move drives the camera itself; the controls take over again
      // the moment it lands. Short of one, a held pose does.
      const move = moveRef.current;
      const held = heldPoseRef.current;
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
          const height = container.clientHeight;
          const distance = camera.position.distanceTo(controls.target);
          onFocusReachedRef.current?.({
            pose: pose(),
            // What one world unit measures on screen at this distance —
            // the scale a 2-D view has to start at to continue this one.
            pixelsPerUnit: scaleForDistance(distance, height),
            height,
          });
          onPoseRef.current?.(pose());
        }
      } else if (held) {
        camera.position.set(...held.position);
        controls.target.set(...held.target);
        camera.up.set(...held.up);
        camera.lookAt(controls.target);
        controls.enabled = false;
        wasHeld = true;
      } else {
        if (wasHeld) {
          controls.dispose();
          controls = makeControls(controls.target.clone());
          wasHeld = false;
        }
        // Only here: while a move runs or a pose is held, that alone places
        // the camera, and the controls' damping would add to it.
        controls.update();
      }
      if (camera === orthographic) fitOrthographic();

      handleLayer.sync(handlesRef.current);
      flushDrag();
      const hover = pendingHover;
      pendingHover = null;
      if (hover === "clear") {
        hoveredEntity = null;
        hoveredHandle = null;
        onHoverRef.current?.(null);
      } else if (hover) {
        // Handles are drawn on top of everything, so they are hit first.
        hoveredHandle = handleLayer.pick(hover);
        hoveredEntity = hoveredHandle ? null : (pickGizmo(gizmo, hover, pickableRef.current)?.entity ?? null);
        onHoverRef.current?.(hoveredEntity || hoveredHandle ? null : viewRay(hover));
        renderer.domElement.style.cursor = hoveredHandle ? "grab" : "";
      }
      handleLayer.update(camera, container.clientHeight, drag?.handle ?? hoveredHandle);
      const lit = hoveredEntity ? [...highlightsRef.current, hoveredEntity] : highlightsRef.current;
      updateGizmo(gizmo, camera, container.clientHeight, pickableRef.current, lit);
      datumLayer.sync(datumsRef.current, hiddenRef.current.filter((name) => !lit.some((l) => sameEntity(l, { type: "Datum", name }))), extentRef.current.size, extentRef.current.center);
      datumLayer.update(camera, container.clientHeight, pickableRef.current, lit);

      const current = markersRef.current ?? [];
      const signature = current.map((m) => `${m.point.join(",")}:${m.color}`).join("|");
      if (signature !== markerSignature) {
        markerSignature = signature;
        markerGroup.clear();
        for (const marker of current) {
          const mesh = new THREE.Mesh(
            markerGeometry,
            new THREE.MeshBasicMaterial({ color: marker.color, depthTest: false }),
          );
          mesh.position.set(...marker.point);
          mesh.renderOrder = 999;
          markerGroup.add(mesh);
        }
      }

      renderer.render(threeScene, camera);
      frame = requestAnimationFrame(animate);
    });

    return () => {
      cancelAnimationFrame(frame);
      resizeObserver.disconnect();
      container.removeEventListener("pointerdown", onHandleDown, { capture: true });
      renderer.domElement.removeEventListener("pointerdown", onPointerDown);
      renderer.domElement.removeEventListener("pointerup", onPointerUp);
      renderer.domElement.removeEventListener("pointermove", onPointerMove);
      renderer.domElement.removeEventListener("pointerleave", onPointerLeave);
      controls.dispose();
      renderer.dispose();
      container.removeChild(renderer.domElement);
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
    extentRef.current = sceneExtent(group);
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

  return <div ref={containerRef} style={{ width: "100%", height: "100%", minHeight: 0 }} />;
}
