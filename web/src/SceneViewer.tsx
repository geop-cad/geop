import { useEffect, useMemo, useRef, useState } from "react";
import * as THREE from "three";
import { TrackballControls } from "three/examples/jsm/controls/TrackballControls.js";
import { CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import {
  CAMERA_FOV,
  DEFAULT_POSE,
  REACH_PX,
  fitPose,
  regionName,
  standardPose,
  worldPerPixel,
  type CameraPose,
  type Projection,
} from "./camera";
import { DatumLayer } from "./datums3d";
import {
  litDatums,
  locatedKey,
  parseKey,
  type DatumInfo,
  type DatumKind,
  type Lighting,
  type Frame,
  type PartView,
  type Pointer,
  type PointerEvent_,
  type Prompt,
  type Reach,
  type Vec3,
  type ViewInstance,
  type Visual,
  type GizmoView,
} from "./geop";
import { CAP_COLOR, applyHighlight, buildSceneGroup, disposeGroup, flatten, frameMatrix, type Scene } from "./partScene";
import { PlacedLayer, type PlacedLook } from "./placed3d";
import type { SectionPlane } from "./section";
import { PlaneGrid } from "./planeGrid";
import { VisualLayer } from "./visuals3d";
import { GizmoLayer } from "./gizmo3d";
import { isDoubleClick, movedFromPress, wheelSteps, zoomToward } from "./gestures";
import { withShortcut } from "./shortcuts";
import { ViewCube } from "./viewCube";

/** The kinds of datum a click can pick something of, picking elements of the kinds `pickable`: every kind, if it picks datums or frames on them. */
function datumKinds(pickable: string[]): DatumKind[] {
  return pickable.includes("datum") || pickable.includes("frame") ? ["point", "axis", "plane", "frame"] : [];
}

/** How long a [[Props.focus]] move takes, in milliseconds. */
const FOCUS_MS = 550;

/** How fast a drag pans, as the controls measure it: alike in either projection (see the render loop). */
const PAN_SPEED = 0.6;

/** How far the view turns per pixel the view cube is dragged, in radians. */
const CUBE_TURN = 0.01;

interface Props {
  /** The part to draw, with its datums. */
  part: PartView;
  /** The parts placed in it, however deep, each drawn from its component's view. */
  instances: ViewInstance[];
  /** The views of the components the placed parts are drawn from, by key. */
  components: Record<string, PartView>;
  /** What the step being edited shows, drawn over the model. */
  visuals?: Visual[];
  /** A gizmo to move, turn or scale by, drawn over everything. */
  gizmo?: GizmoView | null;
  /** What to draw lit, as the kernel says: what is picked, what a click would pick, what a list points at. */
  lit: Lighting;
  /** What a click picks right now: reference geometry of these kinds stands out, the rest fades. */
  pickable?: string[];
  /** Sketches and datums, by name, not to draw. */
  hidden?: string[];
  /**
   * A plane to work in: no orbiting — dragging pans — and a grid on it.
   * Facing it is the caller's, through [[Props.focus]].
   */
  plane?: Frame | null;
  /** Whether a grid is drawn on the [[Props.plane]]: not on a sheet of paper. */
  grid?: boolean;
  /** A value asked for in place: an input drawn where its point is. */
  prompt?: Prompt | null;
  /** The value typed into the [[Props.prompt]]: Enter gives it, Escape gives up. */
  onPrompt?: (key: string, text: string) => void;
  onPromptCancel?: () => void;
  /** Whether a press where the pointer hovers starts a drag, sent to [[Props.onPointer]], rather than moving the camera. */
  grab?: boolean;
  /**
   * What the user does with the pointer: hovers, clicks, drags — as rays.
   * Resolves to whether a press where the pointer now is grabs.
   */
  onPointer?: (event: PointerEvent_) => Promise<boolean>;
  /** How the view projects; switching keeps the view (see [[Projection]]). */
  projection: Projection;
  /** A pose to glide to; set it to move the camera, `null` to leave it alone. */
  focus?: CameraPose | null;
  /**
   * Set afresh to frame the whole drawing — the part and the parts placed
   * in it — looking from `from`'s direction, with its up, else from the
   * direction the camera looks now: a glide like [[Props.focus]]'s.
   */
  fit?: { from: CameraPose | null } | null;
  /** Fired when a [[Props.focus]] move finishes, with the pose reached. */
  onFocusReached?: (pose: CameraPose) => void;
  /** Fired whenever the user finishes moving the camera, so a caller can come back to it later. */
  onPose?: (pose: CameraPose) => void;
  /** A section view: what lies on the side the normal points to is not drawn, and cut solids are capped. */
  section?: SectionPlane | null;
  /** Which of what lies under the pointer is lit, of how many, as the kernel says (see `Presentation.cycle`). */
  cycle?: { at: number; of: number } | null;
  /** The view cube's home button: back to the home view. */
  onHome?: () => void;
  /** The home view's key, for the button's tooltip. */
  homeKey?: string | null;
  /**
   * How many pixels at the bottom of the view something lies over (the
   * welcome): the view is centred in the rest, so what is in its middle —
   * the origin of an empty part — is not under it.
   */
  covered?: number;
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

/** The datums of the placed part `instance` — drawn from `view` — where it is, named behind it. */
function placedDatums(instance: ViewInstance, view: PartView | undefined): DatumInfo[] {
  if (!view) return [];
  const m = frameMatrix(instance.frame);
  const point = (p: Vec3) => arr(vec(p).applyMatrix4(m));
  const direction = (d: Vec3) => arr(vec(d).transformDirection(m));
  return view.datums.map((d) => ({
    name: locatedKey(instance.name, d.name),
    kind: d.kind,
    frame: { origin: point(d.frame.origin), u: direction(d.frame.u), v: direction(d.frame.v), w: direction(d.frame.w) },
  }));
}

/**
 * Renders a part (a [[PartView]] from the wasm crate) with three.js, and
 * what the step being edited shows over it.
 *
 * The renderer, camera and controls are created once and kept: a new scene
 * (a committed op, or a preview updating under a slider) only swaps the
 * geometry group, so the view the user has orbited to stays exactly where
 * it was.
 *
 * It picks nothing itself: hovers, clicks and drags go to
 * [[Props.onPointer]] as rays, with how far they reach, and the kernel
 * decides what they hit.
 */
export function SceneViewer({
  part,
  instances,
  components,
  visuals,
  gizmo,
  lit,
  pickable,
  hidden,
  plane,
  grid,
  grab,
  onPointer,
  prompt,
  onPrompt,
  onPromptCancel,
  projection,
  focus,
  fit,
  onFocusReached,
  onPose,
  section,
  cycle,
  onHome,
  homeKey,
  covered,
}: Props) {
  const sectionRef = useRef(section ?? null);
  sectionRef.current = section ?? null;
  const coveredRef = useRef(covered ?? 0);
  coveredRef.current = covered ?? 0;
  const containerRef = useRef<HTMLDivElement>(null);
  // Read inside the effects via refs so a prop change doesn't tear down
  // anything.
  const visualsRef = useRef(visuals ?? []);
  visualsRef.current = visuals ?? [];
  const gizmoRef = useRef(gizmo ?? null);
  gizmoRef.current = gizmo ?? null;
  const litRef = useRef(lit);
  litRef.current = lit;
  const pickableRef = useRef(pickable ?? []);
  pickableRef.current = pickable ?? [];
  const partRef = useRef(part);
  partRef.current = part;
  const instancesRef = useRef(instances);
  instancesRef.current = instances;
  const componentsRef = useRef(components);
  componentsRef.current = components;
  // Every datum drawn: the part's own, and those of the parts placed in it,
  // where they are.
  const datums = useMemo(
    () => [...part.datums, ...instances.flatMap((i) => placedDatums(i, components[i.component]))],
    [part, instances, components],
  );
  const datumsRef = useRef(datums);
  datumsRef.current = datums;
  const hiddenRef = useRef(hidden ?? []);
  hiddenRef.current = hidden ?? [];
  const planeRef = useRef(plane ?? null);
  planeRef.current = plane ?? null;
  const gridRef = useRef(grid ?? true);
  gridRef.current = grid ?? true;
  const grabRef = useRef(grab ?? false);
  grabRef.current = grab ?? false;
  const onPointerRef = useRef(onPointer);
  onPointerRef.current = onPointer;
  const promptAtRef = useRef<Vec3 | null>(null);
  promptAtRef.current = prompt?.at ?? null;
  const promptElRef = useRef<HTMLDivElement | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const groupRef = useRef<THREE.Group | null>(null);
  const cameraRef = useRef<THREE.Camera | null>(null);
  const controlsRef = useRef<TrackballControls | null>(null);
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

  // The placed parts drawn: a layer of the three.js scene below, so gone
  // with it.
  const placedRef = useRef<PlacedLayer | null>(null);
  // The view cube, in the corner, and where the pointer stepped behind
  // what is in front of it last (see `Pointer.cycle`): where the hint is.
  const cubeRef = useRef<HTMLDivElement>(null);
  const [cubeTitle, setCubeTitle] = useState<string | null>(null);
  const [cycling, setCycling] = useState<{ x: number; y: number } | null>(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const placed = new PlacedLayer();
    placedRef.current = placed;

    const renderer = new THREE.WebGLRenderer({ antialias: true, stencil: true, alpha: true });
    renderer.localClippingEnabled = true;
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);
    // Labels are HTML over the canvas: crisp text, and they never catch
    // the pointer.
    const labelRenderer = new CSS2DRenderer();
    Object.assign(labelRenderer.domElement.style, { position: "absolute", top: "0", left: "0", pointerEvents: "none" });
    container.appendChild(labelRenderer.domElement);

    const threeScene = new THREE.Scene();
    // Transparent: the viewport's backdrop shows through (see `.viewport`).
    threeScene.background = null;
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

    // Trackball controls: the view tumbles freely about its target —
    // turning about its own axis too — rather than orbiting about a fixed
    // up. The controls are made afresh whenever a focus move lands, which
    // drops any momentum left over from before the move; the same holds
    // for the camera itself: the controls are made for one.
    const makeControls = (target: THREE.Vector3): TrackballControls => {
      const made = new TrackballControls(camera, renderer.domElement);
      // The left button picks: only the right turns the view, and the
      // middle moves it sideways (see the render loop).
      made.mouseButtons = { LEFT: null, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.ROTATE };
      made.rotateSpeed = 3;
      made.zoomSpeed = 1.2;
      made.panSpeed = PAN_SPEED;
      made.dynamicDampingFactor = 0.15;
      // The wheel zooms towards the pointer (see `onWheel`), not the target.
      made.noZoom = true;
      // Its keys switch what a drag does — and are the sketch's tools'.
      made.keys = ["", "", ""];
      made.target.copy(target);
      made.addEventListener("end", () => {
        onPoseRef.current?.(pose());
        sendView();
      });
      controlsRef.current = made;
      return made;
    };
    let controls = makeControls(vec(DEFAULT_POSE.target));
    const pose = (): CameraPose => ({
      position: arr(camera.position),
      target: arr(controls.target),
      up: arr(camera.up),
    });

    // A studio rig: sky over ground, a key light and a softer fill from the
    // other side, so a face turned away from the key still reads.
    threeScene.add(new THREE.HemisphereLight(0xf2f5ff, 0x30343c, 0.9));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.1);
    keyLight.position.set(5, 8, 6);
    threeScene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0xdfe8ff, 0.45);
    fillLight.position.set(-6, -2, -5);
    threeScene.add(fillLight);
    const datumLayer = new DatumLayer();
    threeScene.add(datumLayer.group);
    threeScene.add(placed.group);
    const visualLayer = new VisualLayer();
    threeScene.add(visualLayer.group);
    const gizmoLayer = new GizmoLayer();
    threeScene.add(gizmoLayer.group);
    const grid = new PlaneGrid();
    threeScene.add(grid.group);

    // A section view: the model's materials clipped by one plane, and a cap
    // drawn on it wherever the stencil counters say a solid was cut open —
    // resetting the count as it goes, so each pixel is capped once.
    const clipPlane = new THREE.Plane();
    const clipping = [clipPlane];
    const cap = new THREE.Mesh(
      new THREE.PlaneGeometry(1, 1),
      new THREE.MeshStandardMaterial({
        color: CAP_COLOR,
        side: THREE.DoubleSide,
        roughness: 0.8,
        stencilWrite: true,
        stencilRef: 0,
        stencilFunc: THREE.NotEqualStencilFunc,
        stencilFail: THREE.ReplaceStencilOp,
        stencilZFail: THREE.ReplaceStencilOp,
        stencilZPass: THREE.ReplaceStencilOp,
      }),
    );
    cap.renderOrder = 2;
    cap.visible = false;
    threeScene.add(cap);
    /** Clip every material of the part, and of the parts placed in it, by the section — or by nothing. */
    const applySection = () => {
      const cut = sectionRef.current;
      if (cut) {
        const normal = vec(cut.normal).normalize();
        clipPlane.setFromNormalAndCoplanarPoint(normal.clone().negate(), vec(cut.origin));
        const { extent } = partRef.current;
        const center = vec(extent.center);
        cap.position.copy(clipPlane.projectPoint(center, new THREE.Vector3()));
        cap.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), normal);
        cap.scale.setScalar(extent.size * 3);
      }
      cap.visible = cut != null;
      const groups = [groupRef.current, placedRef.current?.group];
      for (const group of groups) {
        group?.traverse((o) => {
          if (o.userData.stencil) o.visible = cut != null;
          const material = (o as THREE.Mesh).material as THREE.Material | undefined;
          if (material && !Array.isArray(material)) material.clippingPlanes = cut ? clipping : null;
        });
      }
    };

    const resize = () => {
      const { clientWidth, clientHeight } = container;
      renderer.setSize(clientWidth, clientHeight);
      labelRenderer.setSize(clientWidth, clientHeight);
      perspective.aspect = clientWidth / clientHeight;
      perspective.updateProjectionMatrix();
      controlsRef.current?.handleResize();
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
     * tube in an orthographic view — meaning the `cycle`-th of what lies
     * under it (see `Pointer.cycle`).
     */
    const pointerAt = (x: number, y: number, cycle = 0): Pointer => {
      const rect = container.getBoundingClientRect();
      const ndc = new THREE.Vector2(((x - rect.left) / rect.width) * 2 - 1, -((y - rect.top) / rect.height) * 2 + 1);
      const raycaster = new THREE.Raycaster();
      raycaster.setFromCamera(ndc, camera);
      const height = Math.max(container.clientHeight, 1);
      const reach: Reach =
        camera instanceof THREE.OrthographicCamera
          ? { type: "tube", radius: (REACH_PX * (camera.top - camera.bottom)) / camera.zoom / height }
          : { type: "cone", slope: (REACH_PX * 2 * Math.tan(THREE.MathUtils.degToRad(CAMERA_FOV) / 2)) / height };
      return { ray: { origin: arr(raycaster.ray.origin), dir: arr(raycaster.ray.direction) }, reach, cycle };
    };
    const send = (event: PointerEvent_) => onPointerRef.current?.(event) ?? Promise.resolve(false);
    // Which way is up on screen, for a step to name what it shows as seen:
    // sent whenever the view settles, and when the pointer comes in (a step
    // opened since then has not been told).
    const sendView = () =>
      void send({ type: "view", up: arr(new THREE.Vector3(0, 1, 0).applyQuaternion(camera.quaternion)) });
    // For the end-to-end checks, which drive the view by the pointer: where
    // a point of the scene is on screen, in client coordinates, and how
    // long a reach is there — what the kernel lays a gizmo out in.
    (window as unknown as { geopView: unknown }).geopView = {
      project: (p: Vec3) => {
        const ndc = vec(p).project(camera);
        const rect = container.getBoundingClientRect();
        return [rect.left + ((ndc.x + 1) / 2) * rect.width, rect.top + ((1 - ndc.y) / 2) * rect.height];
      },
      reach: (p: Vec3) => worldPerPixel(camera, vec(p), Math.max(container.clientHeight, 1)) * REACH_PX,
      pose: () => pose(),
      /** Whether the view is orthographic now. */
      orthographic: () => camera === orthographic,
      /** Where the view cube's region `region` (see `cubeRegion`) is on screen, in client coordinates: a point of it inside its outline. */
      cube: (region: Vec3) => {
        const rect = cubeRef.current?.getBoundingClientRect();
        if (!rect) return null;
        const many = region.filter((k) => k !== 0).length > 1;
        const ndc = new THREE.Vector3(...region.map((k) => k * (many ? 0.45 : 0.5))).project(cube.camera);
        return [rect.left + ((ndc.x + 1) / 2) * rect.width, rect.top + ((1 - ndc.y) / 2) * rect.height];
      },
      /** The names of the datums drawn. */
      datums: () => datumLayer.drawn(),
    };

    // Which of what lies under the pointer it means (see `Pointer.cycle`):
    // a right click steps it one further behind, a move away from where it
    // was stepped starts again at the front.
    let cycle = 0;
    let cycledAt: { x: number; y: number } | null = null;
    const frontmost = () => {
      cycle = 0;
      cycledAt = null;
      setCycling(null);
    };
    // A press: a click if it does not move, else the camera's — or, where
    // the hover offered a grab, a drag of what is under it.
    let press: {
      id: number;
      x: number;
      y: number;
      button: number;
      grabbed: boolean;
      from: Pointer;
      moved: boolean;
    } | null = null;
    let lastClick: { x: number; y: number; time: number } | null = null;
    // The latest pointer position, handled once per frame.
    let pendingHover: { x: number; y: number } | "leave" | null = null;
    // Shift turns snapping off: held or let go over a still pointer, the
    // hover is sent again.
    let shift = false;
    let lastHover: { x: number; y: number } | null = null;
    const onShift = (e: KeyboardEvent) => {
      if (e.shiftKey === shift) return;
      shift = e.shiftKey;
      if (lastHover && !press) pendingHover = lastHover;
    };
    window.addEventListener("keydown", onShift);
    window.addEventListener("keyup", onShift);
    /** Whether `e` is on what is drawn over the view — a prompt's input, the view cube — not the view. */
    const onOverlay = (e: PointerEvent) =>
      (e.target as HTMLElement | null)?.closest?.(".viewport-prompt, .view-cube-area") != null;
    let pendingDrag: { x: number; y: number } | null = null;
    /** Whether a hover or drag is sent and not yet answered. */
    let following = false;

    // A finger never hovers, so whether its press grabs is not known yet
    // when it lands: the press is held back from the controls while the
    // kernel is asked, and handed to them after all if it does not grab.
    let asking: number | null = null;
    let replaying = false;
    const take = (e: PointerEvent, grabbed: boolean) => {
      press = {
        id: e.pointerId,
        x: e.clientX,
        y: e.clientY,
        button: e.button,
        grabbed,
        from: pointerAt(e.clientX, e.clientY, cycle),
        moved: false,
      };
      if (grabbed) {
        // The drag is the step's: neither the controls nor a click see it.
        controls.enabled = false;
        container.setPointerCapture(e.pointerId);
      }
    };
    const onPointerDown = (e: PointerEvent) => {
      if (moveRef.current || replaying || onOverlay(e)) return;
      // A second finger is the camera's: a pinch, or a two-finger pan.
      if (e.pointerType === "touch" && (asking != null || press != null)) {
        if (press?.grabbed) e.stopPropagation();
        return;
      }
      if (e.pointerType === "touch" && e.button === 0) {
        e.stopPropagation();
        asking = e.pointerId;
        const landed = new PointerEvent("pointerdown", e);
        void send({ type: "hover", pointer: pointerAt(e.clientX, e.clientY), shift: e.shiftKey }).then((grabbed) => {
          // Lifted, or the view moved, while asking: the press is gone.
          if (asking !== e.pointerId) return;
          asking = null;
          take(e, grabbed);
          if (!grabbed) {
            replaying = true;
            renderer.domElement.dispatchEvent(landed);
            replaying = false;
          }
        });
        return;
      }
      const grabbed = e.button === 0 && grabRef.current;
      take(e, grabbed);
      if (grabbed) e.stopPropagation();
    };
    const onPointerMove = (e: PointerEvent) => {
      shift = e.shiftKey;
      if (press) {
        if (e.pointerId !== press.id) return;
        if (movedFromPress(press, { x: e.clientX, y: e.clientY })) press.moved = true;
        if (press.grabbed && press.moved) pendingDrag = { x: e.clientX, y: e.clientY };
        return;
      }
      if (onOverlay(e)) {
        if (lastHover) pendingHover = "leave";
        lastHover = null;
        return;
      }
      if (cycledAt && movedFromPress(cycledAt, { x: e.clientX, y: e.clientY })) frontmost();
      if (e.buttons === 0) pendingHover = lastHover = { x: e.clientX, y: e.clientY };
    };
    const onPointerUp = (e: PointerEvent) => {
      if (onOverlay(e) && !press) return;
      if (asking === e.pointerId) {
        // Lifted before the kernel answered: a tap.
        asking = null;
        take(e, false);
      }
      const done = press;
      if (!done || e.pointerId !== done.id) return;
      press = null;
      if (done.grabbed) {
        controls.enabled = !moveRef.current;
        pendingDrag = null;
        if (done.moved) {
          send({ type: "drag", from: done.from, to: pointerAt(e.clientX, e.clientY), done: true, shift: e.shiftKey });
          pendingHover = { x: e.clientX, y: e.clientY };
          return;
        }
      }
      if (done.moved) return;
      // A right click steps behind what the pointer is over: the next of
      // what lies under it is lit, and a left click picks that one.
      if (done.button === 2) {
        cycle += 1;
        cycledAt ??= { x: e.clientX, y: e.clientY };
        const rect = container.getBoundingClientRect();
        setCycling({ x: e.clientX - rect.left, y: e.clientY - rect.top });
        pendingHover = lastHover = { x: e.clientX, y: e.clientY };
        return;
      }
      if (done.button !== 0) return;
      const now = performance.now();
      const at = { x: e.clientX, y: e.clientY };
      const double = isDoubleClick(lastClick, at, now);
      lastClick = double ? null : { x: e.clientX, y: e.clientY, time: now };
      send({ type: "click", pointer: pointerAt(e.clientX, e.clientY, cycle), double, shift: e.shiftKey });
      frontmost();
    };
    // Taken from us — by the system, say: a drag ends where it was, a
    // press is no click.
    const onPointerCancel = (e: PointerEvent) => {
      if (asking === e.pointerId) asking = null;
      if (press?.id !== e.pointerId) return;
      if (press.grabbed && press.moved) onPointerUp(e);
      else {
        if (press.grabbed) controls.enabled = !moveRef.current;
        press = null;
      }
    };
    const onPointerLeave = () => {
      pendingHover = "leave";
      lastHover = null;
      frontmost();
    };
    const onContextMenu = (e: Event) => e.preventDefault();
    // The middle button pans: not the browser's scrolling, nor pasting.
    const onMiddle = (e: MouseEvent) => {
      if (e.button === 1) e.preventDefault();
    };
    // A press in the view is work in the view: a dialog's field it leaves
    // gives the keys back to the step.
    const onPress = () => {
      const focused = document.activeElement;
      if (focused instanceof HTMLInputElement || focused instanceof HTMLTextAreaElement) {
        // A number field has no selection (`selectionEnd` is null), and
        // setting one throws.
        if (focused.selectionEnd != null) focused.setSelectionRange(focused.selectionEnd, focused.selectionEnd);
        focused.blur();
      }
    };
    container.addEventListener("pointerdown", onPress, { capture: true });
    // The wheel zooms towards what is under the pointer, which stays under
    // it — as the desktop app zooms — and the view is told once it rests.
    let wheelRest: ReturnType<typeof setTimeout> | null = null;
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      if (moveRef.current || !controls.enabled) return;
      const rect = container.getBoundingClientRect();
      const ndc = new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
      const raycaster = new THREE.Raycaster();
      raycaster.setFromCamera(ndc, camera);
      const back = camera.position.clone().sub(controls.target).normalize();
      const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(back, controls.target);
      const under = raycaster.ray.intersectPlane(plane, new THREE.Vector3()) ?? controls.target.clone();
      const zoomed = zoomToward(arr(camera.position), arr(controls.target), arr(under), wheelSteps(e.deltaY, e.deltaMode));
      camera.position.set(...zoomed.position);
      controls.target.set(...zoomed.target);
      if (wheelRest) clearTimeout(wheelRest);
      wheelRest = setTimeout(() => {
        wheelRest = null;
        onPoseRef.current?.(pose());
        sendView();
      }, 150);
    };
    container.addEventListener("wheel", onWheel, { passive: false });
    // In the capture phase: a grab must be taken before the controls see
    // the press.
    container.addEventListener("pointerdown", onPointerDown, { capture: true });
    container.addEventListener("pointermove", onPointerMove);
    container.addEventListener("pointerup", onPointerUp);
    container.addEventListener("pointercancel", onPointerCancel);
    container.addEventListener("pointerleave", onPointerLeave);
    container.addEventListener("pointerenter", sendView);
    container.addEventListener("contextmenu", onContextMenu);
    container.addEventListener("mousedown", onMiddle);
    container.addEventListener("auxclick", onMiddle);

    // The view cube: turned as the view is, drawn in its corner after the
    // scene. A click on a face, an edge or a corner of it turns the view
    // to look from there; a drag turns the view about its target.
    const cube = new ViewCube();
    const cubeEl = cubeRef.current;
    let cubePress: { id: number; x: number; y: number; last: { x: number; y: number }; moved: boolean } | null = null;
    /** The region of the cube at the client point `(x, y)`, if the cube is there. */
    const cubeRegionAt = (x: number, y: number) => {
      const rect = cubeEl?.getBoundingClientRect();
      if (!rect) return null;
      const ndc = new THREE.Vector2(((x - rect.left) / rect.width) * 2 - 1, -((y - rect.top) / rect.height) * 2 + 1);
      return cube.regionAt(ndc);
    };
    /** Turn the view about its target as dragging the cube `dx`, `dy` pixels turns the cube. */
    const turnBy = (dx: number, dy: number) => {
      const target = controls.target;
      const eye = camera.position.clone().sub(target);
      const right = eye.clone().negate().cross(camera.up).normalize();
      const turn = new THREE.Quaternion()
        .setFromAxisAngle(camera.up.clone().normalize(), -dx * CUBE_TURN)
        .multiply(new THREE.Quaternion().setFromAxisAngle(right, -dy * CUBE_TURN));
      eye.applyQuaternion(turn);
      camera.up.applyQuaternion(turn).normalize();
      camera.position.copy(target).add(eye);
      camera.lookAt(target);
    };
    const onCubeDown = (e: PointerEvent) => {
      if (e.button !== 0 || moveRef.current) return;
      e.stopPropagation();
      cubeEl?.setPointerCapture(e.pointerId);
      cubePress = { id: e.pointerId, x: e.clientX, y: e.clientY, last: { x: e.clientX, y: e.clientY }, moved: false };
    };
    const onCubeMove = (e: PointerEvent) => {
      if (cubePress?.id === e.pointerId) {
        if (movedFromPress(cubePress, { x: e.clientX, y: e.clientY })) cubePress.moved = true;
        if (cubePress.moved) {
          turnBy(e.clientX - cubePress.last.x, e.clientY - cubePress.last.y);
          cubePress.last = { x: e.clientX, y: e.clientY };
        }
        return;
      }
      const region = cubeRegionAt(e.clientX, e.clientY);
      cube.light(region);
      setCubeTitle(region ? `${regionName(region)} — click to look from here, drag to turn` : null);
    };
    const onCubeUp = (e: PointerEvent) => {
      const done = cubePress;
      if (done?.id !== e.pointerId) return;
      cubePress = null;
      if (done.moved) {
        onPoseRef.current?.(pose());
        sendView();
        return;
      }
      const region = cubeRegionAt(e.clientX, e.clientY);
      if (region) glideRef.current?.(standardPose(region, pose()));
    };
    const onCubeLeave = () => {
      cube.light(null);
      setCubeTitle(null);
    };
    cubeEl?.addEventListener("pointerdown", onCubeDown);
    cubeEl?.addEventListener("pointermove", onCubeMove);
    cubeEl?.addEventListener("pointerup", onCubeUp);
    cubeEl?.addEventListener("pointercancel", onCubeUp);
    cubeEl?.addEventListener("pointerleave", onCubeLeave);
    /** Draws the cube into its corner of the canvas, over the scene, turned as the camera is. */
    const drawCube = () => {
      const rect = cubeEl?.getBoundingClientRect();
      if (!rect || rect.width === 0) return;
      const canvas = renderer.domElement.getBoundingClientRect();
      cube.orient(camera.quaternion);
      const [x, y] = [rect.left - canvas.left, canvas.bottom - rect.bottom];
      renderer.setScissorTest(true);
      renderer.setViewport(x, y, rect.width, rect.height);
      renderer.setScissor(x, y, rect.width, rect.height);
      renderer.autoClear = false;
      renderer.clearDepth();
      renderer.render(cube.scene, cube.camera);
      renderer.autoClear = true;
      renderer.setScissorTest(false);
      const size = renderer.getSize(new THREE.Vector2());
      renderer.setViewport(0, 0, size.x, size.y);
    };

    /** What the datums were last drawn from (see the loop below), and which of them are lit. */
    const datumsDrawn: { inputs: unknown[]; lit: string[] } = { inputs: [], lit: [] };
    let lastFrame = performance.now();
    let frame = requestAnimationFrame(function animate() {
      const now = performance.now();
      // Momentum keeps 0.85 of itself per sixtieth of a second, whatever
      // the display's rate: a tumble let go glides as far at 120 Hz as at
      // 60, and as far as in the desktop app.
      const dt = Math.min(0.1, (now - lastFrame) / 1000);
      lastFrame = now;
      controls.dynamicDampingFactor = 1 - Math.pow(0.85, dt * 60);
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
          sendView();
        }
      } else {
        // Working in a plane, the view stays head on to it: the right
        // button pans too.
        const inPlane = planeRef.current != null;
        controls.noRotate = inPlane;
        controls.mouseButtons.RIGHT = inPlane ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
        // The controls scale an orthographic camera's pan by its width over
        // the viewport's, as if its size were fixed; ours is sized from its
        // distance (see `fitOrthographic`), which they scale by already.
        const width = orthographic.right - orthographic.left;
        controls.panSpeed = camera === orthographic && width > 0 ? (PAN_SPEED * container.clientWidth) / width : PAN_SPEED;
        controls.update();
      }
      // Near and far follow the distance to the target and the drawing's
      // size, so that neither a large assembly nor a close look is clipped.
      const reach = camera.position.distanceTo(controls.target) + partRef.current.extent.size;
      perspective.near = reach / 10000;
      perspective.far = reach * 10;
      // The middle of the view is the middle of what is not covered: the
      // picture shifted up by half the covered height.
      const lift = Math.round(coveredRef.current / 2);
      const { clientWidth: w, clientHeight: h } = container;
      for (const c of [perspective, orthographic]) {
        if (lift > 0 && w > 0 && h > 0) c.setViewOffset(w, h, 0, lift, w, h);
        else if (c.view?.enabled) c.clearViewOffset();
      }
      perspective.updateProjectionMatrix();
      orthographic.near = -reach * 10;
      orthographic.far = reach * 10;
      if (camera === orthographic) fitOrthographic();
      const height = container.clientHeight;

      // One hover or drag in flight at a time, the latest pointer sent once
      // it is answered: a kernel slower than a frame — a native one, a
      // message away — falls behind by one answer, not by a growing queue.
      if (!following) {
        const hover = pendingHover;
        pendingHover = null;
        const drag = pendingDrag;
        pendingDrag = null;
        const event: PointerEvent_ | null =
          drag && press
            ? { type: "drag", from: press.from, to: pointerAt(drag.x, drag.y), done: false, shift }
            : hover === "leave"
              ? { type: "leave" }
              : hover
                ? { type: "hover", pointer: pointerAt(hover.x, hover.y, cycle), shift }
                : null;
        if (event) {
          following = true;
          void send(event).finally(() => (following = false));
        }
      }
      renderer.domElement.style.cursor = press?.grabbed ? "grabbing" : grabRef.current ? "grab" : "";

      // Which datums are drawn, and lit, only when what decides it
      // changed: not every frame — an assembly has a datum per part.
      const datumInputs = [
        litRef.current,
        instancesRef.current,
        componentsRef.current,
        partRef.current,
        hiddenRef.current,
        datumsRef.current,
      ];
      if (datumInputs.some((input, k) => input !== datumsDrawn.inputs[k])) {
        const lit = litDatums(litRef.current);
        // A hidden datum is shown while it, or a component of it (its
        // axis, its plane), is lit — as the desktop app shows it.
        const litShows = (name: string) => lit.some((l) => l === name || l.startsWith(`${name}:`));
        const hidden = new Set(hiddenRef.current.filter((name) => !litShows(name)));
        const { extent } = partRef.current;
        datumLayer.sync(datumsRef.current, hidden, extent.size, vec(extent.center));
        datumsDrawn.inputs = datumInputs;
        datumsDrawn.lit = lit;
      }
      datumLayer.update(camera, height, datumKinds(pickableRef.current), datumsDrawn.lit);
      visualLayer.sync(visualsRef.current);
      visualLayer.update(camera, height, controls.target);
      gizmoLayer.sync(gizmoRef.current);
      gizmoLayer.update(camera, height);
      placed.detail(camera, height);
      grid.sync(gridRef.current ? planeRef.current : null);
      grid.update(camera, height, controls.target);

      // The prompt's input, where its point is on screen.
      const promptAt = promptAtRef.current;
      const promptEl = promptElRef.current;
      if (promptAt && promptEl) {
        const ndc = vec(promptAt).project(camera);
        const x = ((ndc.x + 1) / 2) * container.clientWidth;
        const y = ((1 - ndc.y) / 2) * height;
        promptEl.style.transform = `translate(${x}px, ${y}px) translate(-50%, -50%)`;
      }

      applySection();
      renderer.render(threeScene, camera);
      drawCube();
      labelRenderer.render(threeScene, camera);
      frame = requestAnimationFrame(animate);
    });

    return () => {
      cancelAnimationFrame(frame);
      resizeObserver.disconnect();
      container.removeEventListener("pointerdown", onPointerDown, { capture: true });
      container.removeEventListener("pointermove", onPointerMove);
      container.removeEventListener("pointerup", onPointerUp);
      container.removeEventListener("pointercancel", onPointerCancel);
      container.removeEventListener("pointerleave", onPointerLeave);
      container.removeEventListener("pointerenter", sendView);
      container.removeEventListener("contextmenu", onContextMenu);
      container.removeEventListener("mousedown", onMiddle);
      container.removeEventListener("auxclick", onMiddle);
      cubeEl?.removeEventListener("pointerdown", onCubeDown);
      cubeEl?.removeEventListener("pointermove", onCubeMove);
      cubeEl?.removeEventListener("pointerup", onCubeUp);
      cubeEl?.removeEventListener("pointercancel", onCubeUp);
      cubeEl?.removeEventListener("pointerleave", onCubeLeave);
      cube.dispose();
      container.removeEventListener("wheel", onWheel);
      container.removeEventListener("pointerdown", onPress, { capture: true });
      if (wheelRest) clearTimeout(wheelRest);
      window.removeEventListener("keydown", onShift);
      window.removeEventListener("keyup", onShift);
      controls.dispose();
      renderer.dispose();
      container.removeChild(renderer.domElement);
      container.removeChild(labelRenderer.domElement);
      sceneRef.current = null;
      cameraRef.current = null;
      controlsRef.current = null;
      // The placed parts were drawn in this scene: built again in the next.
      placed.dispose();
      placedRef.current = null;
    };
  }, []);

  /** Glide from where the camera is to `to`. */
  const glideTo = (to: CameraPose) => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    if (!camera || !controls) return;
    controls.enabled = false;
    const from: CameraPose = { position: arr(camera.position), target: arr(controls.target), up: arr(camera.up) };
    moveRef.current = {
      from,
      to,
      fromQuat: cameraOrientation(from),
      toQuat: cameraOrientation(to),
      start: performance.now(),
    };
  };

  // The view cube glides too: through this, made once the camera is.
  const glideRef = useRef(glideTo);
  glideRef.current = glideTo;

  // Start gliding whenever a new focus arrives.
  useEffect(() => {
    if (focus) glideTo(focus);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [focus]);

  // Frame the drawing whenever asked to.
  useEffect(() => {
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    const container = containerRef.current;
    if (fit == null || !camera || !controls || !container) return;
    const now: CameraPose = { position: arr(camera.position), target: arr(controls.target), up: arr(camera.up) };
    const aspect = container.clientWidth / Math.max(container.clientHeight, 1);
    glideTo(fitPose(partRef.current.extent, fit.from ?? now, aspect));
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [fit]);

  // Swap in the current part's geometry, leaving camera and controls alone.
  // A solid hidden is not built at all: what is hidden is part of it.
  const hiddenKey = JSON.stringify(hidden ?? []);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  const scene = useMemo(() => flatten(part, hidden ?? []), [part, hiddenKey]);
  /** The scene the group shown was built of: its faces are taken over by the next, if it draws them alike. */
  const builtRef = useRef<Scene | null>(null);
  useEffect(() => {
    const threeScene = sceneRef.current;
    if (!threeScene) return;
    const previous = groupRef.current;
    const group = buildSceneGroup(
      scene,
      previous && builtRef.current ? { scene: builtRef.current, group: previous } : undefined,
    );
    if (previous) {
      threeScene.remove(previous);
      disposeGroup(previous);
    }
    builtRef.current = scene;
    groupRef.current = group;
    threeScene.add(group);
    applyHighlight(group, scene, litRef.current[""] ?? null, hiddenRef.current);
  }, [scene]);

  // The placed parts: each component built once, its placed parts drawn
  // together, and those lit or partly hidden on their own (see `placed3d.ts`).
  /** How each placed part looks, as what is lit and what is hidden say. */
  const placedLook = (): ((name: string) => PlacedLook) => {
    const lit = litRef.current;
    const hidden = new Set(hiddenRef.current);
    // What is hidden of each placed part, by its name: what follows its
    // name — of the sketches it draws, the only thing of it a name hides.
    const hiddenIn = new Map<string, string[]>();
    for (const h of hiddenRef.current) {
      const { instance, name } = parseKey(h);
      if (!instance) continue;
      const list = hiddenIn.get(instance) ?? [];
      list.push(name);
      hiddenIn.set(instance, list);
    }
    const component = new Map(instancesRef.current.map((i) => [i.name, components[i.component]]));
    return (name) => {
      const view = component.get(name);
      const sketches = new Set(view?.sketches.map((sketch) => sketch.name));
      return {
        lit: lit[name] ?? null,
        hidden: (hiddenIn.get(name) ?? []).filter((h) => sketches.has(h)),
        // Hidden with the part it is placed in, as the kernel picks it.
        visible: !name.split("/").some((_, k, path) => hidden.has(path.slice(0, k + 1).join("/"))),
      };
    };
  };
  useEffect(() => {
    placedRef.current?.update(instances, components, placedLook());
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [instances, components]);

  // Re-tint in place when what is lit or what is hidden changes — sent
  // anew only when it changed, so compared by identity: nothing is rebuilt.
  useEffect(() => {
    if (groupRef.current) applyHighlight(groupRef.current, scene, litRef.current[""] ?? null, hiddenRef.current);
    placedRef.current?.light(placedLook());
    // `scene` is the one the group was built from: a new one rebuilds it above.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [lit, hidden]);

  // No touch is the browser's to scroll or zoom the page by: every one is the view's.
  return (
    <div
      ref={containerRef}
      className="scene-view"
      style={{ position: "relative", width: "100%", height: "100%", minHeight: 0, touchAction: "none" }}
    >
      <div className="view-cube-area">
        <button
          className="view-home"
          title={withShortcut("Home: the whole part, from the standard corner", homeKey)}
          aria-label="Home view"
          onClick={() => onHome?.()}
        >
          ⌂
        </button>
        <div
          ref={cubeRef}
          className="view-cube"
          role="img"
          aria-label="View cube"
          title={cubeTitle ?? "The view's orientation: click a face, an edge or a corner to look from there, drag to turn"}
        />
      </div>
      {cycling && cycle && cycle.of > 1 && (
        <div className="cycle-hint" style={{ left: cycling.x + 14, top: cycling.y + 14 }}>
          {cycle.at + 1}/{cycle.of} — left click to pick
        </div>
      )}
      {prompt && (
        <div ref={promptElRef} className="viewport-prompt">
          <input
            key={`${prompt.key}:${prompt.label}`}
            autoFocus
            defaultValue={prompt.value}
            title={`${prompt.label} — a number, or a formula of the parameters · Enter applies, Esc cancels`}
            onFocus={(e) => e.currentTarget.select()}
            onKeyDown={(e) => {
              e.stopPropagation();
              if (e.key === "Enter") onPrompt?.(prompt.key, e.currentTarget.value);
              if (e.key === "Escape") onPromptCancel?.();
            }}
          />
        </div>
      )}
    </div>
  );
}
