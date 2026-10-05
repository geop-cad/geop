import { useEffect, useMemo, useRef } from "react";
import * as THREE from "three";
import { TrackballControls } from "three/examples/jsm/controls/TrackballControls.js";
import { CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";
import { CAMERA_FOV, DEFAULT_POSE, REACH_PX, fitPose, worldPerPixel, type CameraPose, type Projection } from "./camera";
import { DatumLayer } from "./datums3d";
import {
  sameEntity,
  type DatumInfo,
  type DatumKind,
  type EntityRef,
  type Frame,
  type PartView,
  type Pointer,
  type PointerEvent_,
  type Prompt,
  type Reach,
  type Role,
  type Vec3,
  type ViewInstance,
  type Visual,
  type GizmoView,
} from "./geop";
import { CAP_COLOR, applyHighlight, buildSceneGroup, disposeGroup, flatten, frameMatrix, localTo } from "./partScene";
import { PlacedLayer, type PlacedLook } from "./placed3d";
import type { SectionPlane } from "./section";
import { PlaneGrid } from "./planeGrid";
import { VisualLayer } from "./visuals3d";
import { GizmoLayer } from "./gizmo3d";

/** How far, in pixels, a press may move and still be a click. */
const CLICK_PX = 4;
/** How soon, in milliseconds, a second click makes a double click. */
const DOUBLE_MS = 350;

/** The kinds of datum a click can pick something of, looking for `roles`: a frame's origin, axes and planes are points, lines and planes too. */
function datumKinds(roles: Role[]): DatumKind[] {
  const kinds: DatumKind[] = [];
  if (roles.includes("point")) kinds.push("point");
  if (roles.includes("line")) kinds.push("axis");
  if (roles.includes("plane")) kinds.push("plane");
  if (kinds.length) kinds.push("frame");
  return kinds;
}

/** How long a [[Props.focus]] move takes, in milliseconds. */
const FOCUS_MS = 550;

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
  /** What to draw highlighted — what is picked, what a click would pick. */
  highlights?: EntityRef[];
  /** What a click picks right now: reference geometry of these kinds stands out, the rest fades. */
  pickable?: Role[];
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
    name: `${instance.name}/${d.name}`,
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
  highlights,
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
}: Props) {
  const sectionRef = useRef(section ?? null);
  sectionRef.current = section ?? null;
  const containerRef = useRef<HTMLDivElement>(null);
  // Read inside the effects via refs so a prop change doesn't tear down
  // anything.
  const visualsRef = useRef(visuals ?? []);
  visualsRef.current = visuals ?? [];
  const gizmoRef = useRef(gizmo ?? null);
  gizmoRef.current = gizmo ?? null;
  const highlightsRef = useRef(highlights ?? []);
  highlightsRef.current = highlights ?? [];
  const pickableRef = useRef(pickable ?? []);
  pickableRef.current = pickable ?? [];
  const partRef = useRef(part);
  partRef.current = part;
  const instancesRef = useRef(instances);
  instancesRef.current = instances;
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

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const placed = new PlacedLayer();
    placedRef.current = placed;

    const renderer = new THREE.WebGLRenderer({ antialias: true, stencil: true });
    renderer.localClippingEnabled = true;
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

    // Trackball controls: the view tumbles freely about its target —
    // turning about its own axis too — rather than orbiting about a fixed
    // up. The controls are made afresh whenever a focus move lands, which
    // drops any momentum left over from before the move; the same holds
    // for the camera itself: the controls are made for one.
    const makeControls = (target: THREE.Vector3): TrackballControls => {
      const made = new TrackballControls(camera, renderer.domElement);
      made.rotateSpeed = 3;
      made.zoomSpeed = 1.2;
      made.panSpeed = 0.6;
      made.dynamicDampingFactor = 0.15;
      // Its keys switch what a drag does — and are the sketch's tools'.
      made.keys = ["", "", ""];
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
    const send = (event: PointerEvent_) => onPointerRef.current?.(event) ?? Promise.resolve(false);
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
    /** Whether `e` is on the input of a prompt, not the view. */
    const inPrompt = (e: PointerEvent) => (e.target as HTMLElement | null)?.closest?.(".viewport-prompt") != null;
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
        from: pointerAt(e.clientX, e.clientY),
        moved: false,
      };
      if (grabbed) {
        // The drag is the step's: neither the controls nor a click see it.
        controls.enabled = false;
        container.setPointerCapture(e.pointerId);
      }
    };
    const onPointerDown = (e: PointerEvent) => {
      if (moveRef.current || replaying || inPrompt(e)) return;
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
        if (Math.hypot(e.clientX - press.x, e.clientY - press.y) > CLICK_PX) press.moved = true;
        if (press.grabbed && press.moved) pendingDrag = { x: e.clientX, y: e.clientY };
        return;
      }
      if (e.buttons === 0) pendingHover = lastHover = { x: e.clientX, y: e.clientY };
    };
    const onPointerUp = (e: PointerEvent) => {
      if (inPrompt(e)) return;
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
    };
    const onContextMenu = (e: Event) => e.preventDefault();
    // In the capture phase: a grab must be taken before the controls see
    // the press.
    container.addEventListener("pointerdown", onPointerDown, { capture: true });
    container.addEventListener("pointermove", onPointerMove);
    container.addEventListener("pointerup", onPointerUp);
    container.addEventListener("pointercancel", onPointerCancel);
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
        controls.noRotate = inPlane;
        controls.mouseButtons.LEFT = inPlane ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
        controls.update();
      }
      // Near and far follow the distance to the target and the drawing's
      // size, so that neither a large assembly nor a close look is clipped.
      const reach = camera.position.distanceTo(controls.target) + partRef.current.extent.size;
      perspective.near = reach / 10000;
      perspective.far = reach * 10;
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
                ? { type: "hover", pointer: pointerAt(hover.x, hover.y), shift }
                : null;
        if (event) {
          following = true;
          void send(event).finally(() => (following = false));
        }
      }
      renderer.domElement.style.cursor = press?.grabbed ? "grabbing" : grabRef.current ? "grab" : "";

      const lit = highlightsRef.current;
      const hidden = hiddenRef.current.filter((name) => !lit.some((l) => sameEntity(l, { type: "Datum", name })));
      const { extent } = partRef.current;
      datumLayer.sync(datumsRef.current, hidden, extent.size, vec(extent.center));
      datumLayer.update(camera, height, datumKinds(pickableRef.current), lit);
      visualLayer.sync(visualsRef.current);
      visualLayer.update(camera, height, controls.target);
      gizmoLayer.sync(gizmoRef.current);
      gizmoLayer.update(camera, height);
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
      container.removeEventListener("contextmenu", onContextMenu);
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

  // The placed parts: each component built once, its placed parts drawn
  // together, and those lit or partly hidden on their own (see `placed3d.ts`).
  /** How each placed part looks, as the highlights, what is hidden and the visuals lighting them say. */
  const placedLook = (): ((name: string) => PlacedLook) => {
    const lit = new Set(
      visualsRef.current.flatMap((v) =>
        v.shape === "instance" && (v.style === "hover" || v.style === "selected") ? [v.name] : [],
      ),
    );
    const hidden = new Set(hiddenRef.current);
    // What is hidden of each placed part, by its name: what follows its
    // name — of the sketches it draws, the only thing of it a name hides.
    const hiddenIn = new Map<string, string[]>();
    for (const h of hiddenRef.current) {
      const at = h.lastIndexOf("/");
      if (at < 0) continue;
      const list = hiddenIn.get(h.slice(0, at)) ?? [];
      list.push(h.slice(at + 1));
      hiddenIn.set(h.slice(0, at), list);
    }
    const component = new Map(instancesRef.current.map((i) => [i.name, components[i.component]]));
    const highlighted = highlightsRef.current;
    return (name) => {
      const view = component.get(name);
      const highlights = highlighted.length > 0 ? localTo(name, highlighted) : [];
      if (lit.has(name)) {
        highlights.push(...(view?.solids ?? []).map((solid): EntityRef => ({ type: "Solid", name: solid })));
      }
      const sketches = new Set(view?.sketches.map((sketch) => sketch.name));
      return {
        highlights,
        hidden: (hiddenIn.get(name) ?? []).filter((h) => sketches.has(h)),
        visible: !hidden.has(name),
      };
    };
  };
  useEffect(() => {
    placedRef.current?.update(instances, components, placedLook());
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [instances, components]);

  // Re-tint in place when the highlights, what is hidden or what the
  // visuals light change: nothing is rebuilt.
  const appearanceKey = JSON.stringify([
    highlights ?? [],
    hidden ?? [],
    (visuals ?? []).flatMap((v) => (v.shape === "instance" ? [[v.name, v.style]] : [])),
  ]);
  useEffect(() => {
    if (groupRef.current) applyHighlight(groupRef.current, scene, highlightsRef.current, hiddenRef.current);
    placedRef.current?.light(placedLook());
    // `scene` is the one the group was built from: a new one rebuilds it above.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [appearanceKey]);

  // No touch is the browser's to scroll or zoom the page by: every one is the view's.
  return (
    <div
      ref={containerRef}
      style={{ position: "relative", width: "100%", height: "100%", minHeight: 0, touchAction: "none" }}
    >
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
