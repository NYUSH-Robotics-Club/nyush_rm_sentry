import { PanelExtensionContext } from "@foxglove/extension";
import { ros2humble } from "@foxglove/rosmsg-msgs-common";
import { CSSProperties, ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  Grid,
  gridCellAt,
  gridCorners,
  gridOriginInFrame,
  invertTransform,
  Point2D,
  Pose,
  Transform2D,
  yawOf,
} from "./mapMath";
import { asBytes, CloudPoint, drawPointCloud, PointCloud2, samplePointCloud } from "./pointCloud";

const COMMAND_TOPIC = "/odin_dashboard/command";
const GOAL_TOPIC = "/goal_pose";

type Mode = "mapping" | "relocalization" | "nav";
type Status = {
  world: string;
  selected_mode: Mode;
  active_mode: Mode | null;
  phase: string;
  detail: string;
  odin_live: boolean;
  localized: boolean;
  nav_ready: boolean;
  saving: boolean;
  goal_state: string;
  odin_map_exists: boolean;
  nav_map_exists: boolean;
};
type PoseStamped = { header: { frame_id: string }; pose: Pose };
type Path = { header: { frame_id: string }; poses: PoseStamped[] };
type TFMessage = {
  transforms: {
    header: { frame_id: string };
    child_frame_id: string;
    transform: { translation: Point2D; rotation: Pose["orientation"] };
  }[];
};
type Layers = { global: boolean; local: boolean; tf: boolean; path: boolean };
type View = { minX: number; maxY: number; scale: number; offsetX: number; offsetY: number };
type Target = Point2D & { yaw: number };
type CompressedImage = { format: string; data: Uint8Array | readonly number[] | ArrayBuffer };
type MapGesture = { pointerId: number; kind: "pan" | "goal"; start: Point2D; last: Point2D; moved: boolean };
type SplitGesture = { pointerId: number; startX: number; startWidth: number };

const EMPTY_STATUS: Status = {
  world: "—", selected_mode: "mapping", active_mode: null, phase: "disconnected",
  detail: "Waiting for Odin dashboard data", odin_live: false, localized: false,
  nav_ready: false, saving: false, goal_state: "idle", odin_map_exists: false, nav_map_exists: false,
};

function makeGridImage(grid: Grid, kind: "map" | "costmap"): HTMLCanvasElement {
  const width = grid.info.width;
  const height = grid.info.height;
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d");
  if (!ctx) { return canvas; }
  const image = ctx.createImageData(width, height);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const value = grid.data[y * width + x] ?? -1;
      const index = ((height - 1 - y) * width + x) * 4;
      if (kind === "costmap") {
        if (value <= 0) { continue; }
        image.data[index] = value >= 80 ? 239 : 244;
        image.data[index + 1] = value >= 80 ? 65 : 153;
        image.data[index + 2] = value >= 80 ? 68 : 55;
        image.data[index + 3] = value >= 80 ? 205 : Math.min(190, 45 + value * 2);
      } else {
        const color = value < 0 ? 174 : Math.round(245 - (value / 100) * 213);
        image.data[index] = color;
        image.data[index + 1] = color;
        image.data[index + 2] = color;
        image.data[index + 3] = 255;
      }
    }
  }
  ctx.putImageData(image, 0, 0);
  return canvas;
}

function mapView(grid: Grid, width: number, height: number, zoom: number, pan: Point2D): View {
  const corners = gridCorners(grid);
  const minX = Math.min(...corners.map((point) => point.x));
  const maxX = Math.max(...corners.map((point) => point.x));
  const minY = Math.min(...corners.map((point) => point.y));
  const maxY = Math.max(...corners.map((point) => point.y));
  const scale = Math.min((width - 48) / Math.max(maxX - minX, 0.1),
                         (height - 48) / Math.max(maxY - minY, 0.1)) * zoom;
  const drawWidth = (maxX - minX) * scale;
  const drawHeight = (maxY - minY) * scale;
  return { minX, maxY, scale, offsetX: (width - drawWidth) / 2 + pan.x,
           offsetY: (height - drawHeight) / 2 + pan.y };
}

function worldToScreen(point: Point2D, view: View): Point2D {
  return { x: view.offsetX + (point.x - view.minX) * view.scale,
           y: view.offsetY + (view.maxY - point.y) * view.scale };
}

function screenToWorld(point: Point2D, view: View): Point2D {
  return { x: view.minX + (point.x - view.offsetX) / view.scale,
           y: view.maxY - (point.y - view.offsetY) / view.scale };
}

function drawGrid(ctx: CanvasRenderingContext2D, grid: Grid, bitmap: HTMLCanvasElement,
                  view: View, frameTransform?: Transform2D): void {
  const origin = gridOriginInFrame(grid, frameTransform);
  const screen = worldToScreen(origin, view);
  const width = grid.info.width * grid.info.resolution * view.scale;
  const height = grid.info.height * grid.info.resolution * view.scale;
  ctx.save();
  ctx.translate(screen.x, screen.y);
  ctx.rotate(-origin.yaw);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(bitmap, 0, -height, width, height);
  ctx.restore();
}

function drawFrameAxes(ctx: CanvasRenderingContext2D, point: Point2D, yaw: number,
                       label: string): void {
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-yaw);
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#e65a4f";
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(18, 0); ctx.stroke();
  ctx.strokeStyle = "#30a879";
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(0, -18); ctx.stroke();
  ctx.restore();
  ctx.fillStyle = "#1d4661";
  ctx.font = "11px sans-serif";
  ctx.fillText(label, point.x + 5, point.y + 26);
}

function badge(label: string, state: "ok" | "off"): ReactElement {
  return <div className="odin-badge"><span className={state === "ok" ? "odin-dot good" : "odin-dot"} />{label}</div>;
}

function OdinDashboard({ context }: { context: PanelExtensionContext }): ReactElement {
  const [status, setStatus] = useState<Status>(EMPTY_STATUS);
  const [lastStatusAt, setLastStatusAt] = useState(0);
  const [now, setNow] = useState(Date.now());
  const [savedMap, setSavedMap] = useState<Grid>();
  const [globalCostmap, setGlobalCostmap] = useState<Grid>();
  const [localCostmap, setLocalCostmap] = useState<Grid>();
  const [robotPose, setRobotPose] = useState<PoseStamped>();
  const [path, setPath] = useState<Path>();
  const [mapOdom, setMapOdom] = useState<Transform2D>();
  const [layers, setLayers] = useState<Layers>({ global: true, local: false, tf: true, path: true });
  const [target, setTarget] = useState<Target>();
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState<Point2D>({ x: 0, y: 0 });
  const [leftWidth, setLeftWidth] = useState<number>();
  const [isPanning, setIsPanning] = useState(false);
  const [cameraUrl, setCameraUrl] = useState<string>();
  const [cameraAt, setCameraAt] = useState(0);
  const [cameraError, setCameraError] = useState("");
  const [cloudPoints, setCloudPoints] = useState<CloudPoint[]>([]);
  const [cloudFrame, setCloudFrame] = useState("");
  const [cloudAt, setCloudAt] = useState(0);
  const [cloudSize, setCloudSize] = useState({ width: 320, height: 240 });
  const [panelError, setPanelError] = useState("");
  const [renderDone, setRenderDone] = useState<(() => void)>();
  const [size, setSize] = useState({ width: 600, height: 520 });
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const mapBoxRef = useRef<HTMLDivElement>(null);
  const contentRef = useRef<HTMLDivElement>(null);
  const mapGesture = useRef<MapGesture>();
  const splitGesture = useRef<SplitGesture>();
  const wheelHandler = useRef<(event: WheelEvent) => void>();
  const viewRef = useRef<View>();
  const cloudCanvasRef = useRef<HTMLCanvasElement>(null);
  const lastCameraDecodeAt = useRef(0);
  const lastCloudDecodeAt = useRef(0);

  const online = now - lastStatusAt < 3000;
  const map = savedMap;
  const mapFrame = map?.header.frame_id;
  const baseBitmap = useMemo(() => map ? makeGridImage(map, "map") : undefined, [map]);
  const globalBitmap = useMemo(() => globalCostmap ? makeGridImage(globalCostmap, "costmap") : undefined, [globalCostmap]);
  const localBitmap = useMemo(() => localCostmap ? makeGridImage(localCostmap, "costmap") : undefined, [localCostmap]);

  useLayoutEffect(() => {
    context.subscribe([
      { topic: "/odin_dashboard/status" }, { topic: "/odin_dashboard/map" },
      { topic: "/odin_dashboard/pose" },
      { topic: "/global_costmap/costmap" }, { topic: "/local_costmap/costmap" },
      { topic: "/plan" }, { topic: "/tf" },
      { topic: "/odin1/image/compressed" }, { topic: "/odin1/cloud_slam" },
    ]);
    context.watch("currentFrame");
    context.onRender = (renderState, done) => {
      for (const event of renderState.currentFrame ?? []) {
        const message = event.message;
        switch (event.topic) {
          case "/odin_dashboard/status":
            try {
              setStatus(JSON.parse((message as { data: string }).data) as Status);
              setLastStatusAt(Date.now());
            } catch { setPanelError("Invalid status message"); }
            break;
          case "/odin_dashboard/map": setSavedMap(message as Grid); break;
          case "/global_costmap/costmap": setGlobalCostmap(message as Grid); break;
          case "/local_costmap/costmap": setLocalCostmap(message as Grid); break;
          case "/odin_dashboard/pose": setRobotPose(message as PoseStamped); break;
          case "/plan": setPath(message as Path); break;
          case "/odin1/image/compressed": {
            const receivedAt = Date.now();
            if (receivedAt - lastCameraDecodeAt.current < 200) { break; }
            lastCameraDecodeAt.current = receivedAt;
            const camera = message as CompressedImage;
            if (!camera.format.toLowerCase().includes("jpeg")) {
              setCameraError(`Unsupported camera format: ${camera.format}`);
              break;
            }
            const bytes = asBytes(camera.data);
            if (bytes.byteLength === 0) { break; }
            setCameraUrl(URL.createObjectURL(new Blob([Uint8Array.from(bytes)], { type: "image/jpeg" })));
            setCameraAt(receivedAt);
            setCameraError("");
            break;
          }
          case "/odin1/cloud_slam": {
            const receivedAt = Date.now();
            if (receivedAt - lastCloudDecodeAt.current < 200) { break; }
            lastCloudDecodeAt.current = receivedAt;
            const cloud = message as PointCloud2;
            setCloudPoints(samplePointCloud(cloud));
            setCloudFrame(cloud.header.frame_id);
            setCloudAt(receivedAt);
            break;
          }
          case "/tf": {
            const transforms = (message as TFMessage).transforms;
            for (const transform of transforms) {
              const tf = { x: transform.transform.translation.x, y: transform.transform.translation.y,
                           yaw: yawOf(transform.transform.rotation) };
              if (transform.header.frame_id === "map" && transform.child_frame_id === "odom") {
                setMapOdom(tf);
              } else if (transform.header.frame_id === "odom" && transform.child_frame_id === "map") {
                setMapOdom(invertTransform(tf));
              }
            }
            break;
          }
        }
      }
      setRenderDone(() => done);
    };
    return () => { context.subscribe([]); context.onRender = undefined; };
  }, [context]);

  useEffect(() => { renderDone?.(); }, [renderDone]);
  useEffect(() => () => { if (cameraUrl) { URL.revokeObjectURL(cameraUrl); } }, [cameraUrl]);
  useEffect(() => {
    setTarget(undefined);
    setRobotPose(undefined);
    setGlobalCostmap(undefined);
    setLocalCostmap(undefined);
    setPath(undefined);
    setMapOdom(undefined);
    setCloudPoints([]);
    setCloudAt(0);
  }, [status.active_mode]);
  useEffect(() => {
    const timer = window.setInterval(() => { setNow(Date.now()); }, 1000);
    return () => { window.clearInterval(timer); };
  }, []);
  useEffect(() => {
    const box = mapBoxRef.current;
    if (!box) { return; }
    const observer = new ResizeObserver(() => { setSize({ width: box.clientWidth, height: box.clientHeight }); });
    observer.observe(box);
    return () => { observer.disconnect(); };
  }, []);

  useEffect(() => {
    if (!map || !baseBitmap || !canvasRef.current) { return; }
    const canvas = canvasRef.current;
    const dpr = window.devicePixelRatio > 0 ? window.devicePixelRatio : 1;
    canvas.width = Math.max(1, Math.round(size.width * dpr));
    canvas.height = Math.max(1, Math.round(size.height * dpr));
    const ctx = canvas.getContext("2d");
    if (!ctx) { return; }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, size.width, size.height);
    const view = mapView(map, size.width, size.height, zoom, pan);
    viewRef.current = view;
    drawGrid(ctx, map, baseBitmap, view);
    if (layers.global && globalCostmap && globalBitmap && globalCostmap.header.frame_id === mapFrame) {
      drawGrid(ctx, globalCostmap, globalBitmap, view);
    }
    if (layers.local && localCostmap && localBitmap) {
      const tf = localCostmap.header.frame_id === mapFrame ? undefined :
        localCostmap.header.frame_id === "odom" && mapFrame === "map" ? mapOdom : undefined;
      if (localCostmap.header.frame_id === mapFrame || tf) {
        drawGrid(ctx, localCostmap, localBitmap, view, tf);
      }
    }
    if (layers.path && path && path.poses.length > 0 && path.header.frame_id === mapFrame) {
      ctx.strokeStyle = "#3578d5";
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      path.poses.forEach((item, index) => {
        const p = worldToScreen(item.pose.position, view);
        if (index === 0) { ctx.moveTo(p.x, p.y); } else { ctx.lineTo(p.x, p.y); }
      });
      ctx.stroke();
    }
    if (layers.tf) {
      drawFrameAxes(ctx, worldToScreen({ x: 0, y: 0 }, view), 0, mapFrame ?? "map");
      if (mapFrame === "map" && mapOdom) {
        drawFrameAxes(ctx, worldToScreen(mapOdom, view), mapOdom.yaw, "odom");
      }
    }
    if (layers.tf && robotPose && robotPose.header.frame_id === mapFrame) {
      const p = worldToScreen(robotPose.pose.position, view);
      const yaw = yawOf(robotPose.pose.orientation);
      ctx.save();
      ctx.translate(p.x, p.y);
      ctx.rotate(-yaw);
      ctx.fillStyle = "#1d6ded";
      ctx.strokeStyle = "white";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(16, 0); ctx.lineTo(-10, -9); ctx.lineTo(-7, 0); ctx.lineTo(-10, 9);
      ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.restore();
      drawFrameAxes(ctx, p, yaw, status.active_mode === "nav" ? "base_link" : "imu");
    }
    if (target) {
      const p = worldToScreen(target, view);
      ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(-target.yaw);
      ctx.strokeStyle = "#e55440"; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(0, 0, 8, 0, Math.PI * 2); ctx.moveTo(0, 0); ctx.lineTo(25, 0); ctx.stroke();
      ctx.restore();
    }
  }, [map, baseBitmap, globalCostmap, globalBitmap, localCostmap, localBitmap, robotPose,
      path, mapOdom, mapFrame, target, layers, size, zoom, pan, status.active_mode]);

  useEffect(() => {
    const body = cloudCanvasRef.current?.parentElement;
    if (!body) { return; }
    const observer = new ResizeObserver(() => {
      const dpr = window.devicePixelRatio > 0 ? window.devicePixelRatio : 1;
      setCloudSize({
        width: Math.max(1, Math.round(body.clientWidth * dpr)),
        height: Math.max(1, Math.round(body.clientHeight * dpr)),
      });
    });
    observer.observe(body);
    return () => { observer.disconnect(); };
  }, []);
  useEffect(() => {
    if (cloudCanvasRef.current) { drawPointCloud(cloudCanvasRef.current, cloudPoints); }
  }, [cloudPoints, cloudSize]);

  useEffect(() => {
    if (!context.advertise) { setPanelError("This connection does not support publishing"); return; }
    try {
      context.advertise(COMMAND_TOPIC, "std_msgs/String", {
        datatypes: new Map([["std_msgs/String", ros2humble["std_msgs/String"]]]),
      });
      context.advertise(GOAL_TOPIC, "geometry_msgs/PoseStamped", {
        datatypes: new Map([
          ["builtin_interfaces/Time", ros2humble["builtin_interfaces/Time"]],
          ["std_msgs/Header", ros2humble["std_msgs/Header"]],
          ["geometry_msgs/Point", ros2humble["geometry_msgs/Point"]],
          ["geometry_msgs/Quaternion", ros2humble["geometry_msgs/Quaternion"]],
          ["geometry_msgs/Pose", ros2humble["geometry_msgs/Pose"]],
          ["geometry_msgs/PoseStamped", ros2humble["geometry_msgs/PoseStamped"]],
        ]),
      });
    } catch (error) { setPanelError(`Publishing unavailable: ${String(error)}`); }
    return () => { context.unadvertise?.(COMMAND_TOPIC); context.unadvertise?.(GOAL_TOPIC); };
  }, [context]);

  const publishCommand = (action: string, mode?: Mode): void => {
    try {
      context.publish?.(COMMAND_TOPIC, { data: JSON.stringify(mode ? { action, mode } : { action }) });
      setPanelError("");
    } catch (error) { setPanelError(`Command failed: ${String(error)}`); }
  };

  const publishGoal = (): void => {
    if (!target || !map || status.active_mode !== "nav" || !status.nav_ready || !status.localized) { return; }
    const value = gridCellAt(map, target);
    if (value == undefined || value < 0 || value >= 65) {
      setPanelError("Selected point is unknown, occupied, or outside the map");
      return;
    }
    try {
      context.publish?.(GOAL_TOPIC, {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "map" },
        pose: {
          position: { x: target.x, y: target.y, z: 0 },
          orientation: { x: 0, y: 0, z: Math.sin(target.yaw / 2), w: Math.cos(target.yaw / 2) },
        },
      });
      setPanelError("");
    } catch (error) { setPanelError(`Goal publish failed: ${String(error)}`); }
  };

  const canvasPoint = (event: React.PointerEvent<HTMLCanvasElement>): Point2D => {
    const bounds = event.currentTarget.getBoundingClientRect();
    return { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
  };
  const onPointerDown = (event: React.PointerEvent<HTMLCanvasElement>): void => {
    if (event.button !== 0 || !viewRef.current || !map) { return; }
    const start = canvasPoint(event);
    const kind = status.active_mode === "nav" && event.shiftKey ? "goal" : "pan";
    mapGesture.current = { pointerId: event.pointerId, kind, start, last: start, moved: false };
    setIsPanning(kind === "pan");
    event.currentTarget.setPointerCapture(event.pointerId);
  };
  const onPointerMove = (event: React.PointerEvent<HTMLCanvasElement>): void => {
    const gesture = mapGesture.current;
    if (gesture?.pointerId !== event.pointerId) { return; }
    const next = canvasPoint(event);
    const wasMoved = gesture.moved;
    if (Math.hypot(next.x - gesture.start.x, next.y - gesture.start.y) > 5) { gesture.moved = true; }
    if (gesture.kind === "pan" && gesture.moved) {
      const previous = wasMoved ? gesture.last : gesture.start;
      const dx = next.x - previous.x;
      const dy = next.y - previous.y;
      if (dx !== 0 || dy !== 0) { setPan((old) => ({ x: old.x + dx, y: old.y + dy })); }
    }
    gesture.last = next;
  };
  const onPointerUp = (event: React.PointerEvent<HTMLCanvasElement>): void => {
    const gesture = mapGesture.current;
    if (gesture?.pointerId !== event.pointerId) { return; }
    mapGesture.current = undefined;
    setIsPanning(false);
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    if (status.active_mode !== "nav" || !viewRef.current || !map ||
        (gesture.kind === "pan" && gesture.moved)) { return; }
    const start = gesture.start;
    const end = canvasPoint(event);
    const point = screenToWorld(start, viewRef.current);
    if (gridCellAt(map, point) == undefined) { return; }
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    setTarget({ ...point, yaw: gesture.kind === "goal" && Math.hypot(dx, dy) > 8 ? Math.atan2(-dy, dx) : 0 });
  };
  const onPointerCancel = (): void => { mapGesture.current = undefined; setIsPanning(false); };

  const zoomAround = (factor: number, anchor: Point2D): void => {
    if (!map || !viewRef.current) { return; }
    const nextZoom = Math.min(32, Math.max(0.25, zoom * factor));
    if (nextZoom === zoom) { return; }
    const world = screenToWorld(anchor, viewRef.current);
    const nextView = mapView(map, size.width, size.height, nextZoom, { x: 0, y: 0 });
    const projected = worldToScreen(world, nextView);
    setPan({ x: anchor.x - projected.x, y: anchor.y - projected.y });
    setZoom(nextZoom);
  };
  const onMapWheel = (event: WheelEvent): void => {
    event.preventDefault();
    event.stopPropagation();
    const bounds = canvasRef.current?.getBoundingClientRect();
    if (!bounds) { return; }
    const anchor = { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
    zoomAround(Math.exp(Math.max(-3, Math.min(3, -event.deltaY * 0.0015))), anchor);
  };
  wheelHandler.current = onMapWheel;
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) { return; }
    const handle = (event: WheelEvent): void => { wheelHandler.current?.(event); };
    canvas.addEventListener("wheel", handle, { passive: false });
    return () => { canvas.removeEventListener("wheel", handle); };
  }, [map]);

  const splitMax = (): number => Math.max(220, (contentRef.current?.clientWidth ?? 1100) - 670);
  const onSplitDown = (event: React.PointerEvent<HTMLDivElement>): void => {
    if (event.button !== 0 || !contentRef.current) { return; }
    const sensors = contentRef.current.querySelector(".odin-sensors");
    if (!(sensors instanceof HTMLElement)) { return; }
    splitGesture.current = { pointerId: event.pointerId, startX: event.clientX,
                             startWidth: sensors.getBoundingClientRect().width };
    event.currentTarget.setPointerCapture(event.pointerId);
  };
  const onSplitMove = (event: React.PointerEvent<HTMLDivElement>): void => {
    const gesture = splitGesture.current;
    if (gesture?.pointerId !== event.pointerId) { return; }
    setLeftWidth(Math.min(splitMax(), Math.max(220, gesture.startWidth + event.clientX - gesture.startX)));
  };
  const onSplitUp = (event: React.PointerEvent<HTMLDivElement>): void => {
    if (splitGesture.current?.pointerId !== event.pointerId) { return; }
    splitGesture.current = undefined;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
  };
  const onSplitKeyDown = (event: React.KeyboardEvent<HTMLDivElement>): void => {
    if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") { return; }
    event.preventDefault();
    const sensors = contentRef.current?.querySelector(".odin-sensors");
    const current = leftWidth ?? (sensors instanceof HTMLElement ? sensors.getBoundingClientRect().width : 500);
    const delta = event.key === "ArrowRight" ? 24 : -24;
    setLeftWidth(Math.min(splitMax(), Math.max(220, current + delta)));
  };

  const toggle = (key: keyof Layers): void => { setLayers((old) => ({ ...old, [key]: !old[key] })); };
  const actionDisabled = !online || !context.publish;
  const canSend = online && status.active_mode === "nav" && status.localized && status.nav_ready && Boolean(target);

  return <div className="odin-root">
    <style>{styles}</style>
    <header className="odin-header">
      <div className="odin-brand"><span className="odin-mark">O</span><div><strong>Odin1 Control</strong><small>Foxglove · {status.world}</small></div></div>
      <div className={online ? "odin-connection online" : "odin-connection"}>
        <span className="odin-dot good" />{online ? "ROBOT CONNECTED" : "WAITING FOR ROBOT"}
      </div>
    </header>
    <div className="odin-content" ref={contentRef}
         style={{ "--odin-sensor-width": leftWidth == undefined ? undefined : `${leftWidth}px` } as CSSProperties}>
      <section className="odin-sensors" aria-label="Odin sensor views">
        <div className="odin-sensor-card">
          <div className="odin-sensor-title"><strong>Odin camera</strong><span>/odin1/image/compressed</span></div>
          <div className="odin-sensor-body odin-camera">
            {cameraUrl && <img src={cameraUrl} alt="Odin1 camera view" onError={() => { setCameraError("Could not decode Odin camera JPEG"); }} />}
            {now - cameraAt >= 3000 && <div className="odin-sensor-empty">{cameraError || "Waiting for camera image"}</div>}
          </div>
        </div>
        <div className="odin-sensor-card">
          <div className="odin-sensor-title"><strong>SLAM point cloud</strong><span>{cloudPoints.length} pts · {cloudFrame || "odom"}</span></div>
          <div className="odin-sensor-body odin-cloud">
            <canvas ref={cloudCanvasRef} width={cloudSize.width} height={cloudSize.height} />
            {now - cloudAt >= 3000 && <div className="odin-sensor-empty">Waiting for /odin1/cloud_slam</div>}
          </div>
        </div>
      </section>
      <div className="odin-splitter" role="separator" aria-label="Resize camera and map panels"
           aria-orientation="vertical" tabIndex={0}
           onPointerDown={onSplitDown} onPointerMove={onSplitMove}
           onPointerUp={onSplitUp} onPointerCancel={onSplitUp}
           onLostPointerCapture={onSplitUp} onKeyDown={onSplitKeyDown} />
      <section className="odin-map-section">
        <div className="odin-map-toolbar">
          <div><div className="odin-eyebrow">MAP VIEW</div><strong>Saved occupancy map</strong></div>
          <div className="odin-toggles">
            <button className={layers.global ? "active" : ""} onClick={() => { toggle("global"); }}>Global costmap</button>
            <button className={layers.local ? "active" : ""} onClick={() => { toggle("local"); }}>Local costmap</button>
            <button className={layers.tf ? "active" : ""} onClick={() => { toggle("tf"); }}>TF / Robot</button>
            <button className={layers.path ? "active" : ""} onClick={() => { toggle("path"); }}>Path</button>
          </div>
        </div>
        <div className="odin-map-box" ref={mapBoxRef}>
          {map ? <canvas ref={canvasRef} className={isPanning ? "panning" : ""}
                         onPointerDown={onPointerDown} onPointerMove={onPointerMove}
                         onPointerUp={onPointerUp} onPointerCancel={onPointerCancel}
                         onLostPointerCapture={onPointerCancel} /> :
            <div className="odin-empty"><strong>No map yet</strong><span>Save the Odin map to generate a Nav2 map.</span></div>}
          <div className="odin-zoom"><button onClick={() => { zoomAround(1 / 1.4, { x: size.width / 2, y: size.height / 2 }); }}>−</button><button onClick={() => { setZoom(1); setPan({ x: 0, y: 0 }); }}>Fit</button><button onClick={() => { zoomAround(1.4, { x: size.width / 2, y: size.height / 2 }); }}>＋</button></div>
          {map && <div className="odin-map-scale">{map.info.resolution.toFixed(3)} m/cell · {map.header.frame_id}</div>}
        </div>
        <div className="odin-goalbar">
          <div><strong>{target ? `Target  x ${target.x.toFixed(2)}  ·  y ${target.y.toFixed(2)}  ·  θ ${(target.yaw * 180 / Math.PI).toFixed(0)}°` : "Click map to choose a goal"}</strong>
            <small>Scroll to zoom, drag to pan. In navigation mode, click a goal; Shift+drag to set its heading.</small></div>
          <button className="odin-primary" disabled={!canSend} onClick={publishGoal}>Send goal ↗</button>
        </div>
      </section>
      <aside className="odin-sidebar">
        <div className="odin-card">
          <div className="odin-eyebrow">OPERATION</div><h2>Mode & controls</h2>
          <div className="odin-modes">
            {(["mapping", "relocalization", "nav"] as const).map((mode) =>
              <button key={mode} className={status.selected_mode === mode ? "chosen" : ""}
                      disabled={actionDisabled} onClick={() => { publishCommand("select", mode); }}>{mode === "mapping" ? "Mapping" : mode === "relocalization" ? "Relocalize" : "Navigate"}</button>)}
          </div>
          <div className="odin-action-row">
            <button className="odin-primary" disabled={actionDisabled || Boolean(status.active_mode)} onClick={() => { publishCommand("start"); }}>▶ Start</button>
            <button className="odin-secondary" disabled={actionDisabled || !status.active_mode || status.saving || status.phase === "stopping"} onClick={() => { publishCommand("stop"); }}>■ Stop</button>
          </div>
          {status.active_mode === "mapping" && <button className="odin-wide" disabled={actionDisabled || status.saving || status.phase !== "mapping" || !status.odin_live} onClick={() => { publishCommand("save_map"); }}>{status.saving ? "Saving map…" : "Save Odin + Nav2 maps"}</button>}
          {status.active_mode === "nav" && <button className="odin-wide" disabled={actionDisabled || status.goal_state !== "active"} onClick={() => { publishCommand("cancel_goal"); }}>Cancel navigation goal</button>}
        </div>
        <div className="odin-card">
          <div className="odin-eyebrow">SYSTEM STATUS</div><h2>{status.phase.replace(/_/g, " ")}</h2>
          <div className="odin-status-grid">
            {badge("Odin stream", status.odin_live ? "ok" : "off")}{badge("Relocalized", status.localized ? "ok" : "off")}
            {badge("Native map", status.odin_map_exists ? "ok" : "off")}{badge("Nav2 grid", status.nav_map_exists ? "ok" : "off")}
            {badge("Nav2 ready", status.nav_ready ? "ok" : "off")}{badge(`Goal: ${status.goal_state}`, status.goal_state === "active" || status.goal_state === "succeeded" ? "ok" : "off")}
          </div>
          <div className="odin-detail">{status.detail}</div>
          {panelError && <div className="odin-error">{panelError}</div>}
        </div>
        <div className="odin-card odin-help"><div className="odin-eyebrow">WORKFLOW</div>
          <p><b>Mapping</b> — drive the whole robot through the scene. Save the map before stopping.</p>
          <p><b>Relocalize</b> — move the whole robot until Odin reports its position.</p>
          <p><b>Navigate</b> — wait for Nav2 ready, click a free map cell, then send the goal.</p>
        </div>
      </aside>
    </div>
  </div>;
}

const styles = `
.odin-root{box-sizing:border-box;width:100%;height:100%;min-height:550px;background:#f5f7fa;color:#142338;font:13px/1.35 Inter,Arial,sans-serif;display:flex;flex-direction:column}
.odin-root *{box-sizing:border-box}.odin-header{height:58px;flex:none;background:#10263e;color:#fff;display:flex;align-items:center;justify-content:space-between;padding:0 18px;border-bottom:3px solid #28a9ab}
.odin-brand{display:flex;align-items:center;gap:10px}.odin-brand strong{font-size:15px;display:block}.odin-brand small{font-size:11px;color:#9db2c7;display:block}.odin-mark{display:grid;place-items:center;width:32px;height:32px;border:2px solid #62d3d1;border-radius:9px;color:#62d3d1;font-weight:800;font-size:19px}
.odin-connection{display:flex;align-items:center;gap:8px;color:#aab9c8;font-size:10px;font-weight:800;letter-spacing:.08em}.odin-connection.online{color:#8ee1c2}.odin-dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#a3afba;flex:none}.odin-dot.good{background:#22c58b;box-shadow:0 0 0 3px #22c58b23}
.odin-content{min-height:0;flex:1;display:grid;grid-template-columns:var(--odin-sensor-width,clamp(220px,calc(100% - 870px),750px)) 12px minmax(0,1fr) 290px;gap:8px;padding:12px}.odin-splitter{min-width:12px;border-radius:6px;cursor:col-resize;touch-action:none;background:linear-gradient(90deg,transparent 4px,#c6d4dd 4px,#c6d4dd 8px,transparent 8px)}.odin-splitter:hover,.odin-splitter:focus-visible{background:linear-gradient(90deg,transparent 4px,#25a8a7 4px,#25a8a7 8px,transparent 8px);outline:none}.odin-map-section{min-width:0;min-height:0;display:flex;flex-direction:column;border:1px solid #dbe3ea;border-radius:10px;overflow:hidden;background:#fff;box-shadow:0 2px 8px #1634540d}
.odin-map-toolbar{min-height:58px;padding:9px 13px;display:flex;align-items:center;justify-content:space-between;gap:10px;border-bottom:1px solid #e5ebf0}.odin-eyebrow{font-size:10px;color:#6e859a;font-weight:800;letter-spacing:.1em;margin-bottom:3px}.odin-map-toolbar strong{font-size:14px}.odin-toggles{display:flex;flex-wrap:wrap;gap:5px;justify-content:flex-end}.odin-toggles button{padding:5px 7px;border:1px solid #cbd8e3;border-radius:6px;background:#fff;color:#596e82;font-size:10px;cursor:pointer}.odin-toggles button.active{background:#e7f4f4;border-color:#66c7c4;color:#166866}
.odin-map-box{position:relative;min-height:160px;flex:1;background-color:#e9edf1;background-image:linear-gradient(#ffffff65 1px,transparent 1px),linear-gradient(90deg,#ffffff65 1px,transparent 1px);background-size:24px 24px;overflow:hidden}.odin-map-box canvas{width:100%;height:100%;display:block;cursor:grab;touch-action:none}.odin-map-box canvas.panning{cursor:grabbing}.odin-empty{position:absolute;inset:0;display:flex;flex-direction:column;align-items:center;justify-content:center;color:#6a7e91;gap:3px}.odin-empty strong{color:#31475a;font-size:16px}.odin-zoom{position:absolute;right:10px;bottom:10px;display:flex;gap:3px}.odin-zoom button{border:1px solid #ccd8e1;border-radius:5px;background:#fff;color:#28465c;padding:5px 9px;cursor:pointer;box-shadow:0 2px 5px #0002}.odin-map-scale{position:absolute;left:10px;bottom:11px;background:#ffffffda;border-radius:4px;padding:4px 7px;color:#526b7d;font-size:10px}
.odin-sensors{min-height:0;display:grid;grid-template-rows:repeat(2,minmax(0,1fr));gap:12px}.odin-sensor-card{min-width:0;min-height:0;display:flex;flex-direction:column;border:1px solid #d6e1e9;border-radius:10px;overflow:hidden;background:white;box-shadow:0 2px 8px #1634540d}.odin-sensor-title{height:36px;flex:none;display:flex;align-items:center;justify-content:space-between;gap:7px;padding:0 10px;color:#23445b}.odin-sensor-title strong{white-space:nowrap;font-size:11px}.odin-sensor-title span{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;color:#778b9e;font-size:9px}.odin-sensor-body{position:relative;min-height:0;flex:1;background:#0c1d2d}.odin-sensor-body img,.odin-sensor-body canvas{display:block;width:100%;height:100%;object-fit:contain}.odin-sensor-empty{position:absolute;inset:0;display:grid;place-items:center;text-align:center;padding:8px;background:#0c1d2dcc;color:#c4d5e1;font-size:11px}
.odin-goalbar{min-height:64px;display:flex;align-items:center;justify-content:space-between;gap:12px;padding:10px 13px;border-top:1px solid #e5ebf0}.odin-goalbar strong{font-size:12px}.odin-goalbar small{display:block;color:#6e8193;font-size:10px;margin-top:3px}.odin-root button{font-family:inherit}.odin-primary{border:0;border-radius:7px;background:#157b81;color:white;font-weight:700;padding:9px 13px;cursor:pointer}.odin-primary:hover:not(:disabled){background:#0b666e}.odin-root button:disabled{opacity:.42;cursor:not-allowed}
.odin-sidebar{min-height:0;overflow:auto;display:flex;flex-direction:column;gap:10px}.odin-card{border:1px solid #dbe3ea;border-radius:10px;padding:14px;background:#fff;box-shadow:0 2px 8px #1634540d}.odin-card h2{font-size:17px;margin:0 0 13px;text-transform:capitalize}.odin-modes{display:grid;grid-template-columns:repeat(3,1fr);gap:5px;margin-bottom:10px}.odin-modes button{border:1px solid #d5dfe7;background:#f8fafb;color:#466073;border-radius:6px;padding:8px 2px;font-size:10px;font-weight:700;cursor:pointer}.odin-modes button.chosen{border-color:#1a9999;background:#e8f7f5;color:#117878}.odin-action-row{display:grid;grid-template-columns:1fr 1fr;gap:7px}.odin-secondary{border:1px solid #d5a7a4;color:#a53c36;background:#fff;border-radius:7px;padding:8px 10px;font-weight:700;cursor:pointer}.odin-wide{width:100%;border:1px solid #a7c7ce;background:#f1f9fa;color:#176d74;border-radius:7px;margin-top:8px;padding:9px;font-weight:700;cursor:pointer}
.odin-status-grid{display:grid;grid-template-columns:1fr 1fr;gap:7px}.odin-badge{display:flex;align-items:center;gap:7px;padding:7px 5px;background:#f7f9fa;border-radius:5px;color:#43596c;font-size:10px}.odin-detail{margin-top:12px;padding:9px;border-left:3px solid #31aca9;background:#f1f8f8;color:#315467;font-size:11px;overflow-wrap:anywhere}.odin-error{margin-top:7px;color:#b33d37;background:#fff1f0;padding:7px;border-radius:5px;font-size:11px;overflow-wrap:anywhere}.odin-help p{font-size:11px;color:#5a7184;margin:8px 0 0}.odin-help b{color:#183b56}
@media(max-width:950px){.odin-content{grid-template-columns:1fr;overflow:auto}.odin-splitter{display:none}.odin-sensors{min-height:220px;grid-template-columns:repeat(2,minmax(0,1fr));grid-template-rows:220px}.odin-map-section{min-height:540px}.odin-sidebar{overflow:visible;display:grid;grid-template-columns:repeat(2,minmax(0,1fr))}.odin-help{grid-column:1/-1}}@media(max-width:570px){.odin-sidebar{grid-template-columns:1fr}.odin-help{grid-column:auto}.odin-map-toolbar{align-items:flex-start;flex-direction:column}.odin-map-section{min-height:680px}.odin-sensors{grid-template-columns:1fr;grid-template-rows:repeat(2,190px)}}
`;

export function initOdinDashboard(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<OdinDashboard context={context} />);
  return () => { root.unmount(); };
}
