export type Vector3 = { x: number; y: number; z: number };
export type Quaternion = { x: number; y: number; z: number; w: number };
export type Pose = { position: Vector3; orientation: Quaternion };
export type Grid = {
  header: { frame_id: string };
  info: { width: number; height: number; resolution: number; origin: Pose };
  data: readonly number[] | Int8Array;
};
export type Transform2D = { x: number; y: number; yaw: number };
export type Point2D = { x: number; y: number };

export function yawOf(q: Quaternion): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

export function applyTransform(p: Point2D, tf: Transform2D): Point2D {
  return {
    x: tf.x + Math.cos(tf.yaw) * p.x - Math.sin(tf.yaw) * p.y,
    y: tf.y + Math.sin(tf.yaw) * p.x + Math.cos(tf.yaw) * p.y,
  };
}

export function invertTransform(tf: Transform2D): Transform2D {
  const yaw = -tf.yaw;
  return {
    x: -Math.cos(yaw) * tf.x + Math.sin(yaw) * tf.y,
    y: -Math.sin(yaw) * tf.x - Math.cos(yaw) * tf.y,
    yaw,
  };
}

export function gridOriginInFrame(grid: Grid, frameTransform?: Transform2D): Transform2D {
  const origin = grid.info.origin;
  const local = { x: origin.position.x, y: origin.position.y, yaw: yawOf(origin.orientation) };
  if (!frameTransform) {
    return local;
  }
  const position = applyTransform(local, frameTransform);
  return { ...position, yaw: frameTransform.yaw + local.yaw };
}

export function gridCorners(grid: Grid, frameTransform?: Transform2D): Point2D[] {
  const tf = gridOriginInFrame(grid, frameTransform);
  const width = grid.info.width * grid.info.resolution;
  const height = grid.info.height * grid.info.resolution;
  return [
    applyTransform({ x: 0, y: 0 }, tf),
    applyTransform({ x: width, y: 0 }, tf),
    applyTransform({ x: 0, y: height }, tf),
    applyTransform({ x: width, y: height }, tf),
  ];
}

export function gridCellAt(grid: Grid, point: Point2D): number | undefined {
  const origin = gridOriginInFrame(grid);
  const dx = point.x - origin.x;
  const dy = point.y - origin.y;
  const x = Math.floor((Math.cos(origin.yaw) * dx + Math.sin(origin.yaw) * dy) / grid.info.resolution);
  const y = Math.floor((-Math.sin(origin.yaw) * dx + Math.cos(origin.yaw) * dy) / grid.info.resolution);
  if (x < 0 || y < 0 || x >= grid.info.width || y >= grid.info.height) {
    return undefined;
  }
  return grid.data[y * grid.info.width + x];
}
