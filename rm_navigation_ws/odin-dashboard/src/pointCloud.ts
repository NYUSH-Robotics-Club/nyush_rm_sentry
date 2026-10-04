export type BinaryData = Uint8Array | readonly number[] | ArrayBuffer;
export type PointField = { name: string; offset: number; datatype: number };
export type PointCloud2 = {
  header: { frame_id: string };
  height: number;
  width: number;
  fields: PointField[];
  is_bigendian: boolean;
  point_step: number;
  row_step: number;
  data: BinaryData;
};
export type CloudPoint = { x: number; y: number; z: number; r: number; g: number; b: number };

const MAX_PREVIEW_POINTS = 12_000;
const FLOAT32 = 7;
const UINT32 = 6;

export function asBytes(data: BinaryData): Uint8Array {
  if (data instanceof Uint8Array) { return data; }
  if (data instanceof ArrayBuffer) { return new Uint8Array(data); }
  return Uint8Array.from(data);
}

export function samplePointCloud(cloud: PointCloud2): CloudPoint[] {
  const bytes = asBytes(cloud.data);
  const xField = cloud.fields.find((field) => field.name === "x" && field.datatype === FLOAT32);
  const yField = cloud.fields.find((field) => field.name === "y" && field.datatype === FLOAT32);
  const zField = cloud.fields.find((field) => field.name === "z" && field.datatype === FLOAT32);
  const colorField = cloud.fields.find((field) =>
    (field.name === "rgb" || field.name === "rgba") &&
    (field.datatype === FLOAT32 || field.datatype === UINT32));
  if (!xField || !yField || !zField || cloud.width <= 0 || cloud.height <= 0 ||
      cloud.point_step < 12 || cloud.row_step < cloud.width * cloud.point_step ||
      [xField, yField, zField].some((field) => field.offset < 0 || field.offset + 4 > cloud.point_step)) {
    return [];
  }
  const count = cloud.width * cloud.height;
  const step = Math.max(1, Math.ceil(count / MAX_PREVIEW_POINTS));
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !cloud.is_bigendian;
  const result: CloudPoint[] = [];
  for (let i = 0; i < count; i += step) {
    const offset = Math.floor(i / cloud.width) * cloud.row_step +
                   (i % cloud.width) * cloud.point_step;
    if (offset < 0 || offset + cloud.point_step > bytes.byteLength) { continue; }
    const x = view.getFloat32(offset + xField.offset, littleEndian);
    const y = view.getFloat32(offset + yField.offset, littleEndian);
    const z = view.getFloat32(offset + zField.offset, littleEndian);
    if (![x, y, z].every(Number.isFinite)) { continue; }
    const rgb = colorField && colorField.offset + 4 <= cloud.point_step ?
      view.getUint32(offset + colorField.offset, littleEndian) : 0x77d1d4;
    result.push({ x, y, z, r: (rgb >> 16) & 255, g: (rgb >> 8) & 255, b: rgb & 255 });
  }
  return result;
}

export function drawPointCloud(canvas: HTMLCanvasElement, points: readonly CloudPoint[]): void {
  const width = canvas.width;
  const height = canvas.height;
  const ctx = canvas.getContext("2d");
  if (!ctx) { return; }
  const image = ctx.createImageData(width, height);
  for (let index = 0; index < image.data.length; index += 4) {
    image.data[index] = 12;
    image.data[index + 1] = 29;
    image.data[index + 2] = 45;
    image.data[index + 3] = 255;
  }
  if (points.length === 0) { ctx.putImageData(image, 0, 0); return; }
  const yaw = -Math.PI / 4;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const pitch = Math.PI / 5;
  const projected = points.map((point) => {
    const side = cy * point.x - sy * point.y;
    const depth = sy * point.x + cy * point.y;
    return { x: side, y: -(Math.cos(pitch) * point.z + Math.sin(pitch) * depth), point };
  });
  let minX = Infinity; let maxX = -Infinity; let minY = Infinity; let maxY = -Infinity;
  for (const item of projected) {
    minX = Math.min(minX, item.x); maxX = Math.max(maxX, item.x);
    minY = Math.min(minY, item.y); maxY = Math.max(maxY, item.y);
  }
  const scale = Math.min((width - 28) / Math.max(maxX - minX, 0.5),
                         (height - 28) / Math.max(maxY - minY, 0.5));
  for (const item of projected) {
    const x = Math.round((item.x - (minX + maxX) / 2) * scale + width / 2);
    const y = Math.round((item.y - (minY + maxY) / 2) * scale + height / 2);
    for (let dy = 0; dy < 2; dy++) {
      for (let dx = 0; dx < 2; dx++) {
        if (x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) { continue; }
        const index = ((y + dy) * width + x + dx) * 4;
        image.data[index] = item.point.r;
        image.data[index + 1] = item.point.g;
        image.data[index + 2] = item.point.b;
      }
    }
  }
  ctx.putImageData(image, 0, 0);
}
