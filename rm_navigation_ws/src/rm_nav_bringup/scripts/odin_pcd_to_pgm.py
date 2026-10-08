#!/usr/bin/env python3
"""Create an Odin/Nav2 occupancy map from an exported XYZ binary PCD."""

import argparse
import os
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt


def read_xyz_pcd(path):
    with path.open('rb') as source:
        header = {}
        for _ in range(32):
            line = source.readline(256)
            if not line or len(line) >= 256:
                raise ValueError(f'Invalid PCD header: {path}')
            parts = line.decode('ascii').strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            header[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        if (header.get('FIELDS') != ['x', 'y', 'z'] or
                header.get('SIZE') != ['4', '4', '4'] or
                header.get('TYPE') != ['F', 'F', 'F'] or
                header.get('COUNT') != ['1', '1', '1'] or
                header.get('DATA') != ['binary']):
            raise ValueError('Expected XYZ float32 binary PCD from Odin map_to_ply')
        count = int(header['POINTS'][0])
        if count <= 0 or path.stat().st_size - source.tell() != count * 12:
            raise ValueError('PCD point count or file size is invalid')
        points = np.fromfile(source, dtype='<f4', count=count * 3).reshape((-1, 3))
    return points[np.isfinite(points).all(axis=1)]


def estimate_floor_z(z):
    low = z[z <= np.quantile(z, 0.4)]
    if len(low) < 100:
        raise ValueError('Too few map points to estimate the floor height')
    edges = np.arange(low.min(), low.max() + 0.10, 0.05)
    counts, edges = np.histogram(low, bins=edges)
    peak = int(np.argmax(counts))
    return float(np.median(low[(low >= edges[peak]) & (low < edges[peak + 1])]))


def create_grid(points, resolution=0.05, floor_z=None):
    if resolution <= 0:
        raise ValueError('resolution must be positive')
    if len(points) < 100:
        raise ValueError('Too few PCD points for an occupancy map')
    floor_z = estimate_floor_z(points[:, 2]) if floor_z is None else floor_z
    z = points[:, 2]
    ground = points[(z >= floor_z - 0.12) & (z <= floor_z + 0.15)]
    obstacles = points[(z >= floor_z + 0.25) & (z <= floor_z + 1.80)]
    if len(ground) < 100:
        raise ValueError(f'No usable ground points near z={floor_z:.3f} m')

    margin = 1.0
    lower = np.floor((ground[:, :2].min(axis=0) - margin) / resolution).astype(np.int64)
    upper = np.ceil((ground[:, :2].max(axis=0) + margin) / resolution).astype(np.int64)
    width, height = (upper - lower + 1).tolist()
    if width * height > 30_000_000:
        raise ValueError('Map exceeds 30 million cells; inspect PCD outliers or increase resolution')

    floor_hits = np.zeros((height, width), dtype=bool)
    ground_xy = np.floor(ground[:, :2] / resolution).astype(np.int64) - lower
    floor_hits[ground_xy[:, 1], ground_xy[:, 0]] = True
    distance_to_floor = distance_transform_edt(~floor_hits, sampling=resolution)
    free = distance_to_floor <= 0.20

    obstacle_hits = np.zeros((height, width), dtype=np.uint16)
    obstacle_xy = np.floor(obstacles[:, :2] / resolution).astype(np.int64) - lower
    inside = ((obstacle_xy[:, 0] >= 0) & (obstacle_xy[:, 0] < width) &
              (obstacle_xy[:, 1] >= 0) & (obstacle_xy[:, 1] < height))
    obstacle_xy = obstacle_xy[inside]
    np.add.at(obstacle_hits, (obstacle_xy[:, 1], obstacle_xy[:, 0]), 1)
    occupied = (obstacle_hits >= 2) & (distance_to_floor <= 0.75)

    # ROS map_server uses 0=occupied, 254=free, and 205=unknown by default.
    image = np.full((height, width), 205, dtype=np.uint8)
    image[free] = 254
    image[occupied] = 0
    if not np.any(occupied) or not np.any(free):
        raise ValueError('PCD projection has no free cells or no obstacles')
    return image[::-1], lower.astype(float) * resolution, floor_z


def save_grid(pcd_path, prefix, resolution=0.05, floor_z=None):
    points = read_xyz_pcd(pcd_path)
    image, origin, floor_z = create_grid(points, resolution, floor_z)
    height, width = image.shape
    pgm_path = prefix.with_suffix('.pgm')
    yaml_path = prefix.with_suffix('.yaml')
    temp_pgm = pgm_path.with_name(pgm_path.name + '.tmp')
    temp_yaml = yaml_path.with_name(yaml_path.name + '.tmp')
    try:
        temp_pgm.write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + image.tobytes())
        temp_yaml.write_text(
            f'image: {pgm_path.name}\nresolution: {resolution}\n'
            f'origin: [{origin[0]}, {origin[1]}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
            encoding='utf-8')
        os.replace(temp_pgm, pgm_path)
        os.replace(temp_yaml, yaml_path)
    finally:
        temp_pgm.unlink(missing_ok=True)
        temp_yaml.unlink(missing_ok=True)
    return pgm_path, yaml_path, floor_z


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=Path)
    parser.add_argument('--output-prefix', required=True, type=Path)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--floor-z', type=float, help='Override the estimated floor height in map coordinates')
    args = parser.parse_args()
    pgm, yaml_path, floor_z = save_grid(args.pcd, args.output_prefix,
                                         args.resolution, args.floor_z)
    print(f'Nav2 grid: {pgm}, {yaml_path}; estimated floor z={floor_z:.3f} m')


if __name__ == '__main__':
    main()
