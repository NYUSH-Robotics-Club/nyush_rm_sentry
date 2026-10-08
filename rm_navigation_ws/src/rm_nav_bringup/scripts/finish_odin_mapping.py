#!/usr/bin/env python3
"""Save Odin's BIN, export PLY/PCD, and create the Nav2 grid from the PCD."""

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from odin_ros_driver.srv import SaveMap
from odin_pcd_to_pgm import save_grid
from rclpy.node import Node


def call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f'Service unavailable: {client.srv_name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception():
        raise RuntimeError(f'Service failed: {client.srv_name}')
    return future.result()


def find_map_to_ply(explicit):
    candidate = explicit or os.environ.get('ODIN_MAP_TO_PLY')
    if candidate:
        path = Path(candidate).expanduser().resolve()
        if not path.is_file() or not os.access(path, os.X_OK):
            raise RuntimeError(f'map_to_ply is missing or not executable: {path}')
        return str(path)
    for name in ('map_to_ply', 'map_to_ply_arm64_v1.2.0', 'map_to_ply_amd64_v1.2.0'):
        path = shutil.which(name)
        if path:
            return path
    raise RuntimeError('map_to_ply is not installed; set ODIN_MAP_TO_PLY to the official executable')


def ply_to_pcd(ply_path, pcd_path):
    """Stream the official XYZ float32 binary PLY payload into a binary PCD."""
    with ply_path.open('rb') as source:
        lines = []
        for _ in range(32):
            line = source.readline(256)
            if not line or len(line) >= 256:
                raise ValueError(f'Invalid PLY header: {ply_path}')
            lines.append(line.decode('ascii').strip())
            if lines[-1] == 'end_header':
                break
        if not lines or lines[0] != 'ply' or lines[-1] != 'end_header':
            raise ValueError(f'Invalid PLY header: {ply_path}')
        if lines[1] != 'format binary_little_endian 1.0':
            raise ValueError('Expected binary little-endian PLY from map_to_ply')
        vertex = [line for line in lines if line.startswith('element vertex ')]
        if len(vertex) != 1:
            raise ValueError('Expected one PLY vertex element')
        count = int(vertex[0].split()[2])
        if count <= 0 or [line for line in lines if line.startswith('property ')] != [
                'property float x', 'property float y', 'property float z']:
            raise ValueError('Expected nonempty XYZ float32 PLY from map_to_ply')
        if any(line.startswith('element ') and line != vertex[0] for line in lines):
            raise ValueError('PLY contains unsupported non-point elements')
        size = ply_path.stat().st_size - source.tell()
        if size != count * 12:
            raise ValueError(f'PLY point data size mismatch: expected {count * 12}, got {size}')
        header = (f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
                  f'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                  f'WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                  f'POINTS {count}\nDATA binary\n')
        with pcd_path.open('wb') as output:
            output.write(header.encode('ascii'))
            shutil.copyfileobj(source, output, length=1024 * 1024)
    return count


def export_point_cloud(native_map, converter):
    ply_path = native_map.with_suffix('.ply')
    pcd_path = native_map.with_suffix('.pcd')
    temporary_ply = ply_path.with_name(ply_path.name + '.tmp')
    temporary_pcd = pcd_path.with_name(pcd_path.name + '.tmp')
    try:
        result = subprocess.run([converter, str(native_map), str(temporary_ply)],
                                capture_output=True, text=True, timeout=300, check=False)
        if result.returncode != 0:
            raise RuntimeError(f'map_to_ply failed ({result.returncode}): '
                               f'{(result.stderr or result.stdout).strip()}')
        if not temporary_ply.is_file():
            raise RuntimeError('map_to_ply succeeded but produced no PLY file')
        count = ply_to_pcd(temporary_ply, temporary_pcd)
        os.replace(temporary_ply, ply_path)
        os.replace(temporary_pcd, pcd_path)
        return ply_path, pcd_path, count
    finally:
        temporary_ply.unlink(missing_ok=True)
        temporary_pcd.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', required=True)
    parser.add_argument('--world', required=True)
    parser.add_argument('--timeout', type=float, default=300)
    parser.add_argument('--grid-resolution', type=float, default=0.05)
    parser.add_argument('--floor-z', type=float, help='Override estimated floor height in map coordinates')
    parser.add_argument('--map-to-ply', help='Path to the official map_to_ply executable; '
                        'defaults to ODIN_MAP_TO_PLY or PATH')
    args = parser.parse_args()
    target = Path(args.map_dir).expanduser() / f'{args.world}.bin'
    status_path = Path(f'{target}.save_status')
    rclpy.init()
    node = Node('finish_odin_mapping')
    try:
        save_client = node.create_client(SaveMap, '/odin1/save_map')
        result = call(node, save_client, SaveMap.Request(value=1), 20)
        if not result.success:
            raise RuntimeError(f'Odin rejected save_map: rc={result.rc}')
        print('Odin accepted save request; waiting for SDK completion...', flush=True)
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            if status_path.exists():
                rc = int(status_path.read_text(encoding='utf-8').strip())
                if rc != 0:
                    raise RuntimeError(f'Odin SDK map save failed: rc={rc}')
                if not target.is_file() or target.stat().st_size == 0:
                    raise RuntimeError(f'Odin reported success but map is missing/empty: {target}')
                break
            time.sleep(1)
        else:
            raise RuntimeError(f'Timed out waiting for Odin SDK completion: {status_path}')
        print(f'Odin map: {target}', flush=True)
        try:
            converter = find_map_to_ply(args.map_to_ply)
            ply_path, pcd_path, count = export_point_cloud(target, converter)
        except Exception as exc:
            raise RuntimeError(f'Odin .bin saved, but PLY/PCD export failed: {exc}') from exc
        print(f'PLY map: {ply_path}\nPCD map: {pcd_path} ({count} points)', flush=True)
        try:
            pgm_path, yaml_path, floor_z = save_grid(
                pcd_path, target.with_suffix(''), args.grid_resolution, args.floor_z)
        except Exception as exc:
            raise RuntimeError(f'Odin .bin, PLY and PCD saved, but Nav2 grid export failed: {exc}') from exc
        print(f'Nav2 grid: {pgm_path}, {yaml_path}; floor z={floor_z:.3f} m', flush=True)
        return 0
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
