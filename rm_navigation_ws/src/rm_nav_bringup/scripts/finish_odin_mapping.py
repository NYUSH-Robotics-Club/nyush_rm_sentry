#!/usr/bin/env python3
"""Save the Odin .bin, wait for transfer, then save the companion Nav2 grid."""

import argparse
import sys
import time
from pathlib import Path

import rclpy
from odin_ros_driver.srv import SaveMap
from rclpy.node import Node
from std_srvs.srv import Trigger


def call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f'Service unavailable: {client.srv_name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.exception():
        raise RuntimeError(f'Service failed: {client.srv_name}')
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-dir', required=True)
    parser.add_argument('--world', required=True)
    parser.add_argument('--timeout', type=float, default=300)
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
        grid_client = node.create_client(Trigger, '/odin_grid_map/save')
        grid = call(node, grid_client, Trigger.Request(), 30)
        if not grid.success:
            raise RuntimeError(f'Odin .bin saved, but Nav2 grid failed: {grid.message}')
        print(f'Odin map: {target}\nNav2 grid: {grid.message}', flush=True)
        return 0
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
