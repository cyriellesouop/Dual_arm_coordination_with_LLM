#!/usr/bin/env python3
"""
calibration/convert.py — Build camera_calibration_nodes.yaml for all 4 streams.

Stream 1 is the world origin.  Streams 0, 2, 3 are registered to stream 1
via their respective stereo calibration files.  Intrinsics are taken from the
jointly-calibrated values inside each stereo file, which is more consistent
with the R/T from the same optimization than using standalone per-camera files.

Usage (from repo root):
    python3 calibration/convert.py
    python3 calibration/convert.py --out calibration/camera_calibration_nodes.yaml
"""

import argparse
import pathlib

import numpy as np
import yaml

HERE = pathlib.Path(__file__).parent

# Each stereo file has stream 1 as left (world origin) and the keyed stream as right.
STEREO_FILES = {
    0: HERE / 'stereo_aruco_stream1_stream0_720p.yaml',
    2: HERE / 'stereo_aruco_stream1_stream2_720p.yaml',
    3: HERE / 'stereo_aruco_stream1_stream3_720p.yaml',
}
ORIGIN_ID = 1


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def build(out_path):
    # Stream 1 intrinsics come from the left side of any stereo file — all identical.
    origin_stereo = load_yaml(STEREO_FILES[0])
    K1 = np.array(origin_stereo['k_matrix_left'],  dtype=float)
    D1 = np.array(origin_stereo['distortion_left'], dtype=float).flatten()

    cameras_out = {}

    # Stream 1 — world origin: identity rotation, zero translation.
    cameras_out['stream_1'] = {
        'intrinsics': {
            'k_matrix':   K1.tolist(),
            'distortion': [D1.tolist()],
        },
        'extrinsics': {
            'R_c2w': np.eye(3).tolist(),
            't_c2w': [0.0, 0.0, 0.0],
        },
    }

    # Streams 0, 2, 3 — pose derived from stereo extrinsics relative to stream 1.
    for sid, stereo_path in STEREO_FILES.items():
        stereo = load_yaml(stereo_path)
        K = np.array(stereo['k_matrix_right'],   dtype=float)
        D = np.array(stereo['distortion_right'],  dtype=float).flatten()
        R = np.array(stereo['R'],                 dtype=float)
        T = np.array(stereo['T'],                 dtype=float).flatten()

        # Stereo convention: X_right = R @ X_left + T
        #   → camera-to-world:  R_c2w = R.T,  t_c2w = -(R.T @ T)
        R_c2w = R.T
        t_c2w = -(R.T @ T)

        cameras_out[f'stream_{sid}'] = {
            'intrinsics': {
                'k_matrix':   K.tolist(),
                'distortion': [D.tolist()],
            },
            'extrinsics': {
                'R_c2w': R_c2w.tolist(),
                't_c2w': t_c2w.tolist(),
            },
        }

    doc = {
        'unit': 'meters',
        'world_origin_stream': ORIGIN_ID,
        'cameras': cameras_out,
    }

    with open(out_path, 'w') as f:
        yaml.dump(doc, f, default_flow_style=False, sort_keys=False)

    print(f'Wrote {out_path}')
    print(f'  stream_1  t_c2w = [0.0, 0.0, 0.0]  (world origin)')
    for sid in sorted(STEREO_FILES):
        t        = cameras_out[f'stream_{sid}']['extrinsics']['t_c2w']
        baseline = np.linalg.norm(t)
        print(f'  stream_{sid}  t_c2w = {[round(v, 4) for v in t]}  '
              f'baseline = {baseline:.4f} m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(HERE / 'camera_calibration_nodes.yaml'))
    args = ap.parse_args()
    build(args.out)


if __name__ == '__main__':
    main()
