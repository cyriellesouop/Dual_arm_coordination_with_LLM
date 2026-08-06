#!/usr/bin/env python3
# jetson/rtsp_server.py
# Grant Hillman 4/25/2026
#
# Streams a CSI camera from a Jetson Nano (JetPack 4.x) as an RTSP feed.
# No ROS required — runs as a plain Python process on the Jetson host.
#
# URL served:  rtsp://<jetson_ip>:<port>/stream<sensor_id>
#
# Prerequisites (run once on each Jetson):
#   sudo apt-get install -y python3-gi gir1.2-gst-rtsp-server-1.0
#
# Usage:
#   python3 rtsp_server.py [--sensor-id N] [--port PORT]
#                          [--width W] [--height H] [--fps F] [--bitrate B]
#
# Or via the wrapper:
#   SENSOR_ID=0 ./run_rtsp_server.sh

import argparse
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib


def build_pipeline(sensor_id: int, width: int, height: int, fps: int, bitrate: int) -> str:
    # NV12 stays in NVMM through nvv4l2h264enc — no CPU copy until network.
    # preset-level=1 = UltraFast encoder; iframeinterval=15 = 0.5s GOP.
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, '
        f'format=(string)NV12, framerate=(fraction){fps}/1 ! '
        f'nvv4l2h264enc '
        f'insert-sps-pps=true '
        f'iframeinterval=15 '
        f'bitrate={bitrate} '
        f'preset-level=1 ! '
        f'video/x-h264, stream-format=byte-stream ! '
        f'h264parse ! '
        f'rtph264pay name=pay0 pt=96 config-interval=1'
    )


def main():
    parser = argparse.ArgumentParser(description='Jetson CSI -> RTSP server (no ROS)')
    parser.add_argument('--sensor-id', type=int, default=0,
                        help='nvarguscamerasrc sensor-id (CSI slot, default 0)')
    parser.add_argument('--width',     type=int, default=1280)
    parser.add_argument('--height',    type=int, default=720)
    parser.add_argument('--fps',       type=int, default=30)
    parser.add_argument('--bitrate',   type=int, default=4_000_000,
                        help='H.264 bitrate in bits/sec (default 4 Mbps)')
    parser.add_argument('--port',      type=str, default='8554')
    args = parser.parse_args()

    mount_path = f'/stream{args.sensor_id}'

    Gst.init(None)
    loop   = GLib.MainLoop()
    server = GstRtspServer.RTSPServer()
    server.set_service(args.port)

    pipeline = build_pipeline(
        args.sensor_id, args.width, args.height, args.fps, args.bitrate
    )

    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(f'( {pipeline} )')
    factory.set_shared(True)  # one pipeline instance shared across all clients

    server.get_mount_points().add_factory(mount_path, factory)
    server.attach(None)

    print(f'RTSP server ready')
    print(f'  URL     : rtsp://0.0.0.0:{args.port}{mount_path}')
    print(f'  Sensor  : {args.sensor_id}')
    print(f'  Res/fps : {args.width}x{args.height} @ {args.fps} fps')
    print(f'  Bitrate : {args.bitrate // 1000} kbps (H.264 UltraFast)')
    print('Press Ctrl-C to stop.')

    try:
        loop.run()
    except KeyboardInterrupt:
        print('\nShutting down.')


if __name__ == '__main__':
    main()
