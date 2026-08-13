#!/usr/bin/env python3
"""
Ground-truth failure detection for the ICRA dual-arm evaluation harness.

Polls mujoco_ros's /mujoco_server/get_body_state service (confirmed live against
the running dual-arm sim, domain 4) to watch tracked objects/end-effectors and
emit failure events. Runs against MuJoCo ground truth, not vision -- the point is
noise-free failure detection so the 3-baseline comparison isolates the recovery
strategy, not perception quality. This is the ONE piece of detection machinery
all three baselines (A/B/C) share identically.

Interface is topic-based (String+JSON, matching this project's existing
convention for /detected_objects, /selected_object -- see
auro_controller/README.md) rather than a direct importable Python API, so
dual_arm_coordinator.py doesn't need to embed rclpy reentrancy concerns from
calling into another node's service client. See _monitor_tick below for why
that matters here specifically.

Failure modes implemented:
  - drop / failed_grasp: object stops rigidly tracking the carrying arm's
    end-effector link while it should be attached.
  - misplacement: final object pose vs. intended target pose, one-shot check.
  - collision (DETECTION-ONLY, proxy): mujoco_ros_msgs exposes no contact-array
    service (checked: GetBodyState/SetBodyState/GetSimInfo -- no GetContacts
    equivalent), so this is approximated as inter-object proximity below a
    threshold while one object is being carried. A real limitation, not true
    contact detection -- documented rather than silently assumed exact (see the
    evaluation-harness plan: collision injection is explicitly out of scope for
    v1, detection is a best-effort proxy).

Commands in (topic: /failure_monitor/cmd, String JSON):
  {"cmd": "track_grasp", "object": "cup", "ee_link": "armA_end_effector_link"}
  {"cmd": "release", "object": "cup"}
  {"cmd": "check_placement", "object": "cup", "target": [x, y, z],
   "tolerance": 0.05, "request_id": "..."}

Events out (topic: /failure_events, String JSON):
  {"type": "drop"|"failed_grasp"|"misplacement"|"collision", "timestamp": ..., ...}

Placement-check results out (topic: /failure_monitor/result, String JSON):
  {"request_id": "...", "success": bool, "distance": float}
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from mujoco_ros_msgs.srv import GetBodyState
from std_msgs.msg import String


RIGID_TRACKING_TOLERANCE_M = 0.03   # object-to-gripper offset drift beyond this => drop
FAILED_GRASP_DISTANCE_M = 0.09      # object-to-gripper distance at grasp time beyond this => never actually grasped
COLLISION_PROXIMITY_M = 0.05        # proxy threshold, NOT a true contact check (see module docstring)
GET_BODY_STATE_SERVICE = '/mujoco_server/get_body_state'
MONITOR_PERIOD_SEC = 0.05           # 20 Hz


def _pos(pose_stamped):
    p = pose_stamped.pose.position
    return (p.x, p.y, p.z)


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


class FailureMonitor(Node):

    def __init__(self):
        super().__init__('failure_monitor')
        self._get_body_state = self.create_client(GetBodyState, GET_BODY_STATE_SERVICE)
        self._events_pub = self.create_publisher(String, '/failure_events', 10)
        self._result_pub = self.create_publisher(String, '/failure_monitor/result', 10)
        self.create_subscription(String, '/failure_monitor/cmd', self._on_cmd, 10)

        # object_name -> {'ee_link': str, 'grasp_offset': (dx,dy,dz)}
        self._held = {}
        self._tracked_objects = ['cup', 'straw']

        # Timer callbacks in rclpy may be coroutines ("async def") -- the executor
        # (including the default SingleThreadedExecutor) steps them as tasks and
        # processes awaited futures correctly. A plain "def" callback calling
        # spin_until_future_complete(self, ...) on THIS node's own client would
        # deadlock: the executor is already inside this callback, so it can never
        # reach the step that would resolve that future. async/await sidesteps
        # that entirely -- this is the standard pattern for a node that must call
        # a service from its own timer.
        self.create_timer(MONITOR_PERIOD_SEC, self._monitor_tick)

    async def _get_pose(self, name, timeout_sec=0.5):
        if not self._get_body_state.service_is_ready():
            return None
        req = GetBodyState.Request()
        req.name = name
        req.admin_hash = ''
        future = self._get_body_state.call_async(req)
        try:
            result = await future
        except Exception:
            return None
        if result is None or not result.success:
            return None
        return _pos(result.state.pose)

    def _emit(self, failure_type, **context):
        event = {'type': failure_type, 'timestamp': time.time(), **context}
        self._events_pub.publish(String(data=json.dumps(event)))
        self.get_logger().warn(f'[failure_monitor] {failure_type}: {context}')
        return event

    async def _on_cmd(self, msg):
        # A coroutine callback, same reasoning as _monitor_tick above: this needs
        # to await GetBodyState calls, and subscription callbacks support being
        # coroutines exactly like timer callbacks do -- no separate task-scheduling
        # API needed (Node has no reliably-portable public create_task in Humble;
        # awaiting directly here is simpler and avoids relying on it).
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'[failure_monitor] bad /failure_monitor/cmd payload: {msg.data!r}')
            return

        if cmd['cmd'] == 'track_grasp':
            await self._track_grasp(cmd['object'], cmd['ee_link'])
        elif cmd['cmd'] == 'release':
            self._held.pop(cmd['object'], None)
        elif cmd['cmd'] == 'check_placement':
            await self._check_placement(
                cmd['object'], tuple(cmd['target']), cmd['tolerance'], cmd['request_id'])
        else:
            self.get_logger().error(f'[failure_monitor] unknown cmd: {cmd["cmd"]!r}')

    async def _track_grasp(self, object_name, ee_link_name):
        obj_pos = await self._get_pose(object_name)
        ee_pos = await self._get_pose(ee_link_name)
        if obj_pos is None or ee_pos is None:
            self._emit('failed_grasp', object=object_name, ee_link=ee_link_name,
                       reason='pose unreadable at grasp confirmation time')
            return
        d = _dist(obj_pos, ee_pos)
        if d > FAILED_GRASP_DISTANCE_M:
            self._emit('failed_grasp', object=object_name, ee_link=ee_link_name, distance=d)
            return
        offset = tuple(obj_pos[i] - ee_pos[i] for i in range(3))
        self._held[object_name] = {'ee_link': ee_link_name, 'grasp_offset': offset}

    async def _check_placement(self, object_name, target_xyz, tolerance_m, request_id):
        obj_pos = await self._get_pose(object_name)
        if obj_pos is None:
            self._emit('misplacement', object=object_name, reason='pose unreadable')
            self._result_pub.publish(String(data=json.dumps(
                {'request_id': request_id, 'success': False, 'distance': None})))
            return
        d = _dist(obj_pos, target_xyz)
        success = d <= tolerance_m
        if not success:
            self._emit('misplacement', object=object_name, distance=d,
                       target=list(target_xyz), actual=list(obj_pos))
        self._result_pub.publish(String(data=json.dumps(
            {'request_id': request_id, 'success': success, 'distance': d})))

    async def _monitor_tick(self):
        for object_name, info in list(self._held.items()):
            obj_pos = await self._get_pose(object_name)
            ee_pos = await self._get_pose(info['ee_link'])
            if obj_pos is None or ee_pos is None:
                continue
            expected = tuple(ee_pos[i] + info['grasp_offset'][i] for i in range(3))
            drift = _dist(obj_pos, expected)
            if drift > RIGID_TRACKING_TOLERANCE_M:
                self._emit('drop', object=object_name, ee_link=info['ee_link'], drift=drift)
                self._held.pop(object_name, None)  # one event per drop, not one per tick

        if not self._held:
            return

        positions = {}
        for name in self._tracked_objects:
            pos = await self._get_pose(name)
            if pos is not None:
                positions[name] = pos
        names = list(positions.keys())
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                if a in self._held or b in self._held:
                    d = _dist(positions[a], positions[b])
                    if d < COLLISION_PROXIMITY_M:
                        self._emit('collision', objects=[a, b], distance=d)


def main():
    rclpy.init()
    node = FailureMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
