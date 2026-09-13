"""ROS-level test for the mission operation sequence."""

import asyncio
from types import SimpleNamespace

from msg_interface.action import TraverseAndPick
import pytest
from r2_mission.mission_server import MissionServer
import rclpy


def test_mission_sequences_navigation_climb_and_gripper(monkeypatch):
    rclpy.init()
    node = MissionServer()
    goal = TraverseAndPick.Goal()
    goal.navigate_to_terrain = True
    goal.terrain_pose.header.frame_id = 'map'
    goal.terrain_pose.pose.orientation.w = 1.0
    goal.terrain_action = 'up'
    goal.navigate_to_pickup = True
    goal.pickup_pose.header.frame_id = 'map'
    goal.pickup_pose.pose.orientation.w = 1.0
    goal.tool = 'gripper'
    settings = node._normalize_goal(goal)
    calls = []

    async def navigate(_handle, _pose, _timeout, phase):
        calls.append(('navigate', phase))

    async def command(_handle, name, values=(), timeout=20.0):
        calls.append((name, tuple(values), timeout))

    async def target(_handle, _settings):
        return (0.50, 0.0, 0.30)

    monkeypatch.setattr(node, '_navigate', navigate)
    monkeypatch.setattr(node, '_robot_command', command)
    monkeypatch.setattr(node, '_wait_target', target)
    handle = SimpleNamespace(
        request=goal,
        publish_feedback=lambda _feedback: None,
    )

    try:
        asyncio.run(node._run_mission(handle, settings))
        assert calls[:5] == [
            ('navigate', 'NAVIGATE_TERRAIN'),
            ('chassis_stop', (), 5.0),
            ('climb_up', (), 180.0),
            ('navigate', 'NAVIGATE_PICKUP'),
            ('chassis_stop', (), 5.0),
        ]
        assert [item[0] for item in calls[5:]] == [
            'select_gripper',
            'gripper_open',
            'arm_move',
            'arm_move',
            'gripper_close',
            'arm_move',
        ]
        assert calls[7][1] == pytest.approx((420.0, 0.0, 350.0))
        assert calls[8][1] == pytest.approx((500.0, 0.0, 300.0))
        assert calls[10][1] == pytest.approx((400.0, 0.0, 380.0))
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
