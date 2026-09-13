"""Action server coordinating Nav2, terrain traversal and object picking."""

import asyncio
from math import isfinite
from threading import Lock
from time import monotonic

from action_msgs.msg import GoalStatus
from msg_interface.action import TraverseAndPick
from msg_interface.msg import PnpResult, RobotCommandResult
from msg_interface.srv import RobotCommand
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from .mission_core import arm_waypoints, TargetStabilizer


class MissionError(RuntimeError):
    """Expected mission failure with a user-facing explanation."""


class MissionCanceled(MissionError):
    """Mission cancellation requested by the action client."""


class MissionServer(Node):
    """Execute one navigation, terrain and picking mission at a time."""

    def __init__(self, **node_kwargs):
        super().__init__('r2_mission_server', **node_kwargs)
        self.declare_parameter('mission_action', 'traverse_and_pick')
        self.declare_parameter('nav_action', 'navigate_to_pose')
        self.declare_parameter('robot_service', 'robot/command')
        self.declare_parameter('robot_result_topic', 'robot/command_result')
        self.declare_parameter('target_topic', 'pnp_result')
        self.declare_parameter('target_frame_id', 'arm_base_link')

        group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter('nav_action').value),
            callback_group=group,
        )
        self.robot_client = self.create_client(
            RobotCommand,
            str(self.get_parameter('robot_service').value),
            callback_group=group,
        )
        self.create_subscription(
            RobotCommandResult,
            str(self.get_parameter('robot_result_topic').value),
            self._command_result_callback,
            50,
            callback_group=group,
        )
        self.create_subscription(
            PnpResult,
            str(self.get_parameter('target_topic').value),
            self._target_callback,
            20,
            callback_group=group,
        )
        self.action_server = ActionServer(
            self,
            TraverseAndPick,
            str(self.get_parameter('mission_action').value),
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=group,
        )

        self.target_frame = str(
            self.get_parameter('target_frame_id').value
        )
        self.reservation_lock = Lock()
        self.goal_reserved = False
        self.command_results = {}
        self.collect_target = False
        self.stabilizer = None
        self.stable_target = None
        self.current_nav_goal = None
        self.active_goal_handle = None
        self.get_logger().info('TraverseAndPick action server is ready')

    @staticmethod
    def _finite_pose(pose):
        values = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        return all(isfinite(value) for value in values)

    def _normalize_goal(self, goal):
        terrain = goal.terrain_action.strip().lower() or 'none'
        tool = goal.tool.strip().lower() or 'gripper'
        if terrain not in ('none', 'up', 'down'):
            raise ValueError('terrain_action must be none, up or down')
        if tool not in ('gripper', 'suction'):
            raise ValueError('tool must be gripper or suction')
        for enabled, pose, name in (
            (goal.navigate_to_terrain, goal.terrain_pose, 'terrain_pose'),
            (goal.navigate_to_pickup, goal.pickup_pose, 'pickup_pose'),
        ):
            if enabled and (
                not pose.header.frame_id or not self._finite_pose(pose)
            ):
                raise ValueError(name + ' needs a frame and finite pose')
            if enabled:
                orientation = pose.pose.orientation
                norm_squared = (
                    orientation.x * orientation.x
                    + orientation.y * orientation.y
                    + orientation.z * orientation.z
                    + orientation.w * orientation.w
                )
                if norm_squared < 0.25:
                    raise ValueError(name + ' needs a valid orientation')

        settings = {
            'terrain': terrain,
            'tool': tool,
            'navigation_timeout': goal.navigation_timeout_s or 180.0,
            'target_timeout': goal.target_timeout_s or 10.0,
            'stable_samples': goal.stable_samples or 5,
            'stable_tolerance': goal.stable_tolerance_m or 0.02,
            'pregrasp_backoff': goal.pregrasp_backoff_m or 0.08,
            'pregrasp_lift': goal.pregrasp_lift_m or 0.05,
            'grasp_forward': goal.grasp_forward_offset_m,
            'retreat_backoff': goal.retreat_backoff_m or 0.10,
            'retreat_lift': goal.retreat_lift_m or 0.08,
        }
        if (
            not isfinite(settings['navigation_timeout'])
            or not 1 <= settings['navigation_timeout'] <= 1800
        ):
            raise ValueError('navigation_timeout_s must be in [1, 1800]')
        if (
            not isfinite(settings['target_timeout'])
            or not 1 <= settings['target_timeout'] <= 120
        ):
            raise ValueError('target_timeout_s must be in [1, 120]')
        TargetStabilizer(
            int(settings['stable_samples']), settings['stable_tolerance']
        )
        arm_waypoints(
            (0.0, 0.0, 0.0),
            settings['pregrasp_backoff'],
            settings['pregrasp_lift'],
            settings['grasp_forward'],
            settings['retreat_backoff'],
            settings['retreat_lift'],
        )
        return settings

    def _goal_callback(self, goal):
        try:
            self._normalize_goal(goal)
        except ValueError as error:
            self.get_logger().warning(f'rejected mission goal: {error}')
            return GoalResponse.REJECT
        with self.reservation_lock:
            if self.goal_reserved:
                self.get_logger().warning('rejected mission: another is active')
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _command_result_callback(self, message):
        if message.state != 'running':
            self.command_results[message.request_id] = (
                message.state, message.detail
            )
            if len(self.command_results) > 1000:
                oldest = next(iter(self.command_results))
                del self.command_results[oldest]

    def _target_callback(self, message):
        if (
            not self.collect_target
            or not message.valid
            or message.predicted
            or message.header.frame_id != self.target_frame
        ):
            return
        stable = self.stabilizer.add((message.x, message.y, message.z))
        if stable is not None:
            self.stable_target = stable

    def _feedback(self, goal_handle, phase, detail):
        feedback = TraverseAndPick.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _check_cancel(goal_handle):
        if goal_handle.is_cancel_requested:
            raise MissionCanceled('mission canceled by client')

    async def _wait_ready(self, ready, goal_handle, timeout, description):
        deadline = monotonic() + timeout
        while not ready():
            self._check_cancel(goal_handle)
            if monotonic() >= deadline:
                raise MissionError(description + ' is unavailable')
            await asyncio.sleep(0.05)

    async def _wait_future(self, future, goal_handle, timeout, description):
        deadline = monotonic() + timeout
        while not future.done():
            self._check_cancel(goal_handle)
            if monotonic() >= deadline:
                raise MissionError(description + ' timed out')
            await asyncio.sleep(0.05)
        try:
            return future.result()
        except Exception as error:
            raise MissionError(f'{description} failed: {error}') from error

    async def _navigate(self, goal_handle, pose, timeout, phase):
        self._feedback(goal_handle, phase, 'waiting for Nav2')
        await self._wait_ready(
            self.nav_client.server_is_ready,
            goal_handle,
            10.0,
            'Nav2 NavigateToPose server',
        )
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        sent = await self._wait_future(
            self.nav_client.send_goal_async(nav_goal),
            goal_handle,
            10.0,
            'Nav2 goal submission',
        )
        if not sent.accepted:
            raise MissionError('Nav2 rejected the goal')
        self.current_nav_goal = sent
        self._feedback(goal_handle, phase, 'Nav2 goal accepted')
        result = await self._wait_future(
            sent.get_result_async(), goal_handle, timeout, 'Nav2 navigation'
        )
        self.current_nav_goal = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise MissionError(f'Nav2 ended with status {result.status}')

    async def _robot_command(self, goal_handle, command, values=(),
                             timeout=20.0):
        self._check_cancel(goal_handle)
        await self._wait_ready(
            self.robot_client.service_is_ready,
            goal_handle,
            10.0,
            'robot command service',
        )
        request = RobotCommand.Request()
        request.command = command
        request.values = [float(value) for value in values]
        request.timeout_s = float(timeout)
        response = await self._wait_future(
            self.robot_client.call_async(request),
            goal_handle,
            10.0,
            command + ' service call',
        )
        if not response.accepted:
            raise MissionError(f'{command} rejected: {response.message}')

        deadline = monotonic() + timeout + 2.0
        while response.request_id not in self.command_results:
            self._check_cancel(goal_handle)
            if monotonic() >= deadline:
                raise MissionError(command + ' result timed out')
            await asyncio.sleep(0.05)
        state, detail = self.command_results.pop(response.request_id)
        if state != 'succeeded':
            raise MissionError(f'{command} {state}: {detail}')
        return detail

    async def _wait_target(self, goal_handle, settings):
        self.stabilizer = TargetStabilizer(
            int(settings['stable_samples']), settings['stable_tolerance']
        )
        self.stable_target = None
        self.collect_target = True
        deadline = monotonic() + settings['target_timeout']
        try:
            while self.stable_target is None:
                self._check_cancel(goal_handle)
                if monotonic() >= deadline:
                    raise MissionError('stable measured target timed out')
                await asyncio.sleep(0.05)
            return self.stable_target
        finally:
            self.collect_target = False

    async def _stop_outputs(self):
        if not self.robot_client.service_is_ready():
            return
        request = RobotCommand.Request()
        request.command = 'stop'
        try:
            self.robot_client.call_async(request)
        except Exception:
            pass

    async def _cancel_navigation(self):
        if self.current_nav_goal is None:
            return
        try:
            self.current_nav_goal.cancel_goal_async()
        except Exception:
            pass
        self.current_nav_goal = None

    async def _park_after_navigation(self, goal_handle, phase):
        self._feedback(
            goal_handle, phase, 'waiting for cmd_vel lease and stopping chassis'
        )
        await asyncio.sleep(0.25)
        await self._robot_command(
            goal_handle, 'chassis_stop', timeout=5.0
        )

    async def _run_mission(self, goal_handle, settings):
        goal = goal_handle.request
        if goal.navigate_to_terrain:
            await self._navigate(
                goal_handle,
                goal.terrain_pose,
                settings['navigation_timeout'],
                'NAVIGATE_TERRAIN',
            )
            await self._park_after_navigation(
                goal_handle, 'PARK_AT_TERRAIN'
            )
        if settings['terrain'] != 'none':
            command = 'climb_up' if settings['terrain'] == 'up' else 'climb_down'
            self._feedback(goal_handle, 'TRAVERSE_TERRAIN', command)
            await self._robot_command(goal_handle, command, timeout=180.0)
        if goal.navigate_to_pickup:
            await self._navigate(
                goal_handle,
                goal.pickup_pose,
                settings['navigation_timeout'],
                'NAVIGATE_PICKUP',
            )
            await self._park_after_navigation(
                goal_handle, 'PARK_AT_PICKUP'
            )

        self._feedback(goal_handle, 'ACQUIRE_TARGET', 'collecting stable samples')
        target = await self._wait_target(goal_handle, settings)
        pregrasp, grasp, retreat = arm_waypoints(
            target,
            settings['pregrasp_backoff'],
            settings['pregrasp_lift'],
            settings['grasp_forward'],
            settings['retreat_backoff'],
            settings['retreat_lift'],
        )
        tool = settings['tool']
        self._feedback(goal_handle, 'PREPARE_TOOL', 'selecting ' + tool)
        await self._robot_command(goal_handle, 'select_' + tool, timeout=10.0)
        await self._robot_command(
            goal_handle,
            'gripper_open' if tool == 'gripper' else 'suction_off',
            timeout=10.0,
        )

        self._feedback(goal_handle, 'PREGRASP', 'moving to pre-grasp point')
        await self._robot_command(
            goal_handle, 'arm_move', [value * 1000 for value in pregrasp]
        )
        self._feedback(goal_handle, 'APPROACH', 'moving to grasp point')
        await self._robot_command(
            goal_handle, 'arm_move', [value * 1000 for value in grasp]
        )
        self._feedback(goal_handle, 'GRASP', 'activating ' + tool)
        await self._robot_command(
            goal_handle,
            'gripper_close' if tool == 'gripper' else 'suction_on',
            timeout=10.0,
        )
        self._feedback(goal_handle, 'RETREAT', 'moving away with the object')
        await self._robot_command(
            goal_handle, 'arm_move', [value * 1000 for value in retreat]
        )

    async def _execute(self, goal_handle):
        self.active_goal_handle = goal_handle
        result = TraverseAndPick.Result()
        try:
            settings = self._normalize_goal(goal_handle.request)
            await self._run_mission(goal_handle, settings)
            result.success = True
            result.detail = (
                'sequence completed; tool output confirmed, '
                'object retention is not sensor-verified'
            )
            self._feedback(goal_handle, 'COMPLETE', result.detail)
            goal_handle.succeed()
        except MissionCanceled as error:
            await self._cancel_navigation()
            await self._stop_outputs()
            result.success = False
            result.detail = str(error)
            goal_handle.canceled()
        except (MissionError, ValueError) as error:
            await self._cancel_navigation()
            await self._stop_outputs()
            result.success = False
            result.detail = str(error)
            self.get_logger().error(result.detail)
            goal_handle.abort()
        finally:
            self.collect_target = False
            self.active_goal_handle = None
            with self.reservation_lock:
                self.goal_reserved = False
        return result

    def close(self):
        """Destroy action resources before the node shuts down."""
        self.action_server.destroy()
        self.nav_client.destroy()


def main(args=None):
    """Run the mission server with callbacks available during long actions."""
    rclpy.init(args=args)
    node = MissionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
