# Talks to MoveIt2's move_group node.
# Receives a clean Pose in base_link frame and handles all motion planning and execution.
# All other files feed into this one — nothing else touches MoveIt2 directly.

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from enum import Enum
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes

class MoveResult(Enum):
    SUCCEEDED = "succeeded" # motion completed as planned
    FAILED = "failed"       # motion started but didn't finish
    ABORTED = "aborted"     # planning succeeded but execution was cancelled
    INVALID = "invalid"     # bad input - unreachable pose

class Arm(Enum):
    RIGHT = "right"
    LEFT = "left"

class MoveItClient:
    def __init__(self):
        #rclpy.init removed. rclpy init should be called for each use cases. 
        self.node = Node('moveit_client')
        self.callback_group = ReentrantCallbackGroup()

        #creates a moveit2 object
        self.moveit2_r = MoveIt2(
            node=self.node,
            joint_names=['arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
                'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7'],
            base_link_name='base_link',
            end_effector_name='end_effector_r_link',
            group_name='arm_r',
            callback_group=self.callback_group,
            use_move_group_action=True,
        )

        self.moveit2_l = MoveIt2(
            node=self.node,
            joint_names=['arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
                'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7'],
            base_link_name='base_link',
            end_effector_name='end_effector_l_link',
            group_name='arm_l',
            callback_group=self.callback_group,
            use_move_group_action=True,
        )

        # Wait for move_group action servers before accepting any motion commands
        self.node.get_logger().info('Waiting for move_group action servers...')
        while not self.moveit2_r._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn('Right arm move_group not yet available...')
            rclpy.spin_once(self.node, timeout_sec=0.1)
        while not self.moveit2_l._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn('Left arm move_group not yet available...')
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info('move_group action servers ready')

        # Spin until joint states are available before any motion method is called
        while self.moveit2_r.joint_state is None or self.moveit2_l.joint_state is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.get_logger().info('Joint states ready')

    def _arm(self, arm: Arm) -> MoveIt2:
        return self.moveit2_r if arm == Arm.RIGHT else self.moveit2_l

#MOVE Functions
    def _wait(self, moveit2) -> bool:
        while moveit2._MoveIt2__is_motion_requested or moveit2._MoveIt2__is_executing:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return moveit2.motion_suceeded

    def move_to_pose(self, pose, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        moveit2.max_velocity = velocity_scaling
        moveit2.max_acceleration = acceleration_scaling
        moveit2.allowed_planning_time = 10.0
        moveit2.num_planning_attempts = 5
        moveit2.move_to_pose(pose=pose)
        success = self._wait(moveit2)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._to_move_result(success, moveit2)

    def move_to_position(self, x, y, z, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        return self.move_with_stomp_smoothing(x, y, z, arm, velocity_scaling, acceleration_scaling)

    def move_to_joints(self, joint_positions: list, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        moveit2.max_velocity = velocity_scaling
        moveit2.max_acceleration = acceleration_scaling
        moveit2.move_to_configuration(joint_positions)
        success = self._wait(moveit2)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._to_move_result(success, moveit2)

    def cartesian_move(self, pose, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        moveit2.pipeline_id = "pilz_industrial_motion_planner"
        moveit2.planner_id = "LIN"

        moveit2.max_velocity = velocity_scaling
        moveit2.max_acceleration = acceleration_scaling
        moveit2.allowed_planning_time = 10.0

        moveit2.move_to_pose(pose=pose)
        success = self._wait(moveit2)
        rclpy.spin_once(self.node, timeout_sec=0.2)

        #ALWAYS reset
        moveit2.pipeline_id = ""
        moveit2.planner_id = ""

        return self._to_move_result(success, moveit2)

    def move_to_position_measured(self, x, y, z, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1):
        """Plan, measure, then execute. Returns (MoveResult, waypoint_count, path_length_rad)."""
        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        moveit2.max_velocity = velocity_scaling
        moveit2.max_acceleration = acceleration_scaling
        moveit2.allowed_planning_time = 10.0
        moveit2.num_planning_attempts = 5

        future = moveit2.plan_async(
            position=[x, y, z],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            tolerance_orientation=3.14159,
        )
        if future is None:
            return MoveResult.INVALID, 0, 0.0

        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        trajectory = moveit2.get_trajectory(future)
        if trajectory is None:
            return MoveResult.INVALID, 0, 0.0

        points = trajectory.points
        waypoint_count = len(points)
        path_length = sum(
            math.sqrt(sum((a - b) ** 2 for a, b in zip(points[i].positions, points[i + 1].positions)))
            for i in range(len(points) - 1)
        )

        moveit2.execute(trajectory)
        while moveit2._MoveIt2__is_motion_requested or moveit2._MoveIt2__is_executing:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(self.node, timeout_sec=0.2)

        result = self._to_move_result(moveit2.motion_suceeded, moveit2)
        return result, waypoint_count, path_length

    def move_with_stomp_smoothing(self, x, y, z, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            MotionPlanRequest, PlanningOptions, Constraints,
            JointConstraint, TrajectoryConstraints,
            PositionConstraint, OrientationConstraint, BoundingVolume,
        )
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import PoseStamped

        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        joint_names = (
            ['arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
             'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7']
            if arm == Arm.RIGHT else
            ['arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
             'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7']
        )
        group_name   = 'arm_r' if arm == Arm.RIGHT else 'arm_l'
        end_effector = 'end_effector_r_link' if arm == Arm.RIGHT else 'end_effector_l_link'

        # Step 1: plan with OMPL (no execution yet)
        moveit2.allowed_planning_time = 10.0
        moveit2.num_planning_attempts = 5
        ompl_future = moveit2.plan_async(
            position=[x, y, z],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            tolerance_orientation=0.01,
        )
        if ompl_future is None:
            return MoveResult.INVALID

        while not ompl_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        ompl_trajectory = moveit2.get_trajectory(ompl_future)
        if ompl_trajectory is None:
            return MoveResult.INVALID

        self.node.get_logger().info('OMPL plan succeeded, attempting STOMP smoothing')

        # Step 2: convert OMPL trajectory to TrajectoryConstraints seed for STOMP
        tc = TrajectoryConstraints()
        for point in ompl_trajectory.points:
            c = Constraints()
            for name, pos in zip(joint_names, point.positions):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = pos
                jc.tolerance_above = 0.2
                jc.tolerance_below = 0.2
                jc.weight = 1.0
                c.joint_constraints.append(jc)
            tc.constraints.append(c)

        # Step 3: build goal constraints from target position
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.w = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target.pose]

        pos_c = PositionConstraint()
        pos_c.header = target.header
        pos_c.link_name = end_effector
        pos_c.constraint_region = bv
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header = target.header
        ori_c.link_name = end_effector
        ori_c.orientation = target.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.01
        ori_c.absolute_y_axis_tolerance = 0.01
        ori_c.absolute_z_axis_tolerance = 0.01
        ori_c.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pos_c]
        goal_constraints.orientation_constraints = [ori_c]

        # Step 4: build STOMP MoveGroup action goal with OMPL seed injected
        request = MotionPlanRequest()
        request.group_name = group_name
        request.pipeline_id = 'stomp'
        request.planner_id = ''
        request.allowed_planning_time = 30.0
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        request.goal_constraints = [goal_constraints]
        request.trajectory_constraints = tc

        planning_options = PlanningOptions()
        planning_options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = planning_options

        # Step 5: send to STOMP, fall back to OMPL trajectory on failure
        action_client = moveit2._MoveIt2__move_action_client
        send_future = action_client.send_goal_async(goal)
        while not send_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('STOMP goal rejected, falling back to OMPL trajectory')
            return self._execute_fallback(moveit2, ompl_trajectory)

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:  # MoveItErrorCodes.SUCCESS
            self.node.get_logger().info('STOMP smoothing succeeded')
            rclpy.spin_once(self.node, timeout_sec=0.2)
            return MoveResult.SUCCEEDED

        self.node.get_logger().warn(f'STOMP failed (error={error_code}), falling back to OMPL trajectory')
        return self._execute_fallback(moveit2, ompl_trajectory)

    def move_pose_with_stomp_smoothing(self, pose: Pose, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            MotionPlanRequest, PlanningOptions, Constraints,
            JointConstraint, TrajectoryConstraints,
            PositionConstraint, OrientationConstraint, BoundingVolume,
        )
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import PoseStamped

        moveit2 = self._arm(arm)
        moveit2.motion_suceeded = False
        joint_names = (
            ['arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
             'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7']
            if arm == Arm.RIGHT else
            ['arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
             'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7']
        )
        group_name   = 'arm_r' if arm == Arm.RIGHT else 'arm_l'
        end_effector = 'end_effector_r_link' if arm == Arm.RIGHT else 'end_effector_l_link'

        # Step 1: plan with OMPL (no execution yet). Tight orientation tolerance for exact poses.
        moveit2.allowed_planning_time = 10.0
        moveit2.num_planning_attempts = 5
        ompl_future = moveit2.plan_async(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            tolerance_orientation=0.1,
        )
        if ompl_future is None:
            self.node.get_logger().error('move_pose_with_stomp_smoothing: plan_async returned None (IK failed or bad input)')
            return MoveResult.INVALID

        while not ompl_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        ompl_trajectory = moveit2.get_trajectory(ompl_future)
        if ompl_trajectory is None:
            self.node.get_logger().error(
                f'move_pose_with_stomp_smoothing: OMPL found no solution for '
                f'pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) '
                f'quat=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, '
                f'{pose.orientation.z:.3f}, {pose.orientation.w:.3f}) '
                f'tolerance=0.1 rad'
            )
            return MoveResult.INVALID

        self.node.get_logger().info('OMPL plan succeeded, attempting STOMP smoothing')

        # Step 2: convert OMPL trajectory to TrajectoryConstraints seed for STOMP
        tc = TrajectoryConstraints()
        for point in ompl_trajectory.points:
            c = Constraints()
            for name, pos in zip(joint_names, point.positions):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = pos
                jc.tolerance_above = 0.2
                jc.tolerance_below = 0.2
                jc.weight = 1.0
                c.joint_constraints.append(jc)
            tc.constraints.append(c)

        # Step 3: build goal constraints from target pose
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.pose = pose

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]

        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [target.pose]

        pos_c = PositionConstraint()
        pos_c.header = target.header
        pos_c.link_name = end_effector
        pos_c.constraint_region = bv
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header = target.header
        ori_c.link_name = end_effector
        ori_c.orientation = target.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.1
        ori_c.absolute_y_axis_tolerance = 0.1
        ori_c.absolute_z_axis_tolerance = 0.1
        ori_c.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pos_c]
        goal_constraints.orientation_constraints = [ori_c]

        # Step 4: build STOMP MoveGroup action goal with OMPL seed injected
        request = MotionPlanRequest()
        request.group_name = group_name
        request.pipeline_id = 'stomp'
        request.planner_id = ''
        request.allowed_planning_time = 30.0
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        request.goal_constraints = [goal_constraints]
        request.trajectory_constraints = tc

        planning_options = PlanningOptions()
        planning_options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = planning_options

        # Step 5: send to STOMP, fall back to OMPL trajectory on failure
        action_client = moveit2._MoveIt2__move_action_client
        send_future = action_client.send_goal_async(goal)
        while not send_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('STOMP goal rejected, falling back to OMPL trajectory')
            return self._execute_fallback(moveit2, ompl_trajectory)

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:  # MoveItErrorCodes.SUCCESS
            self.node.get_logger().info('STOMP smoothing succeeded')
            rclpy.spin_once(self.node, timeout_sec=0.2)
            return MoveResult.SUCCEEDED

        self.node.get_logger().warn(f'STOMP failed (error={error_code}), falling back to OMPL trajectory')
        return self._execute_fallback(moveit2, ompl_trajectory)

    def _execute_fallback(self, moveit2, trajectory) -> MoveResult:
        """Execute the OMPL trajectory directly when STOMP fails."""
        self.node.get_logger().info('Executing OMPL fallback trajectory')
        moveit2.execute(trajectory)
        while moveit2._MoveIt2__is_motion_requested or moveit2._MoveIt2__is_executing:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._to_move_result(moveit2.motion_suceeded, moveit2)

    def move_to_home(self, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        return self.move_to_joints(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            arm=arm,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling
        )

#Monitor Functions
    def get_joint_positions(self, arm=Arm.RIGHT) -> list:
        moveit2 = self._arm(arm)
        joint_state = moveit2.joint_state

        if joint_state is None:
            self.node.get_logger().warn('Joint state not yet received')
            return []
        return list(joint_state.position)

    def get_current_pose(self, arm=Arm.RIGHT) -> Pose | None:
        moveit2 = self._arm(arm)
        future = moveit2.compute_fk_async()

        if future is None:
            self.node.get_logger().warn('Forward Kinematics computation failed')
            return None

        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        result = moveit2.get_compute_fk_result(future)

        if result is None:
            return None

        return result.pose

    def check_reachable(self, pose: Pose, arm: Arm = Arm.RIGHT) -> bool:
        moveit2 = self._arm(arm)
        future = moveit2.compute_ik_async(pose)
        if future is None:
            return False
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        result = moveit2.get_compute_ik_result(future)
        return result is not None and len(result.solution.joint_state.position) > 0

#Others
    def _to_move_result(self, success: bool, moveit2: MoveIt2) -> MoveResult:
        if success:
            return MoveResult.SUCCEEDED

        error = moveit2.get_last_execution_error_code()        # this is the moveit method to return the raw error

        if error is None:
            return MoveResult.INVALID

        if error.val == MoveItErrorCodes.PREEMPTED:
            return MoveResult.ABORTED

        return MoveResult.FAILED


    def destroy(self):
        self.node.destroy_node()
        pos_c = PositionConstraint()
        pos_c.header = target.header
        pos_c.link_name = end_effector
        pos_c.constraint_region = bv
        pos_c.weight = 1.0

        ori_c = OrientationConstraint()
        ori_c.header = target.header
        ori_c.link_name = end_effector
        ori_c.orientation = target.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.01
        ori_c.absolute_y_axis_tolerance = 0.01
        ori_c.absolute_z_axis_tolerance = 0.01
        ori_c.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pos_c]
        goal_constraints.orientation_constraints = [ori_c]

        # Step 4: build STOMP MoveGroup action goal with OMPL seed injected
        request = MotionPlanRequest()
        request.group_name = group_name
        request.pipeline_id = 'stomp'
        request.planner_id = ''
        request.allowed_planning_time = 30.0
        request.num_planning_attempts = 1
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        request.goal_constraints = [goal_constraints]
        request.trajectory_constraints = tc

        planning_options = PlanningOptions()
        planning_options.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = planning_options

        # Step 5: send to STOMP, fall back to OMPL trajectory on failure
        action_client = moveit2._MoveIt2__move_action_client
        send_future = action_client.send_goal_async(goal)
        while not send_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('STOMP goal rejected, falling back to OMPL trajectory')
            return self._execute_fallback(moveit2, ompl_trajectory)

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:  # MoveItErrorCodes.SUCCESS
            self.node.get_logger().info('STOMP smoothing succeeded')
            rclpy.spin_once(self.node, timeout_sec=0.2)
            return MoveResult.SUCCEEDED

        self.node.get_logger().warn(f'STOMP failed (error={error_code}), falling back to OMPL trajectory')
        return self._execute_fallback(moveit2, ompl_trajectory)

    def _execute_fallback(self, moveit2, trajectory) -> MoveResult:
        """Execute the OMPL trajectory directly when STOMP fails."""
        self.node.get_logger().info('Executing OMPL fallback trajectory')
        moveit2.execute(trajectory)
        while moveit2._MoveIt2__is_motion_requested or moveit2._MoveIt2__is_executing:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        rclpy.spin_once(self.node, timeout_sec=0.2)
        return self._to_move_result(moveit2.motion_suceeded, moveit2)

    def move_to_home(self, arm=Arm.RIGHT, velocity_scaling=0.1, acceleration_scaling=0.1) -> MoveResult:
        return self.move_to_joints(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            arm=arm,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling
        )

#Monitor Functions
    def get_joint_positions(self, arm=Arm.RIGHT) -> list:
        moveit2 = self._arm(arm)
        joint_state = moveit2.joint_state

        if joint_state is None:
            self.node.get_logger().warn('Joint state not yet received')
            return []
        return list(joint_state.position)

    def get_current_pose(self, arm=Arm.RIGHT) -> Pose | None:
        moveit2 = self._arm(arm)
        future = moveit2.compute_fk_async()

        if future is None:
            self.node.get_logger().warn('Forward Kinematics computation failed')
            return None

        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        result = moveit2.get_compute_fk_result(future)

        if result is None:
            return None

        return result.pose

    def check_reachable(self, pose: Pose, arm: Arm = Arm.RIGHT) -> bool:
        moveit2 = self._arm(arm)
        future = moveit2.compute_ik_async(pose)
        if future is None:
            return False
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        result = moveit2.get_compute_ik_result(future)
        return result is not None and len(result.solution.joint_state.position) > 0

#Others
    def _to_move_result(self, success: bool, moveit2: MoveIt2) -> MoveResult:
        if success:
            return MoveResult.SUCCEEDED

        error = moveit2.get_last_execution_error_code()        # this is the moveit method to return the raw error

        if error is None:
            return MoveResult.INVALID

        if error.val == MoveItErrorCodes.PREEMPTED:
            return MoveResult.ABORTED

        return MoveResult.FAILED


    def destroy(self):
        self.node.destroy_node()
