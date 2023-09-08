#!/usr/bin/env python3
import rospy

from giskardpy.configs.behavior_tree_config import OpenLoopBTConfig
from giskardpy.configs.collision_avoidance_config import CollisionAvoidanceConfig
from giskardpy.configs.giskard import Giskard
from giskardpy.configs.world_config import WorldConfig, Derivatives, np
from giskardpy.configs.robot_interface_config import RobotInterfaceConfig
from giskardpy.configs.qp_controller_config import QPControllerConfig, SupportedQPSolver
from giskardpy.model.collision_world_syncer import CollisionCheckerLib


class WorldWithTiagoDual(WorldConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        super().__init__()
        self.map_name = map_name
        self.odom = 'odom'
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.set_default_limits({Derivatives.velocity: 1,
                                              Derivatives.acceleration: np.inf,
                                              Derivatives.jerk: 30})
        self.add_empty_link(self.map_name)
        self.add_empty_link(self.odom)
        self.add_fixed_joint(self.map_name,
                             self.odom)
        self.add_robot_from_parameter_server()
        root_link_name = self.get_root_link_of_group(self.robot_group_name)
        self.add_diff_drive_joint(name=self.drive_joint_name,
                                               parent_link_name=self.odom,
                                               child_link_name=root_link_name,
                                               translation_limits={
                                                   Derivatives.velocity: 0.4,
                                                   Derivatives.acceleration: np.inf,
                                                   Derivatives.jerk: 10,
                                               },
                                               rotation_limits={
                                                   Derivatives.velocity: 0.5,
                                                   Derivatives.acceleration: np.inf,
                                                   Derivatives.jerk: 20
                                               },
                                               robot_group_name=self.robot_group_name)
        left_gripper_tool_frame = 'left_gripper_tool_frame'
        right_gripper_tool_frame = 'right_gripper_tool_frame'
        self.add_empty_link(left_gripper_tool_frame)
        self.add_fixed_joint(parent_link='gripper_left_grasping_frame',
                             child_link=left_gripper_tool_frame,
                             homogenous_transform=np.array([[1,0,0,0.07],
                                                            [0,1,0,0],
                                                            [0,0,1,0],
                                                            [0,0,0,1]]))
        self.add_empty_link(right_gripper_tool_frame)
        self.add_fixed_joint(parent_link='gripper_right_grasping_frame',
                             child_link=right_gripper_tool_frame,
                             homogenous_transform=np.array([[1, 0, 0, 0.07],
                                                            [0, 1, 0, 0],
                                                            [0, 0, 1, 0],
                                                            [0, 0, 0, 1]]))


class TiagoCollisionAvoidanceConfig(CollisionAvoidanceConfig):
    def __init__(self, drive_joint_name: str = 'brumbrum'):
        super().__init__(collision_checker=CollisionCheckerLib.none)
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.load_self_collision_matrix('package://giskardpy/self_collision_matrices/iai/tiago_dual_tall.srdf')
        self.overwrite_external_collision_avoidance(self.drive_joint_name,
                                                    number_of_repeller=2,
                                                    soft_threshold=0.2,
                                                    hard_threshold=0.1)
        self.fix_joints_for_collision_avoidance(['head_1_joint',
                                                 'head_2_joint',
                                                 'gripper_left_left_finger_joint',
                                                 'gripper_left_right_finger_joint',
                                                 'gripper_right_left_finger_joint',
                                                 'gripper_right_right_finger_joint'])
        self.overwrite_external_collision_avoidance('arm_right_7_joint',
                                                    number_of_repeller=4,
                                                    soft_threshold=0.05,
                                                    hard_threshold=0.0,
                                                    max_velocity=0.2)
        self.overwrite_external_collision_avoidance('arm_left_7_joint',
                                                    number_of_repeller=4,
                                                    soft_threshold=0.05,
                                                    hard_threshold=0.0,
                                                    max_velocity=0.2)
        self.set_default_self_collision_avoidance(hard_threshold=0.04,
                                                  soft_threshold=0.08)
        self.set_default_external_collision_avoidance(hard_threshold=0.03,
                                                      soft_threshold=0.08)

class TiagoMultiverse(RobotInterfaceConfig):
    map_name: str
    localization_joint_name: str
    odom_link_name: str
    drive_joint_name: str

    def __init__(self,
                 map_name: str = 'map',
                 localization_joint_name: str = 'localization',
                 odom_link_name: str = 'odom',
                 drive_joint_name: str = 'brumbrum'):
        self.map_name = map_name
        self.localization_joint_name = localization_joint_name
        self.odom_link_name = odom_link_name
        self.drive_joint_name = drive_joint_name

    def setup(self):
        self.sync_joint_state_topic('/joint_states')
        self.sync_odometry_topic(odometry_topic='/odom',
                                 joint_name=self.drive_joint_name)
        self.add_follow_joint_trajectory_server(namespace='/arm_left_controller/follow_joint_trajectory',
                                                state_topic='/arm_left_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/arm_right_controller/follow_joint_trajectory',
                                                state_topic='/arm_right_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/head_controller/follow_joint_trajectory',
                                                state_topic='/head_controller/state')
        self.add_follow_joint_trajectory_server(namespace='/torso_controller/follow_joint_trajectory',
                                                state_topic='/torso_controller/state')
        self.add_base_cmd_velocity(cmd_vel_topic='/cmd_vel',
                                   joint_name=self.drive_joint_name)

if __name__ == '__main__':
    rospy.init_node('giskard')
    giskard = Giskard(world_config=WorldWithTiagoDual(),
                      collision_avoidance_config=TiagoCollisionAvoidanceConfig(),
                      robot_interface_config=TiagoMultiverse(),
                      behavior_tree_config=OpenLoopBTConfig(),
                      qp_controller_config=QPControllerConfig())
    giskard.live()
