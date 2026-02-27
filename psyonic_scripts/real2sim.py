from isaaclab.app import AppLauncher
"""
Starts IsaacLab App. Needs to be above everything else
"""
app_launcher = AppLauncher()
simulation_app = app_launcher.app


from pprint import pprint

import os
import time
import numpy as np

import omni.kit.commands
import omni.usd

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.robot_motion.motion_generation")
enable_extension("isaacsim.ros2.bridge")
import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from isaaclab.assets import ArticulationCfg, Articulation, AssetBaseCfg, RigidObject, RigidObjectCfg
import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveSceneCfg, InteractiveScene
from isaaclab.utils import configclass
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg
from isaaclab.controllers import DifferentialIKControllerCfg


import torch


@configclass
class ActionsCfg:
    """
    Actions in ManagerBasedEnv
    """
    # Joint actions no offset needed
    hand_joint_positions = mdp.JointPositionActionCfg(
        asset_name="robot", 
        use_default_offset=False, 
        joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "index_q1", "middle_q1", "ring_q1", "pinky_q1", "thumb_q1", "thumb_q2"]
    )


@configclass
class ObservationsCfg:
    """
    Observations for ManagerBasedEnv
    """

    @configclass
    class RobotObsCfg(ObsGroup):
        """
        Observes joint positions of Robot
        """
        joint_pos = ObsTerm(func=mdp.joint_pos)

    Robot_obs: RobotObsCfg = RobotObsCfg()


    @configclass
    class LastAction(ObsGroup):
        """
        Observes last action performed by Robot
        """
        last_action = ObsTerm(func=mdp.last_action)
    
    Last_action: LastAction = LastAction()



@configclass
class EventCfg:
    """
    Events for ManagerBasedEnv
    """

    #resets Robot joints when called
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "index_q1", "middle_q1", "ring_q1", "pinky_q1", "thumb_q1", "thumb_q2"]),
            "position_range": (-0.0001, 0.00001),
            "velocity_range": (-0.0001, 0.0001),
        },
    )


@configclass
class PsyonicURSceneCfg(InteractiveSceneCfg):
    """
    Interactive Scene for Psyonic Robot
    """

    # Initializes the Articulation of psyonic robot
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot", 
        spawn=sim_utils.UsdFileCfg(
            usd_path="USD_assets/psyonic_UR_right_playground.usd", ##############################
        ),
        # initial position of robot
        init_state=ArticulationCfg.InitialStateCfg(
            #root position
            pos=(0.0, 0.0, 0.035),
            # joint positions
            joint_pos={
                "shoulder_pan_joint": -0.2, 
                "shoulder_lift_joint": -1.27, 
                "elbow_joint": 2.0, 
                "wrist_1_joint": -0.80, 
                "wrist_2_joint": 1.28, 
                "wrist_3_joint": -1.36, 
                "index_q1": 0.0, 
                "middle_q1": 0.0, 
                "ring_q1": 0.0, 
                "pinky_q1": 0.0, 
                "thumb_q1": 0.0, 
                "thumb_q2": 0.0
            }
        ),
        # initializaes actuators of Robot
        actuators={
            "shoulder": ImplicitActuatorCfg(
                                        joint_names_expr=["shoulder_.*"],
                                        stiffness=1320.0, #1320
                                        damping=72.6636085, #72.66
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "elbow": ImplicitActuatorCfg(
                                        joint_names_expr=["elbow_joint"],
                                        stiffness=600.0, #600
                                        damping=34.64101615, #34.64
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "wrist": ImplicitActuatorCfg(
                                        joint_names_expr=["wrist_.*"],
                                        stiffness=216.0, #216
                                        damping=29.39387691, #29.4
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "hand": ImplicitActuatorCfg(
                                        joint_names_expr=["index_q1", "middle_q1", "ring_q1", "pinky_q1", "thumb_q1", "thumb_q2"],
                                        stiffness=10000.0,
                                        damping=500.0,
                                        friction=0.0,
                                        armature=0.0,
                                    ),
                                    
        }
    )

    # Add frame for center of the hand (used for data collection)
    center_hand: FrameTransformerCfg = FrameTransformerCfg(
    prim_path="{ENV_REGEX_NS}/Robot/ur5e/wrist_3_link",
    target_frames=[
        FrameTransformerCfg.FrameCfg(
            name="center_hand",
            prim_path="{ENV_REGEX_NS}/Robot/ur5e/wrist_3_link",
            offset=OffsetCfg(
                pos=(0.0, 0.04, 0.11),
            ),
        ),
    ],
)
    # Initializes the ground floot
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # Initializes the lights of scene
    dome_light = AssetBaseCfg(
        prim_path="/World/dome_light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0),
    )


@configclass
class PickupCupCfg(ManagerBasedEnvCfg):
    """
    Sets up the ManagerBasedEnv for the Interactive Scene
    """

    # Initializes interactive scene
    scene = PsyonicURSceneCfg(num_envs=1, env_spacing=2.0)

    # Initializes observations, actions, and events in ManagerBasedEnv
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    # Controls the viewing angle and simulation settings
    def __post_init__(self):
        
        # viewer settings
        self.viewer.eye = [1.25, 0.0, 1.75]
        self.viewer.lookat = [0.35, 0.0, 0.0]

        # step settings
        self.decimation = 1  # env step ex: every 4 sim steps: (200Hz / 4 = 50Hz)
        # simulation settings
        self.sim.dt = 0.05  # sim step every 5ms: 200Hz


class PsyonicNode(Node):
    """
    Manages the ROS2 Node for Joint State Subscriptions
    """
    def __init__(self):
        super().__init__('psyonic_node')
        
        # qos profile for volatile data leading to faster polling with queue of 1 to get latest reading only
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1
        )

        # creates subscriber object to topic Psyonic_Real2Sim_Topic
        self.subscriber = self.create_subscription(JointState, 'Psyonic_Real2Sim_Topic', self.sim_callback, qos_profile_best_effort)
        
        # prevent non-use error (useless)
        self.subscriber

        # initializes MangerBasedEnv
        env_cfg = PickupCupCfg()

        # plugs in config
        self.env = ManagerBasedEnv(cfg=env_cfg)
        
        # joint names of msg
        self.msg_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q1','index_q2', 'middle_q2', 'ring_q2', 'pinky_q2', 'thumb_q2']
        
        # initializes robot into class variable
        self.robot = self.env.scene["robot"]

        # initializes actions to speed up allocation
        actions = torch.zeros(self.env.num_envs, self.env.action_manager.total_action_dim, device=self.env.device)

        # initial reset of IL environment
        self.env.reset()

    # callback for each subscription msg
    def sim_callback(self, msg):
        self.msg = msg

        # offsets for arm position if needed
        self.msg.position[0] += 1.5708 #####ADD ARM OFFSETS HERE!!!
        # self.msg.position[1] += 0.0 #####ADD ARM OFFSETS HERE!!!
        # self.msg.position[2] += 0.0 #####ADD ARM OFFSETS HERE!!!
        # self.msg.position[3] += 0.0 #####ADD ARM OFFSETS HERE!!!
        # self.msg.position[4] += 0.0 #####ADD ARM OFFSETS HERE!!!
        # self.msg.position[5] += 0.0 #####ADD ARM OFFSETS HERE!!!

        # converts psyonic finger position into degrees then radians (messy TODO)
        self.msg.position[6] *= (3.14/180/1.1)
        self.msg.position[7] *= (3.14/180/1.1)
        self.msg.position[8] *= (3.14/180/1.1)
        self.msg.position[9] *= (3.14/180/1.1)
        self.msg.position[10] *= (3.14/180/1.1)
        self.msg.position[11] *= (3.14/180/1.1)
        self.msg.position[10], self.msg.position[11] = self.msg.position[11], self.msg.position[10]
        
        # logs msg for debugging purposes
        self.get_logger().info(f'recieved: {self.msg.position}')
        
        # transfers msg into a tensor
        action = torch.tensor(msg.position, device=self.env.device)
        
        # creates appropriate action space if multiple environments
        actions = action.repeat(self.env.num_envs, 1)

        # steps through environment with desired action
        obs, _ = self.env.step(actions)


    
def main():
    
    # initializes ros2
    ros2.init()

    # initializes ros2 node
    psy_sub = PsyonicNode()

    # spin indefinitely
    ros2.spin(psy_sub)

    # after interrupt, destroy node for clean exit
    psy_sub.destroy_node()
    ros2.shutdown()


if __name__ == "__main__":
    print("Starting real2sim")
    main()
