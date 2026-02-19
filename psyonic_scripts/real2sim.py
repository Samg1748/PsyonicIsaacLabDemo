from isaaclab.app import AppLauncher

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
    # 1. Hand remains as standard joint positions
    hand_joint_positions = mdp.JointPositionActionCfg(
        asset_name="robot", 
        use_default_offset=False, 
        joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "index_q1", "middle_q1", "ring_q1", "pinky_q1", "thumb_q1", "thumb_q2"]
    )


@configclass
class ObservationsCfg:

    @configclass
    class RobotObsCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos)

    Robot_obs: RobotObsCfg = RobotObsCfg()


    @configclass
    class TargetObsCfg(ObsGroup):
        ik_pos = ObsTerm(func=mdp.root_pos_w, params={"asset_cfg": SceneEntityCfg(name="target")})

    Target_obs: TargetObsCfg = TargetObsCfg()


    @configclass
    class LastAction(ObsGroup):
        last_action = ObsTerm(func=mdp.last_action)
    
    Last_action: LastAction = LastAction()



@configclass
class EventCfg:
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
class MyEventCfg:
    # This event happens every time an episode resets
    reset_target_root = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0), "yaw": (-3.14, 3.14)},
            "velocity_range": {}, # Default is zero if left empty
            "asset_cfg": SceneEntityCfg("target"),
        },
    )


@configclass
class PsyonicURSceneCfg(InteractiveSceneCfg):

    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot", 
        spawn=sim_utils.UsdFileCfg(
            usd_path="PsyonicIsaacLabDemo/USD_assets/psyonic_UR_right_playground.usd", ##############################
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
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
        actuators={
            "shoulder": ImplicitActuatorCfg(
                                        joint_names_expr=["shoulder_.*"],
                                        stiffness=1320000.0, #1320
                                        damping=72000.6636085, #72.66
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "elbow": ImplicitActuatorCfg(
                                        joint_names_expr=["elbow_joint"],
                                        stiffness=600000.0, #600
                                        damping=34000.64101615, #34.64
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "wrist": ImplicitActuatorCfg(
                                        joint_names_expr=["wrist_.*"],
                                        stiffness=2160000.0, #216
                                        damping=29000.39387691, #29.4
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "hand": ImplicitActuatorCfg(
                                        joint_names_expr=["index_q1", "middle_q1", "ring_q1", "pinky_q1", "thumb_q1", "thumb_q2"],
                                        stiffness=10000.0,
                                        damping=50.0,
                                        friction=0.0,
                                        armature=0.0,
                                    ),
                                    
        }
    )
    
    target: RigidObjectCfg = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/target",
            spawn=sim_utils.UsdFileCfg(
                usd_path="PsyonicIsaacLabDemo/USD_assets/beaker_500ml.usd", 
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    rigid_body_enabled=True,
                    max_linear_velocity=1000.0,
                ),
                scale=(0.45, 0.45, 0.9), #0.5, 0.5, 1.0
                collision_props=sim_utils.CollisionPropertiesCfg(),
                mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(0.6, 0.3, 0.25)),
    )


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

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    dome_light = AssetBaseCfg(
        prim_path="/World/dome_light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0),
    )


@configclass
class PickupCupCfg(ManagerBasedEnvCfg):

    scene = PsyonicURSceneCfg(num_envs=1, env_spacing=2.0)

    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
    # viewer settings
    
        self.viewer.eye = [1.25, 0.0, 1.75]
        self.viewer.lookat = [0.35, 0.0, 0.0]

        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        # simulation settings
        self.sim.dt = 0.005  # sim step every 5ms: 200Hz


class PsyonicNode(Node):
    def __init__(self):
        super().__init__('psyonic_node')
        
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1
        )
        self.subscriber = self.create_subscription(JointState, 'Psyonic_Real2Sim_Topic', self.sim_callback, qos_profile_best_effort)
        self.subscriber

        env_cfg = PickupCupCfg()
        self.env = ManagerBasedEnv(cfg=env_cfg)
        self.msg_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q1','index_q2', 'middle_q2', 'ring_q2', 'pinky_q2', 'thumb_q2']
        

        self.robot = self.env.scene["robot"]
        actions = torch.zeros(self.env.num_envs, self.env.action_manager.total_action_dim, device=self.env.device)


        self.env.reset()


    def sim_callback(self, msg):
        self.msg = msg
        self.msg.position[0] += 1.5708 #####CHECK THISSSS
        self.get_logger().info(f'recieved: {self.msg.position}')
        action = torch.tensor(msg.position, device=self.env.device)
        actions = action.repeat(self.env.num_envs, 1)
        obs, _ = self.env.step(actions)


    
def main():
    ros2.init()
    psy_sub = PsyonicNode()

    ros2.spin(psy_sub)
    psy_sub.destroy_node()
    ros2.shutdown()


if __name__ == "__main__":
    print("Starting real2sim")
    main()
