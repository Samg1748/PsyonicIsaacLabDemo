from isaaclab.app import AppLauncher

app_launcher = AppLauncher()
simulation_app = app_launcher.app

# from isaaclab.sim import SimulationCfg, SimulationContext

# sim_cfg = SimulationCfg(dt=1/60, rendering_dt=1/60, device="gpu")
# sim = SimulationContext(sim_cfg)

# simulation_context = SimulationContext()

# import argparse
from pprint import pprint

import os
import time
import numpy as np

import omni.kit.commands
import omni.usd

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.robot_motion.motion_generation")
enable_extension("isaacsim.ros2.bridge")
# import rclpy as ros2
# from rclpy.node import Node
# from sensor_msgs.msg import JointState

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

from isaaclab_ros2.managers import Ros2JointStatePublisherCfg

import torch


@configclass
class ActionsCfg:
    # 1. Hand remains as standard joint positions
    hand_joint_positions = mdp.JointPositionActionCfg(
        asset_name="robot", 
        use_default_offset=False, 
        joint_names=["index_q1", "middle_q1", "ring_q1", "pinky_q1", "thumb_q1", "thumb_q2"]
    )
    
    # 2. Arm uses Differential IK
    arm_joint_positions = mdp.DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        body_name="wrist_3_link", # Must be a literal body name in the USD
        controller=DifferentialIKControllerCfg(
            command_type="pose", # Expects [x, y, z, qw, qx, qy, qz]
            ik_method="dls",
            use_relative_mode=False,
        ),
        # ADD THIS: This tells the IK to track the offset 
        # defined in your FrameTransformer
        body_offset=OffsetCfg(pos=(0.00, 0.04, 0.11)), #0.0, 0.04, 0.11
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
    class ObservationsCfg:
        # This automatically creates the ROS 2 publisher without needing rclpy in your script
        joint_state = Ros2JointStatePublisherCfg(
            topic_name="joint_states",
            asset_cfg=SceneEntityCfg("robot")
        )


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
class PsyonicURSceneCfg(InteractiveSceneCfg):

    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot", 
        spawn=sim_utils.UsdFileCfg(
            usd_path="psyonic_UR_right_playground.usd",
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={
                "shoulder_pan_joint": -0.2, #0.0
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
                                        stiffness=1320.0,
                                        damping=72.6636085,
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "elbow": ImplicitActuatorCfg(
                                        joint_names_expr=["elbow_joint"],
                                        stiffness=600.0,
                                        damping=34.64101615,
                                        friction=0.0,
                                        armature=0.0,
                                    ),
            "wrist": ImplicitActuatorCfg(
                                        joint_names_expr=["wrist_.*"],
                                        stiffness=216.0,
                                        damping=29.39387691,
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
                usd_path="assets/props/Beaker/beaker_500ml.usd",
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    rigid_body_enabled=True,
                    max_linear_velocity=1000.0,
                ),
                scale=(0.5, 0.5, 1.0),
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
                pos=(0.0, 0.04, 0.11), #0.0, 0.04, 0.11
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
        self.publisher = self.create_publisher(JointState, 'Psyonic_Topic', 10)
        

    def publish_actions(self, actions):
        self.msg = JointState()
        self.msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q1','index_q2', 'middle_q2', 'ring_q2', 'pinky_q2', 'thumb_q2']
        self.msg.position = actions[0].cpu().tolist()
        self.publisher.publish(self.msg)
        self.get_logger().info('Published actions to Psyonic_Topic')





class PickupCup:
    def __init__(self):

        env_cfg = PickupCupCfg()
        self.env = ManagerBasedEnv(cfg=env_cfg)

        self.robot = self.env.scene["robot"]

        actions = torch.zeros(self.env.num_envs, self.env.action_manager.total_action_dim, device=self.env.device)

        ros2.init()
        self.psy_node = PsyonicNode()

        # self.robot_indices = [0,1,2,3,4,5,6,7,8,9,10,15]
        self.robot_indices = [0,1,2,3,4,9]


        self.center_hand = self.env.scene["center_hand"]

        self.home_pos = torch.tensor([[0.55, -0.05, 0.3]], dtype=torch.float32, device=self.env.device)
        self.waypoint_1 = torch.tensor([[0.6, 0.0, 0.08]], dtype=torch.float32, device=self.env.device)
        self.waypoint_2 = torch.tensor([[0.6, 0.4, 0.08]], dtype=torch.float32, device=self.env.device)
        self.home = False

        self.hand_indices = torch.tensor([6, 7, 8, 9, 10, 15], device=self.env.device) ##########
        # self.hand_indices = torch.tensor([0, 1, 2, 3, 4, 9], device=self.env.device)
        self.arm_indices = torch.tensor([10, 11, 12, 13, 14, 15, 16], device=self.env.device)


        self.env.reset()
        self.grasping = False
        self.buf_x = 0.0
        self.buf_y = -0.1
        self.buf_z = 0.015


    def create_action(self, hand_joints, arm_pos):

        actions = torch.cat((hand_joints, arm_pos), dim=1)
        return actions


    def init_hand(self):
        # self.world.step(render=True)
        initial_hand_joints = torch.tensor([[0.2, 0.2, 0.2, 0.2, -1.7, 0.0]], dtype=torch.float32, device=self.env.device)
        
        initial_arm_pos = torch.tensor([[0.4, -0.15, 0.4, 0.707, -0.707, 0.0, 0.0]], device=self.env.device)
        initial_arm_pos = torch.tensor([[0.4, 0.0, 0.4, 0.707, 0.0, 0.707, 0.0]], device=self.env.device)
        init_action = self.create_action(initial_hand_joints, initial_arm_pos)
        for _ in range(20):
            self.obs, _ = self.env.step(init_action)
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
            print("Initializing hand position.")
            print(self.obs["Robot_obs"])

    def grasp(self):
        self.grasping = True

        self.target_pos = self.hand_pos
        grasp_hand_joints = torch.tensor([[0.52, 0.52, 0.52, 0.52, -1.7, 0.15]], dtype=torch.float32, device=self.env.device)
        arm_pos = self.obs["Last_action"][:, 6:]
        grasp_hand_action = self.create_action(grasp_hand_joints, arm_pos)
        self.obs, _ = self.env.step(grasp_hand_action)
        self.ros_command = self.obs["Robot_obs"]
        self.psy_node.publish_actions(self.ros_command)

    def grip_release(self):
        self.grasping = False

        self.target_pos = self.hand_pos
        grasp_hand_joints = torch.tensor([[0.2, 0.2, 0.2, 0.2, -1.7, 0.0]], dtype=torch.float32, device=self.env.device)
        arm_pos = self.obs["Last_action"][:, 6:]
        grasp_hand_action = self.create_action(grasp_hand_joints, arm_pos)
        self.obs, _ = self.env.step(grasp_hand_action)
        self.ros_command = self.obs["Robot_obs"]
        self.psy_node.publish_actions(self.ros_command)

    def step_to(self, position):
        self.waypoint = position
        self.target_pos = self.waypoint
        print(f"Stepping to position: {self.target_pos}\n\n")

        self.calc_vect_to_goal(self.target_pos)

        while (torch.max(torch.linalg.norm(self.vect_to_goal, dim=1)).item() >= 0.025):
            
            self.calc_vect_to_goal(self.target_pos)
            print(f"INTEREST!!!!!!! Distance to target: {torch.max(torch.linalg.norm(self.vect_to_goal, dim=1)).item()}")
            print(f"self.target_pos: {self.obs['Robot_obs']}")

            hand_actions = self.obs["Last_action"]
            hand_actions = hand_actions[:, :6]

            pos = self.target_pos[:, :3] 
            orientation = torch.tensor([[0.707, 0.0, 0.707, 0.0]], device=self.env.device)
            
            expanded_orient = orientation.expand(self.env.num_envs, -1)
            arm_pos = torch.cat((pos, expanded_orient), dim=1)

            actions = self.create_action(hand_actions, arm_pos)
            self.obs, _ = self.env.step(actions)
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)

    def step_to_cup(self):

        self.target_pos= self.obs["Target_obs"]
        self.calc_vect_to_goal(self.target_pos)

        while (torch.max(torch.linalg.norm(self.vect_to_goal, dim=1)).item() >= 0.04):
            print(f"Distance to cup: {torch.max(torch.linalg.norm(self.vect_to_goal, dim=1)).item()}")
            self.target_pos= self.obs["Target_obs"]
            print(f"self.target_pos: {self.target_pos}")
            self.calc_vect_to_goal(self.target_pos)

            hand_actions = self.obs["Last_action"]
            hand_actions = hand_actions[:, :6]

            pos = self.target_pos
            orientation = torch.tensor([[0.707, 0.0, 0.707, 0.0]], device=self.env.device)
            expanded_orient = orientation.expand(self.env.num_envs, -1)

            arm_pos = torch.cat((pos, expanded_orient), dim=1)
            actions = self.create_action(hand_actions, arm_pos)
            self.obs, _ = self.env.step(actions)
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)

    def step_home(self):
        self.home_pos = self.obs["Target_obs"]
        self.home_pos[:,2] += 0.3
        self.target_pos = self.home_pos

        self.calc_vect_to_goal(self.target_pos)

        while (torch.max(torch.linalg.norm(self.vect_to_goal, dim=1)).item() >= 0.05):
            
            self.calc_vect_to_goal(self.target_pos)
            print(f"Distance to home: {torch.max(torch.linalg.norm(self.vect_to_goal, dim=1)).item()}")

            hand_actions = self.obs["Last_action"][:, :6]
            pos = self.target_pos[:, :3]
            
            orientation = torch.tensor([[0.707, 0.0, 0.707, 0.0]], device=self.env.device)
            expanded_orient = orientation.expand(self.env.num_envs, -1)
            arm_pos = torch.cat((pos, expanded_orient), dim=1)

            actions = self.create_action(hand_actions, arm_pos)
            self.obs, _ = self.env.step(actions)
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
            print("homing...")
    
    def calc_vect_to_goal(self, target_pos):
        self.hand_pos = self.center_hand.data.target_pos_w[:, 0, :]
        print(f"self.hand_pos: {self.hand_pos}")

        self.vect_to_goal = torch.abs(target_pos - self.hand_pos)

    def rmpflow_cycle(self):


        target_pos = self.obs["Target_obs"]
        target_pos[:,0] += self.buf_x
        target_pos[:,1] += self.buf_y
        target_pos[:,2] += self.buf_z

        self.step_to(target_pos) ##

        print("Reached target. Preparing to grasp.")

        for _ in range(10):
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
        self.step_to_cup()
        print("Reached cup. Preparing to grasp.")
       
        target_pos = self.obs["Target_obs"]
        target_pos[:,0] += 0.015
        for _ in range(10):
            self.step_to(target_pos)

        for _ in range(10):
            self.grasp()
            print("grasped")

        target_pos[:,2] += 0.2
        self.step_to(target_pos)

        self.step_home()
        print("Reached home position.")

        for _ in range(10):
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
        print("Moving to drop-off position.")
        waypoint_1 = self.waypoint_1
        waypoint_1[:,2] = 0.09
        self.step_to(waypoint_1)


        for _ in range(10):
            self.step_to(self.waypoint_1)
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
        print("Reached drop-ofself.psy_node.publish_actions(self.ros_command)f position.")

        for _ in range(10):
            self.grip_release()
            print("released")
        
        for _ in range(10):
            self.step_to(self.waypoint_1)
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
        target_pos = self.obs["Target_obs"]##
        target_pos[:,0] += -0.1
        target_pos[:,1] += -0.1

        self.step_to(target_pos)

        self.step_home()

        for _ in range(10):
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)

        #end of cycle

        target_pos = self.obs["Target_obs"]
        target_pos[:,0] += self.buf_x - 0.15
        target_pos[:,1] += self.buf_y
        target_pos[:,2] += self.buf_z

        self.step_to(target_pos) ##

        print("Reached target. Preparing to grasp.")

        for _ in range(10):
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
        self.step_to_cup()
        print("Reached cup. Preparing to grasp.")
       
        target_pos = self.obs["Target_obs"]
        target_pos[:,0] += 0.015
        for _ in range(10):
            self.step_to(target_pos)

        for _ in range(10):
            self.grasp()
            print("grasped")

        target_pos[:,2] += 0.2
        self.step_to(target_pos)

        self.step_home()
        print("Reached home position.")

        for _ in range(10):
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
        print("Moving to drop-off position.")
        waypoint_1 = self.waypoint_2
        waypoint_1[:,2] = 0.09
        self.step_to(waypoint_1)


        for _ in range(10):
            self.step_to(self.waypoint_2)
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
        print("Reached drop-ofself.psy_node.publish_actions(self.ros_command)f position.")

        for _ in range(10):
            self.grip_release()
            print("released")
        
        for _ in range(10):
            self.step_to(self.waypoint_2)
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
        target_pos = self.obs["Target_obs"]##
        target_pos[:,0] += -0.1
        target_pos[:,1] += -0.1

        self.step_to(target_pos)

        self.step_home()

        for _ in range(10):
            self.obs, _ = self.env.step(self.obs["Last_action"])
            self.ros_command = self.obs["Robot_obs"]
            self.psy_node.publish_actions(self.ros_command)
    
    def main(self):

        for _ in range(100):

            self.init_hand()

            self.target_pos= self.obs["Target_obs"]##
            self.calc_vect_to_goal(self.target_pos)

            if simulation_app.is_running():
                self.target_pos= self.obs["Target_obs"]##
                self.rmpflow_cycle()
                self.env.reset()

        simulation_app.close()

if __name__ == "__main__":
    print("Starting PickupCup script...")
    pickup_cup = PickupCup()
    pickup_cup.main()
