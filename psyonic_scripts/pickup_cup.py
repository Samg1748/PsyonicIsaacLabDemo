from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# import argparse
from pprint import pprint

import os
import time
import numpy as np
import omni.kit.commands
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.prims import SingleRigidPrim, SingleXFormPrim
from isaacsim.core.api.objects import cuboid
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation.articulation_motion_policy import ArticulationMotionPolicy
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    get_supported_robot_policy_pairs,
    load_supported_motion_policy_config,
)
from isaacsim.robot_motion.motion_generation.lula import RmpFlow
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path
from omni.kit.viewport.utility import get_active_viewport
from pxr import Usd, UsdGeom


class PickupCup:
    def __init__(self):
        robot_name = "UR5e"
        usd_path = "psyonic_UR_right_playground.usd"
        target_usd_path = "assets/props/Beaker/beaker_500ml.usd"
        prim_path = "/ur5e"

        add_reference_to_stage(usd_path=usd_path, prim_path='/World/ur5e')
        add_reference_to_stage(usd_path=target_usd_path, prim_path='/World/target')

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        self.my_robot = Robot(prim_path='/World/ur5e', name=robot_name)
        robot = self.world.scene.add(self.my_robot)
        self.target = self.world.scene.add(SingleXFormPrim(prim_path="/World/target/beaker", name="target", scale=np.array([0.5, 0.5, 1.0]), position=np.array([0.4, 0.3, 0.25])))
        self.center_hand= self.world.scene.add(SingleXFormPrim(prim_path="/World/ur5e/ur5e/wrist_3_link/center_hand", name="center_hand", translation=np.array([0.0, 0.04, 0.11])))


        rmp_config = load_supported_motion_policy_config(robot_name, "RMPflow")

        rmp_config['rmpflow_config_path'] = os.path.join(os.getcwd(), 'assets/psyonic_right_UR5e/rmpflow', 'psyonic_ur5e_rmpflow_config.yaml')
        rmp_config['robot_description_path'] = os.path.join(os.getcwd(), 'assets/psyonic_right_UR5e/rmpflow', 'psyonic_ur5e_rmpflow_description.yaml')
        rmp_config['urdf_path'] = os.path.join(os.getcwd(), 'assets/psyonic_right_UR5e', 'psyonic_right_UR5e.urdf')
        rmp_config['end_effector_frame_name'] = 'center_hand'

        print(
            f"Successfully referenced RMPflow config for {robot_name}.  Using the following parameters to initialize RmpFlow class:"
        )
        pprint(rmp_config)
        print()

        self.home_pos = np.array([0.4, -0.15, 0.4], dtype=np.float32)
        self.waypoint_1 = np.array([0.5, 0.0, 0.08], dtype=np.float32)
        self.waypoint_2 = np.array([0.5, 0.3, 0.08], dtype=np.float32)
        self.home = False

        # Initialize an RmpFlow object
        self.rmpflow = RmpFlow(**rmp_config)

        physics_dt = 1 / 60.0
        self.articulation_rmpflow = ArticulationMotionPolicy(robot, self.rmpflow, physics_dt)

        self.articulation_controller = robot.get_articulation_controller()


        # Frame the existing viewport camera on the robot (like pressing F)
        stage = omni.usd.get_context().get_stage()
        active_viewport = get_active_viewport()
        if active_viewport and active_viewport.camera_path:
            usd_camera = UsdGeom.Camera.Get(stage, active_viewport.camera_path)
            if usd_camera:
                usd_camera.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)
            resolution = active_viewport.resolution
            # aspect_ratio = resolution[0] / resolution[1]


        self.world.reset()
        self.grasping = False
        self.buf_x = 0.0
        self.buf_y = -0.2
        self.buf_z = 0.015

    def init_hand(self):
        # self.world.step(render=True)
        initial_hand_joints = np.array([0.5, 0.5, 0.5, 0.5, -2.0, 0.0])
        initial_hand_action = ArticulationAction(joint_positions=initial_hand_joints, joint_indices=np.array([6,7,8,9,10,15]))
        self.my_robot.apply_action(initial_hand_action)
        for _ in range(20):
            self.world.step()

    def grasp(self):
        self.grasping = True

        self.target_pos = self.hand_pos.copy()
        grasp_hand_joints = np.array([0.52, 0.52, 0.52, 0.52, -2.0, 0.2])
        grasp_hand_action = ArticulationAction(joint_positions=grasp_hand_joints, joint_indices=np.array([6,7,8,9,10,15]))
        self.my_robot.apply_action(grasp_hand_action)
        

    def grip_release(self):
        self.grasping = False

        self.target_pos = self.hand_pos.copy()
        grasp_hand_joints = np.array([0.1, 0.1, 0.1, 0.1, -2.0, 0.0])
        grasp_hand_action = ArticulationAction(joint_positions=grasp_hand_joints, joint_indices=np.array([6,7,8,9,10,15]))
        self.my_robot.apply_action(grasp_hand_action)
        

    def step_to(self, position):
        self.waypoint = position
        self.target_pos = self.waypoint

        self.calc_vect_to_goal(self.target_pos)

        while (np.linalg.norm(self.vect_to_goal) >= 0.05):
            
            self.calc_vect_to_goal(self.target_pos)
            print(self.vect_to_goal)

            self.rmpflow.set_end_effector_target(
            target_position=self.target_pos, target_orientation=np.array([0.707, -0.707, 0, 0])
            )

            # Query the current obstacle position
            self.rmpflow.update_world()

            actions = self.articulation_rmpflow.get_next_articulation_action()
            self.articulation_controller.apply_action(actions)

            self.world.step()

    def step_to_cup(self):

        self.target_pos, self.target_rot = self.target.get_world_pose()
        self.calc_vect_to_goal(self.target_pos)

        while (np.linalg.norm(self.vect_to_goal) >= 0.0095):
            
            self.target_pos, self.target_rot = self.target.get_world_pose()
            self.calc_vect_to_goal(self.target_pos)

            self.rmpflow.set_end_effector_target(
            target_position=self.target_pos, target_orientation=np.array([0.707, -0.707, 0, 0])
            )

            # Query the current obstacle position
            self.rmpflow.update_world()

            actions = self.articulation_rmpflow.get_next_articulation_action()
            self.articulation_controller.apply_action(actions)

            self.world.step()

    def step_home(self):
        self.target_pos = self.home_pos

        self.calc_vect_to_goal(self.target_pos)

        while (np.linalg.norm(self.vect_to_goal) >= 0.005):
            
            self.calc_vect_to_goal(self.target_pos)

            self.rmpflow.set_end_effector_target(
            target_position=self.target_pos, target_orientation=np.array([0.707, -0.707, 0, 0])
            )

            # Query the current obstacle position
            self.rmpflow.update_world()

            actions = self.articulation_rmpflow.get_next_articulation_action()
            self.articulation_controller.apply_action(actions)

            self.world.step()
    
    def calc_vect_to_goal(self, target_pos):
        self.hand_pos, self.hand_rot = self.center_hand.get_world_pose()

        self.hand_pos = np.array(self.hand_pos, dtype=np.float32)
        self.target_pos = np.array(target_pos, dtype=np.float32)

        self.vect_to_goal = np.abs(self.target_pos - self.hand_pos)



    def rmpflow_cycle(self):


        target_pos, target_rot = self.target.get_world_pose()
        target_pos[1] += self.buf_y
        target_pos[2] += self.buf_z
        target_pos = np.array(target_pos, dtype=np.float32)
        self.step_to(target_pos)
        print("Reached target. Preparing to grasp.")
        for _ in range(10):
            self.world.step()

        self.step_to_cup()
        print("Reached cup. Preparing to grasp.")

        # for _ in range(1000):
        #     self.world.step()

        self.grasp()
        print("grasped")
        for _ in range(10):
            self.world.step()

        self.step_home()
        print("Reached home position.")

        for _ in range(10):
            self.world.step()

        print("Moving to drop-off position.")
        waypoint_1 = self.waypoint_1.copy()
        waypoint_1[2] = 0.15
        self.step_to(waypoint_1)

        # for _ in range(10):
        #     self.world.step()

        for _ in range(10):
            self.step_to(self.waypoint_1)
            self.world.step()
        print("Reached drop-off position.")

        self.grip_release()
        for _ in range(10):
            self.step_to(self.waypoint_1)
            self.world.step()

        target_pos, target_rot = self.target.get_world_pose()
        target_pos[0] -= 0.15
        target_pos[1] -= 0.15
        target_pos = np.array(target_pos, dtype=np.float32)
        self.step_to(target_pos)
        self.step_home()


        #end of cycle

        target_pos, target_rot = self.target.get_world_pose()
        target_pos[0] -= 0.1
        target_pos[1] += self.buf_y
        target_pos[2] += self.buf_z
        target_pos = np.array(target_pos, dtype=np.float32)
        self.step_to(target_pos)
        print("Reached target. Preparing to grasp.")
        for _ in range(10):
            self.world.step()

        self.step_to_cup()
        print("Reached cup. Preparing to grasp.")

        # for _ in range(1000):
        #     self.world.step()

        self.grasp()
        print("grasped")
        for _ in range(10):
            self.world.step()

        self.step_home()
        print("Reached home position.")

        for _ in range(10):
            self.world.step()

        print("Moving to drop-off position.")
        waypoint_2 = self.waypoint_2.copy()
        waypoint_2[2] = 0.15
        self.step_to(waypoint_2)

        for _ in range(10):
            self.world.step()

        for _ in range(10):
            self.step_to(self.waypoint_2)
            self.world.step()
        print("Reached drop-off position.")

        self.grip_release()
        for _ in range(10):
            self.step_to(self.waypoint_2)
            self.world.step()

        target_pos, target_rot = self.target.get_world_pose()
        target_pos[0] -= 0.1
        target_pos[1] -= 0.15
        target_pos = np.array(target_pos, dtype=np.float32)
        self.step_to(target_pos)
        self.step_home()

    
    def main(self):

        for _ in range(100):

            self.init_hand()

            self.target_pos, self.target_rot = self.target.get_world_pose()
            self.calc_vect_to_goal(self.target_pos)

            if simulation_app.is_running():
                self.target_pos, self.target_rot = self.target.get_world_pose()
                self.rmpflow_cycle()

        simulation_app.close()

if __name__ == "__main__":
    print("Starting PickupCup script...")
    pickup_cup = PickupCup()
    pickup_cup.main()