import rclpy as ros2
from rclpy.node import Node
import rtde_control
from ah_wrapper import AHSerialClient
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time

class Real2SimNode(Node):
    def __init__(self):
        super().__init__('Psyonic_Real2Sim')
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
        self.client = AHSerialClient()

        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1 
        )
        self.publisher = self.create_publisher(JointState, 'Psyonic_Real2Sim_Topic', qos_profile_best_effort)
        

    def publish_actions(self, arm_command, hand_command):
        self.msg = JointState()
        self.msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q1', 'thumb_q2']
        actions = arm_command + hand_command
        self.msg.position = actions
        self.publisher.publish(self.msg)
        self.rtde_c.servoJ(arm_command, 1.05, 1.4, 0.11, 0.2, 300)
        self.client.set_position(positions=[hand_command[0]* (180/3.14*1.1), hand_command[1]* (180/3.14*1.1), hand_command[2]* (180/3.14*1.1), hand_command[3]* (180/3.14*1.1), hand_command[4]* (180/3.14*1.1), hand_command[5]* (180/3.14*1.1)], reply_mode=2)
    

def main():
    ros2.init()
    psy_node = Real2SimNode()

    start_arm = [-91 * (3.14/180), -80 * (3.14/180), 107 * (3.14/180), -32 * (3.14/180), 90 * (3.14/180), -100 * (3.14/180)]
    cup_arm_pos = [-85 * (3.14/180), -61 * (3.14/180), 130 * (3.14/180), -69 * (3.14/180), 90 * (3.14/180), -90 * (3.14/180)]
    hand_open = [0, 0, 0, 0, -1.7, 0.15]
    cup_flipped_arm_pos = [-70 * (3.14/180), -60 * (3.14/180), 105 * (3.14/180), -45 * (3.14/180), 107 * (3.14/180), -270 * (3.14/180)]
    hand_closed = [0.65, 0.65, 0.65, 0.65, -1.7, 0.15]
    cup_flipped_set_arm_pos = [-65 * (3.14/180), -51 * (3.14/180), 106 * (3.14/180), -50 * (3.14/180), 107 * (3.14/180), -270 * (3.14/180)]
    cup_flipped_set_far_pos = [-59 * (3.14/180), -57 * (3.14/180), 100 * (3.14/180), -45 * (3.14/180), 120 * (3.14/180), -270 * (3.14/180)]
    arm_hand_demo = [-60 * (3.14/180), -50 * (3.14/180), 100 * (3.14/180), -140 * (3.14/180), 90 * (3.14/180), -200 * (3.14/180)]


    while True:

        for _ in range(1000000):
            psy_node.publish_actions(start_arm, hand_open)
        for _ in range(1000000):
            psy_node.publish_actions(cup_arm_pos, hand_open)
        for _ in range(1000000):
            psy_node.publish_actions(cup_arm_pos, hand_closed)
        for _ in range(1000000):
            psy_node.publish_actions(cup_flipped_arm_pos, hand_closed)
        for _ in range(1000000):
            psy_node.publish_actions(cup_flipped_set_arm_pos, hand_closed)
        for _ in range(1000000):
            psy_node.publish_actions(cup_flipped_set_arm_pos, hand_open)
        for _ in range(1000000):
            psy_node.publish_actions(cup_flipped_set_far_pos, hand_open)
        for _ in range(1000000):
            psy_node.publish_actions(arm_hand_demo, hand_open)
        


if __name__ == "__main__":
    main()