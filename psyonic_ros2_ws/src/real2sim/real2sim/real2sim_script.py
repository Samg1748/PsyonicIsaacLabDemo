import rclpy as ros2
from rclpy.node import Node
import rtde_control
import rtde_receive
from ah_wrapper import AHSerialClient
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
import threading
import time

class Real2SimNode(Node):
    def __init__(self):
        super().__init__('Psyonic_Real2Sim')
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")
        self.client = AHSerialClient()

        self.lock = threading.Lock()
        start_arm = [-91 * (3.14/180), -80 * (3.14/180), 107 * (3.14/180), -32 * (3.14/180), 90 * (3.14/180), -100 * (3.14/180)]
        hand_open = [0, 0, 0, 0, 10, -100]
        self.latest_robot_position = start_arm + hand_open
        self.running = True
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1 
        )
        self.publisher = self.create_publisher(JointState, 'Psyonic_Real2Sim_Topic', qos_profile_best_effort)
        self.pub_timer = self.create_timer(0.05, self.pub_callback)

        self.control_thread = threading.Thread(target=self.robot_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        

    def pub_callback(self):
        self.msg = JointState()
        self.msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q2', 'thumb_q1']
        arm_command = self.rtde_r.getActualQ()
        hand_command = self.client.hand.get_position()
        obs = arm_command + hand_command
        with self.lock:
            self.latest_robot_position = obs
        self.msg.position = self.latest_robot_position
        self.publisher.publish(self.msg)

    def send_robot_action(self, arm_position, hand_position):
        with self.lock:
            self.latest_robot_position = arm_position + hand_position
        
        self.rtde_c.moveJ(arm_position, 0.5, 1.0) #1.05 1.4
        self.client.set_position(hand_position)
        time.sleep(0.05)


    def robot_control_loop(self):

        start_arm = [-91 * (3.14/180), -80 * (3.14/180), 107 * (3.14/180), -32 * (3.14/180), 90 * (3.14/180), -100 * (3.14/180)]
        cup_arm_pos = [-85 * (3.14/180), -61 * (3.14/180), 130 * (3.14/180), -69 * (3.14/180), 90 * (3.14/180), -90 * (3.14/180)]
        hand_open = [0, 0, 0, 0, 10, -100]
        cup_flipped_arm_pos = [-70 * (3.14/180), -60 * (3.14/180), 105 * (3.14/180), -45 * (3.14/180), 107 * (3.14/180), -270 * (3.14/180)]
        hand_closed = [40, 40, 40, 40, 10, -100]
        cup_flipped_set_arm_pos = [-65 * (3.14/180), -51 * (3.14/180), 106 * (3.14/180), -50 * (3.14/180), 107 * (3.14/180), -270 * (3.14/180)]
        cup_flipped_set_far_pos = [-59 * (3.14/180), -57 * (3.14/180), 100 * (3.14/180), -45 * (3.14/180), 120 * (3.14/180), -270 * (3.14/180)]
        arm_hand_demo = [-60 * (3.14/180), -50 * (3.14/180), 100 * (3.14/180), -140 * (3.14/180), 90 * (3.14/180), -200 * (3.14/180)]


        while self.running and ros2.ok():

            self.send_robot_action(start_arm, hand_open)

            self.send_robot_action(cup_arm_pos, hand_open)

            self.send_robot_action(cup_arm_pos, hand_closed)

            self.send_robot_action(cup_flipped_arm_pos, hand_closed)

            self.send_robot_action(cup_flipped_set_arm_pos, hand_closed)

            self.send_robot_action(cup_flipped_set_arm_pos, hand_open)

            self.send_robot_action(cup_flipped_set_far_pos, hand_open)

            self.send_robot_action(arm_hand_demo, hand_open)


    def destroy_node(self):
        self.running = False
        self.control_thread.join()
        super().destroy_node()



        

def main():
    ros2.init()
    psy_node = Real2SimNode()

    executor = MultiThreadedExecutor()
    executor.add_node(psy_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    psy_node.destroy_node()
    ros2.shutdown()

        
        


if __name__ == "__main__":
    main()