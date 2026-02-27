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
    """
    Handles ROS2 Node and Publishing of UR control
        has 2 threads: robot control thread and ros2 thread
    """
    def __init__(self):
        super().__init__('Psyonic_Real2Sim')

        # initializes UR_RTDE
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")
        
        # initializes ABH client
        self.client = AHSerialClient()

        # initializes lock for shared variables (to prevent race conditions)
        self.lock = threading.Lock()

        # start pose of hand and arm (can be changed)
        start_arm = [-91 * (3.14/180), -80 * (3.14/180), 107 * (3.14/180), -32 * (3.14/180), 90 * (3.14/180), -100 * (3.14/180)]
        hand_open = [0, 0, 0, 0, 10, -100]
        self.latest_robot_position = start_arm + hand_open
        
        # boolean to for joining threads if running stops
        self.running = True

        # qos profile to help with faster polling
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1 
        )

        # initializes publisher to topic Psyonic_Real2Sim_Topic
        self.publisher = self.create_publisher(JointState, 'Psyonic_Real2Sim_Topic', qos_profile_best_effort)
        self.pub_timer = self.create_timer(0.05, self.pub_callback)

        # Starts robot control thread (doing robot_control_loop)
        self.control_thread = threading.Thread(target=self.robot_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        

    def pub_callback(self):
        # initializes msg
        self.msg = JointState()
        self.msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q2', 'thumb_q1']
        # gets UR_RTDE arm values
        arm_command = self.rtde_r.getActualQ()
        # gets ability hand values
        hand_command = self.client.hand.get_position()
        obs = arm_command + hand_command
        
        # with lock on shared variables, publish message to topic (might not need this lock TODO)
        with self.lock:
            self.latest_robot_position = obs
        self.msg.position = self.latest_robot_position
        self.publisher.publish(self.msg)

    def send_robot_action(self, arm_position, hand_position):
        # with lock on shared variables save to latest_position variable to send command
        with self.lock:
            self.latest_robot_position = arm_position + hand_position
        
        # send move command to UR arm
        self.rtde_c.moveJ(arm_position, 2.0, 1.4) #1.05 1.4
        # send position command to ability hand
        self.client.set_position(hand_position)
        # slight wait to prevent resource loop with no way out
        time.sleep(0.05)


    def robot_control_loop(self):

        hand_open = [0, 0, 0, 0, 10, -100]
        hand_closed = [40, 40, 40, 40, 10, -100]
        
        cup_arm_pos = [-93.5 * (3.14/180), -60 * (3.14/180), 130 * (3.14/180), -70 * (3.14/180), 90 * (3.14/180), -91 * (3.14/180)]
        cup_up_arm_pos = [-93.5 * (3.14/180), -78 * (3.14/180), 125 * (3.14/180), -47 * (3.14/180), 90 * (3.14/180), -91 * (3.14/180)]
        cup_flipped_arm_pos = [-72 * (3.14/180), -71 * (3.14/180), 115 * (3.14/180), -43 * (3.14/180), 110 * (3.14/180), -270 * (3.14/180)]
        cup_flipped_down_pos = [-72 * (3.14/180), -56 * (3.14/180), 120 * (3.14/180), -62 * (3.14/180), 110 * (3.14/180), -270 * (3.14/180)]
        arm_flipped_back_pos = [-60 * (3.14/180), -56 * (3.14/180), 120 * (3.14/180), -63 * (3.14/180), 124 * (3.14/180), -270 * (3.14/180)]
        arm_reset_pos = [-106 * (3.14/180), -70 * (3.14/180), 147 * (3.14/180), -80 * (3.14/180), 75 * (3.14/180), -90 * (3.14/180)]
        # arm_demo_pos = [-77 * (3.14/180), -84 * (3.14/180), 117 * (3.14/180), -32 * (3.14/180), 105 * (3.14/180), -180 * (3.14/180)]
        


        while self.running and ros2.ok():

            # self.send_robot_action(arm_demo_pos, hand_open)

            self.send_robot_action(arm_reset_pos, hand_open)

            self.send_robot_action(cup_arm_pos, hand_open)

            self.send_robot_action(cup_arm_pos, hand_closed)

            self.send_robot_action(cup_up_arm_pos, hand_closed)

            self.send_robot_action(cup_flipped_arm_pos, hand_closed)

            self.send_robot_action(cup_flipped_down_pos, hand_closed)

            self.send_robot_action(cup_flipped_down_pos, hand_open)
            
            self.send_robot_action(arm_flipped_back_pos, hand_open)
 ########################### end of series, not reverse out
            self.send_robot_action(arm_flipped_back_pos, hand_open)

            self.send_robot_action(cup_flipped_down_pos, hand_open)

            self.send_robot_action(cup_flipped_down_pos, hand_closed)

            self.send_robot_action(cup_flipped_arm_pos, hand_closed)

            self.send_robot_action(cup_up_arm_pos, hand_closed)

            self.send_robot_action(cup_arm_pos, hand_open)

            self.send_robot_action(arm_reset_pos, hand_open)

            # self.send_robot_action(arm_demo_pos, hand_open)


    def destroy_node(self):

        # destroys node when stopped
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