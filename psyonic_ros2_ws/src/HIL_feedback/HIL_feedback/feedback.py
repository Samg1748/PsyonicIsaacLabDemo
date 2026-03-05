import rclpy as ros2
from rclpy.node import Node
import rtde_control
import rtde_receive
from ah_wrapper import AHSerialClient
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time


class Feedback_Psyonic(Node):
    
    def __init__(self):
        super().__init__('Feedback')

        # initializes UR_RTDE
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")
        self.current_ur_pose = self.rtde_r.getActualTCPPose()
        # initializes ABH client
        self.client = AHSerialClient()

        # qos profile to help with faster polling
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1 
        )

        # initializes publisher to topic Psyonic_Real2Sim_Topic
        self.publisher = self.create_publisher(JointState, 'PsyonicFeedback_Topic', qos_profile_best_effort)
        self.pub_timer = self.create_timer(0.025, self.pub_callback)
        self.subscriber = self.create_subscription(JointState, 'Psyonic_Topic', self.sub_callback, qos_profile_best_effort)

    #will these deadlock??
    def pub_callback(self):
        # initializes msg
        self.msg = JointState()
        self.msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q2', 'thumb_q1']
        # gets UR_RTDE arm values
        arm_command = self.rtde_r.getActualQ()
        # gets ability hand values
        hand_command = self.client.hand.get_position()

        self.msg.position = arm_command + hand_command
        self.publisher.publish(self.msg) #hand commands in norm values

    def sub_callback(self, msg):
        #format msg into arm_position and hand_position
        rel_i3_position = list(msg.position[:6])
        hand_position = list(msg.position[6:])

        # send move command to UR arm
        ##
        current_ur_joints = self.rtde_r.getActualQ()
        delta_ur = [self.current_ur_pose[0] + rel_i3_position[0], self.current_ur_pose[1] + rel_i3_position[1], self.current_ur_pose[2] + rel_i3_position[2], self.current_ur_pose[3], self.current_ur_pose[4], self.current_ur_pose[5]]
        try:
            ur_new_joints = self.rtde_c.getInverseKinematics(delta_ur, current_ur_joints)
        except Exception as e:
            ur_new_joints = current_ur_joints
        
        print(ur_new_joints)
        #temp disabled!!
        # maybe do delta_3d_pose for UR w/moveL or servoL??? or use IK
        self.rtde_c.servoJ(ur_new_joints, 1.05, 1.4, 0.11, 0.2, 300)
        
        # send position command to ability hand
        self.client.set_position(hand_position) #in norm values
        

    def destroy_node(self):
        super().destroy_node()
        self.client.close()
        ros2.shutdown()





        

def main():
    ros2.init()

    feedback = Feedback_Psyonic()

    ros2.spin(feedback)

    feedback.destroy_node()

        
        


if __name__ == "__main__":
    main()