import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

class GTC_demo(Node):
    def __init__(self):
        super().__init__('GTC_demo')
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1 
        )
        self.sub_inverse3manus = self.create_subscription(JointState, 'Inverse3Manus_Topic', self.inverse3_manus_sub_callback, qos_profile_best_effort)
        self.sub_feedback = self.create_subscription(JointState, 'PsyonicFeedback_Topic', self.psyonic_feedback_sub_callback, qos_profile_best_effort)
        
        self.pub_psyonic = self.create_publisher(JointState, 'Psyonic_Topic', qos_profile_best_effort)
        self.pub_isaaclab = self.create_publisher(JointState, 'IsaacLab_Topic', qos_profile_best_effort)

    def publish_psyonic(self, msg):
        # publish to Psyonic_Topic
        self.pub_psyonic.publish(msg)
        

    def publish_isaaclab(self, msg):
        # publish msg to IsaacLab_Topic
        
        self.pub_isaaclab.publish(msg)

    def inverse3_manus_sub_callback(self, msg):
        # publish new msg to Psyonic_Topic
        #confirm manus hands publish in norm vals
        self.publish_psyonic(msg)
        

    def psyonic_feedback_sub_callback(self, msg):
        # format feedback psyonic position and ur position into joint angles for isaaclab
        hand_position = list(msg.position[6:])
        # confirm order of manus hand is [index, middle, ring, pinky, thumb, thumb_rotate]
        
        arm = list(msg.position[:6])
        hand = [math.radians(hand_position[i]) for i in range(6)]

        format_msg = arm + hand
        msg.position = format_msg
        # publish feedback to isaaclab through isaaclab_topic
        self.publish_isaaclab(msg)
        




def main():
    ros2.init()
    ros_main_ = GTC_demo()

    ros2.spin(ros_main_)
    
    ros_main_.destroy_node()
    ros2.shutdown()


if __name__ == "__main__":
    main()