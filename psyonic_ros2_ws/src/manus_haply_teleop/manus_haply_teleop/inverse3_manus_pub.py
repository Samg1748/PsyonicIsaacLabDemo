import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class Inverse3ManusPubisher(Node):
    def __init__(self):
        super().__init__("Inverse3ManusPub")
        qos_profile_best_effort = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
                    depth=1 
                )
        self.publisher = self.create_publisher(JointState, 'Inverse3Manus_Topic', qos_profile_best_effort)
        self.pub_timer = self.create_timer(0.05, self.teleop_pub_callback)
        self.count = 0.0

    def teleop_pub_callback(self):
        self.msg = JointState()
        self.msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q2', 'thumb_q1']
        # get values of Inverse3 and Manus gloves

        # formats to Inverse_Manus_Topic

        # publishes to Inverse_Manus_Topic
        self.msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.count, self.count/2, self.count, 0.0, 0.0, -self.count]
        self.count += 0.05
        self.publisher.publish(self.msg)


def main():
    ros2.init()
    pub = Inverse3ManusPubisher()

    ros2.spin(pub)

    pub.destroy_node()
    ros2.shutdown()


if __name__ == "__main__":
    main()