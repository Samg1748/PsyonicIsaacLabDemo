import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rclpy.executors import MultiThreadedExecutor
import threading
import time

class Inverse3ManusPubisher(Node):
    def __init__(self):
        super().__init__("Inverse3ManusPub")
        qos_profile_best_effort = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
                    depth=1 
                )
        self.publisher = self.create_publisher(JointState, 'Inverse3Manus_Topic', qos_profile_best_effort)
        self.sub_manus = self.create_subscription(ManusGlove, 'manus_glove_0', self.teleop_manus_callback, qos_profile_best_effort)
        
        self.pub_msg = JointState()
        self.pub_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q2', 'thumb_q1']

        self.manus_latest = list(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.haply_latest = list(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) #CHECK THISSS

        self.lock = threading.lock()
        self.running = True

        self.haply_thread = threading.Thread(target=self.teleop_haply)
        self.haply_thread.daemon = True
        self.haply_thread.start()


    def teleop_manus_callback(self, msg):
        # gets manus reading
        msg
        # formats manus reading to store in latest var
        self.manus_latest

        # gets haply latest
        with self.lock:
            self.pub_msg.position = self.manus_latest + self.haply_latest

        # publishes to Inverse_Manus_Topic
        self.pub_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(self.pub_msg)
        pass

    def teleop_haply(self):

        # while node is active, thread runs
        while self.running and ros2.ok():
            pass
        # gets haply reading

        # formats reading to store in latest var
        self.haply_latest
        pass
        
    def destroy_node(self):
        self.running= False
        self.haply_thread.join()
        super().destroy_node()


def main():
    ros2.init()
    pub = Inverse3ManusPubisher()

    executor = MultiThreadedExecutor()
    executor.add_node(pub)

    try:
        ros2.spin(pub)
    except KeyboardInterrupt:
        pass


    pub.destroy_node()
    ros2.shutdown()


if __name__ == "__main__":
    main()