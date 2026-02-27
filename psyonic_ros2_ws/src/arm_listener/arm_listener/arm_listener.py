import rclpy as ros2
from rclpy.node import Node
import rtde_control
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ArmListener(Node):
    def __init__(self):
        super().__init__('Arm_listener')
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.1.10")
        
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
            depth=1
        )
        self.subscription = self.create_subscription(JointState, 'Psyonic_Topic', self.listener_callback, qos_profile_best_effort) #10
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        # msg.position[0] += -1.5708 ##EXTREMELY SPECIFIC TO MassRobotics USE CASE
        # self.rtde_c.moveJ(msg.position[:6], 1.05, 1.4)
        self.rtde_c.servoJ(msg.position[:6], 1.05, 1.4, 0.11, 0.2, 300)
        self.get_logger().info(f'recieved: {msg.position[:6]}')

def main():
    ros2.init()
    arm_listener = ArmListener()

    ros2.spin(arm_listener)
    arm_listener.destroy_node()
    ros2.shutdown()

if __name__ == "__main__":
    main()
