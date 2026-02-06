import rclpy as ros2
from rclpy.node import Node
# import rtde_control
from sensor_msgs.msg import JointState

class ArmListener(Node):
    def __init__(self):
        super().__init__('Arm_listener')
        # self.rtde_c = rtde_control.RTDEControlInterface("---")
        self.subscription = self.create_subscription(JointState, 'Psyonic_Topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        # self.rtde_c.moveJ(msg.position[:6], 1.05, 1.4)
        print(f"recieved: {msg.position[:6]}")

def main():
    ros2.init()
    arm_listener = ArmListener()

if __name__ == "__main__":
    main()