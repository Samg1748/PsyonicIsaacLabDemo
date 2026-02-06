import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ah_wrapper import AHSerialClient

class HandListener(Node):
    def __init__(self):
        super().__init__('Hand_listener')
        # self.client = AHSerialClient()
        self.subscription = self.create_subscription(JointState, 'Psyonic_Topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        try:
            # self.client.set_position(positions=[msg.position[6], msg.position[7], msg.position[8], msg.position[9], msg.position[10], msg.position[15]], reply_mode=2)
            self.get_logger().info(f' recieved: {msg.position[6]} {msg.position[7]} {msg.position[8]} {msg.position[9]} {msg.position[10]} {msg.position[15]}')
        except KeyboardInterrupt:
            print("Keyboard interrupt!!")
        finally:
            # self.client.close()
            pass #remove later



def main():
    ros2.init()
    hand_listener = HandListener()

    ros2.spin(hand_listener)
    hand_listener.destroy_node()
    ros2.shutdown()

if __name__ == "__main__":
    main()
