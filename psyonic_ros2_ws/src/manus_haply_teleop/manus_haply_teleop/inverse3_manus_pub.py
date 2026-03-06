import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rclpy.executors import MultiThreadedExecutor
import threading
import time

import HaplyHardwareAPI
from scipy.spatial.transform import Rotation as R
import numpy as np

class Inverse3ManusPubisher(Node):
    def __init__(self):
        super().__init__("Inverse3ManusPub")
        qos_profile_best_effort = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE, # Volatile is typical for best effort according to internet
                    depth=1 
                )
        self.publisher = self.create_publisher(JointState, 'Inverse3Manus_Topic', qos_profile_best_effort)
        # temp disabled!!
        # self.sub_manus = self.create_subscription(ManusGlove, 'manus_glove_0', self.teleop_manus_callback, qos_profile_best_effort)
        # replaced with this for testing
        self.timer = self.create_timer(0.005, self.teleop_manus_callback)
        self.pub_msg = JointState()
        self.pub_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'index_q1', 'middle_q1', 'ring_q1', 'pinky_q1', 'thumb_q2', 'thumb_q1']

        self.manus_latest = list([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.haply_latest = list([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # form of delta 3d pos then 3 euler angles

        self.lock = threading.Lock()
        self.running = True

        self.haply_thread = threading.Thread(target=self.teleop_haply)
        self.haply_thread.daemon = True
        self.haply_thread.start()


    def teleop_manus_callback(self, msg=None): #has msg as input in real
        # gets manus reading
        msg
        # formats manus reading to store in latest var
        self.manus_latest

        # gets haply latest
        with self.lock:
            # self.pub_msg.position =  self.haply_latest + self.manus_latest
            self.pub_msg.position = self.haply_latest + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # publishes to Inverse_Manus_Topic
        self.publisher.publish(self.pub_msg)
        

    def teleop_haply(self):

        com_stream = HaplyHardwareAPI.SerialStream(
            "/dev/serial/by-id/usb-Teensyduino_Haply_inverse3_12651940-if00"
        ) 
        
        # will need to be tailored to each cpu!!
        inverse3 = HaplyHardwareAPI.Inverse3(com_stream)
        
        response_to_wakeup = inverse3.device_wakeup_dict()
        # print the response to the wakeup command
        print("connected to device {}".format(response_to_wakeup["device_id"]))
        
        # will need to be tailored to each cpu!!
        handle_stream = HaplyHardwareAPI.SerialStream(
            "/dev/serial/by-id/usb-ZEPHYR_Haply_USB_Transceiver_75F377C62ED31F59-if00"
        )
        versegrip = HaplyHardwareAPI.Handle(handle_stream)


        start_pos, _ = inverse3.end_effector_force()

        while True:
            start_orient = versegrip.GetVersegripStatus()
            quat = start_orient['quaternion']  # [w,x,y,z]

            quat_scipy = [quat[1], quat[2], quat[3], quat[0]]

            if np.linalg.norm(quat_scipy) > 1e-6:
                print("got good val")
                break

        start_rot = R.from_quat(quat_scipy) # see that you get valid quat start


        # while node is active, thread runs
        while self.running and ros2.ok():
            # gets delta_haply reading
            position, velocity = inverse3.end_effector_force()
            response = versegrip.GetVersegripStatus()
            try:
                # reorder Haply quaternion for scipy
                quat_scipy = [response['quaternion'][1],
                            response['quaternion'][2],
                            response['quaternion'][3],
                            response['quaternion'][0]]
                new_rot = R.from_quat(quat_scipy)

                # relative rotation
                rot_delta = new_rot * start_rot.inv()

                # axis-angle vector
                rotvec_delta = rot_delta.as_rotvec()
                # angle = np.linalg.norm(rotvec_delta)
                # if angle > 1e-6:
                #     axis = rotvec_delta / angle
                # else:
                #     axis = [0, 0, 0]
                # mind the coordinate frame and change here if needed (x,y,z, Rx, Ry, Rz)
                true_pos = [-2*(position[1] - start_pos[1]), -2*(position[0] - start_pos[0]), 2*(position[2] - start_pos[2]), float(rotvec_delta[0]), float(rotvec_delta[1]), float(rotvec_delta[2])]
                # print(f"position: {true_pos}")
                print(rot_delta.as_euler(seq="xyz", degrees=True))
            except Exception as e:
                pass

            time.sleep(0.005)


            # formats reading to store in latest var
            with self.lock:
                self.haply_latest = true_pos

        
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
        executor.spin()
    except KeyboardInterrupt:
        pass


    pub.destroy_node()
    ros2.shutdown()


if __name__ == "__main__":
    main()