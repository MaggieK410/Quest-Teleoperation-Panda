import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class TrajectoryReplay(Node):

    def __init__(self, path):
        super().__init__("trajectory_replay")

        self.pub = self.create_publisher(JointState, "/joint_states", 10)

        with h5py.File(path, "r") as f:
            self.states = f["states"][:]
        print("states: ", len(self.states))
        self.index = 0
        self.timer = self.create_timer(0.02, self.step)

        self.joint_names = [
            "fer_joint1",
            "fer_joint2",
            "fer_joint3",
            "fer_joint4",
            "fer_joint5",
            "fer_joint6",
            "fer_joint7",
            "fer_finger_joint1",
            "fer_finger_joint2",
        ]

    def step(self):

        if self.index >= len(self.states):
            self.get_logger().info("Replay finished")
            rclpy.shutdown()
            return

        state = self.states[self.index]

        arm = state[25:32]
        gripper = state[21:23]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = np.concatenate([arm, gripper]).tolist()

        self.pub.publish(msg)

        self.index += 1


def main():
    rclpy.init()
    node = TrajectoryReplay("/home/gepetto/demos/2026-03-09_13-12-25/demo.hdf5")
    rclpy.spin(node)