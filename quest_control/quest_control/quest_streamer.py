import numpy as np
import argparse
import struct
import socket
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
import pinocchio as pin

#rclpy imports
import rclpy
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


#Imports needed to send things to the MPC controller
from agimus_msgs.msg import MpcInput
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from linear_feedback_controller_msgs.msg import Sensor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import copy
import h5py

import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

#Imports needed to get robot params
from agimus_controller.factory.robot_model import RobotModelParameters, RobotModels
from agimus_controller_ros.simple_trajectory_publisher import TrajectoryPublisherBase
from agimus_controller_ros.trajectory_weights_parameters import (
    trajectory_weights_params,
)
from agimus_controller_ros.ros_utils import (
    weighted_traj_point_to_mpc_msg,
    get_param_from_node,
)
from agimus_controller.trajectory import (
    TrajectoryPoint,
    TrajectoryPointWeights,
    WeightedTrajectoryPoint,
)

from agimus_demo_08_collision_avoidance.goal_publisher_parameters import (
    goal_publisher,
)

from quest_control.franka_gripper_client import FrankaGripperClient
import threading
#Visualization imports
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA, Header, Int32
from rclpy.duration import Duration


import datetime
import cv2
import threading
from quest_control.multi_camera_recorder import MultiCameraRecorder


@dataclass
class ControllerPose:
    matrix: np.ndarray  # shape (4,4)

@dataclass
class ControllerInput:
    joystick: tuple[float, float]
    index_trigger: float
    hand_trigger: float
    buttons: dict[str, bool]

@dataclass
class VRFrame:
    head_pose: ControllerPose
    left_pose: ControllerPose
    right_pose: ControllerPose
    left_input: ControllerInput
    right_input: ControllerInput


class QuestTrajectoryPublisher(TrajectoryPublisherBase):
    def __init__(self):
        super().__init__("quest_trajectory_publisher")
        #self.param_listener = goal_publisher.ParamListener(self)
        #self.params = self.param_listener.get_params()


        self.ee_frame_name = 'fer_hand_tcp'
        self.future_init_done = Future()
        self.future_trajectory_done = Future()
        self.get_logger().info("Simple trajectory publisher node started.")
        self._marker_base: Marker | None = None
        self._target_pose_marker_pub = self.create_publisher(Marker, "target_pose_marker", 10)
        self.buffer=[]

        self.rotation=False #True

        self.franka_gripper_client = FrankaGripperClient(self, arm_id="fer") #might have to be chnaged to fr3?
        self.current_gripper_status = None

        self.currently_recording = False
        self.prev_button_state = False
        self.all_observations= {"timesteps": [], "states": []}
        self.start_of_episode = datetime.datetime.now()
        self.demo_filename = ""
        #This is used to know where and how to save the data
        self.base_dir = os.path.expanduser("~/demos/red_cube_small_without_numbers_with_timesteps")
        
        
        self.recorder = MultiCameraRecorder(self.base_dir)
        #self.recorder.initialize_cameras()
        self.start_of_episode = None

        os.makedirs(self.base_dir, exist_ok=True)
        self.timestamp = None

        self.ee_pose = None
        self.last_yaw = None
        self.ee_yaw = None
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.data_group = ReentrantCallbackGroup()

        self.joint_states = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.create_subscription(Sensor, "/linear_feedback_controller/sensor", self.sensor_cb, 10)
        #self.mpc_pub = self.create_publisher(MpcInput, "/linear_feedback_controller/mpc_input", 10)
        #self.publisher_.publish(mpc_msg)

        #I dont need this if Im not doing IK myself
        #pkg_dir = get_package_share_directory("franka_description")  # change to your package
        #urdf_path = os.path.join(pkg_dir, "robots", "common", "franka_robot.xacro") 
        #self.pin_model = pin.buildModelFromUrdf(urdf_path)
        #self.pin_data = pin.Data(self.pin_model)
        #self.q = None
        #self.dq = None

        #Set up quest stream
        UDP_IP = "0.0.0.0"
        UDP_PORT = 5000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        #self.sock.setblocking(0)
        self.R_head_flat=None

        
        self.lock = threading.Lock()
        self.latest_data=None
        self.recv_thread = threading.Thread(
            target=self.recv_loop,
            daemon=True
        )
        self.recv_thread.start()
        

        #self.last_pos=None
        #self.timer = None#self.create_timer(0.01, self.stream_quest_to_robot)
        self._marker_base = Marker(header=Header(frame_id="fer_link0"),
                ns="goal_publisher",
                type=Marker.SPHERE,
                action=Marker.ADD,
                scale=Vector3(x=0.05, y=0.05, z=0.05),
                color=ColorRGBA(
                    **dict(zip("rgba", [1.0, 0.0, 0.0, 0.5]))
                ),
                lifetime=Duration(seconds=0.01 * 10.0).to_msg(),
            )
        self.get_logger().info("Quest trajectory publisher started, waiting for Quest data...")
    def publish_controller_tf(self, T, parent_frame="world", child_frame="quest_right_controller"):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # translation
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])

        # rotation matrix → quaternion
        quat = R.from_matrix(T[:3, :3]).as_quat()  # xyzw

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(t)
        
    def recv_loop(self):

        while rclpy.ok():

            data, _ = self.sock.recvfrom(4096)

            width = self.decode_packet(data)

            with self.lock:
                self.latest_data = width
    
    def joint_state_callback(self, msg: JointState):
        self.joint_states = msg
    def ready_callback(self):
        self.get_logger().info(
            f"QuestTrajectoryPublisher ready. q0 = {[round(v, 3) for v in self.q0]}"
        )

        # Start streaming only once everything is ready
        self.timer = self.create_timer(0.01, self.stream_quest_to_robot, callback_group = self.control_group)
        self.gripper_timer = self.create_timer(0.01, self.gripper_control_robot, callback_group = self.control_group)
        #self.data_collection_timer = self.create_timer(0.05, self.collect_data, callback_group = self.data_group)
        #self.record_timer = self.create_timer(0.05, self.record_step)


    #def sensor_cb(self, msg):
    #    self.q = np.array(msg.robot_configuration)
    #    self.get_logger().info("Robot configuration: "+str(self.q))
    #    self.dq = np.array(msg.robot_velocity)
    #    if not self.future_init_done.done():
    #        self.future_init_done.set_result(True)


    def collect_data(self):
        #self.recorder.initialize_cameras()
        if self.latest_data != None:

            save_button = self.latest_data.right_input.buttons['A']

            if save_button and not self.prev_button_state:

                if not self.currently_recording:
                    self.get_logger().info("Starting recording")
                    self.timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                    self.recorder.start_recording(self.timestamp)
                    self.currently_recording = True
                    self.demo_filename = f""
                    #self.capture_current_state_as_parquet(start_of_episode)
                    self.start_of_episode = datetime.datetime.now()

                else:
                    self.get_logger().info("Stopping recording")
                    self.recorder.stop_recording()
                    self.currently_recording = False
                    self.flush_observation_hdf()

            self.prev_button_state = save_button

            self.recorder.capture_step()
    
    def record_step(self):
        if not self.currently_recording:
            return

        if self.start_of_episode is None:
            return

        self.capture_current_state_as_parquet(self.start_of_episode)

    def flush_observation_hdf(self):
        filepath = f"{self.base_dir}/{self.timestamp}/demo.hdf5"
        os.makedirs(f"{self.base_dir}/{self.timestamp}", exist_ok=True)

        with h5py.File(filepath, "w") as f:
            for key, value in self.all_observations.items():
                f.create_dataset(key, data=value)
        self.get_logger().info(f"Dumped demo states at {filepath}")
        
        #flush the dictionary
        self.all_observations= {"timesteps": [], "states": []}
        return
              
    def capture_current_state_as_parquet(self, timestep):
        if (self.ee_pose is None) or (self.current_q is None):
            #make sure we have a self ee pose
            return None
        
        current_timestep = (datetime.datetime.now() - timestep).total_seconds()
        
        data = pin.Data(self.robot_models.robot_model)
        pin.forwardKinematics(self.robot_models.robot_model, data, self.current_q)
        pin.updateFramePlacements(self.robot_models.robot_model, data)

        base_id = self.robot_models.robot_model.getFrameId("fer_link0")
        ee_id = self.robot_models.robot_model.getFrameId(self.ee_frame_name)

        # The absolute and relative positions are equal because we have the fact that the base is the parent most frame if we are with the real robot. Potentially this is different in simulation
        base_to_world = data.oMf[base_id]
        ee_to_world = data.oMf[ee_id]
        ee_to_base = data.oMf[base_id].inverse() * ee_to_world

        #Positions
        base_position_absolute = base_to_world.translation
        
        ee_position_absolute = ee_to_world.translation
        ee_posisiton_relative = ee_to_base.translation #data.oMf[ee_id].translation

        #Rotations
        ee_rotation_absolute = pin.Quaternion(ee_to_world.rotation).coeffs() #xyzw, verify this for other libraries
        ee_rotation_relative = pin.Quaternion(ee_to_base.rotation).coeffs()

        base_rotation_absolute = pin.Quaternion(base_to_world.rotation).coeffs()

        #Gripper qpos, qvel
        gripper_indices = [
            self.joint_states.name.index('fer_finger_joint1'),
            self.joint_states.name.index('fer_finger_joint2')
        ]
        gripper_qpos = np.array([self.joint_states.position[i] for i in gripper_indices])
        gripper_qvel = np.array([self.joint_states.velocity[i] for i in gripper_indices])

        #Joint positions, velocities and cos and sin
        joint_indices = [
            self.joint_states.name.index('fer_joint1'),
            self.joint_states.name.index('fer_joint2'),
            self.joint_states.name.index('fer_joint3'),
            self.joint_states.name.index('fer_joint4'),
            self.joint_states.name.index('fer_joint5'),
            self.joint_states.name.index('fer_joint6'),
            self.joint_states.name.index('fer_joint7')
        ]
        joint_positions = np.array([self.joint_states.position[i] for i in joint_indices])
        joint_velocities =  np.array([self.joint_states.velocity[i] for i in joint_indices])
        joint_pos_cos = np.cos(joint_positions)
        joint_pos_sin = np.sin(joint_positions)
        
        #Package observations, these are later used to make actions from them 
        '''
        self.all_observations["timesteps"].append(current_timestep)
        self.get_logger().info(f"Base position absolute: {base_position_absolute}")
        self.get_logger().info(f"base_rotation_absolute: {base_rotation_absolute}")
        self.get_logger().info(f"ee_position_absolute: {ee_position_absolute}")
        self.get_logger().info(f"ee_posisiton_relative: {ee_posisiton_relative}")
        self.get_logger().info(f"ee_rotation_absolute: {ee_rotation_absolute}")
        self.get_logger().info(f"ee_rotation_relative: {ee_rotation_relative}")
        self.get_logger().info(f"gripper_qpos: {gripper_qpos}")
        self.get_logger().info(f"gripper_qvel: {gripper_qvel}")
        self.get_logger().info(f"joint_positions: {joint_positions}")
        self.get_logger().info(f"joint_velocities: {joint_velocities}")
        self.get_logger().info(f"joint_pos_cos: {joint_pos_cos}")
        self.get_logger().info(f"joint_pos_sin: {joint_pos_sin}")
        '''
        all_obs = np.concatenate((base_position_absolute,
                                base_rotation_absolute,
                                ee_position_absolute,
                                ee_posisiton_relative,
                                ee_rotation_absolute,
                                ee_rotation_relative,
                                gripper_qpos,
                                gripper_qvel,
                                joint_positions,
                                joint_pos_cos,
                                joint_pos_sin,
                                joint_velocities), axis = None)
        
        self.all_observations["states"].append(all_obs)
        self.all_observations["timesteps"].append(current_timestep)


    def gripper_control_robot(self):
        if self.latest_data != None: #try:
            #data, addr = self.sock.recvfrom(4096)
            #frame = self.decode_packet(data)
            #self.get_logger().info(f"Grip Button? {self.latest_data.right_input.buttons['GripButton']}")

            if self.latest_data.right_input.buttons['GripButton'] != self.current_gripper_status :
                self.current_gripper_status  = self.latest_data.right_input.buttons['GripButton'] #update current command when change is noticed
                self.send_new_gripper_goal()
        #except BlockingIOError:
        else:
            return
    def send_new_gripper_goal(self):
        if self.current_gripper_status == True:
            #self.get_logger().info("Senfing close command")
            
            #Use this in real life
            self.franka_gripper_client.grasp()
    
            #Use this in sim
            #self.franka_gripper_client.send_goal(position=0.0001, max_effort=40.0)
        else:
            #self.get_logger().info("Senfing open command")
            self.franka_gripper_client.send_goal(position=0.039, max_effort=10.0)
    def decode_packet(self, data: bytes) -> VRFrame:
        floats = struct.unpack('<66f', data)
        f = iter(floats)

        def next_mat():
            return np.array([ [next(f), next(f), next(f), next(f)],
                            [next(f), next(f), next(f), next(f)],
                            [next(f), next(f), next(f), next(f)],
                            [next(f), next(f), next(f), next(f)] ], dtype=np.float32)

        head_pose  = ControllerPose(next_mat())
        left_pose  = ControllerPose(next_mat())
        right_pose = ControllerPose(next_mat())

        # Analog values
        left_joy   = (next(f), next(f))
        right_joy  = (next(f), next(f))
        left_index, right_index, left_grip, right_grip = next(f), next(f), next(f), next(f)

        # Buttons
        btnA, btnB, btnX, btnY, thumbL, thumbR, trigL, trigR, gripL, gripR = [int(next(f)) for _ in range(10)]

        left_buttons = {
            "X": bool(btnX),
            "Y": bool(btnY),
            "Thumbstick": bool(thumbL),
            "TriggerButton": bool(trigL),
            "GripButton": bool(gripL),
        }
        right_buttons = {
            "A": bool(btnA),
            "B": bool(btnB),
            "Thumbstick": bool(thumbR),
            "TriggerButton": bool(trigR),
            "GripButton": bool(gripR),
        }

        left_input  = ControllerInput(left_joy, left_index, left_grip, left_buttons)
        right_input = ControllerInput(right_joy, right_index, right_grip, right_buttons)

        return VRFrame(head_pose, left_pose, right_pose, left_input, right_input)

    def stream_quest_to_robot(self):
        #self.get_logger().info("Q zero: "+str(self.current_q))

        if self.current_q is None:
            self.get_logger().info("Returned, no current q")
            return
        if self.latest_data != None: #try:
            #data, addr = self.sock.recvfrom(4096)
            #frame = self.decode_packet(data)

            #Get a fresh head position every time

            #self.publish_controller_tf(self.latest_data.right_pose.matrix)
            if not hasattr(self, "initialized"):
                #self.get_logger().info(" Robot models attributed: "+ str(dir(self.robot_models)))
                ee_id = self.robot_models.robot_model.getFrameId(self.ee_frame_name)
                data = pin.Data(self.robot_models.robot_model)
                pin.forwardKinematics(self.robot_models.robot_model, data, self.current_q)
                pin.updateFramePlacements(self.robot_models.robot_model, data)

                # Extract the end-effector SE3 at the beginning
                self.ee_pose = data.oMf[ee_id]
                self.quest_ref=self.ee_pose.homogeneous
                self.last_pos=self.latest_data.right_pose.matrix.copy()

                T_head = self.latest_data.head_pose.matrix
                T_hand = self.latest_data.right_pose.matrix
                T_head_inv = np.linalg.inv(T_head)
                T_hand_in_head = T_head_inv @ T_hand


                self.last_hand_in_head = T_hand_in_head.copy()
                self.last_yaw = np.arctan2(T_hand_in_head[:3, :3][1,0], T_hand_in_head[:3, :3][0,0])
                self.initialized=True
                

                '''
                head_fwd=self.latest_data.head_pose.matrix[:3, 2]
                head_fwd[1]=0
                head_fwd=head_fwd/(np.linalg.norm(head_fwd)+1e-6)

                head_right=self.latest_data.head_pose.matrix[:3, 0]
                head_right[1]=0
                head_right=head_right/(np.linalg.norm(head_right)+1e-6)

                R_head_flat=np.stack((head_right, np.array([0, 1, 0]), head_fwd))
                self.R_head_flat=R_head_flat.copy()
                
                self.last_pos=self.latest_data.right_pose.matrix.copy()
                '''
                return
        else: #except BlockingIOError:
            #self.get_logger().info("Filling up buffer with current pos")

            #This will throw an error because the current q is not an SE3 element
            #self.buffer.append(self.current_q)
            quat_repr = pin.XYZQUATToSE3(self.current_q)
            self.publish_buffer()
            return


        #self.get_logger().info("EE pos of the robot: "+str(frame.right_pose.matrix))
        
        #With this, I projet the forward and right head orientation on the ground plane to project my hand movements into.
        #Using this, I avoid any effect when I look up or down
        #Get the head forward vector, which is the last column of the rotation matrix
        head_fwd=self.latest_data.head_pose.matrix[:3, 2]
        head_fwd[1]=0
        head_fwd=head_fwd/(np.linalg.norm(head_fwd)+1e-6)

        #Get the head right vector, which is the first column of the rotation matrix
        head_right=self.latest_data.head_pose.matrix[:3, 0]
        head_right[1]=0
        head_right=head_right/(np.linalg.norm(head_right)+1e-6)

        R_head_flat=np.stack((head_right, np.array([0, 1, 0]), head_fwd))
        self.R_head_flat=R_head_flat.copy()
        #self.last_pos=self.latest_data.right_pose.matrix.copy()
        
        #With this transformation matrix, the movements of the robot actually reflect the 
        R_rq = np.array([
                [ 0,  0,  1],   # Unity Z → Robot X
                [-1,  0,  0],   # Unity X → Robot -Y
                [ 0,  1,  0],   # Unity Y → Robot Z
            ])

        Tq=self.latest_data.right_pose.matrix
        #Tq0 = self.quest_ref
        
        #delta quest movement
        #delta_T = np.linalg.inv(self.last_pos) @ Tq
        #delta_right_movement=Tq[:3, 3] - self.last_pos[:3, 3]
        #delta_T =  Tq[:3, 3] - self.last_pos[:3, 3]
        #self.last_pos=Tq.copy()

        #hand motion relative to the head
        T_head = self.latest_data.head_pose.matrix
        T_hand = self.latest_data.right_pose.matrix
        T_head_inv = np.linalg.inv(T_head)
        T_hand_in_head = T_head_inv @ T_hand

        if self.rotation:
            yaw_curr = np.arctan2(T_hand_in_head[2,0], T_hand_in_head[0,0])
            yaw_prev = np.arctan2(self.last_hand_in_head[2,0], self.last_hand_in_head[0,0])

            # delta yaw (how much the hand rotated since last frame)
            delta_yaw = yaw_curr - yaw_prev
            yaw_threshold = 0.0001  # radians
            if abs(delta_yaw) > yaw_threshold:
                R_ee = self.quest_ref[:3,:3]
                R_ee = R_ee @ pin.exp3(np.array([0, 0, delta_yaw]))
            else:
                R_ee = self.quest_ref[:3,:3]

        delta_T =  T_hand_in_head[:3, 3] - self.last_hand_in_head[:3, 3]
        self.last_hand_in_head = T_hand_in_head.copy()
        #delta_forward=np.dot(delta_quest, self.R_head_flat[:, 1])
        #delta_right=np.dot(delta_quest, self.R_head_flat[:, 2])
        #delta_vertical=delta_quest[1]


        dr=self.R_head_flat @ delta_T
        dr=R_rq @ dr 

        #Slow down movement
        #delta_T[:3, 3] *= 0.8 
        Tee= self.quest_ref[:3, 3]+dr #self.quest_ref @ delta_T

        #self.get_logger().info("Delta movements: "+str(Tee))
        #self.get_logger().info("Original Pos: "+str(self.quest_ref[:3, :3]))
        #get position and set velocity to zero
        #R = frame.right_pose.matrix[:3, :3]
        #t = frame.right_pose.matrix[:3, 3]
        
        #ee_des_pos
        if self.rotation:
            self.quest_ref= pin.SE3(R_ee, Tee)#self.quest_ref= pin.SE3(self.quest_ref[:3, :3], Tee)
        else:
            self.quest_ref= pin.SE3(self.quest_ref[:3, :3], Tee)
        self.buffer.append(self.quest_ref) #ee_des_pos
        self.quest_ref=self.quest_ref.homogeneous
        self.publish_controller_tf(self.quest_ref)
        #self.get_logger().info("Publishing data")
        self.publish_buffer()
        #self.get_logger().info("Desired pose: "+str(pin.SE3ToXYZQUAT(ee_des_pos)))
        #self.last_pos=copy.deepcopy(frame.right_pose.matrix)
            
        #get IK and torques
        #q, dq = self.inverse_kinematics(ee_des_pos, ee_des_vel)
        #ddq = np.zeros(self.pin_model.nv)
        #u = pin.rnea(self.pin_model, self.pin_data, q, dq, ddq)



    def publish_buffer(self):
        for i in range(len(self.buffer)):
            el=self.buffer.pop(0)
            #self.get_logger().info(f"Element: {el}")
            traj_point = TrajectoryPoint(
                id=0,
                time_ns=self.get_clock().now().nanoseconds, #+ int(0.2e9),
                robot_configuration=self.current_q,
                robot_velocity=np.zeros_like(self.current_q),
                robot_acceleration=np.zeros_like(self.current_q),
                robot_effort=np.zeros_like(self.current_q),
                end_effector_poses={self.ee_frame_name: pin.SE3ToXYZQUAT(el)},
            )

            traj_weights = TrajectoryPointWeights(
                w_robot_configuration=np.full_like(self.current_q, 1.0), #dtype=float), #self.w_q,
                w_robot_velocity=np.full_like(self.current_q, 0.1), #self.w_qdot,
                w_robot_acceleration= np.full_like(self.current_q, 0.000001), #self.w_qddot,
                w_robot_effort=np.full_like(self.current_q, 0.0008), #self.w_robot_effort,
                w_end_effector_poses={self.ee_frame_name: [30.0, 30.0, 30.0, 30.0, 30.0, 30.0] },#self.w_pose
                #w_end_effector_poses={self.ee_frame_name: [3.0, 3.0, 3.0, 1.0, 1.0, 1.0] },
            )
            weighted_traj_point=WeightedTrajectoryPoint(point=traj_point, weights=traj_weights)

            mpc_msg = weighted_traj_point_to_mpc_msg(weighted_traj_point)
            self.publisher_.publish(mpc_msg)
            self._marker_base.pose=mpc_msg.ee_inputs[0].pose
            #self.get_logger().info("Marker base pos: "+str(self._marker_base.pose))
            self._target_pose_marker_pub.publish(self._marker_base)



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)
    node = QuestTrajectoryPublisher()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__== '__main__':
    main()
    