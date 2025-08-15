import os
import numpy as np  # Linear Algebra

import pinocchio as pin  # Pinocchio library

import example_robot_data

from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.shortcuts import buildModelsFromUrdf
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g


class Talos:
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        urdf_path = os.path.join(dir_path, "talos/urdf/talos.urdf")
        model_path = os.path.join(dir_path, "talos")

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path, package_dirs=[model_path]
        )

        self.data = self.model.createData()

        self.left_foot_frames = ["left_mm", "left_mp", "left_pp", "left_pm"]
        # self.left_foot_frame_ids = [
        #     self.model.getFrameId(f) for f in self.left_foot_frames
        # ]

        self.right_foot_frames = ["right_mm", "right_mp", "right_pp", "right_pm"]
        # self.right_foot_frame_ids = [
        #     self.model.getFrameId(f) for f in self.right_foot_frames
        # ]

        self.left_gripper_frames = ["gripper_left_inner_single_link"]
        # self.left_gripper_frames_ids = [
        #     self.model.getFrameId(f) for f in self.left_gripper_frames
        # ]

        self.right_gripper_frames = ["gripper_right_inner_single_link"]
        # self.right_gripper_frames_ids = [
        #     self.model.getFrameId(f) for f in self.right_gripper_frames
        # ]

    def fk_all(self, q, v=None):
        if v is not None:
            pin.forwardKinematics(
                self.model, self.data, q, v
            )  # FK and Forward Velocities
        else:
            pin.forwardKinematics(self.model, self.data, q)  # FK
        pin.updateFramePlacements(self.model, self.data)  # Update frames

    def go_neutral(self):
        q = pin.neutral(self.model)

        q[2] = 1.0274
        # left arm
        q[21] = 0.3
        q[22] = 0.4
        q[23] = -0.5
        q[24] = -1.2
        # right arm
        q[28] = -0.3
        q[29] = -0.4
        q[30] = 0.5
        q[31] = -1.2

        # left leg
        q[9] = -0.4
        q[10] = 0.8
        q[11] = -0.4

        # right leg
        q[15] = -0.4
        q[16] = 0.8
        q[17] = -0.4

        self.fk_all(q)
        return q


if __name__ == "__main__":
    talos = Talos()
    # talos.load_visualizer()
    q = talos.go_neutral()
    # talos.display(q)

    talos.fk_all(q)

    for i, frame in enumerate(talos.model.frames):
        print(
            f"Frame {i}: Name = {frame.name}, Type = {frame.type}, Parrent = {frame.parentJoint} "
        )

    print(talos.model)

    # for i in range(talos.model.fr):
    # print(i, talos.data.joints[i].M)
