import os
import numpy as np  # Linear Algebra

import pinocchio as pin  # Pinocchio library

import example_robot_data

from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.shortcuts import buildModelsFromUrdf
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g


class G1:
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        urdf_path = os.path.join(dir_path, "g1/urdf/g1_29dof.urdf")
        model_path = os.path.join(dir_path, "g1")

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path, package_dirs=[model_path]
        )

        self.data = self.model.createData()

        self.left_foot_frames = [
            "left_foot_upper_left",
            "left_foot_upper_right",
            "left_foot_lower_left",
            "left_foot_lower_right",
        ]
        self.right_foot_frames = [
            "right_foot_upper_left",
            "right_foot_upper_right",
            "right_foot_lower_left",
            "right_foot_lower_right",
        ]
        self.left_gripper_frames = ["left_hand_point_contact"]
        self.right_gripper_frames = ["right_hand_point_contact"]

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
        q[2] = 0.6756
        q[7] = -0.6
        q[10] = 1.2
        q[11] = -0.6
        q[13] = -0.6
        q[16] = 1.2
        q[17] = -0.6

        self.fk_all(q)
        return q


if __name__ == "__main__":
    robot = G1()
    # talos.load_visualizer()
    q = robot.go_neutral()
    # talos.display(q)

    robot.fk_all(q)

    for i, frame in enumerate(robot.model.frames):
        print(
            f"Frame {i}: Name = {frame.name}, Type = {frame.type}, Parrent = {frame.parentJoint} "
        )

    print(robot.model)

    # for i in range(talos.model.fr):
    # print(i, talos.data.joints[i].M)
