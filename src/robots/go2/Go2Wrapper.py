import os
import numpy as np  # Linear Algebra

import pinocchio as pin  # Pinocchio library

import example_robot_data

from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.shortcuts import buildModelsFromUrdf
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as g


class Go2:
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        urdf_path = os.path.join(dir_path, "go2/urdf/go2.urdf")
        model_path = os.path.join(dir_path, "go2")

        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path, package_dirs=[model_path]
        )

        self.data = self.model.createData()

        self.left_foot_frames = ["RL_foot"]
        self.right_foot_frames = ["RR_foot"]
        self.left_gripper_frames = ["FL_foot"]
        self.right_gripper_frames = ["FR_foot"]

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
        q[2] = 0.31131
        q[8] = 0.7
        q[9] = -1.5
        q[11] = 0.7
        q[12] = -1.5
        q[14] = 0.7
        q[15] = -1.5
        q[17] = 0.7
        q[18] = -1.5

        self.fk_all(q)
        return q


if __name__ == "__main__":
    go2 = Go2()
    # talos.load_visualizer()
    q = go2.go_neutral()
    # talos.display(q)

    go2.fk_all(q)

    for i, frame in enumerate(go2.model.frames):
        print(
            f"Frame {i}: Name = {frame.name}, Type = {frame.type}, Parrent = {frame.parentJoint} "
        )

    print(go2.model)

    # for i in range(go2.model.fr):
    # print(i, go2.data.joints[i].M)
