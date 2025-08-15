import time

import numpy as np
import pinocchio as pin

from nltrajopt.trajectory_optimization import NLTrajOpt
from nltrajopt.contact_scheduler import ContactScheduler
from nltrajopt.node import Node
from nltrajopt.constraint_models import *
from nltrajopt.cost_models import *
from terrain.terrain_grid import TerrainGrid
from robots.talos.TalosWrapper import Talos
from visualiser.visualiser import TrajoptVisualiser

import nltrajopt.params as pars

VIS = pars.VIS
DT = 0.05

terrain = TerrainGrid(10, 10, 0.9, -1.0, -5.0, 5.0, 5.0)
terrain.set_zero()

robot = Talos()
q = robot.go_neutral()

contacts_dict = {
    "l_foot": robot.left_foot_frames,
    "r_foot": robot.right_foot_frames,
    "l_gripper": robot.left_gripper_frames,
    "r_gripper": robot.right_gripper_frames,
}

contact_scheduler = ContactScheduler(robot.model, dt=DT, contact_frame_dict=contacts_dict)

contact_scheduler.add_phase(["l_foot", "r_foot"], 1.0)
for i in range(3):
    contact_scheduler.add_phase(["l_foot"], 0.4)
    contact_scheduler.add_phase(["l_foot", "r_foot"], 0.3)
    contact_scheduler.add_phase(["r_foot"], 0.4)
    contact_scheduler.add_phase(["l_foot", "r_foot"], 0.3)
contact_scheduler.add_phase(["l_foot", "r_foot"], 1.0)


frame_contact_seq = contact_scheduler.contact_sequence_fnames

contact_frame_names = robot.left_foot_frames + robot.right_foot_frames + robot.left_gripper_frames + robot.right_gripper_frames

stages = []

for contact_phase_fnames in frame_contact_seq:
    stage_node = Node(
        nv=robot.model.nv,
        contact_phase_fnames=contact_phase_fnames,
        contact_fnames=contact_frame_names,
    )

    dyn_const = WholeBodyDynamics()
    stage_node.dynamics_type = dyn_const.name

    stage_node.constraints_list.extend(
        [
            dyn_const,
            TimeConstraint(min_dt=DT, max_dt=DT, total_time=None),
            SemiEulerIntegration(),
            TerrainGridFrictionConstraints(terrain),
            TerrainGridContactConstraints(terrain),
        ]
    )

    stage_node.costs_list.extend([ConfigurationCost(q.copy()[7:], np.eye(robot.model.nv - 6) * 1e-6)])

    stages.append(stage_node)

opti = NLTrajOpt(model=robot.model, nodes=stages, dt=DT)

opti.set_initial_pose(q)
qf = np.copy(q)
qf[0] = 2.0
qf[2] += terrain.height(qf[0], qf[1])
opti.set_target_pose(qf)

result = opti.solve(50, 1e-3, parallel=False, print_level=5)
opti.save_solution("walk")


K = len(result["nodes"])
dts = [result["nodes"][k]["dt"] for k in range(K)]
qs = [result["nodes"][k]["q"] for k in range(K)]
forces = [result["nodes"][k]["forces"] for k in range(K)]

if VIS:
    tvis = TrajoptVisualiser(robot)
    tvis.display_robot_q(robot, qs[0])

    time.sleep(1)
    while True:
        for i in range(len(qs)):
            time.sleep(dts[i])
            tvis.display_robot_q(robot, qs[i])
            tvis.update_forces(robot, forces[i], 0.01)
        tvis.update_forces(robot, {}, 0.01)
