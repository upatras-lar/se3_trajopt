import numpy as np
import time
import json
import pinocchio as pin

from nltrajopt.trajectory_optimization import NLTrajOpt
from nltrajopt.contact_scheduler import ContactScheduler
from nltrajopt.node import Node
from nltrajopt.constraint_models import *
from nltrajopt.cost_models import *
import nltrajopt.utils as reprutils


from terrain.terrain_grid import TerrainGrid
from robots.talos.TalosWrapper import Talos


from visualiser.visualiser import TrajoptVisualiser
import nltrajopt.params as pars

np.set_printoptions(precision=2, suppress=False)


VIS = pars.VIS
DT = 0.05

robot = Talos()
q = robot.go_neutral()

terrain = TerrainGrid(10, 10, 0.6, -1.0, -5.0, 5.0, 5.0)
terrain.set_zero()

contacts_dict = {
    "l_foot": robot.left_foot_frames,
    "r_foot": robot.right_foot_frames,
    "l_gripper": robot.left_gripper_frames,
    "r_gripper": robot.right_gripper_frames,
}

contact_scheduler = ContactScheduler(robot.model, dt=DT, contact_frame_dict=contacts_dict)

contact_scheduler.add_phase(["l_foot", "r_foot"], 0.5)
contact_scheduler.add_phase(["l_foot", "r_foot", "l_gripper", "r_gripper"], 0.2)
k1 = len(contact_scheduler.contact_sequence_fnames)
contact_scheduler.add_phase(["l_gripper", "r_gripper"], 2.0)
k2 = len(contact_scheduler.contact_sequence_fnames)
contact_scheduler.add_phase(["l_foot", "r_foot", "l_gripper", "r_gripper"], 0.2)
contact_scheduler.add_phase(["l_foot", "r_foot"], 0.5)


frame_contact_seq = contact_scheduler.contact_sequence_fnames


contact_frame_names = robot.left_foot_frames + robot.right_foot_frames + robot.left_gripper_frames + robot.right_gripper_frames


stages = []
K = len(frame_contact_seq)
print("K = ", K)
for k, contact_phase_fnames in enumerate(frame_contact_seq):

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
            TerrainGridContactConstraints(terrain),
            TerrainGridFrictionConstraints(terrain, max_delta_force=100.0),
        ]
    )

    nu = robot.model.nv - 6
    ref = np.zeros((nu,))

    stage_node.costs_list.extend([ConfigurationCost(q.copy()[7:], np.eye(robot.model.nv - 6) * 1e-9)])
    # stage_node.costs_list.extend([JointVelocityCost(np.zeros((robot.model.nv - 6,)), np.eye(robot.model.nv - 6) * 1e-6)])
    # stage_node.costs_list.append(JointAccelerationCost(ref, np.eye(nu) * 1e-3))

    stages.append(stage_node)


opti = NLTrajOpt(model=robot.model, nodes=stages, dt=DT)


opti.set_initial_pose(q)
qf = np.copy(q)
opti.set_target_pose(qf)


theta = np.pi
# warm start
for k, node in enumerate(opti.nodes):
    if k1 <= k <= k2:
        opti.x0[node.q_id] = reprutils.rpy2rep(q, [0.0, theta, 0.0])


result = opti.solve(200, 1e-3, False, 0)

opti.save_solution("handstand")


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
