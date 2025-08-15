import time

import numpy as np
import pinocchio as pin

from nltrajopt.trajectory_optimization import NLTrajOpt
from nltrajopt.contact_scheduler import ContactScheduler
from nltrajopt.node import Node
from nltrajopt.constraint_models import *
from nltrajopt.cost_models import *
from terrain.terrain_grid import TerrainGrid

from robots.g1.G1Wrapper import G1

from visualiser.visualiser import TrajoptVisualiser

VIS = True
DT = 0.05

# terrain = TerrainGrid(10, 10, 0.9, -1.0, -5.0, 5.0, 5.0)
# terrain.set_zero()
# terrain.grid[3:, :] = 0.05

terrain = TerrainGrid(100, 100, 0.9, -0.5, -2.0, 1.5, 2.0)
terrain.set_zero()
terrain.grid[60:65, :] = 0.65

robot = G1()
q = robot.go_neutral()

contacts_dict = {
    "l_foot": robot.left_foot_frames,
    "r_foot": robot.right_foot_frames,
    "l_gripper": robot.left_gripper_frames,
    "r_gripper": robot.right_gripper_frames,
}

contact_scheduler = ContactScheduler(robot.model, dt=DT, contact_frame_dict=contacts_dict)

contact_scheduler.add_phase(["l_foot", "r_foot"], 0.5)
# contact_scheduler.add_phase(["r_foot"], 0.2)
contact_scheduler.add_phase(["l_foot"], 0.1)
# contact_scheduler.add_phase(["r_foot"], 0.2)
# contact_scheduler.add_phase(["l_gripper", "l_foot", "r_foot"], 0.1)
contact_scheduler.add_phase(["l_foot", "r_foot"], 0.5)

frame_contact_seq = contact_scheduler.contact_sequence_fnames

contact_frame_names = robot.left_foot_frames + robot.right_foot_frames + robot.left_gripper_frames + robot.right_gripper_frames

stages = []

for contact_phase_fnames in frame_contact_seq:
    stage_node = Node(
        robot.model.nv,
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
            # FrictionConstraints(0.8),
            # ContactConstraints(),
            TerrainGridFrictionConstraints(terrain),
            TerrainGridContactConstraints(terrain),
        ]
    )

    # stage_node.costs_list.extend(
    #     [ConfigurationCost(q.copy()[7:], np.eye(robot.model.nv - 6) * 1e-6)]
    # )

    stages.append(stage_node)


opti = NLTrajOpt(model=robot.model, nodes=stages, dt=DT)

opti.set_initial_pose(q)
qf = np.copy(q)
qf[0] = 0.0
qf[2] += terrain.height(qf[0], qf[1])
opti.set_target_pose(qf)

result = opti.solve(50, 1e-3, False, 5)

K = len(result["nodes"])
dts = [result["nodes"][k]["dt"] for k in range(K)]
state_trajectory = [result["nodes"][k]["q"] for k in range(K)]
forces = [result["nodes"][k]["forces"] for k in range(K)]

if VIS:
    tvis = TrajoptVisualiser(robot)

    interp_states = state_trajectory
    tvis.display_robot_q(robot, state_trajectory[0])

    # Visualise the terrain
    tvis.load_terrain(terrain)

    time.sleep(1)
    while True:
        for i in range(len(interp_states)):
            time.sleep(dts[i])
            tvis.display_robot_q(robot, interp_states[i])
            tvis.update_forces(robot, forces[i], 0.01)

        tvis.update_forces(robot, {}, 0.01)
