import time

import pinocchio as pin
import numpy as np

from robots.talos.TalosWrapper import Talos
from robots.g1.G1Wrapper import G1
from robots.go2.Go2Wrapper import Go2
from nltrajopt.trajectory_optimization import NLTrajOpt
from nltrajopt.se3tangent import *

from visualiser.visualiser import TrajoptVisualiser

import matplotlib.pyplot as plt

import imageio
import os

robot = Talos()
# robot = G1()
# robot = Go2()


TRAJ_PATH = "trajopt_solutions_batch/backflip/tan/backflip_20052025_141802.json"
parent_path = os.path.dirname(TRAJ_PATH)

results = NLTrajOpt.load_solution(TRAJ_PATH)

K = len(results["nodes"])
dts = [results["nodes"][k]["dt"] for k in range(K)]
qs = [results["nodes"][k]["q"] for k in range(K)]
forces = [results["nodes"][k]["forces"] for k in range(K)]


rdata = robot.model.createData()

print(1 / np.mean(dts))
print(K)
time.sleep(1)
tvis = TrajoptVisualiser(robot)
tvis.display_robot_q(robot, qs[0])

frames = []


stop_flag = [False]


try:
    while True:
        for i in range(len(qs)):
            time.sleep(dts[i])
            tvis.display_robot_q(robot, qs[i])
except KeyboardInterrupt:
    print("end")
    pass
