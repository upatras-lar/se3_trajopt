import unittest
import numpy as np
import pinocchio as pin
from nltrajopt.trajectory_optimization import *
from robots.talos.TalosWrapper import Talos
from nltrajopt.cost_models import *
import copy

from terrain.terrain_grid import TerrainGrid

from nltrajopt.constraint_models import TimeConstraint
from nltrajopt.constraint_models.casadi import *

np.set_printoptions(precision=3, suppress=True, linewidth=400)


def finite_diff(f, z, M, eps=1e-6):
    N = z.shape[0]
    jac = np.zeros((M, N))
    for i in range(N):
        zp = np.copy(z)
        zp[i] += eps
        zm = np.copy(z)
        zm[i] -= eps
        jac[:, i : i + 1] = ((f(zp) - f(zm)) / (2.0 * eps)).reshape((-1, 1))
    return jac


class TestWBTrajOpt(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Create test terrain
        terrain = TerrainGrid(10, 10, 0.9, -5.0, -5.0, 5.0, 5.0)
        terrain.set_zero()
        terrain.grid = np.random.randn(10, 10)

        # Load the Talos model once for all tests
        cls.talos = Talos()
        DT = 0.05

        contact_gait = [["left", "right"]] * 4
        contact_gait[1] = ["left"]
        contact_gait[2] = ["right"]
        contact_gait[3] = []

        left_frames = ["left_mm", "left_mp", "left_pp", "left_pm"]
        right_frames = ["right_mm", "right_mp", "right_pp", "right_pm"]

        contact_frames = [[] for _ in range(len(contact_gait))]
        for k, phase in enumerate(contact_gait):
            if "left" in phase:
                contact_frames[k].extend(left_frames)
            if "right" in phase:
                contact_frames[k].extend(right_frames)

        stages = []

        for contact_phase_fnames in contact_frames:
            stage_node = Node(
                nq=cls.talos.model.nv,
                nv=cls.talos.model.nv,
                contact_phase_fnames=contact_phase_fnames,
                contact_fnames=left_frames + right_frames,
            )

            dyn_const = RPYWholeBodyDynamics()
            stage_node.dynamics_type = dyn_const.name

            stage_node.constraints_list.extend(
                [
                    # RPYWholeBodyDynamics(),
                    # TimeConstraint(min_dt=DT, max_dt=DT, total_time=None),
                    RPYSemiEulerIntegration(),
                    # # FrictionConstraints(0.7),
                    # # ContactConstraints(),
                    # RPYTerrainGridFrictionConstraints(terrain),
                    # RPYTerrainGridContactConstraints(terrain),
                ]
            )

            # for fname in contact_phase_fnames:
            #     const = ContactConstraints(fname, [1.0, None, 1.8])
            #     stage_node.constraints_list.append(const)

            q = np.random.rand((cls.talos.model.nv - 6))
            stage_node.costs_list.append(ConfigurationCost(q, np.eye(cls.talos.model.nv - 6)))

            stages.append(stage_node)

        cls.opti = NLTrajOpt(model=cls.talos.model, dt=DT, nodes=stages)

    def test_node_grad(self):
        xx = np.random.random(self.opti.x0.shape)

        JacDense = self.opti.jac_test(xx)
        JacSparse = self.opti.jacobian(xx)
        JacDiff = finite_diff(self.opti.constraints, xx, self.opti.cons_dim)

        print()
        print(JacDense.shape)
        print(JacDiff.shape)
        dif = JacDense - JacDiff
        print(np.linalg.norm(dif))

        k = 0
        for i in range(dif.shape[0]):
            for j in range(dif.shape[1]):
                if np.abs(dif[i, j]) >= 1e-5:
                    print((i, j), JacDiff[i, j], JacDense[i, j])
                    k += 1
        print(k)

        sparse_structure = self.opti.jacobianstructure()
        diff_structure = JacDiff.nonzero()

        extracted_dense_jac = JacDense[diff_structure]  # Extract corresponding elements
        extracted_diff_jac = JacDiff[diff_structure]  # Extract corresponding elements

        dif = extracted_dense_jac - extracted_diff_jac
        k = 0
        for i in range(len(diff_structure[0])):
            if np.abs(dif[i]) >= 1e-5:
                print(
                    (diff_structure[0][i], diff_structure[1][i]),
                    extracted_dense_jac[i],
                    extracted_diff_jac[i],
                )
                k += 1
        print(k)

        # self.assertTrue(np.allclose(extracted_diff_jac, extracted_dense_jac, rtol=1e-6, atol=1e-9))
        self.assertTrue(np.allclose(JacDiff, JacDense, 1e-8, 1e-5))

    # def test_cost_grad(self):
    #     # self.opti.R_base_ori = np.eye(3)
    #     # self.opti.base_zori_ref = np.array([[0, 0, -1]]).T
    #     xx = np.random.random(self.opti.x0.shape)

    #     Jac = self.opti.gradient(xx)
    #     JacDiff = finite_diff(self.opti.objective, xx, 1)

    #     print()
    #     Jac = Jac.reshape(1, -1)
    #     print(Jac.shape)
    #     print(JacDiff.shape)
    #     dif = Jac - JacDiff
    #     print(np.linalg.norm(dif))

    #     k = 0
    #     for i in range(dif.shape[0]):
    #         for j in range(dif.shape[1]):
    #             if np.abs(dif[i, j]) >= 1e-5:
    #                 print((i, j), JacDiff[i, j], Jac[i, j])
    #                 k += 1
    #     print(k)


if __name__ == "__main__":
    unittest.main()
