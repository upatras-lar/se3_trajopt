from nltrajopt.constraint_models.abstract_constraint import *
from terrain.terrain_grid import TerrainGrid


class TerrainGridContactConstraints(AbstractConstraint):
    """Contact position and velocity constraints"""

    def __init__(self, terrain):
        """
        Args:
            terrain: Terrain grid model of ground
        """
        self.terrain = terrain

    @property
    def name(self) -> str:
        return "terrain_grid_contact"

    def compute_constraints(self, node_curr: Node, node_next, state_vars, c, model, data):
        """Compute contact position and velocity constraints"""
        q = q_tan2pin(state_vars[node_curr.q_id])
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        for frame in node_curr.contact_fnames:
            frame_id = model.getFrameId(frame)

            if frame not in node_curr.contact_phase_fnames:
                ee_x = data.oMf[frame_id].translation[0]
                ee_y = data.oMf[frame_id].translation[1]
                ee_z = data.oMf[frame_id].translation[2]
                # Height constraint: p_z ≥ ground
                c[node_curr.c_z_ids[frame]] = ee_z - self.terrain.height(ee_x, ee_y)
            else:

                # Position constraint: p_contact - p_frame = 0
                c[node_curr.c_contact_kinematics_ids[frame]] = data.oMf[frame_id].translation - state_vars[node_curr.contact_pos_ids[frame]]

                ee_x = state_vars[node_curr.contact_pos_ids[frame]][0]
                ee_y = state_vars[node_curr.contact_pos_ids[frame]][1]
                ee_z = state_vars[node_curr.contact_pos_ids[frame]][2]
                c[node_curr.c_z_ids[frame]] = ee_z - self.terrain.height(ee_x, ee_y)

                # Velocity constraint: p_next - p_curr = 0 (if next node exists)
                if node_next and frame in node_next.contact_phase_fnames:
                    c[node_curr.c_vel_ids[frame]] = (
                        state_vars[node_next.contact_pos_ids[frame]] - state_vars[node_curr.contact_pos_ids[frame]]
                    )

    def compute_jacobians(self, node_curr: Node, node_next, w, jac, model, data):
        """Compute contact Jacobians"""
        q = q_tan2pin(w[node_curr.q_id])
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        for frame in node_curr.contact_fnames:
            frame_id = model.getFrameId(frame)
            J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL_WORLD_ALIGNED)
            J[:, :6] = J[:, :6] @ pin.Jexp6(w[node_curr.q_id][:6])  # Convert to tangent space

            if frame not in node_curr.contact_phase_fnames:
                ee_x = data.oMf[frame_id].translation[0]
                ee_y = data.oMf[frame_id].translation[1]
                dx_dh = self.terrain.dx_dheight(ee_x, ee_y)
                dy_dh = self.terrain.dy_dheight(ee_x, ee_y)
                jac[node_curr.c_z_ids[frame], node_curr.q_id] = J[2, :] - dx_dh * J[0, :] - dy_dh * J[1, :]

            else:
                # Position constraint Jacobians
                jac[node_curr.c_contact_kinematics_ids[frame], node_curr.q_id] = J[:3, :]
                jac[
                    node_curr.c_contact_kinematics_ids[frame],
                    node_curr.contact_pos_ids[frame],
                ] = -np.eye(3)

                # Height constraint Jacobian
                ee_x = w[node_curr.contact_pos_ids[frame]][0]
                ee_y = w[node_curr.contact_pos_ids[frame]][1]
                dx_dh = self.terrain.dx_dheight(ee_x, ee_y)
                dy_dh = self.terrain.dy_dheight(ee_x, ee_y)
                jac[node_curr.c_z_ids[frame], node_curr.contact_pos_ids[frame]] = [
                    -dx_dh,
                    -dy_dh,
                    1,
                ]

                # Velocity constraint Jacobians
                if node_next and frame in node_next.contact_phase_fnames:
                    jac[node_curr.c_vel_ids[frame], node_next.contact_pos_ids[frame]] = np.eye(3)
                    jac[node_curr.c_vel_ids[frame], node_curr.contact_pos_ids[frame]] = -np.eye(3)

    def get_structure_ids(self, node_curr, node_next, row_ids, col_ids):
        """Contact constraints sparsity pattern"""

        for frame in node_curr.contact_fnames:

            if frame not in node_curr.contact_phase_fnames:
                extend_ids_lists(row_ids, col_ids, node_curr.c_z_ids[frame], node_curr.q_id)
            else:
                extend_ids_lists(
                    row_ids,
                    col_ids,
                    node_curr.c_contact_kinematics_ids[frame],
                    node_curr.q_id,
                )
                extend_ids_lists(
                    row_ids,
                    col_ids,
                    node_curr.c_contact_kinematics_ids[frame],
                    node_curr.contact_pos_ids[frame],
                )
                extend_ids_lists(
                    row_ids,
                    col_ids,
                    node_curr.c_z_ids[frame],
                    node_curr.contact_pos_ids[frame],
                )
                if node_next is not None and frame in node_next.contact_phase_fnames:
                    extend_ids_lists(
                        row_ids,
                        col_ids,
                        node_curr.c_vel_ids[frame],
                        node_next.contact_pos_ids[frame],
                    )
                    extend_ids_lists(
                        row_ids,
                        col_ids,
                        node_curr.c_vel_ids[frame],
                        node_curr.contact_pos_ids[frame],
                    )

    def get_bounds(
        self,
        node: "Node",
        lb: np.ndarray,
        ub: np.ndarray,
        clb: np.ndarray,
        cub: np.ndarray,
        model: pin.Model,
    ) -> None:
        """Contact bounds"""

        for frame in node.contact_fnames:
            if frame not in node.contact_phase_fnames:
                cub[node.c_z_ids[frame]] = [None]


class TerrainGridFrictionConstraints(AbstractConstraint):
    """Friction cone constraints (pyramid approximation)"""

    def __init__(self, terrain, max_force: float = -1.0, max_delta_force=-1.0):
        """
        Args:
            terrain: Terrain grid model of ground
        """
        self.terrain = terrain
        self.max_force = max_force
        self.max_delta_force = max_delta_force

    @property
    def name(self) -> str:
        return "friction"

    def compute_constraints(self, node_curr, node_next, state_vars, c, model, data):
        """Compute friction cone constraints (4 linear inequalities per contact + 1 max force)"""
        q = q_tan2pin(state_vars[node_curr.q_id])
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        max_force = 1.5 * pin.computeTotalMass(model) * np.linalg.norm(model.gravity.vector) if self.max_force <= 0.0 else self.max_force

        for frame in node_curr.contact_phase_fnames:
            frame_id = model.getFrameId(frame)
            F_world = (data.oMf[frame_id].rotation @ state_vars[node_curr.forces_ids[frame]]).reshape((3, 1))

            ee_x = data.oMf[frame_id].translation[0]
            ee_y = data.oMf[frame_id].translation[1]

            n = self.terrain.n(ee_x, ee_y)
            t1 = self.terrain.t1(ee_x, ee_y)
            t2 = self.terrain.t2(ee_x, ee_y)

            mu = self.terrain.mu
            c[node_curr.c_friction_ids[frame]] = [
                (F_world.T @ (n * mu - t1))[0, 0],  # mu_Fz - Fx
                (F_world.T @ (n * mu + t1))[0, 0],  # mu_Fz + Fx
                (F_world.T @ (n * mu - t2))[0, 0],  # mu_Fz - Fy
                (F_world.T @ (n * mu + t2))[0, 0],  # mu_Fz + Fy
                # max_force - (F_world.T @ n)[0, 0],  # Fz ≤ max_force
                (F_world.T @ n)[0, 0],  # Fz>=0
                max_force - ((F_world.T @ F_world) ** (0.5))[0, 0],  # ||F|| ≤ max_force
            ]

            # delta force constraint
            if node_next is None:
                return
            f1 = state_vars[node_curr.forces_ids[frame]]
            if frame in node_next.contact_phase_fnames:
                f2 = state_vars[node_next.forces_ids[frame]]
            else:
                f2 = np.zeros((3,))

            c[node_curr.c_delta_force_ids[frame]] = f2 - f1

    def compute_jacobians(self, node_curr, node_next, w, jac, model, data):
        """Compute friction cone Jacobians"""
        q = q_tan2pin(w[node_curr.q_id])
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        for frame in node_curr.contact_phase_fnames:
            frame_id = model.getFrameId(frame)
            R = data.oMf[frame_id].rotation

            Jf = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)
            Jf[:, :6] = Jf[:, :6] @ pin.Jexp6(w[node_curr.q_id][:6])

            Jw = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            Jw[:, :6] = Jw[:, :6] @ pin.Jexp6(w[node_curr.q_id][:6])

            f = w[node_curr.forces_ids[frame]].reshape((3, 1))
            F_world = data.oMf[frame_id].rotation @ f

            ee_x = data.oMf[frame_id].translation[0]
            ee_y = data.oMf[frame_id].translation[1]

            n = self.terrain.n(ee_x, ee_y)
            t1 = self.terrain.t1(ee_x, ee_y)
            t2 = self.terrain.t2(ee_x, ee_y)

            dx_dn = self.terrain.dx_dn(ee_x, ee_y)
            dy_dn = self.terrain.dy_dn(ee_x, ee_y)
            dx_dt1 = self.terrain.dx_dt1(ee_x, ee_y)
            dy_dt1 = self.terrain.dy_dt1(ee_x, ee_y)
            dx_dt2 = self.terrain.dx_dt2(ee_x, ee_y)
            dy_dt2 = self.terrain.dy_dt2(ee_x, ee_y)

            dg_dn = np.block([dx_dn, dy_dn])
            dg_dt1 = np.block([dx_dt1, dy_dt1])
            dg_dt2 = np.block([dx_dt2, dy_dt2])

            tmp = -R @ hat(f) @ Jf[3:, :]

            # dc df
            s_idx = node_curr.c_friction_ids[frame].start
            f_ids = node_curr.forces_ids[frame]
            mu = self.terrain.mu
            jac[s_idx + 0, f_ids] = (n * mu - t1).T @ R
            jac[s_idx + 1, f_ids] = (n * mu + t1).T @ R
            jac[s_idx + 2, f_ids] = (n * mu - t2).T @ R
            jac[s_idx + 3, f_ids] = (n * mu + t2).T @ R
            jac[s_idx + 4, f_ids] = n.T @ R
            jac[s_idx + 5, f_ids] = -((F_world) / ((F_world.T @ F_world) ** (0.5))).T @ R

            # dc dq
            s_idx = node_curr.c_friction_ids[frame].start
            q_ids = node_curr.q_id
            dn_dq = dg_dn @ Jw[:2, :]
            dt1_dq = dg_dt1 @ Jw[:2, :]
            dt2_dq = dg_dt2 @ Jw[:2, :]
            mu = self.terrain.mu

            dc1_dq = F_world.T @ (dn_dq * mu - dt1_dq) + (n * mu - t1).T @ tmp
            dc2_dq = F_world.T @ (dn_dq * mu + dt1_dq) + (n * mu + t1).T @ tmp
            dc3_dq = F_world.T @ (dn_dq * mu - dt2_dq) + (n * mu - t2).T @ tmp
            dc4_dq = F_world.T @ (dn_dq * mu + dt2_dq) + (n * mu + t2).T @ tmp
            dc5_dq = F_world.T @ dn_dq + n.T @ tmp
            dc6_dq = -((F_world) / ((F_world.T @ F_world) ** (0.5))).T @ tmp
            jac[s_idx + 0, q_ids] = dc1_dq
            jac[s_idx + 1, q_ids] = dc2_dq
            jac[s_idx + 2, q_ids] = dc3_dq
            jac[s_idx + 3, q_ids] = dc4_dq
            jac[s_idx + 4, q_ids] = dc5_dq
            jac[s_idx + 5, q_ids] = dc6_dq

            # deltaforce
            if node_next is None:
                return
            if frame in node_next.contact_phase_fnames:
                jac[node_curr.c_delta_force_ids[frame], node_next.forces_ids[frame]] = np.eye(3)
            jac[node_curr.c_delta_force_ids[frame], node_curr.forces_ids[frame]] = -np.eye(3)

    def get_structure_ids(self, node_curr, node_next, row_ids, col_ids):
        """Friction cone sparsity pattern"""

        for frame in node_curr.contact_phase_fnames:
            extend_ids_lists(
                row_ids,
                col_ids,
                node_curr.c_friction_ids[frame],
                node_curr.forces_ids[frame],
            )
            extend_ids_lists(row_ids, col_ids, node_curr.c_friction_ids[frame], node_curr.q_id)

            # deltaforce
            if node_next is None:
                return
            if frame in node_next.contact_phase_fnames:
                extend_ids_lists(row_ids, col_ids, node_curr.c_delta_force_ids[frame], node_next.forces_ids[frame])
            extend_ids_lists(row_ids, col_ids, node_curr.c_delta_force_ids[frame], node_curr.forces_ids[frame])

    def get_bounds(
        self,
        node: "Node",
        lb: np.ndarray,
        ub: np.ndarray,
        clb: np.ndarray,
        cub: np.ndarray,
        model: pin.Model,
    ) -> None:
        """Friction bounds (0 ≤ constraint ≤ ∞)"""
        for frame in node.contact_phase_fnames:
            cub[node.c_friction_ids[frame]] = [None] * 6

            # forces
            if self.max_delta_force <= 0:
                cub[node.c_delta_force_ids[frame]] = [None] * 3
                clb[node.c_delta_force_ids[frame]] = [None] * 3
            else:
                cub[node.c_delta_force_ids[frame]] = self.max_delta_force * np.ones((3,))
                clb[node.c_delta_force_ids[frame]] = -self.max_delta_force * np.ones((3,))
