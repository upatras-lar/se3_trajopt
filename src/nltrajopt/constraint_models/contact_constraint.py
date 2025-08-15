from nltrajopt.constraint_models.abstract_constraint import *


class FrictionConstraints(AbstractConstraint):
    """Friction cone constraints (pyramid approximation)"""

    def __init__(self, mu: float = 0.5, max_force: float = -1.0):
        """
        Args:
            mu: Friction coefficient
            bounds_fn: Optional function to compute dynamic bounds
        """
        self.mu = mu
        self.max_force = max_force

    @property
    def name(self) -> str:
        return "friction"

    def compute_constraints(self, node_curr, node_next, state_vars, c, model, data):
        """Compute friction cone constraints (4 linear inequalities per contact + 1 max force)"""
        q = q_tan2pin(state_vars[node_curr.q_id])

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        max_force = 1.0 * pin.computeTotalMass(model) * np.linalg.norm(model.gravity.vector) if self.max_force <= 0.0 else self.max_force
        for frame in node_curr.contact_phase_fnames:
            frame_id = model.getFrameId(frame)
            F_world = (data.oMf[frame_id].rotation @ state_vars[node_curr.forces_ids[frame]]).reshape((3, 1))
            n = np.array([[0.0, 0.0, 1.0]]).T
            t1 = np.array([[1.0, 0.0, 0.0]]).T
            t2 = np.array([[0.0, 1.0, 0.0]]).T

            c[node_curr.c_friction_ids[frame]] = [
                (F_world.T @ (n * self.mu - t1))[0, 0],  # mu_Fz - Fx
                (F_world.T @ (n * self.mu + t1))[0, 0],  # mu_Fz + Fx
                (F_world.T @ (n * self.mu - t2))[0, 0],  # mu_Fz - Fy
                (F_world.T @ (n * self.mu + t2))[0, 0],  # mu_Fz + Fy
                (F_world.T @ n)[0, 0],  # Fz>=0
                max_force - ((F_world.T @ F_world) ** (0.5))[0, 0],  # ||F|| ≤ max_force
            ]

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

            f = w[node_curr.forces_ids[frame]].reshape((3, 1))
            F_world = R @ f
            n = np.array([[0.0, 0.0, 1.0]]).T
            t1 = np.array([[1.0, 0.0, 0.0]]).T
            t2 = np.array([[0.0, 1.0, 0.0]]).T

            tmp = -R @ hat(f) @ Jf[3:, :]

            jac[node_curr.c_friction_ids[frame].start + 0, node_curr.forces_ids[frame]] = (n * self.mu - t1).T @ R
            jac[node_curr.c_friction_ids[frame].start + 1, node_curr.forces_ids[frame]] = (n * self.mu + t1).T @ R
            jac[node_curr.c_friction_ids[frame].start + 2, node_curr.forces_ids[frame]] = (n * self.mu - t2).T @ R
            jac[node_curr.c_friction_ids[frame].start + 3, node_curr.forces_ids[frame]] = (n * self.mu + t2).T @ R
            jac[node_curr.c_friction_ids[frame].start + 4, node_curr.forces_ids[frame]] = n.T @ R
            jac[node_curr.c_friction_ids[frame].start + 5, node_curr.forces_ids[frame]] = (
                -((F_world) / ((F_world.T @ F_world) ** (0.5))).T @ R
            )

            jac[node_curr.c_friction_ids[frame].start + 0, node_curr.q_id] = (n * self.mu - t1).T @ tmp
            jac[node_curr.c_friction_ids[frame].start + 1, node_curr.q_id] = (n * self.mu + t1).T @ tmp
            jac[node_curr.c_friction_ids[frame].start + 2, node_curr.q_id] = (n * self.mu - t2).T @ tmp
            jac[node_curr.c_friction_ids[frame].start + 3, node_curr.q_id] = (n * self.mu + t2).T @ tmp
            jac[node_curr.c_friction_ids[frame].start + 4, node_curr.q_id] = n.T @ tmp
            jac[node_curr.c_friction_ids[frame].start + 5, node_curr.q_id] = -((F_world) / ((F_world.T @ F_world) ** (0.5))).T @ tmp

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


class ContactConstraint(AbstractConstraint):
    """Contact position and velocity constraints"""

    def __init__(self, contact_fname, contact_pos):
        self.contact_fname = contact_fname
        self.contact_pos = contact_pos

    @property
    def name(self) -> str:
        return "contact"

    def compute_constraints(self, node_curr: Node, node_next, state_vars, c, model, data):
        """Compute contact position and velocity constraints"""
        q = q_tan2pin(state_vars[node_curr.q_id])
        vq = state_vars[node_curr.vq_id]
        aq = state_vars[node_curr.aq_id]
        pin.forwardKinematics(model, data, q, vq, aq)
        pin.updateFramePlacements(model, data)

        frame = self.contact_fname
        frame_id = model.getFrameId(frame)

        # Position constraint: p_contact - p_frame = 0
        c[node_curr.c_contact_kinematics_ids[frame]] = data.oMf[frame_id].translation - state_vars[node_curr.contact_pos_ids[frame]]

        # contact_velocity = 0
        c[node_curr.c_vel_ids[frame]] = pin.getFrameVelocity(model, data, frame_id, pin.ReferenceFrame.LOCAL).vector[:3]

    def compute_jacobians(self, node_curr: Node, node_next, w, jac, model, data):
        """Compute contact Jacobians"""

        q = q_tan2pin(w[node_curr.q_id])
        vq = w[node_curr.vq_id]
        aq = w[node_curr.aq_id]
        pin.computeForwardKinematicsDerivatives(model, data, q, vq, aq)
        pin.updateFramePlacements(model, data)

        frame = self.contact_fname
        frame_id = model.getFrameId(frame)

        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL_WORLD_ALIGNED)
        J[:, :6] = J[:, :6] @ pin.Jexp6(w[node_curr.q_id][:6])  # Convert to tangent space

        # Position constraint Jacobians
        jac[node_curr.c_contact_kinematics_ids[frame], node_curr.q_id] = J[:3, :]
        jac[node_curr.c_contact_kinematics_ids[frame], node_curr.contact_pos_ids[frame]] = -np.eye(3)

        # Velocity constraint Jacobians
        dacc = pin.getFrameAccelerationDerivatives(model, data, frame_id, pin.ReferenceFrame.LOCAL)
        jac[node_curr.c_vel_ids[frame], node_curr.q_id] = dacc[0][:3]
        jac[node_curr.c_vel_ids[frame], node_curr.vq_id] = dacc[3][:3]

    def get_structure_ids(self, node_curr, node_next, row_ids, col_ids):
        """Contact constraints sparsity pattern"""

        frame = self.contact_fname

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
            node_curr.c_vel_ids[frame],
            node_curr.q_id,
        )
        extend_ids_lists(
            row_ids,
            col_ids,
            node_curr.c_vel_ids[frame],
            node_curr.vq_id,
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

        frame = self.contact_fname

        pos_id_ = node.contact_pos_ids[frame].start
        kin_id_ = node.c_contact_kinematics_ids[frame].start

        for i, pos in enumerate(self.contact_pos):
            if pos is None:
                cub[kin_id_ + i] = clb[kin_id_ + i] = None
            else:
                lb[pos_id_ + i] = ub[pos_id_ + i] = pos
