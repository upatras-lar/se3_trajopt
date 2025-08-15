from nltrajopt.constraint_models.abstract_constraint import *


class WholeBodyDynamics(AbstractConstraint):
    """Whole-body multi-body dynamics constraints"""

    @property
    def name(self) -> str:
        return "whole_body_dynamics"

    def _get_fext(self, nd: Node, model_: pin.Model, data_: pin.Data, x_: np.ndarray) -> List[pin.Force]:
        # Initialize zero forces for all joints
        fext = [pin.Force.Zero() for _ in range(model_.njoints)]

        # Process all contact frames
        for frame in nd.contact_phase_fnames:
            # Get frame information
            frame_id = model_.getFrameId(frame)
            frame_data = model_.frames[frame_id]

            # Get parent joint of this frame
            joint_id = frame_data.parentJoint

            # Get transform from joint to frame
            jMf = frame_data.placement

            # Get force in world frame and convert to joint frame
            f_world = pin.Force(x_[nd.forces_ids[frame]], np.zeros(3))
            f_joint = jMf.act(f_world)

            # Add to joint external force
            fext[joint_id] += f_joint

        return fext

    def compute_constraints(self, node_curr, node_next, state_vars, c, model, data):
        """Compute full multi-body dynamics constraints: M(q)v̇ + C(q,v)v + g(q) = τ + Jᵀf"""
        q = q_tan2pin(state_vars[node_curr.q_id])
        v = state_vars[node_curr.vq_id]
        a = state_vars[node_curr.aq_id]

        # Compute dynamics terms
        pin.forwardKinematics(model, data, q, v, a)
        pin.updateFramePlacements(model, data)

        # Compute constraint: M(q)a + C(q,v)v + g(q) - Jᵀf = τ
        fext = self._get_fext(node_curr, model, data, state_vars)
        c[node_curr.c_dh_id] = pin.rnea(model, data, q, v, a, fext)

    def compute_jacobians(self, node_curr, node_next, w, jac, model, data):
        """Compute Jacobians for full dynamics constraints"""
        q = q_tan2pin(w[node_curr.q_id])
        v = w[node_curr.vq_id]
        a = w[node_curr.aq_id]

        # Compute derivatives of M(q)a + h(q,v)
        pin.forwardKinematics(model, data, q, v, a)
        pin.updateFramePlacements(model, data)

        fext = self._get_fext(node_curr, model, data, w)
        pin.computeRNEADerivatives(model, data, q, v, a, fext)
        dtau_dq = data.dtau_dq
        dtau_dq[:, :6] = dtau_dq[:, :6] @ pin.Jexp6(w[node_curr.q_id][:6])
        jac[node_curr.c_dh_id, node_curr.q_id] = dtau_dq
        jac[node_curr.c_dh_id, node_curr.vq_id] = data.dtau_dv
        jac[node_curr.c_dh_id, node_curr.aq_id] = data.M

        for frame in node_curr.contact_phase_fnames:
            frame_id = model.getFrameId(frame)
            J_f = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)
            jac[node_curr.c_dh_id, node_curr.forces_ids[frame]] = -J_f.T[:, :3]

    def get_structure_ids(self, node_curr, node_next, row_ids, col_ids):
        """Add Jacobian structure for whole-body dynamics"""

        # Dependencies on q, v, a
        extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.q_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.vq_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.aq_id)

        for frame in node_curr.contact_phase_fnames:
            extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.forces_ids[frame])

    def get_bounds(
        self,
        node: "Node",
        lb: np.ndarray,
        ub: np.ndarray,
        clb: np.ndarray,
        cub: np.ndarray,
        model: pin.Model,
    ) -> None:
        """Get bounds for whole-body dynamics constraints"""
        clb[node.c_dh_id.start + 6 : node.c_dh_id.stop] = -model.effortLimit[6:]
        cub[node.c_dh_id.start + 6 : node.c_dh_id.stop] = model.effortLimit[6:]
