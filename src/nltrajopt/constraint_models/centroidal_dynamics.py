from nltrajopt.constraint_models.abstract_constraint import *


class CentroidalDynamics(AbstractConstraint):
    """Single rigid body dynamics constraints"""

    @property
    def name(self) -> str:
        return "centroidal_dynamics"

    def compute_constraints(self, node_curr, node_next, state_vars, c, model, data):
        q = q_tan2pin(state_vars[node_curr.q_id])
        v = state_vars[node_curr.vq_id]
        a = state_vars[node_curr.aq_id]
        pin.forwardKinematics(model, data, q, v, a)
        pin.updateFramePlacements(model, data)

        pin.computeCentroidalMomentumTimeVariation(model, data, q, v, a)
        m = pin.computeTotalMass(model)
        g = model.gravity.vector
        com = data.com[0]

        dmomentum = data.dhg.vector - m * g

        for frame in node_curr.contact_phase_fnames:
            dmomentum[:3] -= state_vars[node_curr.forces_ids[frame]]
            dmomentum[3:] -= pin.skew(state_vars[node_curr.contact_pos_ids[frame]] - com) @ state_vars[node_curr.forces_ids[frame]]

        c[node_curr.c_dh_id] = dmomentum

    def compute_jacobians(self, node_curr, node_next, w, jac, model, data):
        q = q_tan2pin(w[node_curr.q_id])
        v = w[node_curr.vq_id]
        a = w[node_curr.aq_id]
        pin.forwardKinematics(model, data, q, v, a)
        pin.updateFramePlacements(model, data)

        # Centroidal dynamics Jacobians
        dhdot_partials = pin.computeCentroidalDynamicsDerivatives(model, data, q, v, a)
        Jcom = pin.jacobianCenterOfMass(model, data, q)
        Jcom[:, :6] = Jcom[:, :6] @ pin.Jexp6(w[node_curr.q_id][:6])

        com = data.com[0]
        dhdot_dq = dhdot_partials[1]
        dhdot_dq[:, :6] = dhdot_dq[:, :6] @ pin.Jexp6(w[node_curr.q_id][:6])

        jac[node_curr.c_dh_id, node_curr.q_id] = dhdot_dq
        jac[node_curr.c_dh_id, node_curr.vq_id] = dhdot_partials[2]
        jac[node_curr.c_dh_id, node_curr.aq_id] = dhdot_partials[3]

        for frame in node_curr.contact_phase_fnames:
            # dhdot/dq -> dcom/dq
            jac[node_curr.c_dh_id.start + 3 : node_curr.c_dh_id.stop, node_curr.q_id] -= -pin.skew(w[node_curr.forces_ids[frame]]).T @ Jcom

            # dhdot/dcontact
            jac[
                node_curr.c_dh_id.start + 3 : node_curr.c_dh_id.stop,
                node_curr.contact_pos_ids[frame],
            ] = -pin.skew(w[node_curr.forces_ids[frame]]).T

            # dhdot/dF
            jac[
                node_curr.c_dh_id.start : node_curr.c_dh_id.start + 3,
                node_curr.forces_ids[frame],
            ] = -np.eye(3)
            jac[
                node_curr.c_dh_id.start + 3 : node_curr.c_dh_id.stop,
                node_curr.forces_ids[frame],
            ] = -pin.skew(w[node_curr.contact_pos_ids[frame]] - com)

    def get_structure_ids(self, node_curr, node_next, row_ids, col_ids):
        """Add Jacobian structure for dynamics constraints"""
        extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.q_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.vq_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.aq_id)

        for frame in node_curr.contact_phase_fnames:
            extend_ids_lists(row_ids, col_ids, node_curr.c_dh_id, node_curr.contact_pos_ids[frame])
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
        pass
