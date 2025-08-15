from nltrajopt.constraint_models.abstract_constraint import *


class SemiEulerIntegration(AbstractConstraint):
    """State integration constraints (q_next = q ⊕ vΔt)"""

    @property
    def name(self) -> str:
        return "integration"

    def compute_constraints(self, node_curr, node_next, state_vars, c, model, data):
        """Compute integration constraints"""
        if node_next is None:
            return

        q_curr = state_vars[node_curr.q_id]
        v_curr = state_vars[node_curr.vq_id]
        a_curr = state_vars[node_curr.aq_id]
        q_next = state_vars[node_next.q_id]
        v_next = state_vars[node_next.vq_id]

        dt = state_vars[node_curr.dt_id][0]

        # Velocity integration: c_v = v_next - (v + aΔt)
        v_integrated = v_curr + a_curr * dt
        c[node_curr.c_vq_integration_id] = v_next - v_integrated

        # Position integration: c_q = q_next ⊖ (q ⊕ vΔt)
        q_integrated = integrate_tan(model, q_curr, v_integrated * dt)
        c[node_curr.c_q_integration_id] = diff_tan(model, q_next, q_integrated)

    def compute_jacobians(self, node_curr, node_next, w, jac, model, data):
        """Compute integration Jacobians"""
        if node_next is None:
            return

        dt = w[node_curr.dt_id][0]
        q_curr = w[node_curr.q_id]
        q_next = w[node_next.q_id]
        v_curr = w[node_curr.vq_id]
        a_curr = w[node_curr.aq_id]

        v_integrated = v_curr + a_curr * dt
        q_integrated = pin.integrate(model, q_tan2pin(q_curr), v_integrated * dt)

        # Velocity integration derivatives
        jac[node_curr.c_vq_integration_id, node_next.vq_id] = np.eye(node_curr.nv)
        jac[node_curr.c_vq_integration_id, node_curr.vq_id] = -np.eye(node_curr.nv)
        jac[node_curr.c_vq_integration_id, node_curr.aq_id] = -np.eye(node_curr.nv) * dt
        jac[node_curr.c_vq_integration_id, node_curr.dt_id.start] = -a_curr

        # Position integration derivatives
        dDiff = pin.dDifference(model, q_tan2pin(q_next), q_integrated)
        dInt = pin.dIntegrate(model, q_tan2pin(q_curr), v_integrated * dt)

        dDiff[0][:6, :6] = dDiff[0][:6, :6] @ pin.Jexp6(q_next[:6])
        # dDiff[1][:6, :6] = dDiff[1][:6, :6] @ pin.Jexp6(pin.log6_quat(q_integrated[:7]))

        dInt[0][:6, :6] = dInt[0][:6, :6] @ pin.Jexp6(q_curr[:6])
        # q_integrated_tan = q_pin2tan(q_integrated)
        # dInt[0][:6, :6] = pin.Jlog6(pin.exp6(q_integrated_tan[:6])) @ dInt[0][:6, :6] @ pin.Jexp6(w[node_curr.q_id][:6])
        # dInt[1][:6, :6] = pin.Jlog6(pin.exp6(q_integrated_tan[:6])) @ dInt[1][:6, :6]

        jac[node_curr.c_q_integration_id, node_next.q_id] = dDiff[0]
        jac[node_curr.c_q_integration_id, node_curr.q_id] = dDiff[1] @ dInt[0]
        jac[node_curr.c_q_integration_id, node_curr.vq_id] = dDiff[1] @ dInt[1] * dt
        jac[node_curr.c_q_integration_id, node_curr.aq_id] = dDiff[1] @ dInt[1] * dt * dt
        jac[node_curr.c_q_integration_id, node_curr.dt_id.start] = dDiff[1] @ dInt[1] @ (v_curr + 2 * a_curr * dt)

    def get_structure_ids(self, node_curr, node_next, row_ids, col_ids):
        """Integration constraints sparsity pattern"""
        if node_next is None:
            return

        extend_ids_lists(row_ids, col_ids, node_curr.c_q_integration_id, node_next.q_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_q_integration_id, node_curr.q_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_q_integration_id, node_curr.vq_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_q_integration_id, node_curr.aq_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_q_integration_id, node_curr.dt_id)

        extend_ids_lists(row_ids, col_ids, node_curr.c_vq_integration_id, node_next.vq_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_vq_integration_id, node_curr.vq_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_vq_integration_id, node_curr.aq_id)
        extend_ids_lists(row_ids, col_ids, node_curr.c_vq_integration_id, node_curr.dt_id)

    def get_bounds(
        self,
        node: "Node",
        lb: np.ndarray,
        ub: np.ndarray,
        clb: np.ndarray,
        cub: np.ndarray,
        model: pin.Model,
    ) -> None:
        """Integration bounds (exact constraints)"""
        # qs
        lb[node.q_id.start + 6 : node.q_id.stop] = model.lowerPositionLimit[7:]
        ub[node.q_id.start + 6 : node.q_id.stop] = model.upperPositionLimit[7:]

        # vqs
        lb[node.vq_id.start + 6 : node.vq_id.stop] = -model.velocityLimit[6:]
        ub[node.vq_id.start + 6 : node.vq_id.stop] = model.velocityLimit[6:]
