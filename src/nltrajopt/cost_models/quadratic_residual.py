import numpy as np
import pinocchio as pin
import nltrajopt.params as pars
from .abstract_cost import AbstractCostFunction


class ConfigurationCost(AbstractCostFunction):
    def obj(self, opt_vect, node, next_node):
        var = opt_vect[node.q_id.start + pars.SPACE_NQ : node.q_id.stop].reshape(-1, 1)
        res = self.compute_residual(var)
        cost = self.compute_cost(res)
        return cost

    def grad(self, opt_vect, cost_grad, node, next_node):
        var = opt_vect[node.q_id.start + pars.SPACE_NQ : node.q_id.stop].reshape((-1, 1))
        res = self.compute_residual(var)
        jac = self.compute_gradient(res)
        cost_grad[node.q_id.start + pars.SPACE_NQ : node.q_id.stop] += jac


class JointVelocityCost(AbstractCostFunction):
    def obj(self, opt_vect, node, next_node):
        var = opt_vect[node.vq_id.start + 6 : node.vq_id.stop].reshape((-1, 1))
        res = self.compute_residual(var)
        cost = self.compute_cost(res)
        return cost

    def grad(self, opt_vect, cost_grad, node, next_node):
        var = opt_vect[node.vq_id.start + 6 : node.vq_id.stop].reshape((-1, 1))
        res = self.compute_residual(var)
        jac = self.compute_gradient(res)
        cost_grad[node.vq_id.start + 6 : node.vq_id.stop] += jac


class JointAccelerationCost(AbstractCostFunction):
    def obj(self, opt_vect, node, next_node):
        var = opt_vect[node.aq_id.start + 6 : node.aq_id.stop].reshape(-1, 1)
        res = self.compute_residual(var)
        cost = self.compute_cost(res)
        return cost

    def grad(self, opt_vect, cost_grad, node, next_node):
        var = opt_vect[node.aq_id.start + 6 : node.aq_id.stop].reshape((-1, 1))
        res = self.compute_residual(var)
        jac = self.compute_gradient(res)
        cost_grad[node.aq_id.start + 6 : node.aq_id.stop] += jac


class ForceCost(AbstractCostFunction):
    def obj(self, opt_vect, node, next_node):
        cost = 0
        for frame in node.contact_phase_fnames:
            f = opt_vect[node.forces_ids[frame]]
            cost += self.compute_cost(f)
        return cost

    def grad(self, opt_vect, cost_grad, node, next_node):
        for frame in node.contact_phase_fnames:
            f = opt_vect[node.forces_ids[frame]]
            jac = self.compute_gradient(f)
            cost_grad[node.forces_ids[frame]] = jac
