import numpy as np


class AbstractCostFunction:
    """Base class for cost functions with common functionality"""

    def __init__(self, ref: np.array, weight: np.array):
        self.ref = ref.reshape(-1, 1)  # Ensure reference is column vector
        self.Q = weight

    def compute_residual(self, var):
        return var - self.ref

    def compute_cost(self, residual):
        return 0.5 * np.sum(residual.T @ self.Q @ residual)

    def compute_gradient(self, residual):
        return (residual.T @ self.Q).reshape((-1,))

    def obj(self, opt_vect, node, next_node=None):
        raise NotImplementedError

    def grad(self, opt_vect, cost_grad, node, next_node=None):
        raise NotImplementedError
