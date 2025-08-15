from nltrajopt.constraint_models.abstract_constraint import *


class TimeConstraint:
    """Handles all time-related constraints including dt bounds and total duration"""

    def __init__(self, min_dt: float = 0.01, max_dt: float = 0.5, total_time: float = 2.0):
        """
        Args:
            min_dt: Minimum time step duration (seconds)
            max_dt: Maximum time step duration (seconds)
            total_time: Total trajectory duration (seconds)
        """
        self.min_dt = min_dt
        self.max_dt = max_dt
        self.total_time = total_time
        self._first_node_processed = False

    @property
    def name(self) -> str:
        return "time"

    def compute_constraints(
        self,
        node_curr: "Node",
        node_next: Optional["Node"],
        state_vars: np.ndarray,
        constraint_values: np.ndarray,
        model: pin.Model,
        data: pin.Data,
    ) -> None:
        """
        Compute time constraints (total duration constraint)
        Stores result in constraint_values[0] for the first node only
        """

        if node_curr.k == 0:
            constraint_values[node_curr.c_tf_id] = 0.0

        constraint_values[node_curr.c_tf_id] += state_vars[node_curr.dt_id][0]

    def compute_jacobians(
        self,
        node_curr: "Node",
        node_next: Optional["Node"],
        state_vars: np.ndarray,
        jacobian: np.ndarray,
        model: pin.Model,
        data: pin.Data,
    ) -> None:
        """
        Compute Jacobians for time constraints
        ∂(sum(dt))/∂dt = 1 for all time variables
        """

        jacobian[node_curr.c_tf_id, node_curr.dt_id.start] = 1.0

    def get_structure_ids(
        self,
        node_curr: "Node",
        node_next: Optional["Node"],
        row_ids: List[int],
        col_ids: List[int],
    ) -> None:
        """
        Generate sparsity pattern for time constraints Adds dependencies between total time constraint (row 0) and all dt variables
        """
        row_ids.append(node_curr.c_tf_id)
        col_ids.append(node_curr.dt_id.start)

    def get_bounds(
        self,
        node: "Node",
        model: pin.Model,
        lb: List[float],
        ub: List[float],
        clb: List[float],
        cub: List[float],
    ) -> None:
        """
        Set bounds for time variables and constraints
        Updates the bound lists in-place
        """
        lb[node.dt_id] = [self.min_dt]
        ub[node.dt_id] = [self.max_dt]

        # Set total duration constraint bounds (only once)
        if node.k == 0 and not self._first_node_processed:
            if self.total_time is not None:
                clb[0] = self.total_time
                cub[0] = self.total_time
            else:
                cub[0] = None
            self._first_node_processed = True
