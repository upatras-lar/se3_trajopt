import numpy as np
import pinocchio as pin

from typing import List, Optional, Protocol
from itertools import product

from nltrajopt.node import Node
from nltrajopt.se3tangent import *


def extend_ids_lists(row_ids, col_ids, slice_rows, slice_cols):
    row_indices = range(*slice_rows.indices(10**6))
    col_indices = range(*slice_cols.indices(10**6))

    for r, c in product(row_indices, col_indices):
        row_ids.extend([r])
        col_ids.extend([c])


class AbstractConstraint(Protocol):
    """Base interface for all constraint types"""

    @property
    def name(self) -> str: ...

    def compute_constraints(
        self,
        node_curr: "Node",
        node_next: Optional["Node"],
        state_vars: np.ndarray,
        constraint_values: np.ndarray,
        model: pin.Model,
        data: pin.Data,
    ) -> None: ...

    def compute_jacobians(
        self,
        node_curr: "Node",
        node_next: Optional["Node"],
        state_vars: np.ndarray,
        jacobian: np.ndarray,
        model: pin.Model,
        data: pin.Data,
    ) -> None: ...

    def get_structure_ids(
        self,
        node_curr: "Node",
        node_next: Optional["Node"],
        row_ids: List[int],
        col_ids: List[int],
    ) -> None: ...

    def get_bounds(
        self,
        node: "Node",
        lb: np.ndarray,
        ub: np.ndarray,
        clb: np.ndarray,
        cub: np.ndarray,
        model: pin.Model,
    ) -> None: ...
