from copy import copy
from typing import Dict, List, Optional


class Node:
    def __init__(
        self,
        nv: int,
        contact_phase_fnames: List[str],
        contact_fnames: List[str],
    ):
        self.nv = nv
        self.contact_fnames = contact_fnames
        self.contact_phase_fnames = contact_phase_fnames

        self.constraints_list = []
        self.costs_list = []

        # optimization_variables
        self.dt_id = None
        self.q_id = None
        self.vq_id = None
        self.aq_id = None
        self.forces_ids: Dict[str, slice] = {}
        self.contact_pos_ids: Dict[str, slice] = {}

    def init_node_ids(self, v_id, c_id, k):
        self.k = k

        # Initialize variable slices
        self.x_dim = 0
        self._init_state_variables(v_id)
        self._init_contact_variables()

        # Initialize constraint slices
        self.c_dim = 0
        self.c_tf_id = 0
        self._init_dynamics_constraints(c_id)
        self._init_contact_constraints()
        self._init_foot_constraints()

    def _init_state_variables(self, v_id: int) -> None:
        """Initialize state variable slices."""
        self.dt_id = slice(v_id, v_id + 1)
        self.q_id = slice(self.dt_id.stop, self.dt_id.stop + self.nv)
        self.vq_id = slice(self.q_id.stop, self.q_id.stop + self.nv)
        self.aq_id = slice(self.vq_id.stop, self.vq_id.stop + self.nv)
        self.x_dim = 1 + self.nv + self.nv * 2  # dt + q + vq + aq

    def _init_contact_variables(self) -> None:
        """Initialize contact-related variable slices."""
        force_dim = 3
        pos_dim = 3

        prev_slice = copy(self.aq_id)
        for frame in self.contact_phase_fnames:
            self.forces_ids[frame] = slice(prev_slice.stop, prev_slice.stop + force_dim)
            self.contact_pos_ids[frame] = slice(self.forces_ids[frame].stop, self.forces_ids[frame].stop + pos_dim)
            prev_slice = copy(self.contact_pos_ids[frame])
            self.x_dim += force_dim + pos_dim

    def _init_dynamics_constraints(self, c_id: int) -> None:
        """Initialize dynamics constraint slices."""
        # Integration constraints
        self.c_q_integration_id = slice(c_id, c_id + self.nv)
        self.c_vq_integration_id = slice(self.c_q_integration_id.stop, self.c_q_integration_id.stop + self.nv)

        dh_dim = None

        # Dynamics constraints (dimension depends on dynamics type)
        if self.dynamics_type == "centroidal_dynamics":
            dh_dim = 6
        elif self.dynamics_type == "whole_body_dynamics":
            dh_dim = self.nv

        self.c_dh_id = slice(self.c_vq_integration_id.stop, self.c_vq_integration_id.stop + dh_dim)  # Full dynamics

        self.c_dim = self.nv + self.nv + dh_dim

    def _init_contact_constraints(self) -> None:
        """Initialize contact-related constraint slices."""
        fric_dim = 6  # 4 linearized friction cone constraints + max force + positive force
        f_dim = 3
        kinem_dim = 3  # 3D position constraints
        self.c_friction_ids: Dict[str, slice] = {}
        self.c_contact_kinematics_ids: Dict[str, slice] = {}
        self.c_delta_force_ids: Dict[str, slice] = {}

        prev_slice = copy(self.c_dh_id)
        for fname in self.contact_phase_fnames:
            self.c_friction_ids[fname] = slice(prev_slice.stop, prev_slice.stop + fric_dim)
            self.c_contact_kinematics_ids[fname] = slice(
                self.c_friction_ids[fname].stop,
                self.c_friction_ids[fname].stop + kinem_dim,
            )
            self.c_delta_force_ids[fname] = slice(
                self.c_contact_kinematics_ids[fname].stop, self.c_contact_kinematics_ids[fname].stop + f_dim
            )
            prev_slice = copy(self.c_delta_force_ids[fname])
            self.c_dim += fric_dim + kinem_dim + f_dim

    def _init_foot_constraints(self) -> None:
        """Initialize foot-related constraint slices."""
        height_dim = 1  # Foot height
        vel_dim = 3  # Foot velocity
        self.c_z_ids: Dict[str, slice] = {}
        self.c_vel_ids: Dict[str, slice] = {}

        if self.contact_phase_fnames:
            prev_slice = self.c_delta_force_ids[self.contact_phase_fnames[-1]]
        else:
            prev_slice = self.c_dh_id

        for fname in self.contact_fnames:
            self.c_z_ids[fname] = slice(prev_slice.stop, prev_slice.stop + height_dim)
            prev_slice = copy(self.c_z_ids[fname])
            self.c_dim += height_dim

            if fname in self.contact_phase_fnames:
                self.c_vel_ids[fname] = slice(prev_slice.stop, prev_slice.stop + vel_dim)
                prev_slice = copy(self.c_vel_ids[fname])
                self.c_dim += vel_dim
