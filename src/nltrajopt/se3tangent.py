import numpy as np
import pinocchio as pin


def q_tan2pin(q_tan: np.ndarray) -> np.ndarray:
    """Convert tangent space representation to Pinocchio quaternion representation."""
    return np.hstack((pin.exp6_quat(q_tan[:6]), q_tan[6:]))


def q_pin2tan(q_pin: np.ndarray) -> np.ndarray:
    """Convert Pinocchio quaternion representation to tangent space representation."""
    return np.hstack((pin.log6_quat(q_pin[:7]).vector, q_pin[7:]))


def diff_tan(model_, q1_, q2_):
    q1 = q_tan2pin(q1_)
    q2 = q_tan2pin(q2_)
    return pin.difference(model_, q1, q2)


def integrate_tan(model_, q_, vq_):
    q_pin = q_tan2pin(q_)
    q_new = pin.integrate(model_, q_pin, vq_)
    return q_pin2tan(q_new)

def hat(vec):
    v = vec.reshape((3,))
    return np.array([
        [0., -v[2], v[1]],
        [v[2], 0., -v[0]],
        [-v[1], v[0], 0.]
    ])
