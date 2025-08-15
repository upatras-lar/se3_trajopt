import nltrajopt.se3tangent as se3tangent
import nltrajopt.params as pars
import pinocchio as pin
import numpy as np


def rep2pin(q):
    return se3tangent.q_tan2pin(q)


def pin2rep(q):
    return se3tangent.q_pin2tan(q)

# used for warm-starting
def rpy2rep(q, rpy):
    xb = q[:3]
    r, p, y = rpy[0], rpy[1], rpy[2]
    Rb = pin.rpy.rpyToMatrix(r, p, y)

    T = pin.SE3(Rb, xb)
    tau = pin.log6(T).vector

    return np.hstack((tau, q[7:]))
