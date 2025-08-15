import numpy as np
import pinocchio as pin
import nltrajopt.se3tangent as se3tan


def finite_diff(f, z, M, eps=1e-6):
    N = z.shape[0]
    jac = np.zeros((M, N))
    for i in range(N):
        zp = np.copy(z)
        zp[i] += eps
        zm = np.copy(z)
        zm[i] -= eps
        jac[:, i : i + 1] = ((f(zp) - f(zm)) / (2.0 * eps)).reshape((-1, 1))
    return jac


ori = np.random.random((3, 1))
# ori = np.zeros((3, 1))
z_axis = np.array([[0, 0, 1]]).T
R_base_ori = np.eye(3)
axis_ref = np.array([[0, 0, -1]]).T


def cost(ori_):
    # res = pin.log3(pin.exp3(ori_) @ pin.exp(base_zori_ref))

    R = pin.exp3(ori_)
    res = R @ z_axis - axis_ref

    return (res.T @ R_base_ori @ res)[0, 0]


print(cost(ori))


def grad(ori_):
    R = pin.exp3(ori_)
    res = R @ z_axis - axis_ref

    J_r = pin.Jexp3(ori_)
    J_res = -R @ pin.skew(z_axis) @ J_r

    grad = 2 * res.T @ R_base_ori @ J_res

    return grad


Jac = grad(ori)
JacDiff = finite_diff(cost, ori, 1)


print(Jac)
print(JacDiff)

print(np.allclose(Jac, JacDiff))
