"""
================================================================
 jacobian.py - Jacobian dong hoc & diem ky di (singularity)
----------------------------------------------------------------
 - position_jacobian(): Jacobian vi tri J_v (3x4) cua dau mo han
                        (tinh bang sai phan so - tan dung FK san co)
 - manipulability()   : w = sqrt(det(J_v . J_v^T))
                        w lon  -> robot kheo leo
                        w ~ 0  -> gan diem ky di (singularity)
================================================================
"""

import numpy as np


def position_jacobian(robot, q, h=1e-6):
    """
    Tinh Jacobian vi tri J_v (3x4) bang sai phan trung tam.

    Y tuong: J_v[:, j] = thay doi cua (x,y,z) khi nhich khop j mot luong nho h.
        cot j = ( FK(q + h o khop j) - FK(q - h o khop j) ) / (2h)

    robot : doi tuong Robot
    q     : list 4 phan tu [q1,q2,q3,q4], moi phan tu la mang numpy (N,) hoac scalar
    Tra ve: mang J co shape (..., 3, 4)
    """
    q = [np.asarray(qi) * 1.0 for qi in q]      # ep ve float/mang
    shape = q[0].shape
    J = np.zeros(shape + (3, 4))

    for j in range(4):                          # lan luot tung khop
        qp = [qi.copy() for qi in q]; qp[j] = qp[j] + h    # nhich len
        qm = [qi.copy() for qi in q]; qm[j] = qm[j] - h    # nhich xuong

        xp, yp, zp = robot.forward_kinematics(qp)
        xm, ym, zm = robot.forward_kinematics(qm)

        J[..., 0, j] = (xp - xm) / (2 * h)      # d(x)/d(qj)
        J[..., 1, j] = (yp - ym) / (2 * h)      # d(y)/d(qj)
        J[..., 2, j] = (zp - zm) / (2 * h)      # d(z)/d(qj)

    return J


def manipulability(robot, q):
    """
    Chi so manipulability w = sqrt(det(J_v . J_v^T)).
    J_v la 3x4 (khong vuong) nen khong dung det(J) truc tiep;
    J_v . J_v^T la 3x3 -> lay det duoc.

    Tra ve: mang w (cung kich thuoc voi so mau).
            w cang nho cang gan ky di. w = 0 la ky di.
    """
    J = position_jacobian(robot, q)             # (..., 3, 4)
    JJt = J @ np.swapaxes(J, -1, -2)            # (..., 3, 3)
    det = np.linalg.det(JJt)
    det = np.clip(det, 0.0, None)               # chong sai so am nho
    return np.sqrt(det)