#!/usr/bin/env python3
"""
Task 12.2: Transform your troops
Ubuntu 20.04 + Python3
"""

import numpy as np
import math
import matplotlib.pyplot as plt

# -------------------------
# Utility functions
# -------------------------
def deg2rad(deg):
    return deg * math.pi / 180.0

def rotation_matrix_from_euler_xyz(thetax_deg, thetay_deg, thetaz_deg):
    """Rotation matrix R = Rz * Ry * Rx"""
    tx = deg2rad(thetax_deg)
    ty = deg2rad(thetay_deg)
    tz = deg2rad(thetaz_deg)

    Rx = np.array([[1, 0, 0],
                   [0, math.cos(tx), -math.sin(tx)],
                   [0, math.sin(tx), math.cos(tx)]])
    Ry = np.array([[math.cos(ty), 0, math.sin(ty)],
                   [0, 1, 0],
                   [-math.sin(ty), 0, math.cos(ty)]])
    Rz = np.array([[math.cos(tz), -math.sin(tz), 0],
                   [math.sin(tz),  math.cos(tz), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def quaternion_from_euler_xyz(thetax_deg, thetay_deg, thetaz_deg):
    """Quaternion (w, x, y, z)"""
    tx = deg2rad(thetax_deg) / 2
    ty = deg2rad(thetay_deg) / 2
    tz = deg2rad(thetaz_deg) / 2

    cx, sx = math.cos(tx), math.sin(tx)
    cy, sy = math.cos(ty), math.sin(ty)
    cz, sz = math.cos(tz), math.sin(tz)

    # q = qz * qy * qx
    qx = (cx, sx, 0.0, 0.0)
    qy = (cy, 0.0, sy, 0.0)
    qz = (cz, 0.0, 0.0, sz)

    def qmult(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw
        )

    return qmult(qz, qmult(qy, qx))

def rotate_points_with_quaternion(points, q):
    """Rotate Nx3 points with quaternion q = (w,x,y,z)"""
    w, x, y, z = q
    q_vec = np.array([x, y, z])
    v = points
    cross1 = np.cross(q_vec, v)
    cross2 = np.cross(q_vec, cross1 + w*v)
    v_rot = v + 2.0*cross2
    return v_rot

# -------------------------
# Transformation functions
# -------------------------
def transform_points_htm(points, tx, ty, tz, thetax, thetay, thetaz):
    R = rotation_matrix_from_euler_xyz(thetax, thetay, thetaz)
    t = np.array([tx, ty, tz])
    return (R @ points.T).T + t

def transform_points_quaternion(points, tx, ty, tz, thetax, thetay, thetaz):
    q = quaternion_from_euler_xyz(thetax, thetay, thetaz)
    rotated = rotate_points_with_quaternion(points, q)
    t = np.array([tx, ty, tz])
    return rotated + t

# -------------------------
# Main demo
# -------------------------
if __name__ == "__main__":
    # Example points
    points = np.array([
        [0.5, 0.5, 0.0],
        [0.5, -0.5, 0.0],
        [-0.5, 0.5, 0.0],
        [-0.5, -0.5, 0.0],
        [0.0, 0.0, 0.0],
        [0.2, 0.1, 0.3]
    ])

    # Example transform
    translation = (1.0, 0.5, 0.2)
    rotation = (30, -15, 45)

    transformed_htm = transform_points_htm(points, *translation, *rotation)
    transformed_quat = transform_points_quaternion(points, *translation, *rotation)

    # Print results
    print("Original:\n", points)
    print("\nTransformed (HTM):\n", transformed_htm)
    print("\nTransformed (Quaternion):\n", transformed_quat)

    # Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(points[:,0], points[:,1], points[:,2], c="blue", label="Original")
    ax.scatter(transformed_htm[:,0], transformed_htm[:,1], transformed_htm[:,2], c="red", marker="^", label="HTM")
    ax.scatter(transformed_quat[:,0], transformed_quat[:,1], transformed_quat[:,2], c="green", marker="s", label="Quaternion")
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.legend()
    plt.show()
