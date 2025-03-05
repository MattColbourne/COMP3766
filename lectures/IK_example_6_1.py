import numpy as np
import modern_robotics as mr
import math

# Example 6.1
# Set i=0
theta0 = (0, 30)
theta0rad = math.radians(theta0[0]), math.radians(theta0[1])
Tsd = np.array([[  -0.5,  -0.866,  0,  0.366],
                [ 0.866,    -0.5,  0,  1.366],
                [     0,       0,  1,      0],
                [     0,       0,  0,      1]])

eomg = 0.001 # rad (or 0.057 deg)
ev = 0.0001 # m (100 microns).

M = np.array([[1, 0, 0, 2],
              [0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

B1 = np.array([0, 0, 1, 0, 2, 0])
B2 = np.array([0, 0, 1, 0, 1, 0])
Blist = np.array([B1,B2]).T
thetalist= theta0rad

Tsb_0 = mr.FKinBody(M, Blist, thetalist)
print("Tsb_0\n", Tsb_0)

# Tbd_i = Tsb_i_inv * Tsd = Tbs_i * Tsd:
Tbs_0 = mr.TransInv(Tsb_0) # Tsb_i_inv
print("Tbs_0\n", Tbs_0)
Tbd_0 = np.dot(Tbs_0, Tsd)
print("Tbd_0\n", Tbd_0)

# [Vb] = log Tbd(thetai):
V_ = mr.MatrixLog6(Tbd_0)
print(V_)

Vb = mr.se3ToVec(V_)
print(Vb)

print("||w||", np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
print("||v||", np.linalg.norm([Vb[3], Vb[4], Vb[5]]))

err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

print("err", err)
# thetai+1 = thetai + Jyb (thetai)Vb.
J_b_inv = np.linalg.pinv(mr.JacobianBody(Blist, thetalist))
theta_ip1 = theta0rad + np.dot(J_b_inv, Vb)

print("theta_ip1", theta_ip1)
print("theta_ip1 deg", math.degrees(theta_ip1[0]), math.degrees(theta_ip1[1]))

# Set i=1
theta1rad = np.copy(theta_ip1)
thetalist= theta1rad

Tsb_1 = mr.FKinBody(M, Blist, thetalist)
print("Tsb_1\n", Tsb_1)

# Tbd_i = Tsb_i_inv * Tsd = Tbs_i * Tsd:
Tbs_1 = mr.TransInv(Tsb_1) # Tsb_i_inv
print("Tbs_1\n", Tbs_1)
Tbd_1 = np.dot(Tbs_1, Tsd)
print("Tbd_1\n", Tbd_1)

# [Vb] = log Tbd(thetai):
V_ = mr.MatrixLog6(Tbd_1)
print(V_)

Vb = mr.se3ToVec(V_)
print(Vb)

print("||w||", np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
print("||v||", np.linalg.norm([Vb[3], Vb[4], Vb[5]]))

err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

print("err", err)
# thetai+1 = thetai + Jyb (thetai)Vb.
J_b_inv = np.linalg.pinv(mr.JacobianBody(Blist, thetalist))
theta_ip1 = theta1rad + np.dot(J_b_inv, Vb)

print("theta_ip1", theta_ip1)
print("theta_ip1 deg", math.degrees(theta_ip1[0]), math.degrees(theta_ip1[1]))

# Set i=2

theta2rad = np.copy(theta_ip1)
thetalist= theta2rad

Tsb_2 = mr.FKinBody(M, Blist, thetalist)
print("Tsb_2\n", Tsb_2)

# Tbd_i = Tsb_i_inv * Tsd = Tbs_i * Tsd:
Tbs_2 = mr.TransInv(Tsb_2) # Tsb_i_inv
print("Tbs_2\n", Tbs_2)
Tbd_2 = np.dot(Tbs_2, Tsd)
print("Tbd_2\n", Tbd_2)

# [Vb] = log Tbd(thetai):
V_ = mr.MatrixLog6(Tbd_2)
print(V_)

Vb = mr.se3ToVec(V_)
print(Vb)

print("||w||", np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
print("||v||", np.linalg.norm([Vb[3], Vb[4], Vb[5]]))

err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

print("err", err)
# thetai+1 = thetai + Jyb (thetai)Vb.
J_b_inv = np.linalg.pinv(mr.JacobianBody(Blist, thetalist))
theta_ip1 = theta2rad + np.dot(J_b_inv, Vb)

print("theta_ip1", theta_ip1)
print("theta_ip1 deg", math.degrees(theta_ip1[0]), math.degrees(theta_ip1[1]))

# Set i = 3

theta3rad = np.copy(theta_ip1)
thetalist= theta3rad

Tsb_3 = mr.FKinBody(M, Blist, thetalist)
print("Tsb_3\n", Tsb_3)

# Tbd_i = Tsb_i_inv * Tsd = Tbs_i * Tsd:
Tbs_3 = mr.TransInv(Tsb_3) # Tsb_i_inv
print("Tbs_3\n", Tbs_3)
Tbd_3 = np.dot(Tbs_3, Tsd)
print("Tbd_3\n", Tbd_3)

# [Vb] = log Tbd(thetai):
V_ = mr.MatrixLog6(Tbd_3)
print(V_)

Vb = mr.se3ToVec(V_)
print(Vb)

print("||w||", np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
print("||v||", np.linalg.norm([Vb[3], Vb[4], Vb[5]]))

err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev

print("err", err)
# thetai+1 = thetai + Jyb (thetai)Vb.
J_b_inv = np.linalg.pinv(mr.JacobianBody(Blist, thetalist))
theta_ip1 = theta3rad + np.dot(J_b_inv, Vb)

print("theta_ip1", theta_ip1)
print("theta_ip1 deg", math.degrees(theta_ip1[0]), math.degrees(theta_ip1[1]))