import numpy as np
from scipy.linalg import solve

x_disp = -0.06
y_disp = 0.04
z_disp = -2.09

yaw = 0.8745
pitch = 0.4620
roll = -0.1857

a1 = np.array([[np.cos(roll), -np.sin(roll), 0], [np.sin(roll), np.cos(roll), 0], [0, 0, 1]])
a2 = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
a3 = np.array([[1, 0, 0], [0, np.cos(yaw), -np.sin(yaw)], [0, np.sin(yaw), np.cos(yaw)]])

a = np.matmul(np.matmul(a1, a2), a3)

def transform(pos_list):
	x = pos_list[0]
	y = pos_list[1]
	z = pos_list[2]

	c = np.concatenate((a, np.array([[0, 0, 0]])), axis = 0)
	c = np.concatenate((c, np.array([[0, 0, 0, 1]]).T), axis = 1)
	result = np.dot(c, np.array([-y, -z, x, 1]).T)
	result = [-result[1] + x_disp, -result[2] + y_disp, result[0] - z_disp]

	return result

def back_transform(pos_list):
	k1 = -(pos_list[0] - x_disp)
	k2 = -(pos_list[1] - y_disp)
	k0 = pos_list[2] + z_disp

	x = solve(a, np.array([k0, k1, k2]))
	return [x[2], -x[0], -x[1]]