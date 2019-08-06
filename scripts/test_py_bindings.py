import sys
sys.path.append('../quest_gnc/build27')

import numpy as np
from numpy import sin, cos 
from matplotlib import pyplot as plt
from scipy import integrate
from scipy.spatial.transform import Rotation
from quest_gncpy import LeeControl, WorldParams, MultirotorModel, RigidBodyModel

if __name__ == "__main__":

	# test getters and setters

	lee = LeeControl()

	atti_ctrl = False # True for attitude control, False for position control
	rpVelOnly = False
	yawVelOnly = False
	doSaturation = True

	# initialize worldparams and multirotormodel

	# print('model test')
	# print(lee.GetModel())

	m = 1.0
	J = np.array([[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]])
	rigidBody = RigidBodyModel()
	rigidBody.mass = m
	rigidBody.Ixx = J[0, 0]
	rigidBody.Iyy = J[1, 1]
	rigidBody.Izz = J[2, 2]
	rigidBody.Ixy = J[0, 1]
	rigidBody.Ixz = J[0, 2]
	rigidBody.Iyz = J[1, 2]
	mrModel = MultirotorModel()
	mrModel.rigidBody = rigidBody
	lee.SetModel(mrModel)

	# print(lee.GetModel())
	# print

	# print('params test')
	# print(lee.GetWorldParams())

	g = 9.80665
	rho = 1.2
	wParams = WorldParams()
	wParams.gravityMag = g
	wParams.atmosphereDensity = rho
	lee.SetWorldParams(wParams)

	# print(lee.GetWorldParams())
	# print

	# set gains

	# print('gains test')
	# print(lee.GetGains())

	k_x = np.array([1.0, 1.0, 1.0])
	k_v = np.array([0.1, 0.1, 0.1])
	k_R = np.array([1.0, 1.0, 1.0])
	k_omega = np.array([0.1, 0.1, 0.1])
	lee.SetGains(k_x, k_v, k_R, k_omega)

	# print(lee.GetGains())
	# print

	# set odometry

	# print('state test')
	# print(lee.GetState())

	# x0 = np.array([0.0, 0.0, 0.5])
	# v0 = np.array([-0.5, -0.5, 0.0])
	# # a0 = np.array([0.0, 0.0, 0.0])
	# R0 = np.array([[0.5, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
	# R = Rotation.from_dcm(R0)
	# q0 = Rotation.as_quat(R)
	# omega0 = np.array([0.0, 1.0, 0.0])
	# a0 = np.array([1.0, 0.0, 0.0])
	# lee.SetOdometry(x0, q0, v0, omega0)
	# lee.SetAttitudeAngVel(q0, omega0)
	# lee.SetPositionLinVelAcc(x0, v0, a0)

	x0 = np.array([0.0, 0.0, 0.0])
	v0 = np.array([0.0, 0.0, 0.0])
	a0 = np.array([0.0, 0.0, 0.0])
	R0 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
	R = Rotation.from_dcm(R0)
	q0 = Rotation.as_quat(R)
	omega0 = np.array([0.0, 0.0, 0.0])
	X0 = np.array([x0[0], x0[1], x0[2], v0[0], v0[1], v0[2], R0[0, 0], R0[0, 1], R0[0, 2], R0[1, 0], R0[1, 1], R0[1, 2], R0[2, 0], R0[2, 1], R0[2, 2], omega0[0], omega0[1], omega0[2]])
	# set initial conditions in controller
	lee.SetOdometry(x0, q0, v0, omega0)
	# lee.SetAttitudeAngVel(q0, omega0)
	# lee.SetPositionLinVelAcc(x0, v0, a0)

	# print(lee.GetState())
	# print

	# set position / attitude desired

	# TODO either GetDesired or the Desired setters aren't working

	print('desired test')
	print(lee.GetDesired())

	# x_w__des = np.array([0.5, 0.5, 1.0])
	# v_w__des = np.array([0.5, 0.0, 0.5])
	# a_w__des = np.array([0.0, 0.5, 0.0])
	# j_w__des = np.array([0.1, 0.1, 0.1])
	# w_R_b__des = np.array([[1.0, 0.0, 0.0], [0.5, 0.5, 0.0], [0.0, 0.0, 1.0]])
	# R = Rotation.from_dcm(w_R_b__des)
	# w_q_b__des = Rotation.as_quat(R)
	# omega_b__des = np.array([1.0, 0.0, 0.0])
	# alpha_b__des = np.array([0.0, 1.0, 0.0])
	# yaw_des = 0.5
	# yawdot_des = 2.0

	x_w__des = np.array([0.0, 0.0, 0.1])
	v_w__des = np.array([0.0, 0.0, 0.0])
	a_w__des = np.array([0.0, 0.0, 0.0])
	j_w__des = np.array([0.0, 0.0, 0.0])
	w_R_b__des = Rotation.from_dcm(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))
	w_q_b__des = Rotation.as_quat(w_R_b__des)
	omega_b__des = np.array([0.0, 0.0, 0.0])
	alpha_b__des = np.array([0.0, 0.0, 0.0])
	yaw_des = 0.0
	yawdot_des = 0.0

	if atti_ctrl:
		lee.SetAttitudeAngAccelDes(w_q_b__des, omega_b__des, alpha_b__des)
	else:
		lee.SetPositionDes(x_w__des, v_w__des, a_w__des, j_w__des)
		lee.SetYawDes(yaw_des, yawdot_des)

	# lee.SetPositionDes(x_w__des, v_w__des, a_w__des, j_w__des)
	# lee.SetPositionAngAccelDes(x_w__des, v_w__des, alpha_b__des)
	# lee.SetVelocityDes(v_w__des, a_w__des, j_w__des)
	# lee.SetYawDes(yaw_des, yawdot_des)
	# lee.SetYawDotDes(yawdot_des)
	# lee.SetAttitudeDes(w_q_b__des, omega_b__des, rpVelOnly, yawVelOnly, doSaturation)
	# lee.SetAttitudeAngAccelDes(w_q_b__des, omega_b__des, alpha_b__des)

	print(lee.GetDesired())
	print

	# get acceleration / ang acce

	print('command test')

	if atti_ctrl:
		alpha_b__comm, _ = lee.GetAngAccelCommand(rpVelOnly, yawVelOnly)
		print(alpha_b__comm)
	else:
		a_w__comm, alpha_b__comm, _ = lee.GetAccelAngAccelCommand()
		print(a_w__comm, alpha_b__comm)