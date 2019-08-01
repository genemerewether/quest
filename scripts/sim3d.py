import numpy as np
from numpy import sin, cos 
from matplotlib import pyplot as plt
from scipy import integrate
from scipy.spatial.transform import Rotation
from quest_gncpy import LeeControl, WorldParams, MultirotorModel, RigidBodyModel

# TODO make sure you're updating the controller and your variables correctly - like is your R taking into account the changes to w_R_b?
# TODO reimplent your expression lol doofus
# TODO note the difference between feedback setters and command setters

# need

# lee = LeeControl()
 	# initializes class attributes k_x, k_v, k_R, k_omega, sat_x, sat_v, sat_R, sat_omega, sat_yaw, mrModel, invMass, inertia, wParams, x_w, w_R_b, v_b, a_b, omega_b, x_w__des, v_w__des, a_w__des, w_R_b__des, omega_b__des, alpha_b__des, bodyFrame, and rates
# lee.GetAccelAngAccelCommand(a_w__comm, alpha_b__comm)
	# returns -1 if saturated, else 0, modifies a_w__comm and alpha_b_ in place
# lee.GetAngAccelCommand(alpha_b__comm, rpVelOnly, yawVelOnly)
	# returns 0, modifies alpha_b__comm in place, assuming rpVelOnly and yawVelOnly should both be False
# lee.SetAttitudeAngAccelDes(w_q_b__des, omega_b__des, alpha_b__des)
	# returns -1 if saturated, else 0, sets class attributes w_R_b__des, omega_b__des, and alpha_b__des
# lee.SetPositionDes(x_w__des, v_w__des, a_w__des, j_w__des) 
	# returns -1 if saturated, else 0, sets class attributes x_w__des, v_w__des, a_w__des, and j_w__des
# lee.SetGains(k_x, k_v, k_R, k_omega) 
	# returns 0, sets class attributes k_x, k_v, k_R, and k_omega
# lee.SetModel(mrModel) 
	# returns -1 if mass too low, else 0, sets class attributes mrModel, invMass, and inertia
# lee.SetWorldParams(wParams) 
	# returns 0, sets class attribute wParams

'''

# init Lee controller
lee = LeeControl()

# controller gains
k_x = np.array([1.0, 1.0, 1.0])
k_v = np.array([0.1, 0.1, 0.1])
k_R = np.array([1.0, 1.0, 1.0])
k_omega = np.array([0.1, 0.1, 0.1])
lee.SetGains(k_x, k_v, k_R, k_omega)

# quad params
m = 1.0
J = np.array([[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]])
d = 1.0
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

# world params
g = 9.80665
rho = 1.2
wParams = WorldParams()
wParams.gravityMag = g
wParams.atmosphereDensity = rho
lee.SetWorldParams(wParams)

# controller options
atti_ctrl = False # True for attitude control, False for position control
rpVelOnly = False
yawVelOnly = False

# initial conditions
x0 = np.array([0.0, 0.0, 0.5])
v0 = np.array([-0.5, -0.5, 0.0])
a0 = np.array([0.0, 0.0, 0.0])
R0 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
R = Rotation.from_dcm(R0)
q0 = Rotation.as_quat(R)
omega0 = np.array([0.0, 0.0, 0.0])
X0 = np.array([x0[0], x0[1], x0[2], v0[0], v0[1], v0[2], R0[0, 0], R0[0, 1], R0[0, 2], R0[1, 0], R0[1, 1], R0[1, 2], R0[2, 0], R0[2, 1], R0[2, 2], omega0[0], omega0[1], omega0[2]])
# set initial conditions in controller
lee.SetOdometry(x0, q0, v0, omega0)
lee.SetAttitudeAngVel(q0, omega0)
lee.SetPositionLinVelAcc(x0, v0, a0)

# desired state
x_w__des = np.array([0.5, 0.5, 1.0])
v_w__des = np.array([0.0, 0.0, 0.0])
a_w__des = np.array([0.0, 0.0, 0.0])
j_w__des = np.array([0.0, 0.0, 0.0])
w_R_b__des = Rotation.from_dcm(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))
w_q_b__des = Rotation.as_quat(w_R_b__des)
omega_b__des = np.array([0.0, 0.0, 0.0])
alpha_b__des = np.array([0.0, 0.0, 0.0])
yaw_des = 0.0
yawdot_des = 0.0

# set controller desired state
if atti_ctrl:
	lee.SetAttitudeAngAccelDes(w_q_b__des, omega_b__des, alpha_b__des)
else:
	lee.SetPositionDes(x_w__des, v_w__des, a_w__des, j_w__des)
	lee.SetYawDes(yaw_des, yawdot_des)

# lee.SetPositionDes()
# lee.SetPositionAngAccelDes()
# lee.SetVelocityDes()
# lee.SetYawDes()
# lee.SetYawDotDes()
# lee.SetAttitudeDes()
# lee.SetAttitudeAngAccelDes()

# xdes = np.ones((3,))
# vdes = np.ones((3,))
# omegades = np.ones((3,))
# Rdes = Rotation.from_dcm(np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]]))
# qdes = Rotation.as_quat(Rdes)
xdes, _, qdes, vdes, omegades, _ = lee.GetDesired()
R = Rotation.from_quat(qdes)
Rdes = Rotation.as_dcm(R)
# print(xdes, Rdes, vdes, omegades)
# print

# helper functions

def hat(phi):
	return np.array([[0.0, -phi[2], phi[1]], [phi[2], 0.0, -phi[0]], [-phi[1], phi[0], 0.0]])

def vee(psi):
	return np.array([psi[2, 1], psi[0, 2], psi[1, 0]])

def get_state(lee):
	x_w = np.zeros((3,))
	w_R_b = np.zeros((3, 3))
	v_b = np.zeros((3,))
	omega_b = np.zeros((3,))
	a_b = np.zeros((3,))
	lee.GetState(x_w, w_R_b, v_b, omega_b, a_b)
	# print(x_w, w_R_b, v_b, omega_b, a_b)
	return x_w, w_R_b, v_b, omega_b, a_b

# equations of motion plus Lee controller

def simulate_states_over_time(t, X):

	x = np.array([X[0], X[1], X[2]])
	v = np.array([X[3], X[4], X[5]])
	R = np.array([[X[6], X[7], X[8]], [X[9], X[10], X[11]], [X[12], X[13], X[14]]])
	omega = np.array([X[15], X[16], X[17]])

	# print('x', x)
	# print('v', v)
	# print('R', R)
	# print('omega', omega)

	# TODO don't make new variables for these every time, and same elsewhere
	state = get_state(lee)
	x_w = state[0]
	w_R_b = state[1]
	v_b = state[2]
	omega_b = state[3]
	a_b = state[4]

	# attitude tracking error
	e_R = np.zeros((3,))
	e_omega = np.zeros((3,))
	lee.so3Error(e_R, e_omega, rpVelOnly, yawVelOnly)

	# position tracking error
	x_w__err = x_w__des - x_w
	v_w__err = v_w__des - np.dot(w_R_b, v_b) # TODO should this be inverse?

	if atti_ctrl:

		# attitude tracking controller
		alpha_b__comm = np.zeros((3,))
		lee.GetAngAccelCommand(alpha_b__comm, rpVelOnly, yawVelOnly) # TODO this does nothing
		f = np.dot(m * g * np.array([0.0, 0.0, 1.0]), np.dot(R, np.array([0.0, 0.0, 1.0])))
		M = alpha_b__comm

	else:

		# position tracking controller
		a_w__comm = np.zeros((3,))
		alpha_b__comm = np.zeros((3,))
		lee.GetAccelAngAccelCommand(a_w__comm, alpha_b__comm) # TODO this does nothing
		# print(a_w__comm, alpha_b__comm)
		f = m * a_w__comm
		M = alpha_b__comm

	# print('f', f)
	# print('M', M)

	# use equations of motion to update state
	xdot = np.array([X[3], X[4], X[5]])
	vdot = g * np.array([0.0, 0.0, 1.0]) - np.dot(np.dot(f, R), np.array([0.0, 0.0, 1.0])) / m
	Rdot = np.dot(R, hat(omega))
	omegadot = np.dot(np.transpose(J), (M - np.cross(omega, np.dot(J, omega))))
	Xdot = np.array([xdot[0], xdot[1], xdot[2], vdot[0], vdot[1], vdot[2], Rdot[0, 0], Rdot[0, 1], Rdot[0, 2], Rdot[1, 0], Rdot[1, 1], Rdot[1, 2], Rdot[2, 0], Rdot[2, 1], Rdot[2, 2], omegadot[0], omegadot[1], omegadot[2]])

	# print('Xdot', Xdot)

	return Xdot, x_w, x_w__err, v_b, v_w__err, a_b, a_w__comm, w_R_b, e_R, omega_b, e_omega, alpha_b__comm

'''

if __name__ == "__main__":

	# test getters and setters

	lee = LeeControl()

	atti_ctrl = False # True for attitude control, False for position control
	rpVelOnly = False
	yawVelOnly = False

	# initialize worldparams and multirotormodel

	print(lee.GetModel())

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

	print(lee.GetModel())

	print(lee.GetWorldParams())

	g = 9.80665
	rho = 1.2
	wParams = WorldParams()
	wParams.gravityMag = g
	wParams.atmosphereDensity = rho
	lee.SetWorldParams(wParams)

	print(lee.GetWorldParams())

	# set gains

	print(lee.GetGains())

	k_x = np.array([1.0, 1.0, 1.0])
	k_v = np.array([0.1, 0.1, 0.1])
	k_R = np.array([1.0, 1.0, 1.0])
	k_omega = np.array([0.1, 0.1, 0.1])
	lee.SetGains(k_x, k_v, k_R, k_omega)

	print(lee.GetGains())

	# set odometry

	print(lee.GetState())

	x0 = np.array([0.0, 0.0, 0.5])
	v0 = np.array([-0.5, -0.5, 0.0])
	# a0 = np.array([0.0, 0.0, 0.0])
	R0 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
	R = Rotation.from_dcm(R0)
	q0 = Rotation.as_quat(R)
	omega0 = np.array([0.0, 0.0, 0.0])
	lee.SetOdometry(x0, q0, v0, omega0)
	# lee.SetAttitudeAngVel(q0, omega0)
	# lee.SetPositionLinVelAcc(x0, v0, a0)

	print(lee.GetState())

	# set position / attitude desired

	print(lee.GetDesired())

	x_w__des = np.array([0.5, 0.5, 1.0])
	v_w__des = np.array([0.0, 0.0, 0.0])
	a_w__des = np.array([0.0, 0.0, 0.0])
	j_w__des = np.array([0.0, 0.0, 0.0])
	w_R_b__des = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
	R = Rotation.from_dcm(w_R_b__des)
	w_q_b__des = Rotation.as_quat(R)
	omega_b__des = np.array([0.0, 0.0, 0.0])
	alpha_b__des = np.array([0.0, 0.0, 0.0])
	yaw_des = 0.0
	yawdot_des = 0.0

	if atti_ctrl:
		lee.SetAttitudeAngAccelDes(w_q_b__des, omega_b__des, alpha_b__des)
	else:
		lee.SetPositionDes(x_w__des, v_w__des, a_w__des, j_w__des)
		lee.SetYawDes(yaw_des, yawdot_des)

	print(lee.GetDesired())

	# get acceleration / ang acce

	if atti_ctrl:
		alpha_b__comm = lee.GetAngAccelCommand(rpVelOnly, yawVelOnly) # TODO this does nothing
		print(alpha_b__comm)
	else:
		a_w__comm, alpha_b__comm = lee.GetAccelAngAccelCommand() # TODO this does nothing
		print(a_w__comm, alpha_b__comm)

	# t = 0
	# X = np.array([x0[0], x0[1], x0[2], v0[0], v0[1], v0[2], R0[0, 0], R0[0, 1], R0[0, 2], R0[1, 0], R0[1, 1], R0[1, 2], R0[2, 0], R0[2, 1], R0[2, 2], omega0[0], omega0[1], omega0[2]])
	# simulate_states_over_time(t, X)
	
	# integrate equations of motion over time with control
	# tspan = np.linspace(0, 10, 1001)
	# tspan = np.linspace(0, 10, 11)
	# X = np.zeros((len(tspan), len(X0)))
	# X[0, :] = np.reshape(X0, (18,))
	# r = integrate.ode(simulate_states_over_time).set_integrator("dopri5")
	# r.set_initial_value(X0, 0)
	# for i in range(1, tspan.size):
	# 	X[i, :] = r.integrate(tspan[i])
	# 	if not r.successful():
	# 		raise RuntimeError("Could not integrate")

	'''

	# plotting

	Xdot = np.zeros((tspan.size, 18))
	x_w = np.zeros((tspan.size, 3))
	x_w__err = np.zeros((tspan.size, 3))
	v_b = np.zeros((tspan.size, 3))
	v_w__err = np.zeros((tspan.size, 3))
	a_b = np.zeros((tspan.size, 3))
	a_w__comm = np.zeros((tspan.size, 3))
	w_R_b = np.zeros((tspan.size, 9))
	e_R = np.zeros((tspan.size, 3))
	omega_b = np.zeros((tspan.size, 3))
	e_omega = np.zeros((tspan.size, 3))
	alpha_b__comm = np.zeros((tspan.size, 3))
	for i in range(len(tspan)):
		tup = simulate_states_over_time(tspan[i], X[i, :])
		Xdot[i, :] = tup[0]
		x_w[i, :] = tup[1]
		x_w__err[i, :] = tup[2]
		v_b[i, :] = tup[3]
		v_w__err[i, :] = tup[4]
		a_b[i, :] = tup[5]
		a_w__comm[i, :] = tup[6]
		w_R_b[i, :3] = tup[7][:, 0]
		w_R_b[i, 3:6] = tup[7][:, 1]
		w_R_b[i, 6:] = tup[7][:, 2]
		e_R[i, :] = tup[8]
		omega_b[i, :] = tup[9]
		e_omega[i, :] = tup[10]
		alpha_b__comm[i, :] = tup[11]
	x_w__des = x_w__des * np.ones((tspan.size, 3))
	v_w__des = v_w__des * np.ones((tspan.size, 3))
	a_w__des = a_w__des * np.ones((tspan.size, 3))
	R = w_R_b__des.as_dcm()
	R = np.array([R[0, 0], R[0, 1], R[0, 2], R[1, 0], R[1, 1], R[1, 2], R[2, 0], R[2, 1], R[2, 2]])
	w_R_b__des = R * np.ones((tspan.size, 9))
	omega_b__des = omega_b__des * np.ones((tspan.size, 3))
	alpha_b__des = alpha_b__des * np.ones((tspan.size, 3))

	# x tracking (X[0:3] or x_w, x_w__des, x_w__err)
	plt.figure(1)
	plt.plot(tspan, X[:, 0], tspan, x_w__des[:, 0], tspan, X[:, 1], tspan, x_w__des[:, 1], tspan, X[:, 2], tspan, x_w__des[:, 2])
	plt.title('Position Tracking')
	plt.xlabel('time (s)')
	plt.ylabel('displacement (m)')
	plt.legend(['x', 'x_des', 'y', 'y_des', 'z', 'z_des'])

	# xdot tracking (X[3:6] or v_b, v_w__des, v_w__err)
	plt.figure(2)
	plt.plot(tspan, X[:, 3], tspan, v_w__des[:, 0], tspan, X[:, 4], tspan, v_w__des[:, 1], tspan, X[:, 5], tspan, v_w__des[:, 2])
	plt.title('Velocity Tracking')
	plt.xlabel('time (s)')
	plt.ylabel('velocity (m/s)')
	plt.legend(['v_x', 'v_xdes', 'v_y', 'v_ydes', 'v_z', 'v_zdes'])

	# a tracking (Xdot[3:6] or a_b, a_w__des or a_w__comm, difference of some combo of these)
	plt.figure(3)
	plt.plot(tspan, Xdot[:, 3], tspan, a_w__des[:, 0], tspan, Xdot[:, 4], tspan, a_w__des[:, 1], tspan, Xdot[:, 5], tspan, a_w__des[:, 2])
	plt.title('Acceleration Tracking')
	plt.xlabel('time (s)')
	plt.ylabel('acceleration (m/s^2')
	plt.legend(['a_x', 'a_xdes', 'a_y', 'a_ydes', 'a_z', 'a_zdes'])

	# R tracking (X[6:15] or w_R_b, w_R_b__des, e_R)
	plt.figure(4)
	plt.plot(tspan, e_R[:, 0], tspan, e_R[:, 1], tspan, e_R[:, 2])
	plt.title('Orientation Tracking')
	plt.xlabel('time (s)')
	plt.ylabel('orientation error')
	plt.legend(['roll error', 'pitch error', 'yaw error'])

	# omega tracking (X[15:] or omega_b, omega_b__des, e_omega)
	plt.figure(5)
	plt.plot(tspan, X[:, 15], tspan, omega_b__des[:, 0], tspan, X[:, 16], tspan, omega_b__des[:, 1], tspan, X[:, 17], tspan, omega_b__des[:, 2])
	plt.title('Angular Velocity Tracking')
	plt.xlabel('time (s)')
	plt.ylabel('angular velocity (rad/s)')
	plt.legend(['omega_x', 'omega_xdes', 'omega_y', 'omega_ydes', 'omega_z', 'omega_zdes'])

	# alpha tracking (Xdot[15:], alpha_b__comm or alpha_b__des, difference of some combo of these])
	plt.figure(6)
	plt.plot(tspan, Xdot[:, 15], tspan, alpha_b__comm[:, 0], tspan, Xdot[:, 16], tspan, alpha_b__comm[:, 1], tspan, Xdot[:, 17], tspan, alpha_b__comm[:, 2])
	plt.title('Angular Acceleration Tracking')
	plt.xlabel('time (s)')
	plt.ylabel('angular acceleration (rad/s^2')
	plt.legend(['alpha_x', 'alpha_xdes', 'alpha_y', 'alpha_ydes', 'alpha_z', 'alpha_zdes'])

	# x compare (X[0], X[1], X[2], x_w)
	plt.figure(7)
	plt.plot(tspan, X[:, 0], tspan, x_w[:, 0], tspan, X[:, 1], tspan, x_w[:, 1], tspan, X[:, 2], tspan, x_w[:, 2])
	plt.title('Displacement Variables Comparison')
	plt.xlabel('time (s)')
	plt.ylabel('displacement (m)')
	plt.legend(['x_integrate', 'x_getter', 'y_integrate', 'y_getter', 'z_integrate', 'z_getter'])

	# v compare (X[3], X[4], X[5], v_b)
	plt.figure(8)
	plt.plot(tspan, X[:, 3], tspan, v_b[:, 0], tspan, X[:, 4], tspan, v_b[:, 1], tspan, X[:, 5], tspan, v_b[:, 2])
	plt.title('Velocity Variables Comparison')
	plt.xlabel('time (s)')
	plt.ylabel('velocity (m/s)')
	plt.legend(['vx_integrate', 'vx_getter', 'vy_integrate', 'vy_getter', 'vz_integrate', 'vz_getter'])

	# a compare (Xdot[3], Xdot[4], Xdot[5], a_b)
	plt.figure(9)
	plt.plot(tspan, Xdot[:, 3], tspan, a_b[:, 0], tspan, Xdot[:, 4], tspan, a_b[:, 1], tspan, Xdot[:, 5], tspan, a_b[:, 2])
	plt.title('Acceleration Variables Comparison')
	plt.xlabel('time (s)')
	plt.ylabel('acceleration (m/s^2)')
	plt.legend(['ax_xdot', 'ax_getter', 'ay_xdot', 'ay_getter', 'az_xdot', 'az_getter'])

	# a_des compare (a_w__comm, a_w__des)
	plt.figure(10)
	plt.plot(tspan, a_w__comm[:, 0], tspan, a_w__des[:, 0], tspan, a_w__comm[:, 1], tspan, a_w__des[:, 1], tspan, a_w__comm[:, 2], tspan, a_w__des[:, 2])
	plt.title('Desired Acceleration Variables Comparison')
	plt.xlabel('time (s)')
	plt.ylabel('acceleration (m/s^2)')
	plt.legend(['ax_comm', 'ax_des', 'ay_comm', 'ay_des', 'az_comm', 'az_des'])

	# R compare (X[6:15], w_R_b)
	plt.figure(11)
	plt.plot(tspan, X[:, 6], tspan, w_R_b[:, 0], tspan, X[:, 7], tspan, w_R_b[:, 1], tspan, X[:, 8], tspan, w_R_b[:, 2], tspan, X[:, 9], tspan, w_R_b[:, 3], tspan, X[:, 10], tspan, w_R_b[:, 4], tspan, X[:, 11], tspan, w_R_b[:, 5], tspan, X[:, 12], tspan, w_R_b[:, 6], tspan, X[:, 13], tspan, w_R_b[:, 7], tspan, X[:, 14 ], tspan, w_R_b[:, 8])
	plt.title('Orientation Variables Comparison')
	plt.xlabel('time (s)')
	plt.ylabel('rotation matrix term')
	plt.legend(['Rxx_integrate', 'Rxx_getter', 'Rxy_integrate', 'Rxy_getter', 'Rxz_integrate', 'Rxz_getter', 'Ryx_integrate', 'Ryx_getter', 'Ryy_integrate', 'Ryy_getter', 'Ryz_integrate', 'Ryz_getter', 'Rzx_integrate', 'Rzx_getter', 'Rzy_integrate', 'Rzy_getter', 'Rzz_integrate', 'Rzz_getter'])

	# omega compare (X[15], X[16], X[17], omega_b)
	plt.figure(12)
	plt.plot(tspan, X[:, 15], tspan, omega_b[:, 0], tspan, X[:, 16], tspan, omega_b[:, 1], tspan, X[:, 17], tspan, omega_b[:, 2])
	plt.title('Angular Velocity Variables Comparison')
	plt.xlabel('time (s)')
	plt.ylabel('angular velocity (rad/s)')
	plt.legend(['omegax_integrate', 'omegax_getter', 'omegay_integrate', 'omegay_getter', 'omegaz_integrate', 'omegaz_getter'])

	# alpha_des compare (alpha_b__comm, alpha_b__des))
	plt.figure(13)
	plt.plot(tspan, alpha_b__comm[:, 0], tspan, alpha_b__des[:, 0], tspan, alpha_b__comm[:, 1], tspan, alpha_b__des[:, 1], tspan, alpha_b__comm[:, 2], tspan, alpha_b__des[:, 2])
	plt.title('Desired Angular Acceleration Variables Comparison')
	plt.xlabel('time (s)')
	plt.ylabel('angular acceleration (rad/s^2)')
	plt.legend(['alphax_comm', 'alphax_des', 'alphay_comm', 'alphay_des', 'alphaz_comm', 'alphaz_des'])

	plt.show()

	'''