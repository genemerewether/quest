import numpy as np
from numpy import sin, cos 
from matplotlib import pyplot as plt
from scipy import integrate
from scipy.spatial.transform import Rotation
from quest_gncpy import LeeControl, WorldParams, MultirotorModel

# TODO make sure you're updating the controller and your variables correctly - like is your R taking into account the changes to w_R_b?
# TODO make sure you know which version of the flight software you're building the bindings on - your modified one or the original - so you can compare

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

# lee.x_w and lee.x_w__des for x_w__err
# lee.v_w__des, lee.w_R_b, and lee.v_b for v_w__err
# lee.w_R_b and w_R_b__des for e_R
# lee.omega_b__des, lee.omega_b, lee.w_R_b, and lee.w_R_b__des for e_omega

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
mrModel = MultirotorModel(np.array([m, J[0, 0], J[1, 1], J[2, 2], J[0, 1], J[0, 2], J[1, 2]]))
lee.SetModel(mrModel)

# world params
g = 9.80665
rho = 1.2
wParams = WorldParams(np.array([g, rho]))
lee.SetWorldParams(wParams)

# controller options
atti_ctrl = False # True for attitude control, False for position control
rpVelOnly = False
yawVelOnly = False

# initial conditions
x0 = np.array([0.0, 0.0, 0.5])
v0 = np.array([-0.5, -0.5, 0.0])
R0 = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
omega0 = np.array([0.0, 0.0, 0.0])
X0 = np.array([x0[0], x0[1], x0[2], v0[0], v0[1], v0[2], R0[0, 0], R0[0, 1], R0[0, 2], R0[1, 0], R0[1, 1], R0[1, 2], R0[2, 0], R0[2, 1], R0[2, 2], omega0[0], omega0[1], omega0[2]])

# desired state
x_w__des = np.array([0.5, 0.5, 1.0])
v_w__des = np.array([0.0, 0.0, 0.0])
a_w__des = np.array([0.0, 0.0, 0.0])
w_R_b__des = Rotation.from_dcm(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))
w_q_b__des = Rotation.as_quat(w_R_b__des)
omega_b__des = np.array([0.0, 0.0, 0.0])
alpha_b__des = np.array([0.0, 0.0, 0.0])

# set controller desired state
if atti_ctrl:
	lee.SetAttitudeAngAccelDes(w_q_b__des, omega_b__des, alpha_b__des)
else:
	lee.SetPositionDes(x_w__des, v_w__des, a_w__des)

# helper functions

def hat(phi):
	return np.array([[0.0, -phi[2], phi[1]], [phi[2], 0.0, -phi[0]], [-phi[1], phi[0], 0.0]])

def vee(psi):
	return np.array([psi[2, 1], psi[0, 2], psi[1, 0]])

def so3Error(lee, rpVelOnly, yawVelOnly):
	e_R__hat = 0.5 * (np.dot(np.transpose(lee.w_R_b), lee.w_R_b__des) - np.dot(np.transpose(lee.w_R_b__des), lee.w_R_b))
	e_R = vee(e_R__hat)
	if rpVelOnly or yawVelOnly:
		e_omega = lee.omega_b__des - lee.omega_b
	else:
		e_omega = np.dot(np.dot(np.transpose(lee.w_R_b), lee.w_R_b__des), lee.omega_b__des) - lee.omega_b
	if rpVelOnly:
		e_R[0] = 0
		e_R[1] = 0
	if yawVelOnly:
		e_R[2] = 0
	return e_R, e_omega

# equations of motion plus Lee controller

def simulate_states_over_time(t, X):

	x = np.array([X[0], X[1], X[2]])
	v = np.array([X[3], X[4], X[5]])
	R = np.array([[X[6], X[7], X[8]], [X[9], X[10], X[11]], [X[12], X[13], X[14]]])
	omega = np.array([X[15], X[16], X[17]])

	print('x', x)
	print('v', v)
	print('R', R)
	print('omega', omega)

	if atti_ctrl:

		# attitude tracking error
		e_R, e_omega = so3Error(lee, rpVelOnly, yawVelOnly)

		# attitude tracking controller
		lee.GetAngAccelCommand(alpha_b__comm, rpVelOnly, yawVelOnly)
		f = np.dot(m * g * np.array([0.0, 0.0, 1.0]), np.dot(R, np.array([0.0, 0.0, 1.0])))
		M = alpha_b__comm

	else:

		# position tracking error
		x_w__err = lee.x_w__des - lee.x_w
		v_w__err = lee.v_w__des - np.dot(lee.w_R_b, lee.v_b)

		# position tracking controller
		lee.GetAccelAngAccelCommand(a_w__comm, alpha_b__comm)
		f = m * a_w__comm
		M = alpha_b__comm

	print('f', f)
	print('M', M)

	# use equations of motion to update state
	xdot = np.array([X[3], X[4], X[5]])
	vdot = g * np.array([0.0, 0.0, 1.0]) - np.dot(np.dot(f, R), np.array([0.0, 0.0, 1.0])) / m
	Rdot = np.dot(R, hat(omega))
	omegadot = np.dot(np.transpose(J), (M - np.cross(omega, np.dot(J, omega))))
	Xdot = np.array([xdot[0], xdot[1], xdot[2], vdot[0], vdot[1], vdot[2], Rdot[0, 0], Rdot[0, 1], Rdot[0, 2], Rdot[1, 0], Rdot[1, 1], Rdot[1, 2], Rdot[2, 0], Rdot[2, 1], Rdot[2, 2], omegadot[0], omegadot[1], omegadot[2]])

	print('Xdot', Xdot)

	return Xdot, lee.w_R_b, lee.w_R_b__des, lee.omega_b, lee.omega_b__des, e_R, e_omega, lee.x_w_des, lee.x_w, lee.v_w__des, lee.v_b, x_w__err, v_w__err

if __name__ == "__main__":

	t = 0
	X = np.array([x0[0], x0[1], x0[2], v0[0], v0[1], v0[2], R0[0, 0], R0[0, 1], R0[0, 2], R0[1, 0], R0[1, 1], R0[1, 2], R0[2, 0], R0[2, 1], R0[2, 2], omega0[0], omega0[1], omega0[2]])
	simulate_states_over_time(t, X)

	'''
	
	# integrate equations of motion over time with control
	tspan = np.linspace(0, 10, 1001)
	X = np.zeros((len(tspan), len(X0)))
	X[0, :] = np.reshape(X0, (18,))
	r = integrate.ode(simulate_states_over_time).set_integrator("dopri5")
	r.set_initial_value(X0, 0)
	for i in range(1, tspan.size):
		X[i, :] = r.integrate(tspan[i])
		if not r.successful():
			raise RuntimeError("Could not integrate")

	# plotting TODO

	e_R = np.zeros((3, tspan.size))
	omegades = np.zeros((3, tspan.size))
	for i in range(len(tspan)):
		tup = simulate_states_over_time(tspan[i], X[i, :])
		e_R[:, i] = tup[1]
		omegades[:, i] = tup[2]

	plt.figure(1)
	plt.plot(tspan, e_R[0], tspan, e_R[1], tspan, e_R[2])
	plt.xlabel('time (s)')
	plt.ylabel('orientation error')
	plt.legend(['roll error', 'pitch error', 'yaw error'])

	plt.figure(2)
	plt.plot(tspan, X[:, 15], tspan, X[:, 16], tspan, X[:, 17], tspan, omegades[0], tspan, omegades[1], tspan, omegades[2])
	plt.xlabel('time (s)')
	plt.ylabel('angular velocity (rad/s)')
	plt.legend(['omega_x', 'omega_y', 'omega_z' 'omegades_x', 'omegades_y', 'omegades_z'])

	plt.figure(3)
	plt.plot(tspan, X[:, 0], tspan, X[:, 1], tspan, X[:, 2])
	plt.xlabel('time (s)')
	plt.ylabel('position (m)')
	plt.legend(['x', 'y', 'z'])

	plt.show()

	'''