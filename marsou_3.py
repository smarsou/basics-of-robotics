import sys
import numpy as np
import vtk
import matplotlib.pyplot as plt
import vtk_visualizer as vis
from hocook import hocook
from planecontact import planecontact
from mobrobsim import mobrobsimanimate, set_goal, set_map
from scipy import ndimage
from PIL import Image
from camerasim import CameraSimulator
from br_lectures import *


# from skimage import feature
# from skimage.transform import hough_line, hough_line_peaks

# TASK 0
class tool():
	def __init__(self, scene):
		s = scene	
		self.finger1 = vis.cube(0.04, 0.04, 0.08)
		s.add_actor(self.finger1)
		self.finger2 = vis.cube(0.04, 0.04, 0.08)
		s.add_actor(self.finger2)
		self.palm = vis.cube(0.04, 0.25, 0.04)
		s.add_actor(self.palm)
		self.wrist = vis.cylinder(0.02, 0.0305)
		s.add_actor(self.wrist)
		
	def set_configuration(self, g, TGS):	
		TF1G = np.identity(4)
		TF1G[:3,3] = np.array([0, -0.105, 0])
		TF1S = TGS @ TF1G
		vis.set_pose(self.finger1, TF1S)
		TF2G = np.identity(4)
		TF2G[:3,3] = np.array([0, 0.105, 0])	
		TF2S = TGS @ TF2G
		vis.set_pose(self.finger2, TF2S)
		TPG = np.identity(4)
		TPG[:3,3] = np.array([0, 0, -0.06])
		TPS = TGS @ TPG
		vis.set_pose(self.palm, TPS)
		TWG = np.block([[rotx(np.pi/2), np.array([[0], [0], [-0.09525]])], [np.zeros((1, 3)), 1]])
		TWS = TGS @ TWG
		vis.set_pose(self.wrist, TWS)

def set_floor(s, size):
	floor = vis.cube(size[0], size[1], 0.01)
	TFS = np.identity(4)
	TFS[2,3] = -0.005
	vis.set_pose(floor, TFS)
	s.add_actor(floor)

def task0():
	# Scene.
	s = vis.visualizer()

	# Axes.
	axes = vtk.vtkAxesActor()
	#s.add_actor(axes)
	
	# Floor.
	set_floor(s, [2, 2])

	# Cube.
	cube = vis.cube(0.03, 0.03, 0.03)
	TCS = np.identity(4)
	TCS[:3,3] = np.array([0, 0, 1.5])
	vis.set_pose(cube, TCS)
	s.add_actor(cube)
	
	# Tool.
	TGS = np.identity(4)
	TGS[:3,3] = np.array([0, 0, 1.0])
	tool_ = tool(s)
	tool_.set_configuration(0.015, TGS)
		
	# Render scene.
	s.run()


# TASK 1
class robot():
	def __init__(self, scene):
		# q = np.zeros(6)
		# d = np.array([0, 0, 0, -0.065, 0, 0.11])
		# a = np.array([0, 0.25, 0.25, 0, 0, 0])
		# al = np.array([np.pi/2, 0, 0, np.pi/2, np.pi/2, 0])
		

		q = np.array([np.pi/2, -np.pi/2, np.pi/2, 0, 0, -np.pi/2])
		d = np.array([0, 0, 0, 0.605, 0, 0.3905])
		a = np.array([0, 0.6, 0.0, 0, 0, 0])
		al = np.array([-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])

		self.DH = np.stack((q, d, a, al), 1)
	
		s = scene

		
		# Base.
		self.base = vis.cube(0.28, 0.28, 0.225) #redoslijed osi kao u KS L0
		s.add_actor(self.base)	

		# Link 1.
		self.link1 = vis.cube(0.24, 0.225, 0.24) #redoslijed osi kao u KS L1 
		s.add_actor(self.link1)	
		
		# Link 2.
		self.link2 = vis.cube(0.6, 0.22, 0.22) #redoslijed osi kao u KS L2
		s.add_actor(self.link2)

		# Link 3.
		self.link3 = vis.cube(0.24, 0.24, 0.23) #redoslijed osi kao u KS L3 
		s.add_actor(self.link3)
		
		# # Link 4.
		self.link4 = vis.cube(0.15, 0.49, 0.15) #redoslijed osi kao u KS L4 
		s.add_actor(self.link4)	
		
		# Link 5.
		self.link5 = vis.cube(0.04, 0.04, 0.28) #redoslijed osi kao u KS L5
		s.add_actor(self.link5)		
		
		# Tool.
		self.tool = tool(s)
		
	def set_configuration(self, q, g, T0S):
		d = self.DH[:,1]
		a = self.DH[:,2]
		al = self.DH[:,3]
		
		# Base.
		TB0 = np.identity(4)
		TB0[2,3] = -0.225
		TBS = T0S @ TB0
		vis.set_pose(self.base, TBS)
		

		# Link 1.
		T10 = dh(q[0], d[0], a[0], al[0])
		T1S = T0S @ T10
		TL11 = np.identity(4)
		TL1S = T1S @ TL11
		vis.set_pose(self.link1, TL1S)
		# print(T1S)
		
		# Link 2.
		T21 = dh(q[1], d[1], a[1], al[1])
		# print(T21)
		T2S = T1S @ T21	
		TL22 = np.identity(4)	
		TL22[0,3] = -0.3
		TL22[2,3] = 0.23
		TL2S = T2S @ TL22
		vis.set_pose(self.link2, TL2S)
		# print(T2S)

		# Link 3.
		T32 = dh(q[2], d[2], a[2], al[2])
		T3S = T2S @ T32	
		TL33 = np.identity(4)
		TL3S = T3S @ TL33
		vis.set_pose(self.link3, TL3S)
		# print(T3S)
		
		# Link 4.
		T43 = dh(q[3], d[3], a[3], al[3])
		T4S = T3S @ T43	
		TL44 = np.identity(4)
		TL44[1,3] = 0.245
		TL4S = T4S @ TL44
		vis.set_pose(self.link4, TL4S)
		# print(T4S)
		
		# Link 5.
		T54 = dh(q[4], d[4], a[4], al[4])
		T5S = T4S @ T54	
		TL55 = np.identity(4)
		TL55[2,3] = 0.140
		TL5S = T5S @ TL55
		vis.set_pose(self.link5, TL5S)
		# print(T5S)
		
		# Link 6.
		T65 = dh(q[5], d[5], a[5], al[5])
		T6S = T5S @ T65	
		self.tool.set_configuration(g, T6S)
		# print(T6S)
		
		return T6S	

def task1(q):
	# Scene.
	s = vis.visualizer()

	# Axes.
	axes = vtk.vtkAxesActor()
	s.add_actor(axes)

	# Floor.
	set_floor(s, [1, 1])
	
	# Robot.
	T0S = np.identity(4)
	T0S[2,3] = 0.3375
	rob = robot(s)
	rob.set_configuration(q, 0.03, T0S)
	
	# Render scene.
	s.run()	

# TASK 2

def invkin(DH, T60, solution):
	d = DH[:,1]
	a = DH[:,2]
	al = DH[:,3]
	
	p = T60 @ np.expand_dims(np.array([0, 0, -d[5], 1]), 1)
	x = p[0]
	y = p[1]
	z = p[2]
	r = p[:3].T @ p[:3]
	
	q = np.zeros(6)
	
	# q[2]....
	b1 = d[3]**2+a[1]**2
	b2 = 2*a[1]*d[3]
	s3 = (b1 - r) / b2
	#r = k1 = d4²+a2²+2a2d4s3
	s3 = (r - d[3]**2 - a[1]**2)/(2*a[1]*d[3])
	# print(s3)
	if (s3**2>1):
		q[0]=404
		return q

	if (solution[0]==1):
		q[2]=-np.arcsin(s3)
	else:
		q[2]=np.arcsin(s3)
	
	f1 = d[3]*np.sin(q[2])+a[1]
	f2 = -d[3]*np.cos(q[2])
	f3 = 0
	
	g1 = np.sqrt(x**2+y**2)
	g2 = 0

	A = np.array([ [-f2, -f1],
					[f1, -f2]])
	b = np.array([z, g1])
	cosT2, sinT2= np.linalg.solve(A,b)
	if (solution[1]==0):
		q[1] = np.arctan2(sinT2,cosT2)
	else:
		q[1] = np.pi-np.arctan2(sinT2,cosT2)

	cosT1 = x/g1
	sinT1 = y/g1

	if (solution[2]==0):
		q[0] = np.arctan2(sinT1, cosT1)
	else:
		q[0] =  -np.arctan2(sinT1, cosT1)

	T10 = dh(q[0], d[0], a[0], al[0])
	T21 = dh(q[1], d[1], a[1], al[1])
	T32 = dh(q[2], d[2], a[2], al[2])
	T30 = T10 @ T21 @ T32
	R30 = T30[:3,:3]
	R60 = T60[:3,:3]
	R63 = R30.T @ R60

	q[4] = np.arccos( R63[2,2] )
	sinT5 = np.sin(q[4])
	q[3] = np.arctan2(R63[1,2]/sinT5, R63[0,2]/sinT5)
	q[5] = np.arctan2(R63[2,1]/sinT5, -R63[2,0]/sinT5)

	return q

def task2(solution):
	TTS = np.identity(4)
	TTS[0,3]=1
	TTS[1,3] = -0.2
	TTS[2,3] = 0.9

	# TTS2 = np.identity(4)
	# TTS2[0,3]=1
	# TTS2[1,3] = 0.5
	# TTS2[2,3] = 1
	
	# Scene
	s = vis.visualizer()

	# Floor.
	set_floor(s, [1, 1])

	# Axes.
	axes = vtk.vtkAxesActor()
	s.add_actor(axes)
	
	# Target object.
	target = vis.cube(0.1, 0.1, 0.1) #change size
	vis.set_pose(target, TTS)
	s.add_actor(target)	

	# Robot.
	T0S = np.identity(4)
	T0S[2,3] = 0.3375
	T6T = np.identity(4)
	angle = np.pi
	T6T[:3,:3] = roty(angle/2)
	T60 = np.linalg.inv(T0S) @ TTS @ T6T
	rob = robot(s)
	q = invkin(rob.DH, T60, solution)
	# q = np.zeros(6)
	k=1
	e=np.pi/64
	if (q[0]==404):
		print("No solution for this tool pose ! ")
		return
	# while q[0]==404 and k <= np.pi/e:
	# 	T6T[:3,:3] = roty(angle-k*e)
	# 	print(f"Try with {int(np.pi/e)-k}*PI/{int(np.pi/e)}")
	# 	T60 = np.linalg.inv(T0S) @ TTS @ T6T
	# 	q = invkin(rob.DH, T60, solution)
	# 	k+=1

	# if (k > np.pi/e) and q[0]==404:
	# 	print("Error, no tool orientation found to grab the cube ! Try to decrease the variable \"e\" !")
	# 	return
	# else:
	# 	print("Success")
	rob.set_configuration(q, 0.03, T0S)

	# Render scene.
	s.run()

#Task3

class simulator():
	def __init__(self, robot, Qc):
		self.timer_count = 0
		self.robot = robot
		self.Qc = Qc
		self.trajW = []
		self.T0S = np.identity(4)
		self.T0S[2,3] = 0.05

	def execute(self,iren,event):
		T6S = self.robot.set_configuration(self.Qc[:,self.timer_count % self.Qc.shape[1]], 0.03, self.T0S)
		self.trajW.append(T6S)
		iren.GetRenderWindow().Render()
		self.timer_count += 1

from trajectory_code import trajectory

def task3():
	# Scene
	s = vis.visualizer()

	# Floor.
	set_floor(s, [0, 0])
	
	# Axes.
	axes = vtk.vtkAxesActor()
	s.add_actor(axes)

	# Robot.
	rob = robot(s)
	
	# Robot velocity and acceleration limits.
	dqgr=np.pi*np.ones((1,6))
	ddqgr=25*np.pi*np.ones((1,6))
	
	# Trajectory.
	q_home = np.array([np.pi/2, -np.pi/2, np.pi/2, 0, 0, -np.pi/2])

	Q = trajectory(q_home, rob)
	Ts = 0.01
	Qc, dQc, ddQc, tc = hocook(Q, dqgr, ddqgr, Ts)
	
	# Display trajectory.
	plt.plot(tc,Qc[0,:],tc,Qc[1,:],tc,Qc[2,:],tc,Qc[3,:],tc,Qc[4,:],tc,Qc[5,:])
	plt.show()
	plt.plot(tc,dQc[0,:],tc,dQc[1,:],tc,dQc[2,:],tc,dQc[3,:],tc,dQc[4,:],tc,dQc[5,:])
	plt.show()
	plt.plot(tc,ddQc[0,:],tc,ddQc[1,:],tc,ddQc[2,:],tc,ddQc[3,:],tc,ddQc[4,:],tc,ddQc[5,:])
	plt.show()
	
	# Create animation callback.
	sim = simulator(rob, Qc)
	
	# Start animation.
	s.run(animation_timer_callback=sim.execute)
	
	# Display tool trajectory in 3D.
	trajW = np.array(sim.trajW)
	tool_tip_W = trajW[:,:3,3]
	ax = plt.axes(projection='3d')
	ax.plot3D(tool_tip_W[:,0], tool_tip_W[:,1], tool_tip_W[:,2], 'b')
	plt.show()
	
	# Display plane contact.
	n_board = np.array([1, 0, 0])
	d_board = 1
	board_draw = planecontact(tool_tip_W, n_board, d_board)
	fig, ax = plt.subplots(1, 1)
	ax.plot(-board_draw[:,0], board_draw[:,1], 'b.')
	ax.axis('equal')
	plt.show()
		
	return tool_tip_W
	

def main():
	#task0()
	# task1([0, 0, np.pi/2, 0, np.pi/2, 0])
	# task1([np.pi/2, -np.pi/2, np.pi/2, 0, 0, -np.pi/2])
	# task2([0, 0, 0])
	task3()



if __name__ == '__main__':
    main()
