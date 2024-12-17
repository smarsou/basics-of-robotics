import sys
import numpy as np
import vtk
import matplotlib.pyplot as plt
import vtk_visualizer as vis
#import br_lectures as br
from hocook import hocook
from planecontact import planecontact
from mobrobsim import mobrobsimanimate, set_goal, set_map
from scipy import ndimage
from PIL import Image
from camerasim import CameraSimulator
from skimage import feature
from skimage.transform import hough_line, hough_line_peaks

# TASK 0
class tool():
	def __init__(self, scene):
		s = scene	
		self.finger1 = vis.cube(0.02, 0.01, 0.05)
		s.add_actor(self.finger1)
		self.finger2 = vis.cube(0.02, 0.01, 0.05)
		s.add_actor(self.finger2)
		self.palm = vis.cube(0.03, 0.08, 0.03)
		s.add_actor(self.palm)
		self.wrist = vis.cylinder(0.015, 0.04)
		s.add_actor(self.wrist)
		
	def set_configuration(self, g, TGS):	
		TF1G = np.identity(4)
		TF1G[:3,3] = np.array([0, -0.5*g-0.005, -0.025])
		TF1S = TGS @ TF1G
		vis.set_pose(self.finger1, TF1S)
		TF2G = np.identity(4)
		TF2G[:3,3] = np.array([0, 0.5*g+0.005, -0.025])	
		TF2S = TGS @ TF2G
		vis.set_pose(self.finger2, TF2S)
		TPG = np.identity(4)
		TPG[:3,3] = np.array([0, 0, -0.065])
		TPS = TGS @ TPG
		vis.set_pose(self.palm, TPS)
		TWG = np.block([[rotx(np.pi/2), np.array([[0], [0], [-0.1]])], [np.zeros((1, 3)), 1]])
		TWS = TGS @ TWG
		vis.set_pose(self.wrist, TWS)

def rotx(q):
	c = np.cos(q)
	s = np.sin(q)
	return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
	
def roty(q):
	c = np.cos(q)
	s = np.sin(q)
	return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotz(q):
	c = np.cos(q)
	s = np.sin(q)
	return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def set_floor(s, size):
	floor = vis.cube(size[0], size[1], 0.01)
	TFS = np.identity(4)
	TFS[2,3] = -0.005
	vis.set_pose(floor, TFS)
	s.add_actor(floor)

def task0():
	# Scene
	s = vis.visualizer()

	# Floor.
	set_floor(s, [1, 1])

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
		q = np.zeros(6)
		d = np.array([0, 0, 0, -0.065, 0, 0.11])
		a = np.array([0, 0.25, 0.25, 0, 0, 0])
		al = np.array([np.pi/2, 0, 0, np.pi/2, np.pi/2, 0])
		self.DH = np.stack((q, d, a, al), 1)
	
		s = scene
		
		# Base.
		self.base = vis.cylinder(0.025, 0.05)
		s.add_actor(self.base)	

		# Link 1.
		self.link1 = vis.cylinder(0.025, 0.05)
		s.add_actor(self.link1)	
		
		# Link 2.
		self.link2 = vis.cube(0.3, 0.05, 0.05)
		s.add_actor(self.link2)

		# Link 3.
		self.link3 = vis.cube(0.3, 0.05, 0.05)
		s.add_actor(self.link3)
		
		# Link 4.
		self.link4 = vis.cylinder(0.015, 0.04)
		s.add_actor(self.link4)	
		
		# Link 5.
		self.link5 = vis.cylinder(0.02, 0.04)
		s.add_actor(self.link5)		
		
		# Tool.
		self.tool = tool(s)
		
	def set_configuration(self, q, g, T0S):
		d = self.DH[:,1]
		a = self.DH[:,2]
		al = self.DH[:,3]
		
		# Base.
		TB0 = np.identity(4)
		TB0[:3,:3] = rotx(np.pi/2)
		TB0[2,3] = -0.025
		TBS = T0S @ TB0
		vis.set_pose(self.base, TBS)

		# Link 1.
		T10 = dh(q[0], d[0], a[0], al[0])
		T1S = T0S @ T10
		TL11 = np.identity(4)
		TL11[:3,:3] = rotx(np.pi/2)	
		TL1S = T1S @ TL11
		vis.set_pose(self.link1, TL1S)
		
		# Link 2.
		T21 = dh(q[1], d[1], a[1], al[1])
		T2S = T1S @ T21	
		TL22 = np.identity(4)
		TL22[0,3] = -0.12
		TL22[2,3] = 0.05
		TL2S = T2S @ TL22
		vis.set_pose(self.link2, TL2S)

		# Link 3.
		T32 = dh(q[2], d[2], a[2], al[2])
		T3S = T2S @ T32	
		TL33 = np.identity(4)
		TL33[0,3] = -0.125
		TL3S = T3S @ TL33
		vis.set_pose(self.link3, TL3S)
		
		# Link 4.
		T43 = dh(q[3], d[3], a[3], al[3])
		T4S = T3S @ T43	
		TL44 = np.identity(4)
		TL44[1,3] = 0.02
		TL4S = T4S @ TL44
		vis.set_pose(self.link4, TL4S)
		
		# Link 5.
		T54 = dh(q[4], d[4], a[4], al[4])
		T5S = T4S @ T54	
		TL55 = np.identity(4)
		TL5S = T5S @ TL55
		vis.set_pose(self.link5, TL5S)
		
		# Link 6.
		T65 = dh(q[5], d[5], a[5], al[5])
		T6S = T5S @ T65	
		self.tool.set_configuration(g, T6S)
		
		return T6S	

def dh(q, d, a, al):
	cq = np.cos(q)
	sq = np.sin(q)
	ca = np.cos(al)
	sa = np.sin(al)
	T = np.array([[cq, -sq*ca, sq*sa, a*cq],
		[sq, cq*ca, -cq*sa, a*sq],
		[0, sa, ca, d],
		[0, 0, 0, 1]])
	return T

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
	T0S[2,3] = 0.05
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
	
	q[2] = np.arccos((r - a[2]**2 - d[3]**2 - a[1]**2) / (2*a[1]*a[2]))
	if solution[0] == 1:
		q[2] = -q[2]
		
	c3 = np.cos(q[2])
	s3 = np.sin(q[2])
	f1 = a[2]*c3 + a[1]
	
	f2 = a[2]*s3
	
	f3 = d[3]
	
	A = np.sqrt(f1**2+f2**2)
	phi = np.arctan2(f2, f1)
	if solution[1] == 0:
		q[1] = np.arcsin(z/A) - phi
	else:
		q[1] = np.pi - np.arcsin(z/A) - phi
	

	c2 = np.cos(q[1])
	s2 = np.sin(q[1])
	g1 = c2*f1 - s2*f2
		
	g2 = -f3
	
	c1 = g1*x + g2*y
	s1 = -g2*x + g1*y
	q[0] = np.arctan2(s1, c1)
	
	T10 = dh(q[0], d[0], a[0], al[0])
	T21 = dh(q[1], d[1], a[1], al[1])
	T32 = dh(q[2], d[2], a[2], al[2])
	T30 = T10 @ T21 @ T32
	R30 = T30[:3,:3]
	R60 = T60[:3,:3]
	R63 = R30.T @ R60
	
	c5 = -R63[2,2]
	q[4] = np.arccos(c5)
	if solution[2] == 1:
		q[4] = -q[4]
	s5 = np.sin(q[4])
	if np.abs(s5) > 1e-10:
		q[3] = np.arctan2(R63[1,2]/s5, R63[0,2]/s5)
		q[5] = np.arctan2(-R63[2,1]/s5, R63[2,0]/s5)
	else:
		c46 = R63[0,0]
		s46 = R63[0,1]
		q46 = np.arctan2(s46, c46)
		q[3] = q46
		q[5] = 0
	
	return q

def task2(solution):
	TTS = np.identity(4)
	TTS[0,3]=0.3
	TTS[2,3] = 0.1
	
	# Scene
	s = vis.visualizer()

	# Floor.
	set_floor(s, [1, 1])
	
	# Target object.
	target = vis.cube(0.03, 0.03, 0.03) #change size
	vis.set_pose(target, TTS)
	s.add_actor(target)	

	# Robot.
	T0S = np.identity(4)
	T0S[2,3] = 0.05
	T6T = np.identity(4)
	T6T[:3,:3] = roty(np.pi)
	T60 = np.linalg.inv(T0S) @ TTS @ T6T
	rob = robot(s)
	q = invkin(rob.DH, T60, solution)
	rob.set_configuration(q, 0.03, T0S)
	
	# Render scene.
	s.run()

# TASK 3

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

def task3():
	# Scene
	s = vis.visualizer()

	# Floor.
	set_floor(s, [1, 1])
	
	# Robot.
	rob = robot(s)
	
	# Robot velocity and acceleration limits.
	dqgr=np.pi*np.ones((1,6))
	ddqgr=10*np.pi*np.ones((1,6))
	
	# Trajectory.
	q_home = np.array([-np.pi/2, np.pi/2, 0, 0, 0, 0])
	T60_1 = np.identity(4)
	T60_1[:3,:3] = roty(np.pi)
	T60_1[:3,3] = np.array([0.25, -0.2, 0.2])
	#T60_1[:3,3] = np.array([0.25, -0.2, 0.2])
	q1 = invkin(rob.DH,T60_1,[1, 0, 0])
	T60_2 = T60_1.copy()
	T60_2[:3,3] = np.array([0.25, -0.2, 0.02])
	#T60_1[:3,3] = np.array([0.25, -0.2, 0.019])
	q2 = invkin(rob.DH,T60_2,[1, 0, 0])
	T60_3 = T60_1.copy()
	T60_3[:3,3] = np.array([0.25, 0.2, 0.02])
	#T60_1[:3,3] = np.array([0.25, -0.2, 0.019])
	q3 = invkin(rob.DH,T60_3,[1, 0, 0])
	T60_4 = T60_1.copy()
	T60_4[:3,3] = np.array([0.25, 0.2, 0.2])
	#T60_1[:3,3] = np.array([0.25, -0.2, 0.2])
	q4 = invkin(rob.DH,T60_4,[1, 0, 0])
	Q = np.stack((q_home, q1, q2, q3, q4, q_home), 1)
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
	n_board = np.array([0, 0, 1])
	d_board = 0.02
	board_draw = planecontact(tool_tip_W, n_board, d_board)
	fig, ax = plt.subplots(1, 1)
	ax.plot(board_draw[:,0], board_draw[:,1], 'b.')
	ax.axis('equal')
	plt.show()
		
	return tool_tip_W
	
# TASK 4

def imgproc(RGB, camera, camera_height, box_size):
	img_w = camera['img_size'][0]
	img_h = camera['img_size'][1]
	box_w = box_size[0]
	box_h = box_size[1]
	plt.imshow(RGB)
	E=feature.canny(RGB[:,:,0])	
	H, theta, d = hough_line(E)	
	counter = 0
	for _, angle, dist in zip(*hough_line_peaks(H, theta, d, min_distance=20, min_angle=20, threshold=50)):
		#print('angle=%f, dist=%f' % (angle, dist))
		cs = np.cos(angle)
		sn = np.sin(angle)
		if np.abs(cs) >= np.abs(sn):
			v = np.array([0, img_h-1])
			u = (dist - sn * v) / cs
		else:
			u = np.array([0, img_w-1])
			v = (dist - cs * u) / sn
		plt.plot(u, v, 'g')				
		if counter == 0:
			angle1 = angle
			angle2 = None
			d1 = dist
			#print('angle1=', angle1)
		else:
			angle_diff = np.abs(angle - angle1)
			if angle_diff > np.pi/2:
				dist = -dist
				if angle > 0.0:
					angle -= np.pi
				else:
					angle += np.pi
			if angle_diff < np.pi/4:
				dd = np.abs(dist - d1)
				if dd > 0.5 * box_h:
					d1 = 0.5 * (d1 + dist)
					dd1 = dd
					#print('d1=%f dd1=%f' % (d1, dd1))
			elif angle2 is None:
				angle2 = angle
				d2 = dist
				#print('angle2=', angle2)
			else:
				angle2_diff = np.abs(angle - angle2)
				if angle2_diff > np.pi/2:
					angle2_diff = np.pi - angle2_diff
					dist = -dist
					if angle > 0.0:
						angle -= np.pi
					else:
						angle += np.pi					
				if angle2_diff < np.pi/4:
					dd = np.abs(dist - d2)
					if dd > 0.5 * box_h:
						d2 = 0.5 * (d2 + dist)
						dd2 = dd
						#print('d2=%f dd2=%f' % (d2, dd2))
		counter += 1
	n1 = np.array([np.cos(angle1), np.sin(angle1)])
	n2 = np.array([np.cos(angle2), np.sin(angle2)])	
	N = np.stack((n1, n2), 0)
	D = np.array([d1, d2])
	t_img = np.linalg.solve(N,D)	
	if dd1 > dd2:
		x = n1
		alpha = angle1
	else:
		x = n2
		alpha = angle2
	plt.plot(t_img[0],t_img[1],'r+')
	x_axis = np.stack((t_img, t_img + x * 150), 0)
	plt.plot(x_axis[:,0], x_axis[:,1], 'r-')	
	plt.show()
	t = camera_height * np.ones(3)
	t[:2] = (t_img - np.array(camera['principal_point'])) * camera_height / camera['focal_length']
		
	return alpha, t

	

def task4():
	camera = {
		'img_size': [640, 480],
		'focal_length': 1000,
		'principal_point': [320, 240],
		}
	camera_height = 0.5
	box_w = 0.1
	box_h = 0.05
	
	camsim = CameraSimulator(camera, camera_height, [box_w, box_h], 0.05)

	RGB = camsim.get_image()
	
	alpha, t = imgproc(RGB, camera, camera_height, [box_w, box_h])
	
	e_alpha, e_t = camsim.evaluate(alpha, t)
	
	print('orientation error: %f deg position error: %f mm' % (np.rad2deg(e_alpha), 1000 * e_t))	


# TASK 5
	
def task5(map_file_name, start, goal, map_resolution, robot_radius):
	# Load map as image.
	map = Image.open(map_file_name)
	map = map.resize((250,250))
	map = np.array(map)
	map = map[:,:,0]

	# Create binary map.
	map_mean = np.mean(map)
	binary_map = np.ones(map.shape).astype('int32')
	binary_map[map < map_mean] = 0
	plt.imshow(binary_map)
	plt.show()
	
	# Dilate obstacles.
	edt = ndimage.morphology.distance_transform_edt(binary_map)
	plt.imshow(edt)
	plt.show()
	free_space = edt > robot_radius / map_resolution
	if free_space[start[0],start[1]] and free_space[goal[0],goal[1]]:
		print("start and goal values are valids")
	else:
		print("Error: start and/or goal value is/are not valid(s) ")
		return
	# free_space[start[0]][start[1]],free_space[goal[0]][goal[1]],=0,0
	plt.imshow(free_space)
	plt.show()

	# Compute numerical navigation function.
	print("Compute navigation")
	visited = np.zeros(free_space.shape)
	path = np.zeros((free_space.shape[0],free_space.shape[1],2)).tolist()
	goal = (goal[0],goal[1])
	pile=[(start[0],start[1])]
	count = 1
	visited[start[0],start[1]]=count
	while (goal not in pile):
		# print(len(pile))
		toAdd = []
		count+=1
		for p in pile:
			for (i,j) in [(p[0]+1,p[1]+0),(p[0]+0,p[1]+1),(p[0]-1,p[1]+0),(p[0]+0,p[1]-1)]:
				if i>=0 and i<free_space.shape[0] and j>=0 and j<free_space.shape[1]:
					if free_space[i,j] and visited[i,j]==0:
						toAdd.append((i,j))
						path[i][j]=p
						visited[i,j]=count
			pile.remove(p)
		for p in toAdd:
			pile.append(p)
		
	print("Show the result of the computation")
	plt.imshow(visited)
	plt.show()

	print("Restore the path")
	p = goal
	c = visited[p[0],p[1]]
	path = [p]
	while c!=1:
		# print(c)
		if visited[p[0]+1,p[1]+0]<c and visited[p[0]+1,p[1]+0] !=0:
			c = visited[p[0]+1,p[1]+0]
			pmax = [p[0]+1,p[1]+0]
		if visited[p[0]+0,p[1]+1]<c and visited[p[0]+0,p[1]+1] != 0:
			c = visited[p[0]+0,p[1]+1]
			pmax = [p[0]+0,p[1]+1]
		if visited[p[0]-1,p[1]+0]<c and visited[p[0]-1,p[1]+0] != 0:
			c = visited[p[0]-1,p[1]+0]
			pmax = [p[0]-1,p[1]+0]
		if visited[p[0]+0,p[1]-1]<c and visited[p[0]+0,p[1]-1] != 0:
			c = visited[p[0]+0,p[1]-1]
			pmax= [p[0]+0,p[1]-1]
		p = pmax
		path.append(p)
	# print("Path found: c=" + str(c))
	# print(p)
	
	print("Show the path")
	map_with_path = np.ones(map.shape)
	map_with_path[map < map_mean] = 0
	path = np.array(path)
	map_with_path[path[:,0], path[:,1]] = 0.5
	plt.imshow(map_with_path)
	plt.show()
	path[:,1] = map.shape[0] - path[:,1]
	path = path.astype('float64') * map_resolution
	return path
				
def main():
	#task0()
	#task1([0, np.pi/2, -np.pi/2, 0, 0, 0])
	#task2([0, 1, 0])
	#task3()
	#task4()
	task5('map.png', np.array([175,125]), np.array([125, 125]), 1, 5)
	# task5('map.png', np.array([175,175]), np.array([125, 125]), 1, 5)
	# task5('map.png', np.array([30,30]), np.array([175, 230]), 1, 5)

if __name__ == '__main__':
    main()