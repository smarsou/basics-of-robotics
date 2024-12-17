import sys
import numpy as np
import vtk
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")
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
def line_intersection(line1, line2):
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3
    denominator = dx1*dy2 - dy1*dx2
    if abs(denominator) < 10**4 :
        return ('lol','lol') # the lines are parallel
    else:
        t = ((x3-x1)*dy2 - (y3-y1)*dx2) / denominator
        x = x1 + t*dx1
        y = y1 + t*dy1
        return (x, y)

def is_point_on_line(point, line):
	x, y ,x1 , y1 ,x2, y2= point[0], point[1], line[0][0], line[0][1], line[1][0], line[1][1]
	m = (y2 - y1) / (x2 - x1)
	b = y1 - m*x1
	y_expected = m*x + b
	return abs(y - y_expected) < 1e-6

def imgproc(RGB, camera, camera_height, box_size, bool):
	img_w = camera['img_size'][0]
	img_h = camera['img_size'][1]
	box_w = box_size[0]
	box_h = box_size[1]
	plt.imshow(RGB)
	E=feature.canny(RGB[:,:,0])	
	H, theta, d = hough_line(E)	
	counter = 0
	lines = []
	for _, angle, dist in zip(*hough_line_peaks(H, theta, d, min_distance=20, min_angle=20, threshold=50)):
		# print('angle=%f, dist=%f' % (angle, dist))
		cs = np.cos(angle)
		sn = np.sin(angle)
		if np.abs(cs) >= np.abs(sn):
			v = np.array([0, img_h-1])
			u = (dist - sn * v) / cs
		else:
			u = np.array([0, img_w-1])
			v = (dist - cs * u) / sn
		# print(u,v)
		# u is x coordinates of first and second point of the line
		# v is y coordinates of first and second point of the line
		plt.plot(u, v, 'g')	
		lines.append(((u[0],v[0]),(u[1],v[1])))


	#Compute intersection
	corners=[]

	if len(lines)<4:
		if bool:
			print("Error: We found less than 4 lines with hough_line_peaks methods.")
		return -1, -1

	x,y = line_intersection(lines[0], lines[1])
	if x != 'lol':
		corners.append((x,y))
		plt.scatter(x,y,color='r')
	x,y = line_intersection(lines[0], lines[2])
	if x != 'lol':
		corners.append((x,y))
		plt.scatter(x,y,color='r')
	x,y = line_intersection(lines[0], lines[3])
	if x != 'lol':
		corners.append((x,y))
		plt.scatter(x,y,color='r')

	x,y = line_intersection(lines[1], lines[2])
	if x != 'lol':
		corners.append((x,y))
		plt.scatter(x,y,color='r')

	x,y = line_intersection(lines[1], lines[3])
	if x != 'lol':
		corners.append((x,y))
		plt.scatter(x,y,color='r')

	x,y = line_intersection(lines[2], lines[3])
	if x != 'lol':
		corners.append((x,y))
		plt.scatter(x,y,color='r')

	if len(corners) != 4:
		if bool:
			print('Error: There is not 4 corners')
		return -1,-1

	#We take two lines who intersect with each other
	Line1=lines[0]
	for line in lines[1:]:
		if line_intersection(Line1, line) != ('lol','lol'):
			Line2=line

	#We find the common corner and the two others
	cornerLine1= []
	cornerLine2= []
	for corner in corners:
		if is_point_on_line(corner,Line1):
			cornerLine1.append(corner)
	for corner in corners:
		if is_point_on_line(corner,Line2):
			cornerLine2.append(corner)

	if len(cornerLine1)==0 or len(cornerLine2)==0:
		if bool:
			print("Error: an computation error of the corners. Try again.")
		return -1,-1
	
	for corner in cornerLine1:
		sharedCorner = corner
		if corner == cornerLine2[0]:
			corner2 = cornerLine2[1]
		elif corner == cornerLine2[1]:
			corner2 = cornerLine2[0]
	if sharedCorner == cornerLine1[0]:
		corner1 = cornerLine1[1]
	else:
		corner1 = cornerLine1[0]

	#We compute two vectors
	vector1 = corner1[0]-sharedCorner[0], corner1[1]-sharedCorner[1]
	vector2 = corner2[0]-sharedCorner[0], corner2[1]-sharedCorner[1]

	#We find the middle of the object
	x1, y1 = sharedCorner
	x2, y2 = (sharedCorner[0]+vector1[0]/2,sharedCorner[1]+vector1[1]/2)
	x3, y3 = (x2+vector2[0]/2,y2+vector2[1]/2)
	middle = x3,y3
	if bool:
		plt.plot([x1, x2], [y1, y2], 'b-')
		plt.plot([x2, x3], [y2, y3], 'b-')
		# plt.plot([x2, x2+100], [y2, y2], 'c-')
		plt.scatter(middle[0],middle[1],color='c')
	
	
	x = (middle[0] - camera["principal_point"][0])*camera_height/camera["focal_length"]
	y = (middle[1] - camera["principal_point"][1])*camera_height/camera["focal_length"]

	#We have t 
	t=[x,y,camera_height]
	
	#Compute alpha
	n1 = np.linalg.norm(vector1)
	n2 = np.linalg.norm(vector2)
	#We take the longest vector to have the direction of the object
	vector = vector1
	if n2>n1:
		vector = vector2
	
	vectorHorizUni= [1, 0]
	if bool:
		print("direction of the object: " + str(vector))
	
	a = vector
	b = vectorHorizUni
	#We compute alpha through arcos(a.b/|a||b|)
	cosT = (a[0]*b[0]+a[1]*b[1])/np.linalg.norm(a)
	alpha = np.arccos(cosT)

	return alpha, t
	

def task4(bool):
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
	
	alpha, t = imgproc(RGB, camera, camera_height, [box_w, box_h],bool)

	if alpha == -1:
		return -1, -1
	
	e_alpha, e_t = camsim.evaluate(alpha, t)
	if bool:
		print(f"alpha = {alpha} rad\nt= {t}")
		print('orientation error: %f deg position error: %f mm' % (np.rad2deg(e_alpha), 1000 * e_t))	

	if bool:
		plt.show()
	return e_alpha, e_t
	
def statsTask4():
	L1 =[]
	m =0
	L2= []
	for i in range(100):
		e1,e2 = task4(False)
		if (e1,e2)!=(-1,-1):
			L1.append(np.rad2deg(e1))
			L2.append(e2)
			if np.rad2deg(e1) < 5:
				m+=1

	
	plt.subplot(1,2,1)
	plt.title("Angle errors (in deg)")
	for i in range(len(L1)):
		plt.scatter(i, L1[i])
	plt.subplot(1,2,2)
	plt.title("Distance errors (in mm)")
	plt.ylim(-0.01,0.01)
	for i in range(len(L2)):
		plt.scatter(i,L2[i])
	print(str(round(m/len(L1)*100,3)) + "% of error's angle is near to 0.")
	plt.show()

def main():
	#task0()
	#task1([0, np.pi/2, -np.pi/2, 0, 0, 0])
	#task2([0, 1, 0])
	#task3()
	task4(True)
	# statsTask4()
	
		

if __name__ == '__main__':
    main()