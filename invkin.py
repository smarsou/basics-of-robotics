import numpy as np
from br_lectures import dh

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