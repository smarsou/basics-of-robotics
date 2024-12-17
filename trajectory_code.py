from br_lectures import roty
import numpy as np
from invkin import invkin
def trajectory(q_home, rob):
	T60_1 = np.identity(4)
	T60_1[:3,:3] = roty(np.pi/2)
	T60_1[:3,3] = np.array([1, -0.15, 0.6])
	q1 = invkin(rob.DH,T60_1,[0, 0, 0])
	T60_2 = T60_1.copy()
	T60_2[:3,3] = np.array([1, -0.1, 0.85])
	q2 = invkin(rob.DH,T60_2,[0, 0, 0])
	T60_3 = T60_1.copy()
	T60_3[:3,3] = np.array([1, 0, 0.9])
	q3 = invkin(rob.DH,T60_3,[0, 0, 0])
	T60_4 = T60_1.copy()
	T60_4[:3,3] = np.array([1, 0.1, 0.85])
	q4 = invkin(rob.DH,T60_4,[0, 0, 0])
	T60_5 = T60_1.copy()
	T60_5[:3,3] = np.array([1, 0.15, 0.6])
	q5 = invkin(rob.DH,T60_5,[0, 0, 0])
	T60_6 = T60_1.copy()
	T60_6[:3,3] = np.array([1, 0.1, 0.45])
	q6 = invkin(rob.DH,T60_6,[0, 0, 0])
	T60_7 = T60_1.copy()
	T60_7[:3,3] = np.array([1, 0, 0.4])
	q7 = invkin(rob.DH,T60_7,[0, 0, 0])
	T60_8 = T60_1.copy()
	T60_8[:3,3] = np.array([1, -0.15, 0.35])
	q8 = invkin(rob.DH,T60_8,[0, 0, 0])
	T60_9 = T60_1.copy()
	T60_9[:3,3] = np.array([1, -0.2, 0.2])
	q9 = invkin(rob.DH,T60_9,[0, 0, 0])
	T60_10 = T60_1.copy()
	T60_10[:3,3] = np.array([1, -0.15, 0.05])
	q10 = invkin(rob.DH,T60_10,[0, 0, 0])
	T60_11 = T60_1.copy()
	T60_11[:3,3] = np.array([1, 0, 0])
	q11 = invkin(rob.DH,T60_11,[0, 0, 0])
	T60_12 = T60_1.copy()
	T60_12[:3,3] = np.array([1, 0.15, 0.05])
	q12 = invkin(rob.DH,T60_12,[0, 0, 0])
	T60_13 = T60_1.copy()
	T60_13[:3,3] = np.array([1, 0.2, 0.2])
	q13 = invkin(rob.DH,T60_13,[0, 0, 0])
	T60_14 = T60_1.copy()
	T60_14[:3,3] = np.array([1, 0, 0.1])
	q14 = invkin(rob.DH,T60_14,[0, 0, 0])
	T60_15 = T60_1.copy()
	T60_15[:3,3] = np.array([1, -0.15, 0.2])
	q15 = invkin(rob.DH,T60_15,[0, 0, 0])
	T60_16 = T60_1.copy()
	T60_16[:3,3] = np.array([1, 0, 0.3])
	q16 = invkin(rob.DH,T60_16,[0, 0, 0])
	T60_17 = T60_1.copy()
	T60_17[:3,3] = np.array([1, 0.2, 0.6])
	q17 = invkin(rob.DH,T60_17,[0, 0, 0])
	T60_18 = T60_1.copy()
	T60_18[:3,3] = np.array([1, 0, 0.95])
	q18 = invkin(rob.DH,T60_18,[0, 0, 0])
	T60_19 = T60_1.copy()
	T60_19[:3,3] = np.array([1, -0.15, 0.9])
	q19 = invkin(rob.DH,T60_19,[0, 0, 0])
	T60_20 = T60_1.copy()
	T60_20[:3,3] = np.array([1, -0.2, 0.6])
	q20 = invkin(rob.DH,T60_20,[0, 0, 0])
	Q = np.stack((q_home, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14, q15, q16, q17, q18, q19, q20, q_home), 1)
	return Q