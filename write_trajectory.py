f = open("trajectory.txt", "r")
lignes = f.readlines()
f.close()

f2 = open("trajectory_code.py", "w")

f2.write("from br_lectures import roty\nimport numpy as np\nfrom invkin import invkin\ndef trajectory(q_home, rob):\n")
for j in range(len(lignes)):
    x,y,z = [eval(i) for i in lignes[j].split()]
    if j+1 == 1:
        f2.write("	T60_1 = np.identity(4)\n")
        f2.write("	T60_1[:3,:3] = roty(np.pi/2)\n")
    else:
        f2.write(f"	T60_{j+1} = T60_1.copy()\n")
    f2.write(f"	T60_{j+1}[:3,3] = np.array([{x}, {y}, {z}])\n")
    # f2.write(f"	T60_1[:3,3] = np.array([{x}, {y}, {z}])\n")
    f2.write(f"	q{j+1} = invkin(rob.DH,T60_{j+1},[0, 0, 0])\n")
f2.write("	Q = np.stack((q_home, ")
for j in range(len(lignes)):
    f2.write(f"q{j+1}, ")
f2.write("q_home), 1)\n")
f2.write("	return Q")

f2.close()
