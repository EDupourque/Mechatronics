import numpy as np 
from utils import *

'''
just testing functions
'''
def tprint(s) -> None:
	print("\t\t{}".format(s))

def tprint_m(m):
	for r in np.vsplit(m, np.shape(m)[0]):
		tprint(r)



tprint("=== NODE: {} ===".format("Sparton"))


rot = np.identity(3)
pos_1 = np.array([-0.5, -0.5, 0]).reshape(-1,)
pos_2 = np.array([0.5, -0.5,  0]).reshape(-1,)
pos_3 = np.array([0,  1,     0]).reshape(-1,)
bok = np.array([[1,2,3,4], [5,6,7,8], [9,10,11,12], [13,14,15,16]])

node_R_b, node_pos_b = unpack_T(bok) #3x3 and 3x1

#print(node_pos_b)

rot = np.identity(3)
pos_1 = np.array([1, 0, 0])
pos_2 = np.array([0, 1, 0])
pos_3 = np.array([0, 0, 1])

err_vec = np.zeros((3, 1))

T_1 = pack_T(pos_1, rot)
T_2 = pack_T(pos_2, rot)
T_3 = pack_T(pos_3, rot)

lin_accel = np.zeros((3,1))
lin_accel[1] = 1

print(lin_accel[1])


#print(rot)

#T_1 = pack_T(pos_1, rot)
#print(T_1)
#tprint_m(T_1)

#a = np.zeros((3,1))

#print(a)

#print(bok)