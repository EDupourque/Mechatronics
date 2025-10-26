import numpy as  np  
import math


def main():


	g = np.array([[0.0], [0.0], [-9.81]])

		#testing  with x = 27, y = 97, z = 12, roll = 33.1, pitch = 42, yaw = 17

	testAcceleration = np.array([[27.0], [97.0], [12.0]])
	roll = 33.1
	pitch = 42.0
	yaw = 17.0

	roll = math.radians(roll)
	pitch = math.radians(pitch)
	yaw = math.radians(roll)

	Cx = np.cos(roll)
	Cy = np.cos(pitch)
	Cz = np.cos(yaw)

	Sx = np.sin(roll)
	Sy = np.sin(pitch)
	Sz = np.sin(yaw)

	R = np.array([[Cz * Cy, Cz*Sy*Sx - Sz * Cx, Cz*Sy*Cx + Sz*Sx], 
				  [Sz * Cy, Sz*Sy*Sx + Cz * Cx, Sz*Sy*Cx - Cz*Sx], 
				  [-Sy,       Cy * Sx,              Cy*Cx]])


	gForce = np.dot(R, g)

	Corrected = testAcceleration - gForce

	print(Corrected)






if __name__ == "__main__":
	main()