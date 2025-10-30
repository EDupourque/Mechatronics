import numpy as  np  
import math
G_TO_MS2 = 9.80665 # gravity conversion

def adjust_accel(accel_x: float, accel_y: float, acel_z: float, yw: float, ptch: float, rll: float) -> tuple:
        ay = accel_y * G_TO_MS2
        ax = accel_x * G_TO_MS2
        az = acel_z * G_TO_MS2
        a_vector = np.array([[ax], [ay], [az]])

        # convert from degrees to radians
        
        y = math.radians(yw)
        p = math.radians(ptch)
        r = math.radians(rll)
        
        Cx = np.cos(r)
        Cy = np.cos(p)
        Cz = np.cos(y)

        Sx = np.sin(r)
        Sy = np.sin(p)
        Sz = np.sin(y)
        
        R = np.array([[Cz * Cy, Cz*Sy*Sx - Sz * Cx, Cz*Sy*Cx + Sz*Sx],  #rotates from body to earth frame
                    [Sz * Cy, Sz*Sy*Sx + Cz * Cx, Sz*Sy*Cx - Cz*Sx], 
                    [-Sy,       Cy * Sx,              Cy*Cx]])

        g_vector = np.array([[0.0], [0.0], [9.80665]]) #positive G because NED convention

        a_earth =  R @ a_vector #accleration due to gravity  along xyz

        adjusted = a_earth - g_vector #remove gravity from acceleration for linear ax,ay,az


        ax = float(adjusted[0]) 
        ay = float(adjusted[1]) 
        az = float(adjusted[2]) 

        return (ax, ay, az)



def main():
	accel_x = 2.75
	accel_y = 9.89 
	accel_z = 1.22
	yaw = 17.0
	pitch = 42.0
	roll = 33.1
	
	accel_x, accel_y, accel_z = adjust_accel(accel_x, accel_y, accel_z, yaw, pitch, roll)

	#in m/s^2 = 33.79, 93.47, 18.07 
	print(accel_x)
	print(accel_y)
	print(accel_z)



if __name__ == "__main__":
	main()