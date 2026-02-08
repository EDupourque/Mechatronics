import serial
import time

G_TO_MS2 = 9.80665

#example output:  indexes 123: ax: 0.009, ay: 0.015, az: 1.006  indexes 456: ax: -0.000248118, ay: 0.0054822230000000005, az: 1.006592957

class SPARTON:
    def __init__(self, sparton, baud, num_bytes, parity_bits, stop_bits):
        self.sparton = sparton
        self.baud = baud
        self.num_bytes = num_bytes
        self.parity_bits = parity_bits
        self.stop_bits = stop_bits

    def connect(self):
        try:
            self.ser = serial.Serial(port = self.sparton, baudrate = self.baud, bytesize = self.num_bytes, parity = self.parity_bits, stopbits = self.stop_bits, timeout = 1) 
        except:
            raise ValueError("ERROR: Sparton disconnected.")
            return

    def read_accel(self):
        if self.ser: #if connected run

            try:
                self.ser.write(b'1 accelp.p\r\n') #write to sparton command to print raw and linear acceleration 

                while True:
                    #format: first 7 outputs = timestamp, raw x,y,z, linear x,y,z
                    line = self.ser.readline().decode(errors='ignore').strip()

                    if "AP" in line[0:2]: #only print lines with data
                        print(line)
                        
                        data = line[3:].split(',') #dont inlcude AP: in data
                        
                        timestamp = int(data[0]) #time in ms measuing how long the sparton has been plugged in

                        #indexes 4,5,6 are scaled to 1 g
                        ax = float(data[4]) / 1000.0 #convert from mili gs to gs
                        ay = float(data[5]) / 1000.0
                        az = float(data[6]) / 1000.0
                        
                        #conver to meters per second squared
                        ax = ax * G_TO_MS2
                        ay = ay * G_TO_MS2
                        az = az * G_TO_MS2


                        print(f"timestamp: {timestamp} ms, ax: {ax:.2f} m/s2, ay: {ay:.2f} m/s2, az: {az:.2f} m/s2")

    
            except KeyboardInterrupt: 
                self.ser.write(b'0 accelp.p\r\n') #stop sending data
                self.close()

    def read_quat(self):
        if self.ser:

            try:
                self.ser.write(b'1 quat.p\r\n')

                while True:
                    #format QUAT:%f,%f,%f,%f  w,x,y,z

                    line = self.ser.readline().decode(errors = 'ignore').strip()

                    if "QUAT" in line: #make sure reading quaternion data
 
                        
                        data = line[5:].split(',') #remove QUAT and parse data

                        w = float(data[0])
                        x = float(data[1])
                        y = float(data[2])
                        z = float(data[3])

                        print(f"w: {w}, x: {x}, y: {y}, z: {z}")


            except KeyboardInterrupt:
                self.ser.write(b'0 quat.p\r\n') #stop quaternion data
                self.close()

    def close(self):
        ser.write(b'0 accelp.p\r\n')
        ser.write(b'0 quat.p\r\n')
        self.ser.close()
           
def main():
    e = SPARTON("/dev/tty.usbserial-FTG5PLPN", 115200, 8, 'N', 1)
    e.connect()

    e.read_accel()
    e.close()

    '''
    user_input = input("Enter q for quaterion or a for accleration: ")

    if user_input == "q":
        e.read_quat()
    else:
        e.read_accel() #read acceleration by default
    '''

if __name__ == '__main__':
    main()




