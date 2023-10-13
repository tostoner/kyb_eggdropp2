import time
from socket import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress=True)

arduino_ip = '192.168.10.240'
arduino_port = 8888


def arduino_send_receive(estimate):
    udp_socket.sendto(str(estimate).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        return np.array(inbound_message.decode('ascii').split(',')).astype(float)
    except Exception as e:
        print(e)
def predict(dt: float, old_x, old_p, sigma):

    F = np.array([[1, dt], [0,1]])
    G = np.array([[0.5*dt**2],[dt]])

    #print(f"F is {F}")
    #print(f"G is {G}")

    new_x = F @ old_x

    new_p = F @ old_p @ F.T + G @ G.T *sigma
    return new_x, new_p

def update(meas_value, meas_variance, new_x, new_p):
    H = np.array([[1, 0]])
    z = np.array([meas_value])
    R = np.array([meas_variance])
    y = z - H @ new_x
    S = H @ new_p @ H.T + R
    K = new_p @ H.T @ np.linalg.inv(S)

    new_x = new_x + K@y
    new_p = (np.eye(2) - K @ H) @ new_p

    return new_x,new_p



def use_sensor_values_for_something(sensor_value, old_x, old_p, DT, sigma):
    
    new_x,new_p = predict(DT, old_x, old_p, sigma)
    try:
        new_x, new_p = update(sensor_value, sigma, new_x, new_p)
    except Exception as e:
        print(f"error in update: {str(e)}")
        
    #print(new_x)
    return new_x, new_p

def arduino_has_been_reset():
    print("Arduino is offline.. Resetting")




estimate = 0.0
delta = 1.0
T = time.time()
sigma_r = 1.175363805

sigma_a = 1.13105E-07
positions = []
measurements = []
i=0

old_x = np.array([[236],
                      [0]]) #initial position and speed
old_p = np.array([[1,0],
                    [0,2]]) #covariance matrix.. how certain i am about the initial contitions
while i<1000:
    i = i+1
    dt = time.time() - T
    T = time.time()
    try:
        measured_values = arduino_send_receive(estimate)
        range_measurement = measured_values[-1]
    except:
        range_measurement = 10
    #print(measured_values)
    
    old_x, old_y = use_sensor_values_for_something(range_measurement, old_x, old_p, dt, sigma_r)
    positions.append(old_x[0][0])
    measurements.append(range_measurement)
    print(f"old_x is {old_x[0]}, range measurement is {range_measurement}")


plt.xlabel('Time step')
plt.ylabel('Position')
plt.ylim(70, 260)
plt.xlim(0, 5000)
plt.legend()


plt.plot(positions, label='Estimated Position')
plt.plot(measurements, label='Measured Position')
plt.show()

    



    #estimate = estimate + delta
    #if(estimate > 100.0):
    #    delta = -1
    #elif(estimate < -100):
    #    delta = 1
#
    #sensor_values = arduino_send_receive(estimate)
    #if(sensor_values is not None):
    #    use_sensor_values_for_something(sensor_values)
    #else:
    #    arduino_has_been_reset()

