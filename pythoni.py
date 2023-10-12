import time
from socket import *
import numpy as np
import kalmanFilter.kalmanFilter as kf

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


def use_sensor_values_for_something(sensor_values):
    dt = 1.0/60
    sigma_r = 1.908487197

    sigma_a = 1.13105E-07


    F = np.array([[1, dt, 0], [0, 1, dt], [0, 0, (dt**2)/2]]) #X_k

    Q = np.array([[(dt**6)/36, (dt**5)/12, (dt**4)/6], [(dt**5)/12, (dt**4)/6, (dt**3)/2], [(dt**4)/6, (dt**3)/2, dt**2]])
    H_r = np.array([[1, 0, 0]])
    H_a = np.array([[0, 0, 1]])

    R_r = np.array([sigma_r])
    R_a = np.array([sigma_a])


    x = np.linspace(0,0,0)
    measurements = - (x**2 + 2*x - 2)  + np.random.normal(0, 2, 100)

   
    # kf = KalmanFilter(F=F, H=H, Q=Q, R=R)

    predictions = []


def arduino_has_been_reset():
    print("Arduino is offline.. Resetting")


estimate = 0.0
delta = 1.0
while(True):
    estimate = estimate + delta
    if(estimate > 100.0):
        delta = -1
    elif(estimate < -100):
        delta = 1

    sensor_values = arduino_send_receive(estimate)
    if(sensor_values is not None):
        use_sensor_values_for_something(sensor_values)
    else:
        arduino_has_been_reset()

