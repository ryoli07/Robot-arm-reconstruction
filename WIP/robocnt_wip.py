import socket
import maestro
import threading
import numpy as np

pi = np.pi

servo = maestro.Controller('/dev/ttyACM0')
# servo_min = 5*pi/12 * 100
servoMax = 105
servoMin = -130
# servo_1_zero = 6200 #青サーボ1
# servo_2_zero = 5800 #青サーボ2
servo1Zero = 7100 #紫サーボ1
servo2Zero = 6000 #紫サーボ2
conv = 17

ADDRESS = '127.0.0.1'
PORT = 50009

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.bind((ADDRESS,PORT))
s.listen()

def recv_client(connection,address):
    while True:
        try:
            data = connection.recv(1024)
            if data == b"":
                break
            th1Raw = int.from_bytes(data[2:4],'little', signed=True)
            th2Raw = int.from_bytes(data[0:2],'little', signed=True)

            if th1Raw >= servoMax:
                th1 = servoMax
            elif th1Raw <= servoMin:
                th1 = servoMin
            else:
                th1 = th1Raw
            
            if th2Raw >= servoMax:
                th2 = servoMax
            elif th2Raw <= servoMin:
                th2 = servoMin
            else:
                th2 = th2Raw

            th1Rpm = servo1Zero+(th1*conv)
            th2Rpm = servo2Zero+(th2*conv)

            print(f'16bit:{data} [raw/conv/rotate] theta:{th1Raw}/{th1}/{th1Rpm} phi:{th2Raw}/{th2}/{th2Rpm}')

            servo.setTarget(0,th1Rpm)
            servo.setTarget(1,th2Rpm)
           
        except ConnectionResetError:
            break

    connection.shutdown(socket.SHUT_RDWR)
    connection.close()

while True:
    connection, address = s.accept()
    thread = threading.Thread(target=recv_client, args=(connection,address))
    thread.start()
