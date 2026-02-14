import time
import threading
import socket
import select
from udpframe import udpFrame
import keyboard
import matlab.engine
import numpy as np
import maestro
from collections import deque
import signal

eng = matlab.engine.start_matlab()

# 加速度データ加工関数
def parseFrame(rx, datawords): #data words is the number of words in data
    readerID = rx[0]
    channel = rx[1]
    zoneID = rx[2]
    sequence = rx[3]
    data = rx[4:datawords*2+4] 
    serial = 0 
    polarity = 0
    if len(rx) == 16:  #old format
        timestamp = int.from_bytes(rx[12:16], byteorder='big')
        timestamp = float(timestamp)/1000 
    elif len(rx) == 18: # serial number 
        timestamp = int.from_bytes(rx[12:16], byteorder='big')
        timestamp = float(timestamp)/1000 
        serial = int.from_bytes(rx[16:18], byteorder='big'); 
    elif len(rx) == 19: # sensor data plus phase polarity 
        timestamp = int.from_bytes(rx[12:16], byteorder='big')
        timestamp = float(timestamp)/1000 
        serial = int.from_bytes(rx[16:18], byteorder='big'); 
        polarity = rx[18]
    elif len(rx) == 20: # measurement frame
        timestamp = int.from_bytes(rx[14:18], byteorder='big')
        timestamp = float(timestamp)/1000 
        serial = int.from_bytes(rx[-2:], byteorder='big')
    else:  # no time stamp old format too
        timestamp = float(time.time()) 
    sensorID = str(readerID)+'_'+str(channel)+'_'+f'{serial:04x}'
    #print(readerID, channel, zoneID, sequence, data.hex(), timestamp, serial)
    frame = udpFrame(readerID, channel, zoneID, sensorID, sequence, data, timestamp, serial, polarity)
    return frame

# UDP受信スレッド
def udpReceive(frame, srcadd, udpflag,buffer,bufLock):
      sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      try:
        sock.bind(srcadd)
        sock.setblocking(False)
      except: 
        pass
      datawords = 4

      try: 
        while udpflag()==True:
          recv, trns, oob = select.select([sock], [], [], int(30))
          if len(recv) == 0:
            break
          else:
            for s in recv:
              rx, addr = s .recvfrom(frame)

          try:
            cf = parseFrame(rx, datawords)
            x = cf.x
            y = cf.y
            z = cf.z
            data = [x,y,z]
            with bufLock:
               buffer.append(data)
          except:
            pass
      finally: 
        sock.close()

def angle_calc(fs,length,rangeVal,buffer,bufLock,matlabLock,thbuf,thbufLock):
   thNext = 0
   thCurr = 0
   thPrev = 0
   aveTarget = 1000
   aveNum = 300
   removeNum = int(aveTarget - aveNum/2)

   while True:
      with bufLock:
         resultCheck = False
         if len(buffer) >= aveTarget:
            data = list(buffer)
            thListRaw,thCurr,thNext = eng.AngleCalc(matlab.double(data),float(length),float(fs),float(rangeVal),float(thCurr),float(thPrev),nargout=3)
            thList = np.array(thListRaw)
            for _ in range(removeNum):
               buffer.popleft()
            resultCheck = True
      
      if resultCheck:
         with thbufLock:
            for i in range(removeNum):
               thbuf.append(thList[i,0])

      thPrev = thCurr
      thCurr = thNext

def servo_controller(thbuf,thbufLock,servo,fs):
      servoMax = 2500
      servoMin = -2500
      servoZero = 6000
      conv = 2291.84
      servo.setTarget(1,7700)
      while True:
         thRaw = None
         with thbufLock:
               if thbuf:
                  thRaw = thbuf.popleft()
         if thRaw is not None:
            thServo = thRaw*conv
            if thServo >= servoMax:
               th = servoMax
            elif thServo <= servoMin:
               th = servoMin
            else:
               th = thServo
            thRpm = servoZero-th
            servo.setTarget(0,int(thRpm))
         # time.sleep(1/fs)


def main():
  global servo,thbuf,thbufLock
  IPADDRESS = "10.128.1.24"
  PORT = 1234
  RFIC = "jupiter2"
  SENSOR = "adxl367"
  srcadd = (IPADDRESS, PORT)

  fs = 125
  length = 0.42
  rangeVal = 8
  bufferMax = 3000
  buffer = deque(maxlen=bufferMax)
  bufLock = threading.Lock()
  matlabLock = threading.Lock()
  servo = maestro.Controller('COM6')
  thbuf = deque(maxlen=10000)
  thbufLock = threading.Lock()
  
  global udpstat
  udpstat = True

  receiveThread = threading.Thread(target=udpReceive, args=(2048, srcadd, lambda: udpstat,buffer,bufLock),daemon=True)
  calcThread = threading.Thread(target=angle_calc,args=(fs,length,rangeVal,buffer,bufLock,matlabLock,thbuf,thbufLock),daemon=True)
  servoThread = threading.Thread(target=servo_controller,args=(thbuf,thbufLock,servo,fs),daemon=True)
  receiveThread.start()
  calcThread.start()
  servoThread.start()

  try:
     while True:
        time.sleep(1)
  except KeyboardInterrupt:
     print('終了')          
              
if __name__ == '__main__':
  main()
