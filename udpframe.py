import numpy as np
class udpFrame:
    def __init__(self, readerID, channel, zoneID, sensorID, sequence, data, timestamp, serial, polarity=0):
        self.readerID = readerID
        self.channel = channel 
        self.zoneID = zoneID 
        self.sensorID = sensorID
        self.sequence = sequence 
        self.serial = serial
        self.polarity = polarity
        #print(data.hex())
        self.data = data 
        self.timestamp = timestamp
        self.t = self.convSigned(data[0:2])
        self.z = self.convSigned(data[2:4])
        self.y = self.convSigned(data[4:6])
        self.x = self.convSigned(data[6:8])
        if len(data) == 10: #measure frame
            self.sc = self.convSigned(data[8:10]) #subcarrier freq.
    def convSigned(self, xuint16):
        x = int.from_bytes(xuint16, byteorder='big', signed=True)
        #if x > 32768:
        #   x -= 65536 
        return x    
    def logdata(self):
        txt = f'{self.readerID},{self.channel},{self.sensorID},{self.sequence},{self.serial},{self.polarity},{self.timestamp},{self.t},{self.x},{self.y},{self.z}'
        if len(self.data) == 10:
            txt = txt+','+self.sc
        txt = txt+'\n'
        return txt
           

