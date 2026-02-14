import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import maestro
import time
servo = maestro.Controller('/dev/ttyACM0')
import signal

# ---------------- 基本データ ----------------
fs = 100  # サンプリングレート
count = 0

# ---------------- データ読み込み ----------------
Tk().withdraw()  # Tkinter GUIを非表示に
file_path = askopenfilename(filetypes=[("CSV files", "*.csv")])
thList = pd.read_csv(file_path, header=None).to_numpy()

def servo_controller(signum,frame):
    global count
    servoMax = 2500
    servoMin = -2500
    servoZero = 6200
    # conv = 2291.84
    conv = 1762.9
    
    thNext = thList[count]
    thServo = thNext*conv

    if thServo >= servoMax:
        th = servoMax
    elif thServo <= servoMin:
        th = servoMin
    else:
         th = thServo
    thRpm = servoZero-th
    servo.setTarget(0,int(thRpm))
    servo.setTarget(1,7100)

    count += 1

signal.signal(signal.SIGALRM,servo_controller)
signal.setitimer(signal.ITIMER_REAL,0.1,1/fs)

while True:
    time.sleep(1)
