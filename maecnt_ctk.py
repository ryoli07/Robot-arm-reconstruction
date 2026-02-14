# import maestro
import time
import numpy as np
# import tkinter as tk
# from tkinter import ttk
import threading
import customtkinter as tk
running = False

def start():
    global running
    running = True
    maeThread = threading.Thread(target=maeCont,daemon=True)
    maeThread.start()

def stop():
    global running
    running = False

def maeCont():
    global running
    port = port_value.get()
    # servo = maestro.Controller(port)
    model = model_value.get()
    if model == 1:
        servo1Zero = 6200
        servo2Zero = 7100
    elif model == 2:
        servo1Zero = 6000
        servo2Zero = 7700
    conv = 1762.9
    angle = scale.get()
    # rad = angle * (np.pi/180)
    # servoAngle = rad*conv
    # servo.setSpeed(0,100)
    # servo.setTarget(1,servo2Zero)
    while running:
        # servo.setTarget(0,int(servo1Zero-servoAngle))
        # time.sleep(0.8)
        # servo.setTarget(0,int(servo1Zero+servoAngle))
        # time.sleep(0.8)
        print(angle)
        time.sleep(0.8)

root = tk.CTk()
root.title("Maestro Controller")

window_width = 400
window_height = 700
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
x_position = (screen_width // 2) - (window_width //2)
y_position = (screen_height //2) - (window_height //2)
root.geometry(f"{window_width}x{window_height}+{x_position}+{y_position}")

angle_title = tk.CTkLabel(root,text="角度")
angle_title.pack()
angle_value = tk.IntVar()
angle_value.set(40)
scale = tk.CTkSlider(root,from_=30,to=50,variable=angle_value)
angle_value_text = scale.get()
angle_value_label = tk.CTkLabel(root,text=angle_value_text)
angle_value_label.pack()
scale.pack()


speed_title = tk.CTkLabel(root,text="速度")
speed_title.pack()
speed_value = tk.IntVar()
speed_value.set(40)
speed_scale = tk.CTkSlider(root,from_=30,to=50,variable=speed_value)
speed_scale.pack()

model_title = tk.CTkLabel(root,text="サーボの選択")
model_title.pack()
model_value = tk.IntVar()
model_value.set(1)
tk.CTkRadioButton(root,text="モデルサーボ",variable=model_value,value=1).pack()
tk.CTkRadioButton(root,text="再現用サーボ",variable=model_value,value=2).pack()

port_title = tk.CTkLabel(root,text="ポートの選択")
port_title.pack()
port_value = tk.StringVar()
port_value.set("COM3")
tk.CTkRadioButton(root,text="COM1",variable=port_value,value="COM1").pack()
tk.CTkRadioButton(root,text="COM2",variable=port_value,value="COM2").pack()
tk.CTkRadioButton(root,text="COM3",variable=port_value,value="COM3").pack()
tk.CTkRadioButton(root,text="COM4",variable=port_value,value="COM4").pack()
tk.CTkRadioButton(root,text="COM5",variable=port_value,value="COM5").pack()
tk.CTkRadioButton(root,text="COM6",variable=port_value,value="COM6").pack()
tk.CTkRadioButton(root,text="COM7",variable=port_value,value="COM7").pack()
tk.CTkRadioButton(root,text="COM8",variable=port_value,value="COM8").pack()
tk.CTkRadioButton(root,text="ttyACM0",variable=port_value,value="/dev/ttyACM0").pack()

start_button = tk.CTkButton(root,text="Start",command=start)
start_button.pack(pady=10)
stop_button = tk.CTkButton(root,text="Stop",command=stop)
stop_button.pack(pady=10)

root.mainloop()
