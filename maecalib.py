import maestro
servo = maestro.Controller("COM4")   
servo.setTarget(0,6200)
servo.close()
