from gpiozero import Servo
from time import sleep
# from gpiozero.pins.pigpio import PiGPIOFactory

# factory = PiGPIOFactory()

servo = Servo(16, min_pulse_width=0.5/1000)


# servo.mid()
# sleep(3)
            
servo.min()
sleep(3)

servo.max()
sleep(3)

servo.min()
sleep(3)

# servo.value = 1
# sleep(3)
