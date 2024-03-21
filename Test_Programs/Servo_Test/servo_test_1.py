from time import sleep
import RPi.GPIO as GPIO

class servo_control():
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        servo_pin = 36

        GPIO.setup(servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(servo_pin,50)  #Set PWM frequency as 50Hz
                                    #Based on servo motor parameters
        self.pwm.start(5)    #Starting PWM with dutycycle as 5%
                        #Some default value needed within bounds

    def servo_angle_set(self, dutycycle, delay): #dutycycle of servo, delay between each movement
        self.pwm.ChangeDutyCycle(dutycycle) #Change dutycycle of PWM
        sleep(delay) #Delay for given seconds
    
    def servo_power_down(self):
        self.pwm.ChangeDutyCycle(3) #Reset pos  of gripper to closed state
        sleep(1)
        self.pwm.stop() #Stop PWM


p1 = servo_control()
def main():
    p1.servo_angle_set(5.5, 1) #5.5 duty cycle - servo at center (half open)
    p1.servo_angle_set(7.5, 1) #7.5 duty cycle - servo at right (fully open)
    p1.servo_angle_set(5.5, 1) #5.5 duty cycle - servo at center (fully open)
    p1.servo_angle_set(2.5, 1) #2.5 duty cycle - servo at left (closed)
    p1.servo_power_down() #Power down servo pwm and gpio
    


if __name__ == '__main__':
    main()