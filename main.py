import time
import keyboard
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory


factory = PiGPIOFactory()


steeringPin = 17
drivePin0 = 12  # Right drive
drivePin1 = 13  # Left drive


servoSteering = Servo(steeringPin, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)
servoRightDrive = Servo(drivePin0, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)
servoLeftDrive = Servo(drivePin1, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)


steering_angle = 0.0
drive_power = 0.0
step = 0.1

try:
    print("WASD to drive, SPACE to reset steering, Q to quit")
    while True:
        if keyboard.is_pressed('a'):
            steering_angle = max(steering_angle - step, -1.0)
            servoSteering.value = steering_angle
            print(f"Steering Left: {steering_angle:.1f}")
            time.sleep(0.1)

        elif keyboard.is_pressed('d'):
            steering_angle = min(steering_angle + step, 1.0)
            servoSteering.value = steering_angle
            print(f"Steering Right: {steering_angle:.1f}")
            time.sleep(0.1)

        elif keyboard.is_pressed('space'):
            steering_angle = 0.0
            servoSteering.value = steering_angle
            print("Steering Reset to Center")
            time.sleep(0.1)

        elif keyboard.is_pressed('w'):
            drive_power = min(drive_power + step, 1.0)
            servoRightDrive.value = -drive_power  # Inverted
            servoLeftDrive.value = drive_power
            print(f"Driving Forward: {drive_power:.1f}")
            time.sleep(0.1)

        elif keyboard.is_pressed('s'):
            drive_power = max(drive_power - step, -1.0)
            servoRightDrive.value = -drive_power  # Inverted
            servoLeftDrive.value = drive_power
            print(f"Driving Reverse: {drive_power:.1f}")
            time.sleep(0.1)

        elif keyboard.is_pressed('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    print("Stopping...")
    servoRightDrive.value = 0
    servoLeftDrive.value = 0
    servoSteering.value = 0


    # servoRightDrive.value = 0
    # servoLeftDrive.value = 0

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(drivePin0,GPIO.OUT)
# GPIO.setup(drivePin1,GPIO.OUT)
# GPIO.setup(steeringPin,GPIO.OUT)

# motor_pwm0 = GPIO.PWM(drivePin0,1000)
# motor_pwm1 = GPIO.PWM(drivePin1,1000)
# servo_pwm = GPIO.PWM(steeringPin,50)

# motor_pwm0.start(10)
# motor_pwm1.start(-10)
# servo_pwm.start(7.5)

# # motor_pwm0.ChangeDutyCycle(0)
# # motor_pwm1.ChangeDutyCycle(0)


# try:
#     GPIO.output(drivePin0,GPIO.HIGH)
#     GPIO.output(drivePin1,GPIO.HIGH)
#     while True:
#         time.sleep(1)

# except KeyboardInterrupt:
#     pass

# finally:
#     GPIO.output(drivePin0,GPIO.LOW)
#     GPIO.output(drivePin1,GPIO.LOW)
#     GPIO.cleanup()

# def set_motor_speed(speed):
#     speed = max(-1.0,min(1.0,speed))
#     duty = abs(speed) * 100

#     if speed > 0:
#         motor_pwm0.ChangeDutyCycle(duty)
