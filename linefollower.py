import cv2
import numpy as np
import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# Use pigpio factory for accurate PWM
factory = PiGPIOFactory()

# Assign your GPIO pins
steeringPin = 17
drivePin0 = 12  # Right drive
drivePin1 = 13  # Left drive

# Servo setup (adjust pulse widths if needed)
servoSteering = Servo(steeringPin, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)
servoRightDrive = Servo(drivePin0, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)
servoLeftDrive = Servo(drivePin1, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)

# Open USB camera (likely /dev/video0)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Start motors forward slowly
servoRightDrive.value = -0.4  # reverse for right motor
servoLeftDrive.value = 0.4

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Resize and crop ROI
        frame = cv2.resize(frame, (320, 240))
        roi = frame[140:240, :]

        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Black mask (tune for your lighting!)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Compute moments to find centroid
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])  # Center X position of line
            error = cx - 160               # Image center is 160px (320/2)
            steering_value = error / 160.0 # Normalize between -1.0 and 1.0
            steering_value = max(-1.0, min(1.0, steering_value))
            servoSteering.value = steering_value

            print(f"Line: {cx}  Steering: {steering_value:.2f}")

            # Draw debug
            cv2.circle(roi, (cx, 50), 5, (255, 0, 0), -1)
        else:
            print("Line lost!")
            # Optional: stop or keep previous steering

        # Show original + mask (small windows are okay!)
        cv2.imshow("Original", roi)
        cv2.imshow("Black Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    print("Stopping...")
    servoSteering.value = 0
    servoRightDrive.value = 0
    servoLeftDrive.value = 0
    cap.release()
    cv2.destroyAllWindows()
