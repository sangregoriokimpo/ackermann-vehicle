import cv2
import numpy as np

# Open USB camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Allow resizable windows
cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
cv2.namedWindow("Black Mask", cv2.WINDOW_NORMAL)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    frame = cv2.resize(frame, (320, 240))
    roi = frame[140:240, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])

    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Show enlarged windows
    cv2.imshow("Original", roi)
    cv2.imshow("Black Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
