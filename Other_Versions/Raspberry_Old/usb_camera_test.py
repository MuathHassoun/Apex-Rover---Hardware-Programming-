import cv2
import time

CAMERA_INDEX = 0

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

if not cap.isOpened():
    print("ERROR: Could not open USB camera")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("USB camera opened successfully")
print("Press q to quit")

while True:
    ret, frame = cap.read()

    if not ret:
        print("ERROR: Failed to read frame")
        time.sleep(0.5)
        continue

    cv2.imshow("Apex Rover USB Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
