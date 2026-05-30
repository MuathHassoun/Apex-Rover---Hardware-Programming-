
import cv2

camera_ids = [0, 1, 2, 3]

for cam_id in camera_ids:
    print(f"Testing /dev/video{cam_id}")

    cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"/dev/video{cam_id} not opened")
        continue

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    ret, frame = cap.read()

    if ret:
        filename = f"opencv_test_video{cam_id}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
    else:
        print(f"Could not read frame from /dev/video{cam_id}")

    cap.release()
