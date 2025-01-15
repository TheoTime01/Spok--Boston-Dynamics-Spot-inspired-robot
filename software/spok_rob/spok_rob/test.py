import cv2

def list_cameras():
    index = 0
    available_cameras = []
    while True:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            break
        available_cameras.append(index)
        cap.release()
        index += 1
    return available_cameras

def take_picture(camera_index=0, desired_fps=30):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Cannot open camera with index {camera_index}")
        return

    # Set desired FPS (if supported by the camera)
    cap.set(cv2.CAP_PROP_FPS, desired_fps)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Desired FPS: {desired_fps}, Actual FPS: {actual_fps}")

    print(f"Camera {camera_index} opened. Press 's' to save a picture.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        cv2.imshow("Press 's' to save or 'q' to quit", frame)
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite("captured_image.jpg", frame)
            print("Image saved as 'captured_image.jpg'")
            break
        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

# List cameras
cameras = list_cameras()
if cameras:
    print("Available cameras:", cameras)
    # Use the first available camera with desired FPS
    take_picture(camera_index=cameras[0], desired_fps=10)
else:
    print("No cameras found.")
