import cv2

def list_available_cameras(max_index=10):
    available_cameras = []
    for index in range(max_index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            print(f"Camera index {index} is available.")
            available_cameras.append(index)
            cap.release()
        else:
            print(f"Camera index {index} is not available.")
    return available_cameras

if __name__ == "__main__":
    max_index = 10  # Change this to the number of devices you want to check
    available_cameras = list_available_cameras(max_index)
    if available_cameras:
        print(f"Available cameras: {available_cameras}")
    else:
        print("No cameras available.")
