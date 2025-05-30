# import cv2 as cv

# def turn_on():
#     cap1 = cv.VideoCapture(0)
#     cap2 = cv.VideoCapture(2)

#     while True:
#         # Read frames from both cameras
#         ret1, frame_1 = cap1.read()
#         ret2, frame_2 = cap2.read()

#         # Display frames if successfully read
#         if ret1:
#             # frame_1 = cv.cvtColor(frame_1, cv.COLOR_BAYER_BG2BGR)
#             cv.putText(frame_1, "Press 'q' to exit", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
#             cv.imshow("Camera View : 0", frame_1)
#         if ret2:
#             cv.putText(frame_2, "Press 'q' to exit", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)            
#             cv.imshow("Telescope View : 2", frame_2)

#         # Break loop on 'q' key press
#         if cv.waitKey(1) & 0xFF == ord('q'):
#             break

#     # Release the cameras and close windows
#     cap1.release()
#     cap2.release()
#     cv.destroyAllWindows()

# if __name__ == "__main__":
#     turn_on()

import cv2 as cv

def turn_on():
    # Open the camera
    cap = cv.VideoCapture(4)
    

    while True:
        # Read frame from the camera
        ret, frame = cap.read()

        # If frame is read correctly, ret is True
        if not ret:
            print("Failed to grab frame")
            break

        # Check the number of channels
        if len(frame.shape) == 2:
            # The image has a single channel (assumed raw Bayer)
            frame = cv.cvtColor(frame, cv.COLOR_BAYER_BG2BGR)
        elif len(frame.shape) == 3 and frame.shape[2] == 1:
            # The image has a single channel but is in a 3D array
            frame = cv.cvtColor(frame, cv.COLOR_BAYER_BG2BGR)
        elif len(frame.shape) == 3 and frame.shape[2] == 3:
            # The image is already in BGR/RGB format
            pass
        else:
            print("Unexpected number of channels in image")

        # Display the frame
        cv.putText(frame, "Press 'q' to exit", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        cv.imshow("Camera View", frame)
        # print(frame.shape)

        # Break loop on 'q' key press
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close windows
    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    turn_on()
