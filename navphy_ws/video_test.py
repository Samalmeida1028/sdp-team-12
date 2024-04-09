import cv2

def main():
    # Create a VideoCapture object
    cap = cv2.VideoCapture(2)  # 0 for the default camera, you can change it to the desired camera index if you have multiple cameras

    # Check if the camera is opened successfully
    if not cap.isOpened():
        print("Error: Couldn't open camera.")
        return

    # Loop to capture and display frames from the camera
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame is read correctly
        if not ret:
            print("Error: Couldn't read frame.")
            break

        # Display the captured frame
        cv2.imshow('Video Capture', frame)

        # Wait for 'q' key to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture object and close the OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
