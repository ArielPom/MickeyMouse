import cv2
import numpy as np
import threading
from queue import Queue, Empty
import time

class BufferlessVideoCapture_threaded:
    def __init__(self, src):
        max_retries = 10
        retry_delay = 2

        for attempt in range(max_retries):
            try:
                self.cap = cv2.VideoCapture(src)
                self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

                if self.width > 0 and self.height > 0:
                    print(f"Successfully connected on attempt {attempt + 1}, frame dimensions: {self.width}x{self.height}")
                    break
                else:
                    raise ValueError("Invalid frame dimensions")

            except Exception as e:
                print(f"Attempt {attempt + 1} failed: {str(e)}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                else:
                    print("Max retries reached. Exiting.")
                    exit(1)

        self.q = Queue()
        t = threading.Thread(target=self._reader)
        t.daemon = True
        t.start()

    # Read frames as soon as available, keeping only the most recent one
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # Discard previous (unprocessed) frame
                except Empty:
                    pass
            self.q.put(frame)

    def read(self):
        return self.q.get()

    def release(self):
        self.cap.release()


# RTSP stream URL
stream_url = 'rtsp://192.168.1.136:8554/stream1'

# Chessboard dimensions (number of inner corners per row and column)
CHESSBOARD_SIZE = (7, 6)  # Example: Adjust for your calibration pattern

# Square size of the chessboard (e.g., in millimeters or any consistent unit)
SQUARE_SIZE = 25.0  # Adjust based on the chessboard used

# Termination criteria for corner subpixel optimization
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (3D points in real-world space)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Arrays to store object points and image points
object_points = []  # 3D points in real-world space
image_points = []   # 2D points in image plane

# Use BufferlessVideoCapture_threaded for video capture
cap = BufferlessVideoCapture_threaded(stream_url)

print("Press 's' to save a frame for calibration or 'q' to quit.")

frames_captured = 0

while True:
    try:
        frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        # If corners are found, refine and draw them
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            frame = cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners2, ret)

        cv2.imshow('Camera Calibration', frame)

        # Wait for key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret:  # Save the frame with detected corners

            print("Chessboard detected and frame {} saved.".format(frames_captured + 1))
            frames_captured += 1
            object_points.append(objp)
            image_points.append(corners2)
        elif key == ord('q'):  # Quit
            print("Exiting...")
            break

    except Exception as e:
        print(f"Error during capture: {str(e)}")
        break

cap.release()
cv2.destroyAllWindows()

# Perform camera calibration if enough frames were captured
if len(object_points) > 10:  # Minimum number of frames for good calibration
    print("Calibrating camera...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, gray.shape[::-1], None, None
    )
    print("Camera calibration complete.")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(dist_coeffs)
else:
    print("Not enough frames for calibration. Capture more images with chessboard visible.")
