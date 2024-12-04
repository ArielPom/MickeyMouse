import cv2
import numpy as np
from queue import Queue, Empty
import threading
import time
import collections

np.set_printoptions(precision=2, suppress=True)


camera_matrix = np.array([[903.44505133, 0., 948.97021146],
                           [0., 904.07851181, 539.43117405],
                           [0., 0., 1.]], dtype=np.float32)

dist_coeffs = np.array([[1.37039113e-01, 6.21115570e-02, -1.09408142e-03, -1.78864023e-04, -2.72353698e-01]],
                       dtype=np.float32)

aruco_world_location = np.array([1000, 0, 0], dtype=np.float32).reshape(3, 1)

stream_url = 'rtsp://192.168.1.136:8554/stream1'


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

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()
  
  def release(self):
    self.cap.release()


class locationEstimation:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.location_window = collections.deque(maxlen=window_size)
        self.aruco_world_location = aruco_world_location

    def estimate_location(self, rvec, tvec, marker_id):
        
        R, _ = cv2.Rodrigues(rvec)
        dot_product = np.dot(R.T, tvec)
        camera_world_location = np.zeros(3, dtype=np.float32)
        if marker_id == 0:
            camera_world_location[0] = aruco_world_location[0] - dot_product[0] * 100
            camera_world_location[1] = aruco_world_location[1] - dot_product[1] * 100
            camera_world_location[2] = aruco_world_location[2] + dot_product[2] * 100
        elif marker_id == 1:
            camera_world_location[0] = aruco_world_location[2] + dot_product[2] * 100
            camera_world_location[1] = aruco_world_location[1] - dot_product[1] * 100
            camera_world_location[2] = aruco_world_location[0] - dot_product[0] * 100
        camera_world_location = np.round(camera_world_location, 2)

        self.location_window.append(camera_world_location)
        mean_location = np.mean(self.location_window, axis=0).astype(np.float32)
        return mean_location.flatten()
    
    
def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return np.array(rvecs), np.array(tvecs)


def has_gui():
    """Check if OpenCV GUI backend is available"""
    try:
        cv2.namedWindow('Test', cv2.WINDOW_NORMAL)
        cv2.destroyWindow('Test')
        return True
    except cv2.error:
        print("No GUI backend available")
        return False


def draw_text(frame, aruco_coords, camera_coords):

    cv2.putText(frame, "camera world x,y,z: [cm]", (20, frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, f"{camera_coords[0]:.2f}, {camera_coords[1]:.2f}, {camera_coords[2]:.2f}", (20, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3, cv2.LINE_AA)

    cv2.putText(frame, "ArUco tvec x,y,z:", (frame.shape[1] - 640, frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, f"{aruco_coords[0]:.2f}, {aruco_coords[1]:.2f}, {aruco_coords[2]:.2f}", (frame.shape[1] - 640, frame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3, cv2.LINE_AA)


class VideoWriterThread:
    def __init__(self, filename, fourcc, fps, frame_size):
        self.queue = Queue()
        self.stopped = False
        self.writer = cv2.VideoWriter(filename, fourcc, fps, frame_size)
        self.thread = threading.Thread(target=self._write_frames)
        self.thread.start()

    def _write_frames(self):
        while not self.stopped or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=1)
                self.writer.write(frame)
            except Empty:
                continue

    def write(self, frame):
        self.queue.put(frame)

    def stop(self):
        self.stopped = True
        self.thread.join()
        self.writer.release()


if __name__ == '__main__':
    
    locationEstimator = locationEstimation(window_size=10)
    cap = BufferlessVideoCapture_threaded(stream_url)
    
    # Initialize VideoWriterThread to save the video without blocking
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    frame_size = (cap.width, cap.height)
    out_thread = VideoWriterThread('output.avi', fourcc, 20.0, frame_size)
    
    gui_available = has_gui()
    if gui_available:
        cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Video', cap.width, cap.height)

    # Load the aruco dictionary and detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    marker_size = 0.1  # Marker size in meters

    while True:
        frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Remove resize operation
        
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        if ids is not None:
            if gui_available:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id == 0:
                    aruco_world_location = np.array([0, 0, 100], dtype=np.float32).reshape(3, 1)
                elif marker_id == 1:
                    aruco_world_location = np.array([60, 0, 100], dtype=np.float32).reshape(3, 1)
                rvec, tvec = estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, dist_coeffs)
                camera_location = locationEstimator.estimate_location(rvec[0], tvec[0], ids[i][0])
                aruco_location = tvec[0].flatten()

                camera_location_text = f"camera x,y,z: {camera_location}"
                aruco_location_text = f"aruco x,y,z: {aruco_location}"
                print(camera_location_text)
                print(aruco_location_text)

                if gui_available:
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
                    draw_text(frame, aruco_location, camera_location)
                
                break

        # Enqueue the frame for writing
        out_thread.write(frame)

        if gui_available:
            cv2.imshow('Video', frame)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    out_thread.stop()
    if gui_available:
        cv2.destroyAllWindows()