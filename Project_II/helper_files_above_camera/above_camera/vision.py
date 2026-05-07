import cv2
import numpy as np
import os
import threading
import time

class ArUcoCamera:
    """
    A class to handle RTSP video streaming and ArUco marker detection.
    """
    def __init__(self, rtsp_url, marker_size_mm=50, calibration_dir='.'):
        """
        Initialize the ArUcoCamera.

        Args:
            rtsp_url (str): The RTSP URL of the camera stream.
            marker_size_mm (float): The size of the ArUco markers in millimeters.
            calibration_dir (str): Directory containing calibration files.
        """
        self.rtsp_url = rtsp_url
        self.marker_len_meters = marker_size_mm / 1000.0
        
        # Load calibration data
        self.mtx = None
        self.dist = None
        mtx_path = os.path.join(calibration_dir, 'calibration_matrix.npy')
        dist_path = os.path.join(calibration_dir, 'distortion_coefficients.npy')
        
        if os.path.exists(mtx_path) and os.path.exists(dist_path):
            self.mtx = np.load(mtx_path)
            self.dist = np.load(dist_path)
            print("Loaded calibration data.")
        else:
            print("Warning: Calibration data not found. Pose estimation will be limited.")

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Video Capture
        self.cap = cv2.VideoCapture(self.rtsp_url)
        if not self.cap.isOpened():
            raise ValueError(f"Could not open video stream: {self.rtsp_url}")
            
        # Define marker object points for solvePnP
        self.marker_points = np.array([
            [-self.marker_len_meters/2, self.marker_len_meters/2, 0],
            [self.marker_len_meters/2, self.marker_len_meters/2, 0],
            [self.marker_len_meters/2, -self.marker_len_meters/2, 0],
            [-self.marker_len_meters/2, -self.marker_len_meters/2, 0]
        ], dtype=np.float32)

    def get_latest_frame(self):
        """
        Reads the latest frame from the stream.
        
        Returns:
            frame (np.array): The image frame, or None if reading failed.
        """
        if not self.cap.isOpened():
            return None
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def get_marker_positions(self, frame=None):
        """
        Detects markers in the frame and returns their positions.
        
        If a frame is provided, it uses that. Otherwise, it captures a new one.
        
        Args:
            frame (np.array, optional): The frame to process.
            
        Returns:
            tuple: (processed_frame, markers_dict)
                - processed_frame: The frame with drawn markers/axes.
                - markers_dict: A dictionary {marker_id: {'rvec': ..., 'tvec': ..., 'distance': ...}}
        """
        if frame is None:
            frame = self.get_latest_frame()
            
        if frame is None:
            return None, {}

        markers_dict = {}
        corners, ids, rejected = self.detector.detectMarkers(frame)

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            if self.mtx is not None and self.dist is not None:
                # test using 
                #rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_marker_side_length, mtx, dst)
                for i in range(len(corners)):
                    marker_id = int(ids[i][0])
                    
                    # Estimate pose
                    _, rvec, tvec = cv2.solvePnP(self.marker_points, corners[i], self.mtx, self.dist)
                    
                    # Draw axis
                    cv2.drawFrameAxes(frame, self.mtx, self.dist, rvec, tvec, 0.1)
                    
                    # Store data
                    # tvec is [[x], [y], [z]]
                    distance = np.linalg.norm(tvec)
                    markers_dict[marker_id] = {
                        'rvec': rvec.flatten(),
                        'tvec': tvec.flatten(),
                        'distance': distance
                    }
                    
                    # Annotate frame
                    pos_str = f"ID: {marker_id} x: {tvec[0][0]:.2f}m y: {tvec[1][0]:.2f}m r: {rvec[2][0]:.2f}"
                    cv2.putText(frame, pos_str, 
                                (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        return frame, markers_dict
        

    def release(self):
        """Releases the video capture resource."""
        self.cap.release()
