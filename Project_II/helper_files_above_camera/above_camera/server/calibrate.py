import numpy as np
import cv2
import sys
import os
import argparse
from vision import ArUcoCamera

def main():
    parser = argparse.ArgumentParser(description='Camera Calibration using ArUco GridBoard')
    parser.add_argument('url', help='RTSP stream ip (e.g., rtsp://<ip>:8554/cam) or full URL')
    parser.add_argument('--markers_x', type=int, default=5, help='Number of markers in X direction')
    parser.add_argument('--markers_y', type=int, default=7, help='Number of markers in Y direction')
    parser.add_argument('--marker_length', type=float, default=0.04, help='Marker side length (in meters)')
    parser.add_argument('--marker_separation', type=float, default=0.005, help='Separation between markers (in meters)')
    parser.add_argument('--count', type=int, default=20, help='Number of successful frames to collect')
    args = parser.parse_args()

    # Determine URL
    if args.url.startswith('rtsp://') or args.url.startswith('http://'):
        rtsp_url = args.url
    else:
        rtsp_url = f"rtsp://{args.url}:8554/cam2"

    # Initialize ArUcoCamera
    print(f"Connecting to {rtsp_url}...")
    try:
        camera = ArUcoCamera(rtsp_url, marker_size_mm=args.marker_length * 1000)
    except Exception as e:
        print(f"Error initializing camera: {e}")
        sys.exit(1)

    print(f"Starting calibration. Need {args.count} good frames.")
    print(f"Looking for {args.markers_x}x{args.markers_y} ArUco GridBoard.")
    print("Press 'c' to capture a frame if markers are found.")
    print("Press 'q' to quit.")

    # Create ArUco GridBoard
    board = cv2.aruco.GridBoard((args.markers_x, args.markers_y), 
                                args.marker_length, 
                                args.marker_separation, 
                                camera.aruco_dict)

    all_corners = []
    all_ids = []
    image_size = None

    success_count = 0

    while True:
        frame = camera.get_latest_frame()
        if frame is None:
            # Wait briefly if frame isn't ready
            cv2.waitKey(100)
            continue

        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers using the detector initialized in ArUcoCamera
        corners, ids, rejected = camera.detector.detectMarkers(gray)

        display_frame = frame.copy()

        if len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
            cv2.putText(display_frame, f"Found {len(corners)} markers. Press 'c' to capture.", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "Looking for markers...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display_frame, f"Collected: {success_count}/{args.count}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow('Calibration', display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c') and len(corners) > 0:
            all_corners.append(corners)
            all_ids.append(ids)
            success_count += 1
            print(f"Captured frame {success_count}/{args.count} with {len(corners)} markers")

            if success_count >= args.count:
                print("Collected enough frames. Calibrating...")
                break

    camera.release()
    cv2.destroyAllWindows()

    if success_count > 0:
        print("Calibrating camera...")
        try:
            # Prepare data for calibrateCameraAruco
            # calibrateCameraAruco needs: corners (list or tuple of numpy arrays), ids (numpy array), counter (numpy array)
            flat_corners = []
            flat_ids = []
            counter = []
            for corners, ids in zip(all_corners, all_ids):
                flat_corners.extend(corners)
                flat_ids.extend(ids)
                counter.append(len(ids))
                
            flat_corners = tuple(flat_corners)
            flat_ids = np.vstack(flat_ids)
            counter = np.array(counter)
            
            ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraAruco(
                flat_corners, flat_ids, counter, board, image_size, None, None
            )
            
            print("Calibration finished.")
            print(f"Camera matrix:\n{mtx}")
            print(f"Distortion coefficients:\n{dist}")

            np.save('calibration_matrix.npy', mtx)
            np.save('distortion_coefficients.npy', dist)
            print("Saved to calibration_matrix.npy and distortion_coefficients.npy")
        except Exception as e:
            print(f"Calibration failed: {e}")
    else:
        print("Not enough frames captured.")

if __name__ == '__main__':
    main()
