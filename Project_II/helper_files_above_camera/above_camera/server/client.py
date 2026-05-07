import sys
import cv2
from vision import ArUcoCamera

def main():
    if len(sys.argv) < 2:
        print("Usage: python client.py <ip_address_of_pi> [marker_size_mm]")
        print("Example: python client.py 192.168.1.50 50")
        sys.exit(1)

    ip_address = sys.argv[1]
    marker_size = float(sys.argv[2]) if len(sys.argv) > 2 else 50.0
    rtsp_url = f"rtsp://{ip_address}:8554/cam2"

    print(f"Connecting to {rtsp_url}...")
    
    try:
        camera = ArUcoCamera(rtsp_url, marker_size_mm=marker_size)
    except Exception as e:
        print(f"Error initializing camera: {e}")
        sys.exit(1)

    print("Press 'q' to quit.")

    while True:
        # Get processed frame and marker data
        frame, markers = camera.get_marker_positions()
        
        if frame is None:
            print("Failed to get frame.")
            break

        # Log marker data (optional, just to show we have it)
        if markers:
            print(f"Detected markers: {list(markers.keys())}")
            for i in markers.keys() :
            # Example access: markers[id]['tvec']
                print(markers[i]['tvec'])
                print(markers[i]['rvec'])

        cv2.imshow('RTSP Camera Stream', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
