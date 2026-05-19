import threading
import time

class ThreadedCamera:
    """
    A class to handle background camera capture and ArUco marker detection.
    This reduces latency by running the frame retrieval process in a separate
    thread, ensuring the latest frame and markers are always available.
    """
    def __init__(self, camera_obj):
        self.camera = camera_obj
        self.frame = None
        self.markers = None
        self.stopped = False
        self.lock = threading.Lock()
        # Start the background thread
        threading.Thread(target=self.update, args=(), daemon=True).start()

    def update(self):
        while not self.stopped:
            f, m = self.camera.get_marker_positions()
            with self.lock:
                self.frame = f
                self.markers = m
            time.sleep(0.01) # Small sleep to avoid CPU pinning

    def get_marker_positions(self):
        with self.lock:
            return self.frame, self.markers

# Usage in script:
# threaded_cam = ThreadedCamera(camera)