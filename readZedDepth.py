import pyzed.sl as sl
import cv2
import numpy as np

def main():
    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters for the camera
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA  # HD720 resolution (default fps: 60)
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # High quality depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Depth in meters
    init_params.camera_fps = 30  # Limit the camera FPS to 30 to reduce computational load

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Error:", err)
        exit()

    # Create an OpenCV window to display the depth map
    cv2.namedWindow("Depth Map", cv2.WINDOW_NORMAL)

    # Main loop
    while True:
        # Capture a new frame
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the depth map
            depth_data = sl.Mat()
            zed.retrieve_measure(depth_data, sl.MEASURE.DEPTH)

            # Convert the depth map to a numpy array
            depth_image_np = depth_data.get_data()

            #print(depth_image_np)
            #cv2.imshow("Depth Map", depth_image_np)
            # Normalize the depth values for better visualization
            depth_image_np_filtered = np.nan_to_num(depth_image_np, nan=0.0, posinf=20.0, neginf=0.0)
            depth_image_np_normalized = cv2.normalize(depth_image_np_filtered, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            #depth_image_np = cv2.normalize(depth_image_np, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Display the depth map
            cv2.imshow("Depth Map", depth_image_np_normalized)

        # Check for key press and exit if 'q' is pressed
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # Close the camera
    zed.close()

    # Close all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
