import pyzed.sl as sl
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    # Initialize ROS node
    rospy.init_node('zed_camera_publisher')

    # Create publishers for depth map and left image topics
    depth_pub = rospy.Publisher('/zed/depth', Image, queue_size=10)
    left_image_pub = rospy.Publisher('/zed/left_image', Image, queue_size=10)

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
        rospy.logerr("Error opening ZED camera: %s" % sl.ERROR_CODE.to_string(err))
        rospy.signal_shutdown("Error opening ZED camera")
        return

    # Create a CvBridge
    bridge = CvBridge()

    # Main loop
    while not rospy.is_shutdown():
        # Capture a new frame
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the depth map
            depth_data = sl.Mat()
            zed.retrieve_measure(depth_data, sl.MEASURE.DEPTH)
            depth_image_np = depth_data.get_data()

            # Retrieve the left image
            left_image = sl.Mat()
            zed.retrieve_image(left_image, sl.VIEW.LEFT)
            left_image_np_rgba = left_image.get_data()
            left_image_np_bgr = cv2.cvtColor(left_image_np_rgba, cv2.COLOR_RGBA2BGR)
            # Convert BGR image to ROS Image message
            left_image_msg = bridge.cv2_to_imgmsg(left_image_np_bgr, encoding="bgr8")


            # Convert depth map and left image to ROS Image messages
            depth_image_msg = bridge.cv2_to_imgmsg(depth_image_np, encoding="passthrough")


            # Publish depth map and left image
            depth_pub.publish(depth_image_msg)
            left_image_pub.publish(left_image_msg)

    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()
