#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

import cv2
from pupil_apriltags import Detector
from cv_bridge import CvBridge


class TagDetectionNode(Node):
    def __init__(self):
        super().__init__('tag_detection_node')

        # Object for converting from ROS images to opencv for processing
        self.bridge = CvBridge()

        # Apriltag detector object
        self.at_detector = Detector(families='tag36h11',
                            nthreads=1,
                            quad_decimate=1.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)


        self.cam_params = (554.3827128226441,  # fx
                    554.3827128226441,  # fy
                    320.5,  # cx
                    240.5)  # cy
        # Tag size in meters
        self.tag_size = 1.0

        self.visualization = True

        # Define a custom QoS profile to match the one provided by the IMU
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.img_sub = self.create_subscription(Image, "/depth_camera/image_raw", self.imageCallback, qos_profile=qos_profile)

        self.current_tag = 9999

        self.tag_commands = {0: "Full Scan",
                             1: "Wing Scan",
                             2: "Tire Inspection",
                             9999: "Find Next Aircraft"}


    def imageCallback(self, img_msg):
        # Convert from a ROS image message to a cv image
        cv_img = self.bridge.imgmsg_to_cv2(img_msg)

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # Get detected tag objects
        tags = self.at_detector.detect(gray, estimate_tag_pose=True, camera_params=self.cam_params, tag_size=self.tag_size)

        min_dist = 99999
        best_tag = 9999
        for tag in tags:
            for idx in range(len(tag.corners)):
                # Plot the outline of the box using the 4 corners
                cv2.line(
                    cv_img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0), 3
                )

            # Display tag ID
            cv2.putText(
                cv_img,
                str(tag.tag_id),
                org=(
                    tag.corners[0, 0].astype(int) + 10,
                    tag.corners[0, 1].astype(int) + 10,
                ),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.9,
                color=(0, 0, 255),
            )

            # Extract the straight line distance to the tag
            z_dist = tag.pose_t[2][0]

            # If it's closer than others make it the current best tag
            if z_dist < min_dist:
                best_tag = tag.tag_id
                min_dist = z_dist

        # Display current instructions to user
        cv2.putText(
            cv_img,
            self.tag_commands[best_tag],
            org=(
                30,
                30,
            ),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1.2,
            thickness = 2,
            color=(0, 0, 255),
        )

        if self.visualization:
            cv2.imshow("Pupil tags", cv_img)

            k = cv2.waitKey(1)
            if k == ord('q'):  # Hit q to quit
                cv2.destroyAllWindows()
                exit()


def main(args=None):
    rclpy.init(args=args)
    node = TagDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()