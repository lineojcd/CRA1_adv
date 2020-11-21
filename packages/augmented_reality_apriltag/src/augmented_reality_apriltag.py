#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2
from renderClass import Renderer

import rospy
import yaml
import sys
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from dt_apriltags import Detector
from image_geometry import PinholeCameraModel
import rospkg 


"""

This is a template that can be used as a starting point for the CRA1 exercise.
You need to project the model file in the 'models' directory on an AprilTag.
To help you with that, we have provided you with the Renderer class that render the obj file.

"""

class ARNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ARNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh_name = rospy.get_namespace().strip("/")

        rospack = rospkg.RosPack()
        # Initialize an instance of Renderer giving the model in input.
        self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')

        #
        #   Write your code here
        #
        self.bridge = CvBridge()
        self.camera_info = rospy.wait_for_message(f'/{self.veh_name}/camera_node/camera_info', CameraInfo)
        self._K = np.array(self.camera_info.K).reshape((3,3))
        self.dist = np.array(self.camera_info.D[0:4])

        # self.calibration_file = f'/data/config/calibrations/camera_extrinsic/{self.veh_name}.yaml'
        self.calibration_file_extrin = ("/code/catkin_ws/src/cra1_adv/calibrations/camera_extrinsic/" + self.veh_name + ".yaml")
        self.calibration_file_intrin = ("/code/catkin_ws/src/cra1_adv/calibrations/camera_intrinsic/" + self.veh_name + ".yaml")

        self._H = np.array(self.readYamlFile(self.calibration_file_extrin)['homography']).reshape((3,3))
        self._InvH = np.linalg.inv(self._H)
        self.camera_params = (self._InvH[0,0], self._InvH[1,1], self._InvH[0,2],self._InvH[1,2])

        # Load homography
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)
        self.pcm_ = PinholeCameraModel()

        # Load camera parameter
        cam_info = self.load_camera_info()
        self.pcm_.width = cam_info.width
        self.pcm_.height = cam_info.height
        self.pcm_.K = cam_info.K
        self.pcm_.D = cam_info.D
        self.pcm_.R = cam_info.R
        self.pcm_.P = cam_info.P

        #Load camera resolution
        self.res_w = 640
        self.res_h = 480
        rospy.set_param('/%s/camera_node/exposure_mode' %(self.veh_name) , 'off')
        rospy.set_param('/%s/camera_node/res_w' %(self.veh_name), self.res_w)
        rospy.set_param('/%s/camera_node/res_h' %(self.veh_name), self.res_h)

        # april tag
        self.tag_size = 0.065
        self.iter = 0
        self.at_detector = Detector(searchpath=['apriltags'],
                                    families='tag36h11',
                                    nthreads=1,
                                    quad_decimate=2.0,
                                    quad_sigma=0.0,
                                    refine_edges=1,
                                    decode_sharpening=0.25,
                                    debug=0)

        # Initialize ROS topics
        self.pub = rospy.Publisher("/{}/augmented_reality_apriltag/image/compressed".format(self.veh_name),
                                   CompressedImage, queue_size=1)
        rospy.loginfo("Publishing augmented reality on apriltag")
        self.sub_image = rospy.Subscriber("/{}/camera_node/image/compressed".format(self.veh_name),
                                          CompressedImage, self.callback, queue_size=1)

    def projection_matrix(self, intrinsic, homography):
        """
            Write here the compuatation for the projection matrix, namely the matrix
            that maps the camera reference frame to the AprilTag reference frame.
        """

        #
        # Write your code here
        # normalize wrt rl
        InvK = np.linalg.inv(intrinsic)
        _homography = np.dot(InvK, homography)
        norm_r1 = np.linalg.norm(_homography[:,0], ord = 2)
        H_norm = np.array(_homography/ norm_r1)

        # get R
        R = np.column_stack((H_norm[:,0],H_norm[:,1],np.cross(H_norm[:,0],H_norm[:,1])))

        # polar decomp R
        u,s,vt = np.linalg.svd(R)
        _R = np.dot(u,vt)
        _R_T = np.c_[_R, H_norm[:,2]]
        _K_R_T = np.dot(intrinsic,_R_T)
        return _K_R_T



    def readImage(self, msg_image):
        """
            Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []

    def load_homography(self):
        '''Load homography (extrinsic parameters)'''
        # filename = ("/code/catkin_ws/src/cra1_adv/calibrations/camera_extrinsic/"+ self.veh_name + ".yaml")
        rospy.loginfo("Using extrinsic calibration of " + self.veh_name)
        data = self.readYamlFile(self.calibration_file_extrin)
        rospy.loginfo("Loaded homography for {}".format(self.veh_name))
        return np.array(data['homography']).reshape((3,3))

    def load_camera_info(self):
        '''Load camera intrinsics'''
        # filename = ("/code/catkin_ws/src/cra1_adv/calibrations/camera_intrinsic/" + self.veh_name + ".yaml")
        calib_data = self.readYamlFile(self.calibration_file_intrin)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = np.array(calib_data['camera_matrix']['data']).reshape((3,3))
        cam_info.D = np.array(calib_data['distortion_coefficients']['data']).reshape((1,5))
        cam_info.R = np.array(calib_data['rectification_matrix']['data']).reshape((3,3))
        cam_info.P = np.array(calib_data['projection_matrix']['data']).reshape((3,4))
        cam_info.distortion_model = calib_data['distortion_model']
        rospy.loginfo("Loaded camera calibration parameters for {}".format(self.veh_name))
        return cam_info

    def process_image(self, cv_image_raw):
        '''Undistort image'''
        cv_image_rectified = np.zeros(np.shape(cv_image_raw))
        mapx = np.ndarray(shape=(self.pcm_.height, self.pcm_.width, 1), dtype='float32')
        mapy = np.ndarray(shape=(self.pcm_.height, self.pcm_.width, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.pcm_.K, self.pcm_.D, self.pcm_.R, self.pcm_.P, (self.pcm_.width, self.pcm_.height), cv2.CV_32FC1, mapx, mapy)
        return cv2.remap(cv_image_raw, mapx, mapy, cv2.INTER_CUBIC, cv_image_rectified)

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def callback(self, data):
        #Convert compressed image to BGR
        # np_arr = np.fromstring(data.data, np.uint8)
        # img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.readImage(data)
        img_undistorted = self.process_image(img)
        img_gray = cv2.cvtColor(img_undistorted,cv2.COLOR_RGB2GRAY)

        # TODO step 1: Detect the AprilTag and extract its reference frame.

        # Detection of tags in images is done by running the "detect" method of the detector
        # Run detectons on the provided image. The image must be a grayscale image of type numpy.uint8.'''

        # If you also want to extract the tag pose, estimate_tag_pose should be set to True
        # and camera_params ([fx, fy, cx, cy]) and tag_size (in meters) should be supplied.

        # tags = self.at_detector.detect(img_gray, estimate_tag_pose=True, camera_params=None, tag_size=None)
        tags = self.at_detector.detect(img_gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size)

        # The detect method returns a list of Detection objects:
        # https://github.com/duckietown/lib-dt-apriltags
        # each having the following attributes (note that the ones with an asterisks are
        # computed only if estimate_tag_pose=True):
        #  homography: The 3x3 homography matrix describing the projection from an "ideal" tag
        #               (with corners at (-1,1), (1,1), (1,-1), and (-1, -1)) to pixels in the image.
        #  center: The center of the detection in image pixel coordinates.
        #  corners:	The corners of the tag in image pixel coordinates.
        #           These always wrap counter-clock wise around the tag.
        #  pose_R*	Rotation matrix of the pose estimate.
        #  pose_t*	Translation of the pose estimate.

        for tag in tags:
            # TODO step 2: Estimate the homography matrix: determine the transformation from the reference
            # frame of the AprilTag (2D) to the reference frame of the target image (2D).
            # This transformation is the homography matrix.
            # TODO step 4: Project our 3D model in the image (pixel space) and draw it
            # self.renderer.render(img_undistorted,self.projection_matrix(self.mtx, np.array(tag.homography)))
            img_undistorted = self.renderer.render(img_undistorted,self.projection_matrix(self._K, np.array(tag.homography)))

        if self.iter == 0:
            if len(tags) > 0:
                cv2.imwrite("/code/catkin_ws/src/cra1_adv/calibrations/testapriltag_imgs.png", img_undistorted)
                print("Saved the first testpic", self.iter)
                self.iter+=1

        compressed_img = self.bridge.cv2_to_compressed_imgmsg(img_undistorted, dst_format='jpg')
        # publish info
        self.pub.publish(compressed_img)


    def onShutdown(self):
        super(ARNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    camera_node = ARNode(node_name='augmented_reality_apriltag_node')
    # Keep it spinning to keep the node alive
    rospy.spin()