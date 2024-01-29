#!/usr/bin/env python3
import sys
import unittest
import os

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)

import cv2
import rospy
import numpy as np
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from zed_3D_detection.msg import Box3d
import PySimpleGUI as sg
from src.hmi import HMI


class visualization:
    def __init__(self):
        rospy.init_node("Bounding_Box_Visual", anonymous=True)

        self.bridge = CvBridge()

        rospy.sleep(3)

        # Init image subscribers
        rospy.Subscriber("zed2i/zed_node/rgb/image_rect_color",
                         Image, self.get_image)

        # Init corners subscribers
        rospy.Subscriber("/corners_test", Box3d, self.get_corners_data)

        # Robot status
        rospy.Subscriber("/robot_status", Int16, self.get_robot_status)

        # Publish the type of the box
        self.pub = rospy.Publisher('box_type', String, queue_size=10)

        # Get the calibration parameters
        self.param_fp = rospy.get_param("~param_fp")
        with np.load(self.param_fp + '/E2.npz') as X:
            self.mtx, self.dist, self.Mat, self.tvecs = [
                X[i] for i in ("mtx", "dist", "Mat", "tvec")]

        # Define the color used to visualized
        self.colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                       (0, 255, 255), (255, 0, 255)]
        self.corner_data = None
        self.num = 0  # num of the peach
        self.robot_status = 0

        self.gui = HMI(img_sz=(800, 400))

        self.rate = rospy.Rate(10)

    def get_image(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    def get_robot_status(self, data):
        self.robot_status = data.data

    def get_corners_data(self, data):
        corner_data = []
        try:
            self.data = data.corners_data
            for b in data.corners_data:
                self.corners = np.array(b.data).reshape((8, 3))
                corner_data.append(self.corners)
            self.corner_data = np.array(corner_data) / 100
            self.num = self.corner_data.shape[0]
        except:
            rospy.loginfo(f"Not detect the peach!")

    def visualization(self):
        img = self.cv_image
        for index in range(self.num):
            corner_world = self.corner_data[index]
            corner_camera = self.Mat @ (corner_world.T) + self.tvecs
            corner_image = (self.mtx @ corner_camera).T
            corner = corner_image[:, :2] / corner_image[:, 2:3]
            corner = corner.astype(int)

            corner1 = corner[:4, :]
            corner2 = corner[4:8, :]
            pt1 = corner1.reshape((-1, 1, 2))
            pt2 = corner2.reshape((-1, 1, 2))

            color = self.colors[index]
            thickness = 2
            cv2.polylines(img, [pt1], True, color, thickness)
            cv2.polylines(img, [pt2], True, color, thickness)
            for i, j in zip(range(4), range(4, 8)):
                cv2.line(img, tuple(corner[i]), tuple(
                    corner[j]), color, thickness)

            # # option 2 drawing
            index1 = [1, 0, 4, 5]
            index2 = [0, 3, 7, 4]
            index3 = [2, 3, 7, 6]
            index4 = [1, 2, 6, 5]
            zero1 = np.zeros((img.shape), dtype=np.uint8)
            zero2 = np.zeros((img.shape), dtype=np.uint8)
            zero3 = np.zeros((img.shape), dtype=np.uint8)
            zero4 = np.zeros((img.shape), dtype=np.uint8)
            zero_mask1 = cv2.fillConvexPoly(zero1, corner[index1, :], color)
            zero_mask2 = cv2.fillConvexPoly(zero2, corner[index2, :], color)
            zero_mask3 = cv2.fillConvexPoly(zero3, corner[index3, :], color)
            zero_mask4 = cv2.fillConvexPoly(zero4, corner[index4, :], color)
            zeros_mask = np.array(
                (zero_mask1 + zero_mask2 + zero_mask3 + zero_mask4))

            alpha = 1
            beta = 0.55
            gamma = 0
            img = cv2.addWeighted(img, alpha, zeros_mask, beta, gamma)
        cv2.imshow("Image", img)
        cv2.waitKey(5)

    def run(self):
        rospy.on_shutdown(self.gui.quit())
        rospy.sleep(5)
        box_types = ['Type1','Type2', 'Type3']

        self.gui.WaitingLED()
        while not rospy.is_shutdown():

            event, values = self.gui.window.read(timeout=10)
            if event == sg.WIN_CLOSED or event == 'Exit':
                break
            elif event =="Go":
                self.gui.RunningLED()
                for box_type in box_types:
                    if values[box_type] is True:
                        break
                self.pub.publish(box_type)
                try:
                    self.visualization()
                except:
                    rospy.loginfo("Have not detected the peach!")
    
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = visualization()
        node.run()
    except rospy.ROSInterruptException:
        pass

    cv2.destroyAllWindows()
    rospy.loginfo("Exiting camera Extrinsic calibration node!")
