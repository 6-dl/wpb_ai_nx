#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from wpb_ai_bringup.msg import Bbox
import numpy as np

import sys,os
import time
import argparse

import cv2
import pycuda.autoinit  # This is needed for initializing CUDA driver

from utils.yolo_classes import get_cls_dict
from utils.display import open_window, set_display, show_fps
from utils.visualization import BBoxVisualization

from utils.yolo_with_plugins import TrtYOLO

WINDOW_NAME = 'yolo4_node_window'
conf_th = 0.3

new_image = False
image_filename = "/dev/shm/raw.jpg"
def image_callback(msg):
    # print("Received an image!")
    global image_filename
    image_filename = msg.data
    global new_image
    new_image = True  

def main():
    global new_image
    cls_dict,cls_num = get_cls_dict()

    print("-------------1-yolo-init----------------")
    h = w = 416
    trt_yolo = TrtYOLO("wpb-416", (h, w), cls_num, True)
    vis = BBoxVisualization(cls_dict)

    print("-------------2-ros-init----------------")
    rospy.init_node('yolo_node', anonymous=True)
    bbox_pub = rospy.Publisher('/yolo/bbox', Bbox, queue_size=10)
    image_filename_topic = "/yolo/input_filename"
    rospy.Subscriber(image_filename_topic, String, image_callback)
    predict_pub = rospy.Publisher('/yolo/predict', String, queue_size=10)
    info_pub = rospy.Publisher('/yolo/info', String, queue_size=10)
    time.sleep(1)
    info_msg = "start"
    info_pub.publish(info_msg)

    print("-------------3-start----------------")
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        # print("-------------loop----------------")
        if new_image == True:
            global image_filename
            img = cv2.imread(image_filename)
            # print("-------------new_image == True---------------" +  image_filename)
            if img is None:
                new_image = False
                print("ERR: image is None")
                continue
            #  print("------------detect---------------")
            global conf_th
            boxes, confs, clss = trt_yolo.detect(img, conf_th)
             #  print("------------publish result---------------")
            msg_bbox = Bbox()
            for bb, cf, cl in zip(boxes, confs, clss):
                cl = int(cl)
                cls_name = cls_dict[cl]
                msg_bbox.name.append(cls_name)
                x_min, y_min, x_max, y_max = bb[0], bb[1], bb[2], bb[3]
                msg_bbox.left.append(x_min)
                msg_bbox.right.append(x_max)
                msg_bbox.top.append(y_min)
                msg_bbox.bottom.append(y_max)
                msg_bbox.probability.append(100*cf)
            bbox_pub.publish(msg_bbox)
            # rospy.logwarn("predict complete! ")
            
            # print("-------------show---------------")
            img = vis.draw_bboxes(img, boxes, confs, clss)
            predict_image = "/dev/shm/predict.jpg"
            cv2.imwrite(predict_image, img)
            new_image = False
            predict_pub.publish(predict_image)
        rate.sleep()

    print("-------------end----------------")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
