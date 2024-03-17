#!/usr/bin/env python3

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from detection_msgs.msg import BoundingBox, BoundingBoxes, Coordinate, Coordinates
from geometry_msgs.msg import PoseArray, Pose

from geometry_msgs.msg import TransformStamped
import tf2_ros

# add yolov5 submodule to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        self.center = []
        self.floor = ['1', '2', '3', '4', '5', '6']
        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        camera_info_type, camera_info_topic, _ = get_topic_type(rospy.get_param("~camera_info"), blocking = True)
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        input_depth_image_type, input_depth_image_topic, _ = get_topic_type(rospy.get_param("~input_depth_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        self.camera_info = rospy.Subscriber(
            camera_info_topic, CameraInfo, self.info_callback, queue_size=1)

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )

        self.depth_image_sub = rospy.Subscriber(
            input_depth_image_topic, Image, self.dep_callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )

        # Initialize coordinate publisher
        self.coords_pub = rospy.Publisher("yolov5/coordinates", Coordinates, queue_size=10)

        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        self.tf_pub = rospy.Publisher("/tf_yolov5", PoseArray, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def info_callback(self, camera_info_msg):
        # Camera info
        self.fx = camera_info_msg.K[0]
        self.fy = camera_info_msg.K[4]
        self.cx = camera_info_msg.K[2]
        self.cy = camera_info_msg.K[5]
        self.frame_id = camera_info_msg.header.frame_id
        rospy.loginfo_once("fx: {0}, fy: {1}, cx: {2}, cy: {3}, frame: {4}.".format(self.fx, self.fy, self.cx, self.cy, self.frame_id))

    def dep_callback(self, data):
        """3D coordinate"""
        dep_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        coords = Coordinates()
        coords.header = data.header
        coords.image_header = data.header

        tf_msg = PoseArray()
        tf_msg.header.frame_id = self.frame_id

        distances = []
        floor_list = []

    
        if len(self.center):
            for cls, x, y in self.center:
                coord = Coordinate()
                pose = Pose()
                depth = dep_img[y, x]

                if depth == 0:
                    continue
                
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = rospy.Time.now()
                transform_stamped.header.frame_id = self.frame_id
                transform_stamped.child_frame_id = cls

                transform_stamped.transform.translation.x = int(round(depth * (x - self.cx) / self.fx, 0)) / 1000
                transform_stamped.transform.translation.y = int(round(depth * (y - self.cy) / self.fy, 0)) / 1000
                transform_stamped.transform.translation.z = depth / 1000

                transform_stamped.transform.rotation.x = 0
                transform_stamped.transform.rotation.y = 0
                transform_stamped.transform.rotation.z = 0
                transform_stamped.transform.rotation.w = 1

                self.tf_broadcaster.sendTransform(transform_stamped)

                coord.Class = cls
                coord.point.header.frame_id = self.frame_id
                coord.point.point.x = int(round(depth * (x - self.cx) / self.fx, 0)) / 1000
                coord.point.point.y = int(round(depth * (y - self.cy) / self.fy, 0)) / 1000
                coord.point.point.z = depth / 1000
                                
                if cls in self.floor:
                    floor_class = [cls, (coord.point.point.x, coord.point.point.y, coord.point.point.z)]
                    floor_list.append(floor_class)
                

                pose.position.x = int(round(depth * (x - self.cx) / self.fx, 0)) / 1000
                pose.position.y = int(round(depth * (y - self.cy) / self.fy, 0)) / 1000
                pose.position.z = depth / 1000
                pose.orientation.x = 0
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1

                coords.Coordinates.append(coord)
                tf_msg.poses.append(pose)


            # if len(floor_list) > 2:
            #     for cls, point in floor_list:
            #         for i in range(len(floor_list)):
            #             for j in range(i + 1, len(floor_list)):
            #                 cls1, point1 = floor_list[i]
            #                 cls2, point2 = floor_list[j]
            #                 distance = np.sqrt(np.abs(point1[0] - point2[0])**2 + np.abs(point1[1] - point2[1])**2 + np.abs(point1[2] - point2[2])**2)
            #                 distances.append(((cls1, cls2), distance))
            #                 min_distance = min(distances, key=lambda x: x[1])

            #                 if min_distance[1] == distance:
            #                     min_cls1 = cls1
            #                     min_cls2 = cls2
            #                     min_point1 = point1
            #                     min_point2 = point2
                
        #         # print("min_cls1:  ", min_cls1, "    min_point1:  ", min_point1, "   min_cls2:  ", min_cls2, "   min_point2:  ", min_point2, "   distance:  ", min_distance[1])
                # distance_x = np.abs(min_point1[0] - min_point2[0])
                # distance_y = np.abs(min_point1[1] - min_point2[1])
                # distance_z = np.abs(min_point1[2] - min_point2[2])

                # if min_point1[1] > min_point2[1]:
                #     ref_cls = int(min_cls1)
                #     ref_point = min_point1

                # elif min_point1[1] < min_point2[1]:
                #     ref_cls = int(min_cls2)
                #     ref_point = min_point2
                # else:
                #     rospy.logerr("min_point error")

        #         # print("ref_cls:  ", ref_cls, "    ref_point:  ", ref_point, "    x:  ", distance_x, "   y:  ", distance_y, "    z:  ", distance_z)

                # for button in range(len(self.floor)):
                #     pose = Pose()
                #     offset = button - ref_cls + 1
                #     pose.position.x = ref_point[0] + (offset * distance_x)
                #     pose.position.y = ref_point[1] + (offset * distance_y)
                #     pose.position.z = ref_point[2] + (offset * distance_z)
                #     pose.orientation.x = 0
                #     pose.orientation.y = 0
                #     pose.orientation.z = 0
                #     pose.orientation.w = 1
                #     tf_msg.poses.append(pose)
        #             # print ("button:  ", button, "   ref_cls:  ", ref_cls, "     offset:  ", offset)
        self.coords_pub.publish(coords)
        self.tf_pub.publish(tf_msg)
        
    def callback(self, data):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        im, im0 = self.preprocess(im)
        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        ### To-do move pred to CPU and fill BoundingBox messages
        
        # Process predictions 
        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        self.center = []
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                bounding_box = BoundingBox()
                c = int(cls)
                # Fill in bounding box message
                bounding_box.Class = self.names[c]
                bounding_box.probability = conf 
                bounding_box.xmin = int(xyxy[0])
                bounding_box.ymin = int(xyxy[1])
                bounding_box.xmax = int(xyxy[2])
                bounding_box.ymax = int(xyxy[3])

                cx = int(round((int(xyxy[0]) + int(xyxy[2])) / 2))
                cy = int(round((int(xyxy[1]) + int(xyxy[3])) / 2))
                center_point = [self.names[c], cx, cy]
                self.center.append(center_point)
                bounding_boxes.bounding_boxes.append(bounding_box)

                # Annotate the image
                if self.publish_image or self.view_image:  # Add bbox to image
                      # integer class
                    label = f"{self.names[c]} {conf:.2f}"
                    annotator.box_label(xyxy, label, color=colors(c, True))       

                
                ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            im0 = annotator.result()

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)

        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 

    def correct_misrecognized_objects(self, boxes):
        filtered_boxes = []

        for cls, point in boxes:
            if cls in self.floor:

                if cls in [box_cls for box_cls, _ in filtered_box]:
                    temp = []

                    # temp: 같은 class를 가진 box 리스트
                    temp.append([cls, point])
                    for i in filtered_boxes:
                        if i[0] == cls:
                            temp.append(i)
                            filtered_boxes.remove(i)

                    # temp를 point.y를 기준으로 오름차순 정렬
                    sorted_temp = sorted(temp, key=lambda x: x[1][1], reverse=False)

                    # 순서대로 class 1씩 증가
                    k = 0
                    for cls, point in sorted_temp:
                        filtered_box = [str(int(cls) + k) + 'm', point]
                        filtered_boxes.append(filtered_box)
                        k += 1

            else:
                non_filter = [cls, point]
                filtered_boxes.append(non_filter)

        return filtered_boxes


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()
