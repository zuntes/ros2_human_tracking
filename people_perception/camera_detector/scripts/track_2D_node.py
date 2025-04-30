#!/usr/bin/python3

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from ultralytics import YOLO
from ultralytics.engine.results import Boxes
from ultralytics.utils.plotting import Annotator, colors
from ultralytics.engine.results import Results
from ultralytics.engine.results import Keypoints
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from boxmot import DeepOcSort
from boxmot import OcSort

from yolo_msgs.msg import Point2D
from yolo_msgs.msg import BoundingBox2D
from yolo_msgs.msg import KeyPoint2D
from yolo_msgs.msg import KeyPoint2DArray
from yolo_msgs.msg import Track
from yolo_msgs.msg import TrackArray

from typing import List, Tuple, Dict
import numpy as np

import cv2
import numpy as np
import torch
import supervision as sv

class TrackerNode(Node):

    def __init__(self) -> None:
        super().__init__("tracker_image_node")
        self.get_logger().info(f"Start object tracking in image node")

        # Parameters
        self.declare_parameter("use_ReID", False)
        self.use_ReID = self.get_parameter("use_ReID").value

        package_name = "camera_detector"
        package_directory = get_package_share_directory(package_name)

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=100,
        )

        # CV Bridge
        self.cv_bridge = CvBridge()

        # Cuda init
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print("Device: ", self.device)

        # Get yolo model
        model_name = "yolov8m-pose.pt"
        model = os.path.join(package_directory, "models", model_name)

        # Start Yolo
        self.yolo = YOLO(model)
        self.yolo.fuse()

        # Declare Threshold
        self.threshold = 0.75

        if self.use_ReID:
            self.get_logger().info(f"Using Re-Identification")
            self.tracker = DeepOcSort(
                model_weights=Path("osnet_x0_25_msmt17.pt"),  # which ReID model to use
                device=self.device,
                fp16=False,
                det_thresh=0.5,
                max_age=20,
                iou_threshold=0.4,
            )
        else:
            self.tracker = OcSort(
                det_thresh=0.5,
                max_age=10,
                min_hits=3,
            )

        self.tracks = []

        # Publisher
        self.img_pub = self.create_publisher(Image, "/camera/track_image", 50)
        self._pub = self.create_publisher(
            TrackArray, "/camera/track_2D_array", 50
        )

        # Subscriber
        self.img_raw = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, image_qos_profile
        )

    def image_callback(self, msg: Image):

        # Convert msg to img
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        self.height, self.width, _ = cv_image.shape

        track_msg = TrackArray()
        track_msg.number = 0

        # Get detections
        predicts = self.yolo.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            device=self.device,
            conf=self.threshold,
        )

        if predicts[0] is not None:

            results: Results
            results = predicts[0].cpu()

            # Create detections message
            detections = []

            # Get boxes data
            bbox = self.get_bbox_xyxy(results)

            # Get keypoints
            keypoints = self.get_keypoints(results)

            # Get hypothesis
            hypothesis = self.get_hypothesis(results)

            # Extract detections data for tracking
            for obj in range(len(results)):

                class_id = hypothesis[obj]["class_id"]
                label = hypothesis[obj]["class_name"]
                conf = hypothesis[obj]["score"]
                box = bbox[obj]

                detections.append([box[0], box[1], box[2], box[3], conf, class_id])

            if detections == []:
                dets = np.empty((0, 6))
            else:
                dets = np.array(detections)

            self.tracks = self.tracker.update(dets, cv_image)

            # Start track
            for track in self.tracks:

                x1, y1, x2, y2, track_id, score, class_id, inds = track

                # Calculate the center position and size of the bounding box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                width = x2 - x1
                height = y2 - y1

                aux_msg = Track()

                aux_msg.class_id = int(class_id)
                aux_msg.class_name = hypothesis[int(class_id)]["class_name"]
                aux_msg.score = score
                aux_msg.track_id = int(track_id)

                aux_msg.bbox = BoundingBox2D()
                aux_msg.bbox.center.position.x = center_x
                aux_msg.bbox.center.position.y = center_y
                aux_msg.bbox.size.x = width
                aux_msg.bbox.size.y = height

                aux_msg.keypoints = keypoints[int(inds)]

                track_msg.tracks.append(aux_msg)

            # Publish message
            track_msg.header = msg.header
            track_msg.number = len(self.tracks)
            self._pub.publish(track_msg)

            # Publish image
            frame = self.tracker.plot_results(cv_image, show_trajectories=False)
            frame_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            frame_msg.header = msg.header
            # self.img_pub.publish(frame_msg)

            # Debug image
            self.draw_detections(frame_msg, track_msg)

        else:
            # if no detections got
            image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
            self.img_pub.publish(image)

            print("No human detected")
            self._pub.publish(track_msg)
            return

    def get_bbox_xyxy(self, results: Results):

        detections = []

        for obj in results.boxes:

            # get boxes values
            bbox = obj.xyxy[0]
            x1 = float(bbox[0])
            y1 = float(bbox[1])
            x2 = float(bbox[2])
            y2 = float(bbox[3])

            detections.append([x1, y1, x2, y2])

        return detections

    def get_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes

        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf),
            }

            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def get_keypoints(self, results: Results) -> List[KeyPoint2DArray]:
        keypoint_array_list = []
        points: Keypoints

        for points in results.keypoints:
            keypoint_array = KeyPoint2DArray()
            if points.conf is None:
                continue

            # Choose specific keypoints 
            keypoint_indices = [
                0,  # Nose
                1,  # Left eye
                2,  # Right eye
                3,  # Left ear
                4,  # Right ear
                5,  # Left shoulder
                6,  # Right shoulder
                7,  # Left elbow
                8,  # Right elbow
                9,  # Left wrist
                10, # Right wrist
                11, # Left hip
                12, # Right hip
                13, # Left knee
                14, # Right knee
                15, # Left ankle
                16, # Right ankle
            ]

            for kp_id in keypoint_indices:

                p = points.xy[0][kp_id]
                conf = points.conf[0][kp_id]

                if conf >= self.threshold:

                    point_ = KeyPoint2D()
                    point_.id = kp_id

                    point_.point.x = float(p[0])
                    point_.point.y = float(p[1])
                    point_.score = float(conf)

                    keypoint_array.keypoints.append(point_)

            keypoint_array_list.append(keypoint_array)

        return keypoint_array_list

    def draw_detections(self, img_msg : Image, track_msg: TrackArray) -> None:
        
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg)
        
        ann = Annotator(cv_image)

        for track in track_msg.tracks:
            cv2.circle(
                cv_image,
                (int(track.bbox.center.position.x), int(track.bbox.center.position.y)),
                3,
                (0, 0, 0),
                -1,
                lineType=cv2.LINE_AA
            )
            
            keypoints_msg = track.keypoints
            kp : KeyPoint2D
            for kp in keypoints_msg.keypoints:
                color_k = (
                    [int(x) for x in ann.kpt_color[kp.id - 1]]
                    if len(keypoints_msg.keypoints) == 17
                    else colors(kp.id - 1)
                )

                cv2.circle(
                    cv_image,
                    (int(kp.point.x), int(kp.point.y)),
                    3,
                    color_k,
                    -1,
                    lineType=cv2.LINE_AA,
                )
        
        debug_img = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
        self.img_pub.publish(debug_img)

def main():
    rclpy.init()
    node = TrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
