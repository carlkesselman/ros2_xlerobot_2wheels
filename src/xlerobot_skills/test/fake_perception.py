#!/usr/bin/env python3
"""Stands in for the object perception service in tests: answers /perception/detect from a scripted scene.

The scene is a JSON message on /fake_perception/scene (latched): {"objects": [[x, y, z], ...], "codes": [...]},
positions in base_footprint (the centre of each object). An empty object list answers NO_DET; codes such as BUSY are
returned as they are.
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose
from xlerobot_object_perception.srv import DetectObjects


class FakePerception(Node):
    def __init__(self):
        super().__init__("fake_perception")
        self.scene = {"objects": [], "codes": []}
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/fake_perception/scene", self._on_scene, latched)
        self.create_service(DetectObjects, "/perception/detect", self._on_detect)

    def _on_scene(self, m):
        self.scene = json.loads(m.data)

    def _on_detect(self, req, res):
        size = (req.size_mm or 28.0) / 1000.0
        res.detections.header.frame_id = "base_footprint"
        res.detections.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y, z) in enumerate(self.scene.get("objects", [])):
            d = Detection3D()
            d.header = res.detections.header
            d.id = f"{req.label or req.prompt}_{i}"
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = req.label or req.prompt, 0.9
            h.pose.pose.position.x, h.pose.pose.position.y, h.pose.pose.position.z = float(x), float(y), float(z)
            h.pose.pose.orientation.w = 1.0
            d.results.append(h)
            d.bbox.center = h.pose.pose
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = size
            res.detections.detections.append(d)
            res.heights_m.append(size)
        res.codes = list(self.scene.get("codes", [])) or ([] if res.detections.detections else ["NO_DET"])
        res.success = bool(res.detections.detections)
        return res


def main():
    rclpy.init()
    rclpy.spin(FakePerception())


if __name__ == "__main__":
    main()
