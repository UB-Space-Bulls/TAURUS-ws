#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection, Point
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

class ZedArucoDetector(Node):
    def __init__(self):
        super().__init__('zed_aruco_detector')
        self.marker_size = 0.15
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.det = cv2.aruco.ArucoDetector(self.aruco_dict) if hasattr(cv2.aruco, 'ArucoDetector') else None
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Image, '/zed/zed_node/rgb/color/rect/image', self.img_cb, qos)
        self.create_subscription(CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info', self.info_cb, qos)
        self.det_pub = self.create_publisher(AprilTagDetectionArray, '/detections', 10)
        self.img_pub = self.create_publisher(Image, '/aruco/image_detections', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.br, self.k, self.d = CvBridge(), None, None

    def info_cb(self, msg):
        if self.k is None:
            self.k = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.d = np.array(msg.d, dtype=np.float64)

    def img_cb(self, msg):
        f = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        c, ids, _ = self.det.detectMarkers(g) if self.det else cv2.aruco.detectMarkers(g, self.aruco_dict)
        arr = AprilTagDetectionArray(header=msg.header)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(f, c, ids)
            obj = np.array([[-0.075,0.075,0],[0.075,0.075,0],[0.075,-0.075,0],[-0.075,-0.075,0]], dtype=np.float32)
            for i, mid in enumerate(ids.flatten()):
                pts = c[i][0]
                t = AprilTagDetection(family='4x4_50', id=int(mid))
                t.centre = Point(x=float(np.mean(pts[:,0])), y=float(np.mean(pts[:,1])))
                t.corners = [Point(x=float(p[0]), y=float(p[1])) for p in pts]
                arr.detections.append(t)
                if self.k is not None:
                    ok, rv, tv = cv2.solvePnP(obj, pts, self.k, self.d)
                    if ok:
                        dist = float(np.linalg.norm(tv))
                        cv2.putText(f, f'ID:{mid} Dist:{dist:.2f}m', (int(pts[0][0]), int(pts[0][1])-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                        self.get_logger().info(f'Detected ID: {mid} | Z: {tv[2][0]:.3f}m | Dist: {dist:.3f}m')
        self.det_pub.publish(arr)
        out = self.br.cv2_to_imgmsg(f, encoding='bgr8')
        out.header = msg.header
        self.img_pub.publish(out)

if __name__ == '__main__':
    rclpy.init()
    rclpy.spin(ZedArucoDetector())
