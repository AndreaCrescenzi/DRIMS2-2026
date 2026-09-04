import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply

from easy_motion_msgs.srv import DiceIdentification
from geometry_msgs.msg import PoseStamped, TransformStamped


class DiceDetector(Node):
    def __init__(self):
        super().__init__('dice_detector')

        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Parameters
        self.declare_parameter("z", 0.59)
        self.declare_parameter("height_dice", 0.02)

        self.declare_parameter("smoothing_alpha", 0.3)
        self.declare_parameter("face_history_len", 7)

        self.declare_parameter("hue_outer_range", [35, 170])
        self.declare_parameter("saturation_threshold", 140)
        self.declare_parameter("value_threshold", 200)

        self.declare_parameter("dice_aspect_ratio_threshold", [0.5, 2.0])
        self.declare_parameter("dice_area_threshold", 500)

        self.declare_parameter("dot_circularity_threshold", [0.75, 1.25])
        self.declare_parameter("dot_area_threshold", [20, 1000])
        self.declare_parameter("kernel_size", (7, 7))

        self.Z = self.get_parameter("z").value
        self.height_dice = self.get_parameter("height_dice").value

        self.H = self.get_parameter("hue_outer_range").value
        self.S = self.get_parameter("saturation_threshold").value
        self.V = self.get_parameter("value_threshold").value

        self.dice_aspect_ratio_threshold = self.get_parameter("dice_aspect_ratio_threshold").value
        self.dice_area_threshold = self.get_parameter("dice_area_threshold").value
        
        self.dot_circularity_threshold = self.get_parameter("dot_circularity_threshold").value
        self.dot_area_threshold = self.get_parameter("dot_area_threshold").value

        kernel_size = self.get_parameter("kernel_size").value

        self.alpha = self.get_parameter("smoothing_alpha").value
        self.face_history_len = self.get_parameter("face_history_len").value

        self.K = None #np.eye(3)        
        self.d = None #np.zeros((8,1))  
        self.closure_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
        
        self.moving_average = (0, 0)
        self.smoothed_center = None      # np.array([x, y])
        self.smoothed_quat = None        # np.array([qz, qw])  -> continuo, niente discontinuità
        self.face_history = []

        self.face_number = 0
        self.dice_pose = []
        self.frame_id = ""
        
        # Bridge OpenCV <-> ROS
        self.bridge = CvBridge()

        # Subscriber
        self.frame_sub = self.create_subscription(
            CompressedImage,
            '/oak/rgb/image_raw/compressed',
            self.frame_callback,
            10)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.info_callback,
            10)


        # Publisher
        self.dice_mask_pub = self.create_publisher(Image, '/dice_detector/dice_mask', 10)
        self.dice_img_pub = self.create_publisher(Image, '/dice_detector/dice_img', 10)
        self.board_img_pub = self.create_publisher(Image, '/dice_detector/board_img', 10)

        # Service
        self.srv = self.create_service(DiceIdentification, 'dice_identification', self.handle_service)

        self.get_logger().info("Dice Detector node started")

    def smooth_value(self, prev, new, alpha):
        new = np.asarray(new, dtype=float)
        if prev is None:
            return new
        return alpha * new + (1 - alpha) * prev

    def info_callback(self, msg):
        self.get_logger().info("Camera info received")
        self.K = np.array(msg.k).reshape(3, 3)
        self.d = np.array(msg.d).reshape(8,1)
        self.frame_id = msg.header.frame_id
        self.info_sub.destroy()
        self.get_logger().info("Camera info done")

    def get_dice(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        H, S, V = cv2.split(hsv)

        not_green = (H <= self.H[0]) | (H >= self.H[1]) # exclude green
        not_black = V >= self.V                         # exclude dark and shadows
        not_white = S >= self.S                         # exclude white and oversaturation

        mask = (not_green & not_black & not_white).astype(np.uint8) * 255

        # small_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, small_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.closure_kernel)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        [aspect_min, aspect_max] = self.dice_aspect_ratio_threshold

        candidates = []
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            if area < self.dice_area_threshold:
                continue
            aspect = w / float(h)
            if not (aspect_min <= aspect <= aspect_max):   
                continue
            candidates.append({
                "area": area, 
                "x": x, 
                "y": y, 
                "w": w, 
                "h": h, 
                "contour": c})

        if not candidates:
            return None, None, None

        dice_info = max(candidates, key=lambda t: t["area"])
        [area, x, y, w, h, dice_contour] = dice_info.values()
        dice = frame[y:y+h, x:x+w]
        dice_mask = mask[y:y+h, x:x+w]

        return dice, dice_mask, dice_info

    def get_dots(self, dice_mask):
        dot_contours, _ = cv2.findContours(dice_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        dots = []

        [dot_area_min, dot_area_max] = self.dot_area_threshold
        [circularity_min, circularity_max] = self.dot_circularity_threshold

        for contour in dot_contours:
            # if len(contour) >= 5:                     # fitEllipse richiede >=5 punti
            #     (cx, cy), (MA, ma), angle = cv2.fitEllipse(contour)
            #     ratio = min(MA, ma) / max(MA, ma)     # 1.0 = cerchio, basso = ellisse (faccia laterale)
            #     if ratio < 0.7:
            #         continue
            area = cv2.contourArea(contour)
            if area < dot_area_min or area > dot_area_max:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue

            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if not (circularity_min <= circularity <= circularity_max):
                continue

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]

            dots.append({
                "cx": cx,
                "cy": cy,
                "contour": contour
            })

        if dots:
            cx_dice, cy_dice = tuple(np.mean([(dot["cx"], dot["cy"]) for dot in dots], axis=0))
        else:
            return None, None, None
        return dots, cx_dice, cy_dice

    def get_dice_pose(self, cnt, cx_dice, cy_dice):
        fx = self.K[0,0]; fy = self.K[1,1]
        cx = self.K[0,2]; cy = self.K[1,2]

        X = (cx_dice - cx) * self.Z / fx
        Y = (cy_dice - cy) * self.Z / fy

        rect = cv2.minAreaRect(cnt)
        _, _, angle = rect

        yaw = angle % 90
        if yaw >= 45:
            yaw -= 90
        theta = np.deg2rad(yaw) 

        qw = np.cos(theta / 2)
        qx = 0.0
        qy = 0.0
        qz = np.sin(theta / 2)

        return rect, theta, [cx_dice, cy_dice], [X, Y, self.Z, qx, qy, qz, qw]

    def frame_callback(self, msg):
        while self.K is None and self.d is None:
            self.get_logger().info("Waiting for camera info...")
            #rclpy.spin_once(self, timeout_sec=0.1)
            return 
            
        #self.get_logger().info('Starting processing')

        #frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # h, w, _ = frame.shape

        img = cv2.undistort(frame, self.K, self.d)
        img = frame.copy()

        dice_img, dice_mask, dice_info = self.get_dice(frame)
        
        if dice_img is None:
            self.get_logger().warn("No dice detected in the frame.")
            return

        img_msg = self.bridge.cv2_to_imgmsg(dice_mask, encoding='mono8')
        img_msg.header = msg.header
        self.dice_mask_pub.publish(img_msg)

        dots, cx_dice, cy_dice = self.get_dots(dice_mask)

        if dots is None:
            self.get_logger().warn("No dots detected, cannot compute center.")
            return

        cv2.drawContours(dice_img, [dot["contour"] for dot in dots], -1, (0, 0, 255), -1)
        cv2.circle(dice_img, (int(cx_dice), int(cy_dice)), 3, (255, 0, 0), -1)
        
        self.face_number = len(dots)

        self.face_history.append(len(dots))
        if len(self.face_history) > self.face_history_len:
            self.face_history.pop(0)
        self.face_number = int(max(set(self.face_history), key=self.face_history.count))

        #rect, angle, self.dice_pose = self.get_dice_pose(dice_info["contour"], cx_dice + dice_info["x"], cy_dice + dice_info["y"])

        rect, theta, dice2board, raw_pose = self.get_dice_pose(
            dice_info["contour"], cx_dice + dice_info["x"], cy_dice + dice_info["y"]
        )
        raw_cx, raw_cy = raw_pose[0], raw_pose[1]
        raw_qz, raw_qw = raw_pose[5], raw_pose[6]
        # --- filtro centro (EMA) ---
        self.smoothed_center = self.smooth_value(
            self.smoothed_center, [raw_cx, raw_cy], self.alpha
        )
        cx_smooth, cy_smooth = self.smoothed_center
        # --- filtro angolo via quaternione (EMA + rinormalizzazione) ---
        self.smoothed_quat = self.smooth_value(
            self.smoothed_quat, [raw_qz, raw_qw], self.alpha
        )
        self.smoothed_quat = self.smoothed_quat / np.linalg.norm(self.smoothed_quat)
        qz_smooth, qw_smooth = self.smoothed_quat
        self.dice_pose = [cx_smooth, cy_smooth, self.Z, 0.0, 0.0, qz_smooth, qw_smooth]
        
        board_img = img.copy()

        box = cv2.boxPoints(rect) 
        box = np.intp(box)
        cv2.drawContours(board_img, [box], 0, (0, 255, 0), 2)
        cv2.circle(board_img, (int(dice2board[0]), int(dice2board[1])), 3, (255, 0, 0), -1)

        #self.get_logger().info(f"Projected center: ({self.dice_pose[0]}, {self.dice_pose[1]})")
        
        img_msg = self.bridge.cv2_to_imgmsg(dice_img, encoding='bgr8')
        img_msg.header = msg.header
        self.dice_img_pub.publish(img_msg)

        board_img_msg = self.bridge.cv2_to_imgmsg(board_img, encoding='bgr8')
        board_img_msg.header = msg.header
        self.board_img_pub.publish(board_img_msg)



    def handle_service(self, request, response):
        # Here you put your detection values
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id

        if not self.dice_pose or self.face_number == 0:
            response.success = False
            self.get_logger().info("Service called -> ERROR: dice not available")
            return response

        pose.pose.position.x = self.dice_pose[0]
        pose.pose.position.y = self.dice_pose[1]
        pose.pose.position.z = self.dice_pose[2]

        pose.pose.orientation.x = self.dice_pose[3]
        pose.pose.orientation.y = self.dice_pose[4]
        pose.pose.orientation.z = self.dice_pose[5]
        pose.pose.orientation.w = self.dice_pose[6]

        response.face_number = self.face_number
        response.pose = pose
        response.success = True

        
        tf_dice = TransformStamped()
        tf_dice.header.stamp = self.get_clock().now().to_msg()
        tf_dice.header.frame_id = self.frame_id
        tf_dice.child_frame_id = "dice_tf"
        tf_dice.transform.translation.x = self.dice_pose[0]
        tf_dice.transform.translation.y = self.dice_pose[1]
        tf_dice.transform.translation.z = self.dice_pose[2]
        tf_dice.transform.rotation.x = self.dice_pose[3]
        tf_dice.transform.rotation.y = self.dice_pose[4]
        tf_dice.transform.rotation.z = self.dice_pose[5]
        tf_dice.transform.rotation.w = self.dice_pose[6]
        
        self.tf_broadcaster.sendTransform(tf_dice)

        self.get_logger().info("Service called -> returning static dice info")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DiceDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()