#!/usr/bin/env python3
import threading
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from flask import Flask, Response, redirect
from pupil_apriltags import Detector
import numpy as np
import time  # 추가

app = Flask(__name__)
frame_lock = threading.Lock()
latest_frame = None

class AprilTagNode(Node):
    def __init__(self):
        super().__init__('april_tag_node')

        self.detector = Detector(
            families="tagStandard41h12",
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
            debug=False
        )

        # 카메라 초기화
        self.cap = cv2.VideoCapture('/dev/cam_ov3660')  # OV3660 USB 웹캠 (udev 룰로 생성된 심볼릭 링크)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다. 경로를 확인하세요.")
        
        # 카메라 해상도 설정 (2048x1536 @ 30fps)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2048)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1536)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # 실제 설정된 해상도 확인
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"카메라 해상도: {actual_width}x{actual_height} @ {actual_fps}fps")

        # ROS2 Publisher
        self.publisher = self.create_publisher(PoseStamped, '/april_tag_pose', 10)

        # OV3360 카메라 Intrinsic 파라미터 (2048x1536 해상도용)
        # 캘리브레이션 파일: ov3360_2_calib.yaml
        self.camera_matrix = np.array([
            [1470.52468, 0.0, 1101.4257],      # fx, 0, cx
            [0.0, 1468.05761, 682.17243],      # 0, fy, cy
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([-0.313189, 0.087017, -0.002519, -0.001188, 0.000000])
        self.tag_size = 0.028  # 28mm (meter)

        # 주기적으로 프레임 처리
        self.timer = self.create_timer(0.05, self.process_frame)

        self.last_warn_time = 0.0  # 마지막 경고 출력 시간 기록
        self.last_log_time = 0.0
        self.last_no_detect_log = 0.0


    def process_frame(self):
        global latest_frame
        ret, frame = self.cap.read()
        if not ret:
            current_time = time.time()
            if current_time - self.last_warn_time >= 3.0:
                self.get_logger().warn("Camera frame not received.")
                self.last_warn_time = current_time
            return

        # Distortion correction 적용
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        
        gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        # === 탐지 실패 로그 (3초마다 출력) ===
        if len(results) == 0:
            current_time = time.time()
            if current_time - self.last_no_detect_log >= 3.0:
                self.get_logger().warn("No AprilTag detected in frame.")
                self.last_no_detect_log = current_time
                
        
        if len(results) > 0:
            for det in results:
                corners = np.array(det.corners, dtype=np.float32)

            # 실제 태그 3D 좌표 (단위: m)
            obj_points = np.array([
                [-self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2,  self.tag_size/2, 0],
                [-self.tag_size/2,  self.tag_size/2, 0]
            ], dtype=np.float32)

            # solvePnP로 pose 계산
            success, rvec, tvec = cv2.solvePnP(
                obj_points,
                corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if success:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "jetcocam"

                # 위치
                pose_msg.pose.position.x = tvec[0][0]
                pose_msg.pose.position.y = tvec[1][0]
                pose_msg.pose.position.z = tvec[2][0]

                # 회전 (Rodrigues → Quaternion)
                R, _ = cv2.Rodrigues(rvec)
                qw = np.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0
                qx = (R[2,1] - R[1,2]) / (4*qw)
                qy = (R[0,2] - R[2,0]) / (4*qw)
                qz = (R[1,0] - R[0,1]) / (4*qw)

                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)

                self.publisher.publish(pose_msg)
                current_time = time.time()
                if current_time - self.last_log_time >= 3.0:
                    self.get_logger().info(f"Published pose for tag ID {det.tag_id}")
                    self.last_log_time = current_time



        # 최신 프레임을 Flask 스트리밍용으로 저장 (distortion corrected frame 사용)
        with frame_lock:
            latest_frame = undistorted_frame.copy()


# ---------------- Flask Streaming ---------------- #
def generate_frames():
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                continue
            ret, buffer = cv2.imencode('.jpg', latest_frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return redirect('/stream')

@app.route('/stream')
def stream():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

# ---------------- Main ---------------- #
def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()

    t = threading.Thread(target=flask_thread, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    