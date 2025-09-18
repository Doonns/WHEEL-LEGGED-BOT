import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time
import numpy as np

# --- 新增 ---: 导入Twist消息用于速度控制，String用于发布手势结果
from geometry_msgs.msg import Twist
from std_msgs.msg import String as StringMsg # 重命名以避免与python内置的str冲突


# --- 全局变量，用于在ROS节点和HTTP服务器线程之间共享最新一帧图像 ---
last_frame = None
frame_lock = threading.Lock()

# ==============================================================================
# HandDetector 类 (无改动)
# ==============================================================================
class HandDetector:
    def __init__(self, mode=False, maxHands=1, modelComplexity=0, detectionCon=0.5, minTrackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.modelComplex = modelComplexity
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.minTrackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.results = None
        self.lmList = []

    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handNo=0, draw=True):
        self.lmList = []
        if self.results and self.results.multi_hand_landmarks:
            if handNo < len(self.results.multi_hand_landmarks):
                myHand = self.results.multi_hand_landmarks[handNo]
                h, w, c = img.shape
                for id, lm in enumerate(myHand.landmark):
                    px, py = int(lm.x * w), int(lm.y * h)
                    self.lmList.append([id, px, py])
        return self.lmList, {}

    def fingersUp(self):
        fingers = []
        if not self.lmList or len(self.lmList) < 21:
            return []
        lm_map = {item[0]: (item[1], item[2]) for item in self.lmList}
        if lm_map.get(self.tipIds[0]) is None or lm_map.get(self.tipIds[0] - 1) is None: return [] # 安全检查
        if lm_map[self.tipIds[0]][0] > lm_map[self.tipIds[0] - 1][0]:
            fingers.append(1)
        else:
            fingers.append(0)
        for id in range(1, 5):
            if lm_map.get(self.tipIds[id]) is None or lm_map.get(self.tipIds[id] - 2) is None: return [] # 安全检查
            if lm_map[self.tipIds[id]][1] < lm_map[self.tipIds[id] - 2][1]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

# ==============================================================================
# GestureRecognition 类 
# ==============================================================================

class GestureRecognition:
    def __init__(self, model_complexity=0):
        self.detector = HandDetector(maxHands=1, modelComplexity=model_complexity)

    def recognize_gesture(self, frame):
        img = self.detector.findHands(frame, draw=True)
        lmList, _ = self.detector.findPosition(img)
        command = "" # 默认指令为空
        if lmList:
            fingers = self.detector.fingersUp()
            if len(fingers) == 5:
                x1, x2, x3, x4, x5 = fingers # [大拇指, 食指, 中指, 无名指, 小拇指]

                # --- 全新的手势定义逻辑 ---
                
                # 1. 手势 "5" (四指全伸) -> 后退
                if fingers == [0, 1, 1, 1, 1]:
                    command = "5"  
                
                # 2. 手势 "gun"  -> 左转
                elif fingers == [1, 1, 0, 0, 0]:
                    command = "gun"
                
                # 3. 手势 "pink" (小拇指) -> 右转
                elif fingers == [1, 0, 0, 0, 1]:
                    command = "pink"
                
                # 4. 手势 "6"  -> 前进
                elif fingers == [0, 0, 0, 0, 1]:
                    command = "6"
                
              
                
                # 你可以根据需要添加更多的 elif 来定义其他手势

        return img, command
# ==============================================================================
# HTTP 服务器部分 (无改动)
# ==============================================================================
class CamHandler(BaseHTTPRequestHandler):
    # ... (这部分代码保持不变) ...
    def do_GET(self):
        global last_frame, frame_lock
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><head><title>Hand Gesture</title></head><body style="margin:0; background-color:#333;"><img src="/stream.mjpg" style="width:100%; height:auto;"/></body></html>')
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while True:
                try:
                    with frame_lock:
                        if last_frame is None: continue
                        ret, jpg = cv2.imencode('.jpg', last_frame)
                        if not ret: continue
                        frame_bytes = jpg.tobytes()
                    self.wfile.write(b'--jpgboundary\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    time.sleep(1/60)
                except (IOError, ConnectionResetError, BrokenPipeError):
                    break
        else:
            self.send_error(404)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

# ==============================================================================
# 主 ROS2 节点 (包含主要改动)
# ==============================================================================
class HandDetectorNode(Node):
    def __init__(self):
        super().__init__('hand_detector_node_web')
        
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('web_port', 8080)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('model_complexity', 0)

        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        web_port = self.get_parameter('web_port').get_parameter_value().integer_value
        frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        model_complexity = self.get_parameter('model_complexity').get_parameter_value().integer_value
        
        self.gesture_recognizer = GestureRecognition(model_complexity=model_complexity)
        
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"Error: Could not open camera at index {camera_index}")
            rclpy.shutdown(); return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Requested resolution: {frame_width}x{frame_height}, Actual: {actual_width}x{actual_height}")
        
        self.http_server = ThreadedHTTPServer(('', web_port), CamHandler)
        self.server_thread = threading.Thread(target=self.http_server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        self.get_logger().info(f"HTTP server started on port {web_port}. Open http://<your_ip>:{web_port} in a browser.")

        # --- 新增 1: 创建发布者 ---
        # 创建一个发布者用于发送速度指令到 /cmd_vel
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 创建一个发布者用于广播识别出的手势（方便调试）
        self.gesture_publisher = self.create_publisher(StringMsg, '/gesture_result', 10)

        # --- 新增 2: 定义手势到速度的映射 ---
        # 格式: '手势字符串': (线速度x, 角速度z)
        # 你可以根据需要轻松地修改这些值
       
       
        self.gesture_to_velocity_map = {
            '6':    (0.2, 0.0),    # 手势"6": 前进
            '5':    (-0.2, 0.0),   # 手势"5": 后退
            'gun':    (0.0, 0.5),    # 手势"gun": 左转 (逆时针)
            'pink':    (0.0, -0.5),   # 手势"pink": 右转 (顺时针)
            'Fist': (0.0, 0.0),    # 握拳: 停车
            '':     (0.0, 0.0)     # 未识别到手势，也发送停止指令
        }
        self.get_logger().info("New gesture to velocity mapping loaded.")


        
        self.timer = self.create_timer(1/15, self.timer_callback)
        self.get_logger().info("Hand Gesture Detector Node with velocity control is running.")

    def timer_callback(self):
        global last_frame, frame_lock
        
        success, img = self.cap.read()
        if not success:
            return

        processed_image, command = self.gesture_recognizer.recognize_gesture(img)
        
        # --- 新增 3: 发布手势和速度指令 ---
        
        # 1. 发布识别出的手势字符串
        gesture_msg = StringMsg()
        gesture_msg.data = command
        self.gesture_publisher.publish(gesture_msg)

        # 2. 根据手势发布速度指令
        # 使用.get()方法，如果手势不在字典中，则默认返回停止指令(0.0, 0.0)，非常安全
        linear_x, angular_z = self.gesture_to_velocity_map.get(command, (0.0, 0.0))
        
        # 创建Twist消息
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)
        
        # 发布Twist消息
        self.velocity_publisher.publish(twist_msg)
        
        # 在日志中打印当前状态，方便调试
        if command:
            self.get_logger().info(f'Gesture: "{command}", Publishing Vel: x={linear_x}, z={angular_z}')
        
        # 更新视频帧上的显示文本
        display_text = f"Gesture: {command}" if command else "No Gesture"
        (text_width, text_height), _ = cv2.getTextSize(display_text, cv2.FONT_HERSHEY_PLAIN, 2, 2)
        top_right_x = processed_image.shape[1] - text_width - 10
        top_right_y = text_height + 10
        cv2.putText(processed_image, display_text, (top_right_x, top_right_y), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)

        with frame_lock:
            last_frame = processed_image.copy()

    def destroy_node(self):
        self.get_logger().info("Shutting down node...")
        
        # --- 新增 4: 安全停止机器人 ---
        # 在关闭节点前，发布一个零速度指令，确保机器人停止运动
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.velocity_publisher.publish(stop_twist)
        self.get_logger().info('Safety stop command sent to /cmd_vel.')

        # 关闭HTTP服务器和摄像头
        self.http_server.shutdown()
        self.http_server.server_close()
        self.server_thread.join()
        self.cap.release()
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()