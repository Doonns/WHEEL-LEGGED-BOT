import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import struct

class CmdVelSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_serial_node')

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
            self.get_logger().info("Serial port /dev/ttyACM0 opened")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial open failed: {e}")
            raise e

        self.lock = threading.Lock()

        # 状态初始化
        self.v = 0.0           # 目标速度
        self.turn = 0.0        # 目标角速度
        self.v_now = 0.0       # 当前速度
        self.turn_now = 0.0    # 当前角速度

        self.leg = 80
        self.roll = 0
        self.jump = 0
        self.dt = 0.01  # 控制周期：10ms
        # 参数（根据实际情况调整）
        self.max_acc = 5.0      # 最大线加速度 (m/s2)
        self.max_turn_acc = 1.0 # 最大角加速度 (rad/s2)
        self.alpha = 0.3        # 低通滤波系数

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        self.timer = self.create_timer(self.dt, self.send_loop)

    @staticmethod
    def limit_value(min_val, max_val, value):
        return max(min_val, min(max_val, value))

    def cmd_vel_callback(self, msg: Twist):
        with self.lock:
            self.v = self.limit_value(-1.0, 1.0, msg.linear.x)
            self.turn = self.limit_value(-0.5, 0.5, msg.angular.z)

    def send_loop(self):
        with self.lock:
            #  linear_acc
            max_dv = self.max_acc * self.dt
            dv = self.v - self.v_now
            if abs(dv) > max_dv:
                dv = max_dv if dv > 0 else -max_dv
            self.v_now += dv

            # theta_v_acc
            max_dw = self.max_turn_acc * self.dt
            dw = self.turn - self.turn_now
            if abs(dw) > max_dw:
                dw = max_dw if dw > 0 else -max_dw
            self.turn_now += dw

            
            self.v_now = self.alpha * self.v_now + (1 - self.alpha) * self.v
            self.turn_now = self.alpha * self.turn_now + (1 - self.alpha) * self.turn

            
            v_mm = int(self.v_now * 2000)  # BACKUP: 3000

            theta = self.turn_now * self.dt  
            theta_mrad_raw = int(theta * 1000)

            leg_mm = int(self.leg)
            roll_mrad = int(self.roll)
            jump = self.jump

        packed = struct.pack('<hhhhB3x', v_mm, theta_mrad_raw, leg_mm, roll_mrad, jump)

        try:
            self.ser.write(packed)
            self.get_logger().info(f"send: v={v_mm} mm/s, theta={theta_mrad_raw} mrad, leg={leg_mm}, jump={jump}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
            
            
    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
