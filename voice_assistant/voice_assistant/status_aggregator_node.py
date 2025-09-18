import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data 
import json

# 导入需要订阅的传感器消息类型
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure, Illuminance
# 导入要发布的字符串消息类型
from std_msgs.msg import String

class StatusAggregatorNode(Node):
    def __init__(self):
        super().__init__('status_aggregator_node')
        self.get_logger().info("状态聚合节点已启动...")

        # 用于存储最新数据的字典
        self.latest_status = {
            "temperature": None,
            "humidity": None,
            "pressure": None,
            "illuminance": None,
        }

        # 创建对传感器话题的订阅者
        qos_profile = qos_profile_sensor_data

        self.create_subscription(Temperature, '/sensors/temperature', self.temp_callback, qos_profile)
        self.create_subscription(RelativeHumidity, '/sensors/humidity', self.humidity_callback, qos_profile)
        self.create_subscription(FluidPressure, '/sensors/pressure', self.pressure_callback, qos_profile)
        self.create_subscription(Illuminance, '/sensors/illuminance', self.illuminance_callback, qos_profile)

        # 创建一个发布者，用于向Web前端发布JSON字符串
        self.status_publisher = self.create_publisher(String, '/dashboard/status', 10)

        # 创建一个定时器，周期性地发布聚合后的状态
        self.publish_timer = self.create_timer(1.0, self.publish_status) # 每秒发布一次

    def temp_callback(self, msg):
        self.latest_status['temperature'] = round(msg.temperature, 1)

    def humidity_callback(self, msg):
        self.latest_status['humidity'] = round(msg.relative_humidity * 100.0, 1)

    def pressure_callback(self, msg):
        self.latest_status['pressure'] = round(msg.fluid_pressure / 100.0, 1) # 转为hPa

    def illuminance_callback(self, msg):
        self.latest_status['illuminance'] = round(msg.illuminance, 1)

    def publish_status(self):
        # 将字典转换为JSON字符串
        status_json = json.dumps(self.latest_status)
        
        # 创建一个String消息并发布
        msg = String()
        msg.data = status_json
        self.status_publisher.publish(msg)
        self.get_logger().debug(f"发布状态: {status_json}")

def main(args=None):
    rclpy.init(args=args)
    node = StatusAggregatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()