import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import smbus2


import os            # <-- 新增
import traceback     # <-- 新增


# 导入标准ROS2消息类型
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure, Illuminance

# 从我们整理的 drivers 目录和 config 导入
from .drivers import BH1750, BMP280, DHT20
from . import config

# 为了统一接口，我们为函数式的DHT20驱动创建一个简单的包装类
class DHT20Wrapper:
    def __init__(self, bus):
        self.bus = bus
        if not DHT20.aht20_init_sequence_smbus(self.bus):
            raise RuntimeError("Failed to initialize DHT20/AHT20 sensor.")

    def read_data(self):
        # read_aht20_data_smbus 返回 (temperature, humidity, crc_ok)
        return DHT20.read_aht20_data_smbus(self.bus)

class SensorPublisherNode(Node):
    def __init__(self):
        super().__init__('sensor_publisher_node')
        self.get_logger().info("传感器发布节点启动中...")

        # --- 修改: 不再创建单一总线，而是为每个总线创建一个对象 ---
        self.i2c_buses = {}
        self.sensors = {}
        sensor_configs = config.I2C_CONFIG["sensors"]

        # 遍历所有传感器配置，动态初始化
        for name, cfg in sensor_configs.items():
            if not cfg["enabled"]:
                continue

            try:
                bus_num = cfg["bus_number"]
                # 如果这个总线还没有被打开，就打开它并存储起来
                if bus_num not in self.i2c_buses:
                    self.i2c_buses[bus_num] = smbus2.SMBus(bus_num)
                    self.get_logger().info(f"I2C bus {bus_num} 打开成功。")
                
                # 获取该传感器需要使用的总线对象
                bus = self.i2c_buses[bus_num]
                addr = cfg["address"]

                # 根据传感器名称进行初始化
                if name == "dht20":
                    self.sensors["dht20"] = DHT20Wrapper(bus)
                    self.get_logger().info(f"DHT20/AHT20 on bus {bus_num} 初始化成功。")
                elif name == "bmp280":
                    bmp_sensor = BMP280.BMP280(bus, i2c_addr=addr)
                    if bmp_sensor.begin():
                        self.sensors["bmp280"] = bmp_sensor
                        self.get_logger().info(f"BMP280 on bus {bus_num} 初始化成功。")
                    else:
                        self.get_logger().error(f"BMP280 on bus {bus_num} 初始化失败 (begin() 返回 False)。")
                elif name == "bh1750":
                    self.sensors["bh1750"] = BH1750.BH1750(bus, addr=addr)
                    self.get_logger().info(f"BH1750 on bus {bus_num} 初始化成功。")

            except Exception as e:
                self.get_logger().error(f"初始化传感器 {name} on bus {cfg.get('bus_number', 'N/A')} 时失败: {e}")

        # --- 创建发布者 ---
        self.temp_pub = self.create_publisher(Temperature, '/sensors/temperature', qos_profile_sensor_data)
        self.humidity_pub = self.create_publisher(RelativeHumidity, '/sensors/humidity', qos_profile_sensor_data)
        self.pressure_pub = self.create_publisher(FluidPressure, '/sensors/pressure', qos_profile_sensor_data)
        self.illuminance_pub = self.create_publisher(Illuminance, '/sensors/illuminance', qos_profile_sensor_data)

        # --- 创建定时器，每2秒读取并发布一次数据 ---
        self.timer = self.create_timer(2.0, self.publish_sensor_data)
        self.get_logger().info("传感器节点初始化完成，将开始发布数据。")

    def publish_sensor_data(self):
        timestamp = self.get_clock().now().to_msg()

        # 1. 从 DHT20/AHT20 读取温度和湿度
        if "dht20" in self.sensors:
            try:
                temp_c, humidity_rh, crc_ok = self.sensors["dht20"].read_data()
                if crc_ok and temp_c is not None and humidity_rh is not None:
                    # 发布温度
                    temp_msg = Temperature()
                    temp_msg.header.stamp = timestamp
                    temp_msg.header.frame_id = "dht20_link"
                    temp_msg.temperature = float(temp_c)
                    self.temp_pub.publish(temp_msg)
                    # 发布湿度
                    humidity_msg = RelativeHumidity()
                    humidity_msg.header.stamp = timestamp
                    humidity_msg.header.frame_id = "dht20_link"
                    humidity_msg.relative_humidity = float(humidity_rh) / 100.0 # 消息要求是 0.0-1.0
                    self.humidity_pub.publish(humidity_msg)
                else:
                    self.get_logger().warn("从DHT20读取数据失败或CRC校验错误。")
            except Exception as e:
                self.get_logger().error(f"读取DHT20时发生异常: {e}")

        # 2. 从 BMP280 读取气压 (它也能读温度，但我们优先用DHT20的)
        if "bmp280" in self.sensors:
            try:
                pressure_hpa = self.sensors["bmp280"].readPressure()
                if pressure_hpa is not None:
                    pressure_pa = pressure_hpa * 100.0 # 消息单位是帕斯卡 (Pa)
                    pressure_msg = FluidPressure()
                    pressure_msg.header.stamp = timestamp
                    pressure_msg.header.frame_id = "bmp280_link"
                    pressure_msg.fluid_pressure = float(pressure_pa)
                    self.pressure_pub.publish(pressure_msg)
                else:
                    self.get_logger().warn("从BMP280读取数据失败。")
            except Exception as e:
                self.get_logger().error(f"读取BMP280时发生异常: {e}")

        # 3. 从 BH1750 读取光照强度
        if "bh1750" in self.sensors:
            try:
                lux = self.sensors["bh1750"].read_luminance()
                if lux is not None:
                    illuminance_msg = Illuminance()
                    illuminance_msg.header.stamp = timestamp
                    illuminance_msg.header.frame_id = "bh1750_link"
                    illuminance_msg.illuminance = float(lux)
                    self.illuminance_pub.publish(illuminance_msg)
                else:
                    self.get_logger().warn("从BH1750读取数据失败。")
            except Exception as e:
                self.get_logger().error(f"读取BH1750时发生异常: {e}")
                
    def destroy_node(self):
        self.get_logger().info("关闭传感器节点...")
        if hasattr(self, 'i2c_buses'):
            for bus_num, bus in self.i2c_buses.items():
                bus.close()
                self.get_logger().info(f"I2C bus {bus_num} 已关闭。")
        super().destroy_node()


def main(args=None):
    # 定义一个日志文件路径，用于记录致命错误
    log_file_path = os.path.join(os.path.expanduser('~'), 'sensor_node_crash.log')
    try:
        rclpy.init(args=args)
        node = SensorPublisherNode()
        rclpy.spin(node)
    except Exception as e:
        # 如果在初始化或运行中发生任何错误，都将其写入文件
        with open(log_file_path, 'w') as f:
            f.write(f"Sensor node crashed with exception:\n\n")
            # 写入完整的错误堆栈信息
            f.write(traceback.format_exc())
        # 同时在控制台打印，以防万一
        print(f"CRITICAL ERROR in sensor_publisher_node. See log at: {log_file_path}")
        raise e # 重新抛出异常，让ROS知道进程失败了
    finally:
        # 确保即使出错也能尝试清理
        if 'node' in locals() and rclpy.ok() and node.is_valid():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()