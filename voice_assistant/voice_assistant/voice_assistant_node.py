import rclpy
from rclpy.node import Node
import os
import pygame
import threading
import subprocess
import wave
import audioop
import re
import time

# 第三方库
from aip import AipSpeech
from zhipuai import ZhipuAI
import requests
from rclpy.qos import qos_profile_sensor_data 
# 同包下的本地模块
from . import config
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure, Illuminance

class VoiceAssistantNode(Node):
    def __init__(self):
        super().__init__('voice_assistant_node')
        self.get_logger().info("语音助手节点启动中...")

        # --- 新增点 1: 状态管理与唤醒词/休眠机制 ---
        self.current_state = 'SLEEPING'  # 初始状态为休眠
        self.WAKE_WORD = "地瓜"
        self.SLEEP_WORD = "再见"
        self.MAX_NO_INPUT_COUNT = 5
        self.no_input_counter = 0

        # --- 1. 初始化所有组件 ---
        self.is_running = True
        self.music_thread = None
        self._init_environment_and_pygame()

        try:
            self.client_baidu = AipSpeech(
                config.API_CONFIG["baidu"]["app_id"],
                config.API_CONFIG["baidu"]["api_key"],
                config.API_CONFIG["baidu"]["secret_key"]
            )
            self.client_zhipu = ZhipuAI(api_key=config.API_CONFIG["zhipu"]["api_key"])
            self.get_logger().info("API客户端初始化成功。")
        except KeyError as e:
            self.get_logger().fatal(f"配置文件config.py错误: 缺少键 {e}。程序将退出。")
            self.is_running = False
            return
            
        # --- 新增: 用于存储最新传感器数据的成员变量 ---
        self.last_temperature = None
        self.last_humidity = None
        self.last_pressure = None
        self.last_illuminance = None
        qos_profile = qos_profile_sensor_data

        self.temp_sub = self.create_subscription(
            Temperature, '/sensors/temperature', self.temp_callback, qos_profile)
        self.humidity_sub = self.create_subscription(
            RelativeHumidity, '/sensors/humidity', self.humidity_callback, qos_profile)
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/sensors/pressure', self.pressure_callback, qos_profile)
        self.illuminance_sub = self.create_subscription(
            Illuminance, '/sensors/illuminance', self.illuminance_callback, qos_profile)
        self.get_logger().info("已创建传感器数据订阅者。")


        self.greeting_timer = self.create_timer(1.0, self.say_greeting_once)
        self.main_loop_timer = self.create_timer(5.0, self.main_loop) # 定时器周期可以适当调整

    def temp_callback(self, msg):
        self.last_temperature = msg.temperature
        self.get_logger().debug(f"接收到新温度: {self.last_temperature:.2f} C")

    def humidity_callback(self, msg):
        self.last_humidity = msg.relative_humidity * 100.0 # 从 0-1.0 转为 0-100%
        self.get_logger().debug(f"接收到新湿度: {self.last_humidity:.2f} %RH")

    def pressure_callback(self, msg):
        self.last_pressure = msg.fluid_pressure / 100.0 # 从 Pa 转为 hPa (百帕)
        self.get_logger().debug(f"接收到新气压: {self.last_pressure:.2f} hPa")

    def illuminance_callback(self, msg):
        self.last_illuminance = msg.illuminance
        self.get_logger().debug(f"接收到新光照: {self.last_illuminance:.2f} lx")
    def destroy_node(self):
        """节点关闭时清理资源"""
        self.get_logger().info("正在关闭节点并清理资源...")
        if self.music_thread and self.music_thread.is_alive():
            pygame.mixer.music.stop()
        if hasattr(self, 'main_loop_timer'):
            self.main_loop_timer.cancel()
        if pygame.mixer.get_init():
            pygame.mixer.quit()
        

        self.get_logger().info("清理完成。")
        super().destroy_node()

    def say_greeting_once(self):
        """只说一次的欢迎语"""
        # --- 修改点 2: 更改欢迎语以适应唤醒词机制 ---
        self._speak(f"语音助手已启动")
        self.greeting_timer.cancel()

    def main_loop(self):
        """核心逻辑循环，由定时器周期性调用"""
        if not self.is_running or not rclpy.ok():
            if rclpy.ok():
                self.get_logger().info("is_running为False，准备关闭节点。")
                rclpy.shutdown()
            return

        # --- 修改点 3: 引入状态机逻辑 ---
        if self.current_state == 'SLEEPING':
            self.handle_sleeping_state()
        elif self.current_state == 'AWAKE':
            self.handle_awake_state()

    def handle_sleeping_state(self):
        """处理休眠状态，只监听唤醒词"""
        self.get_logger().info(f"处于休眠模式，等待唤醒词“你好地瓜”")
        audio_file = self._record_audio()
        if not audio_file:
            return

        user_request = self._listen(audio_file)
        if user_request and self.WAKE_WORD in user_request:
            self.get_logger().info("唤醒成功！")
            self.current_state = 'AWAKE'
            self.no_input_counter = 0  # 重置计数器
            self._speak("我在，请问有什么可以帮您？")
    
    def handle_awake_state(self):
        """处理唤醒状态，响应用户指令"""
        self.get_logger().info("处于唤醒模式，正在聆听指令...")
        audio_file = self._record_audio()
        if not audio_file:
            return

        user_request = self._listen(audio_file)
        
        # 情况1：未能识别语音
        if not user_request:
            self.get_logger().info("未能识别语音。")
            self.no_input_counter += 1
            self.get_logger().info(f"无有效输入次数: {self.no_input_counter}/{self.MAX_NO_INPUT_COUNT}")
            if self.no_input_counter >= self.MAX_NO_INPUT_COUNT:
                self._speak(f"长时间未收到有效指令，我先去休息了。需要我时，请再说你好{self.WAKE_WORD}。")
                self.current_state = 'SLEEPING'
            return

        # 情况2：识别到语音，重置计数器
        self.no_input_counter = 0
        self.get_logger().info(f'识别到内容: "{user_request}"')

        # 情况3：识别到结束语
        if self.SLEEP_WORD in user_request:
            self._speak("好的，我先去休息了，有需要随时叫我。")
            self.current_state = 'SLEEPING'
            return
            
        # 情况4：处理正常指令
        self._process_command(user_request)



    def _init_environment_and_pygame(self):
        if pygame.mixer.get_init():
            return True
        try:
            if 'XDG_RUNTIME_DIR' not in os.environ:
                os.environ['XDG_RUNTIME_DIR'] = '/tmp/runtime-root'
                os.makedirs(os.environ['XDG_RUNTIME_DIR'], exist_ok=True)
            os.environ['ALSA_PCM_CARD'] = '0'
            os.environ['ALSA_PCM_DEVICE'] = '0'
            pygame.mixer.init(frequency=16000)
            self.get_logger().info("Pygame音频模块初始化成功。")
            return True
        except Exception as e:
            self.get_logger().error(f"Pygame初始化失败: {e}")
            return False

    def _record_audio(self):
        if self.music_thread and self.music_thread.is_alive():
            self.get_logger().info("正在播放音乐，暂时不进行录音...")
            return None
        if pygame.mixer.get_init():
            self.get_logger().debug("释放Pygame Mixer以进行录音...")
            pygame.mixer.quit()
            
        WAVE_OUTPUT_FILENAME = os.path.join(config.PATHS["voices_dir"], "myvoices.wav")
        RECORD_SECONDS = str(config.AUDIO_CONFIG["record_seconds"])
        DEVICE_ID = 'hw:0,0' 

        command = [
            'arecord', '-D', DEVICE_ID, '-f', 'S16_LE',
            '-r', '16000', '-c', '1', '-t', 'wav', 
            '-d', RECORD_SECONDS, WAVE_OUTPUT_FILENAME
        ]
        
        # --- 修改点 4: 根据状态调整提示信息 ---
        if self.current_state == 'AWAKE':
            self.get_logger().info("请说话...")
        # 在休眠模式下，我们不在控制台打印“请说话”，保持安静等待。
        
        try:
            subprocess.run(command, check=True, capture_output=True, text=True)
            self.get_logger().info("录音结束。")
            return WAVE_OUTPUT_FILENAME
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"录音命令执行失败: {e.stderr}")
            return None
        except Exception as e:
            self.get_logger().error(f"录音过程中出现未知错误: {e}")
            return None

    def _listen(self, audio_path):
        try:
            with wave.open(audio_path, 'rb') as wf:
                num_channels, sample_width, original_rate, num_frames, _, _ = wf.getparams()
                audio_data = wf.readframes(num_frames)
            target_rate = 16000
            converted_audio_data, _ = audioop.ratecv(audio_data, sample_width, num_channels, original_rate, target_rate, None)
            result_json = self.client_baidu.asr(converted_audio_data, 'pcm', target_rate, {'dev_pid': 1537}) # 1537是普通话搜索模型，更适合短语识别
            if 'err_no' in result_json and result_json['err_no'] == 0:
                return result_json["result"][0]
            else:
                self.get_logger().error(f"百度ASR API返回错误: {result_json}")
                return None
        except Exception as e:
            self.get_logger().error(f"语音识别或重采样过程中出错: {e}")
            return None

    def _speak(self, text):
        if not self._init_environment_and_pygame():
             self.get_logger().error("无法初始化音频设备，无法播放语音。")
             return
        self.get_logger().info(f'AI正在说: "{text}"')
        result = self.client_baidu.synthesis(text, 'zh', 1, {'vol': 5, 'per': 4, 'spd': 5})
        if not isinstance(result, dict):
            with open(config.PATHS["output_audio"], 'wb') as f:
                f.write(result)
            self._play_audio_blocking(config.PATHS["output_audio"])
        else:
            self.get_logger().error(f"百度TTS API返回错误: {result}")

    def _play_audio_blocking(self, file_path):
        try:
            if not pygame.mixer.get_init():
                self._init_environment_and_pygame()
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        except Exception as e:
            self.get_logger().error(f"音频播放失败: {e}")

    def _process_command(self, text):
        if any(keyword in text for keyword in ["关闭", "退出"]):
            self._handle_shutdown()
        elif "播放音乐" in text:
            self._handle_play_music(text)
        elif "停止音乐" in text:
            self._handle_stop_music()
        elif "天气" in text:
            self._handle_weather()
        elif "温度" in text:
            self._handle_get_temperature()
        elif "湿度" in text:
            self._handle_get_humidity()
        elif "气压" in text:
            self._handle_get_pressure()
        elif any(keyword in text for keyword in ["亮度", "光照"]):
            self._handle_get_illuminance()
        else:
            self._handle_chat(text)

    def _handle_shutdown(self):
        self._speak("好的，系统即将关闭，再见。")
        self.is_running = False

    def _handle_play_music(self, text):
        match = re.search(r"播放音乐[,,]?\s*(.+)", text)
        song_name = match.group(1).strip().rstrip('。，,.') if match else ""
        if not song_name:
            self._speak("请告诉我您想听的歌名。")
            return
        music_file = os.path.join(config.PATHS["music_dir"], f"{song_name}.mp3")
        if os.path.exists(music_file):
            if self.music_thread and self.music_thread.is_alive():
                pygame.mixer.music.stop()
                self.music_thread.join()
            self._speak(f"好的，为您播放《{song_name}》")
            self.music_thread = threading.Thread(target=self._play_audio_blocking, args=(music_file,))
            self.music_thread.daemon = True
            self.music_thread.start()
        else:
            self._speak(f"抱歉，我没有找到歌曲《{song_name}》。")

    def _handle_stop_music(self):
        if pygame.mixer.get_init() and pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
            if self.music_thread and self.music_thread.is_alive():
                self.music_thread.join()
            self._speak("音乐已停止。")
        else:
            self._speak("当前没有音乐在播放。")

    def _handle_weather(self):
        try:
            params = {'location': config.API_CONFIG["weather"]["location"], 'key': config.API_CONFIG["weather"]["api_key"]}
            response = requests.get(config.API_CONFIG["weather"]["api_url"], params=params)
            response.raise_for_status()
            now = response.json().get('now', {})
            report = f"南京当前天气{now['text']}, 温度{now['temp']}摄氏度。"
            self._speak(report)
        except Exception as e:
            self.get_logger().error(f"获取天气失败: {e}")
            self._speak("抱歉，查询天气失败了。")

    def _handle_get_temperature(self):
        # --- 修改: 从成员变量获取数据 ---
        if self.last_temperature is not None:
            response = f"当前室内的温度是 {self.last_temperature:.1f} 摄氏度。"
            if self.last_temperature <25:
                response += " 较为适宜。"            
            if self.last_temperature >35:
                response += " 小心中暑。"
            self._speak(response)
        else:
            self._speak("抱歉，暂时还没有接收到温度数据，请稍后再试。")

    def _handle_get_humidity(self):
        # --- 修改: 从成员变量获取数据 ---
        if self.last_humidity is not None:
            response = f"当前室内的湿度为百分之 {self.last_humidity:.0f}。"
            if self.last_humidity < 30:
                response += " 建议补水。"
            self._speak(response)
        else:
            self._speak("抱歉，暂时还没有接收到湿度数据，请稍后再试。")

    # --- 新增: 处理气压和光照的函数 ---
    def _handle_get_pressure(self):
        if self.last_pressure is not None:
            response = f"当前的大气压大约是 {self.last_pressure:.0f} 百帕。"
            self._speak(response)
        else:
            self._speak("抱歉，暂时还没有接收到气压数据，请稍后再试。")

    def _handle_get_illuminance(self):
        if self.last_illuminance is not None:
            report = f"当前光照强度是 {self.last_illuminance:.0f} 勒克斯。"
            if self.last_illuminance < 50:
                report += " 环境比较暗。"
            elif self.last_illuminance < 400:
                report += " 室内光线正常。"
            else:
                report += " 光线很充足。"
            self._speak(report)
        else:
            self._speak("抱歉，暂时还没有接收到光照数据，请稍后再试。")

    def _handle_chat(self, text):
        try:
            messages = [{"role": "user", "content": text}]
            response = self.client_zhipu.chat.completions.create(model="glm-4", messages=messages)
            ai_response = response.choices[0].message.content
            self._speak(ai_response)
        except Exception as e:
            self.get_logger().error(f"调用智谱AI失败: {e}")
            self._speak("抱歉，我暂时无法回答这个问题。")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceAssistantNode()
    if node.is_running:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('键盘中断，节点关闭...')
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()

if __name__ == '__main__':
    main()