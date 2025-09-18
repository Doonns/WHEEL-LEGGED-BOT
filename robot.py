import wave
from aip import AipSpeech
from zhipuai import ZhipuAI
import pygame
import os
import threading
import time
import temp
import requests
import json
import re
import config
import audioop
import subprocess

# 设置环境变量解决音频问题
if 'XDG_RUNTIME_DIR' not in os.environ:
    os.environ['XDG_RUNTIME_DIR'] = '/tmp/runtime-root'
    os.makedirs(os.environ['XDG_RUNTIME_DIR'], exist_ok=True)

# 设置ALSA环境变量
os.environ['ALSA_PCM_CARD'] = '0'
os.environ['ALSA_PCM_DEVICE'] = '0'

# 延迟初始化pygame，避免启动时的音频错误
pygame_initialized = False

def init_pygame():
    """安全初始化pygame"""
    global pygame_initialized
    if not pygame_initialized:
        try:
            pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
            pygame_initialized = True
            print("pygame音频初始化成功")
            return True
        except Exception as e:
            print(f"pygame初始化失败: {e}")
            return False
    return True

# 使用配置文件中的API设置
client_baidu = AipSpeech(
    config.API_CONFIG["baidu"]["app_id"],
    config.API_CONFIG["baidu"]["api_key"],
    config.API_CONFIG["baidu"]["secret_key"]
)
client_zhipu = ZhipuAI(api_key=config.API_CONFIG["zhipu"]["api_key"])

def my_record(rate=None):
    WAVE_OUTPUT_FILENAME = os.path.join(config.PATHS["voices_dir"], "myvoices.wav")
    RAW_VOICE_FILENAME = os.path.join(config.PATHS["voices_dir"], "myvoices_raw.wav")
    RECORD_SECONDS = str(config.AUDIO_CONFIG["record_seconds"])

    # 构造arecord命令，以硬件接受的参数录制原始音频
    # -f S16_LE: 使用16位深度，兼容性更好
    # -r 48000: 使用硬件支持的48000Hz采样率
    # -c 2: 使用硬件强制要求的2通道（立体声）
    command = [
        'arecord',
        '-D', 'hw:0,0',
        '-f', 'S16_LE',
        '-r', '48000',
        '-c', '2',
        '-t', 'wav',
        '-d', RECORD_SECONDS,
        RAW_VOICE_FILENAME
    ]

    try:
        print("say something...")
        # 使用subprocess执行命令，捕获其标准错误流以便调试
        process = subprocess.run(
            command,
            check=True,
            capture_output=True, # 捕获输出
            text=True # 以文本模式处理输出
        )
        print("Recording finished")
        
        # === 音频数据后期处理 ===
        with wave.open(RAW_VOICE_FILENAME, 'rb') as wf:
            n_channels = wf.getnchannels()
            samp_width = wf.getsampwidth()
            framerate = wf.getframerate()
            n_frames = wf.getnframes()
            raw_data = wf.readframes(n_frames)

        # 1. 从立体声转换为单声道
        mono_data = audioop.tomono(raw_data, samp_width, 1, 1)

        # 2. 采样率转换 (从48000Hz到16000Hz)
        TARGET_RATE = 16000
        converted_data, _ = audioop.ratecv(mono_data, samp_width, 1, framerate, TARGET_RATE, None)
        
        # 3. 保存最终处理过的音频文件
        with wave.open(WAVE_OUTPUT_FILENAME, 'wb') as wf_out:
            wf_out.setnchannels(1)
            wf_out.setsampwidth(samp_width) # 位深度不变（16位）
            wf_out.setframerate(TARGET_RATE)
            wf_out.writeframes(converted_data)

        return WAVE_OUTPUT_FILENAME
    except FileNotFoundError:
        print("Recording error: 'arecord' command not found. Please ensure ALSA tools are installed.")
        return None
    except subprocess.CalledProcessError as e:
        # 打印arecord的具体错误信息
        print(f"Recording error: arecord failed with exit code {e.returncode}.")
        print(f"arecord stderr: {e.stderr}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during recording: {e}")
        return None

def listen(path):
    result_json = None  # 在try块外定义，以便except块可以访问
    try:
        with open(path, 'rb') as fp:
            voices = fp.read()
        
        result_json = client_baidu.asr(voices, 'wav', 16000, {'dev_pid': 1536})
        
        # 检查API是否返回了错误
        if 'err_no' in result_json and result_json['err_no'] != 0:
            print(f"Baidu ASR API Error: {result_json}")
            return None

        result_text = result_json["result"][0]
        print("Recognition result:", result_text)
        return result_text

    except KeyError:
        print("Recognition error: 'result' key not found in Baidu API response.")
        if result_json is not None:
            print(f"Full API response: {result_json}")
        return None
    except Exception as e:
        print(f"An unexpected recognition error occurred: {e}")
        return None

def Turing(text_words="",summary_mode=True, max_length=30):
    messages = [{"role": "user", "content": text_words}]
    response = client_zhipu.chat.completions.create(
        model="glm-4",
        messages=messages,
    )

    try:
        result_content = response.choices[0].message.content
        if result_content is None:
            result_content = "Sorry, I couldn't get a response."
    except (IndexError, AttributeError) as e:
        print("Response error: ", e)
        result_content = "Sorry, I couldn't get a response."
    if summary_mode:
        summary_prompt = f"请将以下内容缩减为不超过{max_length}个字:{result_content}"
        summary_messages = [{"role": "user", "content": summary_prompt}]
        summary_response = client_zhipu.chat.completions.create(
            model="glm-4",
            messages=summary_messages,
        )
    
        try:
            result_content = summary_response.choices[0].message.content
        except (IndexError, AttributeError) as e:
            print("Summary response error: ", e)
            result_content = result_content[:max_length] + "..."
    
    print("AI:", result_content)    
    return result_content

def stop_music():
    if pygame_initialized:
        pygame.mixer.music.stop()
        print("Music stopped.")

def music_player(music_file):
    """统一的音频播放函数，参照simple_audio_test.py方案"""
    if not os.path.exists(music_file):
        print(f"? 音频文件不存在: {music_file}")
        return False
    if not init_pygame():
        print("? pygame初始化失败，无法播放音频")
        return False
    try:
        pygame.mixer.music.load(music_file)
        pygame.mixer.music.play()
        print(f"?? 开始播放: {music_file}")
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
        print("? 播放完成")
        return True
    except Exception as e:
        print(f"? 音频播放失败: {e}")
        return False

def get_valid_temperature_and_humidity(data_pin=None, max_retries=5):
    if data_pin is None:
        data_pin = config.GPIO_CONFIG["dht11_pin"]
    
    # 移除这里的setup调用，因为temp.py现在会自我管理

    for attempt in range(max_retries):
        temperature, humidity = temp.setup_and_read_dht11_data(data_pin=data_pin)
        if temperature is not None and humidity is not None:
            return temperature, humidity
        print(f"Failed to read from DHT11 on attempt {attempt + 1}/{max_retries}")
        time.sleep(2) # 在重试前等待2秒

    print("Error: Could not read temperature/humidity after several attempts.")
    return None, None


HEFENG_API_URL = "https://devapi.qweather.com/v7/weather/now"
HEFENG_API_KEY = "411e05242bb44cc890649383c16e2fb5"
location = '101090201'

def get_and_read_weather(city):
    params = {
        'location': config.API_CONFIG["weather"]["location"],
        'key': config.API_CONFIG["weather"]["api_key"],
        'lang': 'zh',
        'unit': 'm'  
    }
    response = requests.get(config.API_CONFIG["weather"]["api_url"], params=params)
    if response.status_code == 200:
        weather_data = response.json()
        now = weather_data.get('now')
        if now:
            readweather = (
                f"{city}当前天气{now['text']},温度是{now['temp']}摄氏度,湿度为{now['humidity']}%,风向为{now['windDir']},风速{now['windSpeed']}公里每小时"
            )
            print(readweather)  
            result = client_baidu.synthesis(readweather, 'zh', 1, {
                'vol': 5,
                'per': 4,
                'spd': 9
            })
            if not isinstance(result, dict):
                with open(config.PATHS["output_audio"], 'wb') as f:
                    f.write(result)
                
                if music_player(config.PATHS["output_audio"]):
                    pygame.mixer.music.load(config.PATHS["output_audio"])
                    pygame.mixer.music.play()
                    while pygame.mixer.music.get_busy():
                        pygame.time.Clock().tick(10)
                else:
                    print("无法播放天气信息")
        else:
            print("error")
    else:
        print("error", response.status_code)

if __name__ == "__main__":
    running = True
    music_thread = None
    
    # 根据RDK X5规范，在程序开始时设置一次GPIO模式为BOARD
    # BOARD模式意味着使用物理引脚编号
    temp.GPIO.setmode(temp.GPIO.BOARD)

    try:
        # 初始读取一次温湿度
        #get_valid_temperature_and_humidity()
        
        while running:
            file_path = my_record()
            if file_path:
                request = listen(file_path)
                if request:
                    if "关闭" in request.lower():
                        print("Closing the program...")
                        running = False
                        break
                    if "播放音乐" in request:
                        match = re.search(r"播放音乐[，,]?\s*(.+)", request)
                        song_name = match.group(1).strip()
                        song_name = song_name[:-1]
                        music_file = os.path.join(config.PATHS["music_dir"], f"{song_name}.mp3")
                        if os.path.exists(music_file):
                            if music_thread and music_thread.is_alive():
                                stop_music()
                                music_thread.join()
                            music_thread = threading.Thread(target=music_player, args=(music_file,))
                            music_thread.start()
                        else:
                            print(f"File {music_file} not found!")

                    
                    elif "停止音乐" in request:
                        if music_thread and music_thread.is_alive():
                            stop_music()
                            music_thread.join()
                        else:
                            print("No music is playing.")
                    
                    elif "温度" in request or "湿度" in request:
                        temperature, humidity = get_valid_temperature_and_humidity()
                        if "温度" in request:
                            response = f"当前温度为{temperature}摄氏度" 
                        elif "湿度" in request:
                            response = f"当前湿度为{humidity}%" 
                        print(response)
                        result = client_baidu.synthesis(response, 'zh', 1, {
                            'vol': 5,
                            'per': 4,
                            'spd': 9
                        })
                        if not isinstance(result, dict):
                            with open(config.PATHS["output_audio"], 'wb') as f:
                                f.write(result)

                            if music_player(config.PATHS["output_audio"]):
                                pygame.mixer.music.load(config.PATHS["output_audio"])
                                pygame.mixer.music.play()
                                while pygame.mixer.music.get_busy():
                                    pygame.time.Clock().tick(10)
                            else:
                                print("无法播放温湿度信息")
                    
                    elif "天气" in request:
                        city = "保定" 
                        get_and_read_weather(city)
                    
                    else:
                        response = Turing(request)
                        result = client_baidu.synthesis(response, 'zh', 1, {
                            'vol': 5,
                            'per': 4,
                            'spd': 9
                        })

                        if not isinstance(result, dict):
                            with open(config.PATHS["output_audio"], 'wb') as f:
                                f.write(result)

                            if music_player(config.PATHS["output_audio"]):
                                pygame.mixer.music.load(config.PATHS["output_audio"])
                                pygame.mixer.music.play()
                                while pygame.mixer.music.get_busy():
                                    pygame.time.Clock().tick(10)
                            else:
                                print("无法播放AI回复")

                else:
                    print("Recognition failed. Please try again.")
            else:
                print("Recording failed. Please try again.")

    except KeyboardInterrupt:
        print("\nProgram terminated.")
    finally:
        # 确保程序退出时清理所有已使用的GPIO资源
        print("Cleaning up GPIO...")
        temp.GPIO.cleanup()
