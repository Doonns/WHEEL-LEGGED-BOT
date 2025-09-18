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

# ���û������������Ƶ����
if 'XDG_RUNTIME_DIR' not in os.environ:
    os.environ['XDG_RUNTIME_DIR'] = '/tmp/runtime-root'
    os.makedirs(os.environ['XDG_RUNTIME_DIR'], exist_ok=True)

# ����ALSA��������
os.environ['ALSA_PCM_CARD'] = '0'
os.environ['ALSA_PCM_DEVICE'] = '0'

# �ӳٳ�ʼ��pygame����������ʱ����Ƶ����
pygame_initialized = False

def init_pygame():
    """��ȫ��ʼ��pygame"""
    global pygame_initialized
    if not pygame_initialized:
        try:
            pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
            pygame_initialized = True
            print("pygame��Ƶ��ʼ���ɹ�")
            return True
        except Exception as e:
            print(f"pygame��ʼ��ʧ��: {e}")
            return False
    return True

# ʹ�������ļ��е�API����
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

    # ����arecord�����Ӳ�����ܵĲ���¼��ԭʼ��Ƶ
    # -f S16_LE: ʹ��16λ��ȣ������Ը���
    # -r 48000: ʹ��Ӳ��֧�ֵ�48000Hz������
    # -c 2: ʹ��Ӳ��ǿ��Ҫ���2ͨ������������
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
        # ʹ��subprocessִ������������׼�������Ա����
        process = subprocess.run(
            command,
            check=True,
            capture_output=True, # �������
            text=True # ���ı�ģʽ�������
        )
        print("Recording finished")
        
        # === ��Ƶ���ݺ��ڴ��� ===
        with wave.open(RAW_VOICE_FILENAME, 'rb') as wf:
            n_channels = wf.getnchannels()
            samp_width = wf.getsampwidth()
            framerate = wf.getframerate()
            n_frames = wf.getnframes()
            raw_data = wf.readframes(n_frames)

        # 1. ��������ת��Ϊ������
        mono_data = audioop.tomono(raw_data, samp_width, 1, 1)

        # 2. ������ת�� (��48000Hz��16000Hz)
        TARGET_RATE = 16000
        converted_data, _ = audioop.ratecv(mono_data, samp_width, 1, framerate, TARGET_RATE, None)
        
        # 3. �������մ��������Ƶ�ļ�
        with wave.open(WAVE_OUTPUT_FILENAME, 'wb') as wf_out:
            wf_out.setnchannels(1)
            wf_out.setsampwidth(samp_width) # λ��Ȳ��䣨16λ��
            wf_out.setframerate(TARGET_RATE)
            wf_out.writeframes(converted_data)

        return WAVE_OUTPUT_FILENAME
    except FileNotFoundError:
        print("Recording error: 'arecord' command not found. Please ensure ALSA tools are installed.")
        return None
    except subprocess.CalledProcessError as e:
        # ��ӡarecord�ľ��������Ϣ
        print(f"Recording error: arecord failed with exit code {e.returncode}.")
        print(f"arecord stderr: {e.stderr}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during recording: {e}")
        return None

def listen(path):
    result_json = None  # ��try���ⶨ�壬�Ա�except����Է���
    try:
        with open(path, 'rb') as fp:
            voices = fp.read()
        
        result_json = client_baidu.asr(voices, 'wav', 16000, {'dev_pid': 1536})
        
        # ���API�Ƿ񷵻��˴���
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
        summary_prompt = f"�뽫������������Ϊ������{max_length}����:{result_content}"
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
    """ͳһ����Ƶ���ź���������simple_audio_test.py����"""
    if not os.path.exists(music_file):
        print(f"? ��Ƶ�ļ�������: {music_file}")
        return False
    if not init_pygame():
        print("? pygame��ʼ��ʧ�ܣ��޷�������Ƶ")
        return False
    try:
        pygame.mixer.music.load(music_file)
        pygame.mixer.music.play()
        print(f"?? ��ʼ����: {music_file}")
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
        print("? �������")
        return True
    except Exception as e:
        print(f"? ��Ƶ����ʧ��: {e}")
        return False

def get_valid_temperature_and_humidity(data_pin=None, max_retries=5):
    if data_pin is None:
        data_pin = config.GPIO_CONFIG["dht11_pin"]
    
    # �Ƴ������setup���ã���Ϊtemp.py���ڻ����ҹ���

    for attempt in range(max_retries):
        temperature, humidity = temp.setup_and_read_dht11_data(data_pin=data_pin)
        if temperature is not None and humidity is not None:
            return temperature, humidity
        print(f"Failed to read from DHT11 on attempt {attempt + 1}/{max_retries}")
        time.sleep(2) # ������ǰ�ȴ�2��

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
                f"{city}��ǰ����{now['text']},�¶���{now['temp']}���϶�,ʪ��Ϊ{now['humidity']}%,����Ϊ{now['windDir']},����{now['windSpeed']}����ÿСʱ"
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
                    print("�޷�����������Ϣ")
        else:
            print("error")
    else:
        print("error", response.status_code)

if __name__ == "__main__":
    running = True
    music_thread = None
    
    # ����RDK X5�淶���ڳ���ʼʱ����һ��GPIOģʽΪBOARD
    # BOARDģʽ��ζ��ʹ���������ű��
    temp.GPIO.setmode(temp.GPIO.BOARD)

    try:
        # ��ʼ��ȡһ����ʪ��
        #get_valid_temperature_and_humidity()
        
        while running:
            file_path = my_record()
            if file_path:
                request = listen(file_path)
                if request:
                    if "�ر�" in request.lower():
                        print("Closing the program...")
                        running = False
                        break
                    if "��������" in request:
                        match = re.search(r"��������[��,]?\s*(.+)", request)
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

                    
                    elif "ֹͣ����" in request:
                        if music_thread and music_thread.is_alive():
                            stop_music()
                            music_thread.join()
                        else:
                            print("No music is playing.")
                    
                    elif "�¶�" in request or "ʪ��" in request:
                        temperature, humidity = get_valid_temperature_and_humidity()
                        if "�¶�" in request:
                            response = f"��ǰ�¶�Ϊ{temperature}���϶�" 
                        elif "ʪ��" in request:
                            response = f"��ǰʪ��Ϊ{humidity}%" 
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
                                print("�޷�������ʪ����Ϣ")
                    
                    elif "����" in request:
                        city = "����" 
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
                                print("�޷�����AI�ظ�")

                else:
                    print("Recognition failed. Please try again.")
            else:
                print("Recording failed. Please try again.")

    except KeyboardInterrupt:
        print("\nProgram terminated.")
    finally:
        # ȷ�������˳�ʱ����������ʹ�õ�GPIO��Դ
        print("Cleaning up GPIO...")
        temp.GPIO.cleanup()
