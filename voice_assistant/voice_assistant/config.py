# -*- coding: utf-8 -*-
import os

# --- 核心配置：动态基础路径 ---
# 使用 os.path.expanduser('~') 来动态获取当前用户的主目录
# 例如，在 sunrise 用户下，它会是 /home/sunrise
# 我们将所有数据都存放在 '~/voice_assistant_data' 这个文件夹下
APP_BASE_DIR = os.path.join(os.path.expanduser('~'), 'voice_assistant_data')

# --- 文件路径配置 ---
# 所有路径都基于上面定义的动态基础路径
PATHS = {
    # 基础目录
    "base_dir": APP_BASE_DIR,
    # 音乐文件目录 (你可以把音乐文件放在这里)
    "music_dir": os.path.join(APP_BASE_DIR, "music"),
    # 录音和输出音频的目录
    "voices_dir": os.path.join(APP_BASE_DIR, "voices"),
    # 输出的TTS音频文件路径
    "output_audio": os.path.join(APP_BASE_DIR, "voices", "output.mp3"),
}

# --- 音频配置 ---
AUDIO_CONFIG = {
    "sample_rate": 16000,
    "channels": 1,
    "chunk_size": 1024,
    "record_seconds": 10,
}

# --- API配置 (请注意保管好你的密钥) ---
API_CONFIG = {
    "baidu": {
        "app_id": "85295395",
        "api_key": "VnUiJ0uvBsPsKvsJ0V0WiNAT",
        "secret_key": "DkUuPrjrMu7us4WqTJEHHELf4SteYT66"
    },
    "zhipu": {
        "api_key": "5f7c4ae29b914146a9fab43344f15913.JclT1dXQCQqS3AHf"
    },
    "weather": {
        "api_url": "https://devapi.qweather.com/v7/weather/now",
        "api_key": "411e05242bb44cc890649383c16e2fb5",
        "location": "101090201" # 保定
    }
}

I2C_CONFIG = {

    "sensors": {
        "bh1750": {
            "bus_number": 1, 
            "address": 0x23,
            "enabled": True
        },
        "bmp280": {
            "bus_number": 5,  
            "address": 0x76,
            "enabled": True
        },
        "dht20": {
            "bus_number": 0, 
            "address": 0x38,
            "enabled": True
        }
    }
}

# --- 自动创建目录的函数 ---
def _create_directories():
    """
    在模块被导入时，自动创建所有必需的目录。
    这个函数是内部的，所以用下划线开头。
    """
    print("--- Initializing Config ---")
    print(f"Application data will be stored in: {APP_BASE_DIR}")
    # 遍历PATHS字典，找到所有以'_dir'结尾的键，并创建对应的目录
    for key, path in PATHS.items():
        if key.endswith('_dir'):
             print(f"  - Ensuring directory exists: {path}")
             os.makedirs(path, exist_ok=True)
    print("--- Config Initialized ---")
    


# --- 初始化 ---
# 在config.py被第一次导入时，自动执行创建目录的函数
_create_directories()