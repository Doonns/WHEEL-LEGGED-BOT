#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import pyaudio
import threading

CHUNK = 4096
RATE = 44100
CHANNELS = 1
FORMAT = pyaudio.paInt16

PC_IP = '192.168.31.194'  # PC的IP
PC_PORT = 50007

pa = pyaudio.PyAudio()

print("=== Available Audio Input Devices ===")
for i in range(pa.get_device_count()):
    dev = pa.get_device_info_by_index(i)
    if dev['maxInputChannels'] > 0:
        print(f"Index {i}: {dev['name']}")

input_device_index = 0  # 根据打印结果设置正确设备索引

try:
    stream_in = pa.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK,
                        input_device_index=input_device_index)
except Exception as e:
    print(f"[ERROR] Failed to open audio input device: {e}")
    exit(1)

try:
    stream_out = pa.open(format=FORMAT,
                         channels=CHANNELS,
                         rate=RATE,
                         output=True,
                         frames_per_buffer=CHUNK)
except Exception as e:
    print(f"[ERROR] Failed to open audio output device: {e}")
    stream_in.stop_stream()
    stream_in.close()
    exit(1)

s = socket.socket()
try:
    s.connect((PC_IP, PC_PORT))
except Exception as e:
    print(f"[ERROR] Failed to connect to PC {PC_IP}:{PC_PORT}: {e}")
    stream_in.stop_stream()
    stream_in.close()
    stream_out.stop_stream()
    stream_out.close()
    exit(1)

print("? Connected to PC. Start duplex audio...")

def recv_play():
    try:
        while True:
            data = s.recv(CHUNK)
            if not data:
                break
            stream_out.write(data)
    except Exception as e:
        print(f"[ERROR] Exception in receiving audio: {e}")

recv_thread = threading.Thread(target=recv_play, daemon=True)
recv_thread.start()

try:
    while True:
        data = stream_in.read(CHUNK, exception_on_overflow=False)
        s.sendall(data)
except KeyboardInterrupt:
    print("\nInterrupted by user, exiting.")
except Exception as e:
    print(f"[ERROR] Exception during transmission: {e}")
finally:
    print("Cleaning up...")
    stream_in.stop_stream()
    stream_in.close()
    stream_out.stop_stream()
    stream_out.close()
    s.close()
