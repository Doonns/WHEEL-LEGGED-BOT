import sys
import signal
import os
import time

# Import the smbus2 library
try:
    import smbus2
except ImportError:
    print("Error: 'smbus2' library not found.")
    print("Please install it using: pip install smbus2")
    sys.exit(1)

# --- AHT20 I2C Configuration ---
AHT20_I2C_ADDRESS = 0x38
AHT20_I2C_BUS_DEFAULT = 0

# AHT20 Commands (following MicroPython DHT20 lib and AHT20 datasheet)
# 0x00 is used as the command/register address for I2C communication
CMD_MEASURE = [0xAC, 0x33, 0x00]
CMD_INITIALIZE = [0xBE, 0x08, 0x00]

# AHT20 Status Bits
STATUS_BUSY_MASK = 0x80
STATUS_CAL_ENABLE_MASK = 0x08

# Delays (referencing MicroPython DHT20 lib and AHT20 datasheet)
POST_POWER_ON_DELAY_MS = 40
INIT_CMD_DELAY_MS = 10
POST_MEASUREMENT_WAIT_MS = 80
READ_DATA_DELAY_US = 100

# CRC Polynomial
CRC_POLYNOMIAL = 0x31

def signal_handler(signal, frame):
    """Handles Ctrl+C exit signal"""
    print("\nExiting demo. Cleaning up...")
    sys.exit(0)

def calculate_crc8(data_buffer, polynomial=CRC_POLYNOMIAL):
    """Calculates CRC8 checksum (consistent with MicroPython lib logic)"""
    crc = 0xFF
    for byte_val in data_buffer:
        crc ^= byte_val
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ polynomial
            else:
                crc = (crc << 1)
    return crc & 0xFF

def aht20_init_sequence_smbus(bus):
    """Sends AHT20 initialization command using smbus2."""
    print("Sending AHT20 initialization command...")

    try:
        # Using literal 0x00 as the command/register address
        bus.write_i2c_block_data(AHT20_I2C_ADDRESS, 0x00, CMD_INITIALIZE)
        time.sleep(INIT_CMD_DELAY_MS / 1000.0)

        status_byte_after_init = bus.read_byte_data(AHT20_I2C_ADDRESS, 0x00)
        print(f"Status byte after init command: {status_byte_after_init:#04x}")
        
        if not (status_byte_after_init & STATUS_CAL_ENABLE_MASK):
            print("Warning: AHT20 CAL bit (Bit 3) is not set after initialization. Readings may be inaccurate.")
        else:
            print("AHT20 CAL bit is set. Initialization appears successful.")

    except OSError as e:
        print(f"An OSError occurred during AHT20 initialization: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred during AHT20 initialization: {e}")
        return False

    return True


def read_aht20_data_smbus(bus):
    """
    Reads AHT20 raw data and processes it strictly according to MicroPython DHT20 lib logic.
    Returns (temperature_c, humidity_rh, crc_ok)
    """
    try:
        # --- 1. Trigger Measurement ---
        print("Triggering AHT20 measurement...")
        # Using literal 0x00 as the command/register address
        bus.write_i2c_block_data(AHT20_I2C_ADDRESS, 0x00, CMD_MEASURE)
        
        time.sleep(POST_MEASUREMENT_WAIT_MS / 1000.0) 

        # --- 2. Wait for Measurement Completion ---
        start_time = time.time()
        while True:
            status_byte = bus.read_byte_data(AHT20_I2C_ADDRESS, 0x00)
            if status_byte is None:
                print("Error reading status byte to check measurement completion.")
                return None, None, False

            if not (status_byte & STATUS_BUSY_MASK):
                break # Measurement complete

            if time.time() - start_time > (POST_MEASUREMENT_WAIT_MS / 1000.0) + 0.1:
                print("Timeout waiting for measurement to complete.")
                return None, None, False
            time.sleep(READ_DATA_DELAY_US / 1000000.0)

        # --- 3. Read Data ---
        data_bytes = bus.read_i2c_block_data(AHT20_I2C_ADDRESS, 0x00, 7)
        if data_bytes is None or len(data_bytes) < 7:
            print(f"Failed to read complete data block. Received: {data_bytes}")
            return None, None, False
        
        # --- 4. CRC Check ---
        read_crc = data_bytes[6]
        calculated_crc = calculate_crc8(data_bytes[:6])
        crc_ok = (calculated_crc == read_crc)

        # --- 5. Data Parsing and Conversion (strictly following MicroPython DHT20 lib logic) ---
        
        # Humidity parsing
        humidity_raw_combined = 0
        humidity_raw_combined |= data_bytes[1]
        humidity_raw_combined <<= 8
        humidity_raw_combined |= data_bytes[2]
        humidity_raw_combined <<= 8
        humidity_raw_combined |= data_bytes[3]
        
        humidity_processed = humidity_raw_combined >> 4 
        
        humidity_rh = (humidity_processed / 1048576.0) * 100.0

        # Temperature parsing
        temperature_raw_combined = 0
        temperature_raw_combined |= data_bytes[3]
        temperature_raw_combined <<= 8
        temperature_raw_combined |= data_bytes[4]
        temperature_raw_combined <<= 8
        temperature_raw_combined |= data_bytes[5]
        
        temperature_processed = temperature_raw_combined & 0xFFFFF

        temperature_c = (temperature_processed / 1048576.0) * 200.0 - 50.0

        # Clamp values to valid ranges
        humidity_rh = max(0.0, min(100.0, humidity_rh))
        temperature_c = max(-40.0, min(80.0, temperature_c))

        return temperature_c, humidity_rh, crc_ok

    except OSError as e:
        print(f"An OSError occurred during AHT20 data reading: {e}")
        return None, None, False
    except Exception as e:
        print(f"An unexpected error occurred during AHT20 data reading: {e}")
        return None, None, False


def aht20_demo_main_smbus():
    """
    AHT20 sensor reading demo main function (using smbus2), referencing MicroPython DHT20 lib.
    """
    bus_num_str = input(f"Please input I2C BUS num (default is {AHT20_I2C_BUS_DEFAULT}): ")
    if not bus_num_str:
        bus_num = AHT20_I2C_BUS_DEFAULT
    else:
        try:
            bus_num = int(bus_num_str)
        except ValueError:
            print("Invalid input for I2C BUS number. Please enter an integer.")
            return

    device_addr_hex_str = input(f"Please input I2C device num (Hex, AHT20 is 0x{AHT20_I2C_ADDRESS:02X}): ")
    if not device_addr_hex_str:
        device_addr = AHT20_I2C_ADDRESS
    else:
        try:
            device_addr = int("0x" + device_addr_hex_str, 16)
        except ValueError:
            print("Invalid input for I2C device number. Please enter a Hex string (e.g., 38 or 0x38).")
            return
    
    if device_addr != AHT20_I2C_ADDRESS:
        print(f"Warning: Using user-provided address 0x{device_addr:02X} instead of default AHT20 address 0x{AHT20_I2C_ADDRESS:02X}.")

    print(f"Scanning I2C bus {bus_num} for devices...")
    try:
        if os.system('which i2cdetect > /dev/null') == 0:
            os.system(f'sudo i2cdetect -y -r {bus_num}')
        else:
            print("i2cdetect command not found. Skipping I2C scan.")
    except Exception as e:
        print(f"Error running i2cdetect: {e}")


    bus = None
    try:
        print(f"Initializing SMBus for I2C bus {bus_num}...")
        bus = smbus2.SMBus(bus_num)
        print("SMBus initialized.")

        if not aht20_init_sequence_smbus(bus):
            print("AHT20 initialization failed. Aborting.")
            if bus:
                bus.close()
            return

    except FileNotFoundError:
        print(f"Error: I2C bus /dev/i2c-{bus_num} not found.")
        print("Please ensure I2C is enabled on your system and the bus ID is correct.")
        return
    except OSError as e:
        print(f"An OSError occurred during SMBus initialization: {e}")
        if bus:
            bus.close()
        return
    except Exception as e:
        print(f"An unexpected error occurred during SMBus initialization: {e}")
        if bus:
            bus.close()
        return

    print("\nStarting AHT20 data reading loop...")
    print("Press CTRL+C to exit.")

    try:
        while True:
            temperature, humidity, crc_ok = read_aht20_data_smbus(bus)

            if temperature is not None and humidity is not None:
                status_str = ""
                if not crc_ok:
                    status_str = " (CRC FAILED!)"
                print(f"Temp: {temperature:.2f} °C | Humidity: {humidity:.2f} %RH{status_str}")
            else:
                print("Failed to read AHT20 data.")

            time.sleep(1.5)

    except KeyboardInterrupt:
        print("\nExiting demo loop.")
    finally:
        if bus:
            print("Closing SMBus.")
            bus.close()
        print("Demo finished.")

