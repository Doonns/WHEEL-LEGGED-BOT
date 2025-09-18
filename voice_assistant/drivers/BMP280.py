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

# --- BMP280 I2C Configuration ---
BMP280_I2C_ADDRESS_DEFAULT = 0x76 # Default BMP280 I2C address
BMP280_I2C_BUS_DEFAULT = 0

# BMP280 Registers
BMP280_REGISTER_CHIPID = 0xD0
BMP280_REGISTER_DIG_T1 = 0x88
BMP280_REGISTER_DIG_T2 = 0x8A
BMP280_REGISTER_DIG_T3 = 0x8C
BMP280_REGISTER_DIG_P1 = 0x8E
BMP280_REGISTER_DIG_P2 = 0x90
BMP280_REGISTER_DIG_P3 = 0x92
BMP280_REGISTER_DIG_P4 = 0x94
BMP280_REGISTER_DIG_P5 = 0x96
BMP280_REGISTER_DIG_P6 = 0x98
BMP280_REGISTER_DIG_P7 = 0x9A
BMP280_REGISTER_DIG_P8 = 0x9C
BMP280_REGISTER_DIG_P9 = 0x9E
BMP280_REGISTER_CONTROL = 0xF4
BMP280_REGISTER_CONFIG = 0xF5
BMP280_REGISTER_TEMPDATA = 0xFA
BMP280_REGISTER_PRESSUREDATA = 0xF7


class BMP280:
    """
    Driver for the BMP280 sensor, using the logic provided by the user,
    adapted for the smbus2 library on standard Python.
    """
    def __init__(self, bus, i2c_addr=BMP280_I2C_ADDRESS_DEFAULT):
        self.bus = bus
        self.i2c_addr = i2c_addr
        
        # Initialize placeholders for calibration data
        self.dig_T1 = 0
        self.dig_T2 = 0
        self.dig_T3 = 0
        self.dig_P1 = 0
        self.dig_P2 = 0
        self.dig_P3 = 0
        self.dig_P4 = 0
        self.dig_P5 = 0
        self.dig_P6 = 0
        self.dig_P7 = 0
        self.dig_P8 = 0
        self.dig_P9 = 0
        
        # Placeholders for compensated data
        self.T = 0.0 # Temperature in Celsius
        self.P = 0.0 # Pressure in Pascals

    def _short(self, dat):
        """Converts a 16-bit unsigned value to a signed value."""
        if dat > 32767:
            return dat - 65536
        else:
            return dat

    def _set_reg(self, reg, dat):
        """Writes a byte to a specified register."""
        try:
            self.bus.write_byte_data(self.i2c_addr, reg, dat)
        except OSError as e:
            print(f"I2C Write Error at reg 0x{reg:02X} with value 0x{dat:02X}: {e}")

    def _get_reg(self, reg):
        """Reads a single byte from a specified register."""
        try:
            return self.bus.read_byte_data(self.i2c_addr, reg)
        except OSError as e:
            print(f"I2C Read Error at reg 0x{reg:02X}: {e}")
            return 0

    def _get_2reg(self, reg):
        """Reads a 16-bit little-endian value from a starting register."""
        try:
            # read_word_data reads a 16-bit little-endian value, which is
            # equivalent to (t[0] + t[1]*256)
            return self.bus.read_word_data(self.i2c_addr, reg)
        except OSError as e:
            print(f"I2C Read Error at reg 0x{reg:02X}: {e}")
            return 0
            
    def begin(self):
        """
        Initializes the sensor by checking the chip ID, reading calibration
        data, and configuring the sensor for normal operation.
        Returns True on success, False on failure.
        """
        # 1. Check Chip ID
        chip_id = self._get_reg(BMP280_REGISTER_CHIPID)
        if chip_id != 0x58:
            print(f"Error: BMP280 Chip ID mismatch. Expected 0x58, but got 0x{chip_id:02X}.")
            return False
        
        print("BMP280 Chip ID 0x58 confirmed.")

        # 2. Read calibration coefficients
        self.dig_T1 = self._get_2reg(BMP280_REGISTER_DIG_T1)
        self.dig_T2 = self._short(self._get_2reg(BMP280_REGISTER_DIG_T2))
        self.dig_T3 = self._short(self._get_2reg(BMP280_REGISTER_DIG_T3))
        self.dig_P1 = self._get_2reg(BMP280_REGISTER_DIG_P1)
        self.dig_P2 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P2))
        self.dig_P3 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P3))
        self.dig_P4 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P4))
        self.dig_P5 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P5))
        self.dig_P6 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P6))
        self.dig_P7 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P7))
        self.dig_P8 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P8))
        self.dig_P9 = self._short(self._get_2reg(BMP280_REGISTER_DIG_P9))
        print("BMP280 calibration coefficients read successfully.")

        # 3. Write configuration
        # This configuration sets:
        # 0xF4 (ctrl_meas): Temp oversampling x1, Pressure oversampling x4, Normal mode
        # 0x2F = 0b00101111 (osrs_t=1, osrs_p=3, mode=3)
        self._set_reg(BMP280_REGISTER_CONTROL, 0x2F)
        # 0xF5 (config): Standby time 250ms, IIR Filter coeff 4
        # 0x0C = 0b00001100 (t_sb=2, filter=3, spi3w_en=0)
        self._set_reg(BMP280_REGISTER_CONFIG, 0x0C)
        print("BMP280 configured for normal operation.")
        
        return True

    def _read_and_compensate(self):
        """
        Reads raw sensor data and applies compensation formulas to calculate
        temperature and pressure. The results are stored in self.T and self.P.
        """
        # Read raw 20-bit temperature data
        adc_T = (self._get_reg(0xFA) << 12) + (self._get_reg(0xFB) << 4) + (self._get_reg(0xFC) >> 4)
        
        # Temperature compensation
        var1 = (((adc_T >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
        var2 = (((((adc_T >> 4) - self.dig_T1) * ((adc_T >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        t_fine = var1 + var2
        self.T = ((t_fine * 5 + 128) >> 8) / 100.0
        
        # Pressure compensation
        var1 = (t_fine >> 1) - 64000
        var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 1)
        var2 = (var2 >> 2) + (self.dig_P4 << 16)
        var1 = (((self.dig_P3 * ((var1 >> 2) * (var1 >> 2)) >> 13) >> 3) + ((self.dig_P2 * var1) >> 1)) >> 18
        var1 = ((32768 + var1) * self.dig_P1) >> 15
        
        if var1 == 0:
            print("Warning: Division by zero in pressure calculation.")
            self.P = 0 # Avoid exception
            return
            
        # Read raw 20-bit pressure data
        adc_P = (self._get_reg(0xF7) << 12) + (self._get_reg(0xF8) << 4) + (self._get_reg(0xF9) >> 4)
        p = ((1048576 - adc_P) - (var2 >> 12)) * 3125
        
        if p < 0x80000000:
            p = (p << 1) // var1
        else:
            p = (p // var1) * 2
            
        var1 = (self.dig_P9 * (((p >> 3) * (p >> 3)) >> 13)) >> 12
        var2 = (((p >> 2)) * self.dig_P8) >> 13
        
        self.P = p + ((var1 + var2 + self.dig_P7) >> 4)
        
    def readTemperature(self):
        """Returns the temperature in Celsius."""
        self._read_and_compensate()
        return self.T

    def readPressure(self):
        """Returns the pressure in hectoPascals (hPa)."""
        self._read_and_compensate()
        return self.P / 100.0 # Convert from Pascals (Pa) to hPa

    def getAltitude(self, sea_level_hpa=1013.25):
        """Calculates the altitude in meters."""
        self._read_and_compensate()
        pressure_pa = self.P
        # Formula: h = 44330 * [1 - (P/P0)^(1/5.255)]
        # P0 is sea level pressure in Pa
        return 44330 * (1 - (pressure_pa / (sea_level_hpa * 100)) ** (1 / 5.255))

    def power_off(self):
        """Puts the sensor into sleep mode."""
        self._set_reg(BMP280_REGISTER_CONTROL, 0x00)

    def power_on(self):
        """Wakes the sensor up to normal mode with previous settings."""
        self._set_reg(BMP280_REGISTER_CONTROL, 0x2F)


def signal_handler(signal, frame):
    """Handles Ctrl+C exit signal"""
    print("\nExiting demo. Cleaning up...")
    sys.exit(0)


# --- Main BMP280 Demo Function ---
def bmp280_demo_main_smbus():
    """
    BMP280 sensor reading demo main function using the new logic.
    """
    bus_num_str = input(f"Please input I2C BUS num (default is {BMP280_I2C_BUS_DEFAULT}): ")
    if not bus_num_str:
        bus_num = BMP280_I2C_BUS_DEFAULT
    else:
        try:
            bus_num = int(bus_num_str)
        except ValueError:
            print("Invalid input for I2C BUS number. Please enter an integer.")
            return

    device_addr_hex_str = input(f"Please input I2C device address (Hex, default is {BMP280_I2C_ADDRESS_DEFAULT:02X}): ")
    if not device_addr_hex_str:
        device_addr = BMP280_I2C_ADDRESS_DEFAULT
    else:
        try:
            device_addr = int(device_addr_hex_str, 16)
        except ValueError:
            print("Invalid input for I2C device address. Please enter a Hex string (e.g., 76).")
            return
    
    print(f"Scanning I2C bus {bus_num} for devices...")
    try:
        if os.system('which i2cdetect > /dev/null') == 0:
            os.system(f'sudo i2cdetect -y -r {bus_num}')
        else:
            print("i2cdetect command not found. Skipping I2C scan.")
    except Exception as e:
        print(f"Error running i2cdetect: {e}")

    global bus # Make bus globally accessible for cleanup
    bus = None
    try:
        print(f"Initializing SMBus for I2C bus {bus_num}...")
        bus = smbus2.SMBus(bus_num)
        print("SMBus initialized.")

        # *** Instantiate the new BMP280 class ***
        bmp280 = BMP280(bus, i2c_addr=device_addr)
        
        if not bmp280.begin():
            print("BMP280 initialization failed. Aborting.")
            if bus:
                bus.close()
            return

    except FileNotFoundError:
        print(f"Error: I2C bus /dev/i2c-{bus_num} not found.")
        print("Please ensure I2C is enabled on your system and the bus ID is correct.")
        return
    except OSError as e:
        print(f"An OSError occurred during initialization: {e}")
        if bus:
            bus.close()
        return
    except Exception as e:
        print(f"An unexpected error occurred during initialization: {e}")
        if bus:
            bus.close()
        return

    print("\nStarting BMP280 data reading loop...")
    print("Press CTRL+C to exit.")

    try:
        while True:
            temperature = bmp280.readTemperature()
            pressure_hpa = bmp280.readPressure()
            altitude = bmp280.getAltitude() # Uses the same data, no extra read needed

            if temperature is not None and pressure_hpa is not None:
                print(f"Temp: {temperature:.2f} °C | Pressure: {pressure_hpa:.2f} hPa | Altitude: {altitude:.2f} m")
            else:
                print("Failed to read BMP280 data.")

            time.sleep(1) 

    except KeyboardInterrupt:
        print("\nExiting demo loop.")
    finally:
        if bus:
            print("Closing SMBus.")
            bus.close()
        print("Demo finished.")

