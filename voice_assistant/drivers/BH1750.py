import sys
import time
import signal

# Attempt to import the smbus2 library, which is required for I2C communication.
try:
    import smbus2
except ImportError:
    print("Error: 'smbus2' library not found.")
    print("Please install it using: pip install smbus2")
    sys.exit(1)


# =======================================================================
# BH1750 Library Class
# =======================================================================

class BH1750:
    """
    A library class for the BH1750 ambient light sensor.
    This class can be copied into other projects.
    """
    # --- I2C Configuration ---
    # Default address is 0x23. If ADDR pin is pulled high, it's 0x5c.
    DEFAULT_I2C_ADDR = 0x23

    # --- Sensor Commands and Modes (from datasheet) ---
    PWR_OFF = 0x00
    PWR_ON = 0x01
    RESET = 0x07

    # Continuous measurement modes
    CONT_HIRES_1 = 0x10  # 1 lx resolution, 120ms measurement time
    CONT_HIRES_2 = 0x11  # 0.5 lx resolution, 120ms
    CONT_LOWRES = 0x13   # 4 lx resolution, 16ms

    # One-shot measurement modes (sensor goes to Power Down after measurement)
    ONCE_HIRES_1 = 0x20  # 1 lx resolution, 120ms
    ONCE_HIRES_2 = 0x21  # 0.5 lx resolution, 120ms
    ONCE_LOWRES = 0x23   # 4 lx resolution, 16ms

    def __init__(self, bus, addr=DEFAULT_I2C_ADDR):
        """
        Initializes the BH1750 sensor.
        :param bus: An initialized smbus2.SMBus object.
        :param addr: The I2C address of the sensor.
        """
        self.bus = bus
        self.addr = addr
        self.mode = None
        
        # Power on and reset the sensor to a known, clean state upon initialization.
        self.power_on()
        self.reset()

    def _set_mode(self, mode):
        """
        Internal helper to send a command/mode to the sensor.
        :param mode: The command byte to send.
        """
        self.mode = mode
        try:
            # For the BH1750, sending a command is a simple I2C byte write.
            self.bus.write_byte(self.addr, self.mode)
        except OSError as e:
            print(f"I2C Write Error to addr 0x{self.addr:02X}: {e}")
            # This is a critical error, the sensor will not be in the correct state.

    def power_off(self):
        """Turn the sensor off to save power."""
        self._set_mode(self.PWR_OFF)

    def power_on(self):
        """Turn the sensor on."""
        self._set_mode(self.PWR_ON)

    def reset(self):
        """
        Reset the sensor's data register. This is useful for clearing
        stale readings. Per the datasheet, the device must be on to reset.
        """
        self.power_on()
        self._set_mode(self.RESET)

    def read_luminance(self, mode=ONCE_HIRES_1):
        """
        Performs a measurement and returns the result in lux.
        :param mode: The measurement mode to use (e.g., BH1750.ONCE_HIRES_1).
        :return: The measured luminance in lux (float), or None on error.
        """
        # Set the measurement mode. This also starts the measurement for one-shot modes.
        self._set_mode(mode)

        # Wait for the measurement to complete.
        # Datasheet specifies max 180ms for high-res and 24ms for low-res.
        if mode in (self.CONT_LOWRES, self.ONCE_LOWRES):
            time.sleep(0.024)  # Wait 24ms for low-res mode
        else:
            time.sleep(0.180)  # Wait 180ms for high-res modes

        try:
            # Read the 2-byte result from the sensor.
            data = self.bus.read_i2c_block_data(self.addr, 0, 2)
        except OSError as e:
            print(f"I2C Read Error from addr 0x{self.addr:02X}: {e}")
            return None

        # Combine the two bytes (Most Significant Byte, Least Significant Byte)
        # into a single 16-bit integer.
        raw_value = (data[0] << 8) | data[1]

        # Convert the raw value to lux using the formula from the datasheet.
        # The 1.2 is the typical sensitivity adjustment factor.
        # Mode 2 (0.5lx resolution) requires dividing the result by 2.
        if mode in (self.CONT_HIRES_2, self.ONCE_HIRES_2):
            lux = raw_value / (1.2 * 2.0)
        else:
            lux = raw_value / 1.2

        return lux


# =======================================================================
# Main Execution Block (Demonstration)
# =======================================================================

def signal_handler(sig, frame):
    """Handles Ctrl+C exit signal cleanly."""
    print("\nExiting program.")
    # The 'finally' block in main() will handle closing the bus
    sys.exit(0)

def main_demo():
    """Main function to run the BH1750 sensor demo."""
    print("--- BH1750 Ambient Light Sensor Demo ---")

    # --- User Input for I2C Configuration ---
    try:
        bus_num_str = input("Please input I2C BUS number (e.g., 1 for /dev/i2c-1): ")
        bus_num = int(bus_num_str)
        
        addr_str = input(f"Please input I2C device address in hex (default is 0x{BH1750.DEFAULT_I2C_ADDR:02X}): ")
        if not addr_str:
            device_addr = BH1750.DEFAULT_I2C_ADDR
        else:
            device_addr = int(addr_str, 16)
    except ValueError:
        print("Invalid input. Please enter a valid integer for the bus and hex for the address.")
        return

    bus = None
    try:
        # Initialize the I2C bus
        bus = smbus2.SMBus(bus_num)
        print(f"I2C bus {bus_num} opened successfully.")
        
        # Initialize the BH1750 sensor using our library class
        sensor = BH1750(bus, addr=device_addr)
        print(f"BH1750 sensor initialized at address 0x{device_addr:02X}.")

        print("\nStarting measurements... Press CTRL+C to exit.")
        while True:
            # Use the library class method to read luminance.
            # We'll use the default one-shot, high-resolution mode.
            lux = sensor.read_luminance()

            if lux is not None:
                # Format the output to two decimal places
                print(f"Ambient Light: {lux:.2f} lx")
            else:
                print("Failed to read from sensor. Check connections and address.")
            
            # Wait for 2 seconds before the next reading
            time.sleep(2)

    except FileNotFoundError:
        print(f"\nError: I2C bus /dev/i2c-{bus_num} not found.")
        print("Please ensure I2C is enabled on your system and the bus number is correct.")
    except OSError as e:
        print(f"\nOSError: {e}.")
        print("Check I2C wiring, device address, and pull-up resistors.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # This block ensures the I2C bus is closed properly on exit or error.
        if bus:
            bus.close()
            print("I2C bus closed.")

