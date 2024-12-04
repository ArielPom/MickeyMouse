import time
import serial
import VL53L1X

UPDATE_TIME_MICROS = 15000
INTER_MEASUREMENT_PERIOD_MILLIS = 20

# Initialize the VL53L1X sensor
tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
tof.open()
tof.set_timing(UPDATE_TIME_MICROS, INTER_MEASUREMENT_PERIOD_MILLIS)
# tof.set_timing_budget(100)
# tof.set_distance_mode(3)
tof.start_ranging(1)

# Initialize serial communication with the ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200)

# Initialize variables for sliding average
window_size = 1  # Size of the sliding window (adjust as needed)
distance_buffer = []

try:
    while True:
        distance_in_mm = tof.get_distance()
        distance_cm = int(distance_in_mm / 10)
        distance_cm = max(0, min(distance_cm, 256))
        
        # Update distance buffer with latest measurement
        distance_buffer.append(distance_cm)
        if len(distance_buffer) > window_size:
            distance_buffer.pop(0)  # Remove oldest measurement if buffer exceeds window size
        
        # Calculate sliding average
        sliding_avg = sum(distance_buffer) // len(distance_buffer)
        
        print(distance_in_mm)
        
        time.sleep(INTER_MEASUREMENT_PERIOD_MILLIS / 1000.0)

except KeyboardInterrupt:
    # Handle Ctrl+C gracefully
    pass

finally:
    # Stop ranging and close the VL53L1X sensor
    tof.stop_ranging()
    tof.close()
    # Close the serial port
    ser.close()
