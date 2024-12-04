import serial
import time
import numpy as np
import read_maze_wall
import VL53L1X

default_trajectory = [
        {"x": 1000, "y": 0, "theta": 0, "operation": 0},
        {"x": 1000, "y": 0, "theta": np.pi/2, "operation": 1},  
        {"x": 1000, "y": 1000, "theta": np.pi/2, "operation": 0},
        {"x": 1000, "y": 1000, "theta": np.pi, "operation": 1},
        {"x": 0, "y": 1000, "theta": np.pi, "operation": 0},
        {"x": 0, "y": 1000, "theta": 3*np.pi/2, "operation": 1},
        {"x": 0, "y": 0, "theta": 3*np.pi/2, "operation": 0},
        {"x": 0, "y": 0, "theta": 2*np.pi, "operation": 1}  
    ]

class TOF:
    UPDATE_TIME_MICROS = 15000
    INTER_MEASUREMENT_PERIOD_MILLIS = 20

    def __init__(self, i2c_bus=1, i2c_address=0x29):
        self.tof = VL53L1X.VL53L1X(i2c_bus=i2c_bus, i2c_address=i2c_address)
        self.tof.open()
        self.tof.set_timing(self.UPDATE_TIME_MICROS, self.INTER_MEASUREMENT_PERIOD_MILLIS)
        self.tof.start_ranging(1)
        self.pre_t_tof = 0
        self.curr_t_tof = time.monotonic()
        self.dt_tof = 0

    def get_distance(self):
        self.curr_t_tof = time.monotonic()
        self.dt_t_tof = self.curr_t_tof - self.pre_t_tof
        if self.dt_t_tof >= 0.2:
            self.pre_t_tof = time.monotonic()
            return self.tof.get_distance()
        return None

    def stop(self):
        self.tof.stop_ranging()
        self.tof.close()

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.monotonic()

    def compute(self, error):
        current_time = time.monotonic()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# Function to send commands to the ESP32
def send_command(ser, pwm1, pwm2, dir1, dir2):
    try: 	
        ser.write(bytes([abs(pwm1)]))
        ser.write(bytes([abs(pwm2)]))
        ser.write(bytes([dir1]))
        ser.write(bytes([dir2])) 
    except Exception as e:
        print(e) 

# Function to read (x, y, theta) from the ESP32
def read_position(ser, timeout=5):
    start_time = time.time()
    
    while True:
        # Check for timeout
        if time.time() - start_time > timeout:
            raise TimeoutError("Reading from ESP32 timed out.")
        
        try:
            line = ser.readline().decode('utf-8').strip()
        except UnicodeDecodeError as e:
            continue
        
        if line:
            data = line.split(";")
            print(data,',')
            if len(data) == 3:
                try:
                    x = float(data[0].strip())
                    y = float(data[1].strip())
                    theta = float(data[2].strip())
                    return x, y, theta
                except ValueError as e:
                    continue
            else:
                continue

if __name__ == "__main__":

    # Get trajectory steps
    trajectory = default_trajectory
    trajectory = read_maze_wall.read_udp_maze()
    print(trajectory)

    # Open serial connection to ESP32
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)  # Wait for the serial connection to initialize

    # Initialize TOF sensor
    tof_sensor = TOF()

    # Initial position
    v_c = 250 # [mm/sec]
    r = 36.3/2 # in [mm]
    tof2robot = 90 # in [mm]
    delta_x = 0
    delta_y = 0
    prev_x = 0
    prev_y = 0

    for step in trajectory:
        x, y, theta = read_position(ser)
        target_x = step["x"]
        target_y = step["y"]
        target_theta = step["theta"]
        o = step["operation"]
        wall = step["wall"]
        pid_pos = PID(80.0, 0.00, 5.0)
        pid_ori = PID(90.0, 0.00, 5.0)
        while True:
            x, y, theta = read_position(ser)
            x = x - delta_x
            y = y - delta_y
            if o == 0:
                dis_d = np.sqrt((target_y - y)**2 + (target_x - x)**2)
                theta_error = target_theta - theta
                w_c = pid_pos.compute(theta_error)
                vR = v_c + w_c
                vL = v_c - w_c
                if dis_d <= 100:
                    tof_x = ((wall - distance_in_mm - tof2robot) - prev_x)*np.cos(target_theta)
                    tof_y = ((wall - distance_in_mm - tof2robot) - prev_y)*np.sin(target_theta)
                    delta_x = (x - tof_x)*np.cos(target_theta) # in [mm]
                    delta_y = (y - tof_y)*np.sin(target_theta) # in [mm]                     
                    delta_dis = np.sqrt(delta_x**2 + delta_y**2)
                    print(delta_dis)
                    if delta_dis > 100:
                        continue
                    prev_x = x
                    prev_y = y
                    send_command(ser, 0, 0, 1, 1)
                    break      
            if o == 1:
                theta_error = target_theta - theta
                w_c = pid_ori.compute(theta_error)
                vR = w_c
                vL = - w_c         
                if abs(theta_error) <= 0.1:
                    send_command(ser, 0, 0, 1, 1)
                    break	    
            phi_dot1 = vL/(r) # [rad/sec]
            phi_dot2 = vR/(r) # [rad/sec]
            rpm1 = 60*phi_dot1/(2*np.pi)
            rpm2 = 60*phi_dot2/(2*np.pi)
            pwm1 = 255*rpm1/760
            pwm2 = 255*rpm2/760        
            # Cap the PID outputs to the range [-255, 255] for PWM
            pwm1 = int(max(min(pwm1, 255), -255))
            pwm2 = int(max(min(pwm2, 255), -255))

            # Determine direction based on PID output sign
            dir1 = 1 if pwm1 > 0 else 0
            dir2 = 1 if pwm2 > 0 else 0

            distance_in_mm = tof_sensor.get_distance()
            if distance_in_mm is not None:
                if distance_in_mm < 100 and o != 1:
                    send_command(ser, 0, 0, 1, 1) 
                    tof_x = ((wall - distance_in_mm - tof2robot) - prev_x)*np.cos(target_theta)
                    tof_y = ((wall - distance_in_mm - tof2robot) - prev_y)*np.sin(target_theta)
                    delta_x = (x - tof_x)*np.cos(target_theta) # in [mm]
                    delta_y = (y - tof_y)*np.sin(target_theta) # in [mm]                    
                    break	              
            send_command(ser, pwm1, pwm2, dir1, dir2)   

    tof_sensor.stop()
    ser.close()