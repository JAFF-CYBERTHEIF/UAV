import time
import math
import board
import busio
import adafruit_mpu6050
import serial
import pynmea2

# Initialize MPU-6500 sensor
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Initialize NEO-6M GPS module
gps = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

# Set target latitude and longitude coordinates
target_lat = 37.7749
target_lon = -122.4194

# Calculate distance between current and target coordinates using Haversine formula
def calc_distance(lat1, lon1, lat2, lon2):
    R = 6371e3  # Earth's radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi/2) * math.sin(delta_phi/2) + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda/2) * math.sin(delta_lambda/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    d = R * c  # Distance between coordinates in meters
    return d

# Set autopilot control parameters
Kp = 0.1  # Proportional gain
Ki = 0.01  # Integral gain
Kd = 0.01  # Derivative gain
dt = 0.1  # Time step in seconds
error_sum = 0
last_error = 0

# Main loop
while True:
    # Read accelerometer and gyroscope data from MPU-6500
    accel_x, accel_y, accel_z = mpu.acceleration
    gyro_x, gyro_y, gyro_z = mpu.gyro

    # Read GPS data from NEO-6M
    gps_data = gps.readline().decode("utf-8")
    if gps_data.startswith('$GPGGA'):
        msg = pynmea2.parse(gps_data)
        curr_lat = msg.latitude
        curr_lon = msg.longitude

        # Calculate distance between current and target coordinates
        dist = calc_distance(curr_lat, curr_lon, target_lat, target_lon)

        # Calculate heading error using gyro data
        heading = math.atan2(-accel_y, accel_x) * 180 / math.pi
        target_heading = math.atan2(target_lon - curr_lon, target_lat - curr_lat) * 180 / math.pi
        error = target_heading - heading

        # Update error sum and calculate PID output
        error_sum += error * dt
        error_rate = (error - last_error) / dt
        pid_output = Kp * error + Ki * error_sum + Kd * error_rate

        # Print sensor and control data
        print("Distance to target:", dist, "meters")
        print("Current heading:", heading, "degrees")
        print("Target heading:", target_heading, "degrees")
        print("Heading error:", error, "degrees")
        print("PID output:", pid_output)

        last_error = error
    time.sleep(dt)
