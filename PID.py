import time
import board
import busio
import adafruit_bno055
import adafruit_pca9685
from simple_pid import PID

# Initialize the BNO055 IMU sensor and PCA9685 servo controller
i2c = busio.I2C(board.SCL, board.SDA)
imu = adafruit_bno055.BNO055(i2c)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50

# Set up the PID controller for each axis
pid_x = PID(1, 0, 0, setpoint=0)
pid_y = PID(1, 0, 0, setpoint=0)
pid_z = PID(1, 0, 0, setpoint=0)

# Define the servo channels for each control surface
SERVO_SPEED = 0
SERVO_RUDDER = 1
SERVO_ELEVATOR = 2
SERVO_AILERON_L = 3
SERVO_AILERON_R = 4

# Set initial servo positions to neutral
pca.channels[SERVO_SPEED].duty_cycle = 0x7FFF
pca.channels[SERVO_RUDDER].duty_cycle = 0x7FFF
pca.channels[SERVO_ELEVATOR].duty_cycle = 0x7FFF
pca.channels[SERVO_AILERON_L].duty_cycle = 0x7FFF
pca.channels[SERVO_AILERON_R].duty_cycle = 0x7FFF

# Main control loop
while True:
    # Read sensor data from the IMU
    ax, ay, az = imu.acceleration
    gx, gy, gz = imu.gyro

    # Use the PID controller to calculate servo positions for each axis
    servo_speed = int(pid_x(ax) * 1000 + 1500)
    servo_rudder = int(pid_y(gy) * 1000 + 1500)
    servo_elevator = int(pid_z(az) * 1000 + 1500)

    # Apply the calculated positions to the corresponding servos
    pca.channels[SERVO_SPEED].duty_cycle = servo_speed
    pca.channels[SERVO_RUDDER].duty_cycle = servo_rudder
    pca.channels[SERVO_ELEVATOR].duty_cycle = servo_elevator
    pca.channels[SERVO_AILERON_L].duty_cycle = 0x7FFF
    pca.channels[SERVO_AILERON_R].duty_cycle = 0x7FFF

    # Wait for a short time to prevent excessive loop frequency
    time.sleep(0.01)
