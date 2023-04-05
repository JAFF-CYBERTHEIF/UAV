import math
import time
import board
import busio
import adafruit_bno055
import adafruit_gps
import RPi.GPIO as GPIO

# Set up GPIO pins for PWM signals
GPIO.setmode(GPIO.BCM)
GPIO.setup(12, GPIO.OUT)  # Servo motor
GPIO.setup(13, GPIO.OUT)  # BLDC motor
servo_pwm = GPIO.PWM(12, 50)
bldc_pwm = GPIO.PWM(13, 50)

# Initialize the sensors
i2c = busio.I2C(board.SCL, board.SDA)
bno055 = adafruit_bno055.BNO055(i2c)
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart)

# Set up the PID controllers for roll, pitch, and heading
def pid_roll(desired_roll, roll, previous_error, dt):
    kp = 1.0
    ki = 0.0
    kd = 0.0
    error = desired_roll - roll
    integral = previous_error + error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, error

def pid_pitch(desired_pitch, pitch, previous_error, dt):
    kp = 1.0
    ki = 0.0
    kd = 0.0
    error = desired_pitch - pitch
    integral = previous_error + error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, error

def pid_heading(desired_heading, heading, previous_error, dt):
    kp = 1.0
    ki = 0.0
    kd = 0.0
    error = (desired_heading - heading + 180) % 360 - 180
    integral = previous_error + error * dt
    derivative = (error - previous_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, error

# Main loop
current_lat = 37.7749  # Starting latitude (dummy value for now)
current_lon = -122.4194  # Starting longitude (dummy value for now)
current_alt = 100  # Starting altitude (dummy value for now)
previous_roll_error = 0
previous_pitch_error = 0
previous_heading_error = 0
roll = 0
pitch = 0
heading = 0

while True:
    # Read the GPS and IMU data
    gps.update()
    if not gps.has_fix:
        continue
    lat = gps.latitude
    lon = gps.longitude
    speed = gps.speed_knots
    course = gps.track_angle_deg
    roll, pitch, heading = bno055.euler

    # Calculate the distance and bearing to the target coordinates
    target_lat = 37.7748  # Target latitude (dummy value for now)
    target_lon = -122.4193  # Target longitude (dummy value for now)
    target_alt = 200  # Target altitude (dummy value for now)
    R = 6371000  # Earth's radius in meters
    lat1 = math.radians(current_lat)
    lon1 = math.radians(current_lon)
    lat2 = math.radians(target_lat)
    lon2 = math.radians(target_lon)
    dlat = lat2 - lat
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c 
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = (math.atan2(y, x) * 180 / math.pi + 360) % 360

    # Calculate the errors for roll, pitch, and heading
    desired_roll = 0
    desired_pitch = 0
    desired_heading = bearing
    roll_output, previous_roll_error = pid_roll(desired_roll, roll, previous_roll_error, 0.1)
    pitch_output, previous_pitch_error = pid_pitch(desired_pitch, pitch, previous_pitch_error, 0.1)
    heading_output, previous_heading_error = pid_heading(desired_heading, heading, previous_heading_error, 0.1)

    # Apply the PID outputs to the servo and BLDC motors
    servo_pwm.ChangeDutyCycle(7.5 + roll_output + pitch_output)
    bldc_pwm.ChangeDutyCycle(7.5 + heading_output)

    # Check if the target altitude has been reached
    if current_alt < target_alt:
        bldc_pwm.ChangeDutyCycle(10)  # Increase motor speed to climb
    elif current_alt > target_alt:
        bldc_pwm.ChangeDutyCycle(5)  # Decrease motor speed to descend

    # Check if the target coordinates have been reached
    if distance < 5:  # Within 5 meters of the target
        break  # End the loop and land the plane

    # Wait for a short time before the next iteration
    time.sleep(0.1)
servo_pwm.ChangeDutyCycle(7.5) # Set the servo to neutral position
bldc_pwm.ChangeDutyCycle(7.5) # Set the motor to idle
time.sleep(2) # Wait for a few seconds for the plane to land
servo_pwm.stop() # Stop the PWM signals
bldc_pwm.stop()
GPIO.cleanup()

