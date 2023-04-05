import serial

ser = serial.Serial("/dev/ttyAMA0" ,baudrate=9600)

def read_GPS:
  GPS = ser.readline().decode().strip().split(",")
  #returns a list [Time, date, lat, long, speed, course]
  return GPS
