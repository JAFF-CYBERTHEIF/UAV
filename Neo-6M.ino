#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(2, 3);  // RX, TX

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop() {
  String data = gpsSerial.readStringUntil('\n');

  if (data.startsWith("$GNRMC") || data.startsWith("$GPRMC")) {
    // Extract time, date, latitude, longitude, speed, and course from RMC sentence
    int comma1 = data.indexOf(",");
    int comma2 = data.indexOf(",", comma1 + 1);
    int comma3 = data.indexOf(",", comma2 + 1);
    int comma4 = data.indexOf(",", comma3 + 1);
    int comma5 = data.indexOf(",", comma4 + 1);
    int comma6 = data.indexOf(",", comma5 + 1);
    int comma7 = data.indexOf(",", comma6 + 1);
    int comma8 = data.indexOf(",", comma7 + 1);

    String time = data.substring(comma1 + 1, comma2);
    float latitude = data.substring(comma3 + 1, comma4).toFloat();
    float longitude = data.substring(comma5 + 1, comma6).toFloat();
    float speed = data.substring(comma7 + 1, comma8).toFloat();
    float course = data.substring(comma8 + 1).toFloat();

    // Extract date from RMC sentence
    int dateStart = data.indexOf(",", comma8 + 1) + 1;
    int dateEnd = data.indexOf(",", dateStart);
    String date = data.substring(dateStart, dateEnd);

    // Concatenate the data into a comma-separated list
    String output = time + "," + date + "," + String(latitude, 6) + "," + String(longitude, 6) + "," + String(speed, 2) + "," + String(course, 2);
    Serial.println(output);
    
  } 
}

