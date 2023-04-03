#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(3, 4); // RX, TX pins for NEO-6M GPS module
String gpsData;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop() {
  if (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gpsData += c;
    if (c == '\n') { // End of line
      if (gpsData.startsWith("$GPGGA")) { // GGA sentence
        // Parse GPS data
        int commaIndex = gpsData.indexOf(',');
        int time = gpsData.substring(commaIndex+1, commaIndex+7).toInt();
        int latDegrees = gpsData.substring(18, 20).toInt();
        float latMinutes = gpsData.substring(20, commaIndex).toFloat();
        float latitude = latDegrees + latMinutes/60.0;
        if (gpsData.charAt(commaIndex+8) == 'S') {
          latitude *= -1; // Southern hemisphere
        }
        commaIndex = gpsData.indexOf(',', commaIndex+1);
        int lonDegrees = gpsData.substring(commaIndex+1, commaIndex+4).toInt();
        float lonMinutes = gpsData.substring(commaIndex+4, commaIndex+commaIndex+10).toFloat();
        float longitude = lonDegrees + lonMinutes/60.0;
        if (gpsData.charAt(commaIndex+11) == 'W') {
          longitude *= -1; // Western hemisphere
        }
        commaIndex = gpsData.indexOf(',', commaIndex+1);
        int fixQuality = gpsData.substring(commaIndex+1, commaIndex+2).toInt();
        commaIndex = gpsData.indexOf(',', commaIndex+1);
        int numSatellites = gpsData.substring(commaIndex+1, commaIndex+3).toInt();
        commaIndex = gpsData.indexOf(',', commaIndex+1);
        float altitude = gpsData.substring(commaIndex+1, gpsData.indexOf(',', commaIndex+1)).toFloat();
        Serial.print("Latitude: ");
        Serial.print(latitude, 6);
        Serial.print(", Longitude: ");
        Serial.print(longitude, 6);
        Serial.print(", Altitude: ");
        Serial.print(altitude);
        Serial.print(", Fix Quality: ");
        Serial.print(fixQuality);
        Serial.print(", Satellites: ");
        Serial.println(numSatellites);
      }
      gpsData = ""; // Clear GPS data string
    }
  }
}
