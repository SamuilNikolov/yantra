#define LORA
#include <SPI.h>
#include "LoRa-SOLDERED.h"
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
const int csPin = 10;      // LoRa radio chip select
const int resetPin = 4;    // LoRa radio reset
const int irqPin = 2;      // LoRa radio IRQ pin

const uint8_t START_BYTE = 0x7E; // Example start byte
const uint8_t END_BYTE = 0x7F;   // Example end byte

int16_t VelocityBuffer = 0;
float LatBuffer = 0.0;
float LonBuffer = 0.0;

#define SEALEVELPRESSURE_HPA (1015)

Adafruit_BMP3XX bmp;

void setup() {
    // Serial.begin(11s5200);

    Serial2.begin(57600);
    // while (!Serial);

    // Set up LoRa
    LoRa.setPins(csPin, resetPin, irqPin);
    if (!LoRa.begin(433E6)) {
        // Serial.println("LoRa initialization failed!");
        while (1);
    }

    Wire.begin();
    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
      while (1);
    }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);


    // Serial.println("LoRa initialized successfully.");
    LoRa.beginPacket();
    LoRa.print("KO4MIM");
    LoRa.endPacket();

    LoRa.onTxDone(onTxDone); // Set callback on transmission done
    sendData();              // Start the first transmission
    }

void loop() {
    // Example data
    while (Serial2.available()) {
      String gpsData = Serial2.readStringUntil('\n');

      // Check for $GPGGA sentence for latitude and longitude
      if (gpsData.startsWith("$GPGGA")) {
        String latitudeStr = parseNMEAField(gpsData, 2);
        String latDirection = parseNMEAField(gpsData, 3);
        String longitudeStr = parseNMEAField(gpsData, 4);
        String lonDirection = parseNMEAField(gpsData, 5);

        // Convert NMEA to decimal degrees
        float latitude = convertNMEALatLon(latitudeStr, latDirection);
        float longitude = convertNMEALatLon(longitudeStr, lonDirection);

        // Update buffers
        LatBuffer = latitude; // Keep up to 6 decimal places
        LonBuffer = longitude;
      }

      // Check for $GPVTG sentence for velocity
      if (gpsData.startsWith("$GPVTG")) {
        String velocity = parseNMEAField(gpsData, 7);
        // Serial.println(velocity);
        // Update VelocityBuffer
        VelocityBuffer = velocity.toInt();
        // Serial.println(VelocityBuffer);
      }
    }
    // delay(1000); // Adjust as needed
}

float convertNMEALatLon(String nmeaCoord, String direction) {
  float coord = nmeaCoord.toFloat();
  int degrees = int(coord / 100);
  float minutes = coord - (degrees * 100);
  float decimalDegrees = degrees + (minutes / 60.0);

  if (direction == "S" || direction == "W") {
    decimalDegrees = -decimalDegrees;
  }
  return decimalDegrees;
}

String parseNMEAField(String data, int fieldIndex) {
  int startIndex = 0;
  int endIndex = -1;
  int commaCount = 0;

  for (int i = 0; i < data.length(); i++) {
    if (data[i] == ',') {
      commaCount++;
      if (commaCount == fieldIndex) {
        startIndex = i + 1;
      } else if (commaCount == fieldIndex + 1) {
        endIndex = i;
        break;
      }
    }
  }

  if (endIndex == -1) endIndex = data.length();
  
  return data.substring(startIndex, endIndex);
}

void sendData() {
  int16_t altitude = (int16_t) bmp.readAltitude(SEALEVELPRESSURE_HPA);
    // Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    // Create a buffer with start and end bytes
    uint8_t buffer[14]; // 12 bytes for data + 2 for start/end + 2 for CRC (optional)
    buffer[0] = START_BYTE;
    memcpy(&buffer[1], &LatBuffer, sizeof(float));      // 4 bytes
    memcpy(&buffer[5], &LonBuffer, sizeof(float));     // 4 bytes
    memcpy(&buffer[9], &VelocityBuffer, sizeof(int16_t));    // 2 bytes
    memcpy(&buffer[11], &altitude, sizeof(int16_t));   // 2 bytes
    buffer[13] = END_BYTE;
    // Serial.println("1");

    // Send the buffer
    LoRa.beginPacket();
    LoRa.write(buffer, sizeof(buffer));
    if (millis() % (600*1000) == 0) {
        LoRa.print("KO4MIM");   // Callsign
    }
    LoRa.endPacket(true);

    // Debug: Print the raw bytes
    // Serial.print("Transmitted bytes: ");
    for (int i = 0; i < sizeof(buffer); i++) {
        // Serial.print(buffer[i], HEX);
        // Serial.print(" ");
    }
    // Serial.println();
    // Serial.println("2");
}

void onTxDone() {
  // Serial.println("3");
  sendData();              // Send the next packet
  // Serial.println("4");
}