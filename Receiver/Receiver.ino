#include <SPI.h>
#include "LoRa-SOLDERED.h"
#include "LoRa.h"

// LoRa pin definitions
const int csPin = 10;      // LoRa chip select
const int resetPin = 2;    // LoRa reset
const int irqPin = 3;      // LoRa IRQ pin

// Constants for framing
const uint8_t START_BYTE = 0x7E; // Start byte
const uint8_t END_BYTE = 0x7F;   // End byte

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize LoRa
    LoRa.setPins(csPin, resetPin, irqPin);
    if (!LoRa.begin(433E6)) { // Adjust frequency if needed
        // If initialization fails, hang here
        while (1);
    }
    LoRa.setSpreadingFactor(7); // Faster data rate
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        uint8_t buffer[21]; // 14 bytes total: 1 start + 12 payload + 1 end
        int index = 0;
        // Read the packet into the buffer
        while (LoRa.available() && index < sizeof(buffer)) {
            buffer[index++] = LoRa.read();
        }

        // Check start/end bytes
        if (index == 21 && buffer[0] == START_BYTE && buffer[20] == END_BYTE) {
            // Transmit the raw packet over Serial
            Serial.write(buffer, 21);
        }
        // If invalid, do nothing and wait for the next packet
    }
}
