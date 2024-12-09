#include <SPI.h>
#include "LoRa-SOLDERED.h"

const int csPin = 10;      // LoRa radio chip select
const int resetPin = 4;    // LoRa radio reset
const int irqPin = 2;      // LoRa IRQ pin

const uint8_t START_BYTE = 0x7E; // Start byte
const uint8_t END_BYTE = 0x7F;   // End byte

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Set up LoRa
    LoRa.setPins(csPin, resetPin, irqPin);
    if (!LoRa.begin(433E6)) { // Adjust frequency based on your region
        Serial.println("LoRa initialization failed!");
        while (1);
    }

    Serial.println("LoRa receiver initialized.");
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        uint8_t buffer[16]; // 14 bytes for data + extra for validation
        int index = 0;

        // Read the packet into the buffer
        while (LoRa.available() && index < sizeof(buffer)) {
            buffer[index++] = LoRa.read();
        }

        // Validate start and end bytes
        if (buffer[0] == START_BYTE && buffer[index - 1] == END_BYTE) {
            // Transmit only the payload bytes (excluding start and end bytes)
            for (int i = 1; i < index - 1; i++) {
                Serial.write(buffer[i]); // Send each byte directly over serial
            }
        }
    }
}
