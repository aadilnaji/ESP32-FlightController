#include "IBUSReceiver.h"

IBUSReceiver::IBUSReceiver(HardwareSerial& serial, int rxPin, int txPin)
    : serial(serial), rxPin(rxPin), txPin(txPin),
      bufferIndex(0), lastFrameTime(0), frameCount(0), errorCount(0), valid(false) {

    for (int i = 0; i < IBUS_CHANNELS; i++) {
        channels[i] = IBUS_MID;
        failsafeValues[i] = IBUS_MID;
    }
    failsafeValues[0] = IBUS_MIN;  // throttle fails to min
}

void IBUSReceiver::begin() {
    if (rxPin >= 0) {
        serial.begin(115200, SERIAL_8N1, rxPin, txPin);
    } else {
        serial.begin(115200, SERIAL_8N1);
    }

    while (serial.available()) {
        serial.read();
    }

    bufferIndex = 0;
    lastFrameTime = millis();
    valid = false;
}

bool IBUSReceiver::update() {
    bool newFrame = false;

    if (millis() - lastFrameTime > IBUS_TIMEOUT_MS) {
        valid = false;
    }

    while (serial.available()) {
        uint8_t c = serial.read();

        if (bufferIndex == 0) {
            if (c == IBUS_FRAME_LENGTH) {
                buffer[bufferIndex++] = c;
            }
        }
        else if (bufferIndex == 1) {
            if (c == IBUS_COMMAND_DATA) {
                buffer[bufferIndex++] = c;
            } else {
                bufferIndex = 0;
            }
        }
        else {
            buffer[bufferIndex++] = c;

            if (bufferIndex >= IBUS_FRAME_LENGTH) {
                if (parseFrame()) {
                    lastFrameTime = millis();
                    frameCount++;
                    valid = true;
                    newFrame = true;
                } else {
                    errorCount++;
                }
                bufferIndex = 0;
            }
        }
    }

    return newFrame;
}

bool IBUSReceiver::parseFrame() {
    uint16_t checksum = calculateChecksum();
    uint16_t receivedChecksum = buffer[30] | (buffer[31] << 8);

    if (checksum != receivedChecksum) {
        return false;
    }

    // 14 channels, 2 bytes each, little-endian
    for (int i = 0; i < IBUS_CHANNELS; i++) {
        uint16_t value = buffer[2 + i * 2] | (buffer[3 + i * 2] << 8);
        if (value >= IBUS_MIN && value <= IBUS_MAX) {
            channels[i] = value;
        }
    }

    return true;
}

uint16_t IBUSReceiver::calculateChecksum() {
    uint16_t checksum = 0xFFFF;
    for (int i = 0; i < 30; i++) {
        checksum -= buffer[i];
    }
    return checksum;
}

bool IBUSReceiver::isValid() const {
    return valid && (millis() - lastFrameTime < IBUS_TIMEOUT_MS);
}

uint16_t IBUSReceiver::getChannel(uint8_t channel) const {
    if (channel < 1 || channel > IBUS_CHANNELS) {
        return IBUS_MID;
    }
    if (!isValid()) {
        return failsafeValues[channel - 1];
    }
    return channels[channel - 1];
}

float IBUSReceiver::getChannelNormalized(uint8_t channel) const {
    uint16_t value = getChannel(channel);
    return ((float)value - IBUS_MID) / (float)(IBUS_MAX - IBUS_MID) * 2.0f;
}

uint32_t IBUSReceiver::timeSinceLastFrame() const {
    return millis() - lastFrameTime;
}
