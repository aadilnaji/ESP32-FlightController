#ifndef IBUS_RECEIVER_H
#define IBUS_RECEIVER_H

#include <Arduino.h>

/** FlySky IBUS receiver: 115200 8N1, 32-byte frames ~7ms, 14 channels @ 1000-2000us. */

// ===== PROTOCOL =====
#define IBUS_FRAME_LENGTH    32
#define IBUS_HEADER          0x20
#define IBUS_COMMAND_DATA    0x40
#define IBUS_CHANNELS        14
#define IBUS_MIN             1000
#define IBUS_MAX             2000
#define IBUS_MID             1500
#define IBUS_TIMEOUT_MS      500

class IBUSReceiver {
public:
    uint16_t failsafeValues[IBUS_CHANNELS];
    uint16_t channels[IBUS_CHANNELS];

    IBUSReceiver(HardwareSerial& serial, int rxPin = -1, int txPin = -1);

    void begin();

    /** Poll serial; returns true on new frame. */
    bool update();

    bool isValid() const;

    /** Returns channel value in us (1000-2000), or failsafe if invalid. Channel is 1-indexed. */
    uint16_t getChannel(uint8_t channel) const;

    /** Returns channel normalized to -1.0..1.0, or 0.0 if invalid. */
    float getChannelNormalized(uint8_t channel) const;

    uint32_t timeSinceLastFrame() const;
    uint32_t getFrameCount() const { return frameCount; }
    uint32_t getErrorCount() const { return errorCount; }

private:
    HardwareSerial& serial;
    int rxPin, txPin;

    uint8_t buffer[IBUS_FRAME_LENGTH];
    uint8_t bufferIndex;

    uint32_t lastFrameTime;
    uint32_t frameCount;
    uint32_t errorCount;
    bool valid;

    bool parseFrame();
    uint16_t calculateChecksum();
};

#endif
