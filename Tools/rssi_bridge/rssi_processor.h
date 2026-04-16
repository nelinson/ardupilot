#pragma once

#include <cstdint>

class RssiProcessor {
public:
    // Converts dBm to MAVLink RSSI (0..254). Value 255 is reserved for "unknown".
    static uint8_t dbm_to_mavlink_rssi(float dbm);

    // Optional EMA smoothing. Pass alpha in [0..1]. alpha=1 -> no smoothing.
    bool ema_update(float sample, float alpha, float &out_filtered);

private:
    bool _ema_init = false;
    float _ema = 0.0f;
};

