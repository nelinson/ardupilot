#include "rssi_processor.h"

#include <algorithm>

uint8_t RssiProcessor::dbm_to_mavlink_rssi(float dbm)
{
    // Map [-120, -50] dBm linearly to [0, 254].
    // 255 is special ("unknown") and must be avoided.
    if (dbm <= -120.0f) {
        return 0;
    }
    if (dbm >= -50.0f) {
        return 254;
    }
    const float scaled = (dbm + 120.0f) * 254.0f / 70.0f;
    const float clipped = std::max(0.0f, std::min(254.0f, scaled));
    return static_cast<uint8_t>(clipped);
}

bool RssiProcessor::ema_update(float sample, float alpha, float &out_filtered)
{
    if (alpha <= 0.0f) {
        return false;
    }
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    if (!_ema_init) {
        _ema = sample;
        _ema_init = true;
    } else {
        _ema = alpha * sample + (1.0f - alpha) * _ema;
    }
    out_filtered = _ema;
    return true;
}

