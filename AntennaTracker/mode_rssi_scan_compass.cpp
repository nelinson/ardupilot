/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   AntennaTracker - compass-gated RSSI scan (mode RSSI_SCAN_C)
   Fixed yaw/pitch PWM during drive; RSSI sampled on AHRS grid; then point at best.
 */

#include "mode.h"
#include "Tracker.h"

#if AP_RSSI_ENABLED
#include <AP_RSSI/AP_RSSI.h>
#endif

#include <AP_Vehicle/ModeReason.h>

void ModeRSSIScanCompass::handoff_to_rssi_scan()
{
    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: handoff RSSI_SCAN");
    tracker.set_mode(tracker.mode_rssi_scan, ModeReason::MISSION_END);
}

void ModeRSSIScanCompass::reset_for_entry()
{
    _initialized = false;
    _phase = Phase::YAW_DRIVE;
    _yaw_next_k = 0;
    _pitch_next_k = 0;
    _point_stable = 0;
    _yaw_cumulative_cw = 0.0f;
    _pitch_cumulative_up = 0.0f;
}

static constexpr float POINT_ERR_DEG = 2.0f;
static constexpr uint16_t POINT_STABLE_REQ = 25;
static constexpr uint32_t POINT_TIMEOUT_MS = 8000;
// Ignore per-tick heading spikes (EKF/compass jumps); real bench motion is slow at 50 Hz.
static constexpr float MAX_YAW_DEG_PER_TICK = 25.0f;
static constexpr float MAX_PITCH_DEG_PER_TICK = 20.0f;

float ModeRSSIScanCompass::wrap_360_deg(float d)
{
    return wrap_360(d);
}

float ModeRSSIScanCompass::read_rssi_avg() const
{
#if AP_RSSI_ENABLED
    float sum = 0.0f;
    const int n = MAX(1, (int)tracker.g.rssi_scan_samples.get());
    for (int i = 0; i < n; i++) {
        sum += AP::rssi()->read_receiver_rssi();
    }
    return sum / n;
#else
    return 0.0f;
#endif
}

void ModeRSSIScanCompass::apply_yaw_pitch_pwm(uint16_t yaw_pwm, uint16_t pitch_pwm)
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, yaw_pwm);
    SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_yaw);
    SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_pitch, pitch_pwm);
    SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_pitch);
}

void ModeRSSIScanCompass::release_pwm_overrides()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_yaw, 0.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tracker_pitch, 0.0f);
}

void ModeRSSIScanCompass::enter_yaw_point()
{
    release_pwm_overrides();
    tracker.nav_status.bearing = _best_yaw_deg;
    tracker.nav_status.pitch = constrain_float(
        AP::ahrs().get_pitch_deg() + tracker.g.pitch_trim,
        tracker.g.pitch_min,
        tracker.g.pitch_max);
    _point_enter_ms = AP_HAL::millis();
    _point_stable = 0;
    _phase = Phase::YAW_POINT;
    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Auto yaw %.0f", (double)_best_yaw_deg);
}

void ModeRSSIScanCompass::enter_pitch_drive()
{
    _pitch_start_deg = AP::ahrs().get_pitch_deg();
    _pitch_arc_deg = constrain_float(float(tracker.g.rssi_scp_arc.get()), 1.0f, 180.0f);
    _pitch_step_deg = MAX(1, (int)tracker.g.rssi_scan_tilt_step.get());
    _pitch_max_k = int(floorf(_pitch_arc_deg / float(_pitch_step_deg)));
    _pitch_next_k = 0;
    _best_rssi_pitch = -1.0f;
    _best_pitch_deg = _pitch_start_deg;
    _prev_pitch_deg = _pitch_start_deg;
    _pitch_cumulative_up = 0.0f;

    const float pitch_tgt = _pitch_start_deg + _pitch_arc_deg;
    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Pitch start cur=%.0f tgt=%.0f arc=%.0f",
                     (double)_pitch_start_deg, (double)pitch_tgt, (double)_pitch_arc_deg);

    uint16_t yaw_trim = 1500;
    uint16_t pitch_pwm = uint16_t(tracker.g.rssi_scp_pwm.get());
    if (SRV_Channel *ych = SRV_Channels::get_channel_for(SRV_Channel::k_tracker_yaw)) {
        yaw_trim = ych->get_trim();
    }
    apply_yaw_pitch_pwm(yaw_trim, pitch_pwm);
    _last_progress_ms = AP_HAL::millis();
    _phase = Phase::PITCH_DRIVE;
}

void ModeRSSIScanCompass::enter_pitch_point()
{
    release_pwm_overrides();
    tracker.nav_status.bearing = _best_yaw_deg;
    tracker.nav_status.pitch = constrain_float(
        _best_pitch_deg + tracker.g.pitch_trim,
        tracker.g.pitch_min,
        tracker.g.pitch_max);
    _point_enter_ms = AP_HAL::millis();
    _point_stable = 0;
    _phase = Phase::PITCH_POINT;
    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Auto pitch %.0f", (double)_best_pitch_deg);
}

void ModeRSSIScanCompass::update()
{
#if !AP_RSSI_ENABLED
    gcs().send_text(MAV_SEVERITY_ERROR, "RSSI_SC: no RSSI build");
    return;
#else
    if (!AP::rssi()->enabled()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "RSSI_SC: RSSI disabled");
        return;
    }
#endif

    if (!_initialized) {
        _yaw_start_deg = wrap_360_deg(AP::ahrs().get_yaw_deg());
        _yaw_arc_deg = constrain_float(float(tracker.g.rssi_scy_arc.get()), 1.0f, 360.0f);
        _yaw_step_deg = MAX(1, (int)tracker.g.rssi_scan_pan_step.get());
        _yaw_max_k = int(floorf(_yaw_arc_deg / float(_yaw_step_deg)));
        _yaw_next_k = 0;
        _best_rssi_yaw = -1.0f;
        _best_yaw_deg = _yaw_start_deg;
        _prev_yaw_deg = _yaw_start_deg;
        _yaw_cumulative_cw = 0.0f;

        if (_yaw_arc_deg >= 359.5f) {
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Yaw start cur=%.0f arc=%.0f (CW full turn)",
                             (double)_yaw_start_deg, (double)_yaw_arc_deg);
        } else {
            const float yaw_tgt = wrap_360_deg(_yaw_start_deg + _yaw_arc_deg);
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Yaw start cur=%.0f tgt=%.0f arc=%.0f",
                             (double)_yaw_start_deg, (double)yaw_tgt, (double)_yaw_arc_deg);
        }

        const uint16_t yaw_pwm = (uint16_t)constrain_int32(tracker.g.rssi_scy_pwm.get(), 800, 2200);
        uint16_t pitch_trim = 1500;
        if (SRV_Channel *pch = SRV_Channels::get_channel_for(SRV_Channel::k_tracker_pitch)) {
            pitch_trim = pch->get_trim();
        }
        apply_yaw_pitch_pwm(yaw_pwm, pitch_trim);
        // NAV_CONTROLLER_OUTPUT / logs: actual AHRS aim (rssi_sim and GCS use this; yaw not from PID here)
        tracker.nav_status.bearing = wrap_360_deg(AP::ahrs().get_yaw_deg());
        tracker.nav_status.pitch = AP::ahrs().get_pitch_deg();
        _last_progress_ms = AP_HAL::millis();
        _initialized = true;
    }

    const uint32_t now = AP_HAL::millis();
    const uint32_t to_ms = uint32_t(MAX(1000, (int)tracker.g.rssi_sc_to.get()));

    switch (_phase) {
    case Phase::YAW_DRIVE: {
        const uint16_t yaw_pwm = (uint16_t)constrain_int32(tracker.g.rssi_scy_pwm.get(), 800, 2200);
        uint16_t pitch_trim = 1500;
        if (SRV_Channel *pch = SRV_Channels::get_channel_for(SRV_Channel::k_tracker_pitch)) {
            pitch_trim = pch->get_trim();
        }
        apply_yaw_pitch_pwm(yaw_pwm, pitch_trim);
        // NAV_CONTROLLER_OUTPUT / logs: actual AHRS aim (rssi_sim and GCS use this; yaw not from PID here)
        tracker.nav_status.bearing = wrap_360_deg(AP::ahrs().get_yaw_deg());
        tracker.nav_status.pitch = AP::ahrs().get_pitch_deg();

        const float cur_yaw = wrap_360_deg(AP::ahrs().get_yaw_deg());
        const float dpsi = wrap_180(cur_yaw - _prev_yaw_deg);
        _prev_yaw_deg = cur_yaw;
        if (dpsi > 0.0f) {
            _yaw_cumulative_cw += MIN(dpsi, MAX_YAW_DEG_PER_TICK);
        }

        if (_yaw_cumulative_cw + 0.5f >= _yaw_arc_deg) {
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Yaw done best=%.0f rssi=%.0f%%",
                             (double)_best_yaw_deg, (double)(MAX(0.0f, _best_rssi_yaw) * 100.0f));
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Go yaw %.0f", (double)_best_yaw_deg);
            enter_yaw_point();
            break;
        }

        while (_yaw_next_k <= _yaw_max_k &&
               _yaw_cumulative_cw >= (float(_yaw_next_k * _yaw_step_deg)) - 2.0f) {
            const float r = read_rssi_avg();
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Yaw %.0f rssi=%.0f%%",
                            (double)cur_yaw, (double)(r * 100.0f));
            if (r > _best_rssi_yaw) {
                _best_rssi_yaw = r;
                _best_yaw_deg = cur_yaw;
            }
            _yaw_next_k++;
            _last_progress_ms = now;
        }

        if (now - _last_progress_ms > to_ms) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RSSI_SC: Yaw timeout");
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Yaw done best=%.0f rssi=%.0f%%",
                            (double)_best_yaw_deg, (double)(MAX(0.0f, _best_rssi_yaw) * 100.0f));
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Go yaw %.0f", (double)_best_yaw_deg);
            enter_yaw_point();
        }
        break;
    }

    case Phase::YAW_POINT: {
        update_auto();
        const float err = wrap_180(_best_yaw_deg - AP::ahrs().get_yaw_deg());
        if (fabsf(err) < POINT_ERR_DEG) {
            _point_stable++;
        } else {
            _point_stable = 0;
        }
        if (_point_stable >= POINT_STABLE_REQ) {
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Yaw hold OK");
            enter_pitch_drive();
            break;
        }
        if (now - _point_enter_ms > POINT_TIMEOUT_MS) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RSSI_SC: Yaw point timeout");
            enter_pitch_drive();
        }
        break;
    }

    case Phase::PITCH_DRIVE: {
        uint16_t yaw_trim = 1500;
        const uint16_t pitch_pwm = (uint16_t)constrain_int32(tracker.g.rssi_scp_pwm.get(), 800, 2200);
        if (SRV_Channel *ych = SRV_Channels::get_channel_for(SRV_Channel::k_tracker_yaw)) {
            yaw_trim = ych->get_trim();
        }
        apply_yaw_pitch_pwm(yaw_trim, pitch_pwm);
        // NAV_CONTROLLER_OUTPUT / logs: actual AHRS aim (rssi_sim and GCS use this; pitch not from PID here)
        tracker.nav_status.bearing = wrap_360_deg(AP::ahrs().get_yaw_deg());
        tracker.nav_status.pitch = AP::ahrs().get_pitch_deg();

        const float cur_p = AP::ahrs().get_pitch_deg();
        const float dp = cur_p - _prev_pitch_deg;
        _prev_pitch_deg = cur_p;
        if (dp > 0.0f) {
            _pitch_cumulative_up += MIN(dp, MAX_PITCH_DEG_PER_TICK);
        }

        if (_pitch_cumulative_up + 0.5f >= _pitch_arc_deg) {
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Pitch done best=%.0f rssi=%.0f%%",
                            (double)_best_pitch_deg, (double)(MAX(0.0f, _best_rssi_pitch) * 100.0f));
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Go pitch %.0f", (double)_best_pitch_deg);
            enter_pitch_point();
            break;
        }

        while (_pitch_next_k <= _pitch_max_k &&
               _pitch_cumulative_up >= (float(_pitch_next_k * _pitch_step_deg)) - 2.0f) {
            const float r = read_rssi_avg();
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Pitch %.0f rssi=%.0f%%",
                            (double)cur_p, (double)(r * 100.0f));
            if (r > _best_rssi_pitch) {
                _best_rssi_pitch = r;
                _best_pitch_deg = cur_p;
            }
            _pitch_next_k++;
            _last_progress_ms = now;
        }

        if (now - _last_progress_ms > to_ms) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RSSI_SC: Pitch timeout");
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Pitch done best=%.0f rssi=%.0f%%",
                            (double)_best_pitch_deg, (double)(MAX(0.0f, _best_rssi_pitch) * 100.0f));
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Go pitch %.0f", (double)_best_pitch_deg);
            enter_pitch_point();
        }
        break;
    }

    case Phase::PITCH_POINT: {
        update_auto();
        const float err = _best_pitch_deg - AP::ahrs().get_pitch_deg();
        if (fabsf(err) < POINT_ERR_DEG) {
            _point_stable++;
        } else {
            _point_stable = 0;
        }
        if (_point_stable >= POINT_STABLE_REQ) {
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Pitch hold OK");
            handoff_to_rssi_scan();
            break;
        }
        if (now - _point_enter_ms > POINT_TIMEOUT_MS) {
            gcs().send_text(MAV_SEVERITY_WARNING, "RSSI_SC: Pitch point timeout");
            handoff_to_rssi_scan();
        }
        break;
    }

    }
}
