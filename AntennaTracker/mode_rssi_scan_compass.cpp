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
#include <AP_AHRS/AP_AHRS.h>

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
    _debug_last_ms = 0;
    _entry_ms = 0;
    _yaw_settle_until_ms = 0;
    _rssi_wait_until_ms = 0;
    _rssi_wait_last_msg_ms = 0;
    _yaw_nomotion_start_ms = 0;
    _yaw_drive_started = false;
    _yaw_progress_sign = 0;
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

    // Hold yaw neutral during pitch drive. On continuous-rotation yaw setups,
    // using the channel trim here can unintentionally command rotation.
    const uint16_t pitch_pwm = uint16_t(tracker.g.rssi_scp_pwm.get());
    apply_yaw_pitch_pwm(1500, pitch_pwm);
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

    auto rssi_link_ready = []() -> bool {
        AP_RSSI *rssi = AP::rssi();
        if (rssi == nullptr) {
            return false;
        }
#if AP_RSSI_HTTP_ENABLED
        if (rssi->type() == AP_RSSI::RssiType::SOLO8_HTTP) {
            // This returns -1 until SOLO8_HTTP has a *recent valid* reading,
            // which implies mesh discovery + successful localrfstatus parse.
            return rssi->read_receiver_link_quality() > -0.5f;
        }
#endif
        // Non-HTTP backends don't have a reliable "ready" event; don't block.
        return true;
    };

    if (!_initialized) {
        _entry_ms = AP_HAL::millis();
        // Allow AHRS/EKF to finish initial yaw alignment before we start integrating scan progress.
        // Without this, heading "jumps" during startup can consume the whole arc without any servo motion.
        _yaw_settle_until_ms = _entry_ms + 3000;
        // Give the RSSI backend a moment to produce its first valid sample
        // (especially important for HTTP mesh discovery at boot).
        _rssi_wait_until_ms = _entry_ms + 30000;
        _rssi_wait_last_msg_ms = 0;
        _yaw_nomotion_start_ms = 0;
        _yaw_drive_started = false;
        _yaw_progress_sign = 0;

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

        uint16_t pitch_trim = 1500;
        if (SRV_Channel *pch = SRV_Channels::get_channel_for(SRV_Channel::k_tracker_pitch)) {
            pitch_trim = pch->get_trim();
        }
        // Keep yaw neutral while AHRS/EKF settles at startup; do not spin yet.
        apply_yaw_pitch_pwm(1500, pitch_trim);
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

        const float cur_yaw = wrap_360_deg(AP::ahrs().get_yaw_deg());
        const float dpsi = wrap_180(cur_yaw - _prev_yaw_deg);

        // Before we start scanning, wait briefly for a valid RSSI reading.
        // Without this, RSSI_SC can complete its yaw sweep while the HTTP backend
        // is still discovering the mesh / first link sample, producing all-zero RSSI.
        if (!rssi_link_ready() && now < _rssi_wait_until_ms) {
            apply_yaw_pitch_pwm(1500, pitch_trim);
            tracker.nav_status.bearing = cur_yaw;
            tracker.nav_status.pitch = AP::ahrs().get_pitch_deg();
            _prev_yaw_deg = cur_yaw;
            _yaw_nomotion_start_ms = 0;
            if (_rssi_wait_last_msg_ms == 0 || (now - _rssi_wait_last_msg_ms) > 2000) {
                _rssi_wait_last_msg_ms = now;
                gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: waiting for RSSI link");
            }
            break;
        }

        // Stay neutral until settle completes; this avoids many physical turns during EKF startup.
        if (now < _yaw_settle_until_ms) {
            apply_yaw_pitch_pwm(1500, pitch_trim);
            tracker.nav_status.bearing = cur_yaw;
            tracker.nav_status.pitch = AP::ahrs().get_pitch_deg();
            _prev_yaw_deg = cur_yaw;
            _yaw_nomotion_start_ms = 0;
            break;
        }

        if (!_yaw_drive_started) {
            _yaw_drive_started = true;
            _prev_yaw_deg = cur_yaw;
            _yaw_nomotion_start_ms = 0;
            _yaw_progress_sign = 0;
            _last_progress_ms = now;
            gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SC: Yaw drive start pwm=%u", unsigned(yaw_pwm));
        }

        apply_yaw_pitch_pwm(yaw_pwm, pitch_trim);
        // NAV_CONTROLLER_OUTPUT / logs: actual AHRS aim (rssi_sim and GCS use this; yaw not from PID here)
        tracker.nav_status.bearing = cur_yaw;
        tracker.nav_status.pitch = AP::ahrs().get_pitch_deg();
        _prev_yaw_deg = cur_yaw;

        // Robust CW integration:
        // - only integrate once we've passed the initial settle window
        // - only integrate if we're actually commanding motion (PWM away from 1500)
        // - only integrate in the commanded direction (CW is PWM < 1500)
        const int16_t pwm_delta = int16_t(yaw_pwm) - 1500;
        const bool cmd_cw = (pwm_delta < -10);
        const bool cmd_ccw = (pwm_delta > 10);
        const bool cmd_moving = cmd_cw || cmd_ccw;

        // Determine which way AHRS yaw moves for the commanded rotation.
        // On some installations the reported yaw can move with opposite sign to the physical CW direction.
        if (_yaw_progress_sign == 0 && cmd_moving && fabsf(dpsi) > 0.05f) {
            _yaw_progress_sign = (dpsi > 0.0f) ? 1 : -1;
        }

        // If we are commanding rotation but AHRS yaw is not changing, stop early.
        // This prevents multiple physical revolutions when yaw feedback is unhealthy/misconfigured.
        if (cmd_moving) {
            const bool yaw_moving = (fabsf(dpsi) > 0.05f);
            if (!yaw_moving) {
                if (_yaw_nomotion_start_ms == 0) {
                    _yaw_nomotion_start_ms = now;
                } else if (now - _yaw_nomotion_start_ms > 5000) {
                    // hard stop continuous-rotation servo at neutral PWM
                    SRV_Channels::set_output_pwm(SRV_Channel::k_tracker_yaw, 1500);
                    SRV_Channels::constrain_pwm(SRV_Channel::k_tracker_yaw);
                    gcs().send_text(MAV_SEVERITY_WARNING,
                                    "RSSI_SC: yaw not moving (dpsi=%.2f), stopping scan",
                                    (double)dpsi);
                    // Abort yaw phase without entering YAW_POINT. If we enter YAW_POINT, update_auto()
                    // can keep driving yaw for up to POINT_TIMEOUT_MS and cause extra revolutions.
                    // Keep current yaw as best estimate and continue directly to pitch scan.
                    _best_yaw_deg = cur_yaw;
                    enter_pitch_drive();
                    break;
                }
            } else {
                _yaw_nomotion_start_ms = 0;
            }
        } else {
            _yaw_nomotion_start_ms = 0;
        }

        if (cmd_moving) {
            // Integrate yaw progress using the observed sign, so a single turn completes even if
            // reported yaw moves "backwards" for this installation.
            if (_yaw_progress_sign > 0 && dpsi > 0.0f) {
                _yaw_cumulative_cw += MIN(dpsi, MAX_YAW_DEG_PER_TICK);
            } else if (_yaw_progress_sign < 0 && dpsi < 0.0f) {
                _yaw_cumulative_cw += MIN(-dpsi, MAX_YAW_DEG_PER_TICK);
            }
        }

        // Rate-limited debug to diagnose arc progression issues on real hardware.
        // Prints both the raw yaw delta (dpsi) and the integrated CW accumulator.
        if (now - _debug_last_ms >= 1000) {
            _debug_last_ms = now;
            const float next_thr = float(_yaw_next_k * _yaw_step_deg);
            const uint32_t age_ms = now - _last_progress_ms;
            const Vector3f gyro = AP::ahrs().get_gyro(); // rad/s body frame
            const uint16_t yaw_pwm_dbg = yaw_pwm;
            gcs().send_text(MAV_SEVERITY_INFO,
                            "RSSI_SC: Ydbg pwm=%u yaw=%.1f dpsi=%.1f cw=%.1f/%0.0f next=%.0f age=%lu gxyz=%.3f/%.3f/%.3f",
                            unsigned(yaw_pwm_dbg),
                            (double)cur_yaw,
                            (double)dpsi,
                            (double)_yaw_cumulative_cw,
                            (double)_yaw_arc_deg,
                            (double)next_thr,
                            (unsigned long)age_ms,
                            (double)gyro.x,
                            (double)gyro.y,
                            (double)gyro.z);
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
        const uint16_t yaw_neutral = 1500;
        const uint16_t pitch_pwm = (uint16_t)constrain_int32(tracker.g.rssi_scp_pwm.get(), 800, 2200);
        apply_yaw_pitch_pwm(yaw_neutral, pitch_pwm);
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
