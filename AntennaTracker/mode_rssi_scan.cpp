/*
 * Copyright (c) 2026 Colugo. All rights reserved.
 *
 * Proprietary work product of Colugo.
 * Created by Nati Elinson.
 */

/*
 * AntennaTracker - RSSI Scan Mode
 *
 * Algorithm:
 *   1. Sweep pan 0→360° at fixed mid-tilt (same convention as AHRS yaw), record RSSI at each step
 *   2. Move to best pan angle
 *   3. Sweep tilt min→max at best pan, record RSSI at each step
 *   4. Move to best pan+tilt → LOCKED
 *   5. Dither ± RSSI_DITHER degrees on each axis to maintain peak
 *   6. If RSSI drops > RSSI_RESCAN_DROP from lock value → re-scan
 *
 * RSSI input: PWM signal via AP_RSSI (RSSI_TYPE=2)
 */

#include "mode.h"
#include "Tracker.h"

#include <string.h>

static constexpr uint32_t RSSI_RESCAN_GRACE_MS = 3000;

void ModeRSSIScan::reset_for_entry()
{
    _initialized = false;
    _handoff_lock_from_sc = false;
    _lock_acquired_ms = 0;
}

bool ModeRSSIScan::consume_handoff_nav(float &bearing_deg, float &pitch_deg)
{
    if (!_handoff_lock_from_sc) {
        return false;
    }
    bearing_deg = _pan_best;
    pitch_deg = _tilt_best;
    _handoff_lock_from_sc = false;
    return true;
}

void ModeRSSIScan::import_lock_from_compass_scan(const ModeRSSIScanCompass &sc)
{
    const float pan = wrap_360(sc.compass_handoff_pan_deg());
    const float tilt = constrain_float(sc.compass_handoff_tilt_deg(),
                                        tracker.g.pitch_min,
                                        tracker.g.pitch_max);

    float rssi_lock = sc.compass_handoff_rssi_norm();
    if (rssi_lock < 0.0f) {
        rssi_lock = read_rssi_avg();
    }
    rssi_lock = constrain_float(rssi_lock, 0.0f, 1.0f);

    if (rssi_lock < tracker.g.rssi_lock_threshold * 0.01f) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                         "RSSI_SCAN: weak RSSI from RSSI_SC; full scan");
        reset_for_entry();
        return;
    }

    _pan_best = pan;
    _tilt_best = tilt;
    _pan_current = pan;
    _tilt_current = tilt;
    _rssi_best = rssi_lock;
    _rssi_at_lock = rssi_lock;
    _lock_acquired_ms = AP_HAL::millis();
    _initialized = true;
    _handoff_lock_from_sc = true;
    _dither_step = 0;
    memset(_dither_rssi, 0, sizeof(_dither_rssi));

    gcs().send_text(MAV_SEVERITY_INFO,
                     "RSSI_SCAN: locked from RSSI_SC pan=%.1f tilt=%.1f rssi=%.0f%%",
                     (double)pan, (double)tilt, (double)(rssi_lock * 100.0f));

    start_dither();
}

// ---------------------------------------------------------------
// Init
// ---------------------------------------------------------------
//bool ModeRSSIScan::init()
bool ModeRSSIScan::init_rssi_scan()
{
    //NatiE if (!AP::rssi().enabled()) {
    if(!AP::rssi()->enabled()) {
        gcs().send_text(MAV_SEVERITY_ERROR, "RSSI_SCAN: AP_RSSI not enabled. Set RSSI_TYPE=2");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SCAN: Initialising full scan");
    start_pan_scan();
    return true;
}

// ---------------------------------------------------------------
// Main update — called at scheduler rate (~10 Hz typical)
// ---------------------------------------------------------------
void ModeRSSIScan::update()
{
    if (!_initialized) {
        if (!init_rssi_scan()) {
            return;  // RSSI not available
        }
        _initialized = true;
    }

    switch (_state) {
    case ScanState::WAIT_SETTLE:   update_wait_settle();  break;
    case ScanState::SCAN_PAN:      update_scan_pan();     break;
    case ScanState::SCAN_TILT:     update_scan_tilt();    break;
    case ScanState::DITHER:        update_dither();       break;
    }
}

// ---------------------------------------------------------------
// Phase 1: Pan scan
// ---------------------------------------------------------------
void ModeRSSIScan::start_pan_scan()
{
    _handoff_lock_from_sc = false;
    _pan_best   = 0.0f;
    _tilt_best  = 0.0f;
    _rssi_best  = 0.0f;

    // Start at configured midpoint of mechanical tilt range.
    _tilt_current = (tracker.g.pitch_min + tracker.g.pitch_max) * 0.5f;
    _pan_current  = 0.0f;

    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SCAN: Pan sweep 0-360 deg starting");
    move_and_wait(_pan_current, _tilt_current, ScanState::SCAN_PAN);
}

void ModeRSSIScan::update_scan_pan()
{
    float rssi = read_rssi_avg();

    if (rssi > _rssi_best) {
        _rssi_best  = rssi;
        _pan_best   = _pan_current;
    }

    _pan_current += tracker.g.rssi_scan_pan_step;

    if (_pan_current > 360.0f) {
        // Pan sweep done (full circle in 0..360° convention)
        gcs().send_text(MAV_SEVERITY_INFO,
            "RSSI_SCAN: Pan done. Best %.1f deg  RSSI %.0f%%",
            (double)_pan_best, (double)(_rssi_best * 100.0f));
        start_tilt_scan();
        return;
    }

    move_and_wait(_pan_current, _tilt_current, ScanState::SCAN_PAN);
}

// ---------------------------------------------------------------
// Phase 2: Tilt scan at best pan
// ---------------------------------------------------------------
void ModeRSSIScan::start_tilt_scan()
{
    _rssi_best     = 0.0f;   // reset — we search tilt now
    _tilt_current  = tracker.g.pitch_min;

    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SCAN: Tilt sweep starting at pan=%.1f",
        (double)_pan_best);

    move_and_wait(_pan_best, _tilt_current, ScanState::SCAN_TILT);
}

void ModeRSSIScan::update_scan_tilt()
{
    float rssi = read_rssi_avg();

    if (rssi > _rssi_best) {
        _rssi_best  = rssi;
        _tilt_best  = _tilt_current;
    }

    _tilt_current += tracker.g.rssi_scan_tilt_step;
    const float tilt_max = tracker.g.pitch_max;

    if (_tilt_current > tilt_max) {
        // Tilt sweep done — move to peak and start dithering
        gcs().send_text(MAV_SEVERITY_INFO,
            "RSSI_SCAN: Locked! Pan=%.1f Tilt=%.1f RSSI=%.0f%%",
            (double)_pan_best, (double)_tilt_best,
            (double)(_rssi_best * 100.0f));

        if (_rssi_best < (tracker.g.rssi_lock_threshold * 0.01f)) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                "RSSI_SCAN: Signal too weak (%.0f%%), re-scanning",
                (double)(_rssi_best * 100.0f));
            start_pan_scan();
            return;
        }

        _rssi_at_lock = _rssi_best;
        _lock_acquired_ms = AP_HAL::millis();
        start_dither();
        return;
    }

    move_and_wait(_pan_best, _tilt_current, ScanState::SCAN_TILT);
}

// ---------------------------------------------------------------
// Phase 3: Dither to maintain peak
//   Cycle: try pan-, pan+, tilt-, tilt+
//   Move toward the best of the four samples
// ---------------------------------------------------------------
void ModeRSSIScan::start_dither()
{
    _dither_step = 0;
    memset(_dither_rssi, 0, sizeof(_dither_rssi));
    move_and_wait(_pan_best, _tilt_best, ScanState::DITHER);
}

void ModeRSSIScan::update_dither()
{
    float dither = tracker.g.rssi_dither_angle;

    // Check for signal loss → re-scan
    float current_rssi = read_rssi_avg();
    // After lock/handoff, allow a short grace period so transient RSSI settling
    // does not trigger immediate full re-scan.
    const uint32_t now = AP_HAL::millis();
    if ((now - _lock_acquired_ms) < RSSI_RESCAN_GRACE_MS) {
        // Keep lock anchor fresh during grace so threshold starts from actual held signal.
        _rssi_at_lock = MAX(_rssi_at_lock, current_rssi);
    }
    float drop_threshold = _rssi_at_lock - (tracker.g.rssi_rescan_drop * 0.01f);
    if ((now - _lock_acquired_ms) >= RSSI_RESCAN_GRACE_MS &&
        current_rssi < drop_threshold) {
        gcs().send_text(MAV_SEVERITY_WARNING,
            "RSSI_SCAN: Signal dropped (%.0f%%), re-scanning",
            (double)(current_rssi * 100.0f));
        start_pan_scan();
        return;
    }

    // Collect samples at 4 positions around current best
    switch (_dither_step) {
    case 0:
        _dither_rssi[0] = read_rssi_avg();   // pan-
        move_and_wait(_pan_best - dither, _tilt_best, ScanState::DITHER);
        _dither_step = 1;
        return;
    case 1:
        _dither_rssi[1] = read_rssi_avg();   // pan+
        move_and_wait(_pan_best + dither, _tilt_best, ScanState::DITHER);
        _dither_step = 2;
        return;
    case 2:
        _dither_rssi[2] = read_rssi_avg();   // tilt-
        move_and_wait(_pan_best, _tilt_best - dither, ScanState::DITHER);
        _dither_step = 3;
        return;
    case 3:
        _dither_rssi[3] = read_rssi_avg();   // tilt+
        move_and_wait(_pan_best, _tilt_best + dither, ScanState::DITHER);
        _dither_step = 4;
        return;
    case 4: {
        // Find best of the 4 probes
        float best = _dither_rssi[0];
        int   best_idx = 0;
        for (int i = 1; i < 4; i++) {
            if (_dither_rssi[i] > best) {
                best = _dither_rssi[i];
                best_idx = i;
            }
        }
        // Only move if improvement is meaningful (> 2%)
        if (best > current_rssi + 0.02f) {
            switch (best_idx) {
            case 0: _pan_best  -= dither; break;
            case 1: _pan_best  += dither; break;
            case 2: _tilt_best -= dither; break;
            case 3: _tilt_best += dither; break;
            }
            _pan_best = wrap_360(_pan_best);
            _tilt_best = constrain_float(_tilt_best,
                          tracker.g.pitch_min,
                          tracker.g.pitch_max);
        }
        // Return to best and restart dither cycle
        _dither_step = 0;
        move_and_wait(_pan_best, _tilt_best, ScanState::DITHER);
        return;
    }
    }
}

// ---------------------------------------------------------------
// Settle wait state — pause after servo move for vibration/lag
// ---------------------------------------------------------------
void ModeRSSIScan::update_wait_settle()
{
    uint32_t now = AP_HAL::millis();
    if (now - _settle_start_ms >= (uint32_t)tracker.g.rssi_scan_settle_ms) {
        _state = _state_after_settle;
    }
}

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------

// Average multiple RSSI readings to reduce noise
float ModeRSSIScan::read_rssi_avg()
{
    float sum = 0.0f;
    int   n   = MAX(1, (int)tracker.g.rssi_scan_samples);
    for (int i = 0; i < n; i++) {
        sum += AP::rssi()->read_receiver_rssi();   // returns 0.0–1.0
    }
    return sum / n;
}

// Move servos then transition to settle-wait state
void ModeRSSIScan::move_and_wait(float pan_deg, float tilt_deg, ScanState next_state)
{
    set_servos(pan_deg, tilt_deg);
    _settle_start_ms     = AP_HAL::millis();
    _state_after_settle  = next_state;
    _state               = ScanState::WAIT_SETTLE;
}

// Convert degrees to servo commands and apply
void ModeRSSIScan::set_servos(float pan_deg, float tilt_deg)
{
    // Match AHRS yaw (0..360°) and NAV_CONTROLLER_OUTPUT for consistent plotting
    tracker.nav_status.bearing = wrap_360(pan_deg);
    tracker.nav_status.pitch   = tilt_deg;
    update_auto();
}