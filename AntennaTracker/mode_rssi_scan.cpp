/*
 * AntennaTracker - RSSI Scan Mode
 *
 * Algorithm:
 *   1. Sweep pan -180→+180 at fixed mid-tilt, record RSSI at each step
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
    _pan_best   = 0.0f;
    _tilt_best  = 0.0f;
    _rssi_best  = 0.0f;

    // Start tilt at midpoint so pan scan has a fair signal
    //!!!NatiE _tilt_current = (tracker.g.ahrs_trim_y + 0.0f);  // mid tilt — adjust to your mount
    _pan_current  = -180.0f;

    gcs().send_text(MAV_SEVERITY_INFO, "RSSI_SCAN: Pan sweep starting");
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

    if (_pan_current > 180.0f) {
        // Pan sweep done
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
    _tilt_current  = tracker.aparm.angle_max_tilt * -0.01f;  // tilt min (degrees)

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
    float tilt_max = tracker.aparm.angle_max_tilt * 0.01f;

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
    float drop_threshold = _rssi_at_lock - (tracker.g.rssi_rescan_drop * 0.01f);
    if (current_rssi < drop_threshold) {
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
            // Clamp
            _pan_best  = constrain_float(_pan_best,  -180.0f, 180.0f);
            _tilt_best = constrain_float(_tilt_best,
                          tracker.aparm.angle_max_tilt * -0.01f,
                          tracker.aparm.angle_max_tilt *  0.01f);
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
        //!!!NatiE sum += AP::rssi()->get_rssi();   // returns 0.0–1.0
        hal.scheduler->delay(10);
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
    // AntennaTracker uses centidegrees internally
    tracker.nav_status.bearing       = pan_deg;
    tracker.nav_status.pitch         = tilt_deg;
    tracker.update_auto_armed();
    tracker.update_servos_from_nav_status();
}