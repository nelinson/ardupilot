#pragma once

#include <stdint.h>
#include <AP_Math/AP_Math.h>

class Mode {
public:
    enum class Number {
        MANUAL=0,
        STOP=1,
        SCAN=2,
        SERVOTEST=3,
        GUIDED=4,
        RSSI_SCAN=6,    /*NatiE*/
        RSSI_SCAN_C=7,  /* compass-gated RSSI scan */
        AUTO=10,
        INITIALISING=16
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    Mode() {}

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // returns a unique number specific to this mode
    virtual Mode::Number number() const = 0;
    virtual const char* name() const = 0;

    virtual bool requires_armed_servos() const = 0;

    virtual void update() = 0;

protected:
    void update_scan();
    void update_auto();

    bool get_ef_yaw_direction();

    void calc_angle_error(float pitch, float yaw, bool direction_reversed);
    void convert_ef_to_bf(float pitch, float yaw, float& bf_pitch, float& bf_yaw);
    bool convert_bf_to_ef(float pitch, float yaw, float& ef_pitch, float& ef_yaw);
};

class ModeAuto : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::AUTO; }
    const char* name() const override { return "Auto"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;

    void set_target(float target_yaw_deg, float target_pitch_deg) {
        _target_yaw_deg = target_yaw_deg;
        _target_pitch_deg = target_pitch_deg;
    }
    float get_auto_target_yaw_deg() const { return _target_yaw_deg; }
    float get_auto_target_pitch_deg() const { return _target_pitch_deg; }

private:
    float _target_yaw_deg;
    float _target_pitch_deg;
};

class ModeGuided : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::GUIDED; }
    const char* name() const override { return "Guided"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;

    void set_angle(const Quaternion &target_att,
                   bool use_yaw_rate, float yaw_rate_rads,
                   bool use_pitch_rate, float pitch_rate_rads) {
        _target_att = target_att;
        _use_yaw_rate = use_yaw_rate;
        _yaw_rate_rads = yaw_rate_rads;
        _use_pitch_rate = use_pitch_rate;
        _pitch_rate_rads = pitch_rate_rads;
    }
    Quaternion get_attitude_target_quat() { return _target_att; }
    bool get_attitude_target_use_yaw_rate() const { return _use_yaw_rate; }
    float get_attitude_target_yaw_rate_rads() const { return _yaw_rate_rads; }
    bool get_attitude_target_use_pitch_rate() const { return _use_pitch_rate; }
    float get_attitude_target_pitch_rate_rads() const { return _pitch_rate_rads; }

private:
    Quaternion _target_att;
    bool _use_yaw_rate;
    float _yaw_rate_rads;
    bool _use_pitch_rate;
    float _pitch_rate_rads;
};

class ModeInitialising : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::INITIALISING; }
    const char* name() const override { return "Initialising"; }
    bool requires_armed_servos() const override { return false; }
    void update() override {};
};

class ModeManual : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::MANUAL; }
    const char* name() const override { return "Manual"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

class ModeScan : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::SCAN; }
    const char* name() const override { return "Scan"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
};

class ModeServoTest : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::SERVOTEST; }
    const char* name() const override { return "ServoTest"; }
    bool requires_armed_servos() const override { return true; }
    void update() override {};

    bool set_servo(uint8_t servo_num, uint16_t pwm);
};

class ModeStop : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::STOP; }
    const char* name() const override { return "Stop"; }
    bool requires_armed_servos() const override { return false; }
    void update() override {};
};

class ModeRSSIScanCompass;

/*NatiE*/
class ModeRSSIScan : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::RSSI_SCAN; }
    const char *name() const override { return "RSSI_SCAN"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
    void reset_for_entry();
    void import_lock_from_compass_scan(const ModeRSSIScanCompass &sc);
    bool consume_handoff_nav(float &bearing_deg, float &pitch_deg);

private:
    bool _initialized {false};
    bool _handoff_lock_from_sc {false};
    enum class ScanState : uint8_t {
        SCAN_PAN,        // sweeping pan at fixed tilt
        SCAN_TILT,       // sweeping tilt at locked pan
        DITHER,          // maintaining lock with small movements
        WAIT_SETTLE,     // pausing after servo move
    };

    ScanState   _state          {ScanState::SCAN_PAN};
    ScanState   _state_after_settle;   // return state after settle wait

    // ---------- scan tracking ----------
    float   _pan_current        {0.0f};   // degrees
    float   _tilt_current       {0.0f};
    float   _pan_best           {0.0f};
    float   _tilt_best          {0.0f};
    float   _rssi_best          {0.0f};
    float   _rssi_at_lock       {0.0f};   // RSSI when we declared lock

    // ---------- dither ----------
    int8_t  _dither_step        {0};      // -1, 0, +1 for pan/tilt cycle
    float   _dither_rssi[4]     {};       // [pan-, pan+, tilt-, tilt+]

    // ---------- timing ----------
    uint32_t _settle_start_ms   {0};
    uint32_t _lock_acquired_ms  {0};

    // ---------- helpers ----------
    float   read_rssi_avg();
    void    move_and_wait(float pan_deg, float tilt_deg, ScanState next);
    void    set_servos(float pan_deg, float tilt_deg);
    void    start_pan_scan();
    void    start_tilt_scan();
    void    start_dither();

    void    update_scan_pan();
    void    update_scan_tilt();
    void    update_dither();
    void    update_wait_settle();
    bool    init_rssi_scan();
};

class ModeRSSIScanCompass : public Mode {
public:
    Mode::Number number() const override { return Mode::Number::RSSI_SCAN_C; }
    const char *name() const override { return "RSSI_SC"; }
    bool requires_armed_servos() const override { return true; }
    void update() override;
    void reset_for_entry();

    float compass_handoff_pan_deg() const {
        return _best_yaw_deg;
    }
    float compass_handoff_tilt_deg() const {
        return _best_pitch_deg;
    }
    float compass_handoff_rssi_norm() const {
        return MAX(_best_rssi_yaw, _best_rssi_pitch);
    }

private:
    enum class Phase : uint8_t {
        YAW_DRIVE = 0,
        YAW_POINT,
        PITCH_DRIVE,
        PITCH_POINT,
    };

    Phase _phase {Phase::YAW_DRIVE};

    float _yaw_start_deg {0.0f};
    float _pitch_start_deg {0.0f};
    float _yaw_arc_deg {360.0f};
    float _pitch_arc_deg {60.0f};
    int _yaw_step_deg {5};
    int _pitch_step_deg {10};

    int _yaw_next_k {0};
    int _yaw_max_k {0};
    float _best_yaw_deg {0.0f};
    float _best_pitch_deg {0.0f};
    float _best_rssi_yaw {0.0f};
    float _best_rssi_pitch {0.0f};

    int _pitch_next_k {0};
    int _pitch_max_k {0};

    uint32_t _last_progress_ms {0};
    uint32_t _debug_last_ms {0};
    uint32_t _point_enter_ms {0};
    uint16_t _point_stable {0};
    uint32_t _entry_ms {0};
    uint32_t _yaw_settle_until_ms {0};
    uint32_t _rssi_wait_until_ms {0};
    uint32_t _rssi_wait_last_msg_ms {0};
    uint32_t _yaw_nomotion_start_ms {0};
    bool _yaw_drive_started {false};
    int8_t _yaw_progress_sign {0}; // +1 or -1 once motion observed; 0 unknown

    bool _initialized {false};

    // Cumulative scan progress (integrated heading/pitch motion), not chord span
    // start→current, so AHRS alignment spikes do not instantly complete the arc.
    float _prev_yaw_deg {0.0f};
    float _yaw_cumulative_cw {0.0f};
    float _prev_pitch_deg {0.0f};
    float _pitch_cumulative_up {0.0f};

    static float wrap_360_deg(float d);
    float read_rssi_avg() const;
    void apply_yaw_pitch_pwm(uint16_t yaw_pwm, uint16_t pitch_pwm);
    void release_pwm_overrides();
    void enter_yaw_point();
    void enter_pitch_drive();
    void enter_pitch_point();
    void handoff_to_rssi_scan();
};
