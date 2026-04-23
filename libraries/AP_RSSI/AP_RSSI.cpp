/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RSSI_config.h"

#if AP_RSSI_ENABLED

#include <AP_RSSI/AP_RSSI.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

#include <utility>

#if AP_RSSI_UDP_ENABLED || AP_RSSI_HTTP_ENABLED
#include <AP_HAL/utility/Socket.h>
#include <AP_Networking/AP_Networking.h>
#include <string.h>
#include <stdlib.h>
#endif

extern const AP_HAL::HAL& hal;

#ifndef BOARD_RSSI_DEFAULT
#define BOARD_RSSI_DEFAULT 0
#endif

#ifndef BOARD_RSSI_ANA_PIN
#define BOARD_RSSI_ANA_PIN -1
#endif

#ifndef BOARD_RSSI_ANA_PIN_HIGH
#define BOARD_RSSI_ANA_PIN_HIGH 5.0f
#endif

const AP_Param::GroupInfo AP_RSSI::var_info[] = {

    // @Param: TYPE
    // @DisplayName: RSSI Type
    // @Description: Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
    // @Values: 0:Disabled,1:AnalogPin,2:RCChannelPwmValue,3:ReceiverProtocol,4:PWMInputPin,5:TelemetryRadioRSSI,6:UDPEthernet,7:Solo8HTTP
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 0, AP_RSSI, rssi_type,  BOARD_RSSI_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: ANA_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: Pin used to read the RSSI voltage or PWM value. Analog Airspeed ports can be used for Analog inputs (some autopilots provide others also), Non-IOMCU Servo/MotorOutputs can be used for PWM input when configured as "GPIOs". Values for some autopilots are given as examples. Search wiki for "Analog pins" for analog pin or "GPIOs", if PWM input type, to determine pin number.
    // @Values: 8:V5 Nano,11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @Range: -1 127
    // @User: Standard
    AP_GROUPINFO("ANA_PIN", 1, AP_RSSI, rssi_analog_pin,  BOARD_RSSI_ANA_PIN),

    // @Param: PIN_LOW
    // @DisplayName: RSSI pin's lowest voltage
    // @Description: RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the weakest. Some radio receivers put out inverted values so this value may be higher than RSSI_PIN_HIGH. When using pin 103, the maximum value of the parameter is 3.3V.
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIN_LOW", 2, AP_RSSI, rssi_analog_pin_range_low, 0.0f),

    // @Param: PIN_HIGH
    // @DisplayName: RSSI pin's highest voltage
    // @Description: RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the strongest. Some radio receivers put out inverted values so this value may be lower than RSSI_PIN_LOW. When using pin 103, the maximum value of the parameter is 3.3V.
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIN_HIGH", 3, AP_RSSI, rssi_analog_pin_range_high, BOARD_RSSI_ANA_PIN_HIGH),

    // @Param: CHANNEL
    // @DisplayName: Receiver RSSI channel number
    // @Description: The channel number where RSSI will be output by the radio receiver (5 and above).
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("CHANNEL", 4, AP_RSSI, rssi_channel,  0),

    // @Param: CHAN_LOW
    // @DisplayName: RSSI PWM low value
    // @Description: PWM value that the radio receiver will put on the RSSI_CHANNEL or RSSI_ANA_PIN when the signal strength is the weakest. Some radio receivers output inverted values so this value may be lower than RSSI_CHAN_HIGH
    // @Units: PWM
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("CHAN_LOW", 5, AP_RSSI, rssi_channel_low_pwm_value,  1000),

    // @Param: CHAN_HIGH
    // @DisplayName: Receiver RSSI PWM high value
    // @Description: PWM value that the radio receiver will put on the RSSI_CHANNEL or RSSI_ANA_PIN when the signal strength is the strongest. Some radio receivers output inverted values so this value may be higher than RSSI_CHAN_LOW
    // @Units: PWM
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("CHAN_HIGH", 6, AP_RSSI, rssi_channel_high_pwm_value,  2000),

#if AP_RSSI_UDP_ENABLED
    // @Param: UDP_PORT
    // @DisplayName: RSSI UDP listen port
    // @Description: UDP port on which to listen for binary RSSI datagrams (e.g. from a Solo8 receiver on the local Ethernet). Used only when RSSI_TYPE=6. Set to 0 to disable.
    // @Range: 0 65535
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("UDP_PORT", 7, AP_RSSI, rssi_udp_port, 14660),

    // @Param: UDP_LOSS
    // @DisplayName: RSSI UDP link-loss timeout
    // @Description: Milliseconds without a valid UDP packet before RSSI is reported as 0. Used only when RSSI_TYPE=6.
    // @Range: 100 10000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("UDP_LOSS", 8, AP_RSSI, rssi_udp_loss_ms, 1000),

    // @Param: UDP_DBM_LO
    // @DisplayName: dBm mapped to 0% RSSI
    // @Description: Signal level (dBm) that is treated as weakest (0.0 / 0%). Used only when RSSI_TYPE=6.
    // @Range: -150 0
    // @Units: dBm
    // @User: Advanced
    AP_GROUPINFO("UDP_DBM_LO", 9, AP_RSSI, rssi_udp_dbm_low, -90.0f),

    // @Param: UDP_DBM_HI
    // @DisplayName: dBm mapped to 100% RSSI
    // @Description: Signal level (dBm) that is treated as strongest (1.0 / 100%). Used only when RSSI_TYPE=6.
    // @Range: -120 20
    // @Units: dBm
    // @User: Advanced
    AP_GROUPINFO("UDP_DBM_HI", 10, AP_RSSI, rssi_udp_dbm_high, -30.0f),
#endif  // AP_RSSI_UDP_ENABLED

#if AP_RSSI_HTTP_ENABLED
    // @Group: HTTP_IP
    // @Path: ../AP_Networking/AP_Networking_address.cpp
    // @RebootRequired: True
    AP_SUBGROUPINFO(rssi_http_ip, "HTTP_IP", 11, AP_RSSI, AP_Networking_IPV4),

    // @Param: HTTP_PORT
    // @DisplayName: Solo8 HTTP port
    // @Description: TCP port for the Solo8 HTTP server. Used only when RSSI_TYPE=7.
    // @Range: 1 65535
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("HTTP_PORT", 12, AP_RSSI, rssi_http_port, 80),

    // @Param: HTTP_RATE
    // @DisplayName: Solo8 HTTP poll rate
    // @Description: HTTP GET rate for /localrfstatus.json, in Hz. Used only when RSSI_TYPE=7. Solo8 updates that endpoint faster than 5 Hz so 5-10 Hz is a reasonable range.
    // @Range: 1 20
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("HTTP_RATE", 13, AP_RSSI, rssi_http_rate_hz, 5),

    // @Param: HTTP_LOSS
    // @DisplayName: Solo8 HTTP link-loss timeout
    // @Description: Milliseconds without a successful HTTP poll before RSSI is reported as 0. Used only when RSSI_TYPE=7.
    // @Range: 200 10000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("HTTP_LOSS", 14, AP_RSSI, rssi_http_loss_ms, 1500),

    // @Param: HTTP_DBM_LO
    // @DisplayName: dBm mapped to 0% RSSI (Solo8)
    // @Description: Signal level (dBm) that is treated as weakest (0.0 / 0%). Used only when RSSI_TYPE=7.
    // @Range: -150 0
    // @Units: dBm
    // @User: Advanced
    AP_GROUPINFO("HTTP_DBM_LO", 15, AP_RSSI, rssi_http_dbm_low, -90.0f),

    // @Param: HTTP_DBM_HI
    // @DisplayName: dBm mapped to 100% RSSI (Solo8)
    // @Description: Signal level (dBm) that is treated as strongest (1.0 / 100%). Used only when RSSI_TYPE=7.
    // @Range: -120 20
    // @Units: dBm
    // @User: Advanced
    AP_GROUPINFO("HTTP_DBM_HI", 16, AP_RSSI, rssi_http_dbm_high, -30.0f),
#endif  // AP_RSSI_HTTP_ENABLED

    AP_GROUPEND
};

// Public
// ------

// constructor
AP_RSSI::AP_RSSI()
{       
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton) {
        AP_HAL::panic("Too many RSSI sensors");
    }
    _singleton = this;
}

// destructor
AP_RSSI::~AP_RSSI(void)
{       
}

/*
 * Get the AP_RSSI singleton
 */
AP_RSSI *AP_RSSI::get_singleton()
{
    return _singleton;
}

// Initialize the rssi object and prepare it for use
void AP_RSSI::init()
{
    // a pin for reading the receiver RSSI voltage. The scaling by 0.25 
    // is to take the 0 to 1024 range down to an 8 bit range for MAVLink    
    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);

#if AP_RSSI_UDP_ENABLED
    if (RssiType(rssi_type.get()) == RssiType::UDP_ETHERNET) {
        udp_init();
    }
#endif
#if AP_RSSI_HTTP_ENABLED
    if (RssiType(rssi_type.get()) == RssiType::SOLO8_HTTP) {
        http_init();
    }
#endif
}

// Read the receiver RSSI value as a float 0.0f - 1.0f.
// 0.0 represents weakest signal, 1.0 represents maximum signal.
float AP_RSSI::read_receiver_rssi()
{
    switch (RssiType(rssi_type.get())) {
        case RssiType::TYPE_DISABLED:
            return 0.0f;
        case RssiType::ANALOG_PIN:
            return read_pin_rssi();
        case RssiType::RC_CHANNEL_VALUE:
            return read_channel_rssi();
        case RssiType::RECEIVER: {
            int16_t rssi = RC_Channels::get_receiver_rssi();
            if (rssi != -1) {
                return rssi * (1/255.0);
            }
            return 0.0f;
        }
        case RssiType::PWM_PIN:
            return read_pwm_pin_rssi();
        case RssiType::TELEMETRY_RADIO_RSSI:
            return read_telemetry_radio_rssi();
#if AP_RSSI_UDP_ENABLED
        case RssiType::UDP_ETHERNET:
            return read_udp_rssi();
#endif
#if AP_RSSI_HTTP_ENABLED
        case RssiType::SOLO8_HTTP:
            return read_http_rssi();
#endif
    }
    // should never get to here
    return 0.0f;
}

// Only valid for RECEIVER type RSSI selections. Returns -1 if protocol does not provide link quality report.
float AP_RSSI::read_receiver_link_quality()
{
    if (RssiType(rssi_type.get()) == RssiType::RECEIVER) {
        return RC_Channels::get_receiver_link_quality();
    }
    return -1;
}

// Read the receiver RSSI value as an 8-bit integer
// 0 represents weakest signal, 255 represents maximum signal.
uint8_t AP_RSSI::read_receiver_rssi_uint8()
{
    return read_receiver_rssi() * 255; 
}

// Private
// -------

// read the RSSI value from an analog pin - returns float in range 0.0 to 1.0
float AP_RSSI::read_pin_rssi()
{
    if (!rssi_analog_source || !rssi_analog_source->set_pin(rssi_analog_pin)) {
        return 0;
    }
    float current_analog_voltage = rssi_analog_source->voltage_average();

    return scale_and_constrain_float_rssi(current_analog_voltage, rssi_analog_pin_range_low, rssi_analog_pin_range_high);
}

// read the RSSI value from a PWM value on a RC channel
float AP_RSSI::read_channel_rssi()
{
    RC_Channel *c = rc().channel(rssi_channel-1);
    if (c == nullptr) {
        return 0.0f;
    }
    uint16_t rssi_channel_value = c->get_radio_in();
    float channel_rssi = scale_and_constrain_float_rssi(rssi_channel_value, rssi_channel_low_pwm_value, rssi_channel_high_pwm_value);
    return channel_rssi;    
}

// read the PWM value from a pin
float AP_RSSI::read_pwm_pin_rssi()
{
    // check if pin has changed and configure interrupt handlers if required:
    if (!pwm_state.pwm_source.set_pin(rssi_analog_pin, "RSSI")) {
        // disabled (either by configuration or failure to attach interrupt)
        return 0.0f;
    }

    uint16_t pwm_us = pwm_state.pwm_source.get_pwm_us();

    const uint32_t now = AP_HAL::millis();
    if (pwm_us == 0) {
        // no reading; check for timeout:
        if (now - pwm_state.last_reading_ms > 1000) {
            // no reading for a second - something is broken
            pwm_state.rssi_value = 0.0f;
        }
    } else {
        // a new reading - convert pwm value to rssi value
        pwm_state.rssi_value = scale_and_constrain_float_rssi(pwm_us, rssi_channel_low_pwm_value, rssi_channel_high_pwm_value);
        pwm_state.last_reading_ms = now;
    }

    return pwm_state.rssi_value;
}

float AP_RSSI::read_telemetry_radio_rssi()
{
#if HAL_GCS_ENABLED
    return GCS_MAVLINK::telemetry_radio_rssi();
#else
    return 0;
#endif
}

// Scale and constrain a float rssi value to 0.0 to 1.0 range 
float AP_RSSI::scale_and_constrain_float_rssi(float current_rssi_value, float low_rssi_range, float high_rssi_range)
{    
    float rssi_value_range = fabsf(high_rssi_range - low_rssi_range);
    if (is_zero(rssi_value_range)) {
        // User range isn't meaningful, return 0 for RSSI (and avoid divide by zero)
        return 0.0f;   
    }
    // Note that user-supplied ranges may be inverted and we accommodate that here. 
    // (Some radio receivers put out inverted ranges for RSSI-type values).    
    bool range_is_inverted = (high_rssi_range < low_rssi_range);
    // Constrain to the possible range - values outside are clipped to ends 
    current_rssi_value = constrain_float(current_rssi_value, 
                                        range_is_inverted ? high_rssi_range : low_rssi_range, 
                                        range_is_inverted ? low_rssi_range : high_rssi_range);    

    if (range_is_inverted)
    {
        // Swap values so we can treat them as low->high uniformly in the code that follows
        current_rssi_value = high_rssi_range + fabsf(current_rssi_value - low_rssi_range);
        std::swap(low_rssi_range, high_rssi_range);        
    }

    // Scale the value down to a 0.0 - 1.0 range
    float rssi_value_scaled = (current_rssi_value - low_rssi_range) / rssi_value_range;
    // Make absolutely sure the value is clipped to the 0.0 - 1.0 range. This should handle things if the
    // value retrieved falls outside the user-supplied range.
    return constrain_float(rssi_value_scaled, 0.0f, 1.0f);
}

AP_RSSI *AP_RSSI::_singleton = nullptr;

namespace AP {

AP_RSSI *rssi()
{
    return AP_RSSI::get_singleton();
}

};

#if AP_RSSI_UDP_ENABLED
/*
  UDP Ethernet RSSI backend.

  A dedicated worker thread owns a non-blocking datagram socket bound to
  RSSI_UDP_PORT. Incoming datagrams are parsed by parse_udp_packet(); the
  resulting scaled 0..1 value is stored under a semaphore and read by
  read_udp_rssi() from the vehicle main loop. A packet is considered
  fresh for RSSI_UDP_LOSS milliseconds.

  Expected wire format (24 bytes, little-endian) from the Solo8:

    offset  size  field
       0     4    magic   = "SRSI"
       4     1    version = 1
       5     1    valid   (0/1, == Solo8 sigValid)
       6     2    reserved (0)
       8     4    sig_a_dbm  (IEEE-754 float32, == Solo8 sigLevA0)
      12     4    sig_b_dbm  (IEEE-754 float32, == Solo8 sigLevB0)
      16     4    seq        (incrementing counter)
      20     4    padding    (ignored)

  The parser is deliberately kept in one function so it can be replaced
  if the Solo8 vendor packet layout differs.
*/

void AP_RSSI::udp_init()
{
    if (udp_state.thread_started) {
        return;
    }
    // keep a stable reference to the socket pointer for the thread
    udp_state.sock = nullptr;
    udp_state.rssi_value = 0.0f;
    udp_state.last_dbm = 0.0f;
    udp_state.last_reading_ms = 0;
    udp_state.packet_count = 0;
    udp_state.parse_errors = 0;
    udp_state.bound = false;

    if (rssi_udp_port.get() <= 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "RSSI_UDP: disabled (UDP_PORT=0)");
        return;
    }

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_RSSI::udp_thread, void),
            "RSSI_UDP", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RSSI_UDP: thread create failed");
        return;
    }
    udp_state.thread_started = true;
}

void AP_RSSI::udp_thread()
{
    // wait until the networking stack has an IP address
    AP::network().startup_wait();

    udp_state.sock = NEW_NOTHROW SocketAPM(true /*datagram*/);
    if (udp_state.sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RSSI_UDP: socket alloc failed");
        return;
    }
    udp_state.sock->set_blocking(false);
    udp_state.sock->reuseaddress();

    const uint16_t bind_port = uint16_t(rssi_udp_port.get());
    if (!udp_state.sock->bind("0.0.0.0", bind_port)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RSSI_UDP: bind :%u failed", unsigned(bind_port));
        delete udp_state.sock;
        udp_state.sock = nullptr;
        return;
    }
    udp_state.bound = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RSSI_UDP: listening on :%u", unsigned(bind_port));

    uint8_t buf[128];
    while (true) {
        // 50ms timeout keeps the thread responsive and lets packet_loss
        // be detected by read_udp_rssi() ageing logic.
        const ssize_t n = udp_state.sock->recv(buf, sizeof(buf), 50);
        if (n <= 0) {
            continue;
        }
        float dbm = 0.0f;
        bool  valid = false;
        if (!parse_udp_packet(buf, uint16_t(n), dbm, valid)) {
            udp_state.parse_errors++;
            continue;
        }
        if (!valid) {
            continue;
        }

        // Scale dBm -> 0..1 using the user-configured window.
        const float lo = rssi_udp_dbm_low.get();
        const float hi = rssi_udp_dbm_high.get();
        const float scaled = scale_and_constrain_float_rssi(dbm, lo, hi);

        WITH_SEMAPHORE(udp_state.sem);
        udp_state.last_dbm = dbm;
        udp_state.rssi_value = scaled;
        udp_state.last_reading_ms = AP_HAL::millis();
        udp_state.packet_count++;
    }
}

bool AP_RSSI::parse_udp_packet(const uint8_t *buf, uint16_t len,
                               float &out_dbm, bool &out_valid) const
{
    // minimum useful size: magic+version+valid+reserved + 2 floats
    if (len < 16) {
        return false;
    }
    if (buf[0] != 'S' || buf[1] != 'R' || buf[2] != 'S' || buf[3] != 'I') {
        return false;
    }
    if (buf[4] != 1) {
        return false;
    }
    out_valid = (buf[5] != 0);
    float a = 0.0f;
    float b = 0.0f;
    memcpy(&a, buf + 8,  sizeof(float));
    memcpy(&b, buf + 12, sizeof(float));
    // Use the strongest of the two antennas, matching the sidecar
    // behaviour in Tools/rssi_bridge.
    out_dbm = (a > b) ? a : b;
    return true;
}

float AP_RSSI::read_udp_rssi()
{
    WITH_SEMAPHORE(udp_state.sem);
    if (udp_state.last_reading_ms == 0) {
        return 0.0f;
    }
    const uint32_t age = AP_HAL::millis() - udp_state.last_reading_ms;
    const int16_t loss_param = rssi_udp_loss_ms.get();
    const uint32_t loss = uint32_t(loss_param < 100 ? 100 : loss_param);
    if (age > loss) {
        return 0.0f;
    }
    return udp_state.rssi_value;
}
#endif  // AP_RSSI_UDP_ENABLED

#if AP_RSSI_HTTP_ENABLED
/*
  Solo8 HTTP/JSON RSSI backend.

  Solo8 (Silvus IP Mesh Radio) only exposes RF status over HTTP polling
  (confirmed against the vendor's JSON Integration Guide -- no UDP push
  or subscription mechanism is available). This backend issues periodic
  HTTP GETs of /localrfstatus.json on a dedicated worker thread and
  extracts the three fields the existing Tools/rssi_bridge sidecar uses:
  "sigLevA0", "sigLevB0" (dBm, per antenna) and "sigValid" (bool).

  The strongest of the two channels is scaled by HTTP_DBM_LO/HI to the
  0..1 range consumed by the rest of the autopilot (MAVLink, Logger,
  ModeRSSIScan, OSD, ...).

  HTTP Basic auth uses the Solo8 factory credentials baked into the
  sidecar: user="", pass="Eastwood". base64(":Eastwood") is
  "OkVhc3R3b29k" -- hard-coded below. If the field unit has been
  rekeyed, edit SOLO8_HTTP_AUTH_B64 before flashing.
*/

#ifndef SOLO8_HTTP_PATH
#define SOLO8_HTTP_PATH "/localrfstatus.json"
#endif

#ifndef SOLO8_HTTP_AUTH_B64
#define SOLO8_HTTP_AUTH_B64 "OkVhc3R3b29k"   // base64(":Eastwood")
#endif

// The real localrfstatus.json body is ~1.8 KB (pretty-printed, 24-element
// arrays of per-subcarrier SNR/sigLev). Add ~300 bytes of HTTP headers,
// round up for growth across firmware revisions.
#define SOLO8_HTTP_RESP_BUF 4096
#define SOLO8_HTTP_CONNECT_TIMEOUT_MS 500
#define SOLO8_HTTP_RECV_TIMEOUT_MS    400

void AP_RSSI::http_init()
{
    if (http_state.thread_started) {
        return;
    }
    http_state.sock = nullptr;
    http_state.rssi_value = 0.0f;
    http_state.last_dbm = 0.0f;
    http_state.last_reading_ms = 0;
    http_state.poll_count = 0;
    http_state.poll_errors = 0;

    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_RSSI::http_thread, void),
            "RSSI_HTTP", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RSSI_HTTP: thread create failed");
        return;
    }
    http_state.thread_started = true;
}

void AP_RSSI::http_thread()
{
    AP::network().startup_wait();

    // response buffer re-used across polls; sized for a full
    // /localrfstatus.json payload plus HTTP headers.
    char *resp = (char*)calloc(SOLO8_HTTP_RESP_BUF, 1);
    if (resp == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "RSSI_HTTP: alloc failed");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RSSI_HTTP: polling %s:%u%s",
                  rssi_http_ip.get_str(),
                  unsigned(rssi_http_port.get()),
                  SOLO8_HTTP_PATH);

    while (true) {
        const int8_t hz_param = rssi_http_rate_hz.get();
        const uint32_t hz = uint32_t(hz_param < 1 ? 1 : hz_param);
        const uint32_t period_ms = 1000 / hz;
        const uint32_t t0 = AP_HAL::millis();

        uint16_t resp_len = 0;
        const bool ok = http_poll_once(resp, SOLO8_HTTP_RESP_BUF, resp_len);
        if (!ok) {
            http_state.poll_errors++;
        } else {
            // locate the start of the body (after "\r\n\r\n"); if not
            // found, treat the whole buffer as body.
            const char *body = resp;
            uint16_t body_len = resp_len;
            const char *split = strstr(resp, "\r\n\r\n");
            if (split != nullptr) {
                body = split + 4;
                body_len = uint16_t(resp_len - (body - resp));
            }

            float dbm = 0.0f;
            bool  valid = false;
            if (parse_solo8_json(body, body_len, dbm, valid) && valid) {
                const float lo = rssi_http_dbm_low.get();
                const float hi = rssi_http_dbm_high.get();
                const float scaled = scale_and_constrain_float_rssi(dbm, lo, hi);

                WITH_SEMAPHORE(http_state.sem);
                http_state.last_dbm = dbm;
                http_state.rssi_value = scaled;
                http_state.last_reading_ms = AP_HAL::millis();
                http_state.poll_count++;
            }
        }

        const uint32_t elapsed = AP_HAL::millis() - t0;
        if (elapsed < period_ms) {
            hal.scheduler->delay(period_ms - elapsed);
        }
    }
}

/*
  Run one HTTP GET against the Solo8. Returns true on successful
  receipt of *some* response body; parse errors are reported separately
  by parse_solo8_json.
*/
bool AP_RSSI::http_poll_once(char *resp_buf, uint16_t resp_buf_len,
                             uint16_t &resp_len)
{
    resp_len = 0;

    // open a new TCP socket each poll. Cheap on lwIP and keeps the
    // worker simple; the Solo8's embedded web server does not document
    // persistent connection behaviour, so we Connection: close.
    SocketAPM *sock = NEW_NOTHROW SocketAPM(false /*stream*/);
    if (sock == nullptr) {
        return false;
    }

    const char *dest = rssi_http_ip.get_str();
    const uint16_t dport = uint16_t(rssi_http_port.get());
    if (!sock->connect_timeout(dest, dport, SOLO8_HTTP_CONNECT_TIMEOUT_MS)) {
        delete sock;
        return false;
    }
    sock->set_blocking(false);

    char req[256];
    const int req_len = hal.util->snprintf(
        req, sizeof(req),
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Authorization: Basic %s\r\n"
        "User-Agent: ArduPilot\r\n"
        "Accept: application/json\r\n"
        "Connection: close\r\n\r\n",
        SOLO8_HTTP_PATH, dest, SOLO8_HTTP_AUTH_B64);
    if (req_len <= 0) {
        delete sock;
        return false;
    }
    // Loop the send in case lwIP returns a short write on a non-blocking
    // socket. The request is only ~150 bytes so this rarely iterates.
    int sent = 0;
    const uint32_t t_send = AP_HAL::millis();
    while (sent < req_len) {
        if (AP_HAL::millis() - t_send > SOLO8_HTTP_CONNECT_TIMEOUT_MS) {
            delete sock;
            return false;
        }
        const ssize_t n = sock->send(req + sent, size_t(req_len - sent));
        if (n > 0) {
            sent += int(n);
        } else if (!sock->pollout(50)) {
            delete sock;
            return false;
        }
    }

    // drain the response until the server closes the socket or we fill
    // the buffer. recv() with a timeout blocks via pollin() internally.
    uint16_t used = 0;
    const uint32_t t_start = AP_HAL::millis();
    while (used + 1 < resp_buf_len) {
        if (AP_HAL::millis() - t_start > SOLO8_HTTP_RECV_TIMEOUT_MS) {
            break;
        }
        const ssize_t n = sock->recv(resp_buf + used,
                                     resp_buf_len - 1 - used,
                                     50 /*ms*/);
        if (n < 0) {
            // EWOULDBLOCK from pollin() timeout -- keep waiting until
            // overall SOLO8_HTTP_RECV_TIMEOUT_MS budget runs out
            continue;
        }
        if (n == 0) {
            // peer closed -- response complete
            break;
        }
        used = uint16_t(used + n);
    }
    resp_buf[used] = '\0';
    resp_len = used;

    delete sock;
    return used > 0;
}

/*
  Extract sigLevA0, sigLevB0, sigValid from the Solo8 JSON body.

  Anchored on the "LocalDemodStatus" key so we ignore the identically
  named sibling under InterferenceAltStatus (where sigLevA0/B0 are per-
  frequency arrays, not scalars) and any future nested objects the
  vendor may add. This is a lightweight needle-match lexer -- NOT a
  general JSON parser -- which is sufficient because the Solo8 schema
  below "LocalDemodStatus" is stable per the vendor's JSON Integration
  Guide.

  Example excerpt of localrfstatus.json the parser operates on:

      "LocalDemodStatus" : {
        "nChan"    : 2,
        "sigValid" : true,
        ...
        "sigLevA0" : -102,
        "sigLevB0" : -96
      },
      "InterferenceAltStatus" : {
        ...
        "sigLevA0" : [ -120, -120, -101, ... ],   <-- must NOT read this
        "sigLevB0" : [ -120, -120,  -96, ... ]
      }
*/
bool AP_RSSI::parse_solo8_json(const char *body, uint16_t len,
                               float &out_dbm, bool &out_valid) const
{
    if (body == nullptr || len < 32) {
        return false;
    }

    // narrow the search window to the LocalDemodStatus object
    const char *scope = strstr(body, "\"LocalDemodStatus\"");
    if (scope == nullptr) {
        return false;
    }

    auto find_scalar_number = [&](const char *key, float &out) -> bool {
        const char *p = strstr(scope, key);
        if (p == nullptr) {
            return false;
        }
        p = strchr(p + strlen(key), ':');
        if (p == nullptr) {
            return false;
        }
        ++p;
        while (*p == ' ' || *p == '\t') {
            ++p;
        }
        // reject arrays -- the Solo8 uses the same field names inside
        // InterferenceAltStatus but as "[ -120, ..., -120 ]" arrays.
        if (*p == '[') {
            return false;
        }
        char *end = nullptr;
        const float v = strtof(p, &end);
        if (end == p) {
            return false;
        }
        out = v;
        return true;
    };

    float a = 0.0f;
    float b = 0.0f;
    if (!find_scalar_number("\"sigLevA0\"", a)) {
        return false;
    }
    if (!find_scalar_number("\"sigLevB0\"", b)) {
        return false;
    }

    // sigValid is a boolean literal inside LocalDemodStatus.
    out_valid = false;
    const char *pv = strstr(scope, "\"sigValid\"");
    if (pv != nullptr) {
        pv = strchr(pv + sizeof("\"sigValid\"") - 1, ':');
        if (pv != nullptr) {
            ++pv;
            while (*pv == ' ' || *pv == '\t') {
                ++pv;
            }
            out_valid = (strncmp(pv, "true", 4) == 0);
        }
    }

    out_dbm = (a > b) ? a : b;
    return true;
}

float AP_RSSI::read_http_rssi()
{
    WITH_SEMAPHORE(http_state.sem);
    if (http_state.last_reading_ms == 0) {
        return 0.0f;
    }
    const uint32_t age = AP_HAL::millis() - http_state.last_reading_ms;
    const int16_t loss_param = rssi_http_loss_ms.get();
    const uint32_t loss = uint32_t(loss_param < 200 ? 200 : loss_param);
    if (age > loss) {
        return 0.0f;
    }
    return http_state.rssi_value;
}
#endif  // AP_RSSI_HTTP_ENABLED

#endif  // AP_RSSI_ENABLED
