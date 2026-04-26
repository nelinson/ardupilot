/*
 * Copyright (c) 2026 Colugo. All rights reserved.
 *
 * Proprietary work product of Colugo.
 * Created by Nati Elinson.
 */
#include "mavlink_sender.h"

#include "serial_port.h"

// MAVLink headers:
// - If using ArduPilot-generated headers: include path contains "mavlink/v2.0/<dialect>/mavlink.h"
// - If using mavlink/c_library_v2: include path contains "<dialect>/mavlink.h"
#if defined(RSSI_BRIDGE_MAVLINK_FLAT_LAYOUT)
#include <ardupilotmega/mavlink.h>
#else
#include <mavlink/v2.0/ardupilotmega/mavlink.h>
#endif

MavlinkSender::MavlinkSender(uint8_t system_id, uint8_t component_id)
    : _sysid(system_id), _compid(component_id)
{
}

bool MavlinkSender::send_radio(SerialPort& port, uint8_t rssi, std::string& out_error)
{
    out_error.clear();

    // MAVLink reserves 255 for "unknown/no signal" for RADIO.rssi in ArduPilot's usage.
    if (rssi == 255) {
        rssi = 254;
    }

    mavlink_message_t msg;
    // "RADIO" message (mavlink_radio_t). ArduPilot decodes this in handle_radio_status().
    // Other fields are not used for RSSI injection in AP_RSSI (TelemetryRadioRSSI).
    mavlink_msg_radio_pack(_sysid,
                           _compid,
                           &msg,
                           rssi,   // rssi
                           0,      // remrssi
                           0,      // txbuf
                           0,      // noise
                           0,      // remnoise
                           0,      // rxerrors
                           0);     // fixed

    // Ensure a valid MAVLink2 framing.
    msg.magic = MAVLINK_STX;
    msg.compat_flags = 0;
    msg.incompat_flags = 0;
    msg.seq = _seq++;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);

    if (!port.write_all(buf, n, out_error)) {
        return false;
    }
    return true;
}

