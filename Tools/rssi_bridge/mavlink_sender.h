#pragma once

#include <cstdint>
#include <string>

class SerialPort;

class MavlinkSender {
public:
    MavlinkSender(uint8_t system_id, uint8_t component_id);

    // Sends MAVLink RADIO message with RSSI in [0..254]. Returns true on success.
    bool send_radio(SerialPort& port, uint8_t rssi, std::string& out_error);

private:
    uint8_t _sysid;
    uint8_t _compid;
    uint8_t _seq = 0;
};

