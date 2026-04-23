#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Networking/AP_Networking_Config.h>

#ifndef AP_RSSI_ENABLED
#define AP_RSSI_ENABLED 1
#endif

// Master switch for the Solo8 / external-Ethernet-RSSI integration.
// When 0 (the default) the firmware builds exactly like stock ArduPilot
// regardless of whether AP_Networking is linked in: no UDP RSSI receiver
// thread, no HTTP poller thread, no Solo8-specific code reachable.
//
// Defined to 1 only by the Pixhawk6X-TrackerEth hwdef (or by passing
// -DAP_SOLO8_RSSI_EXT_ENABLED=1 to the build), which also powers on the
// onboard Ethernet PHY. Setting this to 0 (or building a different board
// target that does not define it) disables the full feature set in one
// place -- the RSSI_TYPE=6/7 options will still be selectable in the GCS
// but the backends are compiled out, so they will simply report 0.
#ifndef AP_SOLO8_RSSI_EXT_ENABLED
#define AP_SOLO8_RSSI_EXT_ENABLED 0
#endif

// UDP Ethernet RSSI backend (generic: receives a small binary datagram
// and exposes it as the receiver RSSI). Gated by the Solo8 master switch
// *and* the AP_Networking socket layer being available.
#ifndef AP_RSSI_UDP_ENABLED
#define AP_RSSI_UDP_ENABLED (AP_RSSI_ENABLED && AP_SOLO8_RSSI_EXT_ENABLED && AP_NETWORKING_SOCKETS_ENABLED)
#endif

// HTTP/JSON polling backend for receivers that only expose their RF
// status over HTTP (e.g. Silvus Solo8 radios serving
// /localrfstatus.json). Uses the same AP_Networking socket stack; one
// dedicated worker thread performs periodic HTTP GETs and extracts the
// strongest sigLevA0 / sigLevB0 value when sigValid is true.
#ifndef AP_RSSI_HTTP_ENABLED
#define AP_RSSI_HTTP_ENABLED (AP_RSSI_ENABLED && AP_SOLO8_RSSI_EXT_ENABLED && AP_NETWORKING_SOCKETS_ENABLED)
#endif
