#include "http_client.h"
#include "mavlink_sender.h"
#include "rssi_processor.h"
#include "serial_port.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

static volatile std::sig_atomic_t g_should_exit = 0;

static void sigint_handler(int)
{
    g_should_exit = 1;
}

struct Options {
    std::string url = "http://192.168.0.27/localrfstatus.json";
#ifdef _WIN32
    std::string serial_device = "COM12";
#else
    std::string serial_device = "/dev/ttyACM0";
#endif
    int rate_hz = 10;
    uint32_t http_timeout_ms = 1000;
    uint8_t system_id = 255;
    uint8_t component_id = 191; // MAV_COMP_ID_ONBOARD_COMPUTER
    bool enable_ema = false;
    float ema_alpha = 1.0f;
    bool dry_run = false;
};

static void usage(const char* argv0)
{
    std::cerr
        << "Usage: " << argv0 << " [--url <string>] [--serial <path>] [--rate <Hz>]\n"
        << "             [--timeout-ms <ms>] [--sysid <1..255>] [--compid <1..255>]\n"
        << "             [--ema-alpha <0..1>] [--dry-run]\n"
        << "\n"
        << "Defaults:\n"
        << "  --url         http://192.168.0.27/localrfstatus.json\n"
#ifdef _WIN32
        << "  --serial      COM12\n"
#else
        << "  --serial      /dev/ttyACM0\n"
#endif
        << "  --rate        10\n"
        << "  --timeout-ms  1000\n"
        << "\n"
        << "Options:\n"
        << "  --dry-run     Poll/parse/compute RSSI but do not open/write serial\n";
}

static bool parse_int(const char* s, int& out)
{
    if (s == nullptr || *s == '\0') {
        return false;
    }
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (end == nullptr || *end != '\0') {
        return false;
    }
    out = static_cast<int>(v);
    return true;
}

static bool parse_uint8(const char* s, uint8_t& out)
{
    int v = 0;
    if (!parse_int(s, v)) {
        return false;
    }
    if (v < 1 || v > 255) {
        return false;
    }
    out = static_cast<uint8_t>(v);
    return true;
}

static bool parse_float(const char* s, float& out)
{
    if (s == nullptr || *s == '\0') {
        return false;
    }
    char* end = nullptr;
    const float v = std::strtof(s, &end);
    if (end == nullptr || *end != '\0') {
        return false;
    }
    out = v;
    return true;
}

static bool parse_args(int argc, char** argv, Options& opt)
{
    for (int i = 1; i < argc; i++) {
        const std::string a(argv[i]);
        auto need = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (a == "--url") {
            const char* v = need("--url");
            if (!v) return false;
            opt.url = v;
        } else if (a == "--serial") {
            const char* v = need("--serial");
            if (!v) return false;
            opt.serial_device = v;
        } else if (a == "--rate") {
            const char* v = need("--rate");
            if (!v) return false;
            int hz = 0;
            if (!parse_int(v, hz) || hz < 1 || hz > 1000) {
                std::cerr << "Invalid --rate\n";
                return false;
            }
            opt.rate_hz = hz;
        } else if (a == "--timeout-ms") {
            const char* v = need("--timeout-ms");
            if (!v) return false;
            int ms = 0;
            if (!parse_int(v, ms) || ms < 50 || ms > 10000) {
                std::cerr << "Invalid --timeout-ms\n";
                return false;
            }
            opt.http_timeout_ms = static_cast<uint32_t>(ms);
        } else if (a == "--sysid") {
            const char* v = need("--sysid");
            if (!v) return false;
            if (!parse_uint8(v, opt.system_id)) {
                std::cerr << "Invalid --sysid\n";
                return false;
            }
        } else if (a == "--compid") {
            const char* v = need("--compid");
            if (!v) return false;
            if (!parse_uint8(v, opt.component_id)) {
                std::cerr << "Invalid --compid\n";
                return false;
            }
        } else if (a == "--ema-alpha") {
            const char* v = need("--ema-alpha");
            if (!v) return false;
            float alpha = 0.0f;
            if (!parse_float(v, alpha) || alpha < 0.0f || alpha > 1.0f) {
                std::cerr << "Invalid --ema-alpha\n";
                return false;
            }
            opt.enable_ema = true;
            opt.ema_alpha = alpha;
        } else if (a == "--dry-run") {
            opt.dry_run = true;
        } else if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown argument: " << a << "\n";
            usage(argv[0]);
            return false;
        }
    }
    return true;
}

static bool discover_aircraft_node_id_from_status_json(const std::string& body,
                                                       int& out_local_node_id,
                                                       int& out_aircraft_node_id,
                                                       std::string& out_error)
{
    out_error.clear();
    try {
        const auto j = nlohmann::json::parse(body);
        const auto& mesh = j.at("Status").at("Mesh1");
        out_local_node_id = mesh.at("NodeId").get<int>();

        const auto& rs = mesh.at("RemoteStatus");
        for (size_t i = 0; i < rs.size(); i++) {
            const auto& entry = rs.at(i);
            if (!entry.contains("timeout") || !entry.at("timeout").is_boolean() || !entry.at("timeout").get<bool>()) {
                continue;
            }
            if (!entry.contains("UnitStatus") || entry.at("UnitStatus").is_null()) {
                continue;
            }
            const auto& us = entry.at("UnitStatus");
            if (!us.contains("nodeId")) {
                continue;
            }
            const int node_id = us.at("nodeId").get<int>();
            if (node_id != out_local_node_id) {
                out_aircraft_node_id = static_cast<int>(i);
                return true;
            }
        }
        out_error = "no aircraft nodeId found in RemoteStatus";
        return false;
    } catch (const std::exception& e) {
        out_error = e.what();
        return false;
    }
}

static bool parse_snr_from_localrfstatus_json(const std::string& body,
                                             int aircraft_node_id,
                                             float& out_snr_db,
                                             bool& out_sig_valid,
                                             std::string& out_error)
{
    out_error.clear();
    try {
        const auto j = nlohmann::json::parse(body);
        const auto& s = j.at("LocalRfStatus").at("LocalDemodStatus");
        out_sig_valid = s.at("sigValid").get<bool>();

        const auto& snr = s.at("SNR");
        if (!snr.is_array()) {
            out_error = "SNR is not an array";
            return false;
        }
        if (aircraft_node_id < 0 || aircraft_node_id >= static_cast<int>(snr.size())) {
            out_error = "aircraft_node_id out of range";
            return false;
        }
        out_snr_db = snr.at(aircraft_node_id).get<float>();
        return true;
    } catch (const std::exception& e) {
        out_error = e.what();
        return false;
    }
}

static std::string url_dirname(const std::string& url)
{
    const auto pos = url.find_last_of('/');
    if (pos == std::string::npos) {
        return url;
    }
    return url.substr(0, pos);
}

static float scale_snr_to_unit(float snr_db, float snr_lo, float snr_hi)
{
    // Sentinel -3.0 means "no node / no link"
    if (snr_db < 0.0f) {
        return 0.0f;
    }
    if (snr_hi <= snr_lo) {
        return 0.0f;
    }
    const float x = (snr_db - snr_lo) / (snr_hi - snr_lo);
    return std::max(0.0f, std::min(1.0f, x));
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, sigint_handler);

    Options opt;
    if (!parse_args(argc, argv, opt)) {
        return 2;
    }

    const std::string auth_user = "";
    const std::string auth_pass = "Eastwood";

    HttpClient http;
    SerialPort port;
    MavlinkSender mav(opt.system_id, opt.component_id);
    RssiProcessor rssi_proc;

    const int baud = 115200;
    const int period_ms = std::max(1, 1000 / opt.rate_hz);

    std::string err;
    if (!opt.dry_run) {
        if (!port.open(opt.serial_device, baud, err)) {
            std::cerr << "Serial open error (" << opt.serial_device << "): " << err << "\n";
            std::cerr << "Will retry...\n";
        }
    } else {
        std::cerr << "Dry-run mode: not opening serial (" << opt.serial_device << ")\n";
    }

    // Node discovery (cached, rediscovered on no-link)
    int local_node_id = -1;
    int aircraft_node_id = -1;
    const std::string base = url_dirname(opt.url);
    const std::string status_url = base + "/status.json";

    // Repurpose the old dBm scaling range as SNR scaling.
    const float SNR_LO = 3.1f;
    const float SNR_HI = 20.0f;

    while (!g_should_exit) {
        const auto t0 = std::chrono::steady_clock::now();

        // Ensure serial is open
        if (!opt.dry_run) {
            if (!port.is_open()) {
                if (!port.open(opt.serial_device, baud, err)) {
                    std::cerr << "Serial reopen failed (" << opt.serial_device << "): " << err << "\n";
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
                std::cerr << "Serial connected: " << opt.serial_device << "\n";
            }
        }

        if (aircraft_node_id < 0) {
            std::string sbody;
            if (!http.get(status_url, auth_user, auth_pass, opt.http_timeout_ms, sbody, err)) {
                std::cerr << "HTTP status.json error: " << err << "\n";
            } else {
                std::string jerr;
                int local = -1;
                int ac = -1;
                if (!discover_aircraft_node_id_from_status_json(sbody, local, ac, jerr)) {
                    std::cerr << "status.json parse/discovery error: " << jerr << "\n";
                } else {
                    local_node_id = local;
                    aircraft_node_id = ac;
                    std::cerr << "Discovered aircraft node id: " << aircraft_node_id << " (local " << local_node_id << ")\n";
                }
            }
        }

        std::string body;
        if (!http.get(opt.url, auth_user, auth_pass, opt.http_timeout_ms, body, err)) {
            std::cerr << "HTTP localrfstatus.json error: " << err << "\n";
        } else if (aircraft_node_id >= 0) {
            float snr_db = -3.0f;
            bool valid = false;
            std::string jerr;
            if (!parse_snr_from_localrfstatus_json(body, aircraft_node_id, snr_db, valid, jerr)) {
                std::cerr << "localrfstatus.json parse error: " << jerr << "\n";
            } else {
                const float scaled = scale_snr_to_unit(snr_db, SNR_LO, SNR_HI);
                const bool link_ok = valid && (snr_db > 3.1f);
                float scaled_used = scaled;
                if (opt.enable_ema) {
                    float filtered = scaled_used;
                    if (rssi_proc.ema_update(scaled_used, opt.ema_alpha, filtered)) {
                        scaled_used = filtered;
                    }
                }
                const uint8_t rssi = static_cast<uint8_t>(std::max(0, std::min(254, int(scaled_used * 254.0f + 0.5f))));
                const int node_snapshot = aircraft_node_id;

                if (!link_ok) {
                    // Force re-discovery on link loss.
                    aircraft_node_id = -1;
                }

                if (opt.dry_run) {
                    std::cout << "[N:" << node_snapshot << "][SNR:" << snr_db << "][RSSI:" << int(rssi) << "]\n";
                } else {
                    std::string werr;
                    if (!mav.send_radio(port, rssi, werr)) {
                        std::cerr << "Serial write failed: " << werr << " (reconnecting)\n";
                        port.close();
                    } else {
                        std::cout << "[N:" << node_snapshot << "][SNR:" << snr_db << "][RSSI:" << int(rssi) << "]\n";
                    }
                }
            }
        }

        const auto t1 = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        const int sleep_ms = period_ms - static_cast<int>(elapsed_ms);
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    std::cerr << "Exiting.\n";
    return 0;
}

