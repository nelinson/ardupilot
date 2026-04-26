/*
 * Copyright (c) 2026 Colugo. All rights reserved.
 *
 * Proprietary work product of Colugo.
 * Created by Nati Elinson.
 */
#pragma once

#include <cstdint>
#include <string>

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud, std::string& out_error);
    void close();

    bool is_open() const;

    // Best-effort write. Returns true if all bytes were written.
    // Never throws.
    bool write_all(const uint8_t* data, size_t len, std::string& out_error);

private:
    // POSIX uses file descriptor, Windows uses HANDLE stored as void*.
    // Kept opaque here to keep the header portable.
    void* _handle;
    std::string _device;
};

