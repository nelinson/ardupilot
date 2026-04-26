/*
 * Copyright (c) 2026 Colugo. All rights reserved.
 *
 * Proprietary work product of Colugo.
 * Created by Nati Elinson.
 */
#include "serial_port.h"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <string>

static std::string win_error(DWORD err)
{
    char *msg = nullptr;
    const DWORD flags = FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS;
    const DWORD len = FormatMessageA(flags, nullptr, err, 0, reinterpret_cast<LPSTR>(&msg), 0, nullptr);
    std::string out = (len && msg) ? std::string(msg, msg + len) : "unknown";
    if (msg) {
        LocalFree(msg);
    }
    return out;
}

static std::string normalize_com_path(const std::string& device)
{
    // Accept "COM12" or "\\\\.\\COM12".
    if (device.rfind("\\\\.\\", 0) == 0) {
        return device;
    }
    if (device.size() >= 3 && (device.rfind("COM", 0) == 0 || device.rfind("com", 0) == 0)) {
        return std::string("\\\\.\\") + device;
    }
    return device;
}

SerialPort::SerialPort() : _handle(nullptr)
{
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& device, int baud, std::string& out_error)
{
    (void)baud; // For USB CDC, baud is mostly ignored; still configured below.
    out_error.clear();
    close();

    _device = device;
    const std::string path = normalize_com_path(device);

    HANDLE h = CreateFileA(path.c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0,
                           nullptr,
                           OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL,
                           nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        out_error = std::string("CreateFile failed: ") + win_error(GetLastError());
        return false;
    }

    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        out_error = std::string("GetCommState failed: ") + win_error(GetLastError());
        CloseHandle(h);
        return false;
    }

    dcb.BaudRate = static_cast<DWORD>(baud);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(h, &dcb)) {
        out_error = std::string("SetCommState failed: ") + win_error(GetLastError());
        CloseHandle(h);
        return false;
    }

    COMMTIMEOUTS t;
    SecureZeroMemory(&t, sizeof(t));
    // Non-blocking-ish write behavior.
    t.ReadIntervalTimeout = MAXDWORD;
    t.ReadTotalTimeoutConstant = 0;
    t.ReadTotalTimeoutMultiplier = 0;
    t.WriteTotalTimeoutConstant = 0;
    t.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(h, &t);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    _handle = h;
    return true;
}

void SerialPort::close()
{
    if (_handle != nullptr) {
        CloseHandle(reinterpret_cast<HANDLE>(_handle));
        _handle = nullptr;
    }
}

bool SerialPort::is_open() const
{
    return _handle != nullptr;
}

bool SerialPort::write_all(const uint8_t* data, size_t len, std::string& out_error)
{
    out_error.clear();
    if (_handle == nullptr) {
        out_error = "serial not open";
        return false;
    }
    if (data == nullptr || len == 0) {
        return true;
    }

    size_t off = 0;
    while (off < len) {
        DWORD written = 0;
        const DWORD chunk = (len - off > 0xFFFFFFFFu) ? 0xFFFFFFFFu : static_cast<DWORD>(len - off);
        if (!WriteFile(reinterpret_cast<HANDLE>(_handle), data + off, chunk, &written, nullptr)) {
            out_error = std::string("WriteFile failed: ") + win_error(GetLastError());
            return false;
        }
        off += static_cast<size_t>(written);
        if (written == 0) {
            Sleep(1);
        }
    }
    return true;
}

#endif

