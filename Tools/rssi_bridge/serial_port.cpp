/*
 * Copyright (c) 2026 Colugo. All rights reserved.
 *
 * Proprietary work product of Colugo.
 * Created by Nati Elinson.
 */
#include "serial_port.h"

#ifndef _WIN32

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B115200;
    }
}

SerialPort::SerialPort() : _handle(reinterpret_cast<void*>(-1))
{
}

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& device, int baud, std::string& out_error)
{
    out_error.clear();
    close();

    _device = device;
    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        out_error = std::string("open failed: ") + strerror(errno);
        return false;
    }
    _handle = reinterpret_cast<void*>(static_cast<intptr_t>(fd));

    termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        out_error = std::string("tcgetattr failed: ") + strerror(errno);
        close();
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;

    const speed_t spd = baud_to_speed(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        out_error = std::string("tcsetattr failed: ") + strerror(errno);
        close();
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

void SerialPort::close()
{
    const int fd = static_cast<int>(reinterpret_cast<intptr_t>(_handle));
    if (fd >= 0) {
        ::close(fd);
    }
    _handle = reinterpret_cast<void*>(-1);
}

bool SerialPort::is_open() const
{
    const int fd = static_cast<int>(reinterpret_cast<intptr_t>(_handle));
    return fd >= 0;
}

bool SerialPort::write_all(const uint8_t* data, size_t len, std::string& out_error)
{
    out_error.clear();
    const int fd = static_cast<int>(reinterpret_cast<intptr_t>(_handle));
    if (fd < 0) {
        out_error = "serial not open";
        return false;
    }
    if (data == nullptr || len == 0) {
        return true;
    }

    size_t off = 0;
    while (off < len) {
        const ssize_t n = ::write(fd, data + off, len - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            usleep(1000);
            continue;
        }
        out_error = std::string("write failed: ") + strerror(errno);
        return false;
    }
    return true;
}

#else

// Windows implementation is in serial_port_win.cpp
SerialPort::SerialPort() : _handle(nullptr) {}
SerialPort::~SerialPort() { close(); }
bool SerialPort::open(const std::string&, int, std::string& out_error) { out_error = "Windows build misconfigured"; return false; }
void SerialPort::close() {}
bool SerialPort::is_open() const { return false; }
bool SerialPort::write_all(const uint8_t*, size_t, std::string& out_error) { out_error = "Windows build misconfigured"; return false; }

#endif

