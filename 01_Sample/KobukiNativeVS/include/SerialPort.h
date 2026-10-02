#pragma once

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace kobuki {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    void open(const std::string& portName);
    void close();
    bool isOpen() const;

    std::size_t read(std::uint8_t* buffer, std::size_t capacity);
    void write(const std::uint8_t* data, std::size_t size);

    static std::vector<std::string> listPorts();

private:
#ifdef _WIN32
    HANDLE handle_;
#else
    void* handle_;
#endif
};

} // namespace kobuki
