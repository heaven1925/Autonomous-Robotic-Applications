#include "SerialPort.h"

#include <algorithm>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace kobuki {

#ifdef _WIN32
namespace {
std::string windowsErrorMessage(DWORD error) {
    LPSTR message = nullptr;
    const DWORD size = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr, error, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPSTR>(&message), 0, nullptr);
    std::string result = size && message ? std::string(message, size) : "Unknown Windows error";
    if (message) LocalFree(message);
    while (!result.empty() && (result.back() == '\r' || result.back() == '\n')) result.pop_back();
    return result;
}

std::wstring widenAscii(const std::string& s) {
    return std::wstring(s.begin(), s.end());
}
}
#endif

SerialPort::SerialPort()
#ifdef _WIN32
    : handle_(INVALID_HANDLE_VALUE)
#else
    : handle_(nullptr)
#endif
{}

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string& portName) {
#ifdef _WIN32
    close();
    const std::wstring fullName = L"\\\\.\\" + widenAscii(portName);
    handle_ = CreateFileW(fullName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                          OPEN_EXISTING, 0, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
        const DWORD e = GetLastError();
        throw std::runtime_error("Cannot open " + portName + ": " + windowsErrorMessage(e));
    }

    SetupComm(handle_, 4096, 4096);

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(handle_, &dcb)) {
        const DWORD e = GetLastError(); close();
        throw std::runtime_error("GetCommState failed: " + windowsErrorMessage(e));
    }
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    if (!SetCommState(handle_, &dcb)) {
        const DWORD e = GetLastError(); close();
        throw std::runtime_error("SetCommState failed: " + windowsErrorMessage(e));
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = 20;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 200;
    if (!SetCommTimeouts(handle_, &timeouts)) {
        const DWORD e = GetLastError(); close();
        throw std::runtime_error("SetCommTimeouts failed: " + windowsErrorMessage(e));
    }

    PurgeComm(handle_, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
#else
    (void)portName;
    throw std::runtime_error("This SerialPort implementation is Windows-only.");
#endif
}

void SerialPort::close() {
#ifdef _WIN32
    if (handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(handle_);
        handle_ = INVALID_HANDLE_VALUE;
    }
#endif
}

bool SerialPort::isOpen() const {
#ifdef _WIN32
    return handle_ != INVALID_HANDLE_VALUE;
#else
    return false;
#endif
}

std::size_t SerialPort::read(std::uint8_t* buffer, std::size_t capacity) {
#ifdef _WIN32
    if (!isOpen() || !buffer || capacity == 0) return 0;
    DWORD bytesRead = 0;
    const DWORD requested = static_cast<DWORD>((std::min)(capacity, static_cast<std::size_t>(0xFFFFFFFFu)));
    if (!ReadFile(handle_, buffer, requested, &bytesRead, nullptr)) {
        const DWORD e = GetLastError();
        if (e == ERROR_OPERATION_ABORTED) return 0;
        throw std::runtime_error("ReadFile failed: " + windowsErrorMessage(e));
    }
    return static_cast<std::size_t>(bytesRead);
#else
    (void)buffer; (void)capacity; return 0;
#endif
}

void SerialPort::write(const std::uint8_t* data, std::size_t size) {
#ifdef _WIN32
    if (!isOpen()) throw std::runtime_error("Serial port is not open.");
    std::size_t offset = 0;
    while (offset < size) {
        DWORD written = 0;
        const DWORD chunk = static_cast<DWORD>((std::min)(size - offset, static_cast<std::size_t>(0xFFFFFFFFu)));
        if (!WriteFile(handle_, data + offset, chunk, &written, nullptr)) {
            throw std::runtime_error("WriteFile failed: " + windowsErrorMessage(GetLastError()));
        }
        if (written == 0) throw std::runtime_error("WriteFile wrote zero bytes.");
        offset += written;
    }
#else
    (void)data; (void)size;
#endif
}

std::vector<std::string> SerialPort::listPorts() {
    std::vector<std::string> result;
#ifdef _WIN32
    wchar_t target[1024];
    for (int i = 1; i <= 128; ++i) {
        const std::wstring name = L"COM" + std::to_wstring(i);
        if (QueryDosDeviceW(name.c_str(), target, static_cast<DWORD>(std::size(target))) != 0) {
            result.push_back("COM" + std::to_string(i));
        }
    }
#endif
    return result;
}

} // namespace kobuki
