// ComPorts.h - the serial ports Windows knows right now (SetupAPI, "Ports"
// device class), with a guess which one is the Kobuki and which the Arduino:
//   Kobuki  : FTDI adapter (VID_0403 / FTDIBUS)
//   Arduino : CH340 / CH341 clone (VID_1A86) or a genuine board (VID_2341, VID_2A03)
// Same rules as run.ps1 (Get-RobotPorts).
#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <initguid.h>
#include <devguid.h>
#include <setupapi.h>

#include <algorithm>
#include <cwctype>
#include <string>
#include <vector>

namespace ports {

struct Port {
    std::wstring name;        // COM6
    std::wstring friendly;    // USB Serial Port (COM6)
    std::wstring hardwareId;
    bool ftdi = false, arduino = false;
    int number = 0;
};

inline std::wstring upper(std::wstring s) {
    for (wchar_t& c : s) c = static_cast<wchar_t>(std::towupper(c));
    return s;
}

inline std::wstring property(HDEVINFO set, SP_DEVINFO_DATA& dev, DWORD prop) {
    wchar_t buf[1024] = {};
    DWORD type = 0;
    if (!SetupDiGetDeviceRegistryPropertyW(set, &dev, prop, &type, reinterpret_cast<BYTE*>(buf), sizeof(buf) - sizeof(wchar_t), nullptr))
        return {};
    return buf;   // REG_MULTI_SZ: the first string is enough
}

inline std::vector<Port> list() {
    std::vector<Port> out;
    const HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return out;
    SP_DEVINFO_DATA dev{};
    dev.cbSize = sizeof(dev);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &dev); ++i) {
        Port p;
        p.friendly = property(set, dev, SPDRP_FRIENDLYNAME);
        p.hardwareId = property(set, dev, SPDRP_HARDWAREID);
        // Port name: "(COMn)" in the friendly name, else the device's PortName registry value.
        const std::wstring f = p.friendly;
        const auto at = f.rfind(L"(COM");
        if (at != std::wstring::npos) {
            const auto end = f.find(L')', at);
            p.name = f.substr(at + 1, end == std::wstring::npos ? std::wstring::npos : end - at - 1);
        } else {
            const HKEY key = SetupDiOpenDevRegKey(set, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
            if (key != INVALID_HANDLE_VALUE) {
                wchar_t name[64] = {};
                DWORD size = sizeof(name) - sizeof(wchar_t);
                if (RegQueryValueExW(key, L"PortName", nullptr, nullptr, reinterpret_cast<BYTE*>(name), &size) == ERROR_SUCCESS) p.name = name;
                RegCloseKey(key);
            }
        }
        if (p.name.rfind(L"COM", 0) != 0) continue;   // LPT and friends
        p.number = _wtoi(p.name.c_str() + 3);
        const std::wstring u = upper(p.friendly + L" " + p.hardwareId);
        p.ftdi = u.find(L"FTDI") != std::wstring::npos || u.find(L"FT232") != std::wstring::npos || u.find(L"VID_0403") != std::wstring::npos;
        p.arduino = u.find(L"CH340") != std::wstring::npos || u.find(L"CH341") != std::wstring::npos || u.find(L"ARDUINO") != std::wstring::npos ||
                    u.find(L"VID_1A86") != std::wstring::npos || u.find(L"VID_2341") != std::wstring::npos || u.find(L"VID_2A03") != std::wstring::npos;
        out.push_back(p);
    }
    SetupDiDestroyDeviceInfoList(set);
    std::sort(out.begin(), out.end(), [](const Port& a, const Port& b) { return a.number < b.number; });
    return out;
}

// "COM6  -  USB Serial Port (COM6)  [Kobuki?]"
inline std::wstring label(const Port& p) {
    std::wstring s = p.name + L"  -  " + p.friendly;
    if (p.ftdi) s += L"  [FTDI: Kobuki?]";
    if (p.arduino) s += L"  [CH340: Arduino?]";
    return s;
}

// The "COMn" at the start of a combo box text (the user may also type just "COM7").
inline std::wstring portOf(const std::wstring& text) {
    std::wstring t = upper(text);
    const auto at = t.find(L"COM");
    if (at == std::wstring::npos) return {};
    std::size_t end = at + 3;
    while (end < t.size() && std::iswdigit(t[end])) ++end;
    if (end == at + 3) return {};
    return t.substr(at, end - at);
}

} // namespace ports
