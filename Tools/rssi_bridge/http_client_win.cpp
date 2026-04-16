#include "http_client.h"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winhttp.h>

#include <sstream>
#include <string>
#include <vector>

#pragma comment(lib, "winhttp.lib")

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

static std::wstring to_wide(const std::string& s)
{
    if (s.empty()) {
        return std::wstring();
    }
    const int needed = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    if (needed <= 0) {
        return std::wstring();
    }
    std::wstring out(static_cast<size_t>(needed - 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, &out[0], needed);
    return out;
}

HttpClient::HttpClient() : _impl(nullptr)
{
}

HttpClient::~HttpClient()
{
}

bool HttpClient::get(const std::string& url,
                     const std::string& basic_auth_user,
                     const std::string& basic_auth_password,
                     uint32_t timeout_ms,
                     std::string& out_body,
                     std::string& out_error)
{
    out_body.clear();
    out_error.clear();

    URL_COMPONENTSA parts;
    ZeroMemory(&parts, sizeof(parts));
    parts.dwStructSize = sizeof(parts);

    char host[256] = {};
    char path[2048] = {};
    parts.lpszHostName = host;
    parts.dwHostNameLength = sizeof(host);
    parts.lpszUrlPath = path;
    parts.dwUrlPathLength = sizeof(path);
    parts.dwSchemeLength = 1;

    if (!WinHttpCrackUrl(url.c_str(), 0, 0, reinterpret_cast<URL_COMPONENTS*>(&parts))) {
        out_error = std::string("WinHttpCrackUrl failed: ") + win_error(GetLastError());
        return false;
    }

    const bool is_https = (parts.nScheme == INTERNET_SCHEME_HTTPS);
    const INTERNET_PORT port = parts.nPort;

    HINTERNET hSession = WinHttpOpen(L"rssi_bridge/1.0",
                                     WINHTTP_ACCESS_TYPE_DEFAULT_PROXY,
                                     WINHTTP_NO_PROXY_NAME,
                                     WINHTTP_NO_PROXY_BYPASS,
                                     0);
    if (!hSession) {
        out_error = std::string("WinHttpOpen failed: ") + win_error(GetLastError());
        return false;
    }

    WinHttpSetTimeouts(hSession,
                       static_cast<int>(timeout_ms),
                       static_cast<int>(timeout_ms),
                       static_cast<int>(timeout_ms),
                       static_cast<int>(timeout_ms));

    const std::wstring whost = to_wide(std::string(host, host + parts.dwHostNameLength));
    const std::wstring wpath = to_wide(std::string(path, path + parts.dwUrlPathLength));

    HINTERNET hConnect = WinHttpConnect(hSession, whost.c_str(), port, 0);
    if (!hConnect) {
        out_error = std::string("WinHttpConnect failed: ") + win_error(GetLastError());
        WinHttpCloseHandle(hSession);
        return false;
    }

    DWORD flags = is_https ? WINHTTP_FLAG_SECURE : 0;
    HINTERNET hRequest = WinHttpOpenRequest(hConnect,
                                           L"GET",
                                           wpath.c_str(),
                                           nullptr,
                                           WINHTTP_NO_REFERER,
                                           WINHTTP_DEFAULT_ACCEPT_TYPES,
                                           flags);
    if (!hRequest) {
        out_error = std::string("WinHttpOpenRequest failed: ") + win_error(GetLastError());
        WinHttpCloseHandle(hConnect);
        WinHttpCloseHandle(hSession);
        return false;
    }

    if (!basic_auth_user.empty() || !basic_auth_password.empty()) {
        const std::wstring wuser = to_wide(basic_auth_user);
        const std::wstring wpass = to_wide(basic_auth_password);
        WinHttpSetCredentials(hRequest,
                              WINHTTP_AUTH_TARGET_SERVER,
                              WINHTTP_AUTH_SCHEME_BASIC,
                              wuser.c_str(),
                              wpass.c_str(),
                              nullptr);
    } else {
        // Username may be empty in this POC; WinHTTP still accepts it.
        const std::wstring wpass = to_wide(basic_auth_password);
        WinHttpSetCredentials(hRequest,
                              WINHTTP_AUTH_TARGET_SERVER,
                              WINHTTP_AUTH_SCHEME_BASIC,
                              L"",
                              wpass.c_str(),
                              nullptr);
    }

    BOOL ok = WinHttpSendRequest(hRequest,
                                 WINHTTP_NO_ADDITIONAL_HEADERS,
                                 0,
                                 WINHTTP_NO_REQUEST_DATA,
                                 0,
                                 0,
                                 0);
    if (!ok) {
        out_error = std::string("WinHttpSendRequest failed: ") + win_error(GetLastError());
        WinHttpCloseHandle(hRequest);
        WinHttpCloseHandle(hConnect);
        WinHttpCloseHandle(hSession);
        return false;
    }

    ok = WinHttpReceiveResponse(hRequest, nullptr);
    if (!ok) {
        out_error = std::string("WinHttpReceiveResponse failed: ") + win_error(GetLastError());
        WinHttpCloseHandle(hRequest);
        WinHttpCloseHandle(hConnect);
        WinHttpCloseHandle(hSession);
        return false;
    }

    DWORD status_code = 0;
    DWORD status_size = sizeof(status_code);
    WinHttpQueryHeaders(hRequest,
                        WINHTTP_QUERY_STATUS_CODE | WINHTTP_QUERY_FLAG_NUMBER,
                        WINHTTP_HEADER_NAME_BY_INDEX,
                        &status_code,
                        &status_size,
                        WINHTTP_NO_HEADER_INDEX);
    if (status_code < 200 || status_code >= 300) {
        std::ostringstream ss;
        ss << "HTTP " << status_code;
        out_error = ss.str();
        WinHttpCloseHandle(hRequest);
        WinHttpCloseHandle(hConnect);
        WinHttpCloseHandle(hSession);
        return false;
    }

    std::string body;
    for (;;) {
        DWORD avail = 0;
        if (!WinHttpQueryDataAvailable(hRequest, &avail)) {
            out_error = std::string("WinHttpQueryDataAvailable failed: ") + win_error(GetLastError());
            break;
        }
        if (avail == 0) {
            out_body = body;
            WinHttpCloseHandle(hRequest);
            WinHttpCloseHandle(hConnect);
            WinHttpCloseHandle(hSession);
            return true;
        }
        std::vector<char> buf(avail);
        DWORD read = 0;
        if (!WinHttpReadData(hRequest, buf.data(), avail, &read)) {
            out_error = std::string("WinHttpReadData failed: ") + win_error(GetLastError());
            break;
        }
        body.append(buf.data(), buf.data() + read);
    }

    WinHttpCloseHandle(hRequest);
    WinHttpCloseHandle(hConnect);
    WinHttpCloseHandle(hSession);
    return false;
}

#endif

