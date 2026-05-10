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

    const std::wstring wurl = to_wide(url);
    if (wurl.empty() && !url.empty()) {
        out_error = "URL UTF-8 conversion failed";
        return false;
    }

    // Use wide WinHTTP APIs explicitly (URL_COMPONENTSA is not reliable with UNICODE + some SDKs).
    URL_COMPONENTSW uc;
    ZeroMemory(&uc, sizeof(uc));
    uc.dwStructSize = sizeof(uc);

    wchar_t host[256] = {};
    wchar_t path[4096] = {};
    uc.lpszHostName = host;
    uc.dwHostNameLength = static_cast<DWORD>(sizeof(host) / sizeof(host[0]));
    uc.lpszUrlPath = path;
    uc.dwUrlPathLength = static_cast<DWORD>(sizeof(path) / sizeof(path[0]));
    uc.dwSchemeLength = static_cast<DWORD>(-1);

    // Use WinHttpCrackUrl macro (maps to WinHttpCrackUrlW under UNICODE). Some SDKs do not
    // declare WinHttpCrackUrlW as a public C identifier.
    if (!WinHttpCrackUrl(wurl.c_str(), static_cast<DWORD>(wurl.length()), 0, reinterpret_cast<LPURL_COMPONENTS>(&uc))) {
        out_error = std::string("WinHttpCrackUrl failed: ") + win_error(GetLastError());
        return false;
    }

    const bool is_https = (uc.nScheme == INTERNET_SCHEME_HTTPS);
    const INTERNET_PORT port = uc.nPort;

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

    HINTERNET hConnect = WinHttpConnect(hSession, uc.lpszHostName, port, 0);
    if (!hConnect) {
        out_error = std::string("WinHttpConnect failed: ") + win_error(GetLastError());
        WinHttpCloseHandle(hSession);
        return false;
    }

    DWORD flags = is_https ? WINHTTP_FLAG_SECURE : 0;
    HINTERNET hRequest = WinHttpOpenRequest(hConnect,
                                           L"GET",
                                           uc.lpszUrlPath,
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
