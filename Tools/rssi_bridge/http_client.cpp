/*
 * Copyright (c) 2026 Colugo. All rights reserved.
 *
 * Proprietary work product of Colugo.
 * Created by Nati Elinson.
 */
#include "http_client.h"

#ifndef _WIN32

#include <curl/curl.h>

#include <sstream>

static size_t write_cb(char *ptr, size_t size, size_t nmemb, void *userdata)
{
    if (userdata == nullptr || ptr == nullptr) {
        return 0;
    }
    const size_t n = size * nmemb;
    std::string *out = static_cast<std::string*>(userdata);
    out->append(ptr, n);
    return n;
}

HttpClient::HttpClient() : _impl(nullptr)
{
    curl_global_init(CURL_GLOBAL_DEFAULT);
    _impl = curl_easy_init();
}

HttpClient::~HttpClient()
{
    if (_impl != nullptr) {
        curl_easy_cleanup(static_cast<CURL*>(_impl));
        _impl = nullptr;
    }
    curl_global_cleanup();
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

    if (_impl == nullptr) {
        out_error = "curl_easy_init failed";
        return false;
    }

    CURL *curl = static_cast<CURL*>(_impl);
    curl_easy_reset(curl);

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &out_body);

    // Basic auth
    curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    curl_easy_setopt(curl, CURLOPT_USERNAME, basic_auth_user.c_str());
    curl_easy_setopt(curl, CURLOPT_PASSWORD, basic_auth_password.c_str());

    // Timeouts / robustness
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_ms));
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(timeout_ms));
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);

    // Keep it simple for a LAN endpoint
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);

    const CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        out_error = curl_easy_strerror(res);
        return false;
    }

    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    if (http_code < 200 || http_code >= 300) {
        std::ostringstream ss;
        ss << "HTTP " << http_code;
        out_error = ss.str();
        return false;
    }

    return true;
}

#else

// Windows implementation is in http_client_win.cpp
HttpClient::HttpClient() : _impl(nullptr) {}
HttpClient::~HttpClient() {}
bool HttpClient::get(const std::string&,
                     const std::string&,
                     const std::string&,
                     uint32_t,
                     std::string&,
                     std::string& out_error)
{
    out_error = "Windows build misconfigured";
    return false;
}

#endif

