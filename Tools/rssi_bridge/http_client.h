/*
 * Copyright (c) 2026 Colugo. All rights reserved.
 *
 * Proprietary work product of Colugo.
 * Created by Nati Elinson.
 */
#pragma once

#include <cstdint>
#include <string>

class HttpClient {
public:
    HttpClient();
    ~HttpClient();

    HttpClient(const HttpClient&) = delete;
    HttpClient& operator=(const HttpClient&) = delete;

    // Returns true on success and writes response body into out_body.
    // Never throws.
    bool get(const std::string& url,
             const std::string& basic_auth_user,
             const std::string& basic_auth_password,
             uint32_t timeout_ms,
             std::string& out_body,
             std::string& out_error);

private:
    void* _impl;
};

