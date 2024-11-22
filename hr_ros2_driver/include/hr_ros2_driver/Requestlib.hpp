#pragma once

#include <string>
#include <chrono>
#include <nlohmann/json.hpp>
#include <memory>
#include <curl/curl.h>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <future>
#include <deque>
#include <iostream>

class RequestLib {
public:
    RequestLib(const std::string& host = "");
    ~RequestLib();

    std::pair<nlohmann::json, int> get(const std::string& endpoint, const std::string& params = "");
    std::pair<std::string, int> get_str(const std::string& endpoint, const std::string& params = "");
    std::pair<nlohmann::json, int> post(const std::string& endpoint, const std::string& params = "", const nlohmann::json& body = nlohmann::json());
    std::pair<nlohmann::json, int> post_file(const std::string& endpoint, const std::string& params, const std::string& file_content);
    std::pair<nlohmann::json, int> put(const std::string& endpoint, const std::string& params = "", const nlohmann::json& body = nlohmann::json());
    std::pair<std::string, int> del(const std::string& endpoint, const std::string& params = "");

private:
    class Impl;
    std::unique_ptr<Impl> pimpl;
    std::string host_url;

    std::string create_url(const std::string& endpoint, const std::string& params) const {
        std::string url = host_url + endpoint;
        if (!params.empty()) {
            url += "?" + params;
        }
        return url;
    }
};

class RequestLib::Impl {
private:
    struct RequestConfig {
        bool enable_http2 = true;
        int timeout_ms = 8;                     // 8ms 타임아웃
        int connection_timeout_ms = 10;         // 10ms 연결 타임아웃
        int max_concurrent_requests = 10;
        int retry_count = 1;                    // 빠른 실패를 위해 재시도 1회로 감소
        bool enable_compression = false;        // 압축 해제 오버헤드 제거
        int connection_pool_size = 10;          // 연결 풀 증가
        int max_calls_per_second = 200;         // 초당 호출 수 대폭 증가
        std::chrono::milliseconds retry_delay{1}; // 1ms 재시도 간격
    };
    struct CachedConnection {
        CURL* handle;
        std::string endpoint;
        std::chrono::steady_clock::time_point last_used;
        bool in_use;
    };

    class FastRateLimiter {
    public:
        explicit FastRateLimiter(int max_calls_per_second)
            : max_calls_per_second_(max_calls_per_second)
            , min_interval_(std::chrono::microseconds(1000000 / max_calls_per_second))
            , last_call_(std::chrono::steady_clock::now()) {}

        void wait() {
            std::lock_guard<std::mutex> lock(mutex_);
            auto now = std::chrono::steady_clock::now();
            auto elapsed = now - last_call_;
            
            if (elapsed < min_interval_) {
                std::this_thread::sleep_for(min_interval_ - elapsed);
            }
            
            last_call_ = std::chrono::steady_clock::now();
        }

    private:
        const int max_calls_per_second_;
        const std::chrono::microseconds min_interval_;
        std::chrono::steady_clock::time_point last_call_;
        std::mutex mutex_;
    };

    class ConnectionPool {
    public:
        ConnectionPool(size_t size) : pool_size_(size) {
            for (size_t i = 0; i < size; ++i) {
                add_connection();
            }
        }

        ~ConnectionPool() {
            for (auto& conn : connections_) {
                curl_easy_cleanup(conn.handle);
            }
        }

        CachedConnection* acquire(const std::string& endpoint) {
            std::lock_guard<std::mutex> lock(mutex_);
            
            // 이미 있는 연결 재사용
            for (auto& conn : connections_) {
                if (!conn.in_use && conn.endpoint == endpoint) {
                    conn.in_use = true;
                    conn.last_used = std::chrono::steady_clock::now();
                    return &conn;
                }
            }
            
            // 새 연결 필요
            for (auto& conn : connections_) {
                if (!conn.in_use) {
                    curl_easy_reset(conn.handle);  // 추가 필요
                    conn.in_use = true;
                    conn.endpoint = endpoint;
                    conn.last_used = std::chrono::steady_clock::now();
                    return &conn;
                }
            }

            // 풀이 가득 찼다면 가장 오래된 연결 재사용
            auto oldest = std::min_element(connections_.begin(), connections_.end(),
                [](const auto& a, const auto& b) {
                    return a.last_used < b.last_used;
                });
            
            oldest->in_use = true;
            oldest->endpoint = endpoint;
            oldest->last_used = std::chrono::steady_clock::now();
            return &(*oldest);
        }

        void release(CachedConnection* conn) {
            if (!conn) return;
            std::lock_guard<std::mutex> lock(mutex_);
            conn->in_use = false;
        }

    private:
        std::vector<CachedConnection> connections_;
        std::mutex mutex_;
        const size_t pool_size_;

        void add_connection() {
            CURL* handle = curl_easy_init();
            if (handle) {
                connections_.push_back({
                    handle,
                    "",
                    std::chrono::steady_clock::now(),
                    false
                });
            }
        }
    };

public:
    Impl() : config_(), 
             rate_limiter_(config_.max_calls_per_second),
             connection_pool_(config_.connection_pool_size) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    }

    ~Impl() {
        curl_global_cleanup();
    }

    template<typename T>
    T execute_with_retry(const std::function<T(CURL*)>& operation, const std::string& endpoint) {
        rate_limiter_.wait();

        for (int retry = 0; retry < config_.retry_count; ++retry) {
            auto conn = connection_pool_.acquire(endpoint);
            if (!conn || !conn->handle) {
                return create_error_response<T>("Failed to acquire CURL handle", 500);
            }

            initialize_handle(conn->handle);
            auto result = operation(conn->handle);
            connection_pool_.release(conn);

            if (std::get<1>(result) != 408 && std::get<1>(result) != 500) {
                return result;
            }

            if (retry < config_.retry_count - 1) {
                std::this_thread::sleep_for(config_.retry_delay * (retry + 1));
            }
        }

        return create_error_response<T>("All retry attempts failed", 500);
    }

private:
    RequestConfig config_;
    FastRateLimiter rate_limiter_;
    ConnectionPool connection_pool_;

    template<typename T>
    T create_error_response(const std::string& message, int code) {
        if constexpr (std::is_same_v<T, std::pair<nlohmann::json, int>>) {
            return std::make_pair(nlohmann::json{{"error", message}}, code);
        } else {
            return std::make_pair(message, code);
        }
    }

    void initialize_handle(CURL* handle) {
        if (!handle) return;
        
        if (config_.enable_http2) {
            curl_easy_setopt(handle, CURLOPT_HTTP_VERSION, CURL_HTTP_VERSION_2_0);
        }
        
        // 에러 버퍼 추가
        // static thread_local char error_buffer[CURL_ERROR_SIZE];
        // curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, error_buffer);
        // curl_easy_setopt(handle, CURLOPT_VERBOSE, 1L);  // 디버깅을 위해 추가

        curl_easy_setopt(handle, CURLOPT_TIMEOUT_MS, config_.timeout_ms);
        curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT_MS, config_.connection_timeout_ms);
        curl_easy_setopt(handle, CURLOPT_NOSIGNAL, 1L);
        curl_easy_setopt(handle, CURLOPT_TCP_NODELAY, 1L);
        curl_easy_setopt(handle, CURLOPT_TCP_FASTOPEN, 1L);
        
        curl_easy_setopt(handle, CURLOPT_TCP_KEEPALIVE, 1L);
        curl_easy_setopt(handle, CURLOPT_TCP_KEEPIDLE, 60L);
        curl_easy_setopt(handle, CURLOPT_TCP_KEEPINTVL, 30L);
        
        if (config_.enable_compression) {
            curl_easy_setopt(handle, CURLOPT_ACCEPT_ENCODING, "");
        }
        
        curl_easy_setopt(handle, CURLOPT_PIPEWAIT, 1L);
        curl_easy_setopt(handle, CURLOPT_FORBID_REUSE, 0L);
        curl_easy_setopt(handle, CURLOPT_FRESH_CONNECT, 0L);
    }
};