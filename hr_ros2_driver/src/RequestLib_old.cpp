#include "hr_ros2_driver/Requestlib.hpp"
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>

static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* s)
{
    size_t newLength = size * nmemb;
    try {
        s->append((char*)contents, newLength);
    } catch(std::bad_alloc &e) {
        return 0;
    }
    return newLength;
}

RequestLib::RequestLib(const std::string& host)
    : host_url(host), timeout(std::chrono::seconds(5))
{
    curl_global_init(CURL_GLOBAL_DEFAULT);
}

RequestLib::~RequestLib()
{
    curl_global_cleanup();
}

std::pair<nlohmann::json, int> RequestLib::get(const std::string& endpoint, const std::string& params)
{
    return rate_limited([&]() {
        CURL* curl = curl_easy_init();
        std::string readBuffer;
        long response_code = 0;
        std::string error_message;

        if(curl) {
            std::string url = create_url(endpoint, params);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout.count());

            CURLcode res = curl_easy_perform(curl);
            if(res != CURLE_OK) {
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    error_message = "Operation timed out";
                    response_code = 408;  // HTTP status code for Request Timeout
                } else {
                    error_message = curl_easy_strerror(res);
                    response_code = 500;  // Internal Server Error for other CURL errors
                }
            } else {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            }

            curl_easy_cleanup(curl);
        } else {
            error_message = "Failed to initialize CURL";
            response_code = 500;
        }

        if (!error_message.empty()) {
            return std::make_pair(nlohmann::json({{"error", error_message}}), static_cast<int>(response_code));
        }

        try {
            return std::make_pair(nlohmann::json::parse(readBuffer), static_cast<int>(response_code));
        } catch (nlohmann::json::parse_error& e) {
            return std::make_pair(nlohmann::json({{"error", "JSON parse error: " + std::string(e.what())}}), 408);
        }
    });
}

std::pair<std::string, int> RequestLib::get_str(const std::string& endpoint, const std::string& params)
{
    return rate_limited([&]() {
        CURL* curl = curl_easy_init();
        std::string readBuffer;
        long response_code = 0;
        std::string error_message;

        if(curl) {
            std::string url = create_url(endpoint, params);
           
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout.count());

            // std::cout << "URL: " << url << std::endl;  // URL 출력 추가

            CURLcode res = curl_easy_perform(curl);
            if(res != CURLE_OK) {
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    error_message = "Operation timed out";
                    response_code = 408;  // HTTP status code for Request Timeout
                } else {
                    error_message = curl_easy_strerror(res);
                    response_code = 500;  // Internal Server Error for other CURL errors
                }
            } else {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            }

            curl_easy_cleanup(curl);
        } else {
            error_message = "Failed to initialize CURL";
            response_code = 500;
        }

        std::cout << "Response: " << readBuffer << std::endl;  // 응답 출력 추가

        if (!error_message.empty()) {
            return std::make_pair(error_message, static_cast<int>(response_code));
        }

        return std::make_pair(readBuffer, static_cast<int>(response_code));
    });
}

std::pair<nlohmann::json, int> RequestLib::post(const std::string& endpoint, const std::string& params, const nlohmann::json& body)
{
    return rate_limited([&]() {
        CURL* curl = curl_easy_init();
        std::string readBuffer;
        long response_code = 0;
        std::string error_message;

        if(curl) {
            std::string url = create_url(endpoint, params);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            
            std::string json_body = body.dump();
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());

            //std::cout << "URL: " << url.c_str() << std::endl;
            //std::cout << "Body: " << json_body.c_str() << std::endl;
            
            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json; charset=utf-8");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout.count());

            CURLcode res = curl_easy_perform(curl);
            if(res != CURLE_OK) {
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    error_message = "Operation timed out";
                    response_code = 408;  // HTTP status code for Request Timeout
                } else {
                    error_message = curl_easy_strerror(res);
                    response_code = 500;  // Internal Server Error for other CURL errors
                }
            } else {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        } else {
            error_message = "Failed to initialize CURL";
            response_code = 500;
        }

        if (!error_message.empty()) {
            return std::make_pair(nlohmann::json({{"error", error_message}}), static_cast<int>(response_code));
        }

        try {
            return std::make_pair(nlohmann::json::parse(readBuffer), static_cast<int>(response_code));
        } catch (nlohmann::json::parse_error& e) {
            return std::make_pair(nlohmann::json({{"error", "JSON parse error: " + std::string(e.what())}}), 408);
        }
    });
}

std::pair<nlohmann::json, int> RequestLib::post_file(const std::string& endpoint, const std::string& params, const std::string& file_content)
{
    return rate_limited([&]() {
        CURL* curl = curl_easy_init();
        std::string readBuffer;
        long response_code = 0;
        std::string error_message;

        if(curl) {
            std::string url = create_url(endpoint, params);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, file_content.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, file_content.size());
            
            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/octet-stream");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout.count());

            CURLcode res = curl_easy_perform(curl);
            if(res != CURLE_OK) {
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    error_message = "Operation timed out";
                    response_code = 408;  // HTTP status code for Request Timeout
                } else {
                    error_message = curl_easy_strerror(res);
                    response_code = 500;  // Internal Server Error for other CURL errors
                }
            } else {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        } else {
            error_message = "Failed to initialize CURL";
            response_code = 500;
        }

        if (!error_message.empty()) {
            return std::make_pair(nlohmann::json({{"error", error_message}}), static_cast<int>(response_code));
        }

        try {
            return std::make_pair(nlohmann::json::parse(readBuffer), static_cast<int>(response_code));
        } catch (nlohmann::json::parse_error& e) {
            return std::make_pair(nlohmann::json({{"error", "JSON parse error: " + std::string(e.what())}}), 408);
        }
    });
}

std::pair<nlohmann::json, int> RequestLib::put(const std::string& endpoint, const std::string& params, const nlohmann::json& body)
{
    return rate_limited([&]() {
        CURL* curl = curl_easy_init();
        std::string readBuffer;
        long response_code = 0;
        std::string error_message;

        if(curl) {
            std::string url = create_url(endpoint, params);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
            
            std::string json_body = body.dump();
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());

            struct curl_slist *headers = NULL;
            headers = curl_slist_append(headers, "Content-Type: application/json; charset=utf-8");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout.count());

            //std::cout << "URL: " << url.c_str() << std::endl;
            //std::cout << "Body: " << json_body.c_str() << std::endl;

            CURLcode res = curl_easy_perform(curl);
            if(res != CURLE_OK) {
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    error_message = "Operation timed out";
                    response_code = 408;  // HTTP status code for Request Timeout
                } else {
                    error_message = curl_easy_strerror(res);
                    response_code = 500;  // Internal Server Error for other CURL errors
                }
            } else {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        } else {
            error_message = "Failed to initialize CURL";
            response_code = 500;
        }

        if (!error_message.empty()) {
            return std::make_pair(nlohmann::json({{"error", error_message}}), static_cast<int>(response_code));
        }

        try {
            return std::make_pair(nlohmann::json::parse(readBuffer), static_cast<int>(response_code));
        } catch (nlohmann::json::parse_error& e) {
            return std::make_pair(nlohmann::json({{"error", "JSON parse error: " + std::string(e.what())}}), 408);
        }
    });
}

std::pair<std::string, int> RequestLib::del(const std::string& endpoint, const std::string& params)
{
    return rate_limited([&]() {
        CURL* curl = curl_easy_init();
        std::string readBuffer;
        long response_code = 0;
        std::string error_message;

        if(curl) {
            std::string url = create_url(endpoint, params);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout.count());

            //std::cout << "URL: " << url.c_str() << std::endl;
            
            CURLcode res = curl_easy_perform(curl);
            if(res != CURLE_OK) {
                if (res == CURLE_OPERATION_TIMEDOUT) {
                    error_message = "Operation timed out";
                    response_code = 408;  // HTTP status code for Request Timeout
                } else {
                    error_message = curl_easy_strerror(res);
                    response_code = 500;  // Internal Server Error for other CURL errors
                }
            } else {
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            }

            curl_easy_cleanup(curl);
        } else {
            error_message = "Failed to initialize CURL";
            response_code = 500;
        }

        //std::cout << "Response: " << readBuffer << std::endl;  // 응답 출력 추가
        
        if (!error_message.empty()) {
            return std::make_pair(error_message, static_cast<int>(response_code));
        }
        
        return std::make_pair(readBuffer, static_cast<int>(response_code));
    });
}

template<typename Func>
auto RequestLib::rate_limited(Func&& f) -> decltype(f())
{
    static auto last_call = std::chrono::steady_clock::now();
    static std::mutex mtx;

    std::lock_guard<std::mutex> lock(mtx);
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call);
    auto min_interval = std::chrono::milliseconds(1000 / num_calls_per_second);

    if (duration < min_interval) {
        std::this_thread::sleep_for(min_interval - duration);
    }

    last_call = std::chrono::steady_clock::now();
    return f();
}

