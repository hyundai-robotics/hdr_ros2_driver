#include "hdr_ros2_driver/Requestlib.hpp"

static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
    if (!contents || !userp) {  // 추가 필요
        return 0;
    }
    size_t realsize = size * nmemb;
    try {
        userp->append(static_cast<char*>(contents), realsize);
    } catch(const std::bad_alloc&) {
        return 0;
    }
    return realsize;
}

RequestLib::RequestLib(const std::string& host)
    : pimpl(std::make_unique<Impl>())
    , host_url(host)
{
}

RequestLib::~RequestLib() = default;

// GET
std::pair<nlohmann::json, int> RequestLib::get(const std::string& endpoint, const std::string& params)
{
    return pimpl->execute_with_retry<std::pair<nlohmann::json, int>>(
        [this, endpoint, params](CURL* curl) {
            std::string readBuffer;
            long response_code = 0;

            std::string url = create_url(endpoint, params);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);


            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                return std::make_pair(
                    nlohmann::json{{"error", curl_easy_strerror(res)}},
                    (res == CURLE_OPERATION_TIMEDOUT) ? 408 : 500
                );
            }

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

            try {
                return std::make_pair(
                    nlohmann::json::parse(readBuffer),
                    static_cast<int>(response_code)
                );
            } catch (const nlohmann::json::parse_error& e) {
                return std::make_pair(
                    nlohmann::json{{"error", "JSON parse error: " + std::string(e.what())}},
                    500
                );
            }
        },
        endpoint  // endpoint 매개변수 추가
    );
}

// GET String
std::pair<std::string, int> RequestLib::get_str(const std::string& endpoint, const std::string& params)
{
    using ReturnType = std::pair<std::string, int>;  // 명시적 타입 선언
    return pimpl->execute_with_retry<ReturnType>(
        [this, endpoint, params](CURL* curl) -> ReturnType {  // 반환 타입 명시
            std::string readBuffer;
            long response_code = 0;

            std::string url = create_url(endpoint, params);
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                return std::make_pair(
                    std::string(curl_easy_strerror(res)),  // string으로 명시적 변환
                    (res == CURLE_OPERATION_TIMEDOUT) ? 408 : 500
                );
            }

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            return std::make_pair(std::move(readBuffer), static_cast<int>(response_code));
        },
        endpoint  // endpoint 매개변수 추가
    );
}

// POST
std::pair<nlohmann::json, int> RequestLib::post(
    const std::string& endpoint, const std::string& params, const nlohmann::json& body)
{
    return pimpl->execute_with_retry<std::pair<nlohmann::json, int>>(
        [this, endpoint, params, body](CURL* curl) {
            std::string readBuffer;
            long response_code = 0;

            std::string url = create_url(endpoint, params);
            std::string json_body = body.dump();
            
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json; charset=utf-8");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            CURLcode res = curl_easy_perform(curl);
            curl_slist_free_all(headers);

            if (res != CURLE_OK) {
                return std::make_pair(
                    nlohmann::json{{"error", curl_easy_strerror(res)}},
                    (res == CURLE_OPERATION_TIMEDOUT) ? 408 : 500
                );
            }

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

            try {
                return std::make_pair(
                    nlohmann::json::parse(readBuffer),
                    static_cast<int>(response_code)
                );
            } catch (const nlohmann::json::parse_error& e) {
                return std::make_pair(
                    nlohmann::json{{"error", "JSON parse error: " + std::string(e.what())}},
                    500
                );
            }
        },
        endpoint  // endpoint 매개변수 추가
    );
}

// POST File
std::pair<nlohmann::json, int> RequestLib::post_file(
    const std::string& endpoint, const std::string& params, const std::string& file_content)
{
    return pimpl->execute_with_retry<std::pair<nlohmann::json, int>>(
        [this, endpoint, params, file_content](CURL* curl) {
            std::string readBuffer;
            long response_code = 0;

            std::string url = create_url(endpoint, params);
            
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, file_content.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, file_content.size());
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/octet-stream");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            CURLcode res = curl_easy_perform(curl);
            curl_slist_free_all(headers);

            if (res != CURLE_OK) {
                return std::make_pair(
                    nlohmann::json{{"error", curl_easy_strerror(res)}},
                    (res == CURLE_OPERATION_TIMEDOUT) ? 408 : 500
                );
            }

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

            try {
                return std::make_pair(
                    nlohmann::json::parse(readBuffer),
                    static_cast<int>(response_code)
                );
            } catch (const nlohmann::json::parse_error& e) {
                return std::make_pair(
                    nlohmann::json{{"error", "JSON parse error: " + std::string(e.what())}},
                    500
                );
            }
        },
        endpoint  // endpoint 매개변수 추가
    );
}

// PUT
std::pair<nlohmann::json, int> RequestLib::put(
    const std::string& endpoint, const std::string& params, const nlohmann::json& body)
{
    return pimpl->execute_with_retry<std::pair<nlohmann::json, int>>(
        [this, endpoint, params, body](CURL* curl) {
            std::string readBuffer;
            long response_code = 0;

            std::string url = create_url(endpoint, params);
            std::string json_body = body.dump();
            
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json; charset=utf-8");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            CURLcode res = curl_easy_perform(curl);
            curl_slist_free_all(headers);

            if (res != CURLE_OK) {
                return std::make_pair(
                    nlohmann::json{{"error", curl_easy_strerror(res)}},
                    (res == CURLE_OPERATION_TIMEDOUT) ? 408 : 500
                );
            }

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

            try {
                return std::make_pair(
                    nlohmann::json::parse(readBuffer),
                    static_cast<int>(response_code)
                );
            } catch (const nlohmann::json::parse_error& e) {
                return std::make_pair(
                    nlohmann::json{{"error", "JSON parse error: " + std::string(e.what())}},
                    500
                );
            }
        },
        endpoint  // endpoint 매개변수 추가
    );
}

// DELETE
std::pair<std::string, int> RequestLib::del(const std::string& endpoint, const std::string& params)
{
    using ReturnType = std::pair<std::string, int>;  // 명시적 타입 선언
    return pimpl->execute_with_retry<ReturnType>(
        [this, endpoint, params](CURL* curl) -> ReturnType {  // 반환 타입 명시
            std::string readBuffer;
            long response_code = 0;

            std::string url = create_url(endpoint, params);
            
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                return std::make_pair(
                    std::string(curl_easy_strerror(res)),  // string으로 명시적 변환
                    (res == CURLE_OPERATION_TIMEDOUT) ? 408 : 500
                );
            }

            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            return std::make_pair(std::move(readBuffer), static_cast<int>(response_code));
        },
        endpoint  // endpoint 매개변수 추가
    );
}