#pragma once

#include <string>
#include <chrono>
#include <nlohmann/json.hpp>

class RequestLib
{
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
    std::string host_url;
    std::chrono::seconds timeout;
    static constexpr int num_calls_per_second = 5;

    template<typename Func>
    auto rate_limited(Func&& f) -> decltype(f());

    // URL 생성 함수
    std::string create_url(const std::string& endpoint, const std::string& params) {
        std::string url = host_url + endpoint;
        if (!params.empty()) {
            url += "?" + params;
        }
        return url;
    }

};