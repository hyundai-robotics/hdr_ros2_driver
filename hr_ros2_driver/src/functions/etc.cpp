#include "hr_ros2_driver/functions.hpp"
#include <iostream>
#include <nlohmann/json.hpp>

namespace etc {

void etc_date_time(const std::shared_ptr<RequestLib>& client,
                   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, status_code] = client->get("/clock/date_time");
    std::cout << "Clock date Time: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void etc_put_date_time(const std::shared_ptr<RequestLib>& client,
                       const std::shared_ptr<api_msgs::srv::DateTime::Request> request,
                       std::shared_ptr<api_msgs::srv::DateTime::Response> response)
{
    try {
        // 유효한 날짜/시간인지 확인
        std::tm tm = {};
        tm.tm_year = request->year - 1900;  // years since 1900
        tm.tm_mon = request->mon - 1;       // months since January (0-11)
        tm.tm_mday = request->day;
        tm.tm_hour = request->hour;
        tm.tm_min = request->min;
        tm.tm_sec = request->sec;

        std::mktime(&tm);  // 이 함수는 날짜/시간을 정규화하고 유효성을 검사합니다

        if (tm.tm_year != request->year - 1900 || tm.tm_mon != request->mon - 1 || 
            tm.tm_mday != request->day || tm.tm_hour != request->hour || 
            tm.tm_min != request->min || tm.tm_sec != request->sec) {
            throw std::runtime_error("Invalid date/time");
        }

        nlohmann::json param = {
            {"year", request->year},
            {"mon", request->mon},
            {"day", request->day},
            {"hour", request->hour},
            {"min", request->min},
            {"sec", request->sec}
        };

        auto [_, status_code] = client->put("/clock/date_time", "", param);

        if (status_code == 200) {
            response->success = true;
            response->message = std::to_string(status_code);
        } else {
            response->success = false;
            response->message = std::to_string(status_code);
        }
    }
    catch (const std::exception& e) {
        response->success = false;
        response->message = "Date Time Error: " + std::string(e.what());
    }
}

void etc_get_log_manager(const std::shared_ptr<RequestLib>& client,
                         const std::shared_ptr<api_msgs::srv::LogManager::Request> request,
                         std::shared_ptr<api_msgs::srv::LogManager::Response> response)
{
    std::vector<std::string> params;

    // n_item 체크 및 추가
    if (request->n_item <= 0) {
        response->success = false;
        response->message = "Invalid n_item value: must be positive";
        return;
    }
    if (request->n_item != 100) {
       params.push_back("n_item=" + std::to_string(request->n_item));
    }

    // cat_p 체크 및 추가
    if (!request->cat_p.empty()) {
        // 허용된 카테고리 문자들
        std::string valid_categories = "EWNSOIPHCM";
        bool valid = true;
        
        // 쉼표로 분리하여 각 카테고리 검사
        std::istringstream ss(request->cat_p);
        std::string category;
        while (std::getline(ss, category, ',')) {
            if (category.length() != 1 || valid_categories.find(category) == std::string::npos) {
                valid = false;
                break;
            }
        }
        
        if (!valid) {
            response->success = false;
            response->message = "Invalid cat_p format: must be combination of E,W,N,S,O,I,P,H,C,M";
            return;
        }
        params.push_back("cat_p=" + request->cat_p);
    }

    // id_min 체크 및 추가 
    if (request->id_min < 0 || request->id_min > 0xFFFFFFFFFFFFFFFF) {
        response->success = false;
        response->message = "Invalid id_min: value must be between 0 and 0xffffffffffffffff";
        return;
    }

    if (request->id_min != 0) {  // 0이 아닐 때만 파라미터에 추가
        params.push_back("id_min=" + std::to_string(request->id_min));
    }

    // id_max 체크 및 추가
    if (request->id_max < 0 || request->id_max > 0xFFFFFFFFFFFFFFFF) {
        response->success = false;
        response->message = "Invalid id_max: value must be between 0 and 0xffffffffffffffff";
        return;
    }

    if (request->id_max != 0) {  // 0이 아닐 때만 파라미터에 추가
        params.push_back("id_max=" + std::to_string(request->id_max));
    }

    // ts_min 체크 및 추가
    if (!request->ts_min.empty()) {
        // 타임스탬프 형식 검증 (YYYY/MM/DD HH:mm:ss.SSS)
        std::regex timestamp_pattern(R"(\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d{3})");
        if (!std::regex_match(request->ts_min, timestamp_pattern)) {
            response->success = false;
            response->message = "Invalid ts_min format: must be YYYY/MM/DD HH:mm:ss.SSS";
            return;
        }
        params.push_back("ts_min=" + request->ts_min);
    }
    
    // ts_max 체크 및 추가
    if (!request->ts_max.empty()) {
        // 타임스탬프 형식 검증 (YYYY/MM/DD HH:mm:ss.SSS)
        std::regex timestamp_pattern(R"(\d{4}/\d{2}/\d{2} \d{2}:\d{2}:\d{2}\.\d{3})");
        if (!std::regex_match(request->ts_max, timestamp_pattern)) {
            response->success = false;
            response->message = "Invalid ts_max format: must be YYYY/MM/DD HH:mm:ss.SSS";
            return;
        }
        params.push_back("ts_max=" + request->ts_max);
    }
    
    // 파라미터들을 &로 연결
    std::string query_params;
    for (size_t i = 0; i < params.size(); ++i) {
        if (i > 0) query_params += "&";
        query_params += params[i];
    }

    auto [posts, status_code] = client->get_str("/logManager/search", query_params);
    std::cout << "logManager : " << posts << std::endl;

    response->success = (status_code == 200);
    response->message = posts;
}

}