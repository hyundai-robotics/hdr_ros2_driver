#include "hr_ros2_driver/functions.hpp"
#include <iostream>

namespace version {

void api_version(const std::shared_ptr<RequestLib>& client,
                 const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, status_code] = client->get("/api_ver");

    // C++에서는 pprint 대신 일반 출력을 사용합니다.
    std::cout << "API Version: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

void sysver_version(const std::shared_ptr<RequestLib>& client,
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    auto [posts, status_code] = client->get("/versions/sysver");

    std::cout << "System Version: " << posts.dump(4) << std::endl;

    response->success = true;
    response->message = posts.dump();
}

} // namespace version