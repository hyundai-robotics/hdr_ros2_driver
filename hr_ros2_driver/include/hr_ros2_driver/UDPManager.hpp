#ifndef HR_ROS2_DRIVER_UDPMANAGER_HPP_
#define HR_ROS2_DRIVER_UDPMANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/asio/strand.hpp>
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <chrono>

class UDPManager
{
public:
    using receive_callback_t = std::function<void(const std::string &)>;

    // Constructor to initialize UDPManager
    UDPManager(rclcpp::Node *node);

    // Destructor to clean up resources
    ~UDPManager();

    // Start the UDPManager by launching the IO thread
    void start();

    // Stop the UDPManager and clean up the IO thread
    void stop();

    // Start receiving data asynchronously
    void start_receive(receive_callback_t callback);

    // Send data asynchronously as a string
    void send_data(const std::string &data);

    // Send data asynchronously after converting a vector of doubles to JSON
    void send_data(const std::vector<double> &data);

private:
    // Run the IO context in a loop
    void run_io_context();

    // Set up the UDP connection by declaring parameters and initializing sockets
    void setup_udp_connection();

    // Declare and get the UDP parameters from the parameter server
    void declare_and_get_parameters();

    // Set up the UDP sockets for receiving and sending data
    void setup_sockets();

    // Handle received data and call the provided callback function
    void handle_receive(const boost::system::error_code &error,
                        std::size_t bytes_transferred,
                        receive_callback_t callback);

    // Helper function to send data asynchronously
    void async_send(const std::string &data);

    // Check the network status by resolving the remote host
    void check_network_status();

    rclcpp::Node *node_;
    boost::asio::io_context io_context_;
    boost::asio::strand<boost::asio::io_context::executor_type> strand_;
    std::unique_ptr<boost::asio::ip::udp::socket> recv_socket_;
    std::unique_ptr<boost::asio::ip::udp::socket> send_socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    std::vector<char> recv_buffer_;
    std::thread io_thread_;
    std::atomic<bool> is_running_;
    std::string remote_host_;
    std::string local_host_;
    int remote_port_;
    int local_port_;
    rclcpp::TimerBase::SharedPtr network_check_timer_;
};

#endif // HR_ROS2_DRIVER_UDPMANAGER_HPP_