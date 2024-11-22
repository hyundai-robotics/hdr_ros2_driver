#include "hr_ros2_driver/UDPManager.hpp"

// Constructor to initialize UDPManager
UDPManager::UDPManager(rclcpp::Node *node)
    : node_(node), io_context_(), strand_(boost::asio::make_strand(io_context_)), recv_buffer_(262144), is_running_(false)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing UDPManager");
    setup_udp_connection();
}

// Destructor to clean up resources
UDPManager::~UDPManager()
{
    stop();
    RCLCPP_INFO(node_->get_logger(), "UDPManager destroyed, all resources cleaned up.");
}

// Start the UDPManager by launching the IO thread
void UDPManager::start()
{
    if (!is_running_.exchange(true))
    {
        io_thread_ = std::thread(&UDPManager::run_io_context, this);
        RCLCPP_INFO(node_->get_logger(), "UDPManager thread started");
    }
}

// Stop the UDPManager and clean up the IO thread
void UDPManager::stop()
{
    if (is_running_.exchange(false))
    {
        io_context_.stop();
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
        RCLCPP_INFO(node_->get_logger(), "UDPManager thread stopped");
    }
}

// Run the IO context in a loop
void UDPManager::run_io_context()
{
    RCLCPP_INFO(node_->get_logger(), "Starting io_context run loop");
    while (is_running_)
    {
        try
        {
            io_context_.run();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error in io_context run loop: %s", e.what());
        }

        if (is_running_)
        {
            io_context_.restart();
        }
    }
}

// Set up the UDP connection by declaring parameters and initializing sockets
void UDPManager::setup_udp_connection()
{
    try
    {
        declare_and_get_parameters();
        setup_sockets();

        // Set up network status check timer
        network_check_timer_ = node_->create_wall_timer(std::chrono::seconds(60), std::bind(&UDPManager::check_network_status, this));
        RCLCPP_INFO(node_->get_logger(), "UDP connection setup completed successfully");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize UDP: %s", e.what());
        throw;
    }
}

// Declare and get the UDP parameters from the parameter server
void UDPManager::declare_and_get_parameters()
{
    node_->declare_parameter<std::string>("udp_remote_host", "192.168.1.150");
    node_->declare_parameter<std::string>("udp_local_host", "0.0.0.0");
    node_->declare_parameter<int>("udp_remote_port", 7000);
    node_->declare_parameter<int>("udp_local_port", 7001);

    remote_host_ = node_->get_parameter("udp_remote_host").as_string();
    local_host_ = node_->get_parameter("udp_local_host").as_string();
    remote_port_ = node_->get_parameter("udp_remote_port").as_int();
    local_port_ = node_->get_parameter("udp_local_port").as_int();

    RCLCPP_INFO(node_->get_logger(), "UDP parameters: remote_host=%s, remote_port=%d, local_host=%s, local_port=%d",
                remote_host_.c_str(), remote_port_, local_host_.c_str(), local_port_);
}

// Set up the UDP sockets for receiving and sending data
void UDPManager::setup_sockets()
{
    try
    {
        // Set up the receiving socket
        recv_socket_ = std::make_unique<boost::asio::ip::udp::socket>(strand_,
                                                                      boost::asio::ip::udp::endpoint(boost::asio::ip::make_address(local_host_), local_port_));
        recv_socket_->set_option(boost::asio::socket_base::reuse_address(true));
        recv_socket_->set_option(boost::asio::socket_base::receive_buffer_size(262144));

        // Set up the sending socket
        send_socket_ = std::make_unique<boost::asio::ip::udp::socket>(strand_);
        send_socket_->open(boost::asio::ip::udp::v4());
        send_socket_->set_option(boost::asio::socket_base::send_buffer_size(262144));

        RCLCPP_INFO(node_->get_logger(), "Sockets set up successfully");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Socket setup error: %s", e.what());
        throw;
    }
}

// Start receiving data asynchronously
void UDPManager::start_receive(receive_callback_t callback)
{
    recv_socket_->async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::asio::bind_executor(strand_,
                                   [this, callback](const boost::system::error_code &error, std::size_t bytes_transferred)
                                   {
                                       handle_receive(error, bytes_transferred, callback);
                                   }));
}

// Handle received data and call the provided callback function
void UDPManager::handle_receive(const boost::system::error_code &error,
                                std::size_t bytes_transferred,
                                receive_callback_t callback)
{
    if (!error)
    {
        if (bytes_transferred > 0)
        {
            std::string received_data(recv_buffer_.data(), bytes_transferred);
            // RCLCPP_INFO(node_->get_logger(), "Received: %s", received_data.c_str());

            // Pass received data to the callback function
            callback(received_data);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Received empty packet");
        }
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Receive error: %s", error.message().c_str());
        setup_sockets(); // Re-initialize socket on error
    }

    // Clear the buffer before the next receive
    std::fill(recv_buffer_.begin(), recv_buffer_.end(), 0);

    // Continue to listen for incoming packets
    start_receive(callback);
}

// Send data asynchronously as a string
void UDPManager::send_data(const std::string &data)
{
    async_send(data);
}

// Send data asynchronously after converting a vector of doubles to JSON
void UDPManager::send_data(const std::vector<double> &data)
{
    nlohmann::json j = data;
    async_send(j.dump());
}

// Helper function to send data asynchronously
void UDPManager::async_send(const std::string &data)
{
    send_socket_->async_send_to(
        boost::asio::buffer(data), remote_endpoint_,
        boost::asio::bind_executor(strand_,
                                   [this](boost::system::error_code ec, std::size_t bytes_sent)
                                   {
                                       if (!ec)
                                       {
                                        //    RCLCPP_INFO(node_->get_logger(), "Data sent successfully, %zu bytes", bytes_sent);
                                       }
                                       else
                                       {
                                           RCLCPP_ERROR(node_->get_logger(), "Send error: %s", ec.message().c_str());
                                       }
                                   }));
}

// Check the network status by resolving the remote host
void UDPManager::check_network_status()
{
    RCLCPP_DEBUG(node_->get_logger(), "Checking network status");
    try
    {
        boost::asio::ip::udp::resolver resolver(io_context_);
        boost::asio::ip::udp::resolver::results_type endpoints =
            resolver.resolve(boost::asio::ip::udp::v4(), remote_host_, std::to_string(remote_port_));

        if (endpoints.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to resolve remote host");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Network status: OK");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Network status check failed: %s", e.what());
    }
}