#include <rclcpp/rclcpp.hpp>
#include "rcu_spi_comm/msg/motor_servo.hpp"
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

// Define SPI device settings
#define SPI_DEVICE "/dev/spidev0.0"  // SPI device path
#define SPI_MODE SPI_MODE_0          // SPI mode setting
#define SPI_BITS_PER_WORD 8         // Number of bits per word in SPI communication
#define SPI_SPEED 50000             // SPI communication speed

// Create a node class for SPI communication
class SpiCommNode : public rclcpp::Node {
public:
    // Constructor: Initialize SPI and create a subscription to listen to motor/servo data
    SpiCommNode() : Node("spi_comm_node") {
        init_spi();  // Initialize SPI communication
        subscription_ = this->create_subscription<rcu_spi_comm::msg::MotorServo>(
            "motor_servo_data", 10,  // Subscribe to the "motor_servo_data" topic
            std::bind(&SpiCommNode::callback, this, std::placeholders::_1));  // Callback for incoming messages
    }

    // Destructor: Close the SPI device when the node is destroyed
    ~SpiCommNode() {
        if (spi_fd_ >= 0) {
            close(spi_fd_);  // Close the SPI device file
        }
    }

private:
    int spi_fd_;  // File descriptor for the SPI device
    rclcpp::Subscription<rcu_spi_comm::msg::MotorServo>::SharedPtr subscription_;  // Subscription pointer
    rcu_spi_comm::msg::MotorServo::SharedPtr last_msg_;  // Pointer to the last received message

    // Function to initialize the SPI communication settings
    void init_spi() {
        spi_fd_ = open(SPI_DEVICE, O_RDWR);  // Open the SPI device
        if (spi_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open SPI device");  // Log an error if the device cannot be opened
            rclcpp::shutdown();  // Shutdown the node
            return;
        }

        uint8_t mode = SPI_MODE;
        uint8_t bits = SPI_BITS_PER_WORD;
        uint32_t speed = SPI_SPEED;

        ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);  // Set the SPI mode
        ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);  // Set the bits per word
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);  // Set the SPI speed
    }

    // Callback function for incoming messages
    void callback(const rcu_spi_comm::msg::MotorServo::SharedPtr msg) {
        last_msg_ = msg;  // Store the received message
        send_spi();  // Call the function to send data over SPI
    }

    // Function to send data over SPI
    void send_spi() {
        if (!last_msg_) return;  // If no message has been received, do nothing

        const int data_length = 21;  // Length of the data to be transmitted
        uint8_t tx_buffer[data_length] = {0};  // Transmission buffer
        uint8_t rx_buffer[data_length] = {0};  // Reception buffer

        tx_buffer[0] = 0xAA;  // Start byte of the transmission

        // Fill the transmission buffer with servo values
        tx_buffer[1] = static_cast<uint8_t>(last_msg_->servos[0]);
        for (int i = 1; i < 6; i++) {
            tx_buffer[1 + i] = static_cast<uint8_t>(last_msg_->servos[i]);
        }

        // Fill the transmission buffer with motor values (split into high and low bytes)
        for (int i = 0; i < 6; i++) {
            tx_buffer[7 + i * 2] = last_msg_->motors[i] & 0xFF;  // Low byte of motor value
            tx_buffer[8 + i * 2] = (last_msg_->motors[i] >> 8) & 0xFF;  // High byte of motor value
        }

        // Calculate and add checksum to the buffer
        uint8_t checksum = 0;
        for (int i = 1; i <= 18; i++) {
            checksum ^= tx_buffer[i];
        }
        tx_buffer[19] = checksum;  // Add checksum
        tx_buffer[20] = 0xBB;  // End byte of the transmission

        // Prepare the SPI transfer structure
        struct spi_ioc_transfer spi_transfer{};
        spi_transfer.tx_buf = reinterpret_cast<__u64>(tx_buffer);  // Set the transmission buffer
        spi_transfer.rx_buf = reinterpret_cast<__u64>(rx_buffer);  // Set the reception buffer
        spi_transfer.len = data_length;  // Set the data length
        spi_transfer.speed_hz = SPI_SPEED;  // Set the SPI speed
        spi_transfer.bits_per_word = SPI_BITS_PER_WORD;  // Set the bits per word

        // Perform the SPI transfer
        int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &spi_transfer);
        if (ret < 0) {
            RCLCPP_ERROR(this->get_logger(), "SPI transfer failed");  // Log an error if the transfer fails
        } else {
            uint8_t response = rx_buffer[0];  // Get the response byte
            if (response == 0x31 || response == 0x21) {
                RCLCPP_INFO(this->get_logger(), "SPI ACK received: (success)");  // Log a success message if response is valid
            } else {
                RCLCPP_WARN(this->get_logger(), "Unexpected SPI response: 0x%02X", response);  // Log a warning for unexpected response
            }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(10));  // Sleep for a short time before the next transfer
    }
};

// Main function: Initialize the ROS 2 node and start spinning
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<SpiCommNode>());  // Spin the node to process callbacks
    rclcpp::shutdown();  // Shutdown the ROS 2 system
    return 0;  // Return success
}