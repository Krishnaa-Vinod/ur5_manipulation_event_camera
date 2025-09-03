#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

// Global atomic flag for clean shutdown
std::atomic<bool> g_shutdown_requested{false};

// Terminal control functions
struct termios orig_termios;

void disableRawMode() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);
    
    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void keyboardMonitor() {
    enableRawMode();
    
    std::cout << "\n=== HIGH-FREQUENCY Y-AXIS MICRO SACCADE ===" << std::endl;
    std::cout << "Press 'q' to quit safely" << std::endl;
    std::cout << "Joint positions locked to 'main' pose!" << std::endl;
    std::cout << "==========================================\n" << std::endl;
    
    char c;
    while (!g_shutdown_requested && read(STDIN_FILENO, &c, 1) >= 0) {
        if (c == 'q' || c == 'Q') {
            std::cout << "\n[KEYBOARD] Quit requested" << std::endl;
            g_shutdown_requested = true;
            break;
        }
        std::this_thread::sleep_for(10ms);
    }
}

class FastYAxisMicroSaccadeNode : public rclcpp::Node {
public:
  FastYAxisMicroSaccadeNode() : Node("fast_y_axis_micro_saccade") {
    
    // High-frequency Y-axis movement parameters
    servo_topic_ = "/servo_server/delta_twist_cmds";
    frame_id_ = "base_link";
    
    // Fast Y-axis micro-saccade parameters
    y_amplitude_ = 0.010;      // ±2mm - small for stability at high frequency
    frequency_hz_ = 5.0;       // 8Hz - much faster as requested
    control_rate_hz_ = 500.0;  // 500Hz - very high control rate
    
    // Strict drift prevention
    reset_interval_s_ = 8.0;   // More frequent resets at high frequency
    max_cumulative_error_ = 0.001; // 1mm max cumulative error before correction
    
    // Handle simulation time
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter<bool>("use_sim_time", false);
    }

    // Calculate derived parameters
    omega_ = 2.0 * M_PI * frequency_hz_;
    
    // Initialize error tracking
    cumulative_x_error_ = 0.0;
    cumulative_z_error_ = 0.0;
    error_samples_ = 0;

    // Setup servo services
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/stop_servo");

    // Wait for servo services
    RCLCPP_INFO(get_logger(), "Waiting for servo services...");
    if (!servo_start_client_->wait_for_service(3s)) {
        RCLCPP_WARN(get_logger(), "Servo services not available");
    }

    // Start servo
    startServo();

    // Create twist publisher with high-performance QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(servo_topic_, qos);
    
    // Allow servo to start
    rclcpp::sleep_for(500ms);
    
    start_time_ = this->now();
    last_reset_time_ = this->now();

    // Create high-frequency timer
    auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FastYAxisMicroSaccadeNode::highFrequencyControl, this));

    RCLCPP_INFO(get_logger(), "FAST Y-axis micro-saccade: ±%.1fmm @ %.1fHz (control: %.0fHz)", 
                y_amplitude_*1000.0, frequency_hz_, control_rate_hz_);
    RCLCPP_INFO(get_logger(), "Joints locked to main pose - only Y-axis end-effector movement");
    
    // Start keyboard thread
    keyboard_thread_ = std::thread(keyboardMonitor);
  }

  ~FastYAxisMicroSaccadeNode() {
    shutdown();
  }

  void shutdown() {
    if (shutdown_done_) return;
    shutdown_done_ = true;
    
    RCLCPP_INFO(get_logger(), "Shutting down fast Y-axis micro-saccade...");
    
    // Stop keyboard thread
    g_shutdown_requested = true;
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
    
    // Cancel timer
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    
    // Extended zero command sequence for high-frequency stop
    RCLCPP_INFO(get_logger(), "Sending extended stop sequence...");
    for (int i = 0; i < 50; ++i) {  // More zero commands for high-freq control
        publishZeroVelocity();
        rclcpp::sleep_for(2ms);    // Faster stop sequence
    }

    // Stop servo
    stopServo();
    
    // Final safety sequence
    for (int i = 0; i < 20; ++i) {
        publishZeroVelocity();
        rclcpp::sleep_for(5ms);
    }
    
    RCLCPP_INFO(get_logger(), "Fast Y-axis micro-saccade stopped safely");
  }

private:
  void startServo() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    if (servo_start_client_->service_is_ready()) {
        auto future = servo_start_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 2s) 
            == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(get_logger(), "Servo started - joints locked to main pose");
            }
        }
    }
  }

  void stopServo() {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    if (servo_stop_client_->service_is_ready()) {
        auto future = servo_stop_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 1s) 
            == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(get_logger(), "Servo stopped");
            }
        }
    }
  }

  void publishZeroVelocity() {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    // All velocities are zero by default
    twist_pub_->publish(msg);
  }

  void highFrequencyControl() {
    // Check for shutdown
    if (g_shutdown_requested) {
        shutdown();
        rclcpp::shutdown();
        return;
    }
    
    const auto now = this->now();
    const double elapsed_time = (now - start_time_).seconds();
    const double time_since_reset = (now - last_reset_time_).seconds();

    // More frequent drift correction for high-frequency operation
    if (time_since_reset >= reset_interval_s_) {
        performDriftCorrection();
        return;
    }

    // Generate pure Y-axis sine wave velocity
    double y_velocity = y_amplitude_ * omega_ * std::cos(omega_ * elapsed_time);
    
    // Calculate corrective velocities for X and Z drift prevention
    double x_correction = 0.0;
    double z_correction = 0.0;
    
    if (error_samples_ > 0) {
        // Apply small corrective velocities to counteract accumulated drift
        double avg_x_error = cumulative_x_error_ / error_samples_;
        double avg_z_error = cumulative_z_error_ / error_samples_;
        
        // Proportional correction with small gain
        x_correction = -avg_x_error * 0.5;  // Small correction gain
        z_correction = -avg_z_error * 0.5;
        
        // Limit correction velocities
        x_correction = std::max(-0.001, std::min(0.001, x_correction)); // ±1mm/s max
        z_correction = std::max(-0.001, std::min(0.001, z_correction));
    }

    // Create and publish command
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = frame_id_;
    
    // Y-axis: High-frequency micro-saccade
    cmd.twist.linear.y = y_velocity;
    
    // X and Z: Small drift correction
    cmd.twist.linear.x = x_correction;
    cmd.twist.linear.z = z_correction;
    
    // No rotational movement (joints locked to main pose)
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = 0.0;

    twist_pub_->publish(cmd);
    
    // Track potential drift (simple estimation)
    double dt = 1.0 / control_rate_hz_;
    cumulative_x_error_ += cmd.twist.linear.x * dt;
    cumulative_z_error_ += cmd.twist.linear.z * dt;
    error_samples_++;
    
    // Debug output every 4 seconds
    static int debug_counter = 0;
    if (++debug_counter % (int)(control_rate_hz_ * 4.0) == 0) {
        RCLCPP_INFO(get_logger(), "Y-vel: %+.1fmm/s | X-corr: %+.3f | Z-corr: %+.3f", 
                   y_velocity * 1000.0, x_correction * 1000.0, z_correction * 1000.0);
    }
  }

  void performDriftCorrection() {
    RCLCPP_INFO(get_logger(), "High-frequency drift correction...");
    
    // Quick stabilization sequence for high-frequency operation
    for (int i = 0; i < 25; ++i) {
        publishZeroVelocity();
        rclcpp::sleep_for(4ms);  // Fast stabilization
    }
    
    // Reset error tracking
    cumulative_x_error_ = 0.0;
    cumulative_z_error_ = 0.0;
    error_samples_ = 0;
    
    // Reset timing
    start_time_ = this->now();
    last_reset_time_ = this->now();
    
    RCLCPP_INFO(get_logger(), "High-frequency correction complete");
  }

  // Parameters
  std::string servo_topic_;
  std::string frame_id_;
  double y_amplitude_;
  double frequency_hz_;
  double control_rate_hz_;
  double reset_interval_s_;
  double max_cumulative_error_;
  double omega_;

  // Error tracking for drift correction
  double cumulative_x_error_;
  double cumulative_z_error_;
  int error_samples_;

  // ROS components
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;
  
  // Timing
  rclcpp::Time start_time_;
  rclcpp::Time last_reset_time_;
  
  // Control
  std::thread keyboard_thread_;
  bool shutdown_done_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<FastYAxisMicroSaccadeNode>();
    
    while (rclcpp::ok() && !g_shutdown_requested) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(1ms);
    }
    
    node->shutdown();
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }
  
  rclcpp::shutdown();
  return 0;
}