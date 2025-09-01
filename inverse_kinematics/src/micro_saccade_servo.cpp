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

// Function to set terminal to non-blocking mode
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

// Keyboard monitoring function
void keyboardMonitor() {
    enableRawMode();
    
    std::cout << "\n=== MICRO SACCADE CONTROL ===" << std::endl;
    std::cout << "Press 'q' to quit safely" << std::endl;
    std::cout << "Press 'p' to pause/resume" << std::endl;
    std::cout << "============================\n" << std::endl;
    
    char c;
    while (!g_shutdown_requested && read(STDIN_FILENO, &c, 1) >= 0) {
        if (c == 'q' || c == 'Q') {
            std::cout << "\n[KEYBOARD] Quit requested ('q' pressed)" << std::endl;
            g_shutdown_requested = true;
            break;
        }
        std::this_thread::sleep_for(10ms);
    }
}

class MicroSaccadeNode : public rclcpp::Node {
public:
  MicroSaccadeNode() : Node("micro_saccade_servo_cpp"), paused_(false) {
    // ── Fixed Parameters (no command line needed) ──────────────────
    servo_topic_ = "/servo_server/delta_twist_cmds";
    frame_id_    = "base_link";
    axis_        = "y";          // "x" | "y" | "z"
    amplitude_   = 0.01;        // ±10 mm
    freq_hz_     = 3.0;
    profile_     = "square";     // "square" | "sine"
    rate_hz_     = 250.0;
    ramp_ms_     = 15.0;         // softens square flips
    duration_s_  = -1.0;         // -1 = forever

    // Handle use_sim_time parameter safely
    if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter<bool>("use_sim_time", false);
    }

    // Derived parameters
    omega_ = 2.0 * M_PI * freq_hz_;
    edge_width_s_ = std::max(0.001, ramp_ms_ / 1000.0);

    // Setup servo service clients
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/stop_servo");

    // Wait for servo services to be available
    RCLCPP_INFO(get_logger(), "Waiting for servo services...");
    if (!servo_start_client_->wait_for_service(5s)) {
        RCLCPP_WARN(get_logger(), "Servo start service not available, continuing anyway");
    }

    // Start servo
    startServo();

    // QoS that matches Servo's subscriber (BEST_EFFORT, small queue)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(servo_topic_, qos);
    
    // Give servo a moment to start
    rclcpp::sleep_for(500ms);
    
    t0_ = this->now();

    // Wall timer is fine; stamps use node clock (sim or wall)
    auto period = std::chrono::duration<double>(1.0 / std::max(1e-6, rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MicroSaccadeNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "Micro-saccade (CPP): %s ±%.1f mm @ %.2f Hz on %s-axis | topic=%s frame=%s rate=%.0f Hz",
      profile_.c_str(), amplitude_*1000.0, freq_hz_, axis_.c_str(),
      servo_topic_.c_str(), frame_id_.c_str(), rate_hz_);
    
    RCLCPP_INFO(get_logger(), "Servo mode ACTIVATED - Robot is now in micro-saccade control");
    
    // Start keyboard monitoring thread
    keyboard_thread_ = std::thread(keyboardMonitor);
  }

  ~MicroSaccadeNode() override {
    safeShutdown();
  }

  void safeShutdown() {
    if (shutdown_completed_) return;
    shutdown_completed_ = true;
    
    RCLCPP_INFO(get_logger(), "Initiating safe shutdown...");
    
    // Signal keyboard thread to stop
    g_shutdown_requested = true;
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
    
    // Stop the timer first
    if (timer_) {
        timer_->cancel();
        timer_.reset();
    }
    
    // Publish multiple zero twists to ensure robot stops completely
    RCLCPP_INFO(get_logger(), "Publishing zero velocity commands...");
    for (int i = 0; i < 12; ++i) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      // All velocities are zero by default
      pub_->publish(msg);
      rclcpp::sleep_for(8ms);
    }

    // Stop servo to return control to motion planning
    RCLCPP_INFO(get_logger(), "Stopping servo service...");
    stopServo();
    
    // Additional safety: publish more zeros after stopping servo
    for (int i = 0; i < 8; ++i) {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = this->now();
      msg.header.frame_id = frame_id_;
      pub_->publish(msg);
      rclcpp::sleep_for(10ms);
    }
    
    RCLCPP_INFO(get_logger(), "Servo mode DEACTIVATED - Robot returned to motion planning control");
    std::cout << "\n[SHUTDOWN] Safe shutdown completed. Robot should be stopped.\n" << std::endl;
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
                RCLCPP_INFO(get_logger(), "Servo started successfully");
            } else {
                RCLCPP_WARN(get_logger(), "Failed to start servo: %s", response->message.c_str());
            }
        } else {
            RCLCPP_WARN(get_logger(), "Failed to call servo start service");
        }
    } else {
        RCLCPP_WARN(get_logger(), "Servo start service not ready");
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
                RCLCPP_INFO(get_logger(), "Servo stopped successfully");
            } else {
                RCLCPP_WARN(get_logger(), "Failed to stop servo: %s", response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(get_logger(), "TIMEOUT: Failed to call servo stop service!");
        }
    } else {
        RCLCPP_WARN(get_logger(), "Servo stop service not ready");
    }
  }

  void onTimer() {
    // Check for shutdown request
    if (g_shutdown_requested) {
        safeShutdown();
        rclcpp::shutdown();
        return;
    }
    
    const auto now = this->now();
    const double t = (now - t0_).seconds();

    if (duration_s_ > 0.0 && t >= duration_s_) {
      static bool halted = false;
      if (!halted) {
        RCLCPP_INFO(get_logger(), "Duration reached; halting and stopping servo.");
        safeShutdown();
        rclcpp::shutdown();
        halted = true;
      }
      return;
    }

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now;
    cmd.header.frame_id = frame_id_;

    const double v = (profile_ == "sine") ? velSine(t) : velSquare(t);

    if (axis_ == "x" || axis_ == "X")      cmd.twist.linear.x = v;
    else if (axis_ == "y" || axis_ == "Y") cmd.twist.linear.y = v;
    else                                    cmd.twist.linear.z = v;

    pub_->publish(cmd);
  }

  double velSine(double t) const {
    // x(t) = A sin(ω t) => v(t) = A ω cos(ω t)
    return amplitude_ * omega_ * std::cos(omega_ * t);
  }

  double velSquare(double t) {
    // Constant-velocity segments with soft sign flips.
    // To traverse ±A each half-cycle, |v| = 4 * A * f
    const double v_mag = 4.0 * amplitude_ * freq_hz_;
    const double T = 1.0 / std::max(1e-6, freq_hz_);
    double phase = std::fmod(t, T) / T; // [0,1)

    double sign = (phase < 0.5) ? 1.0 : -1.0;

    // tanh-based smoothing around the edges to avoid controller shock
    const double edge_phase = std::min(phase, 1.0 - phase);
    const double smooth = std::tanh((edge_phase / (edge_width_s_ / T + 1e-9)) * 3.0);

    return sign * v_mag * smooth;
  }

  // Fixed parameters (no command line needed)
  std::string servo_topic_, frame_id_, axis_, profile_;
  double amplitude_, freq_hz_, rate_hz_, ramp_ms_, duration_s_;
  double omega_, edge_width_s_;

  // ROS components
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;
  rclcpp::Time t0_;
  
  // Keyboard control
  std::thread keyboard_thread_;
  std::atomic<bool> paused_;
  bool shutdown_completed_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<MicroSaccadeNode>();
    
    // Custom spinning with shutdown check
    while (rclcpp::ok() && !g_shutdown_requested) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(1ms);
    }
    
    // Ensure safe shutdown is called
    node->safeShutdown();
    
  } catch (const std::exception& e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
  }
  
  rclcpp::shutdown();
  return 0;
}