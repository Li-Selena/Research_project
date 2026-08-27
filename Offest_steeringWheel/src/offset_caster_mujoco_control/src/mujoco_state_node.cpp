#include "offset_caster_mujoco_control/forward_kinematics.hpp"
#include "offset_caster_mujoco_control/mujoco_state_reader.hpp"

#include <GLFW/glfw3.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mujoco/mujoco.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace offset_caster_mujoco_control
{

constexpr std::size_t kCommandCount = 2 * kCasterCount;

class MujocoStateNode : public rclcpp::Node
{
public:
  MujocoStateNode()
  : Node("mujoco_state_node"),
    model_(nullptr, mj_deleteModel),
    data_(nullptr, mj_deleteData)
  {
    const std::string model_path = declare_parameter<std::string>(
      "model_path", "/workspace/3Dmodel/offestWheel/MJCF/offset_steering_wheel.xml");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.25);
    enable_viewer_ = declare_parameter<bool>("enable_viewer", false);
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be finite and positive");
    }
    if (!std::isfinite(command_timeout_) || command_timeout_ <= 0.0) {
      throw std::invalid_argument("command_timeout must be finite and positive");
    }

    const double wheel_radius = declare_parameter<double>("wheel_radius", 0.075);
    const double caster_offset = declare_parameter<double>("caster_offset", 0.105);
    const auto pivot_x = declare_parameter<std::vector<double>>(
      "pivot_x", {0.5, 0.5, -0.5, -0.5});
    const auto pivot_y = declare_parameter<std::vector<double>>(
      "pivot_y", {0.35, -0.35, -0.35, 0.35});
    const auto steering_names = declare_parameter<std::vector<std::string>>(
      "steering_joint_names",
      {"steering_joint_1", "steering_joint_2", "steering_joint_3", "steering_joint_4"});
    const auto wheel_names = declare_parameter<std::vector<std::string>>(
      "wheel_joint_names",
      {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3", "wheel_joint_4"});
    const auto steering_actuator_names = declare_parameter<std::vector<std::string>>(
      "steering_actuator_names",
      {"steering_velocity_1", "steering_velocity_2", "steering_velocity_3",
        "steering_velocity_4"});
    const auto wheel_actuator_names = declare_parameter<std::vector<std::string>>(
      "wheel_actuator_names",
      {"wheel_velocity_1", "wheel_velocity_2", "wheel_velocity_3", "wheel_velocity_4"});

    validate_vector_sizes(
      pivot_x, pivot_y, steering_names, wheel_names,
      steering_actuator_names, wheel_actuator_names);

    for (std::size_t i = 0; i < kCasterCount; ++i) {
      geometry_[i] = {pivot_x[i], pivot_y[i], wheel_radius, caster_offset};
      steering_joint_names_[i] = steering_names[i];
      wheel_joint_names_[i] = wheel_names[i];
      command_joint_names_[i] = steering_names[i];
      command_joint_names_[kCasterCount + i] = wheel_names[i];
    }

    load_model(model_path);
    configure_actuators(steering_actuator_names, wheel_actuator_names);
    base_body_id_ = require_named_object(mjOBJ_BODY, "base");

    state_reader_ = std::make_unique<MujocoStateReader>(
      model_.get(), steering_joint_names_, wheel_joint_names_);
    forward_kinematics_ = std::make_unique<ForwardKinematics>(geometry_);

    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    forward_velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/offset_caster/forward_velocity", 10);
    residual_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/offset_caster/forward_kinematics_residual", 10);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(1).best_effort());
    command_subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/offset_caster/joint_velocity_command", 10,
      std::bind(&MujocoStateNode::command_callback, this, std::placeholders::_1));

    last_publish_time_ = -1.0 / publish_rate_;
    if (enable_viewer_) {
      initialize_viewer();
    }

    RCLCPP_INFO(
      get_logger(), "Loaded MuJoCo model with timestep %.6f s: %s",
      model_->opt.timestep, model_path.c_str());
  }

  ~MujocoStateNode() override
  {
    if (viewer_initialized_) {
      mjr_freeContext(&render_context_);
      mjv_freeScene(&scene_);
      glfwDestroyWindow(window_);
      glfwTerminate();
    }
  }

  [[nodiscard]] double timestep() const
  {
    return model_->opt.timestep;
  }

  [[nodiscard]] bool viewer_running() const
  {
    return !viewer_initialized_ || glfwWindowShouldClose(window_) == GLFW_FALSE;
  }

  void step()
  {
    apply_command();
    mj_step(model_.get(), data_.get());
    publish_clock();

    if (data_->time - last_publish_time_ + 1.0e-12 >= 1.0 / publish_rate_) {
      publish_state();
      last_publish_time_ = data_->time;
    }
    render_if_due();
  }

private:
  void validate_vector_sizes(
    const std::vector<double> & pivot_x,
    const std::vector<double> & pivot_y,
    const std::vector<std::string> & steering_names,
    const std::vector<std::string> & wheel_names,
    const std::vector<std::string> & steering_actuator_names,
    const std::vector<std::string> & wheel_actuator_names) const
  {
    if (pivot_x.size() != kCasterCount || pivot_y.size() != kCasterCount ||
      steering_names.size() != kCasterCount || wheel_names.size() != kCasterCount ||
      steering_actuator_names.size() != kCasterCount ||
      wheel_actuator_names.size() != kCasterCount)
    {
      throw std::invalid_argument("Exactly four caster parameters and names are required");
    }
  }

  void load_model(const std::string & model_path)
  {
    char load_error[1024]{};
    model_.reset(mj_loadXML(model_path.c_str(), nullptr, load_error, sizeof(load_error)));
    if (!model_) {
      throw std::runtime_error(
              "Failed to load MuJoCo model '" + model_path + "': " + load_error);
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_) {
      throw std::runtime_error("Failed to allocate MuJoCo data");
    }
  }

  int require_named_object(const mjtObj type, const std::string & name) const
  {
    const int id = mj_name2id(model_.get(), type, name.c_str());
    if (id < 0) {
      throw std::runtime_error("MuJoCo object not found: " + name);
    }
    return id;
  }

  void configure_actuators(
    const std::vector<std::string> & steering_actuator_names,
    const std::vector<std::string> & wheel_actuator_names)
  {
    for (std::size_t i = 0; i < kCasterCount; ++i) {
      const int steering_id = require_named_object(mjOBJ_ACTUATOR, steering_actuator_names[i]);
      const int wheel_id = require_named_object(mjOBJ_ACTUATOR, wheel_actuator_names[i]);
      if (model_->actuator_ctrlnum[steering_id] != 1 ||
        model_->actuator_ctrlnum[wheel_id] != 1)
      {
        throw std::runtime_error("Each caster actuator must have exactly one control input");
      }
      actuator_control_addresses_[i] = model_->actuator_ctrladr[steering_id];
      actuator_control_addresses_[kCasterCount + i] = model_->actuator_ctrladr[wheel_id];
    }
  }

  void command_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr message)
  {
    if (message->points.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected joint command without trajectory points");
      return;
    }
    const auto & velocities = message->points.back().velocities;
    if (velocities.size() != message->joint_names.size()) {
      RCLCPP_WARN(get_logger(), "Rejected joint command with mismatched names and velocities");
      return;
    }

    std::array<double, kCommandCount> resolved{};
    for (std::size_t expected = 0; expected < kCommandCount; ++expected) {
      const auto iterator = std::find(
        message->joint_names.begin(), message->joint_names.end(), command_joint_names_[expected]);
      if (iterator == message->joint_names.end()) {
        RCLCPP_WARN(
          get_logger(), "Rejected joint command missing '%s'",
          command_joint_names_[expected].c_str());
        return;
      }
      const auto index = static_cast<std::size_t>(
        std::distance(message->joint_names.begin(), iterator));
      if (!std::isfinite(velocities[index])) {
        RCLCPP_WARN(get_logger(), "Rejected non-finite joint velocity command");
        return;
      }
      resolved[expected] = velocities[index];
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    target_joint_velocities_ = resolved;
    last_command_time_ = std::chrono::steady_clock::now();
    has_command_ = true;
  }

  void apply_command()
  {
    std::array<double, kCommandCount> command{};
    bool timed_out = true;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (has_command_) {
        const double age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_command_time_).count();
        timed_out = age > command_timeout_;
        if (!timed_out) {
          command = target_joint_velocities_;
        }
      }
    }

    for (std::size_t i = 0; i < kCommandCount; ++i) {
      data_->ctrl[actuator_control_addresses_[i]] = command[i];
    }
    if (timed_out) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000, "No recent joint command; MuJoCo controls set to zero");
    }
  }

  [[nodiscard]] builtin_interfaces::msg::Time simulation_stamp() const
  {
    constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
    const auto nanoseconds = static_cast<std::int64_t>(
      std::llround(data_->time * static_cast<double>(kNanosecondsPerSecond)));
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
    return stamp;
  }

  void publish_clock()
  {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = simulation_stamp();
    clock_publisher_->publish(clock);
  }

  void publish_state()
  {
    const auto caster_states = state_reader_->read(data_.get());
    const auto forward_result = forward_kinematics_->solve(caster_states);
    const auto stamp = simulation_stamp();

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name.reserve(kCommandCount);
    joint_state.position.reserve(kCommandCount);
    joint_state.velocity.reserve(kCommandCount);
    for (std::size_t i = 0; i < kCasterCount; ++i) {
      joint_state.name.push_back(steering_joint_names_[i]);
      joint_state.position.push_back(caster_states[i].steering_angle);
      joint_state.velocity.push_back(caster_states[i].steering_speed);
    }
    for (std::size_t i = 0; i < kCasterCount; ++i) {
      joint_state.name.push_back(wheel_joint_names_[i]);
      joint_state.position.push_back(caster_states[i].wheel_angle);
      joint_state.velocity.push_back(caster_states[i].wheel_speed);
    }
    joint_state_publisher_->publish(joint_state);

    geometry_msgs::msg::TwistStamped forward_velocity;
    forward_velocity.header.stamp = stamp;
    forward_velocity.header.frame_id = "base";
    forward_velocity.twist.linear.x = forward_result.twist.linear_x;
    forward_velocity.twist.linear.y = forward_result.twist.linear_y;
    forward_velocity.twist.angular.z = forward_result.twist.angular_z;
    forward_velocity_publisher_->publish(forward_velocity);

    std_msgs::msg::Float64 residual;
    residual.data = forward_result.residual_rms;
    residual_publisher_->publish(residual);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "base";
    odometry.pose.pose.position.x = data_->xpos[3 * base_body_id_];
    odometry.pose.pose.position.y = data_->xpos[3 * base_body_id_ + 1];
    odometry.pose.pose.position.z = data_->xpos[3 * base_body_id_ + 2];
    odometry.pose.pose.orientation.w = data_->xquat[4 * base_body_id_];
    odometry.pose.pose.orientation.x = data_->xquat[4 * base_body_id_ + 1];
    odometry.pose.pose.orientation.y = data_->xquat[4 * base_body_id_ + 2];
    odometry.pose.pose.orientation.z = data_->xquat[4 * base_body_id_ + 3];
    odometry.twist.twist.linear.x = forward_result.twist.linear_x;
    odometry.twist.twist.linear.y = forward_result.twist.linear_y;
    odometry.twist.twist.angular.z = forward_result.twist.angular_z;
    odometry_publisher_->publish(odometry);
  }

  void initialize_viewer()
  {
    if (glfwInit() == GLFW_FALSE) {
      throw std::runtime_error("GLFW initialization failed; check DISPLAY/X11 access");
    }
    window_ = glfwCreateWindow(1280, 720, "Offset Caster MuJoCo", nullptr, nullptr);
    if (window_ == nullptr) {
      glfwTerminate();
      throw std::runtime_error("MuJoCo viewer window creation failed");
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visualization_options_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&render_context_);
    camera_.type = mjCAMERA_TRACKING;
    camera_.trackbodyid = base_body_id_;
    camera_.distance = 2.8;
    camera_.azimuth = 135.0;
    camera_.elevation = -25.0;
    mjv_makeScene(model_.get(), &scene_, 2000);
    mjr_makeContext(model_.get(), &render_context_, mjFONTSCALE_150);
    next_render_time_ = std::chrono::steady_clock::now();
    viewer_initialized_ = true;
  }

  void render_if_due()
  {
    if (!viewer_initialized_ || std::chrono::steady_clock::now() < next_render_time_) {
      return;
    }
    next_render_time_ += std::chrono::milliseconds(16);
    glfwMakeContextCurrent(window_);
    glfwPollEvents();
    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    const mjrRect viewport{0, 0, width, height};
    mjv_updateScene(
      model_.get(), data_.get(), &visualization_options_, nullptr, &camera_, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &render_context_);
    glfwSwapBuffers(window_);
  }

  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data_;
  std::unique_ptr<MujocoStateReader> state_reader_;
  std::unique_ptr<ForwardKinematics> forward_kinematics_;
  std::array<CasterGeometry, kCasterCount> geometry_{};
  std::array<std::string, kCasterCount> steering_joint_names_{};
  std::array<std::string, kCasterCount> wheel_joint_names_{};
  std::array<std::string, kCommandCount> command_joint_names_{};
  std::array<int, kCommandCount> actuator_control_addresses_{};
  int base_body_id_{-1};
  double publish_rate_{50.0};
  double command_timeout_{0.25};
  double last_publish_time_{0.0};

  std::mutex command_mutex_;
  std::array<double, kCommandCount> target_joint_velocities_{};
  std::chrono::steady_clock::time_point last_command_time_{};
  bool has_command_{false};

  bool enable_viewer_{false};
  bool viewer_initialized_{false};
  GLFWwindow * window_{nullptr};
  mjvCamera camera_{};
  mjvOption visualization_options_{};
  mjvScene scene_{};
  mjrContext render_context_{};
  std::chrono::steady_clock::time_point next_render_time_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr forward_velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr residual_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_subscription_;
};

}  // namespace offset_caster_mujoco_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<offset_caster_mujoco_control::MujocoStateNode>();
  auto next_step = std::chrono::steady_clock::now();
  const auto step_duration = std::chrono::duration<double>(node->timestep());

  while (rclcpp::ok() && node->viewer_running()) {
    rclcpp::spin_some(node);
    node->step();
    next_step += std::chrono::duration_cast<std::chrono::steady_clock::duration>(step_duration);
    std::this_thread::sleep_until(next_step);
  }

  rclcpp::shutdown();
  return 0;
}
