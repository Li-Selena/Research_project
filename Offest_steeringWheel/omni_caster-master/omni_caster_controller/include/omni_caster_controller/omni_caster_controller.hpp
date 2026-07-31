#ifndef OMNI_CASTER_CONTROLLER__OMNI_CASTER_CONTROLLER_HPP_
#define OMNI_CASTER_CONTROLLER__OMNI_CASTER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include "controller_interface/controller_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <realtime_tools/realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_tools/realtime_publisher.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>

#include "omni_caster_controller/omni_caster_controller_parameters.hpp"
#include "omni_caster_controller/omni_caster_chassis_kinematics.hpp"
#include "omni_caster_controller/speed_limiter.hpp"
#include "omni_caster_controller/odometry.hpp"
namespace omni_caster_controller
{
  constexpr auto DEFAULT_COMMAND_STAMPED_TOPIC = "~/cmd_vel_stamped";
  constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel";
  constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
  constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
  constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";

  class OmniCasterController : public controller_interface::ControllerInterface
  {
  public:
    OmniCasterController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State &) override;

    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &) override;

    using Twist = geometry_msgs::msg::Twist;
    using TwistStamped = geometry_msgs::msg::TwistStamped;

  protected:
    struct JointHandle
    {
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_vel;
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_pose;
      std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
    };
    std::vector<JointHandle> wheel_registered_handle_;
    std::vector<JointHandle> steering_registered_handle_;

    Params params_;
    std::shared_ptr<omni_caster_controller::ParamListener> param_listener_;

    // realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> rt_twist_{nullptr};
    realtime_tools::RealtimeBuffer<std::shared_ptr<TwistStamped>> rt_twist_stamped_{nullptr};

    rclcpp::Subscription<Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
    rclcpp::Subscription<TwistStamped>::SharedPtr velocity_command_stamped_subscriber_ = nullptr;

    bool is_halted = false;
    bool use_stamped_vel_ = false;
    int caster_number = 3;
    double distance_steering2chassiscentens = 0.0;
    double offset = 0.0;
    double wheel_radius = 0.0;

    rclcpp::Time previous_update_timestamp_{0};
    // Timeout to consider cmd_vel commands old
    std::chrono::milliseconds cmd_vel_timeout_{500};

    // odometry
    Odometry odometry_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odometry_publisher_ = nullptr;

    // tf
    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>>
        odometry_transform_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_odometry_transform_publisher_ = nullptr;

    // speed limiters
    SpeedLimiter limiter_linear_x_;
    SpeedLimiter limiter_linear_y_;
    SpeedLimiter limiter_angular_;
    std::queue<std::vector<double>> previous_commands_;

    std::unique_ptr<OmniCasterChassisKinematics> kinematics_;

    controller_interface::CallbackReturn config_register(const std::string &joint_name, std::vector<JointHandle> &handle_vector);
    void halt();
    bool reset();
    void reset_chassis_vel(std::shared_ptr<TwistStamped> &msg);
  };

} // namespace omni_caster_controller

#endif // OMNI_CASTER_CONTROLLER__OMNI_CASTER_CONTROLLER_HPP_