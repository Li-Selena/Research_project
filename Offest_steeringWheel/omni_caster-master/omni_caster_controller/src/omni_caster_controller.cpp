#include "omni_caster_controller/omni_caster_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "controller_interface/helpers.hpp"

namespace
{

  using hardware_interface::HW_IF_POSITION;
  using hardware_interface::HW_IF_VELOCITY;

  static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1, // message queue depth
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false};

} // namespace

namespace omni_caster_controller
{

  OmniCasterController::OmniCasterController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn OmniCasterController::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<omni_caster_controller::ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OmniCasterController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto logger = get_node()->get_logger();

    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      RCLCPP_INFO(logger, "Parameters were updated");
    }

    // Init params
    //  caster_number = params_.caster_number;
    distance_steering2chassiscentens = params_.distance_steering2chassiscentens;
    offset = params_.offset;
    wheel_radius = params_.wheel_radius;
    use_stamped_vel_ = params_.use_stamped_vel;
    cmd_vel_timeout_ = std::chrono::milliseconds{static_cast<int>(params_.cmd_vel_timeout * 1000.0)};

    // Init odometry params

    // Init omni caster kinematics
    kinematics_ = std::make_unique<OmniCasterChassisKinematics>(
        caster_number, wheel_radius, offset, distance_steering2chassiscentens);

    // Init speedlimiter
    limiter_linear_x_ = SpeedLimiter(
        params_.linear.has_velocity_limits, params_.linear.has_acceleration_limits,
        params_.linear.has_jerk_limits, params_.linear.min_velocity, params_.linear.max_velocity,
        params_.linear.min_acceleration, params_.linear.max_acceleration, params_.linear.min_jerk,
        params_.linear.max_jerk);

    limiter_linear_y_ = SpeedLimiter(
        params_.linear.has_velocity_limits, params_.linear.has_acceleration_limits,
        params_.linear.has_jerk_limits, params_.linear.min_velocity, params_.linear.max_velocity,
        params_.linear.min_acceleration, params_.linear.max_acceleration, params_.linear.min_jerk,
        params_.linear.max_jerk);

    limiter_angular_ = SpeedLimiter(
        params_.angular.has_velocity_limits, params_.angular.has_acceleration_limits,
        params_.angular.has_jerk_limits, params_.angular.min_velocity,
        params_.angular.max_velocity, params_.angular.min_acceleration,
        params_.angular.max_acceleration, params_.angular.min_jerk, params_.angular.max_jerk);

    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    // init previous_commands_   (fill last two commands with default constructed commands)
    const std::vector<double> empty_command(3);
    previous_commands_.emplace(empty_command);
    previous_commands_.emplace(empty_command);

    // init command  cmd_vel subscribe
    if (use_stamped_vel_)
    {
      velocity_command_stamped_subscriber_ = get_node()->create_subscription<TwistStamped>(
          DEFAULT_COMMAND_STAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const std::shared_ptr<TwistStamped> msg) -> void
          {
            if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
            {
              RCLCPP_WARN_ONCE(
                  get_node()->get_logger(),
                  "Received TwistStamped with zero timestamp, setting it to current "
                  "time, this message will only be shown once");
              msg->header.stamp = get_node()->get_clock()->now();
            }
            rt_twist_stamped_.writeFromNonRT(std::move(msg)); // 使用move 移动减少开销
          });
    }
    else
    {
      velocity_command_subscriber_ =
          get_node()->create_subscription<Twist>(
              DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
              [this](const std::shared_ptr<Twist> msg) -> void
              {
                // Write fake header in the stored stamped command
                auto twist_stamped = std::make_shared<TwistStamped>();
                twist_stamped->twist = *msg;
                twist_stamped->header.stamp = get_node()->get_clock()->now();
                rt_twist_stamped_.writeFromNonRT(std::move(twist_stamped));
              });
    }

    // init RT msg types: twiststamped
    std::shared_ptr<TwistStamped> msg_stamped = std::make_shared<TwistStamped>();
    rt_twist_stamped_.initRT(msg_stamped);
    rt_twist_stamped_.writeFromNonRT(msg_stamped);

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration OmniCasterController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(6);

    command_interfaces_config.names.push_back(params_.forward_wheel_name + "/" + HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(params_.left_wheel_name + "/" + HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(params_.right_wheel_name + "/" + HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(params_.forward_steering_name + "/" + HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(params_.left_steering_name + "/" + HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(params_.right_steering_name + "/" + HW_IF_VELOCITY);
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration OmniCasterController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(12);

    state_interfaces_config.names.push_back(params_.forward_wheel_name + "/" + HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(params_.left_wheel_name + "/" + HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(params_.right_wheel_name + "/" + HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(params_.forward_steering_name + "/" + HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(params_.left_steering_name + "/" + HW_IF_VELOCITY);
    state_interfaces_config.names.push_back(params_.right_steering_name + "/" + HW_IF_VELOCITY);

    state_interfaces_config.names.push_back(params_.forward_wheel_name + "/" + HW_IF_POSITION);
    state_interfaces_config.names.push_back(params_.left_wheel_name + "/" + HW_IF_POSITION);
    state_interfaces_config.names.push_back(params_.right_wheel_name + "/" + HW_IF_POSITION);
    state_interfaces_config.names.push_back(params_.forward_steering_name + "/" + HW_IF_POSITION);
    state_interfaces_config.names.push_back(params_.left_steering_name + "/" + HW_IF_POSITION);
    state_interfaces_config.names.push_back(params_.right_steering_name + "/" + HW_IF_POSITION);
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn OmniCasterController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_halted = false;
    reset_chassis_vel(*(rt_twist_stamped_.readFromRT)());

    wheel_registered_handle_.clear();
    steering_registered_handle_.clear();

    wheel_registered_handle_.reserve(3);
    steering_registered_handle_.reserve(3);

    // 依次注册3个车轮句柄  此处config_register使用顺序决定了枚举类型中的顺序
    const auto f_w = config_register(params_.forward_wheel_name, wheel_registered_handle_);
    const auto l_w = config_register(params_.left_wheel_name, wheel_registered_handle_);
    const auto r_w = config_register(params_.right_wheel_name, wheel_registered_handle_);

    // 依次注册3个转向句柄
    const auto f_s = config_register(params_.forward_steering_name, steering_registered_handle_);
    const auto l_s = config_register(params_.left_steering_name, steering_registered_handle_);
    const auto r_s = config_register(params_.right_steering_name, steering_registered_handle_);
    if (
        f_w == controller_interface::CallbackReturn::ERROR ||
        f_s == controller_interface::CallbackReturn::ERROR ||
        r_w == controller_interface::CallbackReturn::ERROR ||
        r_s == controller_interface::CallbackReturn::ERROR ||
        l_w == controller_interface::CallbackReturn::ERROR ||
        l_s == controller_interface::CallbackReturn::ERROR)
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_DEBUG(get_node()->get_logger(), "active.");
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn OmniCasterController::on_error(const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn OmniCasterController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    // for (size_t i = 0; i < command_interfaces_.size(); ++i)
    // {
    //   command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
    // }
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    wheel_registered_handle_.clear();
    steering_registered_handle_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type OmniCasterController::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    auto logger = get_node()->get_logger();
    previous_update_timestamp_ = time;
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      if (!is_halted)
      {
        halt();
        is_halted = true;
      }
      return controller_interface::return_type::OK;
    }

    auto target_vel = rt_twist_stamped_.readFromRT();

    std::vector<double> Vxyw_cmd;
    Vxyw_cmd.push_back((*target_vel)->twist.linear.x);
    Vxyw_cmd.push_back((*target_vel)->twist.linear.y);
    Vxyw_cmd.push_back((*target_vel)->twist.angular.z);

    const auto age_of_last_command = time - (*target_vel)->header.stamp;
    // Brake if cmd_vel has timeout, override the stored command
    if (age_of_last_command > cmd_vel_timeout_)
    {
      RCLCPP_ERROR(logger, "cmd_vel has timeout  Age_time[%f]", age_of_last_command.seconds());

      for (int i = 0; i < 3; i++)
      {
        Vxyw_cmd[0] = 0.0;
      }
    }

    double &linear_x_command = Vxyw_cmd[0];
    double &linear_y_command = Vxyw_cmd[1];
    double &angular_command = Vxyw_cmd[2];

    auto &last_command = previous_commands_.back();
    auto &second_to_last_command = previous_commands_.front();

    limiter_linear_x_.limit(
        linear_x_command, last_command[0], second_to_last_command[0], period.seconds());
    limiter_linear_y_.limit(
        linear_y_command, last_command[1], second_to_last_command[1], period.seconds());
    limiter_angular_.limit(
        angular_command, last_command[2], second_to_last_command[2], period.seconds());

    previous_commands_.pop();
    previous_commands_.emplace(Vxyw_cmd);

    for (int i = 0; i < caster_number; i++)
    {
      kinematics_->casters[i].chassis_steering_angle = steering_registered_handle_[i].feedback_pose.get().get_value();
    }

    kinematics_->InverseKinematics(Vxyw_cmd, kinematics_->casters);

    std::vector<double> Vxyw_obser(3);
    kinematics_->ForwardKinematics(kinematics_->casters, Vxyw_obser);
    RCLCPP_INFO(logger, "obser : Vx[%f]   Vy[%f]   Wz[%f] ", Vxyw_obser[0], Vxyw_obser[1], Vxyw_obser[2]);
    // 解算

    // RCLCPP_INFO(logger, "target: forward[%f]   Vy[%f]   Wz[%f]  Age_time[%f]", (*target_vel)->twist.linear.x, (*target_vel)->twist.linear.y, (*target_vel)->twist.angular.z, age_of_last_command.seconds());
    RCLCPP_INFO(logger, "limit : forward[%f]   Vy[%f]   Wz[%f] Age_time[%f]", Vxyw_cmd[0], Vxyw_cmd[1], Vxyw_cmd[2], age_of_last_command.seconds());
    for (int i = 0; i < caster_number; i++)
    {
      // RCLCPP_INFO(logger, "wheel[%f]   steering[%f]  i[%d]", kinematics_->casters[i].wheel_angular_velocity, kinematics_->casters[i].steering_angular_velocity, i);
      // RCLCPP_INFO(logger, "steering1[%f]   steering2[%f]   steering3[%f] ", (*target_vel)->linear.x, (*target_vel)->linear.y, (*target_vel)->angular.z);
      wheel_registered_handle_[i].velocity.get().set_value(kinematics_->casters[i].wheel_angular_velocity);
      steering_registered_handle_[i].velocity.get().set_value(kinematics_->casters[i].steering_angular_velocity);
    }

    //***********************debug
    // wheel_registered_handle_[FORWARD_CASTER].velocity.get().set_value(kinematics_->casters[FORWARD_CASTER].wheel_angular_velocity);
    // wheel_registered_handle_[LEFT_CASTER].velocity.get().set_value(kinematics_->casters[LEFT_CASTER].wheel_angular_velocity);
    // wheel_registered_handle_[RIGHT_CASTER].velocity.get().set_value(kinematics_->casters[RIGHT_CASTER].wheel_angular_velocity);
    // steering_registered_handle_[FORWARD_CASTER].velocity.get().set_value(0.1);
    // steering_registered_handle_[LEFT_CASTER].velocity.get().set_value(0.1);
    // steering_registered_handle_[RIGHT_CASTER].velocity.get().set_value(0.1);
    // 示例：读取反馈
    // double forward_wheel_vel = wheel_handles_[FORWARD].feedback_vel.get().get_value();
    // double forward_wheel_pos = wheel_handles_[FORWARD].feedback_pose.get().get_value();

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn OmniCasterController::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OmniCasterController::config_register(
      const std::string &joint_name, std::vector<JointHandle> &handle_vector)
  {
    auto logger = get_node()->get_logger();

    const auto vel_state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&joint_name](const auto &interface)
        {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (vel_state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state vel handle for %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    const auto pose_state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&joint_name](const auto &interface)
        {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == HW_IF_POSITION;
        });

    if (pose_state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state pose handle for %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&joint_name](const auto &interface)
        {
          return interface.get_prefix_name() == joint_name &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    handle_vector.emplace_back(JointHandle{
        std::ref(*vel_state_handle),  // 绑定速度状态接口
        std::ref(*pose_state_handle), // 绑定位置状态接口
        std::ref(*command_handle)     // 绑定速度命令接口
    });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  void OmniCasterController::halt()
  {
    // 停止所有车轮
    for (auto &handle : wheel_registered_handle_)
    {
      handle.velocity.get().set_value(0.0);
    }

    // 停止所有转向节
    for (auto &handle : steering_registered_handle_)
    {
      handle.velocity.get().set_value(0.0);
    }
  }

  bool OmniCasterController::reset()
  {
    is_halted = false;

    odometry_.resetOdometry();

    std::queue<std::vector<double>> empty;
    std::swap(previous_commands_, empty);

    wheel_registered_handle_.clear();
    steering_registered_handle_.clear();

    velocity_command_subscriber_.reset();
    velocity_command_stamped_subscriber_.reset();

    // velocity_command_subscriber_.reset();//reset subscriber
    rt_twist_stamped_.reset();

    return true;
  }

  void OmniCasterController::reset_chassis_vel(std::shared_ptr<TwistStamped> &msg)
  {
    msg->twist.linear.x = 0;
    msg->twist.linear.y = 0;
    msg->twist.linear.z = 0;
    msg->twist.angular.x = 0;
    msg->twist.angular.y = 0;
    msg->twist.angular.z = 0;
    msg->header.stamp = get_node()->get_clock()->now();
  }

} // namespace omni_caster_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    omni_caster_controller::OmniCasterController, controller_interface::ControllerInterface)