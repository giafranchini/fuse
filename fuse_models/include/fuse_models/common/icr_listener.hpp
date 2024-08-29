#ifndef FUSE_MODELS__ICR_LISTENER_HPP_
#define FUSE_MODELS__ICR_LISTENER_HPP_

#include <functional>
#include <memory>
#include <thread>
#include <utility>
#include <cmath>

#include "tf2/time_cache.h"
#include "tf2/time.h"
#include "tf2_ros/visibility_control.h"

#include "emrs_interfaces/msg/point2_d_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/qos.hpp"

namespace fuse_models
{

namespace detail
{
template<class AllocatorT = std::allocator<void>>
rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>
get_default_icr_listener_sub_options()
{
  rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability};
  return options;
}
}  // namespace detail

/** \brief This class provides an easy way to request and receive icr transform information.
 */
class ICRListener
{
public:
  /** \brief Simplified constructor for icr listener.
   *
   * This constructor will create a new ROS 2 node under the hood.
   * If you already have access to a ROS 2 node and you want to associate the ICRListener
   * to it, then it's recommended to use one of the other constructors.
   */
  ICRListener(tf2::TimeCache & cache, bool spin_thread = true)
  : cache_(cache)
  {
    rclcpp::NodeOptions options;
    // create a unique name for the node
    // but specify its name in .arguments to override any __node passed on the command line.
    // avoiding sstream because it's behavior can be overridden by external libraries.
    // See this issue: https://github.com/ros2/geometry2/issues/540
    char node_name[42];
    snprintf(
      node_name, sizeof(node_name), "icr_listener_impl_%zx",
      reinterpret_cast<size_t>(this)
    );
    options.arguments({"--ros-args", "-r", "__node:=" + std::string(node_name)});
    options.start_parameter_event_publisher(false);
    options.start_parameter_services(false);
    optional_default_node_ = rclcpp::Node::make_shared("_", options);
    init(
      optional_default_node_->get_node_base_interface(),
      optional_default_node_->get_node_logging_interface(),
      optional_default_node_->get_node_parameters_interface(),
      optional_default_node_->get_node_topics_interface(),
      spin_thread, tf2_ros::DynamicListenerQoS(),
      detail::get_default_icr_listener_sub_options());
  }

  /** \brief Node constructor */
  template<class NodeT, class AllocatorT = std::allocator<void>>
  ICRListener(
    tf2::TimeCache & cache,
    NodeT && node,
    bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    detail::get_default_icr_listener_sub_options<AllocatorT>())
  : ICRListener(
      cache,
      node->get_node_base_interface(),
      node->get_node_logging_interface(),
      node->get_node_parameters_interface(),
      node->get_node_topics_interface(),
      spin_thread,
      qos,
      options)
  {}

  /** \brief Node interface constructor */
  template<class AllocatorT = std::allocator<void>>
  ICRListener(
    tf2::TimeCache & cache,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    bool spin_thread = true,
    const rclcpp::QoS & qos = tf2_ros::DynamicListenerQoS(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    detail::get_default_icr_listener_sub_options<AllocatorT>())
  : cache_(cache)
  {
    init(
      node_base,
      node_logging,
      node_parameters,
      node_topics,
      spin_thread,
      qos,
      options);
  }

  ~ICRListener()
  {
    if (spin_thread_) {
      executor_->cancel();
      dedicated_listener_thread_->join();
    }
  }

  /// Callback function for ICR position message subscription
  // let base_link be frame a, and icr be frame b
  void subscription_callback(const emrs_interfaces::msg::Point2DStamped & msg)
  {
    // Check if ICR is moving
    if (!isMoving(msg)) {
      return;
    }
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(-msg.x, -msg.y, 0));
    transform.setRotation(tf2::Quaternion(0, 0, 0, 1));

    auto timestamp = tf2::TimePoint(
      std::chrono::seconds(msg.header.stamp.sec) +
      std::chrono::nanoseconds(msg.header.stamp.nanosec));
    setTransform(transform, timestamp);
  }

  void setTransform(const tf2::Transform & tf, const tf2::TimePoint & stamp)
  {
    cache_.insertData(
      tf2::TransformStorage(
        stamp, tf.getRotation(), tf.getOrigin(), 0, 1));
    last_icr_tf_ = tf;
  }

  bool isMoving(const emrs_interfaces::msg::Point2DStamped & msg, const double tol = 1e-3) const
  {
    return std::hypot(
      (msg.x - last_icr_tf_.getOrigin().x()), (msg.y - last_icr_tf_.getOrigin().y())) > tol;
  }

  const tf2::Transform & getLastTransform() const
  {
    return last_icr_tf_;
  }

private:
  template<class AllocatorT = std::allocator<void>>
  void init(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics,
    bool spin_thread,
    const rclcpp::QoS & qos,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options)
  {
    spin_thread_ = spin_thread;
    node_base_interface_ = node_base;
    node_logging_interface_ = node_logging;

    using callback_t = std::function<void (const emrs_interfaces::msg::Point2DStamped &)>;
    callback_t cb = std::bind(
      &ICRListener::subscription_callback, this, std::placeholders::_1);

    if (spin_thread_) {
      // Create new callback group for message_subscription of icr position
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_options = options;
      callback_group_ = node_base_interface_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
      tf_options.callback_group = callback_group_;

      message_subscription_icr_ = rclcpp::create_subscription<emrs_interfaces::msg::Point2DStamped>(
        node_parameters, node_topics, "/emrs/locomotion/icr/filtered/pos", qos, std::move(
          cb), tf_options);

      // Create executor with dedicated thread to spin.
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base_interface_);
      dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
    } else {
      message_subscription_icr_ = rclcpp::create_subscription<emrs_interfaces::msg::Point2DStamped>(
        node_parameters, node_topics, "/emrs/locomotion/icr/filtered/pos", qos, std::move(
          cb), options);
    }

    setTransform(last_icr_tf_, tf2::get_now());
  }

  bool spin_thread_{false};
  std::unique_ptr<std::thread> dedicated_listener_thread_ {nullptr};
  rclcpp::Executor::SharedPtr executor_ {nullptr};

  rclcpp::Node::SharedPtr optional_default_node_ {nullptr};
  rclcpp::Subscription<emrs_interfaces::msg::Point2DStamped>::SharedPtr
    message_subscription_icr_ {nullptr};
  tf2::TimeCache & cache_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_ {nullptr};
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};

  tf2::Transform last_icr_tf_{tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0)};
};
}  // namespace fuse_models

#endif  // FUSE_MODELS__ICR_LISTENER_HPP_
