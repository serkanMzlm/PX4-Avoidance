#include "transform_buffer.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace avoidance {

namespace tf_buffer {

TransformBuffer::TransformBuffer(float buffer_size_s) : buffer_size_(rclcpp::Duration(buffer_size_s, 0)) {
  startup_time_ = rclcpp::Clock().now();
};

std::string TransformBuffer::getKey(const std::string& source_frame, const std::string& target_frame) const {
  return source_frame + "_to_" + target_frame;
}


bool TransformBuffer::interpolateTransform(const geometry_msgs::msg::TransformStamped& tf_earlier, 
                                           const geometry_msgs::msg::TransformStamped& tf_later,
                                           geometry_msgs::msg::TransformStamped& transform) const {
  // Check if the requested timestamp lies between the two given transforms
  if (rclcpp::Time(transform.header.stamp) > rclcpp::Time(tf_later.header.stamp) || 
      rclcpp::Time(transform.header.stamp) < rclcpp::Time(tf_earlier.header.stamp)) {
    return false;
  }

  const rclcpp::Duration timeBetween = rclcpp::Time(tf_later.header.stamp) - rclcpp::Time(tf_earlier.header.stamp);
  const rclcpp::Duration timeAfterEarlier = rclcpp::Time(transform.header.stamp) - rclcpp::Time(tf_earlier.header.stamp);
  const float tau = static_cast<float>(timeAfterEarlier.nanoseconds()) / static_cast<float>(timeBetween.nanoseconds());

  tf2::Vector3 tf_earlier_translation;
  tf2::Quaternion tf_earlier_rotation;
  tf2::Quaternion tf_later_rotation;

  tf_earlier_translation.setX(tf_earlier.transform.translation.x);
  tf_earlier_translation.setY(tf_earlier.transform.translation.y);
  tf_earlier_translation.setZ(tf_earlier.transform.translation.z);

  tf2::fromMsg(tf_earlier.transform.rotation, tf_earlier_rotation);
  tf2::fromMsg(tf_later.transform.rotation, tf_later_rotation);

  const tf2::Vector3 translation = tf_earlier_translation * (1.0f - tau) + tf_earlier_translation * tau;
  const tf2::Quaternion rotation = tf_earlier_rotation.slerp(tf_later_rotation, tau);

  transform.transform.translation.x = translation.x();
  transform.transform.translation.y = translation.y();
  transform.transform.translation.z = translation.z();
  transform.transform.rotation = tf2::toMsg(rotation);

  return true;
}

bool TransformBuffer::insertTransform(const std::string& source_frame, const std::string& target_frame,
                                      geometry_msgs::msg::TransformStamped transform) {
  std::lock_guard<std::mutex> lck(mutex_);
  std::unordered_map<std::string, std::deque<geometry_msgs::msg::TransformStamped>>::iterator iterator =
      buffer_.find(getKey(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    std::deque<geometry_msgs::msg::TransformStamped> empty_deque;
    buffer_[getKey(source_frame, target_frame)] = empty_deque;
    iterator = buffer_.find(getKey(source_frame, target_frame));
  }

  // check if the given transform is newer than the last buffered one
  if (iterator->second.size() == 0 || rclcpp::Time(iterator->second.back().header.stamp) < rclcpp::Time(transform.header.stamp)) {
    iterator->second.push_back(transform);
    // remove transforms which are outside the buffer size
    while (rclcpp::Time(transform.header.stamp) - rclcpp::Time(iterator->second.front().header.stamp) > buffer_size_) {
      iterator->second.pop_front();
    }
    return true;
  }
  return false;
}

bool TransformBuffer::getTransform(const std::string& source_frame, const std::string& target_frame,
                                         const rclcpp::Time& time, geometry_msgs::msg::TransformStamped& transform) const {
    std::lock_guard<std::mutex> lck(mutex_);

    auto it = buffer_.find(getKey(source_frame, target_frame));
    if (it == buffer_.end() || it->second.empty()) {
        print(log_level::error, "TF Buffer: Could not retrieve requested transform from buffer.");
        return false;
    }

    const auto& transforms = it->second;
    rclcpp::Time earliest_time = transforms.front().header.stamp;
    rclcpp::Time latest_time = transforms.back().header.stamp;

    if (time < earliest_time || time > latest_time) {
        print(log_level::warn, "TF Buffer: Requested time is out of bounds.");
        return false;
    }

    for (size_t i = 0; i < transforms.size() - 1; ++i) {
        const auto& tf_earlier = transforms[i];
        const auto& tf_later = transforms[i + 1];

        if (time >= rclcpp::Time(tf_earlier.header.stamp) && time <= rclcpp::Time(tf_later.header.stamp)) {
            interpolateTransform(tf_earlier, tf_later, transform);
            transform.header.stamp = time;
            return true;
        }
    }

    return false;
}

void TransformBuffer::print(const log_level& level, const std::string& msg) const {
  if (rclcpp::Clock().now() - startup_time_ > rclcpp::Duration(3, 0)) {
    switch (level) {
      case error: {
        RCLCPP_ERROR(tf_logger_, "%s", msg.c_str());
        break;
      }
      case warn: {
        RCLCPP_WARN(tf_logger_, "%s", msg.c_str());
        break;
      }
      case info: {
        RCLCPP_INFO(tf_logger_, "%s", msg.c_str());
        break;
      }
      case debug: {
        RCLCPP_DEBUG(tf_logger_, "%s", msg.c_str());
        break;
      }
    }
  }
}
}
}
