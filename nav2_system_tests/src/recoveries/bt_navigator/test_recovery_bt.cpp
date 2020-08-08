//
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_recoveries/test_recoveries.hpp"

using nav2_recoveries::Recovery;
using nav2_recoveries::Status;
using RecoveryAction = nav2_msgs::action::DummyRecovery;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<RecoveryAction>;

using namespace std::chrono_literals;
int main(int argc, char ** argv)
{
    // initialize ROS
    rclcpp::init(argc, argv);
    
    auto node_lifecycle_ =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "LifecycleRecoveryTestNode", rclcpp::NodeOptions());
    node_lifecycle_->declare_parameter(
      "costmap_topic",
      rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
    node_lifecycle_->declare_parameter(
      "footprint_topic",
      rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

    auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      node_lifecycle_->get_node_base_interface(),
      node_lifecycle_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    auto tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string costmap_topic, footprint_topic;
    node_lifecycle_->get_parameter("costmap_topic", costmap_topic);
    node_lifecycle_->get_parameter("footprint_topic", footprint_topic);
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
      node_lifecycle_, costmap_topic);
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_ =
      std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
      node_lifecycle_, footprint_topic, 1.0);
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_ =
      std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
      *costmap_sub_, *footprint_sub_, *tf_buffer_,
      node_lifecycle_->get_name(), "odom");

    auto recovery_ = std::make_shared<DummyRecovery>();
    recovery_->configure(node_lifecycle_, "Recovery", tf_buffer_, collision_checker_);
    recovery_->activate();
    while(rclcpp::ok()){

    };
    // shutdown ROS
    rclcpp::shutdown();
    
    return true;
}