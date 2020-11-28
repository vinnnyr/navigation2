// Copyright (c) 2020 Vinny Ruia
// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2018 Intel Corporation
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
// limitations under the License. Reserved.

#ifndef BEHAVIOR_TREE_TESTER_HPP_
#define BEHAVIOR_TREE_TESTER_HPP_

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav2_behavior_tree/test/test_action_server.hpp"
#include "nav2_behavior_tree/test/test_service.hpp"

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"
#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"
#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"
#include "nav2_behavior_tree/plugins/action/spin_action.hpp"
#include "nav2_behavior_tree/plugins/action/wait_action.hpp"
#include "nav2_behavior_tree/plugins/action/back_up_action.hpp"

namespace nav2_system_tests
{
    template<class ActionT>
    class FakeActionServer : public TestActionServer<ActionT>
    {
    public:
      FakeActionServer()
      : FakeActionServer("fake")
      {}

    protected:
      void execute(
        const typename std::shared_ptr<rclpp_action::ServerGoalHandle<ActionT>>
        goal_handle)
      override
      {
          ActionT::Result::SharedPtr result = std::make_shared<ActionT::Result>();
          bool return_success = getReturnSuccess();
          if (return_success) {
              goal_handle->success(result);
          } else {
              goal_handle->abort(result);
          }
      }
    };

    class ClearEntireCostmapService : public TestService<nav2_msgs::srv::ClearEntireCostmap>
    {
    public:
    ClearEntireCostmapService()
    : TestService("clear_entire_costmap")
    {}
    };

    class ComputePathToPoseActionServer : public FakeActionServer<nav2_msgs::action::ComputePathToPose>
    {
      public:
        ComputePathToPoseActionServer() : FakeActionServer("ComputePathToPoseActionServer")
        {}
    }
    
    class FollowPathActionServer : public FakeActionServer<nav2_msgs::action::FollowPath>
    {
      public:
        FollowPathActionServer() : FakeActionServer("FollowPathActionServer")
        {}
    }
    // Since Clear Costmap is a service not an action server, we will have to 
    // figure out how to fake that
    // class ClearCostmapActionServer : public FakeActionServer<nav2_msgs::action::ClearCostmap>
    // {
    //   public:
    //     FollowPathActionServer() : FakeActionServer("FollowPathActionServer")
    //     {}
    // }
    class SpinActionServer : public FakeActionServer<nav2_msgs::action::Spin>
    {
      public:
        SpinActionServer() : FakeActionServer("SpinActionServer")
        {}
    }
    
    class WaitActionServer : public FakeActionServer<nav2_msgs::action::Wait>
    {
      public:
        WaitActionServer() : FakeActionServer("WaitActionServer")
        {}
    }
    
    class BackUpActionServer : public FakeActionServer<nav2_msgs::action::BackUp>
    {
      public:
        BackUpActionServer() : FakeActionServer("BackUpActionServer")
        {}
    }

    class BehaviorTreeTester
    {
      public:
        using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
        using FollowPath = nav2_msgs::action::FollowPath;
        using Spin = nav2_msgs::action::Spin;
        using Wait = nav2_msgs::action::Wait;
        using BackUp = nav2_msgs::action::BackUp;

        BehaviorTreeTester();
        ~BehaviorTreeTester();

        void activate();

        void deactivate();
      
        bool isActive() const
        {
          return is_active_;
        }
      private:
        bool is_active_;

        rclcpp::Node::SharedPtr node_;

        // Action Client to call action (that will be faked)
        rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_path_to_pose_client_ptr_;
        rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_ptr_;
        rclcpp_action::Client<Spin>::SharedPtr spin_client_ptr_;
        rclcpp_action::Client<Wait>::SharedPtr wait_client_ptr_;
        rclcpp_action::Client<BackUp>::SharedPtr back_up_client_ptr_;
        // Do I need to create a "client" for the Clear CostMap service

        template <class ActionT>
        rclcpp_action::Client<ActionT>::SharedPtr create_client(string clientName);
    }

}  // namespace nav2_system_tests

#endif // BEHAVIOR_TREE_TESTER_HPP_