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

#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"
#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"
#include "nav2_behavior_tree/plugins/action/clear_costmap_service.hpp"
#include "nav2_behavior_tree/plugins/action/spin_action.hpp"
#include "nav2_behavior_tree/plugins/action/wait_action.hpp"
#include "nav2_behavior_tree/plugins/action/back_up_action.hpp"

namespace nav2_system_tests
{

}  // namespace nav2_system_tests

#endif // BEHAVIOR_TREE_TESTER_HPP_