// Copyright (c) 2020 Samsung Research
// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2020 Vinny Ruia
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

#include <cmath>
#include <tuple>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"

#include "behavior_tree_tester.hpp"

using namespace std::chrono_literals;

using nav2_system_tests::BehaviorTreeTester;

struct should_action_server_return_success_t {
  bool compute_path_to_pose = true;
  bool follow_path = true;
  bool wait = true;
  bool back_up = true;
  bool spin = true;
} 

std::string testNameGenerator(const testing::TestParamInfo<std::tuple<float, float>> & param)
{
  std::string name = std::to_string(std::abs(std::get<0>(param.param))) + "_" + std::to_string(
    std::get<1>(param.param));
  name.erase(std::remove(name.begin(), name.end(), '.'), name.end());
  return name;
}

class BehaviorTreeTestFixture
  : public :: testing::TestWithParam<std::tuple<float,float>>
{
public:
  static void SetUpTestCase()
  {
    behavior_tree_tester =  new BehaviorTreeTester();
    if (!behavior_tree_Tester->isActive()) {
      behavior_tree_tester->activate();
    }
  }
}

  static void TearDownTestCase()
  {
    delete behavior_tree_tester;
    behavior_tree_tester = nullptr;
  }

protected:
  static BehaviorTreeTester * behavior_tree_tester;
};

BehaviorTreeTester * BehaviorTreeTestFixture::behavior_tree_tester = nullptr;

TEST_P(BehaviorTreeTestFixture, testBehaviorTree)
{

}

INSTANTIATE_TEST_SUITE_P(
  BehaviorTreeTests,
  BehaviorTreeTestFixture,
  ::testing::Values(
  ),
  testNameGenerator);
)

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}