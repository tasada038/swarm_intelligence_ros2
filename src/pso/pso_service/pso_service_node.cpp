// Copyright (c) 2024 Takumi Asada
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

#include "rclcpp/rclcpp.hpp"
#include "swarm_intelligence_ros2/pso/pso_service/pso_service_component.hpp"

int main(int argc, char **argv)
{
	srand((unsigned int)time(NULL));

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<PsoComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}