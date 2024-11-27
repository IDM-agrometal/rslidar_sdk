/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>

using namespace robosense::lidar;

class RSLidarSDKNode : public rclcpp::Node
{
public:
  RSLidarSDKNode(const rclcpp::NodeOptions& options)
      : Node("rslidar_sdk_node", options)
   {
     // Log driver version
     RCLCPP_INFO(this->get_logger(),
                 "********************************************************");
     RCLCPP_INFO(this->get_logger(),
                 "**********    RSLidar_SDK Version: v%d.%d.%d     **********",
                 RSLIDAR_VERSION_MAJOR, RSLIDAR_VERSION_MINOR, RSLIDAR_VERSION_PATCH);
     RCLCPP_INFO(this->get_logger(),
                 "********************************************************");
      // Get configuration path
     std::string config_path = this->declare_parameter<std::string>("config_path", "");
     if (config_path.empty())
     {
 #ifdef RUN_IN_ROS_WORKSPACE
       config_path = ament_index_cpp::get_package_share_directory("rslidar_sdk") + "/config/config.yaml";
 #else
       config_path = std::string(PROJECT_PATH) + "/config/config.yaml";
 #endif
     }
      // Load configuration file
     YAML::Node config;
     try
     {
       config = YAML::LoadFile(config_path);
     }
     catch (const std::exception& e)
     {
       RCLCPP_ERROR(this->get_logger(), "Error loading config file '%s': %s", config_path.c_str(), e.what());
       throw;
     }
      // Initialize NodeManager
     node_manager_ = std::make_shared<NodeManager>();
     node_manager_->init(config);
     node_manager_->start();
     RCLCPP_INFO(this->get_logger(), "RoboSense-LiDAR-Driver is running...");
   }
    ~RSLidarSDKNode()
   {
     node_manager_->stop();
     RCLCPP_INFO(this->get_logger(), "RoboSense-LiDAR-Driver is stopping...");
   }
  private:
   std::shared_ptr<NodeManager> node_manager_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RSLidarSDKNode);
