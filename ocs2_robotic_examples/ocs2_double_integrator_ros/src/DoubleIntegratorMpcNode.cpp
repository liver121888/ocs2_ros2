/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ocs2_double_integrator/DoubleIntegratorInterface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  const std::string robotName = "double_integrator";
  using interface_t = ocs2::double_integrator::DoubleIntegratorInterface;
  using mpc_ros_t = ocs2::MPC_ROS_Interface;

  // task file
  std::vector<std::string> programArgs =
      rclcpp::remove_ros_arguments(argc, argv);

  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_mpc");

  // Robot interface
  const std::string taskFile =
      ament_index_cpp::get_package_share_directory("ocs2_double_integrator") +
      "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder =
      ament_index_cpp::get_package_share_directory("ocs2_double_integrator") +
      "/auto_generated";
  interface_t doubleIntegratorInterface(taskFile, libFolder);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(
      robotName, doubleIntegratorInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(node);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(
      doubleIntegratorInterface.mpcSettings(),
      doubleIntegratorInterface.ddpSettings(),
      doubleIntegratorInterface.getRollout(),
      doubleIntegratorInterface.getOptimalControlProblem(),
      doubleIntegratorInterface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Launch MPC ROS node
  mpc_ros_t mpcNode(mpc, robotName);
  mpcNode.launchNodes(node);

  // Successful exit
  return 0;
}
