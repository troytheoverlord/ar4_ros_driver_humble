#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

static void plan_and_execute(moveit::planning_interface::MoveGroupInterface& mgi,
                             rclcpp::Logger logger,
                             const std::string& label)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_code = mgi.plan(plan);
  if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Planning failed: " + label);
  }
  auto exec_code = mgi.execute(plan);
  if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
    throw std::runtime_error("Execution failed: " + label);
  }
  RCLCPP_INFO(logger, "Done: %s", label.c_str());
}

static void set_joint_target(moveit::planning_interface::MoveGroupInterface& arm,
                             const std::vector<std::string>& joint_names,
                             const std::vector<double>& joint_values,
                             const std::string& label)
{
  if (joint_names.size() != joint_values.size()) {
    throw std::runtime_error("Joint name/value size mismatch for " + label);
  }
  std::map<std::string, double> target;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    target[joint_names[i]] = joint_values[i];
  }
  arm.setJointValueTarget(target);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ar4_moveit_simple_pick");
  auto log = node->get_logger();

  moveit::planning_interface::MoveGroupInterface arm(node, "ar_manipulator");
  moveit::planning_interface::MoveGroupInterface gripper(node, "ar_gripper");

  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  gripper.setPlanningTime(2.0);
  gripper.setNumPlanningAttempts(5);
  gripper.setMaxVelocityScalingFactor(1.0);
  gripper.setMaxAccelerationScalingFactor(1.0);

  const std::vector<std::string> arm_joints = {
    "joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"
  };

  // Movement 1: Pick and Place
  const std::vector<double> safe = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> pick = {0.0, 1.0647, 0.1047, 0.0, 0.4014, 0.0};

  // Movement 2: Sweep (left to right arc)
  const std::vector<double> sweep_left  = {0.5411, 1.5533, -1.2566, -1.0996, -0.6109, 1.0123};
  const std::vector<double> sweep_right = {-0.5411, 1.5708, -1.2740, 1.1170, -0.6109, -1.0297};

  // Movement 3: Stacking sequence (3 positions)
  const std::vector<double> stack_a = {-1.8151, 1.5533, -1.2043, 1.6581, -1.7977, -1.2217};
  const std::vector<double> stack_b = {2.1817, 1.1170, 0.0, -2.1293, -1.8151, 0.3840};
  const std::vector<double> stack_c = {-0.0349, 1.5010, -1.0996, 0.0873, -0.4189, 0.0873};

  try {
    // ---- Movement 1: Pick and Place ----
    RCLCPP_INFO(log, "=== Movement 1: Pick and Place ===");
    gripper.setNamedTarget("open");
    plan_and_execute(gripper, log, "gripper open");

    set_joint_target(arm, arm_joints, safe, "SAFE");
    plan_and_execute(arm, log, "arm SAFE");

    set_joint_target(arm, arm_joints, pick, "PICK");
    plan_and_execute(arm, log, "arm PICK");

    gripper.setNamedTarget("closed");
    plan_and_execute(gripper, log, "gripper closed");

    set_joint_target(arm, arm_joints, safe, "SAFE return");
    plan_and_execute(arm, log, "arm SAFE return");

    set_joint_target(arm, arm_joints, pick, "PICK return");
    plan_and_execute(arm, log, "arm PICK return");

    gripper.setNamedTarget("open");
    plan_and_execute(gripper, log, "gripper open end");

    set_joint_target(arm, arm_joints, safe, "SAFE final");
    plan_and_execute(arm, log, "arm SAFE final");

    // ---- Movement 2: Sweep ----
    RCLCPP_INFO(log, "=== Movement 2: Sweep ===");
    set_joint_target(arm, arm_joints, sweep_left, "sweep left");
    plan_and_execute(arm, log, "arm sweep left");

    set_joint_target(arm, arm_joints, sweep_right, "sweep right");
    plan_and_execute(arm, log, "arm sweep right");

    set_joint_target(arm, arm_joints, sweep_left, "sweep left return");
    plan_and_execute(arm, log, "arm sweep left return");

    set_joint_target(arm, arm_joints, safe, "SAFE after sweep");
    plan_and_execute(arm, log, "arm SAFE after sweep");

    // ---- Movement 3: Stacking ----
    RCLCPP_INFO(log, "=== Movement 3: Stacking ===");
    set_joint_target(arm, arm_joints, stack_a, "stack position A");
    plan_and_execute(arm, log, "arm stack A");

    set_joint_target(arm, arm_joints, stack_b, "stack position B");
    plan_and_execute(arm, log, "arm stack B");

    set_joint_target(arm, arm_joints, stack_c, "stack position C");
    plan_and_execute(arm, log, "arm stack C");

    set_joint_target(arm, arm_joints, safe, "SAFE after stack");
    plan_and_execute(arm, log, "arm SAFE after stack");

    RCLCPP_INFO(log, "All movements complete.");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(log, "ERROR: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
