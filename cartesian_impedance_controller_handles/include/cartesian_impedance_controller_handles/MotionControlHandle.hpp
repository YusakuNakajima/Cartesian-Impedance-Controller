////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    MotionControlHandle.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2018/06/20
 * Modified for cartesian_impedance_controller_handles
 *
 */
//-----------------------------------------------------------------------------

// Project
#include <cartesian_impedance_controller_handles/MotionControlHandle.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// URDF
#include <urdf/model.h>

// ROS
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>

namespace cartesian_impedance_controller_handles
{

template <class HardwareInterface>
MotionControlHandle<HardwareInterface>::
MotionControlHandle()
 : m_stiffness_matrix(Eigen::Matrix<double, 6, 6>::Identity() * 200.0),
   m_damping_matrix(Eigen::Matrix<double, 6, 6>::Identity() * 1.0)
{
}

template <class HardwareInterface>
MotionControlHandle<HardwareInterface>::
~MotionControlHandle()
{
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
starting(const ros::Time& time)
{
  m_current_pose = getEndEffectorPose();
  m_server->setPose(m_marker.name, m_current_pose.pose);
  m_server->applyChanges();
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
stopping(const ros::Time& time)
{
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Publish marker pose
  m_current_pose.header.stamp = time;
  m_current_pose.header.frame_id = m_robot_base_link;
  m_pose_publisher.publish(m_current_pose);
  m_server->applyChanges();
}

template <class HardwareInterface>
bool MotionControlHandle<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  std::string robot_description;
  urdf::Model robot_model;
  KDL::Tree   robot_tree;

  // Get joint names for this controller
  if (!nh.getParam("joints", m_joint_names))
  {
    ROS_ERROR("Failed to load joint names");
    return false;
  }

  // Get base and end-effector links
  if (!nh.getParam("robot_base_link", m_robot_base_link))
  {
    ROS_ERROR("Failed to load robot_base_link");
    return false;
  }

  if (!nh.getParam("end_effector_link", m_end_effector_link))
  {
    ROS_ERROR("Failed to load end_effector_link");
    return false;
  }

  // Get target frame topic
  if (!nh.getParam("target_frame_topic", m_target_frame_topic))
  {
    m_target_frame_topic = "target_frame";
  }

  // Get joint handles
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    try
    {
      m_joint_handles.push_back(hw->getHandle(m_joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }

  // Get robot description
  if (!nh.getParam("/robot_description", robot_description))
  {
    ROS_ERROR("Failed to load robot description");
    return false;
  }

  // Parse URDF and create KDL tree
  if (!robot_model.initString(robot_description))
  {
    ROS_ERROR("Failed to parse URDF");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    ROS_ERROR("Failed to construct KDL tree");
    return false;
  }

  // Extract chain from tree
  if (!robot_tree.getChain(m_robot_base_link, m_end_effector_link, m_robot_chain))
  {
    ROS_ERROR_STREAM("Failed to get kinematic chain from "
                     << m_robot_base_link << " to " << m_end_effector_link);
    return false;
  }

  // Create FK solver
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_robot_chain));

  // Create publisher
  m_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(m_target_frame_topic, 10);

  // Setup interactive marker server
  m_server.reset(new interactive_markers::InteractiveMarkerServer("impedance_motion_control_handle"));

  // Create marker
  m_marker.header.frame_id = m_robot_base_link;
  m_marker.name = "impedance_handle";
  m_marker.description = "Impedance Control Handle";
  m_marker.scale = 0.2;

  prepareMarkerControls(m_marker);

  // Register callback
  m_server->insert(m_marker,
      boost::bind(&MotionControlHandle::updateMotionControlCallback, this, _1));

  // Apply changes
  m_server->applyChanges();

  return true;
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
updateMotionControlCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    m_current_pose.pose = feedback->pose;
  }
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
updateMarkerMenuCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Handle menu callbacks for impedance parameters
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
prepareMarkerControls(visualization_msgs::InteractiveMarker& marker)
{
  // Add axis controls
  addAxisControl(marker, 1, 0, 0); // X-axis
  addAxisControl(marker, 0, 1, 0); // Y-axis
  addAxisControl(marker, 0, 0, 1); // Z-axis

  // Add visualization
  addMarkerVisualization(marker, 0.1);
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
addAxisControl(visualization_msgs::InteractiveMarker& marker, double x, double y, double z)
{
  visualization_msgs::InteractiveMarkerControl control;
  
  control.always_visible = true;
  control.orientation.w = 1;
  control.orientation.x = x;
  control.orientation.y = y;
  control.orientation.z = z;
  
  // Move control
  control.name = "move_" + std::to_string(x + y + z);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);
  
  // Rotate control
  control.name = "rotate_" + std::to_string(x + y + z);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
addMarkerVisualization(visualization_msgs::InteractiveMarker& marker, double scale)
{
  visualization_msgs::InteractiveMarkerControl control;
  visualization_msgs::Marker sphere;
  
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.scale.x = scale;
  sphere.scale.y = scale;
  sphere.scale.z = scale;
  sphere.color.r = 0.5;
  sphere.color.g = 0.5;
  sphere.color.b = 0.5;
  sphere.color.a = 0.8;
  
  control.always_visible = true;
  control.markers.push_back(sphere);
  marker.controls.push_back(control);
}

template <class HardwareInterface>
geometry_msgs::PoseStamped MotionControlHandle<HardwareInterface>::
getEndEffectorPose()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = m_robot_base_link;
  
  // Get current joint positions
  KDL::JntArray joint_positions(m_joint_handles.size());
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    joint_positions(i) = m_joint_handles[i].getPosition();
  }
  
  // Compute forward kinematics
  KDL::Frame frame;
  if (m_fk_solver->JntToCart(joint_positions, frame) >= 0)
  {
    tf::poseKDLToMsg(frame, pose.pose);
  }
  
  return pose;
}

} // namespace cartesian_impedance_controller_handles