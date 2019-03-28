/**
 * @file optitrack.h
 * @brief ROS wrapper for spewing OptiTrack data
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 20 March 2019
 */

#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include "optitrack/optitrack_client.h"

namespace acl {
namespace optitrack {

  class OptiTrack
  {
  public:
    OptiTrack(const ros::NodeHandle nh);
    ~OptiTrack() = default;

    void spin();
    
  private:
    ros::NodeHandle nh_;

    std::string localIP_; ///< IP address of local NIC to use
    std::string serverIP_; ///< IP address of remote OptiTrack server
    std::string multicastGroup_; ///< Multicast group IP address

    std::unique_ptr<agile::OptiTrackClient> client_;

    /**
     * @brief      Convert from OptiTrack-Raw to ENU, with body flu.
     *
     * @param[in]  p     position array (x, y, z)
     * @param[in]  q     quaternion array (x, y, z, w)
     *
     * @return     Pose of rigid body (flu) w.r.t optitrack (ENU)
     */
    geometry_msgs::Pose toENUPose(const double* p, const double* q);
  };

}
}
