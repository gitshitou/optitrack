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

#include <ros/ros.h>

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

    std::string localIP_; ///<
    std::string serverIP_; ///< IP address of remote OptiTrack server

    std::unique_ptr<agile::OptiTrackClient> client_;
  };

}
}
