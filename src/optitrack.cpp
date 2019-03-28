/**
 * @file optitrack.cpp
 * @brief ROS wrapper for spewing OptiTrack data
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 20 March 2019
 */

#include "optitrack/optitrack.h"

namespace acl {
namespace optitrack {

OptiTrack::OptiTrack(const ros::NodeHandle nh)
  : nh_(nh)
{
  nh.getParam("local", localIP_);
  nh.getParam("server", serverIP_);

  client_ = std::make_unique<agile::OptiTrackClient>(localIP_, serverIP_);

  if (!client_->initConnection()) {
    ROS_ERROR("Could not initiate communication with OptiTrack server!");
    ros::shutdown();
  }
}

// ----------------------------------------------------------------------------

void OptiTrack::spin()
{
  // Keep track of ntime offset.
  int64_t offset_between_windows_and_linux = std::numeric_limits<int64_t>::max();

  // Some vars to calculate twist/acceleration and dts
  // Also keeps track of the various publishers
  std::map<int, ros::Publisher> rosPublishers;
  std::map<int, geometry_msgs::PoseStamped> pastStateMessages;

  int count_ = 0;
  int print_freq_ = 1500;

  bool time_set = false;

  while (ros::ok()) {
    //increase count and publish
    if (count_ % print_freq_ == 0) {
      std::cout<<"iter " + std::to_string(count_)<<std::endl;
    }

    // Wait for mocap Data packet
    client_->getDataPacket();
    if (count_ % print_freq_ == 0) {
      std::cout<<"Data received"<<std::endl;
    }    
    // Wait for data description     
    client_->getCommandPacket();
    if (count_ % print_freq_ == 0) {
      std::cout<<"CommandPacket received"<<std::endl;
    }

    
    //
    // Process OptiTrack "packets" (i.e., rigid bodies)
    //

    std::vector<agile::Packet> mocapPackets = client_->getPackets();
    for (const auto& pkt : mocapPackets) {

      // Skip this rigid body if tracking is invalid
      if (!pkt.tracking_valid) continue;

      // estimate the windows to linux constant offset by taking the minimum seen offset.
      // @TODO: Make offset a rolling average instead of a latching offset.
      int64_t offset = pkt.transmit_timestamp - pkt.receive_timestamp;
      if (offset < offset_between_windows_and_linux && !time_set){
        offset_between_windows_and_linux = offset;
        time_set = true;
      }
      uint64_t packet_ntime = pkt.mid_exposure_timestamp - offset_between_windows_and_linux;

      // Get past state and publisher (if they exist)
      ros::Publisher publisher;
      geometry_msgs::PoseStamped currentState;

      // Initialize publisher for rigid body if not exist.
      if (rosPublishers.find(pkt.rigid_body_id) == rosPublishers.end()) {
        // clean model name for ROS topic---only keep alphanumeric chars
        std::string name = pkt.model_name;
        name.erase(std::remove_if(name.begin(), name.end(),
              [](auto const& c) -> bool { return !std::isalnum(c); }
            ), name.end());

        std::string topic = "/" + name + "/optitrack";
        rosPublishers[pkt.rigid_body_id] = nh_.advertise<geometry_msgs::PoseStamped>(topic, 1);;
      }

      // Get saved publisher and last state
      publisher = rosPublishers[pkt.rigid_body_id];

      // Add timestamp
      currentState.header.stamp = ros::Time(packet_ntime/1e9, packet_ntime%(int64_t)1e9);

      // convert from raw optitrack frame to ENU with body flu
      currentState.pose = toENUPose(pkt.pos, pkt.orientation);

      // Loop through markers and convert positions from NUE to ENU
      // @TODO since the state message does not understand marker locations.

      // Save state for future acceleration and twist computations
      pastStateMessages[pkt.rigid_body_id] = currentState;

      // Publish ROS state.
      publisher.publish(currentState);

      ++count_;
    }
  }
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

geometry_msgs::Pose OptiTrack::toENUPose(const double* p, const double* q)
{
  // rigid body (dots) raw (Y-Up, "fur") w.r.t OptiTrack-Raw (Y-Up, "NUE")
  tf2::Transform T_ORBR;
  T_ORBR.setOrigin(tf2::Vector3(p[0], p[1], p[2]));
  T_ORBR.setRotation(tf2::Quaternion(q[0], q[1], q[2], q[3]));

  // transformation from fur to flu / NUE to NED (same for local / global)
  tf2::Transform T_BRB; // body (flu) w.r.t body-raw (fur)
  T_BRB.setIdentity();
  tf2::Quaternion quat; quat.setRPY(0, -M_PI/2, -M_PI/2);
  T_BRB.setRotation(quat);
  tf2::Transform T_OOR = T_BRB.inverse(); // optitrack-raw (NUE) w.r.t optitrack (ENU)

  // rigid body (dots) (flu) w.r.t OptiTrack frame (ROS ENU)
  tf2::Transform T_OB = T_OOR * T_ORBR * T_BRB;

  geometry_msgs::Pose pose;
  tf2::toMsg(T_OB, pose);
  return pose;
}

} // ns optitrack
} // ns acl
