#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

// Includes for node
#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Include motion capture framework
#include "optitrack/motionCaptureClientFramework.h"

using namespace Eigen;

// Used to convert mocap frame (NUE) to ROS ENU.
static Eigen::Matrix3d R_NUE2ENU = [] {
    Eigen::Matrix3d tmp;
    tmp <<  0, 0, 1,
            1, 0, 0,
            0, 1, 0;
    return tmp;
}();

Vector3d positionConvertNUE2ENU(double* positionNUE){
  Vector3d positionNUEVector, positionENUVector;
  positionNUEVector << positionNUE[0], positionNUE[1], positionNUE[2];

  positionENUVector = R_NUE2ENU * positionNUEVector;
  return positionENUVector;
}

Quaterniond quaternionConvertNUE2ENU(double* quaternionNUE){
    Quaterniond quaternionInNUE;
    quaternionInNUE.x() = quaternionNUE[0];
    quaternionInNUE.y() = quaternionNUE[1];
    quaternionInNUE.z() = quaternionNUE[2];
    quaternionInNUE.w() = quaternionNUE[3];

    Quaterniond quaternionInENU = Quaterniond(R_NUE2ENU * quaternionInNUE.normalized().toRotationMatrix()
                              * R_NUE2ENU.transpose());
    return quaternionInENU;
}

int main(int argc, char *argv[])
{
  // Keep track of ntime offset.
  int64_t offset_between_windows_and_linux = std::numeric_limits<int64_t>::max();

  // Init ROS
  ros::init(argc, argv, "optitrack");
  ros::NodeHandle n("~");

  // Get CMDline arguments for server and local IP addresses.
  std::string szMyIPAddress;
  std::string szServerIPAddress;

  n.getParam("local", szMyIPAddress);
  n.getParam("server", szServerIPAddress);

  // Init mocap framework
  agile::motionCaptureClientFramework mocap_ = agile::motionCaptureClientFramework(szMyIPAddress, szServerIPAddress);

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
    mocap_.getDataPacket();
    if (count_ % print_freq_ == 0) {
      std::cout<<"Data received"<<std::endl;
    }    
    // Wait for data description     
    mocap_.getCommandPacket();
    if (count_ % print_freq_ == 0) {
      std::cout<<"CommandPacket received"<<std::endl;
    }

    
    std::vector<agile::Packet> mocap_packets;
    mocap_packets = mocap_.getPackets();

    for (agile::Packet mocap_packet : mocap_packets) {

      // @TODO: Make getPackets return a list.

      // Skip this rigid body if tracking is invalid
      if (!mocap_packet.tracking_valid)
        continue;

      // estimate the windows to linux constant offset by taking the minimum seen offset.
      // @TODO: Make offset a rolling average instead of a latching offset.
      int64_t offset = mocap_packet.transmit_timestamp - mocap_packet.receive_timestamp;
      if (offset < offset_between_windows_and_linux && !time_set){
        offset_between_windows_and_linux = offset;
        time_set = true;
      }
      uint64_t packet_ntime = mocap_packet.mid_exposure_timestamp - offset_between_windows_and_linux;

      // Get past state and publisher (if they exist)
      bool hasPreviousMessage = (rosPublishers.find(mocap_packet.rigid_body_id) != rosPublishers.end());
      ros::Publisher publisher;
      geometry_msgs::PoseStamped lastState;
      geometry_msgs::PoseStamped currentState;

      // Initialize publisher for rigid body if not exist.
      if (!hasPreviousMessage){
        std::string topic = "/" + mocap_packet.model_name + "/optitrack";

        publisher = n.advertise<geometry_msgs::PoseStamped>(topic, 1);
        rosPublishers[mocap_packet.rigid_body_id] = publisher;
      } else {
        // Get saved publisher and last state
        publisher = rosPublishers[mocap_packet.rigid_body_id];
        lastState = pastStateMessages[mocap_packet.rigid_body_id];
      }

      // Add timestamp
      currentState.header.stamp = ros::Time(packet_ntime/1e9, packet_ntime%(int64_t)1e9);
      // currentState.header.stamp = ros::Time(mocap_packet.transmit_timestamp/1e9, mocap_packet.transmit_timestamp%(int64_t)1e9);

      // Convert rigid body position from NUE to ROS ENU
      Vector3d positionENUVector = positionConvertNUE2ENU(mocap_packet.pos);
      currentState.pose.position.x = positionENUVector(0);
      currentState.pose.position.y = positionENUVector(1);
      currentState.pose.position.z = positionENUVector(2);
      // Convert rigid body rotation from NUE to ROS ENU
      Quaterniond quaternionENUVector = quaternionConvertNUE2ENU(mocap_packet.orientation);
      currentState.pose.orientation.x = quaternionENUVector.x();
      currentState.pose.orientation.y = quaternionENUVector.y();
      currentState.pose.orientation.z = quaternionENUVector.z();
      currentState.pose.orientation.w = quaternionENUVector.w();

      // Loop through markers and convert positions from NUE to ENU
      // @TODO since the state message does not understand marker locations.

      // Save state for future acceleration and twist computations
      pastStateMessages[mocap_packet.rigid_body_id] = currentState;

      // Publish ROS state.
      publisher.publish(currentState);

    ++count_;
    }
  }
}
