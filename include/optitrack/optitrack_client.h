/**
 * MIT License
 * 
 * Copyright (c) 2018 AgileDrones
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Originally from: https://github.com/mit-fast/OptiTrack-Motive-2-Client
 */

#ifndef MOTIONCAPTURECLIENTFRAMEWORK_H
#define MOTIONCAPTURECLIENTFRAMEWORK_H

#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include <unordered_map>
#include <memory>
#include <unistd.h>
#include <linux/limits.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <boost/optional.hpp>

#include "optitrack/udp_socket.h"


/// @todo: varun move these defines to a enum
#define NAT_CONNECT                 0
#define NAT_SERVERINFO              1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100

#define MAX_PACKETSIZE              100000    // actual packet size is dynamic
#define MAX_NAMELENGTH              256

#define MULTICAST_ADDRESS       "239.255.42.99"
#define PORT_COMMAND            1510      // NatNet Command channel
#define PORT_DATA               1511      // NatNet Data channel


namespace agile {


    typedef struct {
        char szName[MAX_NAMELENGTH];            // sending app's name
        uint8_t Version[4];                     // [major.minor.build.revision]
        uint8_t NatNetVersion[4];               // [major.minor.build.revision]
    } sSender;

    typedef struct sSender_Server {
        sSender Common;
        // host's high resolution clock frequency (ticks per second)
        uint64_t HighResClockFrequency;
        uint16_t DataPort;
        bool IsMulticast;
        uint8_t MulticastGroupAddress[4];
    } sSender_Server;

    typedef struct {
        uint16_t iMessage;                      // message ID (e.g. NAT_FRAMEOFDATA)
        uint16_t nDataBytes;                    // Num bytes in payload
        union {
            uint8_t cData[MAX_PACKETSIZE];
            char szData[MAX_PACKETSIZE];
            uint32_t lData[MAX_PACKETSIZE / sizeof(uint32_t)];
            float fData[MAX_PACKETSIZE / sizeof(float)];
            sSender Sender;
            sSender_Server SenderServer;
        } Data;                                 // Payload incoming from NatNet Server
    } sPacket;

    struct Marker
    {
      int id;
      double x;
      double y;
      double z;
      double size;
      double residual;
    };

    struct Packet
    {
      int message_id;

      std::string model_name;
      int rigid_body_id;
      double pos[3]; // x, y, z
      double orientation[4]; // x, y, z, w
      std::vector<Marker> markers_;

      bool tracking_valid;
      float mean_marker_error;
      int labeled_marker_count;

      int frame_number;

      // NOTE: All are nanosecond timestamps
      uint64_t timestamp;
      uint64_t mid_exposure_timestamp;
      uint64_t camera_data_received_timestamp;
      uint64_t transmit_timestamp;
      // Calculated on receive.
      uint64_t receive_timestamp;
    };

class OptiTrackClient
{
  public:
    OptiTrackClient(const std::string& localIP, const std::string& serverIP,
                    const std::string& multicastGroupIP);

    // Starts connection to mocap and initializes settings.
    bool initConnection();

    // Blocking call that waits for a mocap packet to arrive. Then returns.
    void getDataPacket();

    void getCommandPacket();

    uint64_t getServerFrequency() {return server_frequency;};

    uint64_t getTimestamp() {return std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);}

    void setMulticastAddress (std::string address_) {multicast_address = address_;}

    std::vector<Packet> getPackets() {return processedPackets_;}

    bool IPAddress_StringToAddr(char *szNameOrAddress, struct in_addr *Address) const;

  private:
    // Sockets
    int CommandSocket;
    int dataSock_;
    in_addr ServerAddress;
    sockaddr_in HostAddr;
    const std::string localIP_;     ///< IP addr of local NIC to use.
    const std::string serverIP_;    ///< IP addr of server (for commands)
    const std::string multicastIP_; ///< Multicast group (for UDP data)

    std::unique_ptr<acl::utils::UDPSocket> datasock_; ///< UDP/Multicast socket for data
    std::unique_ptr<acl::utils::UDPSocket> cmdsock_; ///< UDP socket for commands

    sSender_Server serverInfo_; ///< The response of the inital connection request.

    // Versioning
    int NatNetVersion[4] = {3, 0, 0, 0};
    int ServerVersion[4] = {0, 0, 0, 0};

    // Command mode global variables
    int gCommandResponse = 0;
    int gCommandResponseSize = 0;
    unsigned char gCommandResponseString[PATH_MAX];
    void CommandListenThread();

    // Instance vars
    uint64_t server_frequency = 0;
    

    bool DecodeTimecode(unsigned int inTimecode,
                    unsigned int inTimecodeSubframe,
                    int *hour,
                    int *minute,
                    int *second,
                    int *frame,
                    int *subframe);

    // Takes timecode and assigns it to a string
    bool TimecodeStringify(unsigned int inTimecode,
                       unsigned int inTimecodeSubframe,
                       char *Buffer,
                       size_t BufferSize);

    void DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID);

    void Unpack (char *pData, std::vector<Packet> &outputs);

    int SendCommand (char *szCommand);

    bool getServerInfo(sSender_Server& serverInfo);

    std::unordered_map<int, std::string> rigid_body_map;

    std::string multicast_address;

    std::vector<Packet> processedPackets_;
    Packet output_packet_;

};
}

#endif // MOTIONCAPTURECLIENTFRAMEWORK_H
