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

#include "optitrack/optitrack_client.h"

namespace agile {

OptiTrackClient::OptiTrackClient(const std::string& localIP,
                                 const std::string& serverIP,
                                 const std::string& multicastGroupIP)
: localIP_(localIP), serverIP_(serverIP), multicastIP_(multicastGroupIP)
{

}

// ----------------------------------------------------------------------------

bool OptiTrackClient::initConnection() {
  
  cmdsock_.reset(new acl::utils::UDPSocket(localIP_, PORT_DATA));
  cmdsock_->setReceiveTimeout(1); // 1 sec

  datasock_.reset(new acl::utils::UDPSocket(localIP_, PORT_COMMAND));
  datasock_->setReceiveTimeout(1); // 1 sec
  datasock_->joinMulticastGroup(multicastIP_);

  // attempt to connect to the server to retrieve basic info
  return getServerInfo(serverInfo_);
}

// ----------------------------------------------------------------------------

void OptiTrackClient::getDataPacket()
{
  char szData[20000];
  socklen_t addr_len = sizeof(struct sockaddr);
  sockaddr_in TheirAddress{};

  // Block until we receive a datagram from the network
  // (from anyone including ourselves)
  //ssize_t nDataBytesReceived =
  recvfrom(dataSock_,
           szData,
           sizeof(szData),
           0,
           (sockaddr *) &TheirAddress,
           &addr_len);
  // Once we have bytes recieved Unpack organizes all the data
  //
  // Clear vector of previous packets.
  processedPackets_.clear();
  Unpack(szData, processedPackets_);

  /*if (outputs.size() > 0) {
      PublishPacketRos(outputs[0]);
    }*/
}

// ----------------------------------------------------------------------------

void OptiTrackClient::getCommandPacket()
{
    agile::sPacket PacketOut{};
    PacketOut.iMessage = NAT_REQUEST_MODELDEF;  //NAT_FRAMEOFDATA;     //NAT_REQUEST_MODELDEF;
    PacketOut.nDataBytes = 0;
    ssize_t iRet = sendto(CommandSocket,
                        (char *) &PacketOut,
                        4 + PacketOut.nDataBytes,
                        0,
                        (sockaddr *) &HostAddr,
                        sizeof(HostAddr));
      // Wait for server response. 
      // This will contain the server tick frequency.
    char ip_as_str[INET_ADDRSTRLEN];
    ssize_t nDataBytesReceived;
    sockaddr_in TheirAddress{};
    agile::sPacket PacketIn{};
    socklen_t addr_len = sizeof(struct sockaddr);
    char szData[20000]; //@Todo: this value is hard-coded
    nDataBytesReceived = recvfrom(CommandSocket,
                                    szData,
                                    sizeof(szData),
                                    0,
                                    (struct sockaddr *) &TheirAddress,
                                    &addr_len);

      // debug - print message
      
/*      inet_ntop(AF_INET, &(TheirAddress.sin_addr), ip_as_str, INET_ADDRSTRLEN);
      printf("[Client] Received command from %s: iMessage=%d, nDataBytes=%d\n",
            ip_as_str, (int) PacketIn.iMessage, (int) PacketIn.nDataBytes);*/
      // processedPackets_.clear();
      Unpack(szData, processedPackets_);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool OptiTrackClient::getServerInfo(sSender_Server& serverInfo)
{
  constexpr int MAX_NUM_TRIES = 3;

  // attempt to send the connection request to the server nrTries times.
  int nrTries = MAX_NUM_TRIES;
  while (nrTries--) {

    //
    // send a message through the socket (non-blocking)
    // 

    {
      // n.b.: the 4 is the size of the packet "header" (iMessage + nDataBytes)
      // and the nDataBytes is the actual size of the payload
      // create a packet with a connection request message
      sPacket pkt{};
      pkt.iMessage = NAT_CONNECT;
      pkt.nDataBytes = 0;
      size_t pktlen = pkt.nDataBytes + 4;

      bool sent = cmdsock_->send(serverIP_, PORT_COMMAND, (char *)&pkt, pktlen);
      if (!sent) return false;
    }

    std::cout << "[OptiTrackClient] Attempting to connect "
                 "to OptiTrack server..." << std::flush;

    {
      sPacket pkt{};

      // wait (with timeout) for server response
      bool recvd = cmdsock_->receive((char *)&pkt, sizeof(pkt));

      if (!recvd) {
        std::cout << "timed out." << std::endl;
        continue;
      } else {
        std::cout << "done!" << std::endl;
      }

      // unsigned char *ptr = (unsigned char *) &pkt;
      // agile::sSender_Server *server_info = (agile::sSender_Server *) (ptr + 4);
      
      serverInfo = pkt.Data.SenderServer;
    }

    return true;
  }

  // number of tries exceeded
  return false;
}

// ----------------------------------------------------------------------------

// Funtion that assigns a time code values to 5 variables passed as arguments
// Requires an integer from the packet as the timecode and timecodeSubframe
bool OptiTrackClient::DecodeTimecode(unsigned int inTimecode,
                                     unsigned int inTimecodeSubframe,
                                     int *hour, int *minute, int *second,
                                     int *frame, int *subframe)
{
  bool bValid = true;

  *hour = (inTimecode >> 24) & 255;
  *minute = (inTimecode >> 16) & 255;
  *second = (inTimecode >> 8) & 255;
  *frame = inTimecode & 255;
  *subframe = inTimecodeSubframe;

  return bValid;
}

// ----------------------------------------------------------------------------

// Takes timecode and assigns it to a string
bool OptiTrackClient::TimecodeStringify(unsigned int inTimecode,
                                        unsigned int inTimecodeSubframe,
                                        char *Buffer, size_t BufferSize)
{
  bool bValid = false;
  int hour, minute, second, frame, subframe;
  bValid = DecodeTimecode(inTimecode, inTimecodeSubframe,
                          &hour, &minute, &second, &frame, &subframe);

  snprintf(Buffer, BufferSize, "%2d:%2d:%2d:%2d.%d",
           hour, minute, second, frame, subframe);
  for (unsigned int i=0; i<strlen(Buffer); i++)
    if (Buffer[i] == ' ')
      Buffer[i] = '0';

  return bValid;
}

// ----------------------------------------------------------------------------

void OptiTrackClient::DecodeMarkerID(int sourceID, int *pOutEntityID, int *pOutMemberID)
{
  if (pOutEntityID)
    *pOutEntityID = sourceID >> 16;

  if (pOutMemberID)
    *pOutMemberID = sourceID & 0x0000ffff;
}

// ----------------------------------------------------------------------------

// Send a command to Motive.
int OptiTrackClient::SendCommand(char *szCommand) {
  // reset global result
  gCommandResponse = -1;

  // format command packet
  sPacket commandPacket{};
  strcpy(commandPacket.Data.szData, szCommand);
  commandPacket.iMessage = NAT_REQUEST;
  commandPacket.nDataBytes =
      (unsigned short) (strlen(commandPacket.Data.szData) + 1);

  // send command and wait (a bit)
  // for command response to set global response var in CommandListenThread
  ssize_t iRet = sendto(CommandSocket,
                        (char *) &commandPacket,
                        4 + commandPacket.nDataBytes,
                        0,
                        (sockaddr *) &HostAddr,
                        sizeof(HostAddr));
  if (iRet == -1) {
    printf("Socket error sending command");
  } else {
    int waitTries = 5;
    while (waitTries--) {
      if (gCommandResponse != -1)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    if (gCommandResponse == -1) {
      printf("Command response not received (timeout)");
    } else if (gCommandResponse == 0) {
      printf("Command response received with success");
    } else if (gCommandResponse > 0) {
      printf("Command response received with errors");
    }
  }

  return gCommandResponse;
}

// ----------------------------------------------------------------------------

// Command response listener thread
void OptiTrackClient::CommandListenThread() {
  char ip_as_str[INET_ADDRSTRLEN];
  ssize_t nDataBytesReceived;
  sockaddr_in TheirAddress{};
  sPacket PacketIn{};
  socklen_t addr_len = sizeof(struct sockaddr);

  while (true) {
    // blocking
    nDataBytesReceived = recvfrom(CommandSocket,
                                  (char *)&PacketIn,
                                  sizeof(sPacket),
                                  0,
                                  (struct sockaddr *)&TheirAddress,
                                  &addr_len);

    if ((nDataBytesReceived == 0) || (nDataBytesReceived == -1))
      continue;

    // debug - print message
    inet_ntop(AF_INET, &(TheirAddress.sin_addr), ip_as_str, INET_ADDRSTRLEN);
    printf("[Client] Received command from %s: Command=%d, nDataBytes=%d\n",
           ip_as_str, (int) PacketIn.iMessage, (int) PacketIn.nDataBytes);

    unsigned char *ptr = (unsigned char *) &PacketIn;
    sSender_Server *server_info = (sSender_Server *) (ptr + 4);

    std::cout << "server tick frequency: " << server_info->HighResClockFrequency << std::endl;

     std::vector<Packet> outputs;

    // handle command
    switch (PacketIn.iMessage) {
      case NAT_MODELDEF:
        std::cout << "[Client] Received NAT_MODELDEF packet";
        Unpack((char *) &PacketIn, outputs);
        break;
      case NAT_FRAMEOFDATA:
        std::cout << "[Client] Received NAT_FRAMEOFDATA packet";
        Unpack((char *) &PacketIn, outputs);
        break;
      case NAT_SERVERINFO:
        // Streaming app's name, e.g., Motive
        std::cout << server_info->Common.szName << " ";
        // Streaming app's version, e.g., 2.0.0.0
        for (int i = 0; i < 4; ++i) {
          std::cout << static_cast<int>(server_info->Common.Version[i]) << ".";
        }
        std::cout << '\b' << std::endl;
        // Streaming app's NatNet version, e.g., 3.0.0.0
        std::cout << "NatNet ";
        int digit;
        for (int i = 0; i < 4; ++i) {
          digit = static_cast<int>(server_info->Common.NatNetVersion[i]);
          std::cout << digit << ".";
        }
        std::cout << '\b' << std::endl;
        // Save versions in global variables
        for (int i = 0; i < 4; i++) {
          NatNetVersion[i] = server_info->Common.NatNetVersion[i];
          ServerVersion[i] = server_info->Common.Version[i];
        }
        break;
      case NAT_RESPONSE:
        gCommandResponseSize = PacketIn.nDataBytes;
        if (gCommandResponseSize == 4)
          memcpy(&gCommandResponse,
                 &PacketIn.Data.lData[0],
                 gCommandResponseSize);
        else {
          memcpy(&gCommandResponseString[0],
                 &PacketIn.Data.cData[0],
                 gCommandResponseSize);
          printf("Response : %s", gCommandResponseString);
          gCommandResponse = 0;   // ok
        }
        break;
      case NAT_UNRECOGNIZED_REQUEST:
        printf("[Client] received 'unrecognized request'\n");
        gCommandResponseSize = 0;
        gCommandResponse = 1;       // err
        break;
      case NAT_MESSAGESTRING:
        printf("[Client] Received message: %s\n",
               PacketIn.Data.szData);
        break;
    }
  }
}

// ----------------------------------------------------------------------------

// Convert IP address string to address
bool OptiTrackClient::IPAddress_StringToAddr(char *szNameOrAddress,
                            struct in_addr *Address) const{
  int retVal;
  struct sockaddr_in saGNI;
  char hostName[256];
  char servInfo[256];
  u_short port;
  port = 0;

  // Set up sockaddr_in structure which is passed to the getnameinfo function
  saGNI.sin_family = AF_INET;
  saGNI.sin_addr.s_addr = inet_addr(szNameOrAddress);
  saGNI.sin_port = htons(port);

  // getnameinfo in WS2tcpip is protocol independent
  // and resolves address to ANSI host name
  if ((retVal = getnameinfo((sockaddr *) &saGNI, sizeof(sockaddr), hostName,
                            256, servInfo, 256, NI_NUMERICSERV)) != 0) {
    // Returns error if getnameinfo failed
    printf("[PacketClient] GetHostByAddr failed\n");
    return false;
  }

  Address->s_addr = saGNI.sin_addr.s_addr;
  return true;
}

// ----------------------------------------------------------------------------

void OptiTrackClient::Unpack(char *pData, std::vector<Packet> &outputs) {
  // Checks for NatNet Version number. Used later in function.
  // Packets may be different depending on NatNet version.
  int major = NatNetVersion[0];
  int minor = NatNetVersion[1];

  char *ptr = pData;

  // printf("Begin Packet\n-------\n");

  uint64_t receiveTimestamp = getTimestamp();
  // First 2 Bytes is message ID
  int MessageID = 0;
  memcpy(&MessageID, ptr, 2);
  ptr += 2;


  // Second 2 Bytes is the size of the packet
  int nBytes = 0;
  memcpy(&nBytes, ptr, 2);
  ptr += 2;

  // printf("Message ID: %d\n", MessageID);
  
  // std::cout <<"MessageID= "<<MessageID<<std::endl;
  if (MessageID == 7)      // FRAME OF MOCAP DATA packet
  {
    // printf("Starting mocap data packet\n");
    // Next 4 Bytes is the frame number
    int frameNumber = 0;
    memcpy(&frameNumber, ptr, 4);
    ptr += 4;
    // printf("Frame # : %d\n", frameNumber);


    // Next 4 Bytes is the number of data sets (markersets, rigidbodies, etc)
    int nMarkerSets = 0;
    memcpy(&nMarkerSets, ptr, 4);
    ptr += 4;

    // Loop through number of marker sets and get name and data
    for (int i = 0; i < nMarkerSets; i++) {
      // Markerset name
      char szName[256];
      strcpy(szName, ptr);
      int nDataBytes = (int) strlen(szName) + 1;
      ptr += nDataBytes;

      // marker data
      int nMarkers = 0;
      memcpy(&nMarkers, ptr, 4);
      ptr += 4;
      // printf("Marker Count : %d\n", nMarkers);

      for (int j = 0; j < nMarkers; j++) {
        float x = 0;
        memcpy(&x, ptr, 4);
        ptr += 4;
        float y = 0;
        memcpy(&y, ptr, 4);
        ptr += 4;
        float z = 0;
        memcpy(&z, ptr, 4);
        ptr += 4;
        // printf("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, x, y, z);
      }
    }

    // Loop through unlabeled markers
    int nOtherMarkers = 0;
    memcpy(&nOtherMarkers, ptr, 4);
    ptr += 4;
    // OtherMarker list is Deprecated
    // // printf("Unidentified Marker Count : %d\n", nOtherMarkers);
    for (int j = 0; j < nOtherMarkers; j++) {
      float x = 0.0f;
      memcpy(&x, ptr, 4);
      ptr += 4;
      float y = 0.0f;
      memcpy(&y, ptr, 4);
      ptr += 4;
      float z = 0.0f;
      memcpy(&z, ptr, 4);
      ptr += 4;

      // Deprecated
      // // printf("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n",j,x,y,z);
    }

    // Loop through rigidbodies
    int nRigidBodies = 0;
    memcpy(&nRigidBodies, ptr, 4);
    ptr += 4;

    // TODO create one output packet per rigid body
    // printf("Rigid Body Count : %d\n", nRigidBodies);
    for (int j = 0; j < nRigidBodies; j++) {
      // Rigid body position and orientation
      int ID = 0;
      memcpy(&ID, ptr, 4);
      ptr += 4;
      // printf("Rigid body ID: %d\n", ID);
      float x = 0.0f;
      memcpy(&x, ptr, 4);
      ptr += 4;
      float y = 0.0f;
      memcpy(&y, ptr, 4);
      ptr += 4;
      float z = 0.0f;
      memcpy(&z, ptr, 4);
      ptr += 4;
      float qx = 0;
      memcpy(&qx, ptr, 4);
      ptr += 4;
      float qy = 0;
      memcpy(&qy, ptr, 4);
      ptr += 4;
      float qz = 0;
      memcpy(&qz, ptr, 4);
      ptr += 4;
      float qw = 0;
      memcpy(&qw, ptr, 4);
      ptr += 4;
      // printf("ID : %d\n", ID);
      // printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
      // printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

      output_packet_ = Packet();
      output_packet_.receive_timestamp = receiveTimestamp;
      output_packet_.message_id = MessageID;
      output_packet_.frame_number = frameNumber;

      output_packet_.rigid_body_id = ID;
      output_packet_.pos[0] = x;
      output_packet_.pos[1] = y;
      output_packet_.pos[2] = z;
      output_packet_.orientation[0] = qx;
      output_packet_.orientation[1] = qy;
      output_packet_.orientation[2] = qz;
      output_packet_.orientation[3] = qw;

      // Before NatNet 3.0, marker data was here
      if (major < 3) {
        // associated marker positions
        int nRigidMarkers = 0;
        memcpy(&nRigidMarkers, ptr, 4);
        ptr += 4;
        // printf("Marker Count: %d\n", nRigidMarkers);
        int nBytes = nRigidMarkers * 3 * sizeof(float);
        float *markerData = (float *) malloc(nBytes);
        memcpy(markerData, ptr, nBytes);
        ptr += nBytes;

        if (major >= 2) {
          // associated marker IDs
          nBytes = nRigidMarkers * sizeof(int);
          int *markerIDs = (int *) malloc(nBytes);
          memcpy(markerIDs, ptr, nBytes);
          ptr += nBytes;

          // associated marker sizes
          nBytes = nRigidMarkers * sizeof(float);
          float *markerSizes = (float *) malloc(nBytes);
          memcpy(markerSizes, ptr, nBytes);
          ptr += nBytes;

          for (int k = 0; k < nRigidMarkers; k++) {
            // printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n",
                   // k,
                   // markerIDs[k],
                   // markerSizes[k],
                   // markerData[k * 3],
                   // markerData[k * 3 + 1],
                   // markerData[k * 3 + 2]);
          }

          if (markerIDs)
            free(markerIDs);
          if (markerSizes)
            free(markerSizes);

        } else {
          for (int k = 0; k < nRigidMarkers; k++) {
            // printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k,
                   // markerData[k * 3], markerData[k * 3 + 1],
                   // markerData[k * 3 + 2]);
          }
        }
        if (markerData)
          free(markerData);
      }

      // NatNet version 2.0 and later
      if (major >= 2) {
        // Mean marker error
        float fError = 0.0f;
        memcpy(&fError, ptr, 4);
        ptr += 4;
        // printf("Mean marker error: %3.2f\n", fError);

        output_packet_.mean_marker_error = fError;
      }

      // NatNet version 2.6 and later
      if (((major == 2) && (minor >= 6)) || (major > 2)) {
        // params
        short params = 0;
        memcpy(&params, ptr, 2);
        ptr += 2;
        // 0x01 : rigid body was successfully tracked in this frame
        bool bTrackingValid = params & 0x01;

        // todo make this an int so it's obvious when it's unset
        output_packet_.tracking_valid = true;
        if (bTrackingValid) {
          // printf("Tracking Valid: True\n");
        } else {
          // printf("Tracking Valid: False\n");
        }
      }

      outputs.push_back(output_packet_);

    } // Go to next rigid body


    // Skeletons (NatNet version 2.1 and later)
    if (((major == 2) && (minor > 0)) || (major > 2)) {
      int nSkeletons = 0;
      memcpy(&nSkeletons, ptr, 4);
      ptr += 4;
      // printf("Skeleton Count : %d\n", nSkeletons);

      // Loop through skeletons
      for (int j = 0; j < nSkeletons; j++) {
        // skeleton id
        int skeletonID = 0;
        memcpy(&skeletonID, ptr, 4);
        ptr += 4;

        // Number of rigid bodies (bones) in skeleton
        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4);
        ptr += 4;
        // printf("Rigid Body Count : %d\n", nRigidBodies);

        // Loop through rigid bodies (bones) in skeleton
        for (int j = 0; j < nRigidBodies; j++) {
          // Rigid body position and orientation
          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          float x = 0.0f;
          memcpy(&x, ptr, 4);
          ptr += 4;
          float y = 0.0f;
          memcpy(&y, ptr, 4);
          ptr += 4;
          float z = 0.0f;
          memcpy(&z, ptr, 4);
          ptr += 4;
          float qx = 0;
          memcpy(&qx, ptr, 4);
          ptr += 4;
          float qy = 0;
          memcpy(&qy, ptr, 4);
          ptr += 4;
          float qz = 0;
          memcpy(&qz, ptr, 4);
          ptr += 4;
          float qw = 0;
          memcpy(&qw, ptr, 4);
          ptr += 4;
          // printf("ID : %d\n", ID);
          // printf("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
          // printf("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx, qy, qz, qw);

          // Before NatNet 3.0, marker data was here
          if (major < 3) {
            // associated marker positions
            int nRigidMarkers = 0;
            memcpy(&nRigidMarkers, ptr, 4);
            ptr += 4;
            // printf("Marker Count: %d\n", nRigidMarkers);
            int nBytes = nRigidMarkers * 3 * sizeof(float);
            float *markerData = (float *) malloc(nBytes);
            memcpy(markerData, ptr, nBytes);
            ptr += nBytes;

            if (major >= 2) {
              // associated marker IDs
              nBytes = nRigidMarkers * sizeof(int);
              int *markerIDs = (int *) malloc(nBytes);
              memcpy(markerIDs, ptr, nBytes);
              ptr += nBytes;

              // associated marker sizes
              nBytes = nRigidMarkers * sizeof(float);
              float *markerSizes = (float *) malloc(nBytes);
              memcpy(markerSizes, ptr, nBytes);
              ptr += nBytes;

              for (int k = 0; k < nRigidMarkers; k++) {
                // printf("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n",
                       // k,
                       // markerIDs[k],
                       // markerSizes[k],
                       // markerData[k * 3],
                       // markerData[k * 3 + 1],
                       // markerData[k * 3 + 2]);
              }

              if (markerIDs)
                free(markerIDs);
              if (markerSizes)
                free(markerSizes);

            } else {
              for (int k = 0; k < nRigidMarkers; k++) {
                // printf("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k,
                       // markerData[k * 3], markerData[k * 3 + 1],
                       // markerData[k * 3 + 2]);
              }
            }
            if (markerData)
              free(markerData);
          }

          // Mean marker error (NatNet version 2.0 and later)
          if (major >= 2) {
            float fError = 0.0f;
            memcpy(&fError, ptr, 4);
            ptr += 4;
            // printf("Mean marker error: %3.2f\n", fError);
          }

          // Tracking flags (NatNet version 2.6 and later)
          if (((major == 2) && (minor >= 6)) || (major > 2)) {
            // params
            short params = 0;
            memcpy(&params, ptr, 2);
            ptr += 2;
            // 0x01 : rigid body was successfully tracked in this frame
            bool bTrackingValid = params & 0x01;
            // output_packet_.tracking_valid = bTrackingValid;
          }

        } // next rigid body

      } // next skeleton
    }

    if (((major == 2) && (minor >= 3)) || (major > 2)) {
      int nLabeledMarkers = 0;
      memcpy(&nLabeledMarkers, ptr, 4);
      ptr += 4;

      // Loop through labeled markers
      for (int j = 0; j < nLabeledMarkers; j++) {
        // id
        // Marker ID Scheme:
        // Active Markers:
        //   ID = ActiveID, correlates to RB ActiveLabels list
        // Passive Markers:
        //   If Asset with Legacy Labels
        //      AssetID   (Hi Word)
        //      MemberID  (Lo Word)
        //   Else
        //      PointCloud ID
        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        int modelID, markerID;
        DecodeMarkerID(ID, &modelID, &markerID);

        // x
        float x = 0.0f;
        memcpy(&x, ptr, 4);
        ptr += 4;
        // y
        float y = 0.0f;
        memcpy(&y, ptr, 4);
        ptr += 4;
        // z
        float z = 0.0f;
        memcpy(&z, ptr, 4);
        ptr += 4;
        // size
        float size = 0.0f;
        memcpy(&size, ptr, 4);
        ptr += 4;

        // NatNet version 2.6 and later
        if (((major == 2) && (minor >= 6)) || (major > 2)) {
          // marker params
          short params = 0;
          memcpy(&params, ptr, 2);
          ptr += 2;
          // marker was not visible (occluded) in this frame
          bool bOccluded = (params & 0x01) != 0;
          // position provided by point cloud solve
          bool bPCSolved = (params & 0x02) != 0;
          // position provided by model solve
          bool bModelSolved = (params & 0x04) != 0;
          if (major >= 3) {
            // marker has an associated model
            bool bHasModel = (params & 0x08) != 0;
            // marker is an unlabeled marker
            bool bUnlabeled = (params & 0x10) != 0;
            // marker is an active marker
            bool bActiveMarker = (params & 0x20) != 0;
          }

        }

        // NatNet version 3.0 and later
        float residual = 0.0f;
        if (major >= 3) {
          // Marker residual
          memcpy(&residual, ptr, 4);
          ptr += 4;
        }

        Marker marker_;
        marker_.id = markerID;
        marker_.residual = residual;
        marker_.size = size;
        marker_.x = x;
        marker_.y = y;
        marker_.z = z;

        // output_packet_.markers_.push_back(marker_);
        // printf("ID  : [MarkerID: %d] [ModelID: %d]\n", markerID, modelID);
        // printf("pos : [%3.2f,%3.2f,%3.2f]\n", x, y, z);
        // printf("size: [%3.2f]\n", size);
        // printf("err:  [%3.2f]\n", residual);
      }
    }

    // Force Plate data (NatNet version 2.9 and later)
    if (((major == 2) && (minor >= 9)) || (major > 2)) {
      int nForcePlates;
      memcpy(&nForcePlates, ptr, 4);
      ptr += 4;
      for (int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++) {
        // ID
        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        // printf("Force Plate : %d\n", ID);

        // Channel Count
        int nChannels = 0;
        memcpy(&nChannels, ptr, 4);
        ptr += 4;

        // Channel Data
        for (int i = 0; i < nChannels; i++) {
          // printf(" Channel %d : ", i);
          int nFrames = 0;
          memcpy(&nFrames, ptr, 4);
          ptr += 4;
          for (int j = 0; j < nFrames; j++) {
            float val = 0.0f;
            memcpy(&val, ptr, 4);
            ptr += 4;
            printf("%3.2f   ", val);
          }
          printf("\n");
        }
      }
    }

    // Device data (NatNet version 3.0 and later)
    if (((major == 2) && (minor >= 11)) || (major > 2)) {
      int nDevices;
      memcpy(&nDevices, ptr, 4);
      ptr += 4;
      for (int iDevice = 0; iDevice < nDevices; iDevice++) {
        // ID
        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        // printf("Device : %d\n", ID);

        // Channel Count
        int nChannels = 0;
        memcpy(&nChannels, ptr, 4);
        ptr += 4;

        // Channel Data
        for (int i = 0; i < nChannels; i++) {
          // printf(" Channel %d : ", i);
          int nFrames = 0;
          memcpy(&nFrames, ptr, 4);
          ptr += 4;
          for (int j = 0; j < nFrames; j++) {
            float val = 0.0f;
            memcpy(&val, ptr, 4);
            ptr += 4;
            printf("%3.2f   ", val);
          }
          printf("\n");
        }
      }
    }

    // software latency (removed in version 3.0)
    if (major < 3) {
      float softwareLatency = 0.0f;
      memcpy(&softwareLatency, ptr, 4);
      ptr += 4;
      // printf("software latency : %3.3f\n", softwareLatency);
    }

    // timecode
    unsigned int timecode = 0;
    memcpy(&timecode, ptr, 4);
    ptr += 4;
    unsigned int timecodeSub = 0;
    memcpy(&timecodeSub, ptr, 4);
    ptr += 4;
    char szTimecode[128] = "";
    TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

    // timestamp
    double timestamp = 0.0f;

    // NatNet version 2.7 and later - increased from single to double precision
    if (((major == 2) && (minor >= 7)) || (major > 2)) {
      memcpy(&timestamp, ptr, 8);
      ptr += 8;
    } else {
      float fTemp = 0.0f;
      memcpy(&fTemp, ptr, 4);
      ptr += 4;
      timestamp = (double) fTemp;
    }

    // Convert to microseconds
    uint64_t ms_timestamp = (timestamp*1e9)/server_frequency;
    // printf("Timestamp : %3.3f\n", timestamp);

    // high res timestamps (version 3.0 and later)
    if (major >= 3) {
      uint64_t cameraMidExposureTimestamp = 0;
      memcpy(&cameraMidExposureTimestamp, ptr, 8);
      ptr += 8;
      // printf("Mid-exposure timestamp : %" PRIu64 "\n",
      //        cameraMidExposureTimestamp);

      uint64_t cameraDataReceivedTimestamp = 0;
      memcpy(&cameraDataReceivedTimestamp, ptr, 8);
      ptr += 8;
      // printf("Camera data received timestamp : %" PRIu64 "\n",
      //        cameraDataReceivedTimestamp);

      uint64_t transmitTimestamp = 0;
      memcpy(&transmitTimestamp, ptr, 8);
      ptr += 8;
      // printf("Transmit timestamp : %" PRIu64 "\n", transmitTimestamp);

      // Convert timestamps to microseconds and save them in the output packets
      for(int i = 0; i < outputs.size(); i++){
        outputs[i].timestamp = ms_timestamp;
        outputs[i].mid_exposure_timestamp = (cameraMidExposureTimestamp*1e9)/server_frequency;
        outputs[i].camera_data_received_timestamp = (cameraDataReceivedTimestamp*1e9)/server_frequency;
        outputs[i].transmit_timestamp = (transmitTimestamp*1e9)/server_frequency; 
      }
    }

    // frame params
    short params = 0;
    memcpy(&params, ptr, 2);
    ptr += 2;
    // 0x01 Motive is recording
    bool bIsRecording = (params & 0x01) != 0;
    // 0x02 Actively tracked model list has changed
    bool bTrackedModelsChanged = (params & 0x02) != 0;


    // end of data tag
    int eod = 0;
    memcpy(&eod, ptr, 4);
    ptr += 4;
    // printf("End Packet\n-------------\n");

  } else if (MessageID == 5) // Data Descriptions ####################################################################3
  {
    bool print_info=0;
    // number of datasets
    int nDatasets = 0;
    memcpy(&nDatasets, ptr, 4);
    ptr += 4;
    if(print_info)
    printf("Dataset Count : %d\n", nDatasets);

    for (int i = 0; i < nDatasets; i++) {
      if(print_info)
      printf("Dataset %d\n", i);

      int type = 0;
      memcpy(&type, ptr, 4);
      ptr += 4;
      if(print_info)
      printf("Type : %d\n", type);

      if (type == 0)   // markerset
      {
        // name
        char szName[256];
        strcpy(szName, ptr);
        int nDataBytes = (int) strlen(szName) + 1;
        ptr += nDataBytes;
        if(print_info)
        printf("Markerset Name: %s\n", szName);

        // marker data
        int nMarkers = 0;
        memcpy(&nMarkers, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("Marker Count : %d\n", nMarkers);

        for (int j = 0; j < nMarkers; j++) {
          char szName[256];
          strcpy(szName, ptr);
          int nDataBytes = (int) strlen(szName) + 1;
          ptr += nDataBytes;
          if(print_info)
          printf("Marker Name: %s\n", szName);
        }
      } else if (type == 1)   // rigid body
      {
        char szName[MAX_NAMELENGTH];  
        if (major >= 2) {
          // name
          strcpy(szName, ptr);
          ptr += strlen(ptr) + 1;
          if(print_info)
          printf("Name: %s\n", szName);
        }

        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("ID : %d\n", ID);

        // Add rigid body - name pair
        rigid_body_map[ID] = szName;

        int parentID = 0;
        memcpy(&parentID, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("Parent ID : %d\n", parentID);

        float xoffset = 0;
        memcpy(&xoffset, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("X Offset : %3.2f\n", xoffset);

        float yoffset = 0;
        memcpy(&yoffset, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("Y Offset : %3.2f\n", yoffset);

        float zoffset = 0;
        memcpy(&zoffset, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("Z Offset : %3.2f\n", zoffset);

        // Per-marker data (NatNet 3.0 and later)
        if (major >= 3) {
          int nMarkers = 0;
          memcpy(&nMarkers, ptr, 4);
          ptr += 4;

          // Marker positions
          nBytes = nMarkers * 3 * sizeof(float);
          float *markerPositions = (float *) malloc(nBytes);
          memcpy(markerPositions, ptr, nBytes);
          ptr += nBytes;

          // Marker required active labels
          nBytes = nMarkers * sizeof(int);
          int *markerRequiredLabels = (int *) malloc(nBytes);
          memcpy(markerRequiredLabels, ptr, nBytes);
          ptr += nBytes;

          for (int markerIdx = 0; markerIdx < nMarkers; ++markerIdx) {
            float *markerPosition = markerPositions + markerIdx * 3;
            const int markerRequiredLabel = markerRequiredLabels[markerIdx];
            if(print_info)
            printf("\tMarker #%d:\n", markerIdx);
            if(print_info)
            printf("\t\tPosition: %.2f, %.2f, %.2f\n",
                   markerPosition[0],
                   markerPosition[1],
                   markerPosition[2]);

            if (markerRequiredLabel != 0) {
              if(print_info)
              printf("\t\tRequired active label: %d\n", markerRequiredLabel);
            }
          }

          free(markerPositions);
          free(markerRequiredLabels);
        }

      } else if (type == 2)   // skeleton
      {
        char szName[MAX_NAMELENGTH];
        strcpy(szName, ptr);
        ptr += strlen(ptr) + 1;
        if(print_info)
        printf("Name: %s\n", szName);

        int ID = 0;
        memcpy(&ID, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("ID : %d\n", ID);

        int nRigidBodies = 0;
        memcpy(&nRigidBodies, ptr, 4);
        ptr += 4;
        if(print_info)
        printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

        for (int i = 0; i < nRigidBodies; i++) {
          if (major >= 2) {
            // RB name
            char szName[MAX_NAMELENGTH];
            strcpy(szName, ptr);
            ptr += strlen(ptr) + 1;
            if(print_info)
            printf("Rigid Body Name: %s\n", szName);
          }

          int ID = 0;
          memcpy(&ID, ptr, 4);
          ptr += 4;
          if(print_info)
          printf("RigidBody ID : %d\n", ID);

          int parentID = 0;
          memcpy(&parentID, ptr, 4);
          ptr += 4;
          if(print_info)
          printf("Parent ID : %d\n", parentID);

          float xoffset = 0;
          memcpy(&xoffset, ptr, 4);
          ptr += 4;
          if(print_info)
          printf("X Offset : %3.2f\n", xoffset);

          float yoffset = 0;
          memcpy(&yoffset, ptr, 4);
          ptr += 4;
          if(print_info)
          printf("Y Offset : %3.2f\n", yoffset);

          float zoffset = 0;
          memcpy(&zoffset, ptr, 4);
          ptr += 4;
          if(print_info)
          printf("Z Offset : %3.2f\n", zoffset);
        }
      }

    }   // next dataset

    // Add names to rigid bodies
    for (int i = 0; i < outputs.size(); i++) {
      outputs[i].model_name = rigid_body_map[outputs[i].rigid_body_id];
    }

    if(print_info)
    printf("End Packet\n-------------\n");
  } else {
    // printf("Unrecognized Packet Type.\n");
  }

}

} // ns agile
