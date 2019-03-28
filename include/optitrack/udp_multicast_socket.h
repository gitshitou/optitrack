/**
 * @file  udp_multicast_socket.h
 * @brief UDP Socket with Multicast Support. Wraps UNIX socket API.
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 28 March 2019
 */

#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <cerrno>

namespace acl {
namespace utils {

  /**
   * @brief      Exception for signaling socket errors.
   */
  class SocketException : public std::runtime_error
  {
    public:
      
      /**
       * @brief      Constructor
       *
       * @param[in]  description  Error message
       */
      SocketException ( std::string description ) : std::runtime_error( description ) {}
      
      ~SocketException () throw() {}
  };



  class UDPMulticastSocket
  {
  public:
    UDPMulticastSocket();
    ~UDPMulticastSocket();

    bool setReceiveTimeout(const int seconds = 1, const int micros = 0);
    bool joinMulticastGroup(const std::string& multicastGroupIP);
    bool receive();
    bool sendto(const std::string& remoteIP, const int remotePort);

  private:
    int socket_; ///< UNIX socket file descriptor
    sockaddr_in localAddr_; ///< Local address info for socket

    static constexpr int BUFSIZE_1MB = 0x100000; ///< 1MB buffer size option
    
  };

} // ns utils
} // ns acl
