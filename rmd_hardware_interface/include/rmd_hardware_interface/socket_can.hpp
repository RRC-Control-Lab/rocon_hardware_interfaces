#ifndef RMD_HARDWARE_INTERFACE__SOCKET_CAN_HPP_
#define RMD_HARDWARE_INTERFACE__SOCKET_CAN_HPP_

#include <fcntl.h>
#include <unistd.h>  // for close()
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <cerrno>
#include <string>

#include "rmd_hardware_interface/epoll_event_loop.hpp"


using FrameProcessor = std::function<void (const can_frame &)>;
using std::chrono::nanoseconds;
using std::chrono::duration_cast;

class SocketCanIntf
{
public:
  bool init(
    const std::string & interface, EpollEventLoop * event_loop,
    FrameProcessor frame_processor);
  void deinit();
  bool send_can_frame(const can_frame & frame);

private:
  std::string interface_;
  int32_t file_descriptor_;
  int socket_id_ = -1;
  EpollEventLoop * event_loop_ = nullptr;
  EpollEventLoop::EvtId socket_evt_id_;
  FrameProcessor frame_processor_;
  bool broken_ = false;

  void on_socket_event(uint32_t mask);
  bool read_nonblocking();
  void wait_for_send(const std::chrono::nanoseconds timeout) const;
  void wait_for_receive(const std::chrono::nanoseconds timeout) const;
  void process_can_frame(const can_frame & frame)
  {
    frame_processor_(frame);
  }
};

#endif  // RMD_HARDWARE_INTERFACE__SOCKET_CAN_HPP_
