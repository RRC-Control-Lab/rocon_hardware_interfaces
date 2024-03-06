#include "ak_hardware_interface/socket_can.hpp"
#include <thread>


bool SocketCanIntf::init(
  const std::string & interface, EpollEventLoop * event_loop,
  FrameProcessor frame_processor)
{
  interface_ = interface;
  event_loop_ = event_loop;
  frame_processor_ = std::move(frame_processor);
  // Create file descriptor
  file_descriptor_ = socket(PF_CAN, static_cast<int32_t>(SOCK_RAW), CAN_RAW);
  if (0 > file_descriptor_) {
    std::cerr << "Failed to create socket" << std::endl;
    return false;
  }

  // Make it non-blocking so we can use timeouts
  if (0 != fcntl(file_descriptor_, F_SETFL, O_NONBLOCK)) {
    std::cerr << "Failed to set CAN socket to nonblocking" << std::endl;
    (void)close(file_descriptor_);
    return false;
  }

  // Set up address/interface name
  struct ifreq ifr;
  // The destination struct is local; don't need address
  (void)strncpy(&ifr.ifr_name[0U], interface_.c_str(), interface_.length() + 1U);
  if (0 != ioctl(file_descriptor_, static_cast<uint32_t>(SIOCGIFINDEX), &ifr)) {
    std::cerr << "Failed to get interface index" << std::endl;
    (void)close(file_descriptor_);
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family = static_cast<decltype(addr.can_family)>(AF_CAN);
  addr.can_ifindex = ifr.ifr_ifindex;

  // Bind socket to interface
  if (0 > bind(file_descriptor_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr))) {
    std::cerr << "Failed to bind CAN socket" << std::endl;
    (void)close(file_descriptor_);
    return false;
  }

  if (!event_loop_->register_event(
      &socket_evt_id_, file_descriptor_, EPOLLIN, [this](uint32_t mask) {
        on_socket_event(mask);
      }))
  {
    std::cerr << "Failed to register socket with event loop" << std::endl;
    (void)close(file_descriptor_);
    return false;
  }

  return true;
}

/// Convert std::chrono duration to timeval (with microsecond resolution)
struct timeval to_timeval(const std::chrono::nanoseconds timeout) noexcept
{
  const auto count = timeout.count();
  constexpr auto BILLION = 1'000'000'000LL;
  struct timeval c_timeout;
  c_timeout.tv_sec = static_cast<decltype(c_timeout.tv_sec)>(count / BILLION);
  c_timeout.tv_usec = static_cast<decltype(c_timeout.tv_usec)>((count % BILLION) / 1000LL);

  return c_timeout;
}

fd_set single_set(int32_t file_descriptor) noexcept
{
  fd_set descriptor_set;
  FD_ZERO(&descriptor_set);
  FD_SET(file_descriptor, &descriptor_set);

  return descriptor_set;
}

void SocketCanIntf::wait_for_send(const std::chrono::nanoseconds timeout) const
{
  // block specified amount of time
  // sleep for 10us
  // sleep(0.0002);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto write_set = single_set(file_descriptor_);
    // Wait
    if (0 == select(file_descriptor_ + 1, NULL, &write_set, NULL, &c_timeout)) {
      std::cerr << "CAN Send Timeout" << std::endl;
    }
    // Check if the file descriptor is ready
    if (!FD_ISSET(file_descriptor_, &write_set)) {
      std::cerr << "CAN Send timeout" << std::endl;
    }
  }
}

void SocketCanIntf::wait_for_receive(const std::chrono::nanoseconds timeout) const
{
  if (decltype(timeout)::zero() < timeout) {
    auto c_timeout = to_timeval(timeout);
    auto read_set = single_set(file_descriptor_);
    // Wait
    if (0 == select(file_descriptor_ + 1, &read_set, NULL, NULL, &c_timeout)) {
      std::cerr << "CAN Receive Timeout" << std::endl;
    }
    //lint --e{9130, 1924, 9123, 9125, 1924, 9126} NOLINT
    if (!FD_ISSET(file_descriptor_, &read_set)) {
      std::cerr << "CAN Receive timeout" << std::endl;
    }
  }
}

void SocketCanIntf::deinit()
{
  if (!broken_) {
    event_loop_->deregister_event(socket_evt_id_);
  }
  (void)close(file_descriptor_);
  broken_ = true;
}

bool SocketCanIntf::send_can_frame(const can_frame & frame)
{
  // Use select call on positive timeout
  // 0.01s timeout
  std::chrono::milliseconds send_timeout_{1LL};
  wait_for_send(send_timeout_);
  constexpr int flags = 0;
  const auto bytes_sent = ::send(file_descriptor_, &frame, sizeof(frame), flags);
  if (0 > bytes_sent) {
    std::cerr << "Failed to send CAN frame" << std::endl;
  }

  return true;
}

void SocketCanIntf::on_socket_event(uint32_t mask)
{
  if (mask & EPOLLIN) {
    while (read_nonblocking() && !broken_) {}
  }
  if (mask & EPOLLERR) {
    std::cerr << "interface disappeared" << std::endl;
    deinit();
    return;
  }
  if (mask & ~(EPOLLIN | EPOLLERR)) {
    std::cerr << "unexpected event " << mask << std::endl;
    deinit();
    return;
  }
  return;
}

bool SocketCanIntf::read_nonblocking()
{
  struct can_frame frame;

  wait_for_send(std::chrono::milliseconds {10LL});

  // Read the CAN frame
  ssize_t nbytes = read(file_descriptor_, &frame, sizeof(frame));
  // Check for errors
  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // std::cerr << "no message received" << std::endl;
      return false;
    } else {
      std::cerr << "Socket read failed: " << std::endl;
      return false;
    }
  }
  if (static_cast<std::size_t>(nbytes) != sizeof(can_frame)) {
    std::cerr << "invalid message length " << nbytes << std::endl;
    return true;
  }

  process_can_frame(frame);
  return true;
}
