#ifndef AK_HARDWARE_INTERFACE__EPOLL_EVENT_LOOP_HPP_
#define AK_HARDWARE_INTERFACE__EPOLL_EVENT_LOOP_HPP_

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <iostream>
#include <functional>
#include <vector>

using std::placeholders::_1;
using Callback = std::function<void (uint32_t)>;

class EpollEventLoop
{
public:
  struct EventContext
  {
    int fd;
    Callback callback;
  };

  using EvtId = EventContext *;

  EpollEventLoop();

  ~EpollEventLoop();

  bool register_event(EvtId * p_evt, int fd, uint32_t events, const Callback & callback);

  bool deregister_event(EvtId evt);

  bool run_until_empty();

  void drop_event(EvtId evt);

private:
  static constexpr size_t kMaxEventsPerIteration = 16;
  int epollfd = -1;
  size_t n_events_ = 0;
  int n_triggered_events_ = 0;
  struct epoll_event triggered_events_[kMaxEventsPerIteration];
};

class EpollEvent
{
public:
  bool init(EpollEventLoop * event_loop, const Callback & callback);
  void deinit();

  bool set();

private:
  void on_trigger(uint32_t event_id);

  EpollEventLoop * event_loop_;
  int fd_ = -1;
  EpollEventLoop::EventContext * evt_;
  Callback callback_;
};


#endif  // AK_HARDWARE_INTERFACE__EPOLL_EVENT_LOOP_HPP_
