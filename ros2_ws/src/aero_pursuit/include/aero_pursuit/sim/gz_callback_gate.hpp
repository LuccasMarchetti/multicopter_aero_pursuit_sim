#pragma once

#include <mutex>
#include <string>

namespace aero_pursuit::sim
{

/// gz-transport delivers subscriptions on its own threads, outside the ROS executor and unaware
/// of the owner's lifetime. Without care, a message arriving during shutdown reaches a callback
/// whose publishers or mutexes were already destroyed (members die in reverse order), which
/// crashed the component container on every simulation stop (rcl_publish on a freed publisher).
///
/// The same holds at start-up: a subscription made in a constructor can fire before the
/// constructor creates the publishers the callback uses (the gimbal crashed the container this
/// way while loading). Owners that subscribe early construct the gate closed and `open()` it as
/// the last statement of their constructor.
///
/// Callbacks run through the gate; the owner's destructor calls `shutdown(node)`, which cancels
/// every subscription and then closes the gate, waiting for a callback still in progress, before
/// any member is destroyed.
class GzCallbackGate
{
public:
  explicit GzCallbackGate(bool open = true)
  : open_(open) {}

  void open()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
  }

  template<typename F>
  void run(F && f)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (open_) {
      f();
    }
  }

  template<typename GzNode>
  void shutdown(GzNode & node)
  {
    for (const std::string & topic : node.SubscribedTopics()) {
      node.Unsubscribe(topic);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = false;
  }

private:
  std::mutex mutex_;
  bool open_;
};

}  // namespace aero_pursuit::sim
