#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>

namespace aero_pursuit::streaming
{

/// File descriptor with automatic close (RAII).
class UniqueFd
{
public:
  UniqueFd() = default;
  explicit UniqueFd(int fd) : fd_(fd) {}
  ~UniqueFd() {reset();}
  UniqueFd(const UniqueFd &) = delete;
  UniqueFd & operator=(const UniqueFd &) = delete;
  UniqueFd(UniqueFd && o) noexcept : fd_(o.release()) {}
  UniqueFd & operator=(UniqueFd && o) noexcept
  {
    if (this != &o) {reset(o.release());}
    return *this;
  }
  int get() const {return fd_;}
  bool valid() const {return fd_ >= 0;}
  int release() {const int f = fd_; fd_ = -1; return f;}
  void reset(int fd = -1);

private:
  int fd_ = -1;
};

/// Streamer connection to the GCS through a Unix socket (same machine). Reconnects by itself;
/// received text lines (commands/configuration) go to `on_line`, called on the
/// reader thread. `send` is thread-safe and never blocks longer than `send_timeout_ms`.
class GcsLink
{
public:
  using LineHandler = std::function<void(std::string_view)>;
  using StateHandler = std::function<void(bool connected)>;

  GcsLink(std::string socket_path, LineHandler on_line, StateHandler on_state);
  ~GcsLink();
  GcsLink(const GcsLink &) = delete;
  GcsLink & operator=(const GcsLink &) = delete;

  /// Sends header + payload in a single write. Returns false if disconnected or if
  /// the GCS did not consume it in time (the connection is then dropped: framing broke).
  bool send(const void * header, std::size_t header_size, const void * payload, std::size_t size);
  bool connected() const {return connected_.load();}

private:
  void run();
  void readLoop(int fd);
  void disconnect();

  const std::string socket_path_;
  LineHandler on_line_;
  StateHandler on_state_;
  std::atomic<bool> running_{true};
  std::atomic<bool> connected_{false};
  std::mutex send_mutex_;
  UniqueFd fd_;
  std::thread thread_;
};

}  // namespace aero_pursuit::streaming
