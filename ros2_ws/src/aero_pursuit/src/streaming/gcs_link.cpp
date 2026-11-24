#include "aero_pursuit/streaming/gcs_link.hpp"

#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>
#include <utility>

namespace aero_pursuit::streaming
{

void UniqueFd::reset(int fd)
{
  if (fd_ >= 0) {::close(fd_);}
  fd_ = fd;
}

GcsLink::GcsLink(std::string socket_path, LineHandler on_line, StateHandler on_state)
: socket_path_(std::move(socket_path)), on_line_(std::move(on_line)),
  on_state_(std::move(on_state)), thread_([this] {run();})
{
}

GcsLink::~GcsLink()
{
  running_ = false;
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (fd_.valid()) {::shutdown(fd_.get(), SHUT_RDWR);}  // unblocks poll/recv
  }
  if (thread_.joinable()) {thread_.join();}
}

void GcsLink::run()
{
  using namespace std::chrono_literals;
  while (running_) {
    UniqueFd fd(::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0));
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
    if (!fd.valid() ||
      ::connect(fd.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
      std::this_thread::sleep_for(1s);  // GCS not up yet
      continue;
    }
    // Send buffer of ~4 1080p H.264 frames: the GCS consumes immediately; if it stalls,
    // the send timeout drops the connection instead of accumulating latency.
    const int sndbuf = 2 * 1024 * 1024;
    ::setsockopt(fd.get(), SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    timeval tv{0, 250000};
    ::setsockopt(fd.get(), SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    const int raw = fd.get();
    {
      std::lock_guard<std::mutex> lock(send_mutex_);
      fd_ = std::move(fd);
    }
    connected_ = true;
    if (on_state_) {on_state_(true);}
    readLoop(raw);
    disconnect();
    if (on_state_) {on_state_(false);}
    if (running_) {std::this_thread::sleep_for(500ms);}
  }
}

void GcsLink::readLoop(int fd)
{
  std::string buffer;
  char chunk[4096];
  while (running_) {
    pollfd p{fd, POLLIN, 0};
    const int r = ::poll(&p, 1, 500);
    if (r < 0 && errno != EINTR) {return;}
    if (r <= 0) {
      if (!connected_) {return;}  // send() dropped the connection
      continue;
    }
    const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0) {return;}
    buffer.append(chunk, static_cast<std::size_t>(n));
    std::size_t nl;
    while ((nl = buffer.find('\n')) != std::string::npos) {
      if (nl > 0 && on_line_) {on_line_(std::string_view(buffer.data(), nl));}
      buffer.erase(0, nl + 1);
    }
  }
}

void GcsLink::disconnect()
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  connected_ = false;
  fd_.reset();
}

bool GcsLink::send(const void * header, std::size_t header_size, const void * payload, std::size_t size)
{
  std::lock_guard<std::mutex> lock(send_mutex_);
  if (!fd_.valid() || !connected_) {return false;}
  iovec iov[2] = {
    {const_cast<void *>(header), header_size},
    {const_cast<void *>(payload), size}};
  int iovcnt = size > 0 ? 2 : 1;
  iovec * cur = iov;
  while (iovcnt > 0) {
    const ssize_t n = ::writev(fd_.get(), cur, iovcnt);
    if (n < 0) {
      if (errno == EINTR) {continue;}
      // EAGAIN = SO_SNDTIMEO expired in the middle of the packet: framing lost.
      connected_ = false;
      ::shutdown(fd_.get(), SHUT_RDWR);
      return false;
    }
    std::size_t left = static_cast<std::size_t>(n);
    while (iovcnt > 0 && left >= cur->iov_len) {
      left -= cur->iov_len;
      ++cur;
      --iovcnt;
    }
    if (iovcnt > 0) {
      cur->iov_base = static_cast<char *>(cur->iov_base) + left;
      cur->iov_len -= left;
    }
  }
  return true;
}

}  // namespace aero_pursuit::streaming
