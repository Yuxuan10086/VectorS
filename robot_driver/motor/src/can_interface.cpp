#include "robot_driver/can_interface.hpp"

#include "robot_driver/pd42_protocol.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <optional>
#include <thread>

namespace robot_driver
{

namespace
{
constexpr std::chrono::microseconds kInterChunkPause{200};
constexpr std::chrono::milliseconds kInterCmdPause{2};
/** 下行（上位机→驱动器）扩展帧 ID，工程约定固定 */
constexpr uint32_t kDownlinkEffId = 0x1001U;
}

CanInterface::CanInterface(std::string interface_name)
: iface_(std::move(interface_name)) {}

CanInterface::~CanInterface()
{
  close();
}

bool CanInterface::open()
{
  if (socket_fd_ >= 0) {
    return true;
  }
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    return false;
  }

  struct ifreq ifr {};
  if (iface_.size() >= IFNAMSIZ) {
    close();
    return false;
  }
  std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close();
    return false;
  }

  const int err_mask = CAN_ERR_MASK;
  setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));

  run_send_ = true;
  send_thread_ = std::thread(&CanInterface::send_thread_loop, this);

  run_rx_ = true;
  rx_thread_ = std::thread(&CanInterface::rx_thread_loop, this);
  return true;
}

void CanInterface::close()
{
  run_send_ = false;
  send_cv_.notify_all();
  if (send_thread_.joinable()) {
    send_thread_.join();
  }

  run_rx_ = false;
  if (socket_fd_ >= 0) {
    shutdown(socket_fd_, SHUT_RDWR);
  }
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanInterface::try_receive(uint32_t route_id, std::vector<uint8_t> & out)
{
  std::lock_guard<std::mutex> lk(rx_mutex_);
  const auto it = rx_queue_by_route_.find(route_id);
  if (it == rx_queue_by_route_.end() || it->second.empty()) {
    return false;
  }
  out = std::move(it->second.front());
  it->second.pop_front();
  if (it->second.empty()) {
    rx_queue_by_route_.erase(it);
  }
  return true;
}

bool CanInterface::send_can_chunk(const uint8_t * data, std::uint8_t len)
{
  struct can_frame frame {};
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = kDownlinkEffId | CAN_EFF_FLAG;
  frame.len = len;
  std::memcpy(frame.data, data, len);

  const int nbytes = write(socket_fd_, &frame, sizeof(frame));
  return nbytes == static_cast<int>(sizeof(frame));
}

bool CanInterface::send(const std::vector<uint8_t> & frame_bytes)
{
  if (frame_bytes.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lk(send_mutex_);
  if (socket_fd_ < 0 || !run_send_.load()) {
    return false;
  }
  send_queue_.push_back(frame_bytes);
  send_cv_.notify_one();
  return true;
}

bool CanInterface::send_one_frame_chunks(const std::vector<uint8_t> & frame_bytes)
{
  if (socket_fd_ < 0) {
    return false;
  }
  for (std::size_t off = 0; off < frame_bytes.size(); off += kCanDataMax) {
    const std::size_t chunk = std::min(kCanDataMax, frame_bytes.size() - off);
    if (!send_can_chunk(frame_bytes.data() + off, static_cast<uint8_t>(chunk))) {
      return false;
    }
    if (off + chunk < frame_bytes.size()) {
      std::this_thread::sleep_for(kInterChunkPause);
    }
  }
  return true;
}

void CanInterface::send_thread_loop()
{
  using clock = std::chrono::steady_clock;
  std::optional<clock::time_point> last_end;

  for (;;) {
    std::unique_lock<std::mutex> lk(send_mutex_);
    send_cv_.wait(lk, [&] {
      return !run_send_.load() || !send_queue_.empty();
    });

    if (!run_send_.load()) {
      send_queue_.clear();
      break;
    }

    if (send_queue_.empty()) {
      continue;
    }

    if (last_end) {
      const auto next = *last_end + kInterCmdPause;
      const auto now = clock::now();
      if (now < next) {
        lk.unlock();
        std::this_thread::sleep_until(next);
        lk.lock();
        continue;
      }
    }

    std::vector<uint8_t> frame = std::move(send_queue_.front());
    send_queue_.pop_front();
    lk.unlock();

    (void)send_one_frame_chunks(frame);

    lk.lock();
    last_end = clock::now();
  }
}

void CanInterface::feed_rx_parser(uint32_t route_id, const uint8_t * data, std::uint8_t len)
{
  std::lock_guard<std::mutex> lk(rx_mutex_);
  std::vector<uint8_t> & asm_buffer = asm_by_route_[route_id];

  for (std::uint8_t i = 0; i < len; ++i) {
    const uint8_t b = data[i];
    if (asm_buffer.empty()) {
      if (b == kFrameHead) {
        asm_buffer.push_back(b);
      }
      continue;
    }
    asm_buffer.push_back(b);
    if (asm_buffer.size() > 512U) {
      asm_buffer.clear();
      continue;
    }
    if (b == kFrameTail) {
      std::vector<uint8_t> complete = std::move(asm_buffer);

      if (!verify_frame_checksum(complete)) {
        continue;
      }
      rx_queue_by_route_[route_id].push_back(std::move(complete));
    }
  }
}

void CanInterface::rx_thread_loop()
{
  struct pollfd pfd {};
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;

  while (run_rx_.load()) {
    const int pr = poll(&pfd, 1, 200);
    if (pr <= 0) {
      continue;
    }
    struct can_frame frame {};
    const int nbytes = read(socket_fd_, &frame, sizeof(frame));
    if (nbytes < 0) {
      continue;
    }
    if (static_cast<std::size_t>(nbytes) < sizeof(struct can_frame)) {
      continue;
    }
    if (frame.can_id & CAN_ERR_FLAG) {
      continue;
    }
    /** 手册：CAN 仅扩展帧（《电机手册提取.md》默认与常用参数） */
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      continue;
    }
    if (frame.len > kCanDataMax) {
      continue;
    }
    const std::uint32_t route_id = frame.can_id & CAN_EFF_MASK;
    feed_rx_parser(route_id, frame.data, frame.len);
  }
}

}  // namespace robot_driver
