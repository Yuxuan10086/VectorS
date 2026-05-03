#include "robot_driver/can_interface.hpp"

#include "robot_driver/pd42_motor_sdk.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <thread>

namespace robot_driver
{

namespace
{
constexpr std::chrono::microseconds kInterChunkPause{200};
constexpr std::chrono::milliseconds kInterCmdPause{2};
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

  run_rx_ = true;
  rx_thread_ = std::thread(&CanInterface::rx_thread_loop, this);
  return true;
}

void CanInterface::close()
{
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

void CanInterface::set_rx_callback(RxCallback cb)
{
  std::lock_guard<std::mutex> lk(cb_mutex_);
  rx_cb_ = std::move(cb);
}

bool CanInterface::send_can_chunk(const uint8_t * data, std::uint8_t len)
{
  struct can_frame frame {};
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = send_eff_id_ | CAN_EFF_FLAG;
  frame.len = len;
  std::memcpy(frame.data, data, len);

  const int nbytes = write(socket_fd_, &frame, sizeof(frame));
  return nbytes == static_cast<int>(sizeof(frame));
}

bool CanInterface::send_serial_over_can(const std::vector<uint8_t> & frame_bytes)
{
  if (socket_fd_ < 0 || frame_bytes.empty()) {
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
  std::this_thread::sleep_for(kInterCmdPause);
  return true;
}

void CanInterface::feed_rx_parser(const uint8_t * data, std::uint8_t len)
{
  std::lock_guard<std::mutex> lk(asm_mutex_);
  for (std::uint8_t i = 0; i < len; ++i) {
    const uint8_t b = data[i];
    if (asm_buffer_.empty()) {
      if (b == kFrameHead) {
        asm_buffer_.push_back(b);
      }
      continue;
    }
    asm_buffer_.push_back(b);
    if (asm_buffer_.size() > 512U) {
      asm_buffer_.clear();
      continue;
    }
    if (b == kFrameTail) {
      std::vector<uint8_t> complete = std::move(asm_buffer_);
      asm_buffer_.clear();

      if (!verify_frame_checksum(complete)) {
        continue;
      }
      RxCallback fn;
      {
        std::lock_guard<std::mutex> lk_cb(cb_mutex_);
        fn = rx_cb_;
      }
      if (fn) {
        fn(complete);
      }
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
    if (!(frame.can_id & CAN_EFF_FLAG)) {
      continue;
    }
    if (frame.len > kCanDataMax) {
      continue;
    }
    feed_rx_parser(frame.data, frame.len);
  }
}

}  // namespace robot_driver
