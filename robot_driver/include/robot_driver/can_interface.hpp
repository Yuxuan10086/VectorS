#ifndef ROBOT_DRIVER__CAN_INTERFACE_HPP_
#define ROBOT_DRIVER__CAN_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_driver
{

/** SocketCAN：扩展帧发送、按 8 字节分包；接收线程拼出完整 C5…5C 帧（手册 6.2.1） */
class CanInterface
{
public:
  using RxCallback = std::function<void (const std::vector<uint8_t> & frame)>;

  explicit CanInterface(std::string interface_name);
  ~CanInterface();

  CanInterface(const CanInterface &) = delete;
  CanInterface & operator=(const CanInterface &) = delete;

  bool is_open() const { return socket_fd_ >= 0; }

  bool open();
  void close();

  void set_send_can_id(uint32_t eff_id) { send_eff_id_ = eff_id; }

  /** 收到完整且校验通过的帧时调用（可为空） */
  void set_rx_callback(RxCallback cb);
0
  bool send_serial_over_can(const std::vector<uint8_t> & frame_bytes);

private:
  void rx_thread_loop();
  void feed_rx_parser(const uint8_t * data, std::uint8_t len);
  bool send_can_chunk(const uint8_t * data, std::uint8_t len);

  std::string iface_;
  int socket_fd_{-1};
  uint32_t send_eff_id_{0x1001U};

  std::thread rx_thread_;
  std::atomic<bool> run_rx_{false};

  std::mutex cb_mutex_;
  RxCallback rx_cb_{};

  std::mutex asm_mutex_;
  std::vector<uint8_t> asm_buffer_;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__CAN_INTERFACE_HPP_
