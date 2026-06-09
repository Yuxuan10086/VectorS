#ifndef ROBOT_DRIVER__CAN_INTERFACE_HPP_
#define ROBOT_DRIVER__CAN_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace robot_driver
{

/** SocketCAN：扩展帧；下行 ID 固定 0x1001；异步发送队列与按路由键组帧接收；绑定网卡名见 can_interface.cpp 内常量。详见 doc/can_interface_README.md */
class CanInterface
{
public:
  CanInterface();
  ~CanInterface();

  CanInterface(const CanInterface &) = delete;
  CanInterface & operator=(const CanInterface &) = delete;

  bool is_open() const { return socket_fd_ >= 0; }

  bool open();
  void close();

  /** 非阻塞：按扩展帧 29 位路由键取一帧完成应答（与手册一致，仅扩展帧） */
  bool try_receive(uint32_t route_id, std::vector<uint8_t> & out);

  /** 将完整协议帧入队异步下发；发送线程负责分包与帧间 ≥2 ms。失败：未 open 或正在 close */
  bool send(const std::vector<uint8_t> & frame_bytes);

  /** 请求-应答事务互斥（同一 CAN 口多线程不得并行 wait 应答） */
  std::mutex & request_mutex() { return request_mutex_; }

  /** 丢弃指定路由键上已排队但未取走的应答（发新指令前清陈旧帧） */
  void drain_rx_route(uint32_t route_id);

private:
  void rx_thread_loop();
  void send_thread_loop();
  void feed_rx_parser(uint32_t route_id, const uint8_t * data, std::uint8_t len);
  bool send_can_chunk(const uint8_t * data, std::uint8_t len);
  bool send_one_frame_chunks(const std::vector<uint8_t> & frame_bytes);

  std::string iface_;
  int socket_fd_{-1};

  std::thread send_thread_;
  std::atomic<bool> run_send_{false};

  std::mutex send_mutex_;
  std::condition_variable send_cv_;
  std::deque<std::vector<uint8_t>> send_queue_;

  std::thread rx_thread_;
  std::atomic<bool> run_rx_{false};

  std::mutex rx_mutex_;
  std::mutex request_mutex_;
  std::unordered_map<uint32_t, std::vector<uint8_t>> asm_by_route_;
  std::unordered_map<uint32_t, std::deque<std::vector<uint8_t>>> rx_queue_by_route_;
};

}  // namespace robot_driver

#endif  // ROBOT_DRIVER__CAN_INTERFACE_HPP_
