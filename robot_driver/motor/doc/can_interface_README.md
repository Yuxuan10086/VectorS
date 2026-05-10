# `can_interface.cpp` 设计说明

本文档描述 `src/robot_driver/src/can_interface.cpp` 与头文件 `include/robot_driver/can_interface.hpp` 中的 **SocketCAN 封装** 实现：面向 PD42 类「串口载荷跑在 CAN 上」的用法（完整协议帧见 `电机手册提取.md`）。

---

## 1. 目标与范围

| 能力 | 说明 |
|------|------|
| **发送** | 仅 **扩展帧**；单 CAN 帧最多 8 字节数据，长载荷按固定间隔拆成多帧连续发出 |
| **接收** | 按 **扩展帧 ID（29 位）** 分流，将多 CAN 帧拼成一条 `0xC5`…`0x5C` 逻辑帧，校验通过后供上层取出 |
| **时序** | 两条 **完整逻辑指令** 之间满足手册要求的 **至少 2 ms** 间隔；同一条指令内的分包另有较短间隔 |

本类 **不负责** 协议语义（功能码、寄存器等），仅传递与重组字节流。

---

## 2. 线程模型

```
调用线程                    发送线程 send_thread_loop          接收线程 rx_thread_loop
    |                                    |                                |
    | open() 成功后启动                   |                                |
    |------------------------------->    |                                |
    |                                    |                                |
send()                                     |                                |
    | 入队 + notify                         |                                |
    |------------------------------->    | poll/read CAN                  |
    |   （立即返回）                      | write CAN                       | feed_rx_parser
    |                                    |                                |
try_receive() （任意线程）                 |                                |
    | 从 rx 队列取帧 <--------------------|-------------------------------|
```

- **调用线程**：可并发调用 `send`、`try_receive`。
- **发送线程**：唯一执行「逻辑帧 → 多 CAN 帧 write」的线程，并强制执行帧间 2 ms。
- **接收线程**：唯一从 socket **读** CAN 的线程，解析后写入接收侧数据结构。

`open()` 在 socket `bind` 成功、`CAN_RAW_ERR_FILTER` 设置完成后：**先** `run_send_ = true` 并启动发送线程，**再** `run_rx_ = true` 并启动接收线程。  
`close()` 顺序相反且保证发送线程在 **仍持有有效 `socket_fd_`** 期间跑完退出逻辑（见第 6 节）。

---

## 3. 发送路径

### 3.1 API 语义

- **`send(const std::vector<uint8_t> & frame_bytes)`**  
  - 将 **一整条** 协议帧（已从 `0xC5` 组到含 `0x5C`）入队，**非阻塞**；成功表示「已接受排队」，**不**表示已发到总线。  
  - 入队失败：`frame_bytes` 为空、`socket` 未就绪，或 `close()` 已将 `run_send_` 置 `false`。

### 3.2 发送队列

队列为 `std::deque<std::vector<uint8_t>>`：每项是一条待发 **完整协议帧**（`0xC5`…`0x5C`）。  
下行扩展帧 ID 在 `send_can_chunk` 内写死为 **`0x1001`**（`kDownlinkEffId`）。多轴仍靠帧内 **从机地址** 区分。

### 3.3 分包与两类时间间隔

实现中有两个常量（匿名命名空间）：

| 常量 | 值 | 用途 |
|------|-----|------|
| `kDownlinkEffId` | `0x1001` | 下行扩展帧 ID（`send_can_chunk` 内使用，固定） |
| `kInterChunkPause` | 200 µs | **同一条逻辑帧内**，相邻两个 CAN 数据帧之间的间隔；满足手册「单条指令内连续、字节间不超过约 1 字节时间」的工程近似 |
| `kInterCmdPause` | 2 ms | **两条完整逻辑帧之间** 的最小间隔（从上一帧 **发送结束时刻** 起算） |

`send_one_frame_chunks`：按 `kCanDataMax`（8）切片循环 `write`，除最后一块外，每块之后 `sleep_for(kInterChunkPause)`；**不含** 2 ms。

`send_thread_loop`：维护 `steady_clock` 的 `last_end`（上一逻辑帧 **全部 chunk 写完** 的时间点）。在发送下一帧之前，若当前时刻早于 `last_end + kInterCmdPause`，则 **`sleep_until`** 再取下一帧。首帧无历史则**不**额外等待 2 ms。

若在等待 2 ms 期间 `close()` 被调用：`run_send_` 为 false 后，线程从 `wait` 醒来会进入收尾分支并 **清空队列**，未发送项被丢弃（见第 6 节）。

### 3.4 同步原语

- **`send_mutex_`**：保护 `send_queue_`、`run_send_` 与条件变量协作时的状态。  
- **`send_cv_`**：队列由空变非空或关闭时 `notify_one` / `notify_all`，发送线程阻塞等待，避免空轮询。

---

## 4. 接收路径

### 4.1 套接字读循环

`rx_thread_loop` 使用 `poll(..., 200ms)` 等待可读，然后 `read` 标准 `struct can_frame`。  
丢弃：**错误帧**（`CAN_ERR_FLAG`）、**非扩展帧**（与《电机手册提取.md》「CAN 仅扩展帧」一致）、非法 `len`。  
对扩展帧：`route_id = can_id & CAN_EFF_MASK`，将数据段交给 `feed_rx_parser(route_id, …)`。

### 4.2 按路由键分流组帧

- **`asm_by_route_[route_id]`**：该路由键上 **尚未以 `0x5C` 收尾** 的半帧字节缓冲。  
- 状态机：仅在缓冲为空时识别帧头 `kFrameHead`（`0xC5`）；其后逐字节追加；超过 512 字节则丢弃缓冲以防异常粘包。  
- 遇到 **`kFrameTail`（`0x5C`）**：取出完整字节序列，调用 **`verify_frame_checksum`**（定义于 `pd42_protocol.hpp`）；失败则丢弃；成功则将 `std::vector` **移入** `rx_queue_by_route_[route_id]`。

多个电机使用不同扩展帧 ID 时，按路由键独立组帧。

### 4.3 上层取帧

**`try_receive(uint32_t route_id, std::vector<uint8_t> & out)`**  
在 **`rx_mutex_`** 下从 `rx_queue_by_route_[route_id]` 的 FIFO 队首弹出一帧；`route_id` 为扩展帧 29 位 ID（与收包时一致）。  
成功弹出后若队列为空则 **erase** 该 map 项，避免 map 无限增长。

---

## 5. 锁与并发安全

| 资源 | 锁 |
|------|-----|
| 发送队列、`run_send_`（与 `wait` 配合） | `send_mutex_` |
| 接收组装缓冲、接收完成队列 | `rx_mutex_` |

**发送与接收使用不同互斥量**，避免在单连接上交叉死锁。

---

## 6. 生命周期：`open` / `close`

**`open()`**  
创建 `PF_CAN`/`SOCK_RAW`，绑定指定接口；设置 `CAN_RAW_ERR_FILTER`；然后启动发送线程与接收线程。

**`close()`**（析构也会调用）

1. `run_send_ = false`，`send_cv_.notify_all()`，**join 发送线程**（此时 **不**关闭 socket，发送线程仍可 `write` 直至退出逻辑清空队列）。  
2. `run_rx_ = false`，`shutdown` socket 使 `poll`/`read` 唤醒，join 接收线程。  
3. `close(socket_fd_)`，`socket_fd_ = -1`。

发送线程在发现 `!run_send_` 时 **`send_queue_.clear()`** 并退出，因此 **关闭时未发出的排队帧会被丢弃**，不做阻塞式刷盘。

---

## 7. 与手册、其它文档的对应关系

- 载荷格式、校验、上下行 ID 约定：见 **`电机手册提取.md`**。  
- 帧头尾常量、`kCanDataMax`、`verify_frame_checksum`：见 **`pd42_protocol.hpp`**。

---

## 8. 使用建议（简要）

1. **`send` 为异步**：若需「发完再读应答」，应在应用层根据业务同步（例如轮询 `try_receive` 或超时）。  
2. **接收 ID**：与驱动应答配置的扩展帧 ID 一致（工程上常用 `default_can_eff_id(addr)`，见协议头文件）。  
3. **队列深度**：当前实现 **无界**；若上层入队快于总线 2 ms 节拍，内存会增长，需在更高层限流或丢弃策略。

---

## 9. 文件索引

| 文件 | 角色 |
|------|------|
| `src/robot_driver/src/can_interface.cpp` | 实现 |
| `include/robot_driver/can_interface.hpp` | 类声明与成员布局 |
