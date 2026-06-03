/**
  ******************************************************************************
  * @file    mrlink.hpp
  * @brief   C++ OneMessage-style typed wrapper over mrlink C library.
  *
  * 在 C 库 `mrlink` 之上提供类型安全的 topic 注册 API:
  *   - s_link.On<T>(cmd, [](const T&) { ... });
  * 库内自动完成 length check + memcpy, 业务 handler 直接拿到 `const T&`。
  *
  * 要求 / Requirements:
  *   - C++14 及以上 (auto / std::function / lambda)
  *   - 链接 mrlink.c (本文件不重写协议层, 仅做 typed 包装)
  *
  * 内存 / Memory (per Instance):
  *   - MrLink_t 内部: ~384 字节 (32 槽 raw handler)
  *   - 本 wrapper: 16 槽 typed slot × ~40 字节 (std::function) ≈ 640 字节
  *   - 合计 ~1 KB / 实例
  *
  * Quick start:
  *   ```cpp
  *   #include "device/mrlink/mrlink.hpp"
  *
  *   static mrlink::Instance s_link;
  *
  *   MrLink_Config_t cfg = { .max_payload_size = 64, .use_crc16 = true };
  *   s_link.Init(&cfg, rx_buf, sizeof(rx_buf), tx_buf, sizeof(tx_buf));
  *
  *   s_link.On<ChassisCmd_t>(0x10, [](const ChassisCmd_t& c) {
  *       Chassis_ApplyRemoteCmd(&c);
  *   });
  *   ```
  ******************************************************************************
  */

#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <type_traits>

#include "device/mrlink/mrlink.h"

namespace mr::link {

/** Per-instance typed handler pool size. */
inline constexpr uint8_t MAX_TYPED_HANDLERS = 16u;

/**
 * @brief 类型安全的 mrlink 实例包装 / Type-safe mrlink instance wrapper.
 *
 * 一个 Instance 对应一条物理通道 (UART / SPI / FDCAN 之一)。
 * 同实例**不建议**同时混用 C 库原生 RegisterHandler 与本 wrapper 的 On<T>,
 * 否则 typed slot 与 raw slot 互不感知, 可能产生意外行为。
 */
class Instance {
 public:
  Instance() = default;

  // 不允许拷贝: 内含 C 库 opaque 状态, 拷贝会破坏内部一致性
  Instance(const Instance&) = delete;
  Instance& operator=(const Instance&) = delete;

  /**
   * @brief 析构: 注销所有已注册 handler, 防止 C 库持有野指针.
   */
  ~Instance() {
    for (uint8_t i = 0u; i < MAX_TYPED_HANDLERS; i++) {
      if (slots_[i].used) {
        (void)MrLink_RegisterHandler(&proto_, slots_[i].cmd,
                                     nullptr, nullptr);
        slots_[i].used = false;
      }
    }
  }

  /**
   * @brief 初始化 / Initialize.
   *
   * @param cfg          C 库 config (NULL 则用默认: max_payload=64, use_crc16=true)
   * @param rx_buf       RX 流缓冲 (外部提供, ≥ 4 字节)
   * @param rx_buf_size
   * @param tx_buf       TX 缓冲 (外部提供, ≥ 4 + max_payload)
   * @param tx_buf_size
   * @return MRLINK_OK / ERR_NULL / ERR_ARGS
   */
  int8_t Init(const MrLink_Config_t* cfg,
              uint8_t* rx_buf, uint16_t rx_buf_size,
              uint8_t* tx_buf, uint16_t tx_buf_size) {
    return MrLink_Init(&proto_, cfg, rx_buf, rx_buf_size, tx_buf, tx_buf_size);
  }

  /** 重置 (清空 RX 缓冲 + stats + handlers) / Reset. */
  void Reset() {
    MrLink_Reset(&proto_);
    for (auto& s : slots_) {
      s.used = false;
      s.cmd = 0u;
      s.typed = nullptr;
    }
  }

  /** 喂入接收到的字节 / Feed received bytes (ISR / 任务均可调用). */
  int8_t FeedBytes(const uint8_t* data, uint16_t len) {
    return MrLink_FeedBytes(&proto_, data, len);
  }

  /** 拉取一帧 (任务上下文) / Pull one frame. */
  int8_t Parse(uint8_t* out_cmd,
               const uint8_t** out_payload,
               uint16_t* out_payload_len) {
    return MrLink_Parse(&proto_, out_cmd, out_payload, out_payload_len);
  }

  /** 构造一帧到 tx_buf / Build one frame into tx_buf. 返回写入字节数 (0=失败). */
  uint16_t Build(uint8_t cmd, const uint8_t* payload, uint16_t payload_len) {
    return MrLink_Build(&proto_, cmd, payload, payload_len);
  }

  /**
   * @brief 注册空 payload handler / Register empty-payload handler.
   *
   * 适用于 heartbeat 等 length=0 的命令。收到匹配 cmd 且 payload_len=0
   * 时调用 handler；若长度非 0，则计入 stats.frame_rx_size_mismatch。
   */
  int8_t On(uint8_t cmd, std::function<void()> handler) {
    if (handler == nullptr) {
      return Off(cmd);
    }

    int slot = FindSlot(cmd);
    if (slot < 0) {
      slot = AllocSlot();
      if (slot < 0) {
        return MRLINK_ERR_ARGS;
      }
    }

    slots_[slot].used = true;
    slots_[slot].cmd = cmd;
    slots_[slot].typed = nullptr;
    slots_[slot].empty = handler;

    return MrLink_RegisterHandler(&proto_, cmd,
                                  &Instance::EmptyTrampoline,
                                  &slots_[slot]);
  }

  /** 获取运行统计 / Get runtime stats. */
  const MrLink_Stats_t* GetStats() const {
    return MrLink_GetStats(&proto_);
  }

  /**
   * @brief 注册类型化 handler / Register typed handler (OneMessage-style).
   *
   * 收到匹配 cmd 的帧时, 库内自动:
   *   1. 校验 len == sizeof(T), 不匹配则记入 stats.frame_rx_unknown_cmd
   *   2. memcpy payload 到栈上 T 变量
   *   3. 调用 handler, 传入 `const T&`
   *
   * @tparam T       payload 类型, 必须是 trivially copyable (POD 风格)
   * @param cmd      命令字 (0x00~0xFF)
   * @param handler  std::function, 支持 lambda 捕获
   * @return MRLINK_OK / ERR_NULL / ERR_ARGS (typed slot 已满)
   *
   * 重复注册同 cmd 会**覆盖**旧 handler。
   * handler = nullptr 等价于 Off(cmd)。
   */
  template <typename T>
  int8_t On(uint8_t cmd, std::function<void(const T&)> handler) {
    static_assert(std::is_trivially_copyable<T>::value,
                  "mrlink::Instance::On<T>: T must be trivially copyable "
                  "(POD-style struct). No std::string, std::vector, etc.");
    if (handler == nullptr) {
      return Off(cmd);
    }

    int slot = FindSlot(cmd);
    if (slot < 0) {
      slot = AllocSlot();
      if (slot < 0) {
        return MRLINK_ERR_ARGS;  // typed pool 满
      }
    }

    slots_[slot].used = true;
    slots_[slot].cmd = cmd;
    slots_[slot].empty = nullptr;
    slots_[slot].typed = [handler](const uint8_t* pl) {
      T value;
      std::memcpy(&value, pl, sizeof(T));
      handler(value);
    };

    /* 长度校验 + memcpy 由 C 库 MrLink_RegisterTypedHandler 负责 */
    return MrLink_RegisterTypedHandler(&proto_, cmd, sizeof(T),
                                           &Instance::Trampoline,
                                           &slots_[slot]);
  }

  /**
   * @brief 注销指定 cmd 的 handler / Unregister handler for cmd.
   */
  int8_t Off(uint8_t cmd) {
    int slot = FindSlot(cmd);
    if (slot < 0) {
      return MRLINK_ERR;  // 没找到
    }
    slots_[slot].used = false;
    slots_[slot].typed = nullptr;
    slots_[slot].empty = nullptr;
    return MrLink_RegisterHandler(&proto_, cmd, nullptr, nullptr);
  }

 private:
  /** Type-erased typed slot: 1 entry per registered cmd. */
  struct Slot {
    bool used = false;
    uint8_t cmd = 0u;
    std::function<void(const uint8_t*)> typed;  // type-erased
    std::function<void()> empty;
  };

  MrLink_t proto_{};
  Slot slots_[MAX_TYPED_HANDLERS]{};

  /**
   * @brief C 库 → typed slot 的桥 / Trampoline from C library to typed slot.
   *
   * C 库已做完 length 校验 + memcpy 到 frame_buf, 此处仅 cast + 调 type-erased handler。
   */
  static int8_t Trampoline(uint8_t cmd, const void* data,
                           uint16_t size, void* ctx) {
    (void)cmd; (void)size;
    const auto* slot = static_cast<const Slot*>(ctx);
    if (slot->typed) {
      slot->typed(static_cast<const uint8_t*>(data));
    }
    return MRLINK_OK;
  }

  static int8_t EmptyTrampoline(uint8_t cmd, const uint8_t* payload,
                                uint16_t payload_len, void* ctx) {
    (void)cmd; (void)payload;
    const auto* slot = static_cast<const Slot*>(ctx);
    if (payload_len != 0u) {
      return MRLINK_ERR_ARGS;
    }
    if (slot->empty) {
      slot->empty();
    }
    return MRLINK_OK;
  }

  int FindSlot(uint8_t cmd) const {
    for (int i = 0; i < MAX_TYPED_HANDLERS; i++) {
      if (slots_[i].used && slots_[i].cmd == cmd) {
        return i;
      }
    }
    return -1;
  }

  int AllocSlot() {
    for (int i = 0; i < MAX_TYPED_HANDLERS; i++) {
      if (!slots_[i].used) {
        return i;
      }
    }
    return -1;
  }
};

}  // namespace mr::link
