#ifndef KUKA_SUPERVISED_CONTROL__SPSC_TRIPLE_BUFFER_HPP_
#define KUKA_SUPERVISED_CONTROL__SPSC_TRIPLE_BUFFER_HPP_

#include <atomic>
#include <cstdint>
#include <type_traits>

namespace kuka_supervised_control
{

// Wait-free single-producer / single-consumer triple buffer.
//  * write(): called by ONE thread only, never blocks.
//  * read():  called by ONE (other) thread only, never blocks.
// Neither side ever takes a mutex, so neither can stall the other
// (this replaces realtime_tools::RealtimeBuffer, whose writer locks a mutex).
// reset() must only be called while both sides are quiescent.
template <typename T>
class SpscTripleBuffer
{
  static_assert(std::is_trivially_copyable_v<T>, "T must be trivially copyable");

public:
  SpscTripleBuffer() { reset(T{}); }
  SpscTripleBuffer(const SpscTripleBuffer &) = delete;
  SpscTripleBuffer & operator=(const SpscTripleBuffer &) = delete;

  void reset(const T & init) noexcept
  {
    buf_[0] = buf_[1] = buf_[2] = init;
    write_idx_ = 0;
    read_idx_  = 2;
    middle_.store(1, std::memory_order_release);
  }

  void write(const T & v) noexcept
  {
    buf_[write_idx_] = v;
    const uint8_t old = middle_.exchange(
      static_cast<uint8_t>(write_idx_ | kDirty), std::memory_order_acq_rel);
    write_idx_ = old & kIdxMask;
  }

  // Returns a reference valid until the next read() by the same consumer.
  // Copy it immediately.
  const T & read() noexcept
  {
    if (middle_.load(std::memory_order_acquire) & kDirty) {
      const uint8_t old = middle_.exchange(read_idx_, std::memory_order_acq_rel);
      read_idx_ = old & kIdxMask;
    }
    return buf_[read_idx_];
  }

private:
  static constexpr uint8_t kDirty   = 0x80;
  static constexpr uint8_t kIdxMask = 0x03;

  T buf_[3]{};
  uint8_t write_idx_{0};          // owned by producer
  uint8_t read_idx_{2};           // owned by consumer
  std::atomic<uint8_t> middle_{1};
};

}  // namespace kuka_supervised_control

#endif
