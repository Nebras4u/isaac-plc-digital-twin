#ifndef KUKA_SUPERVISED_CONTROL__PLC_BRIDGE_HPP_
#define KUKA_SUPERVISED_CONTROL__PLC_BRIDGE_HPP_

#include <atomic>
#include <cstdint>

#include "kuka_supervised_control/plc_transport.hpp"

namespace kuka_supervised_control
{

// Snap7 implementation of PlcTransport.
//  * explicit connect/recv/send timeouts (short, below the heartbeat timeout)
//  * exponential reconnect backoff
//  * any I/O error tears the connection down (caller reconnects)
//  * never throws, never logs, never allocates after configure()
// Snap7 has NO encryption or authentication and needs PUT/GET enabled on
// the PLC: keep it on an isolated network, plan to replace it (OPC UA).
class PlcBridge final : public PlcTransport
{
public:
  PlcBridge() = default;
  ~PlcBridge() override;
  PlcBridge(const PlcBridge &) = delete;
  PlcBridge & operator=(const PlcBridge &) = delete;

  bool     configure(const PlcConfig & cfg) noexcept override;
  IoResult try_connect(Clock::time_point now) override;
  void     disconnect() noexcept override;
  bool     connected() const noexcept override { return connected_; }
  IoResult read_feedback(uint8_t * buf, size_t len) override;
  IoResult write_proposal(const uint8_t * buf, size_t len) override;

  int reconnect_count() const noexcept override { return reconnect_count_.load(std::memory_order_relaxed); }
  int io_error_count()  const noexcept override { return io_error_count_.load(std::memory_order_relaxed); }
  void error_text(int code, char * buf, size_t cap) const noexcept override;

private:
  void schedule_reconnect(Clock::time_point now) noexcept;
  void teardown_on_io_error() noexcept;
  void destroy_client() noexcept;

  PlcConfig  cfg_{};
  uintptr_t  client_{0};          // Snap7 S7Object handle
  bool       connected_{false};
  int        backoff_ms_{100};
  Clock::time_point next_attempt_{};

  std::atomic<int> reconnect_count_{0};
  std::atomic<int> io_error_count_{0};
};

}  // namespace kuka_supervised_control

#endif
