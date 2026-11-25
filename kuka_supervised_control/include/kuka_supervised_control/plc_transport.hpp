#ifndef KUKA_SUPERVISED_CONTROL__PLC_TRANSPORT_HPP_
#define KUKA_SUPERVISED_CONTROL__PLC_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace kuka_supervised_control
{

using Clock = std::chrono::steady_clock;

enum class IoStatus : uint8_t {
  Ok = 0, NotConnected, InvalidSize, ConnectFailed, ReadFailed, WriteFailed,
};

// No std::string here on purpose: no heap allocation on the RT path.
// Use PlcTransport::error_text() from a non-RT context to get the text.
struct IoResult {
  IoStatus status{IoStatus::Ok};
  int      error_code{0};
  bool ok() const noexcept { return status == IoStatus::Ok; }
};

struct PlcConfig {
  std::string ip{"192.168.99.56"};
  int    rack{0};
  int    slot{1};
  int    db_proposal{100};          // ROS -> PLC
  int    db_feedback{101};          // PLC -> ROS
  int    connect_timeout_ms{300};
  int    recv_timeout_ms{40};       // must be <= hb_timeout / 2 (validated)
  int    send_timeout_ms{40};
  int    reconnect_min_ms{100};
  int    reconnect_max_ms{2000};
  size_t proposal_size{72};
  size_t feedback_size{192};
};

// Transport abstraction: Snap7 today, OPC UA / PROFINET tomorrow, a mock
// in unit tests. Used from the executor thread ONLY (no locking).
class PlcTransport
{
public:
  virtual ~PlcTransport() = default;

  virtual bool     configure(const PlcConfig & cfg) noexcept = 0;
  virtual IoResult try_connect(Clock::time_point now) = 0;
  virtual void     disconnect() noexcept = 0;
  virtual bool     connected() const noexcept = 0;
  virtual IoResult read_feedback(uint8_t * buf, size_t len) = 0;
  virtual IoResult write_proposal(const uint8_t * buf, size_t len) = 0;

  virtual int  reconnect_count() const noexcept = 0;
  virtual int  io_error_count() const noexcept = 0;
  virtual void error_text(int code, char * buf, size_t cap) const noexcept = 0;
};

}  // namespace kuka_supervised_control

#endif
