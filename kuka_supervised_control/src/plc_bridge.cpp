#include "kuka_supervised_control/plc_bridge.hpp"

#include <snap7.h>
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace kuka_supervised_control
{

PlcBridge::~PlcBridge()
{
  disconnect();
}

bool PlcBridge::configure(const PlcConfig & c) noexcept
{
  if (c.ip.empty() || c.rack < 0 || c.slot < 0) return false;
  if (c.db_proposal <= 0 || c.db_feedback <= 0) return false;
  if (c.connect_timeout_ms <= 0 || c.recv_timeout_ms <= 0 || c.send_timeout_ms <= 0) return false;
  if (c.reconnect_min_ms <= 0 || c.reconnect_max_ms < c.reconnect_min_ms) return false;
  if (c.proposal_size != 72 || c.feedback_size != 192) return false;

  cfg_ = c;
  backoff_ms_ = c.reconnect_min_ms;
  return true;
}

IoResult PlcBridge::try_connect(Clock::time_point now)
{
  IoResult r;
  if (connected_) return r;
  if (now < next_attempt_) {
    r.status = IoStatus::NotConnected;
    return r;
  }

  if (client_ == 0) {
    client_ = Cli_Create();
    if (client_ == 0) {
      r.status = IoStatus::ConnectFailed;
      r.error_code = -1;
      schedule_reconnect(now);
      return r;
    }
    int ping_ms = cfg_.connect_timeout_ms;
    int recv_ms = cfg_.recv_timeout_ms;
    int send_ms = cfg_.send_timeout_ms;
    Cli_SetParam(client_, p_i32_PingTimeout, &ping_ms);
    Cli_SetParam(client_, p_i32_RecvTimeout, &recv_ms);
    Cli_SetParam(client_, p_i32_SendTimeout, &send_ms);
  }

  const int res = Cli_ConnectTo(client_, cfg_.ip.c_str(), cfg_.rack, cfg_.slot);
  if (res != 0) {
    r.status = IoStatus::ConnectFailed;
    r.error_code = res;
    destroy_client();
    schedule_reconnect(now);
    return r;
  }

  connected_ = true;
  backoff_ms_ = cfg_.reconnect_min_ms;
  next_attempt_ = Clock::time_point{};
  reconnect_count_.fetch_add(1, std::memory_order_relaxed);
  return r;
}

void PlcBridge::disconnect() noexcept
{
  if (client_ != 0) {
    if (connected_) {
      Cli_Disconnect(client_);
    }
    destroy_client();
  }
  connected_ = false;
}

void PlcBridge::destroy_client() noexcept
{
  if (client_ != 0) {
    Cli_Destroy(&client_);
    client_ = 0;
  }
}

void PlcBridge::schedule_reconnect(Clock::time_point now) noexcept
{
  next_attempt_ = now + std::chrono::milliseconds(backoff_ms_);
  backoff_ms_ = std::min(backoff_ms_ * 2, cfg_.reconnect_max_ms);
}

void PlcBridge::teardown_on_io_error() noexcept
{
  connected_ = false;
  io_error_count_.fetch_add(1, std::memory_order_relaxed);
  if (client_ != 0) {
    Cli_Disconnect(client_);
    destroy_client();
  }
  schedule_reconnect(Clock::now());
}

IoResult PlcBridge::read_feedback(uint8_t * buf, size_t len)
{
  IoResult r;
  if (!connected_) {
    r.status = IoStatus::NotConnected;
    return r;
  }
  if (len != cfg_.feedback_size) {
    r.status = IoStatus::InvalidSize;
    return r;
  }

  const int res = Cli_DBRead(client_, cfg_.db_feedback, 0, static_cast<int>(len), buf);
  if (res != 0) {
    r.status = IoStatus::ReadFailed;
    r.error_code = res;
    teardown_on_io_error();
    return r;
  }
  return r;
}

IoResult PlcBridge::write_proposal(const uint8_t * buf, size_t len)
{
  IoResult r;
  if (!connected_) {
    r.status = IoStatus::NotConnected;
    return r;
  }
  if (len != cfg_.proposal_size) {
    r.status = IoStatus::InvalidSize;
    return r;
  }

  const int res = Cli_DBWrite(
    client_, cfg_.db_proposal, 0, static_cast<int>(len), const_cast<uint8_t *>(buf));
  if (res != 0) {
    r.status = IoStatus::WriteFailed;
    r.error_code = res;
    teardown_on_io_error();
    return r;
  }
  return r;
}

void PlcBridge::error_text(int code, char * buf, size_t cap) const noexcept
{
  if (!buf || cap == 0) return;
  if (code == 0) {
    std::snprintf(buf, cap, "OK");
    return;
  }
  Cli_ErrorText(code, buf, static_cast<int>(cap - 1));
  buf[cap - 1] = '\0';
}

}  // namespace kuka_supervised_control
