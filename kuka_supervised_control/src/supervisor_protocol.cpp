#include "kuka_supervised_control/supervisor_protocol.hpp"
#include <cstdio>
#include <cstring>

namespace kuka_supervised_control
{

// Compile-time CRC32 table (polynomial 0xEDB88320, IEEE 802.3 / zlib).
namespace {
constexpr std::array<uint32_t, 256> make_crc_table() {
  std::array<uint32_t, 256> t{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int j = 0; j < 8; ++j) c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    t[i] = c;
  }
  return t;
}
constexpr auto kCrcTable = make_crc_table();
}  // namespace

uint32_t crc32_ieee(const uint8_t * data, size_t len)
{
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) c = kCrcTable[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

// ═══════════════════════════════════════════════════════════════════
void ProposalMsg::encode(uint8_t out[kDbProposalSize]) const
{
  std::memset(out, 0, kDbProposalSize);
  size_t off = 0;
  for (size_t i = 0; i < kNumJoints; ++i) { be::put_f64(out + off, proposed_pos_deg[i]); off += 8; }
  be::put_u32(out + off, seq);            off += 4;
  be::put_i32(out + off, hb_echo);        off += 4;
  be::put_u16(out + off, proposal_id);    off += 2;
  be::put_u16(out + off, state_request);  off += 2;
  out[off++] = flags;
  out[off++] = kProtocolVersion;
  off += 2;
  be::put_u32(out + off, 0u);             off += 4;   // reserved
  be::put_u32(out + off, crc32_ieee(out, off));
}

bool ProposalMsg::decode(const uint8_t in[kDbProposalSize], ProposalMsg & out)
{
  size_t off = 0;
  for (size_t i = 0; i < kNumJoints; ++i) { out.proposed_pos_deg[i] = be::get_f64(in + off); off += 8; }
  out.seq           = be::get_u32(in + off); off += 4;
  out.hb_echo       = be::get_i32(in + off); off += 4;
  out.proposal_id   = be::get_u16(in + off); off += 2;
  out.state_request = be::get_u16(in + off); off += 2;
  out.flags         = in[off++];
  const uint8_t ver = in[off++];
  off += 2 + 4;   // pad + reserved
  const uint32_t rx  = be::get_u32(in + off);
  const uint32_t cal = crc32_ieee(in, off);
  return rx == cal && ver == kProtocolVersion;
}

// ═══════════════════════════════════════════════════════════════════
void FeedbackMsg::encode(uint8_t out[kDbFeedbackSize]) const
{
  std::memset(out, 0, kDbFeedbackSize);
  size_t off = 0;
  for (size_t i = 0; i < kNumJoints; ++i) { be::put_f64(out + off, actual_pos_deg[i]);   off += 8; }
  for (size_t i = 0; i < kNumJoints; ++i) { be::put_f64(out + off, actual_vel_deg_s[i]); off += 8; }
  be::put_i32(out + off, plc_heartbeat);   off += 4;
  be::put_u16(out + off, plc_state);       off += 2;
  be::put_u16(out + off, decision);        off += 2;
  be::put_u16(out + off, reject_reason);   off += 2;
  be::put_u16(out + off, ack_proposal_id); off += 2;
  be::put_u32(out + off, ack_seq);         off += 4;
  out[off++] = plc_flags;
  off += 3;
  for (size_t i = 0; i < kNumJoints; ++i) { be::put_f64(out + off, max_step_deg[i]); off += 8; }
  be::put_f32(out + off, max_vel_deg_s);   off += 4;
  be::put_u32(out + off, crc32_ieee(out, off));
}

bool FeedbackMsg::decode(const uint8_t in[kDbFeedbackSize], FeedbackMsg & out)
{
  size_t off = 0;
  for (size_t i = 0; i < kNumJoints; ++i) { out.actual_pos_deg[i]   = be::get_f64(in + off); off += 8; }
  for (size_t i = 0; i < kNumJoints; ++i) { out.actual_vel_deg_s[i] = be::get_f64(in + off); off += 8; }
  out.plc_heartbeat   = be::get_i32(in + off); off += 4;
  out.plc_state       = be::get_u16(in + off); off += 2;
  out.decision        = be::get_u16(in + off); off += 2;
  out.reject_reason   = be::get_u16(in + off); off += 2;
  out.ack_proposal_id = be::get_u16(in + off); off += 2;
  out.ack_seq         = be::get_u32(in + off); off += 4;
  out.plc_flags       = in[off++];
  off += 3;
  for (size_t i = 0; i < kNumJoints; ++i) { out.max_step_deg[i] = be::get_f64(in + off); off += 8; }
  out.max_vel_deg_s   = be::get_f32(in + off); off += 4;
  const uint32_t rx  = be::get_u32(in + off);
  const uint32_t cal = crc32_ieee(in, off);
  return rx == cal;
}

// ═══════════════════════════════════════════════════════════════════
const char * to_string(PlcState s)
{
  switch (s) {
    case PlcState::Boot:     return "BOOT";
    case PlcState::CommWait: return "COMM_WAIT";
    case PlcState::Idle:     return "IDLE";
    case PlcState::Enabling: return "ENABLING";
    case PlcState::Ready:    return "READY";
    case PlcState::Running:  return "RUNNING";
    case PlcState::Fault:    return "FAULT";
    case PlcState::Estop:    return "ESTOP";
    case PlcState::Lost:     return "LOST";
  }
  return "?";
}
const char * to_string(Decision d)
{
  switch (d) {
    case Decision::Pending:  return "PENDING";
    case Decision::Accepted: return "ACCEPTED";
    case Decision::Rejected: return "REJECTED";
    case Decision::Done:     return "DONE";
    case Decision::Aborted:  return "ABORTED";
  }
  return "?";
}

void reject_reason_strings(uint16_t m, char * out, size_t cap)
{
  if (!cap) return;
  out[0] = '\0';
  if (!m) { std::snprintf(out, cap, "NONE"); return; }
  struct E { uint16_t b; const char * n; };
  static const E k[] = {
    {RejBadState,"BAD_STATE"},{RejSoftLimit,"SOFT_LIMIT"},{RejStepLimit,"STEP_LIMIT"},
    {RejVelLimit,"VEL_LIMIT"},{RejNotBumpless,"NOT_BUMPLESS"},{RejStaleSeq,"STALE_SEQ"},
    {RejCrcError,"CRC_ERROR"},{RejEstopActive,"ESTOP"},{RejFaultActive,"FAULT"},
    {RejTrackingError,"TRACKING"},{RejHwLimit,"HW_LIMIT"},{RejSwLimit,"SW_LIMIT"},
  };
  size_t used = 0;
  for (const auto & e : k) {
    if (!(m & e.b)) continue;
    const int n = std::snprintf(out + used, cap - used, "%s%s", used ? "|" : "", e.n);
    if (n < 0 || size_t(n) >= cap - used) break;
    used += size_t(n);
  }
}

}  // namespace kuka_supervised_control
