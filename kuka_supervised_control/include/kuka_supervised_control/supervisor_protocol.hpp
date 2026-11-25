#ifndef KUKA_SUPERVISED_CONTROL__SUPERVISOR_PROTOCOL_HPP_
#define KUKA_SUPERVISED_CONTROL__SUPERVISOR_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace kuka_supervised_control
{

// Wire format. Bump kProtocolVersion on ANY layout change.
// The PLC asserts this in DB100.byte[61].
inline constexpr uint8_t kProtocolVersion = 1;
inline constexpr size_t  kNumJoints       = 6;
inline constexpr size_t  kDbProposalSize  = 72;    // DB100 (ROS -> PLC)
inline constexpr size_t  kDbFeedbackSize  = 192;   // DB101 (PLC -> ROS)
inline constexpr size_t  kProposalCrcOffset = 68;  // CRC over bytes [0..67]
inline constexpr size_t  kFeedbackCrcOffset = 168; // CRC over bytes [0..167]

inline constexpr double  kDegToRad = 0.017453292519943295;
inline constexpr double  kRadToDeg = 57.29577951308232;

// ── Enums (wire contract: SCL must match exactly) ──────────────────
enum class PlcState : uint16_t {
  Boot = 0, CommWait = 1, Idle = 2, Enabling = 3, Ready = 4,
  Running = 5, Fault = 6, Estop = 7, Lost = 8,
};
enum class Decision : uint16_t {
  Pending = 0, Accepted = 1, Rejected = 2, Done = 3, Aborted = 4,
};
enum class StateRequest : uint16_t {
  Idle = 0, Run = 1, Estop = 2, Reset = 3,
};
enum RejectReason : uint16_t {
  RejNone          = 0,
  RejBadState      = 1u << 0,
  RejSoftLimit     = 1u << 1,
  RejStepLimit     = 1u << 2,
  RejVelLimit      = 1u << 3,
  RejNotBumpless   = 1u << 4,
  RejStaleSeq      = 1u << 5,
  RejCrcError      = 1u << 6,
  RejEstopActive   = 1u << 7,
  RejFaultActive   = 1u << 8,
  RejTrackingError = 1u << 9,
  RejHwLimit       = 1u << 10,
  RejSwLimit       = 1u << 11,
};
enum RosFlags : uint8_t {
  RosFlagRequestEnable = 1u << 0,
  RosFlagRequestEstop  = 1u << 1,
  RosFlagProposalValid = 1u << 2,   // 0 = PLC must ignore proposed_pos_deg
};
enum PlcFlags : uint8_t {
  PlcFlagAxisEnable  = 1u << 0,
  PlcFlagEstopActive = 1u << 1,
  PlcFlagRosAlive    = 1u << 2,
  PlcFlagInPosition  = 1u << 3,
  PlcFlagFaultActive = 1u << 4,
  PlcFlagHomed       = 1u << 5,
};

// CRC32 (IEEE 802.3 / zlib). Implemented identically in SCL.
uint32_t crc32_ieee(const uint8_t * data, size_t len);

namespace be  // Big-endian codecs (S7 stores LReal/DWord/Word as BE)
{
inline void     put_u16(uint8_t * p, uint16_t v) { p[0]=uint8_t(v>>8); p[1]=uint8_t(v); }
inline uint16_t get_u16(const uint8_t * p) { return uint16_t((uint16_t(p[0])<<8)|p[1]); }
inline void     put_u32(uint8_t * p, uint32_t v) {
  p[0]=uint8_t(v>>24); p[1]=uint8_t(v>>16); p[2]=uint8_t(v>>8); p[3]=uint8_t(v);
}
inline uint32_t get_u32(const uint8_t * p) {
  return (uint32_t(p[0])<<24)|(uint32_t(p[1])<<16)|(uint32_t(p[2])<<8)|uint32_t(p[3]);
}
inline void     put_i32(uint8_t * p, int32_t v) { put_u32(p, uint32_t(v)); }
inline int32_t  get_i32(const uint8_t * p) { return int32_t(get_u32(p)); }
inline void     put_f32(uint8_t * p, float v) {
  uint32_t r; std::memcpy(&r, &v, 4); put_u32(p, r);
}
inline float    get_f32(const uint8_t * p) {
  uint32_t r = get_u32(p); float v; std::memcpy(&v, &r, 4); return v;
}
inline void     put_f64(uint8_t * p, double v) {
  uint64_t r; std::memcpy(&r, &v, 8);
  for (int i = 7; i >= 0; --i) *p++ = uint8_t((r >> (i * 8)) & 0xFFu);
}
inline double   get_f64(const uint8_t * p) {
  uint64_t r = 0;
  for (int i = 0; i < 8; ++i) r = (r << 8) | p[i];
  double v; std::memcpy(&v, &r, 8); return v;
}
}  // namespace be

//  DB100 — Proposal (ROS -> PLC) — 72 bytes
//    0..47   proposed_pos_deg[6]       LReal BE
//    48..51  seq                       DWord (+1 per write)
//    52..55  hb_echo                   DInt  (last plc_heartbeat seen)
//    56..57  proposal_id               Word  (+1 only on target change)
//    58..59  state_request             Word  (StateRequest)
//    60      flags                     Byte  (RosFlags)
//    61      version                   Byte  (kProtocolVersion)
//    62..63  pad
//    64..67  reserved                  DWord (=0)
//    68..71  crc                       DWord (over [0..67])
struct ProposalMsg {
  std::array<double, kNumJoints> proposed_pos_deg{};
  uint32_t seq{0};
  int32_t  hb_echo{0};
  uint16_t proposal_id{0};
  uint16_t state_request{0};
  uint8_t  flags{0};

  void encode(uint8_t out[kDbProposalSize]) const;
  // true iff CRC and version match. `out` is always filled (diagnostics)
  // but must NOT be acted upon when false is returned.
  static bool decode(const uint8_t in[kDbProposalSize], ProposalMsg & out);
};

//  DB101 — Feedback (PLC -> ROS) — 192 bytes
//    0..47    actual_pos_deg[6]        LReal BE
//    48..95   actual_vel_deg_s[6]      LReal BE
//    96..99   plc_heartbeat            DInt  (+1 every OB30)
//    100..101 plc_state                Word  (PlcState)
//    102..103 decision                 Word  (Decision)
//    104..105 reject_reason            Word  (RejectReason bitmask)
//    106..107 ack_proposal_id          Word
//    108..111 ack_seq                  DWord
//    112      plc_flags                Byte  (PlcFlags)
//    113..115 pad
//    116..163 max_step_deg[6]          LReal BE (per-cycle budget)
//    164..167 max_vel_deg_s            Real BE
//    168..171 crc                      DWord (over [0..167])
//    172..191 reserved (zeroed)
struct FeedbackMsg {
  std::array<double, kNumJoints> actual_pos_deg{};
  std::array<double, kNumJoints> actual_vel_deg_s{};
  int32_t  plc_heartbeat{0};
  uint16_t plc_state{0};
  uint16_t decision{0};
  uint16_t reject_reason{0};
  uint16_t ack_proposal_id{0};
  uint32_t ack_seq{0};
  uint8_t  plc_flags{0};
  std::array<double, kNumJoints> max_step_deg{};
  float    max_vel_deg_s{0.0f};

  void encode(uint8_t out[kDbFeedbackSize]) const;
  static bool decode(const uint8_t in[kDbFeedbackSize], FeedbackMsg & out);
};

const char * to_string(PlcState s);
const char * to_string(Decision d);
void         reject_reason_strings(uint16_t mask, char * out, size_t cap);

}  // namespace kuka_supervised_control

#endif  // KUKA_SUPERVISED_CONTROL__SUPERVISOR_PROTOCOL_HPP_
