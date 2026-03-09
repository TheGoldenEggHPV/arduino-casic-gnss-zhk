#pragma once
#include <Arduino.h>

struct CasicNavPv
{
  uint32_t runTime_ms = 0;
  uint8_t posValid = 0;
  uint8_t velValid = 0;
  uint8_t systemMask = 0;
  uint8_t numSV = 0;
  uint8_t numSVGPS = 0;
  uint8_t numSVBDS = 0;
  uint8_t numSVGLN = 0;

  float pDop = NAN;

  double lon_deg = NAN;
  double lat_deg = NAN;

  float height_ellipsoidal_m = NAN;
  float sepGeoid_m = NAN;

  float hAcc_var_m2 = NAN;
  float vAcc_var_m2 = NAN;

  float velN_mps = NAN;
  float velE_mps = NAN;
  float velU_mps = NAN;

  float speed3D_mps = NAN;
  float speed2D_mps = NAN;

  float heading_deg = NAN;

  float sAcc_var_m2ps2 = NAN;
  float cAcc_var_deg2 = NAN;

  float hAcc_1sigma_m() const;
  float vAcc_1sigma_m() const;
  float pos3D_1sigma_m() const;
  float speedAcc_1sigma_mps() const;
  float altitude_msl_m() const;
};

struct CasicSat
{
  uint8_t chn = 0;
  uint8_t svid = 0;
  uint8_t flags = 0;
  uint8_t quality = 0;
  uint8_t cno_dbhz = 0;
  int8_t elev_deg = 0;
  int16_t azim_deg = 0;
  float prRes_m = NAN;

  bool usedInSolution() const { return (flags & 0x01) != 0; }
};

struct CasicSatInfo
{
  uint32_t runTime_ms = 0;
  uint8_t numViewSv = 0;
  uint8_t numFixSv = 0;
  uint8_t systemType = 0; // 0=GPS,1=BDS,2=GLONASS
  CasicSat sats[32];
};

struct CasicPortConfig
{
  uint8_t port_id = 0;
  uint8_t proto_mask = 0;
  uint16_t mode = 0;
  uint32_t baud_rate = 0;
};

enum class CasicCfgResult : uint8_t
{
  None = 0,
  Pending,
  Ack,
  Nak,
  Timeout,
};

class Casic
{
public:
  Casic();

  void processByte(uint8_t b);
  void processStream(Stream &s);

  bool navPvUpdated();
  const CasicNavPv &navPv() const { return _navPv; }

  bool satUpdated();
  const CasicSatInfo &satInfo() const { return _satInfo; }

  // ---------------- Configuration (TX) ----------------
  // CASIC expects only one CFG message "in flight" at a time (wait for ACK/NAK).
  bool cfgSetMsgRate(Stream &out, uint8_t msg_cls, uint8_t msg_id, uint16_t rate);
  bool cfgQueryPort(Stream &out, uint8_t port_id);
  bool cfgSetPort(Stream &out, uint8_t port_id, uint8_t proto_mask, uint16_t mode, uint32_t baud);

  // CFG-RATE (0x06 0x04): set navigation solution interval in milliseconds (e.g., 100 for 10Hz)
  bool cfgSetRate(Stream &out, uint16_t interval_ms);

  // Convenience: query then enable binary output / optionally disable NMEA output / optionally change baud.
  // This is a two-step process: call beginEnableBinary(), then:
  //  - wait for portConfigUpdated() to become true
  //  - call cfgSetPort() with your desired mask/mode/baud
  // (We don't auto-send the SET because the library doesn't store a Stream reference.)
  void beginEnableBinary(Stream &out, uint8_t port_id, bool enableBinOut = true, bool disableTextOut = true, uint32_t newBaud = 0);

  static constexpr uint16_t CFG_MASK_PORT = 0x0001;
  static constexpr uint16_t CFG_MASK_MSG = 0x0002;
  static constexpr uint16_t CFG_MASK_ALL = 0xFFFF;
  bool cfgSave(Stream &out, uint16_t mask = CFG_MASK_ALL);

  void serviceConfig();

  // Blocking helper: waits for ACK/NAK/Timeout while continuing to parse bytes from 'io'.
  // Returns Ack/Nak/Timeout. (Timeout occurs if no matching ACK/NAK arrives in time.)
  CasicCfgResult waitForCfgResult(Stream &io, uint32_t timeoutMs = 500);

  // Convenience: send a CFG-* message and block until its ACK/NAK/Timeout.
  // Returns false if a CFG transaction is already pending.
  bool cfgSetMsgRateAndWait(Stream &io, uint8_t msg_cls, uint8_t msg_id, uint16_t rate, uint32_t timeoutMs = 500, CasicCfgResult *outResult = nullptr);
  bool cfgSetRateAndWait(Stream &io, uint16_t interval_ms, uint32_t timeoutMs = 500, CasicCfgResult *outResult = nullptr);
  bool cfgSaveAndWait(Stream &io, uint16_t mask = CFG_MASK_ALL, uint32_t timeoutMs = 800, CasicCfgResult *outResult = nullptr);

  CasicCfgResult readCfgResult();
  CasicCfgResult peekCfgResult() const { return _cfgResult; }

  bool portConfigUpdated();
  const CasicPortConfig &portConfig() const { return _portCfg; }

  uint32_t framesOk() const { return _framesOk; }
  uint32_t framesBadCk() const { return _framesBadCk; }
  uint32_t framesDropped() const { return _framesDropped; }

private:
  enum State : uint8_t
  {
    IDLE = 0,
    SYNC2 = 1,
    LEN1 = 2,
    LEN2 = 3,
    BODY = 4
  };

  static constexpr uint8_t SYNC1_BYTE = 0xBA;
  static constexpr uint8_t SYNC2_BYTE = 0xCE;
  static constexpr uint16_t MAX_PAYLOAD = 512;
  static constexpr uint16_t MAX_FRAME = 2 + 2 + 2 + MAX_PAYLOAD + 4;

  State _state;
  uint16_t _len;
  uint16_t _remaining;

  uint8_t _buf[MAX_FRAME];
  uint16_t _bufLen;

  CasicNavPv _navPv;
  CasicSatInfo _satInfo;
  CasicPortConfig _portCfg;

  bool _navPvUpdated;
  bool _satUpdated;
  bool _portCfgUpdated;

  uint32_t _framesOk;
  uint32_t _framesBadCk;
  uint32_t _framesDropped;

  // Config transaction tracking
  CasicCfgResult _cfgResult;
  bool _cfgPending;
  uint8_t _cfgExpectCls;
  uint8_t _cfgExpectId;
  uint32_t _cfgDeadlineMs;

  void resetFrame();
  void onFrameComplete();

  static uint32_t calcChecksum(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t length);

  static float readLEFloat(const uint8_t *p);
  static double readLEDouble(const uint8_t *p);
  static uint32_t readLEU32(const uint8_t *p);
  static uint16_t readLEU16(const uint8_t *p);
  static int16_t readLEI16(const uint8_t *p);

  static void writeFrame(Stream &out, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t length);
  bool beginCfgTxn(uint8_t expectCls, uint8_t expectId, uint32_t timeoutMs = 300);

  static void buildCfgMsgSet(uint8_t msg_cls, uint8_t msg_id, uint16_t rate, uint8_t outPayload[4]);
  static void buildCfgCfg(uint16_t mask, uint8_t mode, uint8_t outPayload[4]);
  static void buildCfgPrtSet(uint8_t port_id, uint8_t proto_mask, uint16_t mode, uint32_t baud, uint8_t outPayload[8]);
  static void buildCfgPrtQuery(uint8_t port_id, uint8_t outPayload[1]);
  static void buildCfgRateSet(uint16_t interval_ms, uint8_t outPayload[4]);
};
