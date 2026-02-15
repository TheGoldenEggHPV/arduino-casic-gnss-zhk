#include "Casic.h"
#include <math.h>
#include <string.h>

float CasicNavPv::hAcc_1sigma_m() const {
  if (!isfinite(hAcc_var_m2) || hAcc_var_m2 < 0) return NAN;
  return sqrtf(hAcc_var_m2);
}
float CasicNavPv::vAcc_1sigma_m() const {
  if (!isfinite(vAcc_var_m2) || vAcc_var_m2 < 0) return NAN;
  return sqrtf(vAcc_var_m2);
}
float CasicNavPv::pos3D_1sigma_m() const {
  if (!isfinite(hAcc_var_m2) || !isfinite(vAcc_var_m2) || hAcc_var_m2 < 0 || vAcc_var_m2 < 0) return NAN;
  return sqrtf(hAcc_var_m2 + vAcc_var_m2);
}
float CasicNavPv::speedAcc_1sigma_mps() const {
  if (!isfinite(sAcc_var_m2ps2) || sAcc_var_m2ps2 < 0) return NAN;
  return sqrtf(sAcc_var_m2ps2);
}
float CasicNavPv::altitude_msl_m() const {
  if (!isfinite(height_ellipsoidal_m) || !isfinite(sepGeoid_m)) return NAN;
  return height_ellipsoidal_m - sepGeoid_m;
}

Casic::Casic()
: _state(IDLE),
  _len(0),
  _remaining(0),
  _bufLen(0),
  _navPvUpdated(false),
  _satUpdated(false),
  _portCfgUpdated(false),
  _framesOk(0),
  _framesBadCk(0),
  _framesDropped(0),
  _cfgResult(CasicCfgResult::None),
  _cfgPending(false),
  _cfgExpectCls(0),
  _cfgExpectId(0),
  _cfgDeadlineMs(0)
{
  resetFrame();
}

void Casic::resetFrame() {
  _len = 0;
  _remaining = 0;
  _bufLen = 0;
}

void Casic::processStream(Stream& s) {
  while (s.available()) {
    processByte((uint8_t)s.read());
  }
}

void Casic::processByte(uint8_t b) {
  switch (_state) {
    case IDLE:
      if (b == SYNC1_BYTE) {
        resetFrame();
        _buf[_bufLen++] = b;
        _state = SYNC2;
      }
      break;

    case SYNC2:
      if (b == SYNC2_BYTE) {
        _buf[_bufLen++] = b;
        _state = LEN1;
      } else {
        _framesDropped++;
        _state = IDLE;
        if (b == SYNC1_BYTE) {
          resetFrame();
          _buf[_bufLen++] = b;
          _state = SYNC2;
        }
      }
      break;

    case LEN1:
      _buf[_bufLen++] = b;
      _len = b;
      _state = LEN2;
      break;

    case LEN2:
      _buf[_bufLen++] = b;
      _len |= ((uint16_t)b << 8);
      if (_len > MAX_PAYLOAD) {
        _framesDropped++;
        _state = IDLE;
        resetFrame();
      } else {
        _remaining = (uint16_t)(2 + _len + 4);
        _state = BODY;
      }
      break;

    case BODY:
      if (_bufLen >= MAX_FRAME) {
        _framesDropped++;
        _state = IDLE;
        resetFrame();
        break;
      }
      _buf[_bufLen++] = b;
      if (_remaining > 0) _remaining--;
      if (_remaining == 0) {
        onFrameComplete();
        _state = IDLE;
        resetFrame();
      }
      break;
  }
}

bool Casic::navPvUpdated() {
  bool v = _navPvUpdated;
  _navPvUpdated = false;
  return v;
}

bool Casic::satUpdated() {
  bool v = _satUpdated;
  _satUpdated = false;
  return v;
}

bool Casic::portConfigUpdated() {
  bool v = _portCfgUpdated;
  _portCfgUpdated = false;
  return v;
}

CasicCfgResult Casic::readCfgResult() {
  CasicCfgResult r = _cfgResult;
  _cfgResult = CasicCfgResult::None;
  return r;
}

uint32_t Casic::readLEU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
uint16_t Casic::readLEU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
int16_t Casic::readLEI16(const uint8_t* p) {
  return (int16_t)readLEU16(p);
}
float Casic::readLEFloat(const uint8_t* p) {
  float f;
  memcpy(&f, p, sizeof(float));
  return f;
}
double Casic::readLEDouble(const uint8_t* p) {
  double d;
  memcpy(&d, p, sizeof(double));
  return d;
}

uint32_t Casic::calcChecksum(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length) {
  uint32_t ck = ((uint32_t)id << 24) | ((uint32_t)cls << 16) | (uint32_t)length;

  uint16_t pad = (uint16_t)((4 - (length % 4)) % 4);
  uint16_t total = (uint16_t)(length + pad);

  for (uint16_t i = 0; i < total; i += 4) {
    uint32_t w = 0;
    for (uint8_t k = 0; k < 4; k++) {
      uint16_t idx = (uint16_t)(i + k);
      uint8_t byte = (idx < length) ? payload[idx] : 0;
      w |= ((uint32_t)byte << (8 * k));
    }
    ck += w;
  }
  return ck;
}

void Casic::writeFrame(Stream& out, uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length) {
  uint8_t hdr[6];
  hdr[0] = SYNC1_BYTE;
  hdr[1] = SYNC2_BYTE;
  hdr[2] = (uint8_t)(length & 0xFF);
  hdr[3] = (uint8_t)((length >> 8) & 0xFF);
  hdr[4] = cls;
  hdr[5] = id;

  uint32_t ck = calcChecksum(cls, id, payload, length);
  uint8_t ckbytes[4] = {
    (uint8_t)(ck & 0xFF),
    (uint8_t)((ck >> 8) & 0xFF),
    (uint8_t)((ck >> 16) & 0xFF),
    (uint8_t)((ck >> 24) & 0xFF),
  };

  out.write(hdr, sizeof(hdr));
  if (length && payload) out.write(payload, length);
  out.write(ckbytes, sizeof(ckbytes));
}

bool Casic::beginCfgTxn(uint8_t expectCls, uint8_t expectId, uint32_t timeoutMs) {
  if (_cfgPending) return false;
  _cfgPending = true;
  _cfgResult = CasicCfgResult::Pending;
  _cfgExpectCls = expectCls;
  _cfgExpectId = expectId;
  _cfgDeadlineMs = millis() + timeoutMs;
  return true;
}

void Casic::serviceConfig() {
  if (_cfgPending) {
    if ((int32_t)(millis() - _cfgDeadlineMs) > 0) {
      _cfgPending = false;
      _cfgResult = CasicCfgResult::Timeout;
    }
  }
}

CasicCfgResult Casic::waitForCfgResult(Stream& io, uint32_t timeoutMs) {
  // If nothing pending, return current (possibly None).
  uint32_t deadline = millis() + timeoutMs;

  // If a transaction was just started, _cfgResult should be Pending.
  while (true) {
    // Pump RX so ACK/NAK can be parsed
    while (io.available()) {
      processByte((uint8_t)io.read());
    }
    serviceConfig();

    CasicCfgResult r = peekCfgResult();
    if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout) {
      return readCfgResult(); // clear + return
    }

    if ((int32_t)(millis() - deadline) > 0) {
      // Force a timeout result if we still have a pending txn
      if (_cfgPending) {
        _cfgPending = false;
        _cfgResult = CasicCfgResult::Timeout;
      }
      return readCfgResult();
    }

    // Small yield to avoid pegging CPU
    delay(1);
  }
}

bool Casic::cfgSetMsgRateAndWait(Stream& io, uint8_t msg_cls, uint8_t msg_id, uint16_t rate, uint32_t timeoutMs, CasicCfgResult* outResult) {
  if (!cfgSetMsgRate(io, msg_cls, msg_id, rate)) return false;
  CasicCfgResult r = waitForCfgResult(io, timeoutMs);
  if (outResult) *outResult = r;
  return true;
}

bool Casic::cfgSetRateAndWait(Stream& io, uint16_t interval_ms, uint32_t timeoutMs, CasicCfgResult* outResult) {
  if (!cfgSetRate(io, interval_ms)) return false;
  CasicCfgResult r = waitForCfgResult(io, timeoutMs);
  if (outResult) *outResult = r;
  return true;
}

bool Casic::cfgSaveAndWait(Stream& io, uint16_t mask, uint32_t timeoutMs, CasicCfgResult* outResult) {
  if (!cfgSave(io, mask)) return false;
  CasicCfgResult r = waitForCfgResult(io, timeoutMs);
  if (outResult) *outResult = r;
  return true;
}


void Casic::beginEnableBinary(Stream& out, uint8_t port_id, bool enableBinOut, bool disableTextOut, uint32_t newBaud) {
  (void)enableBinOut;
  (void)disableTextOut;
  (void)newBaud;
  // Step 1: query port. You can then modify proto_mask/mode/baud and call cfgSetPort().
  cfgQueryPort(out, port_id);
}

bool Casic::cfgSetMsgRate(Stream& out, uint8_t msg_cls, uint8_t msg_id, uint16_t rate) {
  if (!beginCfgTxn(0x06, 0x01)) return false;
  uint8_t payload[4];
  buildCfgMsgSet(msg_cls, msg_id, rate, payload);
  writeFrame(out, 0x06, 0x01, payload, sizeof(payload));
  return true;
}

bool Casic::cfgSave(Stream& out, uint16_t mask) {
  if (!beginCfgTxn(0x06, 0x05)) return false;
  uint8_t payload[4];
  buildCfgCfg(mask, 1, payload);
  writeFrame(out, 0x06, 0x05, payload, sizeof(payload));
  return true;
}

bool Casic::cfgQueryPort(Stream& out, uint8_t port_id) {
  if (!beginCfgTxn(0x06, 0x00)) return false;
  uint8_t payload[1];
  buildCfgPrtQuery(port_id, payload);
  writeFrame(out, 0x06, 0x00, payload, sizeof(payload));
  return true;
}

bool Casic::cfgSetPort(Stream& out, uint8_t port_id, uint8_t proto_mask, uint16_t mode, uint32_t baud) {
  if (!beginCfgTxn(0x06, 0x00)) return false;
  uint8_t payload[8];
  buildCfgPrtSet(port_id, proto_mask, mode, baud, payload);
  writeFrame(out, 0x06, 0x00, payload, sizeof(payload));
  return true;
}

bool Casic::cfgSetRate(Stream& out, uint16_t interval_ms) {
  // CFG-RATE is class 0x06 id 0x04, payload 4 bytes: interval_ms (U2), reserved (U2=0)
  if (!beginCfgTxn(0x06, 0x04)) return false;
  uint8_t payload[4];
  buildCfgRateSet(interval_ms, payload);
  writeFrame(out, 0x06, 0x04, payload, sizeof(payload));
  return true;
}


void Casic::buildCfgMsgSet(uint8_t msg_cls, uint8_t msg_id, uint16_t rate, uint8_t outPayload[4]) {
  outPayload[0] = msg_cls;
  outPayload[1] = msg_id;
  outPayload[2] = (uint8_t)(rate & 0xFF);
  outPayload[3] = (uint8_t)((rate >> 8) & 0xFF);
}

void Casic::buildCfgCfg(uint16_t mask, uint8_t mode, uint8_t outPayload[4]) {
  outPayload[0] = (uint8_t)(mask & 0xFF);
  outPayload[1] = (uint8_t)((mask >> 8) & 0xFF);
  outPayload[2] = mode;
  outPayload[3] = 0;
}

void Casic::buildCfgPrtSet(uint8_t port_id, uint8_t proto_mask, uint16_t mode, uint32_t baud, uint8_t outPayload[8]) {
  outPayload[0] = port_id;
  outPayload[1] = proto_mask;
  outPayload[2] = (uint8_t)(mode & 0xFF);
  outPayload[3] = (uint8_t)((mode >> 8) & 0xFF);
  outPayload[4] = (uint8_t)(baud & 0xFF);
  outPayload[5] = (uint8_t)((baud >> 8) & 0xFF);
  outPayload[6] = (uint8_t)((baud >> 16) & 0xFF);
  outPayload[7] = (uint8_t)((baud >> 24) & 0xFF);
}

void Casic::buildCfgPrtQuery(uint8_t port_id, uint8_t outPayload[1]) {
  outPayload[0] = port_id;
}

void Casic::buildCfgRateSet(uint16_t interval_ms, uint8_t outPayload[4]) {
  outPayload[0] = (uint8_t)(interval_ms & 0xFF);
  outPayload[1] = (uint8_t)((interval_ms >> 8) & 0xFF);
  outPayload[2] = 0;
  outPayload[3] = 0;
}


void Casic::onFrameComplete() {
  if (_bufLen < 10) return;

  uint16_t length = (uint16_t)_buf[2] | ((uint16_t)_buf[3] << 8);
  if (length != _len) return;

  uint8_t cls = _buf[4];
  uint8_t id  = _buf[5];
  const uint8_t* payload = &_buf[6];
  const uint8_t* ckptr = &_buf[6 + length];

  uint32_t rxCk = readLEU32(ckptr);
  uint32_t calcCk = calcChecksum(cls, id, payload, length);

  if (rxCk != calcCk) {
    _framesBadCk++;
    return;
  }

  _framesOk++;

  // ACK / NAK
  if (cls == 0x05 && (id == 0x01 || id == 0x00) && length >= 4) {
    uint8_t acked_cls = payload[0];
    uint8_t acked_id  = payload[1];

    if (_cfgPending && acked_cls == _cfgExpectCls && acked_id == _cfgExpectId) {
      _cfgPending = false;
      _cfgResult = (id == 0x01) ? CasicCfgResult::Ack : CasicCfgResult::Nak;
    }
    return;
  }

  // CFG-PRT response
  if (cls == 0x06 && id == 0x00 && length >= 8) {
    _portCfg.port_id = payload[0];
    _portCfg.proto_mask = payload[1];
    _portCfg.mode = readLEU16(payload + 2);
    _portCfg.baud_rate = readLEU32(payload + 4);
    _portCfgUpdated = true;
    return;
  }

    // CFG-RATE response (interval_ms, reserved)
  if (cls == 0x06 && id == 0x04 && length >= 4) {
    // uint16_t interval_ms = readLEU16(payload + 0); // available if you want it
    return;
  }

// NAV-PV
  if (cls == 0x01 && id == 0x03 && length >= 80) {
    CasicNavPv pv;
    pv.runTime_ms = readLEU32(payload + 0);
    pv.posValid = payload[4];
    pv.velValid = payload[5];
    pv.systemMask = payload[6];
    pv.numSV = payload[7];
    pv.numSVGPS = payload[8];
    pv.numSVBDS = payload[9];
    pv.numSVGLN = payload[10];
    pv.pDop = readLEFloat(payload + 12);
    pv.lon_deg = readLEDouble(payload + 16);
    pv.lat_deg = readLEDouble(payload + 24);
    pv.height_ellipsoidal_m = readLEFloat(payload + 32);
    pv.sepGeoid_m = readLEFloat(payload + 36);
    pv.hAcc_var_m2 = readLEFloat(payload + 40);
    pv.vAcc_var_m2 = readLEFloat(payload + 44);
    pv.velN_mps = readLEFloat(payload + 48);
    pv.velE_mps = readLEFloat(payload + 52);
    pv.velU_mps = readLEFloat(payload + 56);
    pv.speed3D_mps = readLEFloat(payload + 60);
    pv.speed2D_mps = readLEFloat(payload + 64);
    pv.heading_deg = readLEFloat(payload + 68);
    pv.sAcc_var_m2ps2 = readLEFloat(payload + 72);
    pv.cAcc_var_deg2 = readLEFloat(payload + 76);

    _navPv = pv;
    _navPvUpdated = true;
    return;
  }

  // NAV-xxxINFO
  if (cls == 0x01 && (id == 0x20 || id == 0x21 || id == 0x22) && length >= 8) {
    CasicSatInfo si;
    si.runTime_ms = readLEU32(payload + 0);
    si.numViewSv = payload[4];
    si.numFixSv = payload[5];
    si.systemType = payload[6];

    uint8_t n = si.numViewSv;
    if (n > 32) n = 32;

    for (uint8_t i = 0; i < n; i++) {
      uint16_t off = (uint16_t)(8 + 12 * i);
      if ((uint16_t)(off + 12) > length) break;
      CasicSat s;
      s.chn = payload[off + 0];
      s.svid = payload[off + 1];
      s.flags = payload[off + 2];
      s.quality = payload[off + 3];
      s.cno_dbhz = payload[off + 4];
      s.elev_deg = (int8_t)payload[off + 5];
      s.azim_deg = readLEI16(payload + off + 6);
      s.prRes_m = readLEFloat(payload + off + 8);
      si.sats[i] = s;
    }

    _satInfo = si;
    _satUpdated = true;
    return;
  }
}
