#include <Casic.h>

Casic gps;

/*
  This example:
  1) Queries current port config (CFG-PRT)
  2) Sets port config to enable binary output and disable NMEA (text) output
  3) Sets navigation rate with CFG-RATE (e.g., 100 ms = 10 Hz)
  4) Enables NAV-PV (speed3D/alt/accuracy) and NAV-GPSINFO (sat list)
  5) (Optional) Save to NVM

  NOTE: If you change baud rate via cfgSetPort, you must also change Serial1.begin()
  after you get ACK for the port change.
*/

enum Step {
  S_QUERY_PRT,
  S_WAIT_PRT,
  S_SET_PRT,
  S_WAIT_SET_PRT,
  S_SET_RATE,
  S_WAIT_RATE,
  S_ENABLE_NAVPV,
  S_WAIT_NAVPV,
  S_ENABLE_GPSINFO,
  S_WAIT_GPSINFO,
  S_DONE
};

Step stepState = S_QUERY_PRT;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial1.begin(115200);   // Adjust to your module's current baud

  Serial.println(F("Disable NMEA + Faster updates (CASIC binary)"));
  Serial.println(F("Starting config..."));
}

void loop() {
  gps.processStream(Serial1);
  gps.serviceConfig();

  CasicCfgResult r = gps.peekCfgResult();

  switch (stepState) {
    case S_QUERY_PRT:
      if (gps.cfgQueryPort(Serial1, 1)) { // port_id=1 is common for UART1; adjust if needed
        stepState = S_WAIT_PRT;
      }
      break;

    case S_WAIT_PRT:
      if (gps.portConfigUpdated()) {
        const CasicPortConfig& pc = gps.portConfig();
        Serial.print(F("Port ")); Serial.print(pc.port_id);
        Serial.print(F(" proto_mask=0x")); Serial.print(pc.proto_mask, HEX);
        Serial.print(F(" mode=0x")); Serial.print(pc.mode, HEX);
        Serial.print(F(" baud=")); Serial.println(pc.baud_rate);

        // Build new mask: enable binary out (0x10), disable text out (0x20)
        uint8_t newMask = pc.proto_mask | 0x10;
        newMask = (uint8_t)(newMask & ~0x20);

        // Keep mode and baud the same here (fast + safe)
        if (gps.cfgSetPort(Serial1, pc.port_id, newMask, pc.mode, pc.baud_rate)) {
          stepState = S_WAIT_SET_PRT;
        } else {
          stepState = S_SET_PRT;
        }
      }
      break;

    case S_WAIT_SET_PRT:
      if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout) {
        Serial.print(F("CFG-PRT result: ")); Serial.println((int)gps.readCfgResult());
        stepState = S_SET_RATE;
      }
      break;

    case S_SET_RATE:
      // 100 ms = 10 Hz navigation solution
      if (gps.cfgSetRate(Serial1, 100)) {
        stepState = S_WAIT_RATE;
      }
      break;

    case S_WAIT_RATE:
      if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout) {
        Serial.print(F("CFG-RATE result: ")); Serial.println((int)gps.readCfgResult());
        stepState = S_ENABLE_NAVPV;
      }
      break;

    case S_ENABLE_NAVPV:
      if (gps.cfgSetMsgRate(Serial1, 0x01, 0x03, 1)) { // NAV-PV
        stepState = S_WAIT_NAVPV;
      }
      break;

    case S_WAIT_NAVPV:
      if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout) {
        Serial.print(F("NAV-PV enable result: ")); Serial.println((int)gps.readCfgResult());
        stepState = S_ENABLE_GPSINFO;
      }
      break;

    case S_ENABLE_GPSINFO:
      if (gps.cfgSetMsgRate(Serial1, 0x01, 0x20, 1)) { // NAV-GPSINFO
        stepState = S_WAIT_GPSINFO;
      }
      break;

    case S_WAIT_GPSINFO:
      if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout) {
        Serial.print(F("NAV-GPSINFO enable result: ")); Serial.println((int)gps.readCfgResult());
        stepState = S_DONE;

        Serial.println(F("Config done (not saved). To persist, call gps.cfgSave(Serial1) now."));
      }
      break;

    case S_DONE:
      // Print a couple of live fields as confirmation
      if (gps.navPvUpdated()) {
        const CasicNavPv& pv = gps.navPv();
        Serial.print(F("speed3D m/s=")); Serial.print(pv.speed3D_mps, 2);
        Serial.print(F(" altMSL m=")); Serial.print(pv.altitude_msl_m(), 2);
        Serial.print(F(" hAcc1s m=")); Serial.print(pv.hAcc_1sigma_m(), 2);
        Serial.print(F(" sAcc1s m/s=")); Serial.println(pv.speedAcc_1sigma_mps(), 2);
      }
      break;

    default: break;
  }
}
