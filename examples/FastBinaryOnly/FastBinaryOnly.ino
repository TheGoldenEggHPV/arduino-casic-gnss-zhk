#include <Casic.h>

Casic gps;

/*
  FastBinaryOnly
  -------------
  This example:
   - Sets navigation rate (CFG-RATE) to 10 Hz
   - Disables common NMEA sentences (CFG-MSG with class 0xF0)
   - Enables CASIC binary messages:
        NAV-PV      (0x01 0x03)  -> speed3D, altitude, accuracies, etc.
        NAV-GPSINFO (0x01 0x20)  -> satellite list (GPS)
        NAV-BDSINFO (0x01 0x21)  -> satellite list (BDS)   [optional]
        NAV-GLNINFO (0x01 0x22)  -> satellite list (GLONASS) [optional]
   - Optionally saves to NVM (CFG-CFG)

  No CFG-PRT is used, so there is no guessing of UART/port IDs.
*/

static void printPV(const CasicNavPv& pv) {
  Serial.print(F("speed3D=")); Serial.print(pv.speed3D_mps, 2);
  Serial.print(F(" m/s  altMSL=")); Serial.print(pv.altitude_msl_m(), 2);
  Serial.print(F(" m  hAcc1s=")); Serial.print(pv.hAcc_1sigma_m(), 2);
  Serial.print(F(" m  sAcc1s=")); Serial.print(pv.speedAcc_1sigma_mps(), 2);
  Serial.print(F(" m/s  SV=")); Serial.println(pv.numSV);
}

static void printSatSummary(const CasicSatInfo& si) {
  Serial.print(F("Sat ")); Serial.print(si.systemType);
  Serial.print(F(" visible=")); Serial.print(si.numViewSv);
  Serial.print(F(" used=")); Serial.println(si.numFixSv);
}

static void cfgPrint(const __FlashStringHelper* label, CasicCfgResult r) {
  Serial.print(label);
  Serial.print(F(": "));
  switch (r) {
    case CasicCfgResult::Ack: Serial.println(F("ACK")); break;
    case CasicCfgResult::Nak: Serial.println(F("NAK")); break;
    case CasicCfgResult::Timeout: Serial.println(F("TIMEOUT")); break;
    default: Serial.println((int)r); break;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Set to your receiver's current baud. If you're unsure, try 9600 first.
  Serial1.begin(115200);

  Serial.println(F("Configuring CASIC: fast binary only..."));

  CasicCfgResult r;

  // 10 Hz navigation solution interval
  gps.cfgSetRateAndWait(Serial1, 100, 600, &r);
  cfgPrint(F("CFG-RATE 10Hz"), r);

  // Disable common NMEA sentences (class 0xF0)
  // Note: some modules may not ACK/NAK these; timeout is okay.
  gps.cfgSetMsgRateAndWait(Serial1, 0xF0, 0x00, 0, 300, &r); cfgPrint(F("Disable NMEA GGA"), r);
  gps.cfgSetMsgRateAndWait(Serial1, 0xF0, 0x01, 0, 300, &r); cfgPrint(F("Disable NMEA GLL"), r);
  gps.cfgSetMsgRateAndWait(Serial1, 0xF0, 0x02, 0, 300, &r); cfgPrint(F("Disable NMEA GSA"), r);
  gps.cfgSetMsgRateAndWait(Serial1, 0xF0, 0x03, 0, 300, &r); cfgPrint(F("Disable NMEA GSV"), r);
  gps.cfgSetMsgRateAndWait(Serial1, 0xF0, 0x04, 0, 300, &r); cfgPrint(F("Disable NMEA RMC"), r);
  gps.cfgSetMsgRateAndWait(Serial1, 0xF0, 0x05, 0, 300, &r); cfgPrint(F("Disable NMEA VTG"), r);

  // Enable CASIC binary messages you care about
  gps.cfgSetMsgRateAndWait(Serial1, 0x01, 0x03, 1, 400, &r); cfgPrint(F("Enable NAV-PV"), r);
  gps.cfgSetMsgRateAndWait(Serial1, 0x01, 0x20, 1, 400, &r); cfgPrint(F("Enable NAV-GPSINFO"), r);

  // Optional: enable others if your receiver supports them
  gps.cfgSetMsgRateAndWait(Serial1, 0x01, 0x21, 1, 400, &r); cfgPrint(F("Enable NAV-BDSINFO"), r);
  gps.cfgSetMsgRateAndWait(Serial1, 0x01, 0x22, 1, 400, &r); cfgPrint(F("Enable NAV-GLNINFO"), r);

  // Optional: persist settings
  // gps.cfgSaveAndWait(Serial1, Casic::CFG_MASK_ALL, 1000, &r);
  // cfgPrint(F("CFG-SAVE"), r);

  Serial.println(F("Config complete. Listening..."));
}

void loop() {
  gps.processStream(Serial1);

  if (gps.navPvUpdated()) {
    printPV(gps.navPv());
  }

  if (gps.satUpdated()) {
    printSatSummary(gps.satInfo());
  }
}
