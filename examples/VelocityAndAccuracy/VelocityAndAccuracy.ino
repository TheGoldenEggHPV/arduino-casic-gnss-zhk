#include <Casic.h>

Casic gps;

static void printPV(const CasicNavPv &pv)
{
  Serial.println(F("---- NAV-PV ----"));
  Serial.print(F("lat, lon (deg): "));
  Serial.print(pv.lat_deg, 8);
  Serial.print(F(", "));
  Serial.println(pv.lon_deg, 8);

  Serial.print(F("alt (ellipsoid) m: "));
  Serial.println(pv.height_ellipsoidal_m, 3);

  Serial.print(F("alt (MSL) m: "));
  Serial.println(pv.altitude_msl_m(), 3);

  Serial.print(F("speed3D (m/s): "));
  Serial.println(pv.speed3D_mps, 3);

  Serial.print(F("speed3D (km/h): "));
  Serial.println(pv.speed3D_mps * 3.6f, 2);

  Serial.print(F("speed acc 1-sigma (m/s): "));
  Serial.println(pv.speedAcc_1sigma_mps(), 3);

  Serial.print(F("hAcc 1-sigma (m): "));
  Serial.println(pv.hAcc_1sigma_m(), 3);

  Serial.print(F("vAcc 1-sigma (m): "));
  Serial.println(pv.vAcc_1sigma_m(), 3);

  Serial.print(F("3D pos acc 1-sigma (m): "));
  Serial.println(pv.pos3D_1sigma_m(), 3);

  Serial.print(F("SV used (total/GPS/BDS/GLN): "));
  Serial.print(pv.numSV);
  Serial.print(F(" / "));
  Serial.print(pv.numSVGPS);
  Serial.print(F(" / "));
  Serial.print(pv.numSVBDS);
  Serial.print(F(" / "));
  Serial.println(pv.numSVGLN);
}

enum CfgStep
{
  CFG_IDLE,
  CFG_NAVPV_SEND,
  CFG_NAVPV_WAIT,
  CFG_GPSINFO_SEND,
  CFG_GPSINFO_WAIT,
  CFG_DONE
};
CfgStep cfgStep = CFG_IDLE;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }

  Serial1.begin(115200);

  Serial.println(F("Casic Velocity & Accuracy example"));
  Serial.println(F("Requesting CASIC binary outputs..."));
  cfgStep = CFG_NAVPV_SEND;
}

void loop()
{
  gps.processStream(Serial1);
  gps.serviceConfig();

  if (cfgStep == CFG_NAVPV_SEND)
  {
    if (gps.cfgSetMsgRate(Serial1, 0x01, 0x03, 1))
    { // NAV-PV
      cfgStep = CFG_NAVPV_WAIT;
    }
  }
  else if (cfgStep == CFG_NAVPV_WAIT)
  {
    CasicCfgResult r = gps.peekCfgResult();
    if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout)
    {
      Serial.print(F("NAV-PV cfg result: "));
      Serial.println((int)gps.readCfgResult());
      cfgStep = CFG_GPSINFO_SEND;
    }
  }
  else if (cfgStep == CFG_GPSINFO_SEND)
  {
    if (gps.cfgSetMsgRate(Serial1, 0x01, 0x20, 1))
    { // NAV-GPSINFO
      cfgStep = CFG_GPSINFO_WAIT;
    }
  }
  else if (cfgStep == CFG_GPSINFO_WAIT)
  {
    CasicCfgResult r = gps.peekCfgResult();
    if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout)
    {
      Serial.print(F("NAV-GPSINFO cfg result: "));
      Serial.println((int)gps.readCfgResult());
      cfgStep = CFG_DONE;
      Serial.println(F("Done (not saved). Call gps.cfgSave(Serial1) if you want to persist."));
    }
  }

  if (gps.navPvUpdated())
  {
    printPV(gps.navPv());
  }
}
