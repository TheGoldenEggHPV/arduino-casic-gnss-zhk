#include <Casic.h>

Casic gps;

static const __FlashStringHelper* sysName(uint8_t sys) {
  switch (sys) {
    case 0: return F("GPS");
    case 1: return F("BDS");
    case 2: return F("GLONASS");
    default: return F("Unknown");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial1.begin(115200);

  Serial.println(F("Casic Satellite Status example"));
}

void loop() {
  gps.processStream(Serial1);

  if (gps.satUpdated()) {
    const CasicSatInfo& si = gps.satInfo();

    Serial.println(F("---- NAV-xxxINFO ----"));
    Serial.print(F("System: "));
    Serial.println(sysName(si.systemType));

    Serial.print(F("Visible SV: "));
    Serial.print(si.numViewSv);
    Serial.print(F("  Used in fix: "));
    Serial.println(si.numFixSv);

    uint8_t n = si.numViewSv;
    if (n > 32) n = 32;

    for (uint8_t i = 0; i < n; i++) {
      const CasicSat& s = si.sats[i];
      Serial.print(F("SV "));
      Serial.print(s.svid);
      Serial.print(F("  C/N0="));
      Serial.print(s.cno_dbhz);
      Serial.print(F(" dB-Hz  elev="));
      Serial.print(s.elev_deg);
      Serial.print(F(" deg  az="));
      Serial.print(s.azim_deg);
      Serial.print(F(" deg  used="));
      Serial.println(s.usedInSolution() ? F("Y") : F("N"));
    }
  }
}
