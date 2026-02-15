# ArduinoCasic

Small Arduino library for parsing the CASIC GNSS binary protocol.

Supported messages:
- NAV-PV (class 0x01, id 0x03): position, velocity, `speed3D`, accuracy/variance fields
- NAV-GPSINFO / NAV-BDSINFO / NAV-GLNINFO (class 0x01, ids 0x20/0x21/0x22): satellite view/fix list

Mostly auto-generated from [https://github.com/jclark/casictool](https://github.com/jclark/casictool). casictool includes a .md description of the protocol. See references below for more.

## Usage

```cpp
#include <Casic.h>

Casic gps;

void loop() {
  while (Serial1.available()) {
    gps.processByte((uint8_t)Serial1.read());
  }

  if (gps.navPvUpdated()) {
    const CasicNavPv& pv = gps.navPv();
    Serial.println(pv.speed3D_mps);
  }
}
```

See examples for full printing of speed/altitude/accuracy and satellite status.


## Configuration support

The library can send CFG messages (CFG-MSG, CFG-PRT, CFG-CFG) and parse ACK-ACK/ACK-NACK. See `examples/VelocityAndAccuracy` for enabling NAV-PV and NAV-GPSINFO at runtime.


### CFG-RATE

Use `cfgSetRate(interval_ms)` to change the navigation solution interval (e.g., 100 ms for 10 Hz).


### Example: Disable NMEA for faster updates

See `examples/DisableNMEA_FastUpdates` for a scripted configuration sequence that disables text (NMEA) output and enables binary messages.


## Blocking configuration helpers

If you prefer not to write a configuration state machine, you can use `waitForCfgResult()` and the `*AndWait()` helpers. These helpers keep parsing bytes while waiting for the ACK/NAK.

Example:

```cpp
CasicCfgResult r;
gps.cfgSetRateAndWait(Serial1, 100, 600, &r);
gps.cfgSetMsgRateAndWait(Serial1, 0x01, 0x03, 1, 400, &r);
```

See `examples/FastBinaryOnly`.

## References

- [https://www.espruino.com/files/CASIC_en.pdf](https://www.espruino.com/files/CASIC_en.pdf)
- [https://github.com/jclark/casictool](https://github.com/jclark/casictool)