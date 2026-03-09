#include <Casic.h>

Casic gps;

/*
  This example:
  1) Queries current port config (CFG-PRT)
  2) Sets port config to enable binary output and disable NMEA (text) output
  3) Sets navigation rate with CFG-RATE (e.g., 200 ms = 5 Hz)
  4) Enables NAV-PV (speed3D/alt/accuracy) and NAV-GPSINFO (sat list)
  5) (Optional) Save to NVM

  NOTE: If you change baud rate via cfgSetPort, you must also change GNSS_SERIAL_PORT.begin()
  after you get ACK for the port change. - need to check the ack, doesn't appear to come from
  the module prior to it changing baud.
*/

enum Step
{
  S_QUERY_PRT,
  S_WAIT_PRT,
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

#define DEBUG_SERIAL_PORT Serial // The serial port to print status updates to.
#define GNSS_SERIAL_PORT Serial2 // The serial port that the GNSS module is connected to.
#define GNSS_MODULE_PORT 0       // The serial port on the module (0 or 1 usually).

#define GNSS_BAUD_INITIAL 9600 // The initial baud rate that the GNSS module defaults to.
#define CHANGE_GNSS_BAUD
#ifdef CHANGE_GNSS_BAUD
#define GNSS_BAUD_UPDATED 115200 // The baud rate to switch to after initial configuration.
#endif
#define PIN_GNSS_TX 16 // The pin configured for serial out to the module if this can be changed from the defaults (like esp32-based boards).
#define PIN_GNSS_RX 17 // The pin configured for serial in from the module if this can be changed from the defaults (like esp32-based boards).

void setup()
{
  DEBUG_SERIAL_PORT.begin(115200);
  while (!DEBUG_SERIAL_PORT)
  {
    delay(10);
  }

  GNSS_SERIAL_PORT.begin(GNSS_BAUD_INITIAL, SERIAL_8N1, PIN_GNSS_TX, PIN_GNSS_RX);

  DEBUG_SERIAL_PORT.println(F("Disable NMEA + Faster updates (CASIC binary)"));
  DEBUG_SERIAL_PORT.println(F("Starting config..."));
}

void loop()
{
  gps.processStream(GNSS_SERIAL_PORT);
  gps.serviceConfig();

  CasicCfgResult r = gps.peekCfgResult();

  switch (stepState)
  {
  case S_QUERY_PRT:
    // if (gps.cfgQueryPort(GNSS_SERIAL_PORT, 255)) { // port_id=1 is common for UART1; adjust if needed
    //   stepState = S_WAIT_PRT;
    // } else {
    //   DEBUG_SERIAL_PORT.println("ERROR: A configuration message is already being sent when querying the port.");
    // }
    stepState = S_WAIT_PRT;
    break;

  case S_WAIT_PRT:
  {
    // if (gps.portConfigUpdated()) {
    //   const CasicPortConfig& pc = gps.portConfig();
    //   DEBUG_SERIAL_PORT.print(F("Port ")); DEBUG_SERIAL_PORT.print(pc.port_id);
    //   DEBUG_SERIAL_PORT.print(F(" proto_mask=0x")); DEBUG_SERIAL_PORT.print(pc.proto_mask, HEX);
    //   DEBUG_SERIAL_PORT.print(F(" mode=0x")); DEBUG_SERIAL_PORT.print(pc.mode, HEX);
    //   DEBUG_SERIAL_PORT.print(F(" baud=")); DEBUG_SERIAL_PORT.println(pc.baud_rate);

    //   // Build new mask: enable binary out (0x10), disable text out (0x20)
    //   uint8_t newMask = pc.proto_mask | 0x10;
    //   newMask = (uint8_t)(newMask & ~0x20);

#ifdef CHANGE_GNSS_BAUD
    // Request that the module swap to a new baud rate.
    // bool success = gps.cfgSetPort(GNSS_SERIAL_PORT, pc.port_id, newMask, pc.mode, GNSS_BAUD_UPDATED);
    bool success = gps.cfgSetPort(GNSS_SERIAL_PORT, GNSS_MODULE_PORT, 0b00110011, 0x08c0, GNSS_BAUD_UPDATED);
#else
    // Keep mode and baud the same here (fast + safe)
    bool success = gps.cfgSetPort(GNSS_SERIAL_PORT, pc.port_id, newMask, pc.mode, pc.baud_rate);
#endif
    if (success)
    {
      DEBUG_SERIAL_PORT.println(F("Port update sent."));
      stepState = S_WAIT_SET_PRT;
#ifdef CHANGE_GNSS_BAUD
      // Finish sending and change on the microcontroller end.
      GNSS_SERIAL_PORT.flush();
      GNSS_SERIAL_PORT.end();
      GNSS_SERIAL_PORT.begin(GNSS_BAUD_UPDATED, SERIAL_8N1, PIN_GNSS_TX, PIN_GNSS_RX);
      // NOTE that ESP32 (and probably other boards) can use the following instead of stopping and restarting serial.
      // GNSS_SERIAL_PORT.updateBaudRate(GNSS_BAUD_UPDATED);
#endif
    }
    else
    {
      // Something has gone wrong. Go back to the start and attempt to recover.
      DEBUG_SERIAL_PORT.println(F("ERROR: A configuration message is already being sent when setting the port."));
      delay(100);
      stepState = S_QUERY_PRT;
    }
    // } else if (r == CasicCfgResult::Nak) {
    //   DEBUG_SERIAL_PORT.println("ERROR: Received NACK when querying port.");
    // } else if (r == CasicCfgResult::Timeout) {
    //   DEBUG_SERIAL_PORT.println("ERROR: Timed out when querying port.");
    // }
  }
  break;

  case S_WAIT_SET_PRT:
    if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout)
    {
      DEBUG_SERIAL_PORT.print(F("CFG-PRT result: "));
      DEBUG_SERIAL_PORT.println((int)gps.readCfgResult());
      stepState = S_SET_RATE;
    }
    break;

  case S_SET_RATE:
    // 100 ms = 10 Hz navigation solution.
    // Note that some modules are limited to 5Hz (200ms) updates
    // while others can provide 10Hz updates.
    if (gps.cfgSetRate(GNSS_SERIAL_PORT, 200))
    {
      DEBUG_SERIAL_PORT.println(F("Rate update sent."));
      stepState = S_WAIT_RATE;
    }
    break;

  case S_WAIT_RATE:
    if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout)
    {
      DEBUG_SERIAL_PORT.print(F("CFG-RATE result: "));
      DEBUG_SERIAL_PORT.println((int)gps.readCfgResult());
      stepState = S_ENABLE_NAVPV;
    }
    break;

  case S_ENABLE_NAVPV:
    if (gps.cfgSetMsgRate(GNSS_SERIAL_PORT, 0x01, 0x03, 1))
    { // NAV-PV
      stepState = S_WAIT_NAVPV;
    }
    break;

  case S_WAIT_NAVPV:
    if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout)
    {
      DEBUG_SERIAL_PORT.print(F("NAV-PV enable result: "));
      DEBUG_SERIAL_PORT.println((int)gps.readCfgResult());
      stepState = S_ENABLE_GPSINFO;
    }
    break;

  case S_ENABLE_GPSINFO:
    if (gps.cfgSetMsgRate(GNSS_SERIAL_PORT, 0x01, 0x20, 1))
    { // NAV-GPSINFO
      stepState = S_WAIT_GPSINFO;
    }
    break;

  case S_WAIT_GPSINFO:
    if (r == CasicCfgResult::Ack || r == CasicCfgResult::Nak || r == CasicCfgResult::Timeout)
    {
      DEBUG_SERIAL_PORT.print(F("NAV-GPSINFO enable result: "));
      DEBUG_SERIAL_PORT.println((int)gps.readCfgResult());
      stepState = S_DONE;

      DEBUG_SERIAL_PORT.println(F("Config done (not saved). To persist, call gps.cfgSave(GNSS_SERIAL_PORT) now."));
    }
    break;

  case S_DONE:
    // Print a couple of live fields as confirmation
    if (gps.navPvUpdated())
    {
      const CasicNavPv &pv = gps.navPv();
      DEBUG_SERIAL_PORT.print(F("speed3D m/s="));
      DEBUG_SERIAL_PORT.print(pv.speed3D_mps, 2);
      DEBUG_SERIAL_PORT.print(F(" altMSL m="));
      DEBUG_SERIAL_PORT.print(pv.altitude_msl_m(), 2);
      DEBUG_SERIAL_PORT.print(F(" hAcc1s m="));
      DEBUG_SERIAL_PORT.print(pv.hAcc_1sigma_m(), 2);
      DEBUG_SERIAL_PORT.print(F(" sAcc1s m/s="));
      DEBUG_SERIAL_PORT.println(pv.speedAcc_1sigma_mps(), 2);
    }
    break;
  }
}
