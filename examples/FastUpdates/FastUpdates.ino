/**
 * @file FastUpdates.ino
 * @author Jotham Gates (koyugaDev@gmail.com)
 * @brief Example that communicates with a GNSS module using the CASIC protocol with callbacks.
 * @version 0.0.0
 * @date 2026-03-10
 * 
 */
#include <Casic.h>

// GNSS serial settings
#define SERIAL_GNSS Serial2
#define SERIAL_GNSS_CUSTOM_PINS // Comment this out if your board doesn't support customising serial pins or you wish to use the defaults.
#ifdef SERIAL_GNSS_CUSTOM_PINS
#define PIN_GNSS_TX 16 // The pin configured for serial out to the module if this can be changed from the defaults (like esp32-based boards).
#define PIN_GNSS_RX 17 // The pin configured for serial in from the module if this can be changed from the defaults (like esp32-based boards).
#endif
#define GNSS_BAUD_INITIAL 9600   // The default is usually 9600 baud.
#define GNSS_BAUD_UPDATED 115200 // The baud rate to use for normal operations.

#define SERIAL_DEBUG Serial

#define PIN_LED 13

// Function prototypes so that we can provide them in the gps constructor.
void onNavPv(CasicMsgPayloads::NavPv &msg);
void onNavTimeUTC(CasicMsgPayloads::NavTimeUTC &msg);

Casic gnss(SERIAL_GNSS, onNavPv, onNavTimeUTC);

/**
 * @brief Sets up the demonstration.
 * 
 */
void setup()
{
    // Initialise the serial port used for debugging.
    SERIAL_DEBUG.begin(115200);
    SERIAL_DEBUG.println(F("Casic demonstration. Compiled on " __DATE__ " at " __TIME__ "."));
    SERIAL_DEBUG.println(F("See https://github.com/colinoflynn/arduino-casic-gnss-zhk/"));
    pinMode(PIN_LED, OUTPUT);

#ifdef SERIAL_GNSS_CUSTOM_PINS
    // Initialise with custom pins (on boards which support this).
    SERIAL_GNSS.begin(GNSS_BAUD_INITIAL, SERIAL_8N1, PIN_GNSS_TX, PIN_GNSS_RX);
#else
    // Don't customise the pins (other boards).
    SERIAL_GNSS.begin(GNSS_BAUD_INITIAL);
#endif
    delay(500); // Extra time to allow the module to start.
    setupGNSS();
}

/**
 * @brief Repeated cyclically.
 * 
 */
void loop()
{
    gnss.update();
}

/**
 * @brief Prints the result of the last configurration.
 * 
 * @param msgName the name of the operation.
 */
void waitAndPrintResult(const char *msgName)
{
    digitalWrite(PIN_LED, HIGH);
    gnss.waitForCfg();
    CfgState result = gnss.getCfgState();
    switch (result)
    {
        case CfgState::IDLE:
            SERIAL_DEBUG.print(F("Successfully set "));
            break;
        
        case CfgState::TIMED_OUT:
            SERIAL_DEBUG.print(F("Timed out when setting "));
            break;

        case CfgState::NACK_RECEIVED:
            SERIAL_DEBUG.print(F("NACK recieved when setting "));
            break;
        
        default:
            SERIAL_DEBUG.print(F("Unkown result ("));
            SERIAL_DEBUG.print(static_cast<int>(result));
            SERIAL_DEBUG.print(F(") when setting "));
    }
    SERIAL_DEBUG.print(msgName);
    SERIAL_DEBUG.println(F("."));
    digitalWrite(PIN_LED, LOW);
}

/**
 * @brief Sets up the GNSS module to use the CASIC protocol at high data rates and high baud rate.
 * 
 */
void setupGNSS()
{
    // Set the port settings. Swap back to the initial baud rate just in case.
    SERIAL_GNSS.flush();
    SERIAL_GNSS.updateBaudRate(GNSS_BAUD_INITIAL);

    CasicMsgPayloads::CfgPrt cfgPort;
    cfgPort.portId = CasicMsgPayloads::CfgPrt::PortId::UART0;
    cfgPort.mode.bits.bitsPerChar = CasicMsgPayloads::CfgPrt::UartMode::Bits::BitsPerChar::BITS_8;
    cfgPort.mode.bits.stopBits = CasicMsgPayloads::CfgPrt::UartMode::Bits::StopBits::ONE;
    cfgPort.mode.bits.parity = CasicMsgPayloads::CfgPrt::UartMode::Bits::Parity::NONE;
    cfgPort.protoMask.bits.binaryInput = true;
    cfgPort.protoMask.bits.textInput = true;
    cfgPort.protoMask.bits.binaryOutput = true;
    cfgPort.protoMask.bits.textOutput = false;
    cfgPort.baudRate = GNSS_BAUD_UPDATED;
    gnss.cfgPrt(cfgPort);
    waitAndPrintResult("CFG-PRT");

    SERIAL_GNSS.flush();
    SERIAL_GNSS.updateBaudRate(GNSS_BAUD_UPDATED);

    // Set the message types to enable NAV-PV (position and speed) messages be sent every fix.
    CasicMsgPayloads::CfgMsg cfgMsg;
    cfgMsg.classId = CASIC_NAV_CLASS;
    cfgMsg.messageId = CASIC_NAV_PV_ID;
    cfgMsg.rate = 1;
    gnss.cfgMsg(cfgMsg);
    waitAndPrintResult("CFG-MSG (NAV-PV)");

    // Set the message types to enable NAV-TIMEUTC (date and time) messages be sent every 10 fixes.
    cfgMsg.classId = CASIC_NAV_CLASS;
    cfgMsg.messageId = CASIC_NAV_TIMEUTC_ID;
    cfgMsg.rate = 10;
    gnss.cfgMsg(cfgMsg);
    waitAndPrintResult("CFG-MSG (NAV-TIMEUTC)");

    // Set the update rate to 5Hz (200ms interval).
    CasicMsgPayloads::CfgRate cfgRate;
    cfgRate.interval = 200;
    gnss.cfgRate(cfgRate);
    waitAndPrintResult("CFG-RATE");

}

void onNavPv(CasicMsgPayloads::NavPv &msg)
{
    digitalWrite(PIN_LED, HIGH);
    // Header and run time
    SERIAL_DEBUG.print(F("NAV-PV:\r\n  - Run time:\t"));
    SERIAL_DEBUG.println(msg.runTime);

    // Validity
    SERIAL_DEBUG.print(F("  - Pos valid:\t"));
    switch (msg.posValid)
    {
        case CasicMsgPayloads::NavPv::PositioningValid::INVALID:
            SERIAL_DEBUG.println(F("INVALID"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::EXTERNAL_INPUT:
            SERIAL_DEBUG.println(F("EXTERNAL INPUT"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::ROUGH_ESTIMATE:
            SERIAL_DEBUG.println(F("ROUGH ESTIMATE"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::MAINTAIN_LAST_POS:
            SERIAL_DEBUG.println(F("MAINTAIN LAST POS"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::PROJECTED_FLIGHT_LEVELS:
            SERIAL_DEBUG.println(F("PROJECTED FLIGHT LEVELS"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::FAST_MODE_POS:
            SERIAL_DEBUG.println(F("FAST MODE"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::TWOD_POS:
            SERIAL_DEBUG.println(F("2D POS"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::THREED_POS:
            SERIAL_DEBUG.println(F("3D POS"));
            break;
        case CasicMsgPayloads::NavPv::PositioningValid::GNSS_DR_COMBINED:
            SERIAL_DEBUG.println(F("GNSS+DR"));
            break;
    }

    // Numbers of satellites
    SERIAL_DEBUG.print(F("  - # Sats:\t\t"));
    SERIAL_DEBUG.print(msg.numSV);
    SERIAL_DEBUG.print(F(" ("));
    SERIAL_DEBUG.print(msg.numSVBDS);
    SERIAL_DEBUG.print(F("BDS, "));
    SERIAL_DEBUG.print(msg.numSVGPS);
    SERIAL_DEBUG.print(F("GPS, "));
    SERIAL_DEBUG.print(msg.numSVGLN);
    SERIAL_DEBUG.println((F("GLN)")));

    // Latitude and longitude
    SERIAL_DEBUG.print(F("  - Lat:\t\t"));
    SERIAL_DEBUG.print(msg.lat);
    SERIAL_DEBUG.print(F("deg\r\n  - Lon:\t\t"));
    SERIAL_DEBUG.print(msg.lon);
    SERIAL_DEBUG.println(F("deg"));

    // Velocity.
    SERIAL_DEBUG.print(F("  - Speed:\t"));
    SERIAL_DEBUG.print(msg.speed3D);
    SERIAL_DEBUG.println(F("m/s"));
    digitalWrite(PIN_LED, LOW);
}

void onNavTimeUTC(CasicMsgPayloads::NavTimeUTC &msg)
{
    SERIAL_DEBUG.print("NAV-TIMEUTC\r\n  - Run time:\t");
    SERIAL_DEBUG.println(msg.runTime);

    // Valid:
    SERIAL_DEBUG.print(F("  - Valid:\t\t"));
    SERIAL_DEBUG.println(static_cast<int>(msg.dateValid));

    // Date
    SERIAL_DEBUG.print(F("  - Date:\t\t"));
    SERIAL_DEBUG.print(msg.year);
    SERIAL_DEBUG.print(F("-"));
    SERIAL_DEBUG.print(msg.month);
    SERIAL_DEBUG.print(F("-"));
    SERIAL_DEBUG.println(msg.day);

    // Time
    SERIAL_DEBUG.print(F("  - Time:\t\t"));
    SERIAL_DEBUG.print(msg.hour);
    SERIAL_DEBUG.print(F(":"));
    SERIAL_DEBUG.print(msg.min);
    SERIAL_DEBUG.print(F(":"));
    SERIAL_DEBUG.print(msg.sec);
    SERIAL_DEBUG.print(F(":"));
    SERIAL_DEBUG.print(msg.ms);
    SERIAL_DEBUG.print(F(":"));
    SERIAL_DEBUG.println(msg.msErr);
}