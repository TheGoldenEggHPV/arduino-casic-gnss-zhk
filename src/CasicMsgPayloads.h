/**
 * @file CasicMsgPayloads.h
 * @author Jotham Gates
 * @brief Structures that break out the data for each message type.
 * @version 0.0.0
 * @date 2026-03-09
 *
 * Created by Jotham Gates, 2026-03-09
 */
#pragma once
#include "cstdint"

/**
 * @brief Structures that contain the message payload formats.
 *
 */
namespace CasicMsgPayloads
{
    /**
     * @brief Payload for an acknowledgement (ack-ack or ack-nack).
     * Ack: 0x05 0x01
     * Nack: Ox05 0x00
     */
    struct __attribute__((__packed__)) Ack
    {
        uint8_t classId;
        uint8_t messageId;
        uint16_t reserved;
    };

    /**
     * @brief Payload for current position and velocity information.
     * 0x01 0x03
     */
    struct __attribute__((__packed__)) NavPv
    {
        uint32_t runTime; // Module runtime from power-on/reset (ms).

        /**
         * @brief Enumerator for positioning valid in NAV-PV
         *
         */
        enum class PositioningValid : uint8_t
        {
            INVALID = 0,
            EXTERNAL_INPUT = 1,
            ROUGH_ESTIMATE = 2,
            MAINTAIN_LAST_POS = 3,
            PROJECTED_FLIGHT_LEVELS = 4,
            FAST_MODE_POS = 5,
            TWOD_POS = 6,
            THREED_POS = 7,
            GNSS_DR_COMBINED = 8
        } posValid;

        /**
         * @brief Enumerator for velocity valid in NAV-PV
         *
         */
        enum class VelocityValid : uint8_t
        {
            SPEED_NULLIFICATION = 0,
            EXTERNAL_INPUT = 1,
            ROUGH_ESTIMATE = 2,
            MAINTAIN_LAST_SPEED = 3,
            PROJECT_SPEED = 4,
            FAST_MODE_SPEED = 5,
            TWOD_SPEED = 6,
            THREED_SPEED = 7
        } velValid;

        /**
         * @brief Shows which GNSS systems are being used.
         *
         */
        struct GNSSSystem
        {
            bool gps : 1;
            bool bds : 1;
            bool glonass : 1;
            int reserved : 5;
        } system;

        uint8_t numSV;      // The number of satellites involved in the solution.
        uint8_t numSVGPS;   // The number of GPS satellites involved in the solution.
        uint8_t numSVBDS;   // The number of BDS satellites involved in the solution.
        uint8_t numSVGLN;   // The number of GLONASS satellites involved in the solution.
        uint8_t m_reserved; // Reserved byte.
        float pDop;         // Location DOP
        double lon;         // The longitude in degrees.
        double lat;         // The latitude in degrees.
        float height;       // Geodesic height (ellipsoid as reference) in m.
        float sepGeoid;     // The height anomaly (difference between geodetic height and elevation) in m.
        float hAcc;         // Variance of horizontal position accuracy error in m^2.
        float vAcc;         // Variance of vertical position accuracy error in m^2.
        float velN;         // Northward velocity in the ENU coordinate system in m/s.
        float velE;         // Eastward velocity in the ENU coordinate system in m/s.
        float velU;         // Celestial velocity in the ENU coordinate system in m/s.
        float speed3D;      // 3D speed in m/s.
        float speed2D;      // 2D speed in m/s.
        float heading;      // The heading in degrees.
        float sAcc;         // The variance of the accuracy error for the ground speed in (m/s)^2.
        float cAcc;         // The variance of the accuracy error of the heading in degrees^2.
    };

    /**
     * @brief Payload for UTC time information.
     * 0x01 0x10
     */
    struct __attribute__((__packed__)) NavTimeUTC
    {
        uint32_t runTime; // Module runtime from power-on/reset (ms).
        float tAcc;       // Time estimation accuracy. Multiply by 1/(c^2) to get the time in s^2 (c is the speed of light in m/s).
        float msErr;      // Residual error after ms rounding.
        uint16_t ms;      // Miliseconds part of UTC time (0-999).
        uint16_t year;    // UTC years (1999-2099).
        uint8_t month;    // UTC month (1-12).
        uint8_t day;      // UTC day of the month (1-32).
        uint8_t hour;     // UTC hour of the day (0-23).
        uint8_t min;      // UTC minutes in hours (0-59).
        uint8_t sec;      // Seconds in the minute (0-59).

        /**
         * @brief Describes what is and isn't valid.
         *
         */
        union TimeValid
        {
            uint8_t data;
            struct Bits
            {
                bool validDuringWeek : 1;
                bool validWeekly : 1;
                bool validLeadSecondCorrection : 1;
            };
        } valid;

        /**
         * @brief Enum that specifies where the time is coming from.
         * Some extra fields are described here:
         * https://github.com/jclark/casictool/blob/main/spec/notes.md#cfg-tp-timesource-field
         */
        enum class TimeSource : uint8_t
        {
            GPS = 0,
            BDS = 1,
            GLONASS = 2,
            PREFER_BDS = 4,    // Extra, prefer BDS, fall back if needed.
            PREFER_GPS = 5,    // Extra, prefer GPS, fall back if needed.
            PREFER_GLONASS = 6 // Extra, preger GLONASS, fall back if needed.
        } timeSrc;

        /**
         * @brief Represents whether the date and time are valid.
         *
         */
        enum class DateValid : uint8_t
        {
            INVALID = 0,
            EXTERNAL_INPUT = 1,
            SATELLITE = 2,
            RELIABLE_SATELLITES = 3

        } dateValid;
    };

    /**
     * @brief Payload for solution info
     * - NAV-GPSINFO: 0x01 0x20
     * - NAV-BDSINFO: 0x01 0x21
     * - NAV-GLNINFO: 0x01 0x22
     *
     */
    struct __attribute__((__packed__)) NavSolutionInfo
    {
        uint32_t runTime = 0;  // Module runtime from power-on/reset (ms).
        uint8_t numViewSv = 0; // The number of visible satellites (0-32).
        uint8_t numFixSv = 0;  // The number of satellites used for positioning.

        /**
         * @brief The type of the system this message is for.
         *
         */
        enum class SystemType : uint8_t
        {
            GPS = 0,
            BDS = 1,
            GLONASS = 2
        } systemType;

        uint8_t reserved; // Padding, reserved.

        /**
         * @brief Enumerator that describes each satellite.
         * This repeats numViewSv times, (0-32).
         *
         */
        struct CasicSat
        {
            uint8_t chn;  // The channel number.
            uint8_t svid; // The satellite number.

            /**
             * @brief Shows which GNSS systems are being used.
             *
             */
            union Flags
            {
                uint8_t data;
                struct Bits
                {
                    bool usedInSolution : 1;        // Bit 0. This satellite is used when 1.
                    int reserved : 3;               // Bits 1-3 inclusive.
                    bool invalidPredictionInfo : 1; // Bit 4, invalid satellite prediction information when 1.
                    bool reserved1 : 1;             // Bit 5.
                    enum class SatellitePrediction
                    {
                        RESERVED = 0,
                        ALMANACS = 1, // Prediction based on almanacs.
                        RESERVED1 = 2,
                        EPHEMERIS = 3 // Prediction based on ephemeris.
                    } prediction : 2;
                };
            } system;

            union Quality
            {
                uint8_t data;
                struct Bits
                {
                    bool pseudoDistanceMeasValid : 1;       // = 1 for pseudo-distance measurements prMes valid
                    bool carrierPhaseMeasValid : 1;         // = 1 for carrier phase measurements cpMes valid
                    bool halfPerimeterAmbiguityValid : 1;   // = 1, indicating that half-perimeter ambiguity is valid (inverse PI correction is valid)
                    bool halfPeriodAmbiguitySubtracted : 1; // = 1, indicating that the half-period ambiguity is subtracted from the carrier phase measurement
                    bool reserved : 1;
                    bool carrierFreqValid : 1; // = 1, indicates that the carrier frequency is valid
                    int reserved1 : 2;
                };
            } quality;

            uint8_t cn0;  // Signal to noise ratio in dB-Hz.
            int8_t elev;  // The elevation in degrees of the satellite (-90-90 degrees).
            int16_t azim; // Satellite aximuth (0-360 degrees).
            float prRes;  // Pseudorange residual in m.
        } sats[32];
    };

    /**
     * @brief Port configuration.
     * 0x06 0x00
     *
     */
    struct __attribute__((__packed__)) CfgPrt
    {
        /**
         * @brief The identifier of the port this message relates to.
         *
         */
        enum class PortId : uint8_t
        {
            UART0 = 0,
            UART1 = 1,
            CURRENT_PORT = 0xff
        } portId;

        /**
         * @brief The protocol mask that descibes which protocols are in use (NMEA and or CASIC).
         *
         */
        union ProtocolMask
        {
            uint8_t data;
            struct Bits
            {
                bool binaryInput : 1;  // Binary protocol input (bit 0).
                bool textInput : 1;    // Text protocol input (bit 1).
                int reserved : 2;      // Reserved.
                bool binaryOutput : 1; // Binary protocol output (bit 4).
                bool textOutput : 1;   // Text protocol output (bit 5).
            };
        } protoMask;

        union UartMode
        {
            uint16_t data;
            struct Bits
            {
                int reserved : 6;
                /**
                 * @brief The number of bits per word to send at a time.
                 * 8 is usually the default / safe option nowadays.
                 * Bits [7:6]
                 */
                enum class BitsPerChar
                {
                    BITS_5 = 0,
                    BITS_6 = 1,
                    BITS_7 = 2,
                    BITS_8 = 3
                } bitsPerChar : 2;

                enum class Parity
                {
                    NONE = 0b100,
                    NONE1 = 0b101,
                    ODD = 0x001,
                    EVEN = 0b000,
                    // Everything else is reserved.
                } parity : 3;

                enum class StopBits
                {
                    ONE = 0,
                    ONE_POINT_FIVE = 1,
                    TWO = 2,
                    RESERVED = 3
                } stopBits : 2;
            };
        } mode;

        uint32_t baudRate; // The baud rate in bits / second.
    };

    /**
     * @brief Sets / queries the message sending frequency.
     * 
     */
    struct __attribute__((__packed__)) CfgMsg
    {
        uint8_t classId;
        uint8_t messageId;
        uint16_t rate;

        /**
         * @brief If rate is one of these values there is a special meaning.
         * If not, then the message is output once every <rate> positions.
         * 
         */
        enum Rate : uint16_t
        {
            NO_OUTPUT = 0,
            QUERY_ONLY = 0xffff
        };
    };

    /**
     * @brief Sets / queries the positioning interval.
     * 
     */
    struct __attribute__((__packed__)) CfgRate
    {
        /**
         * @brief The interval in ms.
         * Typically this includes 1000 (1Hz), 500 (2Hz), 200 (5Hz) and sometimes 100 (10Hz).
         * 
         */
        uint16_t interval;
        uint16_t reserved;
    };
};

/**
 * @brief Message classes and IDs.
 * 
 */
#define CASIC_ACK_CLASS 0x05
#define CASIC_ACK_ID 0x01
#define CASIC_NACK_ID 0x00
#define CASIC_NAV_CLASS 0x01
#define CASIC_NAV_PV_ID 0x03
#define CASIC_NAV_TIMEUTC_ID 0x10
#define CASIC_NAV_GPSINFO_ID 0x20
#define CASIC_NAV_BDSINFO_ID 0x21
#define CASIC_NAV_GLNINFO_ID 0x22
#define CASIC_CFG_CLASS 0x06
#define CASIC_CFG_PRT_ID 0x00
#define CASIC_CFG_MSG_ID 0x01
#define CASIC_CFG_RATE_ID 0x04