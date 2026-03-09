/**
 * @file Casic.h
 * @author Colin O'Flynn, Jotham Gates
 * @brief Small Arduino library for parsing the CASIC GNSS binary protocol.
 * @version 0.0.0
 * @date 2026-02-15
 *
 * Created by Colin O'Flynn, 2026-02-15
 * Modified by Jotham Gates, 2026-03-09
 */
#pragma once
#include <Arduino.h>
#include "CasicMsgPayloads.h"

enum class CasicCfgResult : uint8_t
{
    NONE = 0,
    PENDING,
    ACK,
    NAK,
    TIMEOUT,
};

typedef void (*NavPvCallback)(CasicMsgPayloads::NavPv &);
typedef void (*NavTimeUtcCallback)(CasicMsgPayloads::NavTimeUTC &);
typedef void (*NavInfoCallback)(CasicMsgPayloads::NavSolutionInfo &);

// RXM-SVPOS messages are even bigger, but we probably don't wish to reserve 1552 bytes just in case.
#define MAX_PAYLOAD_SIZE max(max(sizeof(CasicMsgPayloads::NavPv), sizeof(CasicMsgPayloads::NavSolutionInfo)), sizeof(CasicMsgPayloads::NavTimeUTC))
#define CASIC_HEADER0 0xba
#define CASIC_HEADER1 0xce
#define CASIC_RECEIVE_TIMEOUT 1000
#define CASIC_CFG_TIMEOUT 1000 // The timeout to wait for acknowledgements.
/**
 * @brief Class for a message.
 *
 */
class CasicMsg
{
public:
    /**
     * @brief Construct a new Casic Msg object
     *
     * @param payload buffer to use for the message payload.
     * @param payloadLength how long the buffer is.
     */
    CasicMsg(char *const payload, const uint16_t payloadLength) : payload(payload),
                                                                  payloadLength(payloadLength) {}
    void send(Stream &serial);

    uint16_t length;
    uint8_t cls;
    uint8_t id;
    char *const payload;
    const uint16_t payloadLength;

    /**
     * @brief Calcultes the checksum for a message.
     *
     * @return uint32_t the calculated checksum.
     */
    uint32_t checksum();

    /**
     * @brief Checks if the received checksum matches the valid one.
     *
     * @param received
     * @return true
     * @return false
     */
    bool checkChecksum(uint32_t received);
};

/**
 * @brief Enumerator that represents the current configuration state.
 * 
 */
enum class CfgState
{
    WAITING,
    IDLE,
    TIMED_OUT,
    NACK_RECEIVED
};

/**
 * @brief Class that implements the CASIC protocol for talking to a GNSS module.
 *
 */
class Casic
{
public:
    Casic(
        Stream &serial,
        NavPvCallback navPvCallback = nullptr,
        NavTimeUtcCallback navTimeUtcCallback = nullptr,
        NavInfoCallback navInfoCallback = nullptr) : m_ser(serial),
                                                     m_navPvCallback(navPvCallback),
                                                     m_navTimeUtcCallback(navTimeUtcCallback),
                                                     m_navInfoCallback(navInfoCallback),
                                                     m_msg(m_msgPayload, sizeof(m_msgPayload)) {}

    /**
     * @brief Reads the latest data from the GPS and processes it.
     * 
     */
    void update();

    /**
     * @brief Checks how configuration is proceeding.
     * This also checks if a timeout has occurred or other error.
     * @return CfgState the latest state.
     */
    CfgState getCfgState();

    /**
     * @brief Configures the serial port(s) of the GNSS module.
     * This also enables and disables NMEA and binary inputs and outputs.
     * 
     * @param cfg the configuration to send. This includes the parameter that specifies which port to apply it to.
     * @return true if the message was sent.
     * @return false if another message is already waiting to be sent.
     */
    bool cfgPrt(CasicMsgPayloads::CfgPrt &cfg);

    /**
     * @brief Configures the rate of each message type being sent.
     * 
     * @param cfg the message settings.
     * @return true if the message was sent.
     * @return false if another message is already waiting to be sent.
     */
    bool cfgMsg(CasicMsgPayloads::CfgMsg &cfg);

    /**
     * @brief Configures the rate of position updates.
     * 
     * @param cfg the message settings.
     * @return true if the message was sent.
     * @return false if another message is already waiting to be sent.
     */
    bool cfgRate(CasicMsgPayloads::CfgRate &cfg);

    /**
     * @brief Convenience function that waits until a config message is acknowledged (or fails).
     * 
     */
    void waitForCfg();

    // TODO: Query config messages.
private:
    /**
     * @brief Processes a single byte from the module.
     *
     * @param value the incoming value.
     */
    inline void m_processByte(char value);

    /**
     * @brief Checks the checksum and if it matches (and the message type is
     * recognised), calls a callback.
     *
     */
    inline void m_checkHandleMsg();

    /**
     * @brief Sends a config message and sets the flags to say we are waiting.
     * 
     * @param cfg the message to send.
     * @return true if the message was sent successfully and we are now waiting for acknowledgement.
     * @return false if we were waiting for another message already.
     */
    bool m_sendCfg(CasicMsg &cfg);
    
    /**
     * @brief Handles an acknowledgement or negative acknowledgement message.
     * 
     * @param isAck true when the message is an ack, false otherwise.
     * @param msg the message that was received.
     */
    inline void m_handleAckNack(bool isAck, CasicMsgPayloads::Ack &msg);

    // Serial and callbacks.
    Stream &m_ser;
    NavPvCallback m_navPvCallback;
    NavTimeUtcCallback m_navTimeUtcCallback;
    NavInfoCallback m_navInfoCallback;

    /**
     * @brief The state of reading each packet.
     *
     */
    enum ReceiveState
    {
        IDLE = 0,
        SYNC2 = 1,
        LEN1 = 2,
        LEN2 = 3,
        CLASS = 4,
        ID = 5,
        PAYLOAD = 6,
        CHECKSUM = 7
    } m_receiveState;
    char m_msgPayload[MAX_PAYLOAD_SIZE];
    CasicMsg m_msg;
    uint16_t m_receivedBytes;
    uint32_t m_receiveChecksum;
    uint32_t m_receiveStartTime;

    // Config stuff.
    CfgState m_cfgState = CfgState::IDLE; // Whether we need an acknowledgement or it has timed out.
    uint8_t m_cfgCls; // The message class we are waiting for an acknowledgement for.
    uint8_t m_cfgId; // The message ID we are waiting for an acknowledgement for.
    uint32_t m_cfgStartTime; // When we started sending a message.
};