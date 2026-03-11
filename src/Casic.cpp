/**
 * @file Casic.cpp
 * @author Colin O'Flynn, Jotham Gates
 * @brief Small Arduino library for parsing the CASIC GNSS binary protocol.
 * @version 0.0.0
 * @date 2026-02-15
 *
 * Created by Colin O'Flynn, 2026-02-15
 * Modified by Jotham Gates, 2026-03-09
 */
#include "Casic.h"
#include <Arduino.h>
void CasicMsg::send(Stream &serial)
{
    serial.write(CASIC_HEADER0);
    serial.write(CASIC_HEADER1);
    serial.write(reinterpret_cast<uint8_t *>(&length), 2);
    serial.write(cls);
    serial.write(id);
    serial.write(payload, length);
    uint32_t sum = checksum();
    serial.write(reinterpret_cast<uint8_t *>(&sum), 4);
}

uint32_t CasicMsg::checksum()
{
    uint32_t checksum = (id << 24) + (cls << 16) + length;
    if (length <= payloadLength)
    {
        for (int i = 0; i < length / 4; i++)
        {
            checksum += reinterpret_cast<uint32_t *>(payload)[i];
        }
        return checksum;
    }
    else
    {
        // The message is longer than the allocated memory.
        return 0;
    }
}

bool CasicMsg::checkChecksum(uint32_t received)
{
    return checksum() == received;
}

void Casic::update()
{
    int bytesAvailable = m_ser.available();
    for (int i = 0; i < bytesAvailable; i++)
    {
        m_processByte(m_ser.read());
    }
}

CfgState Casic::getCfgState()
{
    if (m_cfgState == CfgState::WAITING && micros() - m_cfgStartTime > CASIC_CFG_TIMEOUT)
    {
        // We have timed out.
        m_cfgState = CfgState::TIMED_OUT;
    }
    return m_cfgState;
}

void Casic::m_processByte(char value)
{
    // Receive timeout.
    uint32_t now = micros();
    if (m_receiveState != IDLE && (now - m_receiveStartTime) > CASIC_RECEIVE_TIMEOUT)
    {
        // Timed out. Give up reception.
        m_receiveState = IDLE;
    }

    // Handle the various states of packet reception.
    switch (m_receiveState)
    {
    case IDLE:
        if (value == CASIC_HEADER0)
        {
            m_receiveState = SYNC2;
            m_receiveStartTime = now;
        }
        break;

    case SYNC2:
        if (value == CASIC_HEADER1)
        {
            // Second sync byte successfully received.
            m_receiveState = LEN1;
        }
        else if (value != CASIC_HEADER0)
        {
            // Didn't have a repeated first sync byte. If this were the case, we
            // could get a valid second sync byte on the next cycle, so need to
            // stay on this state. This is not the case, so reset as the first
            // byte was spurious.
            m_receiveState = IDLE;
        }
        break;

    case LEN1:
        m_msg.length = value;
        m_receiveState = LEN2;
        break;

    case LEN2:
        m_msg.length |= value << 8;
        if (m_msg.length <= m_msg.payloadLength)
        {
            // Valid length.
            m_receiveState = CLASS;
        }
        else
        {
            // We can't handle a message this long. Give up.
            m_receiveState = IDLE;
        }
        break;

    case CLASS:
        m_msg.cls = value;
        m_receiveState = ID;
        break;

    case ID:
        m_msg.id = value;
        m_receiveState = PAYLOAD;
        m_receivedBytes = 0;
        break;

    case PAYLOAD:
        m_msg.payload[m_receivedBytes++] = value;
        if (m_receivedBytes == m_msg.length)
        {
            // We are done. Get the checksum.
            m_receiveState = CHECKSUM;
            m_receivedBytes = 0;
            m_receiveChecksum = 0;
        }
        break;

    case CHECKSUM:
        m_receiveChecksum |= value << (m_receivedBytes++ * 8);
        if (m_receivedBytes == 4)
        {
            // We are done. Check the checksum.
            m_checkHandleMsg();
            m_receiveState = IDLE;
        }
    }
}

/**
 * @brief Combines the class and id into a single 16 bit number to make
 * comparisons easier.
 *
 */
#define MERGE_CLASS_ID(CLS, ID) ((CLS) << 8 | ID)

void Casic::m_checkHandleMsg()
{
    if (m_msg.checkChecksum(m_receiveChecksum))
    {
        // The message is valid.
        switch (MERGE_CLASS_ID(m_msg.cls, m_msg.id))
        {
        case MERGE_CLASS_ID(CASIC_ACK_CLASS, CASIC_ACK_ID):
        case MERGE_CLASS_ID(CASIC_ACK_CLASS, CASIC_NACK_ID):
            // Ack. (ID=0x01) or Nack. (ID=0x00)
            if (m_msg.length == sizeof(CasicMsgPayloads::Ack))
            {
                // Callback is assigned and we have a valid message length.
                CasicMsgPayloads::Ack *structRef = reinterpret_cast<CasicMsgPayloads::Ack *>(m_msg.payload);
                m_handleAckNack(m_msg.id, *structRef);
            }
            break;

        case MERGE_CLASS_ID(CASIC_NAV_CLASS, CASIC_NAV_PV_ID):
            // NAV-PV
            if (m_msg.length == sizeof(CasicMsgPayloads::NavPv))
            {
                // Callback is assigned and we have a valid message length.
                CasicMsgPayloads::NavPv &structRef = *reinterpret_cast<CasicMsgPayloads::NavPv *>(m_msg.payload);
                if (m_navPvCallback)
                {
                    m_navPvCallback(structRef);
                }
                if (m_gnssHandler)
                {
                    m_gnssHandler->handleNavPv(structRef);
                }
            }
            break;

        case MERGE_CLASS_ID(CASIC_NAV_CLASS, CASIC_NAV_TIMEUTC_ID):
            // NAV-TIMEUTC
            if (m_msg.length == sizeof(CasicMsgPayloads::NavTimeUTC))
            {
                // Callback is assigned and we have a valid message length.
                CasicMsgPayloads::NavTimeUTC &structRef = *reinterpret_cast<CasicMsgPayloads::NavTimeUTC *>(m_msg.payload);
                if (m_navTimeUtcCallback)
                {
                    m_navTimeUtcCallback(structRef);
                }
                if (m_gnssHandler)
                {
                    m_gnssHandler->handleNavTimeUtc(structRef);
                }
            }
            break;

        case MERGE_CLASS_ID(CASIC_NAV_CLASS, CASIC_NAV_GPSINFO_ID):
        case MERGE_CLASS_ID(CASIC_NAV_CLASS, CASIC_NAV_BDSINFO_ID):
        case MERGE_CLASS_ID(CASIC_NAV_CLASS, CASIC_NAV_GLNINFO_ID):
            // NAV-GPSINFO, NAV-BDSINFO, NAV-GLNINFO respectively.
            if (m_msg.length == sizeof(CasicMsgPayloads::NavSolutionInfo))
            {
                // Callback is assigned and we have a valid message length.
                CasicMsgPayloads::NavSolutionInfo &structRef = *reinterpret_cast<CasicMsgPayloads::NavSolutionInfo *>(m_msg.payload);
                if (m_navInfoCallback)
                {
                    m_navInfoCallback(structRef);
                }
                if (m_gnssHandler)
                {
                    m_gnssHandler->handleNavInfo(structRef);
                }
            }
            break;

        default:
            // Ignore unrecognised messages.
            break;
        }
    }
}

void Casic::m_handleAckNack(bool isAck, CasicMsgPayloads::Ack &msg)
{
    // Calling getCfgState now so we can check for timeouts automatically.
    if (getCfgState() == CfgState::WAITING && msg.classId == m_cfgCls && msg.messageId == m_cfgId)
    {
        // This ack/nack applies to this message.
        if (isAck)
        {
            // Successfully acklowedged, can go back to idle.
            m_cfgState = CfgState::IDLE;
        }
        else
        {
            // The module didn't like something.
            m_cfgState = CfgState::NACK_RECEIVED;
        }
    }
}

bool Casic::m_sendCfg(CasicMsg &cfg)
{
    // Calling getCfgState now so we can check for timeouts automatically.
    if (getCfgState() != CfgState::WAITING)
    {
        // Allow the message to be sent.
        cfg.send(m_ser);
        m_cfgStartTime = micros();
        m_cfgId = cfg.id;
        m_cfgCls = cfg.cls;
        m_cfgState = CfgState::WAITING;
        return true;
    }
    return false;
}

bool Casic::cfgPrt(CasicMsgPayloads::CfgPrt &cfg)
{
    CasicMsg msg(reinterpret_cast<char *>(&cfg), sizeof(cfg));
    msg.cls = CASIC_CFG_CLASS;
    msg.id = CASIC_CFG_PRT_ID;
    msg.length = sizeof(cfg);
    return m_sendCfg(msg);
}

bool Casic::cfgMsg(CasicMsgPayloads::CfgMsg &cfg)
{
    CasicMsg msg(reinterpret_cast<char *>(&cfg), sizeof(cfg));
    msg.cls = CASIC_CFG_CLASS;
    msg.id = CASIC_CFG_MSG_ID;
    msg.length = sizeof(cfg);
    return m_sendCfg(msg);
}

bool Casic::cfgRate(CasicMsgPayloads::CfgRate &cfg)
{
    CasicMsg msg(reinterpret_cast<char *>(&cfg), sizeof(cfg));
    msg.cls = CASIC_CFG_CLASS;
    msg.id = CASIC_CFG_RATE_ID;
    msg.length = sizeof(cfg);
    return m_sendCfg(msg);
}

void Casic::waitForCfg()
{
    while (getCfgState() == CfgState::WAITING)
    {
        update();
        delay(1);
    }
}
