//
// Copyright (c) 2018 Cosylab d.d.
// This software is distributed under the terms found
// in file LICENSE.txt that is included with this distribution.
//

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <string.h>

#include <ADQAPI.h>
#include <nds3/nds.h>

#include "ADQAIChannel.h"
#include "ADQAIChannelGroup.h"
#include "ADQDefinition.h"
#include "ADQDevice.h"

ADQAIChannel::ADQAIChannel(const std::string& name, nds::Node& parentNode, int32_t channelNum) :
    m_channelNum(channelNum),
    m_inputRangePV(nds::PVDelegateIn<double>("InputRange-RB", std::bind(&ADQAIChannel::getInputRange, this,
                                                                        std::placeholders::_1, std::placeholders::_2))),
    m_inputImpedancePV(nds::PVDelegateIn<int32_t>("InputImpedance-RB", std::bind(&ADQAIChannel::getInputImpedance, this, std::placeholders::_1,
                                                                                 std::placeholders::_2))),
    m_dcBiasPV(nds::PVDelegateIn<int32_t>("DCBias-RB", std::bind(&ADQAIChannel::getDcBias, this, std::placeholders::_1,
                                                                 std::placeholders::_2))),
    m_chanDecPV(nds::PVDelegateIn<int32_t>("ChanDec-RB", std::bind(&ADQAIChannel::getChanDec, this,
                                                                   std::placeholders::_1, std::placeholders::_2))),
    m_dataPV(nds::PVDelegateIn<std::vector<int16_t>>("Data", std::bind(&ADQAIChannel::getDataPV, this,
                                                                       std::placeholders::_1, std::placeholders::_2)))
{
    m_node = parentNode.addChild(nds::Node(name));

    m_inputImpedanceChanged = m_inputRangeChanged = m_dcBiasChanged = true;
    m_chanDecChanged = false; //Decimation not supported on some models.

    m_inputRange = 1000; // in mV
    m_inputImpedance = m_dcBias = m_chanDec = 0;

    // PV for input range
    nds::PVDelegateOut<double> nodeFloat(nds::PVDelegateOut<double>(
        "InputRange",
        std::bind(&ADQAIChannel::setInputRange, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ADQAIChannel::getInputRange, this, std::placeholders::_1, std::placeholders::_2)));
    m_node.addChild(nodeFloat);
    m_inputRangePV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_inputRangePV);

    // PV for input impedance
    nds::PVDelegateOut<int32_t> node(nds::PVDelegateOut<int32_t>(
        "InputImpedance",
        std::bind(&ADQAIChannel::setInputImpedance, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ADQAIChannel::getInputImpedance, this, std::placeholders::_1, std::placeholders::_2)));
    m_node.addChild(node);
    m_inputImpedancePV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_inputImpedancePV);

    // PVs for Adjustable Bias
    node = (nds::PVDelegateOut<int32_t>(
                                       "DCBias",
                                       std::bind(&ADQAIChannel::setDcBias, this, std::placeholders::_1, std::placeholders::_2),
                                       std::bind(&ADQAIChannel::getDcBias, this, std::placeholders::_1, std::placeholders::_2)));

    m_node.addChild(node);
    m_dcBiasPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_dcBiasPV);

    // PVs for Sample Decimation
    node = nds::PVDelegateOut<int32_t>("ChanDec",
                                       std::bind(&ADQAIChannel::setChanDec, this, std::placeholders::_1, std::placeholders::_2),
                                       std::bind(&ADQAIChannel::getChanDec, this, std::placeholders::_1, std::placeholders::_2));

    m_node.addChild(node);
    m_chanDecPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_chanDecPV);

    // PV for data
    m_dataPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_dataPV);

    // PV for state machine
    m_stateMachine = nds::StateMachine(true,
                                       std::bind(&ADQAIChannel::switchOn, this),
                                       std::bind(&ADQAIChannel::switchOff, this),
                                       std::bind(&ADQAIChannel::start, this),
                                       std::bind(&ADQAIChannel::stop, this),
                                       std::bind(&ADQAIChannel::recover, this),
                                       std::bind(&ADQAIChannel::allowChange, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    m_node.addChild(m_stateMachine);
}

void ADQAIChannel::setInputRange(const timespec& pTimestamp, const double& pValue)
{
    m_inputRange = pValue;
    m_inputRangePV.getTimestamp() = pTimestamp;
    m_inputRangeChanged = true;
}

void ADQAIChannel::getInputRange(timespec* pTimestamp, double* pValue)
{
    *pValue = m_inputRange;
    *pTimestamp = m_inputRangePV.getTimestamp();
}

void ADQAIChannel::setInputImpedance(const timespec& pTimestamp, const int32_t& pValue)
{
    m_inputImpedance = pValue;
    m_inputImpedancePV.getTimestamp() = pTimestamp;
    m_inputImpedanceChanged = true;
}

void ADQAIChannel::getInputImpedance(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_inputImpedance;
    *pTimestamp = m_inputImpedancePV.getTimestamp();
}

void ADQAIChannel::setDcBias(const timespec& pTimestamp, const int32_t& pValue)
{
    m_dcBias = pValue;
    m_dcBiasPV.getTimestamp() = pTimestamp;
    m_dcBiasChanged = true;
}

void ADQAIChannel::getDcBias(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_dcBias;
    *pTimestamp = m_dcBiasPV.getTimestamp();
}

void ADQAIChannel::setChanDec(const timespec& pTimestamp, const int32_t& pValue)
{
    if (m_chanDec != pValue)
    {
        m_chanDec = pValue;
        m_chanDecPV.getTimestamp() = pTimestamp;
        m_chanDecChanged = true;
    }
}

void ADQAIChannel::getChanDec(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_chanDec;
    *pTimestamp = m_chanDecPV.getTimestamp();
}

/* This function updates readback PVs according to changed each channel's parameter
 * and applies them to the device with ADQAPI functions.
 */
void ADQAIChannel::commitChanges(bool calledFromDaqThread, ADQInterface* adqInterface, nds::PVDelegateIn<std::string>& logMsgPV)
{
    struct timespec now = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &now);
    unsigned int status = 0;
    std::ostringstream textTmp;

    int adqType = adqInterface->GetADQType();
    std::string adqOption = adqInterface->GetCardOption();

    if (!calledFromDaqThread &&
        (m_stateMachine.getLocalState() != nds::state_t::on && m_stateMachine.getLocalState() != nds::state_t::stopping &&
         m_stateMachine.getLocalState() != nds::state_t::initializing))
    {
        return;
    }

    if (m_inputRangeChanged)
    {
        float inputRangeRb = 0;
        m_inputRangeChanged = false;

        if ((adqType == 714 || adqType == 14 || adqType == 8) && (adqOption.find("-VG") != std::string::npos))
        {
            status = adqInterface->HasAdjustableInputRange();
            if (status)
            {
                if (m_inputRange <= 0)
                {
                    m_inputRange = 500;

                    textTmp << "INFO: Input range is set to 500 mVpp by default, CH" << m_channelNum;
                    std::string textForPV(textTmp.str());
                    ndsInfoStream(m_node) << std::string(textForPV) << std::endl;
                }

                if (adqType == 8)
                {
                    if (m_inputRange <= 500)
                        m_inputRange = 500;
                    else if (m_inputRange <= 1000)
                        m_inputRange = 1000;
                    else if (m_inputRange <= 2500)
                        m_inputRange = 2500;
                    else if (m_inputRange > 5000)
                        m_inputRange = 5000;
                }

                status = adqInterface->SetInputRange(m_channelNum + 1, m_inputRange, &inputRangeRb);
                if (status)
                    m_inputRangePV.push(now, (double)inputRangeRb);
                else
                {
                    textTmp << "SetInputRange failed, CH" << m_channelNum;
                    std::string textForPV(textTmp.str());
                    ADQNDS_MSG_WARNLOG_PV(status, textForPV, logMsgPV);
                }
            }
            else
            {
                textTmp << "INFO: Adjustable input range is not supported, CH" << m_channelNum;
                std::string textForPV(textTmp.str());
                ndsInfoStream(m_node) << std::string(textForPV) << std::endl;
            }
        }
        else
        {
            textTmp << "INFO: Adjustable input range is not supported, CH" << m_channelNum;
            std::string textForPV(textTmp.str());
            ndsInfoStream(m_node) << std::string(textForPV) << std::endl;
        }
    }

    if (m_inputImpedanceChanged)
    {
        m_inputImpedanceChanged = false;
        if (adqType == 8)
        {
            status = adqInterface->SetInputImpedance(m_channelNum + 1, m_inputImpedance);
            if (!status)
            {
                textTmp << "SetInputImpedance failed on CH" << m_channelNum;
                std::string textForPV(textTmp.str());
                ADQNDS_MSG_WARNLOG_PV(status, textForPV, logMsgPV);
            }
            else
            {
                unsigned int inputImpedance = 0;
                status = adqInterface->GetInputImpedance(m_channelNum + 1, &inputImpedance);
                m_inputImpedance = inputImpedance;
                m_inputImpedancePV.push(now, m_inputImpedance);
            }
        }
    }

    if (m_dcBiasChanged)
    {
        m_dcBiasChanged = false;

        if (adqInterface->HasAdjustableBias())
        {
            status = adqInterface->SetAdjustableBias(m_channelNum + 1, m_dcBias);
            SLEEP(1000);

            if (!status)
            {
                textTmp << "SetAdjustableBias failed on CH" << m_channelNum;
                std::string textForPV(textTmp.str());
                ADQNDS_MSG_WARNLOG_PV(status, textForPV, logMsgPV);
            }
            else
            {
                int dcBias = 0;
                status = adqInterface->GetAdjustableBias(m_channelNum + 1, &dcBias);
                m_dcBias = dcBias;
                m_dcBiasPV.push(now, m_dcBias);
            }
        }
        else
        {
            textTmp << "INFO: Device doesn't support adjustable bias, CH" << m_channelNum;
            std::string textForPV(textTmp.str());
            ndsInfoStream(m_node) << std::string(textForPV) << std::endl;
        }
    }

    if (m_chanDecChanged)
    {
        m_chanDecChanged = false;

        if ((adqType == 7) && (adqOption.find("-FWSDR") != std::string::npos))
        {
            if (m_chanDec < 0)
            {
                m_chanDec = 0;
            }

            status = adqInterface->SetChannelDecimation(m_channelNum, m_chanDec);
            ADQNDS_MSG_WARNLOG_PV(status, "SetSampleDecimation failed.", logMsgPV);

            if (status)
            {
                unsigned int tmp = 0;
                status = adqInterface->GetChannelDecimation(m_channelNum, &tmp);
                m_chanDec = tmp;
                m_chanDecPV.push(now, m_chanDec);
            }
        }
        else
        {
            ADQNDS_MSG_WARNLOG_PV(0, "Sample channel decimation is not supported on this device.", logMsgPV);
        }
    }
}

void ADQAIChannel::setState(nds::state_t newState)
{
    m_stateMachine.setState(newState);
}

void ADQAIChannel::switchOn()
{
}

void ADQAIChannel::switchOff()
{
}

void ADQAIChannel::start()
{
}

void ADQAIChannel::stop()
{
}

void ADQAIChannel::recover()
{
    throw nds::StateMachineRollBack("INFO: Cannot recover");
}

bool ADQAIChannel::allowChange(const nds::state_t, const nds::state_t, const nds::state_t)
{
    return true;
}

/* @brief Dummy method for the m_dataPV. The readData method pushes data to m_dataPV. @ref readData
 */
void ADQAIChannel::getDataPV(timespec* pTimestamp, std::vector<int16_t>* pValue)
{
    UNUSED(pTimestamp);
    UNUSED(pValue);
}

std::string ADQAIChannel::utc_system_timestamp(struct timespec const& now, char sep) const
{
    // https://stackoverflow.com/questions/15106102/how-to-use-c-stdostream-with-printf-like-formatting
    const int bufsize = 31;
    const int tmpsize = 21;
    char buf[bufsize];
    struct tm* tm = gmtime(&now.tv_sec);
    strftime(buf, tmpsize, "%Y-%m-%d %H:%M:%S.", tm);
    sprintf(buf + tmpsize - 1, "%09lu%c", now.tv_nsec, sep);
    return buf;
}
