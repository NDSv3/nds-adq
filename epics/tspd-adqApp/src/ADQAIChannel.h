//
// Copyright (c) 2018 Cosylab d.d.
// This software is distributed under the terms found
// in file LICENSE.txt that is included with this distribution.
//

#ifndef ADQAICHANNEL_H
#define ADQAICHANNEL_H

#include <nds3/nds.h>
#include <cstring>

/** @file ADQAIChannel.h
 * @brief This file defines ADQAIChannel class.
 */

/** @class ADQAIChannel
 * @brief This class handles channel specific parameters and the transfer of the acquired data to the data PV.
 * Each physical channel is represented by ADQAIChannel with a correcponding channelNum index.
 */
class ADQAIChannel
{
public:
    /** @fn ADQAIChannel(const std::string& name, nds::Node& parentNode, int32_t channelNum, ADQInterface*& adqInterface, nds::PVDelegateIn<std::string> logMsgPV);
     * @brief ADQAIChannel class constructor.
     * @param name a name with which this class will register its child node.
     * @param parentNode a name of a parent node to which this class' node is a child.
     * @param channelNum a number of channel which a constructed class represents.
     */
    ADQAIChannel(const std::string& name, nds::Node& parentNode, int32_t channelNum);
    void resize(int32_t sampleCnt) {
        m_data.resize(sampleCnt);
    }

    /** @var m_channelNum
     * @brief Number of channel.
     */
    int32_t m_channelNum;

    /** @fn setInputRange
    * @brief Sets the channel's input range.
    */
    void setInputRange(const timespec& pTimestamp, const double& pValue);

    /** @fn getInputRange
     * @brief Gets the channel's input range.
     */
    void getInputRange(timespec* pTimestamp, double* pValue);

    /** @fn setInputImpedance
    * @brief Sets the channel's input impedance, 50 Ohm or 1M Ohm.
    */
    void setInputImpedance(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getInputRange
     * @brief Gets the channel's input impedance, 50 Ohm or 1M Ohm.
     */
    void getInputImpedance(timespec* pTimestamp, int32_t* pValue);

    /** @fn setDcBias
     * @brief Sets the channel's DC bias.
     */
    void setDcBias(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getDcBias
     * @brief Gets the channel's DC bias.
     */
    void getDcBias(timespec* pTimestamp, int32_t* pValue);

    /** @fn setChanDec
    * @brief Sets the channel's sample decimation.
    */
    void setChanDec(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getChanDec
    * @brief Gets the channel's sample decimation.
    */
    void getChanDec(timespec* pTimestamp, int32_t* pValue);

    /** @fn setState
     * @brief Sets a new state to the ADQAIChannel class' state machine.
     */
    void setState(nds::state_t newState);

    /** @fn readData
     * @brief This method passes the acquired data to appropriate data PV.
     */
    void readData(int16_t* rawData, int32_t sampleCnt, struct timespec const& now) {
        readData(rawData, sampleCnt);
        pushData(now);
    }

    void readData(int16_t* rawData, int32_t sampleCnt) {
        m_data.resize(sampleCnt);
        memcpy(&(m_data[0]), rawData, sizeof(int16_t)*sampleCnt);
    }
    void pushData(struct timespec const& now) {
        m_dataPV.push(now, m_data);
    }

    /** @fn getDataPV
     * @brief This is a dummy method held by the data PV for appropriate work in NDS3. 
     */
    void getDataPV(timespec* pTimestamp, std::vector<int16_t>* pValue);

    /** @fn commitChanges
     * @brief This method processes changes are applied to channel specific parameters.
     * @param calledFromDaqThread a flag that prevents this function to be called when set to false.
     * @param adqInterface a pointer to the ADQ API interface created in the ADQDevice class.
     * @param logMsgPV a PV from ADQAIGroupChannel that receives any log messages.
     */
    void commitChanges(bool calledFromDaqThread, ADQInterface* adqInterface, nds::PVDelegateIn<std::string>& logMsgPV);

private:
    nds::Node m_node;
    nds::StateMachine m_stateMachine;

    double m_inputRange;
    bool m_inputRangeChanged;

    int32_t m_inputImpedance;
    bool m_inputImpedanceChanged;

    int32_t m_dcBias;
    bool m_dcBiasChanged;

    int32_t m_chanDec;
    bool m_chanDecChanged;

    std::vector<int16_t> m_data;

    void switchOn();
    void switchOff();
    void start();
    void stop();
    void recover();
    bool allowChange(const nds::state_t, const nds::state_t, const nds::state_t);

    nds::PVDelegateIn<double> m_inputRangePV;
    nds::PVDelegateIn<int32_t> m_inputImpedancePV;
    nds::PVDelegateIn<int32_t> m_dcBiasPV;
    nds::PVDelegateIn<int32_t> m_chanDecPV;
    nds::PVDelegateIn<std::vector<int16_t>> m_dataPV;
    /** @def ADQNDS_MSG_WARNLOG_PV
     * @brief Macro for warning information in case of minor failures.
     * Used in ADQAIChannelGroup methods.
     * @param status status of the function that calls this macro.
     * @param text input information message.
     */
    void ADQNDS_MSG_WARNLOG_PV(int status, std::string const& text, nds::PVDelegateIn<std::string>& logMsgPV)
    {
        if (!status)
        {
            struct timespec now = { 0, 0 };
            clock_gettime(CLOCK_REALTIME, &now);
            std::string Warning = "WARNING: " + utc_system_timestamp(now, ' ') + " " + m_node.getFullExternalName() + " " + text;
            logMsgPV.push(now, std::string(Warning));
            ndsWarningStream(m_node) << Warning << std::endl;
        }
    }
    std::string utc_system_timestamp(struct timespec const& now, char sep) const;
};

#endif /* ADQAICHANNEL_H; */
