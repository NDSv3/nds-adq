//
// Copyright (c) 2018 Cosylab d.d.
// This software is distributed under the terms found
// in file LICENSE.txt that is included with this distribution.
//

#ifndef ADQINFO_H
#define ADQINFO_H

#include <mutex>
#include <nds3/nds.h>

/** @file ADQInfo.h
 * @brief This file defines ADQInfo class.
 */

/** @class ADQInfo
 * @brief This class monitors informative parameters of the connected digitizer.
 */
class ADQInfo
{
public:
    /** @fn ADQInfo(const std::string& name, nds::Node& parentNode, ADQInterface*& adqInterface);
     * @brief ADQInfo class constructor.
     * @param name a name with which this class will register its child node.
     * @param parentNode a name of a parent node to which this class' node is a child.
     * @param adqInterface a pointer to the ADQ API interface created in the ADQDevice class.
     */
    ADQInfo(const std::string& name, nds::Node& parentNode, ADQInterface* adqInterface);
    virtual ~ADQInfo();

    /** @fn getProductName
     * @brief Gets the digitizer's product name.
     */
    void getProductName(timespec* pTimestamp, std::string* pValue);

    /** @fn getSerialNumber
     * @brief Gets the digitizer's serial number.
     */
    void getSerialNumber(timespec* pTimestamp, std::string* pValue);

    /** @fn getProductID
     * @brief Gets the digitizer's product ID.
     */
    void getProductID(timespec* pTimestamp, int32_t* pValue);

    /** @fn getADQType
     * @brief Gets the digitizer's type.
     */
    void getADQType(timespec* pTimestamp, int32_t* pValue);

    /** @fn getCardOption
     * @brief Gets the digitizer's card option.
     */
    void getCardOption(timespec* pTimestamp, std::string* pValue);

    /** @fn isAlive
     * @brief Is the digitiser operational and connected?
     */
    void isAlive(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempLocal
     * @brief Gets the digitizer's PCB temperature.
     */
    void getTempLocal(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempADCone
     * @brief Gets the digitizer's ADC1 temperature.
     */
    void getTempADCone(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempADCtwo
     * @brief Gets the digitizer's ADC2 temperature.
     */
    void getTempADCtwo(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempFPGA
     * @brief Gets the digitizer's FPGA temperature.
     */
    void getTempFPGA(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempDCDC2A
     * @brief Gets the digitizer's DCDC2A temperature.
     */
    void getTempDCDC2A(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempDCDC2B
     * @brief Gets the digitizer's DCDC2A temperature.
     */
    void getTempDCDC2B(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempDCDC2B
     * @brief Gets the digitizer's DCDC2A temperature.
     */
    void getTempDCDC1(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTempDCDC2B
     * @brief Gets the digitizer's DCDC2A temperature.
     */
    void getTempRSVD(timespec* pTimestamp, int32_t* pValue);

    /** @fn getPLL1_lock_lost
     * @brief Gets the Phase Lock Loop 1 status of the digitizer.
     */
    void getPLL1_lock_lost(timespec* pTimestamp, int32_t* pValue);
    /** @fn getPLL2_lock_lost
     * @brief Gets the Phase Lock Loop 2 status of the digitizer.
     */
    void getPLL2_lock_lost(timespec* pTimestamp, int32_t* pValue);
    /** @fn getfrequencyDescrepancy
     * @brief Gets the difference between the smaple rate and the Phase Lock Loop estimated frequency.
     */
    void getfrequencyDescrepancy(timespec* pTimestamp, int32_t* pValue);
    /** @fn getSampRate
     * @brief Gets the digitizer's base sample rate.
     */
    void getSampRate(timespec* pTimestamp, double* pValue);

    /** @fn getSampRateDec
     * @brief Gets the digitizer's decimated sample rate.
     */
    void getSampRateDec(timespec* pTimestamp, double* pValue);

    /** @fn getBytesPerSample
     * @brief Gets the number of bytes needed to store each sample.
     */
    void getBytesPerSample(timespec* pTimestamp, int32_t* pValue);

    /** @fn getBusAddr
     * @brief Gets the digitizer's bus address.
     */
    void getBusAddr(timespec* pTimestamp, int32_t* pValue);

    /** @fn getBusType
     * @brief Gets the digitizer's type of connection.
     */
    void getBusType(timespec* pTimestamp, int32_t* pValue);

    /** @fn getPCIeLinkRate
     * @brief Gets the PCIe/PXIe generation if the digitizer is connected over this interface.
     */
    void getPCIeLinkRate(timespec* pTimestamp, int32_t* pValue);

    /** @fn getPCIeLinkWid
     * @brief Gets the PCIe/PXIe width if the digitizer is connected over this interface.
     */
    void getPCIeLinkWid(timespec* pTimestamp, int32_t* pValue);

private:
    std::string m_logMsg;
    nds::PVDelegateIn<std::string> m_logMsgPV;
    std::string m_productName;
    nds::PVDelegateIn<std::string> m_productNamePV;
    nds::PVDelegateIn<std::string> m_serialNumberPV;
    nds::PVDelegateIn<int32_t> m_productIDPV;
    nds::PVDelegateIn<int32_t> m_adqTypePV;
    nds::PVDelegateIn<std::string> m_cardOptionPV;
    nds::PVDelegateIn<int32_t> m_isAlivePV;
    int32_t m_isAlive;
    nds::PVDelegateIn<int32_t> m_tempLocalPV;
    nds::PVDelegateIn<int32_t> m_tempAdcOnePV;
    nds::PVDelegateIn<int32_t> m_tempAdcTwoPV;
    nds::PVDelegateIn<int32_t> m_tempFpgaPV;
    nds::PVDelegateIn<int32_t> m_tempDCDC2APV;
    nds::PVDelegateIn<int32_t> m_tempDCDC2BPV;
    nds::PVDelegateIn<int32_t> m_tempDCDC1PV;
    nds::PVDelegateIn<int32_t> m_tempRSVDPV;
    nds::PVDelegateIn<int32_t> m_pll1_lock_lostPV;
    int32_t m_pll1_lock_lost;
    nds::PVDelegateIn<int32_t> m_pll2_lock_lostPV;
    int32_t m_pll2_lock_lost;
    nds::PVDelegateIn<int32_t>  m_frequencyDescrepancyPV;
    int32_t m_frequencyDescrepancy;
    nds::PVDelegateIn<double> m_sampRatePV;
    nds::PVDelegateIn<int32_t> m_bytesPerSampPV;
    nds::PVDelegateIn<int32_t> m_busTypePV;
    nds::PVDelegateIn<int32_t> m_busAddrPV;
    nds::PVDelegateIn<int32_t> m_pcieLinkRatePV;
    nds::PVDelegateIn<int32_t> m_pcieLinkWidPV;

    int m_adqType;
    int64_t m_SampleRate;
    /** @var m_node
     * @brief ADQInfo class child node that connects to the root node.
     */
    nds::Port m_node;
    void reportLockLost(const char* pllNum, int& m_lock_lost, int lock_lost);

protected:
    /** @var m_adqDevMutex
    * @brief Lock guard.
    *
    * It is used to protect ADQAPI library from simultaneous calling from different threads.
    * For example, updating the temperatures (ADQInfo class) and data acquisition (ADQAIChannelGroup).
    */
    std::mutex m_adqDevMutex;

    /** @fn getLogMsg
     * @brief Gets the log messages.
     */
    nds::PVDelegateIn<std::string>& logMsgPV() {
        return m_logMsgPV;
    }
    void getLogMsg(timespec* pTimestamp, std::string* pValue);
    void setLogMsg(const timespec& pTimestamp, std::string const& pValue);

    /** @def ADQNDS_MSG_WARNLOG_PV
     * @brief Macro for warning information in case of minor failures.
     * Used in ADQAIChannelGroup methods.
     * @param status status of the function that calls this macro.
     * @param text input information message.
     */
    std::string utc_system_timestamp(struct timespec const& now, char sep) const;
    void ADQNDS_MSG_WARNLOG_PV(int status, std::string const& text)
    {
        if (!status)
        {
            struct timespec now = { 0, 0 };
            clock_gettime(CLOCK_REALTIME, &now);
            std::string Warning = "WARNING: " + utc_system_timestamp(now, ' ') + " " + m_node.getFullExternalName() + " " + text;
            m_logMsgPV.push(now, std::string(Warning));
            ndsWarningStream(m_node) << Warning << std::endl;
        }
    }

    ADQInterface* m_adqInterface;

    /** @var m_sampRateDecPV
    * @brief PV fpr sample rate with decimation.
    *
    * Its value is updated when sample skip (ADQAIChannelGroup) is changed.
    */
    nds::PVDelegateIn<double> m_sampRateDecPV;
    int adqType() const {
        return m_adqType;
    }
    int64_t SampleRate() const {
        return m_SampleRate;
    }
};

#endif /* ADQINFO_H */
