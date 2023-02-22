//
// Copyright (c) 2018 Cosylab d.d.
// This software is distributed under the terms found
// in file LICENSE.txt that is included with this distribution.
//

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include <ADQAPI.h>
#include <nds3/nds.h>

#include "ADQDefinition.h"
#include "ADQInfo.h"

ADQInfo::ADQInfo(const std::string& name, nds::Node& parentNode, ADQInterface* adqInterface) :
    m_logMsgPV(nds::PVDelegateIn<std::string>("LogMsg", std::bind(&ADQInfo::getLogMsg, this,
                                              std::placeholders::_1, std::placeholders::_2))),
    m_productNamePV(nds::PVDelegateIn<std::string>("ProdName", std::bind(&ADQInfo::getProductName, this,
                                                                         std::placeholders::_1, std::placeholders::_2))),
    m_serialNumberPV(nds::PVDelegateIn<std::string>("ProdSerial", std::bind(&ADQInfo::getSerialNumber, this,
                                                                            std::placeholders::_1, std::placeholders::_2))),
    m_productIDPV(nds::PVDelegateIn<int32_t>("ProdID", std::bind(&ADQInfo::getProductID, this, std::placeholders::_1,
                                                                 std::placeholders::_2))),
    m_adqTypePV(nds::PVDelegateIn<int32_t>("ProdType", std::bind(&ADQInfo::getADQType, this, std::placeholders::_1,
                                                                 std::placeholders::_2))),
    m_cardOptionPV(nds::PVDelegateIn<std::string>("ProdOpt", std::bind(&ADQInfo::getCardOption, this,
                                                                       std::placeholders::_1, std::placeholders::_2))),
    m_isAlivePV(nds::PVDelegateIn<int32_t>("isAlive", std::bind(&ADQInfo::isAlive, this,
        std::placeholders::_1, std::placeholders::_2))),
    m_tempLocalPV(nds::PVDelegateIn<int32_t>("TempLocal", std::bind(&ADQInfo::getTempLocal, this,
        std::placeholders::_1, std::placeholders::_2))),
    m_tempAdcOnePV(nds::PVDelegateIn<int32_t>("TempADC-1", std::bind(&ADQInfo::getTempADCone, this,
                                                                     std::placeholders::_1, std::placeholders::_2))),
    m_tempAdcTwoPV(nds::PVDelegateIn<int32_t>("TempADC-2", std::bind(&ADQInfo::getTempADCtwo, this,
                                                                     std::placeholders::_1, std::placeholders::_2))),
    m_tempFpgaPV(nds::PVDelegateIn<int32_t>("TempFPGA", std::bind(&ADQInfo::getTempFPGA, this, std::placeholders::_1,
                                                                  std::placeholders::_2))),
    m_tempDCDC2APV(nds::PVDelegateIn<int32_t>("TempDCDC2A", std::bind(&ADQInfo::getTempDCDC2A, this, std::placeholders::_1,
                                                                      std::placeholders::_2))),
    m_tempDCDC2BPV(nds::PVDelegateIn<int32_t>("TempDCDC2B", std::bind(&ADQInfo::getTempDCDC2B, this, std::placeholders::_1,
                                                                      std::placeholders::_2))),
    m_tempDCDC1PV(nds::PVDelegateIn<int32_t>("TempDCDC1", std::bind(&ADQInfo::getTempDCDC1, this, std::placeholders::_1,
                                                                    std::placeholders::_2))),
    m_tempRSVDPV(nds::PVDelegateIn<int32_t>("TempRSVD", std::bind(&ADQInfo::getTempRSVD, this, std::placeholders::_1,
                                                                  std::placeholders::_2))),
    m_pll1_lock_lostPV(nds::PVDelegateIn<int32_t>("PLL1_lock_lost", std::bind(&ADQInfo::getPLL1_lock_lost, this,
                                                                              std::placeholders::_1, std::placeholders::_2))),
    m_pll2_lock_lostPV(nds::PVDelegateIn<int32_t>("PLL2_lock_lost", std::bind(&ADQInfo::getPLL2_lock_lost, this,
                                                                              std::placeholders::_1, std::placeholders::_2))),
    m_frequencyDescrepancyPV(nds::PVDelegateIn<int32_t>("frequencyDescrepancy", std::bind(&ADQInfo::getfrequencyDescrepancy, this,
                                                                                          std::placeholders::_1, std::placeholders::_2))),
    m_sampRatePV(nds::PVDelegateIn<double>("SampRate", std::bind(&ADQInfo::getSampRate, this, std::placeholders::_1,
                                                                 std::placeholders::_2))),
    m_bytesPerSampPV(nds::PVDelegateIn<int32_t>("BytesPerSample", std::bind(&ADQInfo::getBytesPerSample, this,
                                                                            std::placeholders::_1, std::placeholders::_2))),
    m_busTypePV(nds::PVDelegateIn<int32_t>("BusType", std::bind(&ADQInfo::getBusType, this, std::placeholders::_1,
                                                                std::placeholders::_2))),
    m_busAddrPV(nds::PVDelegateIn<int32_t>("BusAddr", std::bind(&ADQInfo::getBusAddr, this, std::placeholders::_1,
                                                                std::placeholders::_2))),
    m_pcieLinkRatePV(nds::PVDelegateIn<int32_t>("PCIeLinkRate", std::bind(&ADQInfo::getPCIeLinkRate, this,
                                                                          std::placeholders::_1, std::placeholders::_2))),
    m_pcieLinkWidPV(nds::PVDelegateIn<int32_t>("PCIeLinkWid", std::bind(&ADQInfo::getPCIeLinkWid, this,
                                                                        std::placeholders::_1, std::placeholders::_2))),
    m_node(nds::Port(name + INFO_DEVICE, nds::nodeType_t::generic)),
    m_adqInterface(adqInterface),
    m_sampRateDecPV(nds::PVDelegateIn<double>("SampRateDec", std::bind(&ADQInfo::getSampRateDec, this,
                                                                       std::placeholders::_1, std::placeholders::_2)))
{
    parentNode.addChild(m_node);
    m_adqType = m_adqInterface->GetADQType();
    double SampleRate = 1000000000;
    int status = m_adqInterface->GetSampleRate(0, &SampleRate);
    m_SampleRate = int64_t(SampleRate + 0.5);
    ADQNDS_MSG_WARNLOG_PV(status, "GetSampleRate failed.");

    // PV for error/warning/info messages
    m_logMsgPV.setScanType(nds::scanType_t::interrupt);
    m_logMsgPV.setMaxElements(512);
    m_node.addChild(m_logMsgPV);

    // PVs for device info
    m_productNamePV.setMaxElements(STRING_ENUM);
    m_productNamePV.processAtInit(PINI);
    m_node.addChild(m_productNamePV);

    m_serialNumberPV.setMaxElements(STRING_ENUM);
    m_serialNumberPV.processAtInit(PINI);
    m_node.addChild(m_serialNumberPV);

    m_productIDPV.processAtInit(PINI);
    m_node.addChild(m_productIDPV);

    m_adqTypePV.processAtInit(PINI);
    m_node.addChild(m_adqTypePV);

    m_cardOptionPV.processAtInit(PINI);
    m_cardOptionPV.setMaxElements(STRING_ENUM);
    m_node.addChild(m_cardOptionPV);

    // PVs for temperatures
    nds::enumerationStrings_t isAliveList = { "Fail", "OK"};
    m_isAlivePV.setEnumeration(isAliveList);
    m_isAlivePV.setScanType(nds::scanType_t::periodic);
    m_isAlivePV.processAtInit(PINI);
    m_node.addChild(m_isAlivePV);
    m_isAlive = true;

    m_tempLocalPV.processAtInit(PINI);
    m_node.addChild(m_tempLocalPV);

    m_tempAdcOnePV.processAtInit(PINI);
    m_node.addChild(m_tempAdcOnePV);

    m_tempAdcTwoPV.processAtInit(PINI);
    m_node.addChild(m_tempAdcTwoPV);

    m_tempFpgaPV.processAtInit(PINI);
    m_node.addChild(m_tempFpgaPV);

    m_tempDCDC2APV.processAtInit(PINI);
    m_node.addChild(m_tempDCDC2APV);

    m_tempDCDC2BPV.processAtInit(PINI);
    m_node.addChild(m_tempDCDC2BPV);

    m_tempDCDC1PV.processAtInit(PINI);
    m_node.addChild(m_tempDCDC1PV);

    m_tempRSVDPV.processAtInit(PINI);
    m_node.addChild(m_tempRSVDPV);

    m_pll1_lock_lostPV.processAtInit(PINI);
    m_node.addChild(m_pll1_lock_lostPV);
    m_pll1_lock_lost = 0;
    m_pll2_lock_lostPV.processAtInit(PINI);
    m_node.addChild(m_pll2_lock_lostPV);
    m_pll2_lock_lost = 0;
    m_frequencyDescrepancyPV.processAtInit(PINI);
    m_node.addChild(m_frequencyDescrepancyPV);
    m_frequencyDescrepancy = 0;

    // PV for sample rate
    m_sampRatePV.processAtInit(PINI);
    m_node.addChild(m_sampRatePV);

    m_sampRateDecPV.processAtInit(PINI);
    m_node.addChild(m_sampRateDecPV);

    // PV for number of bytes per sample
    m_bytesPerSampPV.processAtInit(PINI);
    m_node.addChild(m_bytesPerSampPV);

    // PV for Bus connection
    m_busAddrPV.processAtInit(PINI);
    m_node.addChild(m_busAddrPV);

    m_busTypePV.processAtInit(PINI);
    m_node.addChild(m_busTypePV);

    m_pcieLinkRatePV.processAtInit(PINI);
    m_node.addChild(m_pcieLinkRatePV);

    m_pcieLinkWidPV.processAtInit(PINI);
    m_node.addChild(m_pcieLinkWidPV);
}

void ADQInfo::getLogMsg(timespec* pTimestamp, std::string* pValue)
{
    *pValue = m_logMsg;
    *pTimestamp = m_logMsgPV.getTimestamp();
}

void ADQInfo::setLogMsg(const timespec& pTimestamp, std::string const& pValue)
{
    m_logMsg = pValue;
    m_logMsgPV.push(pTimestamp, m_logMsg);
}

void ADQInfo::getProductName(timespec* pTimestamp, std::string* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    *pValue = m_adqInterface->GetBoardProductName();
    *pTimestamp = m_productNamePV.getTimestamp();
}

void ADQInfo::getSerialNumber(timespec* pTimestamp, std::string* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    *pValue = m_adqInterface->GetBoardSerialNumber();
    *pTimestamp = m_serialNumberPV.getTimestamp();
}

void ADQInfo::getProductID(timespec* pTimestamp, int32_t* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    *pValue = m_adqInterface->GetProductID();
    *pTimestamp = m_productIDPV.getTimestamp();
}

void ADQInfo::getADQType(timespec* pTimestamp, int32_t* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    *pValue = m_adqInterface->GetADQType();
    *pTimestamp = m_adqTypePV.getTimestamp();
}

void ADQInfo::getCardOption(timespec* pTimestamp, std::string* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    *pValue = m_adqInterface->GetCardOption();
    *pTimestamp = m_cardOptionPV.getTimestamp();
}

void ADQInfo::isAlive(timespec* pTimestamp, int32_t* pValue)
{
    if (!m_adqInterface)
        m_isAlive = false;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        m_isAlive = m_adqInterface->IsAlive();
    }
    *pValue = m_isAlive;
    *pTimestamp = m_isAlivePV.getTimestamp();
}

void ADQInfo::getTempLocal(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_LOCAL) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempLocalPV.getTimestamp();
}

void ADQInfo::getTempADCone(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_ADC_ONE) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempAdcOnePV.getTimestamp();
}

void ADQInfo::getTempADCtwo(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_ADC_TWO) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempAdcTwoPV.getTimestamp();
}

void ADQInfo::getTempFPGA(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_FPGA) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempFpgaPV.getTimestamp();
}

void ADQInfo::getTempDCDC2A(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_DCDC2A) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempDCDC2APV.getTimestamp();
}

void ADQInfo::getTempDCDC2B(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_DCDC2B) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempDCDC2BPV.getTimestamp();
}

void ADQInfo::getTempDCDC1(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_DCDC1) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempDCDC1PV.getTimestamp();
}

void ADQInfo::getTempRSVD(timespec* pTimestamp, int32_t* pValue)
{
    if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 0;
    else
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        *pValue = m_adqInterface->GetTemperature(TEMP_RSVD) * CELSIUS_CONVERT;
    }
    *pTimestamp = m_tempRSVDPV.getTimestamp();
}

void ADQInfo::reportLockLost(const char* pllNum, int& m_lock_lost, int lock_lost)
{
    if (m_lock_lost != lock_lost)
    {
        nds::enumerationStrings_t lockStatusList = { "Lock OK", "Lock recovered", "Lock failed" };
        std::string Warning = pllNum;
        Warning += "_lock_lost changed from ";
        nds::enumerationStrings_t::const_iterator Iter = lockStatusList.begin();
        for (int i = 0; (i < m_lock_lost) && (i < int(lockStatusList.size())); i++)
            Iter++;
        Warning += *Iter;
        Warning += " to ";
        Iter = lockStatusList.begin();
        for (int i = 0; (i < lock_lost) && (i < int(lockStatusList.size())); i++)
            Iter++;
        Warning += *Iter;
        ADQNDS_MSG_WARNLOG_PV(0, Warning);
        m_lock_lost = lock_lost;
    }
}

void ADQInfo::getPLL1_lock_lost(timespec* pTimestamp, int32_t* pValue)
{
    if ((m_adqType != 14) && (m_adqType != 7) && (m_adqType != 8))
        *pValue = 0;
    else if ((!m_adqInterface) || (!m_isAlive))
        *pValue = 2;
    else
    {
        ADQClockSystemStatus ADQClockSystemStatus;
        {
            std::lock_guard<std::mutex> lock(m_adqDevMutex);
            int status = m_adqInterface->GetStatus(ADQ_STATUS_ID_CLOCK_SYSTEM, &ADQClockSystemStatus);
            ADQNDS_MSG_WARNLOG_PV(status, "GetStatus failed.");
        }
        int32_t pll1_lock_lost = m_pll1_lock_lost;
        if ((ADQClockSystemStatus.pll1_lock_detect != -1) && (ADQClockSystemStatus.pll1_lock_lost_alarm != -1)) {
            if (!ADQClockSystemStatus.pll1_lock_detect)
                pll1_lock_lost = 2;
            else if (ADQClockSystemStatus.pll1_lock_lost_alarm)
                pll1_lock_lost = 1;
            reportLockLost("pll1", m_pll1_lock_lost, pll1_lock_lost);
        }
        *pValue = m_pll1_lock_lost;
        int32_t pll2_lock_lost = m_pll2_lock_lost;
        if ((ADQClockSystemStatus.pll2_lock_detect != -1) && (ADQClockSystemStatus.pll2_lock_lost_alarm != -1)) {
            if (!ADQClockSystemStatus.pll2_lock_detect)
                pll2_lock_lost = 2;
            else if (ADQClockSystemStatus.pll2_lock_lost_alarm)
                pll2_lock_lost = 1;
            reportLockLost("pll2", m_pll2_lock_lost, pll2_lock_lost);
        }
        if (ADQClockSystemStatus.reference_source_frequency_estimate != -1.0) {
            m_frequencyDescrepancy = int32_t(ADQClockSystemStatus.reference_source_frequency_estimate - m_SampleRate + 0.5);
        }
    }
    *pTimestamp = m_pll1_lock_lostPV.getTimestamp();
}

void ADQInfo::getPLL2_lock_lost(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_pll2_lock_lost;
    *pTimestamp = m_pll2_lock_lostPV.getTimestamp();
}

void ADQInfo::getfrequencyDescrepancy(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_frequencyDescrepancy;
    *pTimestamp = m_frequencyDescrepancyPV.getTimestamp();
}

void ADQInfo::getSampRate(timespec* pTimestamp, double* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    double sampRate = 0;
    if (!m_adqInterface)
        return;
    m_adqInterface->GetSampleRate(0, &sampRate);
    *pValue = sampRate;
    *pTimestamp = m_sampRatePV.getTimestamp();
}

void ADQInfo::getSampRateDec(timespec* pTimestamp, double* pValue)
{
    if (!m_adqInterface)
        return;
    double sampRateDec = 0;
    m_adqInterface->GetSampleRate(1, &sampRateDec);
    *pValue = sampRateDec;
    *pTimestamp = m_sampRateDecPV.getTimestamp();
}

void ADQInfo::getBytesPerSample(timespec* pTimestamp, int32_t* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    unsigned int bytes = 0;
    if (!m_adqInterface)
        return;
    m_adqInterface->GetNofBytesPerSample(&bytes);
    *pValue = bytes;
    *pTimestamp = m_bytesPerSampPV.getTimestamp();
}

void ADQInfo::getBusAddr(timespec* pTimestamp, int32_t* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    if ((m_adqInterface->IsPCIeDevice()) || (m_adqInterface->IsPCIeLiteDevice()))
    {
        *pValue = m_adqInterface->GetPCIeAddress();
    }

    if ((m_adqInterface->IsUSBDevice()) || (m_adqInterface->IsUSB3Device()))
    {
        *pValue = m_adqInterface->GetUSBAddress();
    }

    *pTimestamp = m_busAddrPV.getTimestamp();
}

void ADQInfo::getBusType(timespec* pTimestamp, int32_t* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    if (m_adqInterface->IsPCIeDevice())
    {
        *pValue = 0;
    }

    if (m_adqInterface->IsPCIeLiteDevice())
    {
        *pValue = 1;
    }

    if (m_adqInterface->IsUSBDevice())
    {
        *pValue = 2;
    }

    if (m_adqInterface->IsUSB3Device())
    {
        *pValue = 3;
    }

    *pTimestamp = m_busTypePV.getTimestamp();
}

void ADQInfo::getPCIeLinkRate(timespec* pTimestamp, int32_t* pValue)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    if (!m_adqInterface)
        return;
    if ((m_adqInterface->IsPCIeDevice()) || (m_adqInterface->IsPCIeLiteDevice()))
    {
        *pValue = m_adqInterface->GetPCIeLinkRate();
    }

    *pTimestamp = m_pcieLinkRatePV.getTimestamp();
}

void ADQInfo::getPCIeLinkWid(timespec* pTimestamp, int32_t* pValue)
{
    if (!m_adqInterface)
        return;
    if ((m_adqInterface->IsPCIeDevice()) || (m_adqInterface->IsPCIeLiteDevice()))
    {
        *pValue = m_adqInterface->GetPCIeLinkWidth();
    }

    *pTimestamp = m_pcieLinkWidPV.getTimestamp();
}

std::string ADQInfo::utc_system_timestamp(struct timespec const& now, char sep) const
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

ADQInfo::~ADQInfo()
{
    m_adqInterface = NULL;
}
