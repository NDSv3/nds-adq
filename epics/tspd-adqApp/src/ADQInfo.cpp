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

#include "ADQAIChannel.h"
#include "ADQAIChannelGroup.h"
#include "ADQDefinition.h"
#include "ADQInfo.h"

ADQInfo::ADQInfo(const std::string& name, nds::Node& parentNode, ADQInterface* adqInterface) :
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

    // PV for sample rate
    m_sampRatePV.processAtInit(PINI);
    m_node.addChild(m_sampRatePV);

    m_sampRatePV.processAtInit(PINI);
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

ADQInfo::~ADQInfo()
{
    m_adqInterface = NULL;
}
