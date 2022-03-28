//
// Copyright (c) 2018 Cosylab d.d.
// This software is distributed under the terms found
// in file LICENSE.txt that is included with this distribution.
//

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <algorithm>
#include <time.h>
#include <limits.h>
#include <typeinfo>
#include <math.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#include <ADQAPI.h>
#include <nds3/nds.h>

#ifdef _DEBUG
#define TRACEDEBUG
#else
 //#define TRACEDEBUG
#endif

#include "ADQAIChannel.h"
#include "ADQAIChannelGroup.h"
#include "ADQDefinition.h"
#include "ADQDevice.h"

std::mutex ADQAIChannelGroup::m_StaticMutex;

// 25 ps in ms.
static const double PicoSec25 = 0.000000025;
static const double PicoSec125 = 0.000000125;

/*
 *----------------------------------------------------------------------------------------------------
 *
 **********************************************************
 *
 * module interface
 *
 **********************************************************/

ADQAIChannelGroup::ADQAIChannelGroup(const std::string& name, nds::Node& parentNode, 
    ADQInterface* adqInterface, int32_t masterEnumeration, int32_t thisEnumeration, int32_t nextEnumeration) :
    ADQInfo(name, parentNode, adqInterface), 
    m_node(nds::Port(name + GROUP_CHAN_DEVICE, nds::nodeType_t::generic)),
    m_logMsgPV(createPvRb<std::string>("LogMsg", &ADQAIChannelGroup::getLogMsg)),
    m_masterEnumerationPV(createPvRb<int32_t>("masterEnumeration", &ADQAIChannelGroup::getMasterEnumeration)),
    m_thisEnumerationPV(createPvRb<int32_t>("thisEnumeration", &ADQAIChannelGroup::getThisEnumeration)),
    m_nextEnumerationPV(createPvRb<int32_t>("nextEnumeration", &ADQAIChannelGroup::getNextEnumeration)),
    m_daisyPositionPV(createPvRb<std::vector<int32_t>>("DaisyPosition-RB", &ADQAIChannelGroup::getDaisyPosition)),
    m_sync_immediatePV(createPvRb<int32_t>("sync_immediate-RB", &ADQAIChannelGroup::getsync_immediate)),
    m_daqModePV(createPvRb<int32_t>("DAQMode-RB", &ADQAIChannelGroup::getDaqMode)),
    m_patternModePV(createPvRb<int32_t>("PatternMode-RB", &ADQAIChannelGroup::getPatternMode)),
    m_chanActivePV(createPvRb<int32_t>("ChanActive-RB", &ADQAIChannelGroup::getChanActive)),
    m_chanMaskPV(createPvRb<int32_t>("ChanMask-RB", &ADQAIChannelGroup::getChanMask)),
    m_dbsBypassPV(createPvRb<int32_t>("DBSBypass-RB", &ADQAIChannelGroup::getDbsBypass)),
    m_dbsDcPV(createPvRb<int32_t>("DBSDC-RB", &ADQAIChannelGroup::getDbsDc)),
    m_dbsLowSatPV(createPvRb<int32_t>("DBSLowSat-RB", &ADQAIChannelGroup::getDbsLowSat)),
    m_dbsUpSatPV(createPvRb<int32_t>("DBSUpSat-RB", &ADQAIChannelGroup::getDbsUpSat)),
    m_recordCntPV(createPvRb<int32_t>("RecordCnt-RB", &ADQAIChannelGroup::getRecordCnt)),
    m_recordCntCollectPV(createPvRb<int32_t>("RecordCntCollect-RB", &ADQAIChannelGroup::getRecordCntCollect)),
    m_sampleCntPV(createPvRb<int32_t>("SampCnt-RB", &ADQAIChannelGroup::getSampleCnt)),
    m_sampleCntMaxPV(createPvRb<int32_t>("SampCntMax-RB", &ADQAIChannelGroup::getSampleCntMax)),
    m_sampleCntTotalPV(createPvRb<int32_t>("SampCntTotal-RB", &ADQAIChannelGroup::getSamplesTotal)),
    m_sampleSkipPV(createPvRb<int32_t>("SampSkip-RB", &ADQAIChannelGroup::getSampleSkip)),
    m_sampleDecPV(createPvRb<int32_t>("SampDec-RB", &ADQAIChannelGroup::getSampleDec)),
    m_preTrigSampPV(createPvRb<int32_t>("PreTrigSamp-RB", &ADQAIChannelGroup::getPreTrigSamp)),
    m_trigHoldOffSampPV(createPvRb<int32_t>("TrigHoldOffSamp-RB", &ADQAIChannelGroup::getTrigHoldOffSamp)),
    m_clockSrcPV(createPvRb<int32_t>("ClockSrc-RB", &ADQAIChannelGroup::getClockSrc)),
    m_clockRefOutPV(createPvRb<int32_t>("ClockRefOut-RB", &ADQAIChannelGroup::getClockRefOut)),
    m_trigModePV(createPvRb<int32_t>("TrigMode-RB", &ADQAIChannelGroup::getTrigMode)),
    m_masterModePV(createPvRb<int32_t>("MasterMode-RB", &ADQAIChannelGroup::getMasterMode)),
    m_trigTimeStampPV(createPvRb<double>("TrigTimeStamp", &ADQAIChannelGroup::getTrigTimeStamp)),
    m_daisyRecordStartPV(createPvRb<std::vector<int32_t>>("DaisyRecordStart", &ADQAIChannelGroup::getDaisyRecordStart)),
    m_daisyTimeStampPV(createPvRb<double>("DaisyTimeStamp", &ADQAIChannelGroup::getDaisyTimeStamp)),
    m_swTrigEdgePV(createPvRb<int32_t>("SWTrigEdge-RB", &ADQAIChannelGroup::getSWTrigEdge)),
    m_levelTrigLvlPV(createPvRb<int32_t>("LevelTrigLvl-RB", &ADQAIChannelGroup::getLevelTrigLvl)),
    m_levelTrigEdgePV(createPvRb<int32_t>("LevelTrigEdge-RB", &ADQAIChannelGroup::getLevelTrigEdge)),
    m_levelTrigChanPV(createPvRb<int32_t>("LevelTrigChan-RB", &ADQAIChannelGroup::getLevelTrigChan)),
    m_levelTrigChanMaskPV(createPvRb<int32_t>("LevelTrigChanMask-RB", &ADQAIChannelGroup::getLevelTrigChanMask)),
    m_externTrigDelayPV(createPvRb<int32_t>("ExternTrigDelay-RB", &ADQAIChannelGroup::getExternTrigDelay)),
    m_externTrigThresholdPV(createPvRb<double>("ExternTrigThreshold-RB", &ADQAIChannelGroup::getExternTrigThreshold)),
    m_externTrigEdgePV(createPvRb<int32_t>("ExternTrigEdge-RB", &ADQAIChannelGroup::getExternTrigEdge)),
    m_externTrigInputImpedancePV(createPvRb<int32_t>("ExternTrigInputImpedance-RB", &ADQAIChannelGroup::getExternTrigInputImpedance)),
    m_internTrigHighSampPV(createPvRb<int32_t>("InternTrigHighSamp-RB", &ADQAIChannelGroup::getInternTrigHighSamp)),
    m_internTrigLowSampPV(createPvRb<int32_t>("InternTrigLowSamp-RB", &ADQAIChannelGroup::getInternTrigLowSamp)),
    m_internTrigFreqPV(createPvRb<int32_t>("InternTrigFreq-RB", &ADQAIChannelGroup::getInternTrigFreq)),
    m_internTrigEdgePV(createPvRb<int32_t>("InternTrigEdge-RB", &ADQAIChannelGroup::getInternTrigEdge)),
    m_timeoutPV(createPvRb<int32_t>("Timeout-RB", &ADQAIChannelGroup::getTimeout)),
    m_streamTimePV(createPvRb<double>("StreamTime-RB", &ADQAIChannelGroup::getStreamTime))
{
    parentNode.addChild(m_node);

    m_masterEnumeration = masterEnumeration;
    m_masterEnumerationPV.processAtInit(PINI);
    m_node.addChild(m_masterEnumerationPV);
    m_thisEnumeration = thisEnumeration;
    m_thisEnumerationPV.processAtInit(PINI);
    m_node.addChild(m_thisEnumerationPV);
    m_nextEnumeration = nextEnumeration;
    m_node.addChild(m_nextEnumerationPV);
    m_nextEnumerationPV.processAtInit(PINI);
    createPv<std::vector<int32_t>>("DaisyPosition", m_daisyPositionPV, &ADQAIChannelGroup::setDaisyPosition, &ADQAIChannelGroup::getDaisyPosition);
    m_daisyPositionPV.setScanType(nds::scanType_t::interrupt);

    createPv<int32_t>("sync_immediate", m_sync_immediatePV, &ADQAIChannelGroup::setsync_immediate, &ADQAIChannelGroup::getsync_immediate);
    m_sync_immediatePV.setScanType(nds::scanType_t::interrupt);

    m_chanCnt = m_adqInterface->GetNofChannels();
    m_adqType = m_adqInterface->GetADQType();

    m_daisyPositionChanged = m_daqModeChanged = m_patternModeChanged = m_dbsBypassChanged = m_dbsDcChanged = m_dbsLowSatChanged =
        m_dbsUpSatChanged = m_recordCntChanged = m_recordCntCollectChanged = m_sampleCntChanged = m_sampleSkipChanged = m_sampleDecChanged =
        m_preTrigSampChanged = m_trigHoldOffSampChanged= m_clockRefOutChanged = m_chanActiveChanged =
        m_trigModeChanged = m_masterModeChanged = m_swTrigEdgeChanged = m_levelTrigLvlChanged = m_levelTrigEdgeChanged =
        m_levelTrigChanChanged = m_levelTrigChanMaskChanged = m_externTrigDelayChanged = m_externTrigThresholdChanged = 
        m_externTrigEdgeChanged = m_externTrigInputImpedanceChanged = m_internTrigHighSampChanged = m_internTrigFreqChanged = m_internTrigEdgeChanged =
        m_timeoutChanged = m_streamTimeChanged = m_sync_immediateChanged = false;
    m_clockSrcChanged = true;
    m_daqMode = m_patternMode = m_dbsBypass = m_dbsDc = m_dbsLowSat = m_dbsUpSat = m_recordCnt = m_recordCntCollect =
        m_sampleCnt = m_sampleCntMax = m_sampleCntTotal = m_sampleSkip = m_sampleDec = m_preTrigSamp = m_trigHoldOffSamp =
        m_clockSrc = m_clockRefOut = m_chanActive = m_chanInt = m_chanMask = m_trigMode = m_masterMode = m_swTrigEdge = m_levelTrigLvl =
        m_levelTrigEdge = m_levelTrigChanMask = m_externTrigDelay = m_externTrigThreshold = m_externTrigEdge = m_externTrigInputImpedance =
        m_internTrigHighSamp = m_internTrigLowSamp = m_internTrigFreq = m_internTrigPeriod = m_internTrigEdge =
        m_timeout = m_streamTime = m_trigTimeStamp = m_PRETime = m_sync_immediate = 0;

    m_chanMask = 0xFF;
    m_recordCnt = 1;
    m_recordCntCollect = 1;
    m_sampleCnt = 1024;
    m_sampleSkip = 1;
    m_internTrigFreq = 1;
    double SampleRate = 1000000000;
    int status = m_adqInterface->GetSampleRate(0, &SampleRate);
    m_SampleRate = int64_t(SampleRate + 0.5);
    ADQNDS_MSG_WARNLOG_PV(status, "GetSampleRate failed.");
    m_internTrigPeriod = int32_t(m_SampleRate + 0.5); // 1Hz, in samples
    m_levelTrigEdge = m_internTrigEdge = 1;
    m_internTrigHighSamp = 500;
    m_internTrigLowSamp = m_internTrigPeriod - m_internTrigHighSamp;
    m_levelTrigChan = 1; // 0 is not a valid trigger channel number.
    m_DaisyChainStatus = 0;

    // Create vector of pointers to each chanel
    for (size_t channelNum(0); channelNum != m_chanCnt; ++channelNum)
    {
        std::ostringstream channelName;
        channelName << "CH" << channelNum;
        m_AIChannelsPtr.push_back(std::make_shared<ADQAIChannel>(channelName.str(), m_node, int(channelNum)));
    }

    // PV for error/warning/info messages
    m_logMsgPV.setScanType(nds::scanType_t::interrupt);
    m_logMsgPV.setMaxElements(512);
    m_node.addChild(m_logMsgPV);

    // PV for data acquisition modes
    nds::enumerationStrings_t daqModeList = { "Multi-Record", "Continuous stream", "Triggered stream", "Raw stream" };
    createPvEnum<int32_t>("DAQMode", m_daqModePV, daqModeList, &ADQAIChannelGroup::setDaqMode, &ADQAIChannelGroup::getDaqMode);

    // PV for Pattern Mode
    nds::enumerationStrings_t patternModeList = { "Normal", "Count upwards", "Count downwards", "Alter ups & downs" };
    createPvEnum<int32_t>("PatternMode", m_patternModePV, patternModeList, &ADQAIChannelGroup::setPatternMode, &ADQAIChannelGroup::getPatternMode);

    // PVs for setting active channels and channel mask
    nds::enumerationStrings_t chanMaskList;
    if (m_chanCnt == 8)
    {
        chanMaskList = { "A", "B", "C", "D", "E", "F", "G", "H", "A+B+C+D", "E+F+G+H", "A+B+C+D+E+F+G+H" };
        m_chanActive = 10;
        m_chanMask = 0XFF;
    }
    if (m_chanCnt == 4)
    {
        chanMaskList = { "A", "B", "C", "D", "A+B", "C+D", "A+B+C+D" };
        m_chanActive = 6;
        m_chanMask = 0X0F;
    }
    if (m_chanCnt == 2)
    {
        chanMaskList = { "A", "B", "A+B" };
        m_chanActive = 2;
        m_chanMask = 0X03;
    }
    if (m_chanCnt == 1)
    {
        chanMaskList = { "A" };
        m_chanActive = 1;
        m_chanMask = 0X01;
    }
    createPvEnum<int32_t>("ChanActive", m_chanActivePV, chanMaskList, &ADQAIChannelGroup::setChanActive, &ADQAIChannelGroup::getChanActive);

    m_chanMaskPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_chanMaskPV);

    // PVs for DBS setup
    createPv<int32_t>("DBSBypass", m_dbsBypassPV, &ADQAIChannelGroup::setDbsBypass, &ADQAIChannelGroup::getDbsBypass);
    createPv<int32_t>("DBSDC", m_dbsDcPV, &ADQAIChannelGroup::setDbsDc, &ADQAIChannelGroup::getDbsDc);
    createPv<int32_t>("DBSLowSat", m_dbsLowSatPV, &ADQAIChannelGroup::setDbsLowSat, &ADQAIChannelGroup::getDbsLowSat);
    createPv<int32_t>("DBSUpSat", m_dbsUpSatPV, &ADQAIChannelGroup::setDbsUpSat, &ADQAIChannelGroup::getDbsUpSat);

    // PVs for records
    createPv<int32_t>("RecordCnt", m_recordCntPV, &ADQAIChannelGroup::setRecordCnt, &ADQAIChannelGroup::getRecordCnt);
    createPv<int32_t>("RecordCntCollect", m_recordCntCollectPV, &ADQAIChannelGroup::setRecordCntCollect, &ADQAIChannelGroup::getRecordCntCollect);

    // PVs for samples
    createPv<int32_t>("SampCnt", m_sampleCntPV, &ADQAIChannelGroup::setSampleCnt, &ADQAIChannelGroup::getSampleCnt);

    m_sampleCntMaxPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_sampleCntMaxPV);

    m_sampleCntTotalPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_sampleCntTotalPV);

    createPv<int32_t>("SampSkip", m_sampleSkipPV, &ADQAIChannelGroup::setSampleSkip, &ADQAIChannelGroup::getSampleSkip);
    createPv<int32_t>("SampDec", m_sampleDecPV, &ADQAIChannelGroup::setSampleDec, &ADQAIChannelGroup::getSampleDec);
    createPv<int32_t>("PreTrigSamp", m_preTrigSampPV, &ADQAIChannelGroup::setPreTrigSamp, &ADQAIChannelGroup::getPreTrigSamp);
    createPv<int32_t>("TrigHoldOffSamp", m_trigHoldOffSampPV, &ADQAIChannelGroup::setTrigHoldOffSamp, &ADQAIChannelGroup::getTrigHoldOffSamp);

    // PVs for clock
    createPv<int32_t>("ClockSrc", m_clockSrcPV, &ADQAIChannelGroup::setClockSrc, &ADQAIChannelGroup::getClockSrc);
    createPv<int32_t>("ClockRefOut", m_clockRefOutPV, &ADQAIChannelGroup::setClockRefOut, &ADQAIChannelGroup::getClockRefOut);

    // PV for Trigger Mode
    nds::enumerationStrings_t trigModeList = { "Software", "External", "Level", "Internal", "Daisy chain" };
    createPvEnum<int32_t>("TrigMode", m_trigModePV, trigModeList, &ADQAIChannelGroup::setTrigMode, &ADQAIChannelGroup::getTrigMode);

    // PV for daisy chain Master Mode
    nds::enumerationStrings_t masterModeList = { "None", "Master", "Slave"};
    createPvEnum<int32_t>("MasterMode", m_masterModePV, masterModeList, &ADQAIChannelGroup::setMasterMode, &ADQAIChannelGroup::getMasterMode);

    // PVs for the trigger time stamp and the daisy time stamp(s).
    m_trigTimeStampPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_trigTimeStampPV);
    m_daisyRecordStartPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_daisyRecordStartPV);
    m_daisyTimeStampPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_daisyTimeStampPV);

    // PV for level trigger level
    createPv<int32_t>("LevelTrigLvl", m_levelTrigLvlPV, &ADQAIChannelGroup::setLevelTrigLvl, &ADQAIChannelGroup::getLevelTrigLvl);

    // PV for level trigger edge
    nds::enumerationStrings_t triggerEdgeList = { "Falling edge", "Rising edge", "Both edges" };
    createPvEnum<int32_t>("LevelTrigEdge", m_levelTrigEdgePV, triggerEdgeList, &ADQAIChannelGroup::setLevelTrigEdge, &ADQAIChannelGroup::getLevelTrigEdge);

    // PV for level trigger channel
    nds::enumerationStrings_t trigChanList = { "None", "A", "B", "C", "D", "A+B", "C+D", "A+B+C+D" };
    createPvEnum<int32_t>("LevelTrigChan", m_levelTrigChanPV, trigChanList, &ADQAIChannelGroup::setLevelTrigChan, &ADQAIChannelGroup::getLevelTrigChan);

    createPv<int32_t>("LevelTrigChanMask", m_levelTrigChanMaskPV, &ADQAIChannelGroup::setLevelTrigChanMask, &ADQAIChannelGroup::getLevelTrigChanMask);
    m_levelTrigChanMaskPV.setScanType(nds::scanType_t::interrupt);

    // PV for SW trigger edge
    createPvEnum<int32_t>("SWTrigEdge", m_swTrigEdgePV, triggerEdgeList, &ADQAIChannelGroup::setSWTrigEdge, &ADQAIChannelGroup::getSWTrigEdge);

    // PV for external trigger delay
    createPv<int32_t>("ExternTrigDelay", m_externTrigDelayPV, &ADQAIChannelGroup::setExternTrigDelay, &ADQAIChannelGroup::getExternTrigDelay);

    // PV for external trigger threshold
    createPv<double>("ExternTrigThreshold", m_externTrigThresholdPV, &ADQAIChannelGroup::setExternTrigThreshold, &ADQAIChannelGroup::getExternTrigThreshold);

    // PV for external trigger edge
    createPvEnum<int32_t>("ExternTrigEdge", m_externTrigEdgePV, triggerEdgeList, &ADQAIChannelGroup::setExternTrigEdge, &ADQAIChannelGroup::getExternTrigEdge);

    createPv<int32_t>("ExternTrigInputImpedance", m_externTrigInputImpedancePV, &ADQAIChannelGroup::setExternTrigInputImpedance, &ADQAIChannelGroup::getExternTrigInputImpedance);

    // PV for internal trigger high samples length
    createPv<int32_t>("InternTrigHighSamp", m_internTrigHighSampPV, &ADQAIChannelGroup::setInternTrigHighSamp, &ADQAIChannelGroup::getInternTrigHighSamp);
    // No independant writable PV for low internal trigger samples length. But I/O Intr.
    m_internTrigLowSampPV.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(m_internTrigLowSampPV);

    // PV for trigger frequency
    createPv<int32_t>("InternTrigFreq", m_internTrigFreqPV, &ADQAIChannelGroup::setInternTrigFreq, &ADQAIChannelGroup::getInternTrigFreq);

    // PV for internal trigger edge
    createPvEnum<int32_t>("InternTrigEdge", m_internTrigEdgePV, triggerEdgeList, &ADQAIChannelGroup::setInternTrigEdge, &ADQAIChannelGroup::getInternTrigEdge);

    // PV for flush timeout
    createPv<int32_t>("Timeout", m_timeoutPV, &ADQAIChannelGroup::setTimeout, &ADQAIChannelGroup::getTimeout);

    // PV for streaming time
    createPv<double>("StreamTime", m_streamTimePV, &ADQAIChannelGroup::setStreamTime, &ADQAIChannelGroup::getStreamTime);

    setTriggerIdleState();
    status = m_adqInterface->SetTriggerMode(1); // SW trigger
    ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerMode failed.");

    // PV for state machine
    m_stateMachine = m_node.addChild(nds::StateMachine(true,
                                                       std::bind(&ADQAIChannelGroup::onSwitchOn, this),
                                                       std::bind(&ADQAIChannelGroup::onSwitchOff, this),
                                                       std::bind(&ADQAIChannelGroup::onStart, this),
                                                       std::bind(&ADQAIChannelGroup::onStop, this),
                                                       std::bind(&ADQAIChannelGroup::recover, this),
                                                       std::bind(&ADQAIChannelGroup::allowChange,
                                                                 this,
                                                                 std::placeholders::_1,
                                                                 std::placeholders::_2,
                                                                 std::placeholders::_3)));

        commitChanges();
}

/* This function creates the most common type of PV and sets it readback PV to interrupt mode
 */
template <typename T>
void ADQAIChannelGroup::createPv(const std::string& name, nds::PVDelegateIn<T>& pvRb,
                                 std::function<void(ADQAIChannelGroup*, const timespec&, const T&)> setter,
                                 std::function<void(ADQAIChannelGroup*, timespec*, T*)> getter)
{
    nds::PVDelegateOut<T> node(name,
                               std::bind(setter, this, std::placeholders::_1, std::placeholders::_2),
                               std::bind(getter, this, std::placeholders::_1, std::placeholders::_2));
    m_node.addChild(node);
    pvRb.setScanType(nds::scanType_t::interrupt);
    m_node.addChild(pvRb);
}

/* This function creates the Enumeration type of PV and sets it readback PV to interrupt mode
 */
template <typename T>
void ADQAIChannelGroup::createPvEnum(const std::string& name, nds::PVDelegateIn<T>& pvRb, nds::enumerationStrings_t enumList,
                                     std::function<void(ADQAIChannelGroup*, const timespec&, const T&)> setter,
                                     std::function<void(ADQAIChannelGroup*, timespec*, T*)> getter)
{
    nds::PVDelegateOut<T> node(name,
                               std::bind(setter, this, std::placeholders::_1, std::placeholders::_2),
                               std::bind(getter, this, std::placeholders::_1, std::placeholders::_2));
    node.setEnumeration(enumList);
    m_node.addChild(node);
    pvRb.setScanType(nds::scanType_t::interrupt);
    pvRb.setEnumeration(enumList);
    m_node.addChild(pvRb);
}

/* This function creates and returns the readback PV
 */
template <typename T>
nds::PVDelegateIn<T> ADQAIChannelGroup::createPvRb(const std::string& name,
                                                   std::function<void(ADQAIChannelGroup*, timespec*, T*)> getter)
{
    return nds::PVDelegateIn<T>(name, std::bind(getter, this, std::placeholders::_1, std::placeholders::_2));
}

void ADQAIChannelGroup::getLogMsg(timespec* pTimestamp, std::string* pValue)
{
    *pValue = m_logMsg;
    *pTimestamp = m_logMsgPV.getTimestamp();
}

void ADQAIChannelGroup::setLogMsg(const timespec& pTimestamp, std::string const& pValue)
{
    m_logMsg = pValue;
    m_logMsgPV.push(pTimestamp, m_logMsg);
}

void ADQAIChannelGroup::getMasterEnumeration(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_masterEnumeration;
    *pTimestamp = m_masterEnumerationPV.getTimestamp();
}

void ADQAIChannelGroup::getThisEnumeration(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_thisEnumeration;
    *pTimestamp = m_thisEnumerationPV.getTimestamp();
}

void ADQAIChannelGroup::getNextEnumeration(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_nextEnumeration;
    *pTimestamp = m_nextEnumerationPV.getTimestamp();
}

void ADQAIChannelGroup::setDaisyPosition(const timespec& pTimestamp, const std::vector<int32_t>& pValue)
{
    m_daisyPosition = pValue;
    m_daisyPositionChanged = true;
    m_daisyPositionPV.getTimestamp() = pTimestamp;
    commitChanges();
}

void ADQAIChannelGroup::getDaisyPosition(timespec* pTimestamp, std::vector<int32_t>* pValue)
{
    *pValue = m_daisyPosition;
    *pTimestamp = m_daisyPositionPV.getTimestamp();
}

void ADQAIChannelGroup::setsync_immediate(const timespec& pTimestamp, const int32_t& pValue)
{
    m_sync_immediate = pValue;
    m_sync_immediateChanged = true;
    m_sync_immediatePV.getTimestamp() = pTimestamp;
    commitChanges();
}

void ADQAIChannelGroup::getsync_immediate(timespec* pTimestamp, int32_t *pValue)
{
    *pValue = m_sync_immediate;
    *pTimestamp = m_sync_immediatePV.getTimestamp();
}

void ADQAIChannelGroup::setDaqMode(const timespec& pTimestamp, const int32_t& pValue)
{
    m_daqMode = pValue;
    m_daqModePV.getTimestamp() = pTimestamp;
    m_daqModeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getDaqMode(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_daqMode;
    *pTimestamp = m_daqModePV.getTimestamp();
}

void ADQAIChannelGroup::setPatternMode(const timespec& pTimestamp, const int32_t& pValue)
{
    m_patternMode = pValue;
    m_patternModePV.getTimestamp() = pTimestamp;
    m_patternModeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getPatternMode(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_patternMode;
    *pTimestamp = m_patternModePV.getTimestamp();
}

void ADQAIChannelGroup::setChanActive(const timespec& pTimestamp, const int32_t& pValue)
{
    m_chanActive = pValue;
    m_chanActivePV.getTimestamp() = pTimestamp;
    m_chanActiveChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getChanActive(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_chanActive;
    *pTimestamp = m_chanActivePV.getTimestamp();
}

void ADQAIChannelGroup::getChanMask(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_chanMask;
    *pTimestamp = m_chanMaskPV.getTimestamp();
}

void ADQAIChannelGroup::setDbsBypass(const timespec& pTimestamp, const int32_t& pValue)
{
    m_dbsBypass = pValue;
    m_dbsBypassPV.getTimestamp() = pTimestamp;
    m_dbsBypassChanged = (m_dbsBypass != pValue);
    commitChanges();
}

void ADQAIChannelGroup::getDbsBypass(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_dbsBypass;
    *pTimestamp = m_dbsBypassPV.getTimestamp();
}

void ADQAIChannelGroup::setDbsDc(const timespec& pTimestamp, const int32_t& pValue)
{
    m_dbsDc = pValue;
    m_dbsDcPV.getTimestamp() = pTimestamp;
    m_dbsDcChanged = (m_dbsDc != pValue);
    commitChanges();
}

void ADQAIChannelGroup::getDbsDc(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_dbsDc;
    *pTimestamp = m_dbsDcPV.getTimestamp();
}

void ADQAIChannelGroup::setDbsLowSat(const timespec& pTimestamp, const int32_t& pValue)
{
    m_dbsLowSat = pValue;
    m_dbsLowSatPV.getTimestamp() = pTimestamp;
    m_dbsLowSatChanged = (m_dbsLowSat != pValue);
    commitChanges();
}

void ADQAIChannelGroup::getDbsLowSat(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_dbsLowSat;
    *pTimestamp = m_dbsLowSatPV.getTimestamp();
}

void ADQAIChannelGroup::setDbsUpSat(const timespec& pTimestamp, const int32_t& pValue)
{
    m_dbsUpSat = pValue;
    m_dbsUpSatPV.getTimestamp() = pTimestamp;
    m_dbsUpSatChanged = (m_dbsUpSat != pValue);
    commitChanges();
}

void ADQAIChannelGroup::getDbsUpSat(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_dbsUpSat;
    *pTimestamp = m_dbsUpSatPV.getTimestamp();
}

void ADQAIChannelGroup::setRecordCnt(const timespec& pTimestamp, const int32_t& pValue)
{
    m_recordCnt = pValue;
    m_recordCntPV.getTimestamp() = pTimestamp;
    m_recordCntChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getRecordCnt(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_recordCnt;
    *pTimestamp = m_recordCntPV.getTimestamp();
}

void ADQAIChannelGroup::setRecordCntCollect(const timespec& pTimestamp, const int32_t& pValue)
{
    m_recordCntCollect = pValue;
    m_recordCntCollectPV.getTimestamp() = pTimestamp;
    m_recordCntCollectChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getRecordCntCollect(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_recordCntCollect;
    *pTimestamp = m_recordCntCollectPV.getTimestamp();
}

void ADQAIChannelGroup::setSampleCnt(const timespec& pTimestamp, const int32_t& pValue)
{
    m_sampleCnt = pValue;
    m_sampleCntPV.getTimestamp() = pTimestamp;
    m_sampleCntChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getSampleCnt(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_sampleCnt;
    *pTimestamp = m_sampleCntPV.getTimestamp();
}

void ADQAIChannelGroup::getSampleCntMax(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_sampleCntMax;
    *pTimestamp = m_sampleCntMaxPV.getTimestamp();
}

void ADQAIChannelGroup::getSamplesTotal(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_sampleCntTotal;
    *pTimestamp = m_sampleCntTotalPV.getTimestamp();
}

void ADQAIChannelGroup::setSampleSkip(const timespec& pTimestamp, const int32_t& pValue)
{
    m_sampleSkip = pValue;
    m_sampleSkipPV.getTimestamp() = pTimestamp;
    m_sampleSkipChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getSampleSkip(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_sampleSkip;
    *pTimestamp = m_sampleSkipPV.getTimestamp();
}

void ADQAIChannelGroup::setSampleDec(const timespec& pTimestamp, const int32_t& pValue)
{
    m_sampleDec = pValue;
    m_sampleDecPV.getTimestamp() = pTimestamp;
    // ADQ8 doesn't support decimation, so no need to push it.
    std::string adqOption = m_adqInterface->GetCardOption();
    m_sampleDecChanged = ((m_adqType == 714 || m_adqType == 14) && (adqOption.find("-FWSDR") != std::string::npos));
    commitChanges();
}

void ADQAIChannelGroup::getSampleDec(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_sampleDec;
    *pTimestamp = m_sampleDecPV.getTimestamp();
}

void ADQAIChannelGroup::setPreTrigSamp(const timespec& pTimestamp, const int32_t& pValue)
{
    m_preTrigSamp = pValue;
    m_preTrigSampPV.getTimestamp() = pTimestamp;
    m_preTrigSampChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getPreTrigSamp(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_preTrigSamp;
    *pTimestamp = m_preTrigSampPV.getTimestamp();
}

void ADQAIChannelGroup::setTrigHoldOffSamp(const timespec& pTimestamp, const int32_t& pValue)
{
    m_trigHoldOffSamp = pValue;
    m_trigHoldOffSampPV.getTimestamp() = pTimestamp;
    m_trigHoldOffSampChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getTrigHoldOffSamp(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_trigHoldOffSamp;
    *pTimestamp = m_trigHoldOffSampPV.getTimestamp();
}

void ADQAIChannelGroup::setClockSrc(const timespec& pTimestamp, const int32_t& pValue)
{
    m_clockSrc = pValue;
    m_clockSrcPV.getTimestamp() = pTimestamp;
    m_clockSrcChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getClockSrc(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_clockSrc;
    *pTimestamp = m_clockSrcPV.getTimestamp();
}

void ADQAIChannelGroup::setClockRefOut(const timespec& pTimestamp, const int32_t& pValue)
{
    m_clockRefOut = pValue;
    m_clockRefOutPV.getTimestamp() = pTimestamp;
    m_clockRefOutChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getClockRefOut(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_clockRefOut;
    *pTimestamp = m_clockRefOutPV.getTimestamp();
}

void ADQAIChannelGroup::setTrigMode(const timespec& pTimestamp, const int32_t& pValue)
{
    m_trigMode = pValue;
    m_trigModePV.getTimestamp() = pTimestamp;
    m_trigModeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getTrigMode(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_trigMode;
    *pTimestamp = m_trigModePV.getTimestamp();
}

void ADQAIChannelGroup::setExternTrigInputImpedance(const timespec& pTimestamp, const int32_t& pValue)
{
    m_externTrigInputImpedanceChanged = true;
    m_externTrigInputImpedance = pValue;
    m_externTrigInputImpedancePV.getTimestamp() = pTimestamp;
    commitChanges();
}

void ADQAIChannelGroup::getExternTrigInputImpedance(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_externTrigInputImpedance;
    *pTimestamp = m_externTrigInputImpedancePV.getTimestamp();
}

void ADQAIChannelGroup::setMasterMode(const timespec& pTimestamp, const int32_t& pValue)
{
    m_masterMode = pValue;
    m_masterModeChanged = true;
    m_masterModePV.getTimestamp() = pTimestamp;
    commitChanges();
}

void ADQAIChannelGroup::getMasterMode(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_masterMode;
    *pTimestamp = m_masterModePV.getTimestamp();
}

void ADQAIChannelGroup::getTrigTimeStamp(timespec* pTimestamp, double* pValue)
{
    *pValue = m_trigTimeStamp;
    *pTimestamp = m_trigTimeStampPV.getTimestamp();
}

void ADQAIChannelGroup::getDaisyRecordStart(timespec* pTimestamp, std::vector<int32_t>* pValue)
{
    *pValue = m_daisyRecordStart;
    *pTimestamp = m_daisyRecordStartPV.getTimestamp();
}

void ADQAIChannelGroup::getDaisyTimeStamp(timespec* pTimestamp, double* pValue)
{
    *pValue = m_daisyTimeStamp;
    *pTimestamp = m_daisyTimeStampPV.getTimestamp();
}

void ADQAIChannelGroup::setSWTrigEdge(const timespec& pTimestamp, const int32_t& pValue)
{
    m_swTrigEdge = pValue;
    m_swTrigEdgePV.getTimestamp() = pTimestamp;
    m_swTrigEdgeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getSWTrigEdge(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_swTrigEdge;
    *pTimestamp = m_swTrigEdgePV.getTimestamp();
}

void ADQAIChannelGroup::setLevelTrigLvl(const timespec& pTimestamp, const int32_t& pValue)
{
    m_levelTrigLvl = pValue;
    m_levelTrigLvlPV.getTimestamp() = pTimestamp;
    m_levelTrigLvlChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getLevelTrigLvl(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_levelTrigLvl;
    *pTimestamp = m_levelTrigLvlPV.getTimestamp();
}

void ADQAIChannelGroup::setLevelTrigEdge(const timespec& pTimestamp, const int32_t& pValue)
{
    m_levelTrigEdge = pValue;
    m_levelTrigEdgePV.getTimestamp() = pTimestamp;
    m_levelTrigEdgeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getLevelTrigEdge(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_levelTrigEdge;
    *pTimestamp = m_levelTrigEdgePV.getTimestamp();
}

void ADQAIChannelGroup::setLevelTrigChan(const timespec& pTimestamp, const int32_t& pValue)
{
    m_levelTrigChan = pValue;
    m_levelTrigChanPV.getTimestamp() = pTimestamp;
    m_levelTrigChanChanged = (pValue != m_levelTrigChan);
    commitChanges();
}

void ADQAIChannelGroup::getLevelTrigChan(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_levelTrigChan;
    *pTimestamp = m_levelTrigChanPV.getTimestamp();
}

void ADQAIChannelGroup::setLevelTrigChanMask(const timespec& pTimestamp, const int32_t& pValue)
{
    m_levelTrigChanMask = pValue;
    m_levelTrigChanMaskPV.getTimestamp() = pTimestamp;
    m_levelTrigChanMaskChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getLevelTrigChanMask(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_levelTrigChanMask;
    *pTimestamp = m_levelTrigChanMaskPV.getTimestamp();
}

void ADQAIChannelGroup::setExternTrigDelay(const timespec& pTimestamp, const int32_t& pValue)
{
    m_externTrigDelay = pValue;
    m_externTrigDelayPV.getTimestamp() = pTimestamp;
    m_externTrigDelayChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getExternTrigDelay(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_externTrigDelay;
    *pTimestamp = m_externTrigDelayPV.getTimestamp();
}

void ADQAIChannelGroup::setExternTrigThreshold(const timespec& pTimestamp, const double& pValue)
{
    m_externTrigThreshold = pValue;
    m_externTrigThresholdPV.getTimestamp() = pTimestamp;
    m_externTrigThresholdChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getExternTrigThreshold(timespec* pTimestamp, double* pValue)
{
    *pValue = m_externTrigThreshold;
    *pTimestamp = m_externTrigThresholdPV.getTimestamp();
}

void ADQAIChannelGroup::setExternTrigEdge(const timespec& pTimestamp, const int32_t& pValue)
{
    m_externTrigEdge = pValue;
    m_externTrigEdgePV.getTimestamp() = pTimestamp;
    m_externTrigEdgeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getExternTrigEdge(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_externTrigEdge;
    *pTimestamp = m_externTrigEdgePV.getTimestamp();
}

void ADQAIChannelGroup::setInternTrigHighSamp(const timespec& pTimestamp, const int32_t& pValue)
{
    m_internTrigHighSamp = pValue;
    m_internTrigHighSampPV.getTimestamp() = pTimestamp;
    m_internTrigHighSampChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getInternTrigHighSamp(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_internTrigHighSamp;
    *pTimestamp = m_internTrigHighSampPV.getTimestamp();
}

void ADQAIChannelGroup::getInternTrigLowSamp(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_internTrigLowSamp;
    *pTimestamp = m_internTrigLowSampPV.getTimestamp();
}

void ADQAIChannelGroup::setInternTrigFreq(const timespec& pTimestamp, const int32_t& pValue)
{
    m_internTrigFreq = pValue;
    m_internTrigFreqPV.getTimestamp() = pTimestamp;
    m_internTrigFreqChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::setInternTrigEdge(const timespec& pTimestamp, const int32_t& pValue)
{
    m_internTrigEdge = pValue;
    m_internTrigEdgePV.getTimestamp() = pTimestamp;
    m_internTrigEdgeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getInternTrigEdge(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_internTrigEdge;
    *pTimestamp = m_internTrigEdgePV.getTimestamp();
}

void ADQAIChannelGroup::getInternTrigFreq(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_internTrigFreq;
    *pTimestamp = m_internTrigFreqPV.getTimestamp();
}

void ADQAIChannelGroup::setTimeout(const timespec& pTimestamp, const int32_t& pValue)
{
    m_timeout = pValue;
    m_timeoutPV.getTimestamp() = pTimestamp;
    m_timeoutChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getTimeout(timespec* pTimestamp, int32_t* pValue)
{
    *pValue = m_timeout;
    *pTimestamp = m_timeoutPV.getTimestamp();
}

void ADQAIChannelGroup::setStreamTime(const timespec& pTimestamp, const double& pValue)
{
    m_streamTime = pValue;
    m_streamTimePV.getTimestamp() = pTimestamp;
    m_streamTimeChanged = true;
    commitChanges();
}

void ADQAIChannelGroup::getStreamTime(timespec* pTimestamp, double* pValue)
{
    *pValue = m_streamTime;
    *pTimestamp = m_streamTimePV.getTimestamp();
}

/* This function updates readback PVs according to changed ADQ parameters.
 * It also sets trigger options with ADQ API functions.
 */
void ADQAIChannelGroup::commitChanges(bool calledFromDaqThread)
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    struct timespec now = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &now);

    try
    {
        /* Changes to parameters are allowed when device is ON/STOPPING/INITIALISING states.
     * Changes are not applied when device is on acquisition state.
     */
     // Check changes on channels
        for (auto const& channel : m_AIChannelsPtr)
        {
            channel->commitChanges(calledFromDaqThread, m_adqInterface, m_logMsgPV);
        }

        nds::state_t LocalState = m_stateMachine.getLocalState();
        if (!calledFromDaqThread &&
            (LocalState != nds::state_t::on && LocalState != nds::state_t::stopping &&
                LocalState != nds::state_t::initializing))
        {
            return;
        }

        if (m_daqModeChanged)
            commitDaqMode(now);

        if (m_patternModeChanged)
            commitPatternMode(now);

        if (m_clockSrcChanged)
            commitClockSource(now);

        if (m_trigModeChanged)
            commitTriggerMode(now);

        if ((m_masterModeChanged) || (m_sync_immediateChanged))
            commitDaisyChain(now);

        if (m_chanActiveChanged)
            commitActiveChan(now);

        if (m_dbsBypassChanged || m_dbsDcChanged || m_dbsLowSatChanged || m_dbsUpSatChanged)
            commitSetupDBS(now);

        if (m_recordCntChanged || m_sampleCntChanged)
            commitRecordCount(now);

        if (m_recordCntCollectChanged && (m_daqMode == 0))   // Multi-Record
            commitRecordCountCollect(now);

        if (m_streamTimeChanged && (m_daqMode == 1))   // Continuous streaming
            commitStreamTime(now);

        if (m_sampleSkipChanged)
            commitSampleSkip(now);

        if (m_sampleDecChanged)
            commitSampleDec(now);

        if ((m_preTrigSampChanged) || (m_trigHoldOffSampChanged) || (m_daisyPositionChanged))
            commitTriggerDelayOrHoldoff(now);

        if ((m_swTrigEdgeChanged) && (m_trigMode == 0))
            commitSWTrigEdge(now);

        if ((m_externTrigDelayChanged) && (m_trigMode == 1))   // External trigger
            commitExternTrigDelay(now);

        if ((m_externTrigThresholdChanged) && (m_trigMode == 1))
            commitExternTrigThreshold(now);

        if ((m_externTrigEdgeChanged) && (m_trigMode == 1)) // External trigger
            commitExternTrigEdge(now);

        if ((m_externTrigInputImpedanceChanged) && (m_trigMode == 1))
            commitExternTrigImpedance(now);

        if (m_clockRefOutChanged)
            commitClockRefOut(now);

        if ((m_trigMode == 2) && (m_levelTrigEdgeChanged))   // Level trigger
            commitLevelTrigEdge(now);

        if ((m_trigMode == 2) && (m_levelTrigLvlChanged))   // Level trigger
            commitLevelTrigLvl(now);

        if ((m_trigMode == 2) && (m_levelTrigChanChanged || m_levelTrigChanMaskChanged))
            commitLevelTrigChanMask(now);

        if ((m_internTrigHighSampChanged) && (m_trigMode == 3))  // Internal trigger
            commitInternTrigHighSamp(now);

        if ((m_internTrigFreqChanged) && (m_trigMode == 3)) // Internal trigger
            commitInternTrigFreqChanged(now);

        if ((m_internTrigEdgeChanged) && (m_trigMode == 3)) // Internal trigger
            commitInternTrigEdge(now);

        if (m_timeoutChanged)
        {
            m_timeoutChanged = false;
            if (m_timeout <= 0)
                m_timeout = 1000;
            m_timeoutPV.push(now, m_timeout);
        }
    }
    catch (nds::NdsError const&) {
    }
}

/* Sets the device and its channels to state ON. Allows to apply changes to PVs.
 */
void ADQAIChannelGroup::onSwitchOn()
{
    TraceOutWithTime(m_node, "onSwitchOn called");
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    setLogMsg(now, std::string("")); // Clear any previous log messages.

    // Enable all channels
    for (auto const& channel : m_AIChannelsPtr)
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        channel->setState(nds::state_t::on);
    }
    commitChanges(true);
}

/* Sets the device and its channels to state OFF. Disables applying changes to PVs.
 */
void ADQAIChannelGroup::onSwitchOff()
{
    TraceOutWithTime(m_node, "onSwitchOff called");
    commitChanges();

    // Disable all channels
    for (auto const& channel : m_AIChannelsPtr)
    {
        channel->setState(nds::state_t::off);
    }
}

/* Sets the device and its channels to state RUNNING. 
 * Starts a chosen data acquisition in a thread.
 */
void ADQAIChannelGroup::onStart()
{
    TraceOutWithTime(m_node, "onStart called");
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    setLogMsg(now, std::string("")); // Clear any previous log messages.
    for (auto const& channel : m_AIChannelsPtr)
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        channel->setState(nds::state_t::running);
    }

    if (m_daqMode == 0)
        m_daqThread = m_node.runInThread("DAQ-MultiRecord", std::bind(&ADQAIChannelGroup::daqMultiRecord, this));
    if (m_daqMode == 1)
        m_daqThread = m_node.runInThread("DAQ-ContinStream", std::bind(&ADQAIChannelGroup::daqContinStream, this));
    if (m_daqMode == 2)
        m_daqThread = m_node.runInThread("DAQ-TrigStream", std::bind(&ADQAIChannelGroup::daqTrigStream, this));
    if (m_daqMode == 3)
        m_daqThread = m_node.runInThread("DAQ-RawStream", std::bind(&ADQAIChannelGroup::daqRawStream, this));
}

/* Sets the device and its channels to state ON.
 * Stops data acquisition and joins DAQ thread.
 */
void ADQAIChannelGroup::onStop()
{
    TraceOutWithTime(m_node, "onStop called");
    m_daqThread.join();
    commitChanges();

    for (auto const& channel : m_AIChannelsPtr)
    {
        channel->setState(nds::state_t::on);
    }
}

/* Not supported.
 */
void ADQAIChannelGroup::recover()
{
    throw nds::StateMachineRollBack("INFO: Cannot recover");
}

/* Allows changing the device's state.
 */
bool ADQAIChannelGroup::allowChange(const nds::state_t currentLocal, const nds::state_t currentGlobal, const nds::state_t nextLocal)
{
    UNUSED(currentLocal);
    UNUSED(currentGlobal);
    UNUSED(nextLocal);
    return true;
}

/* Time snap of the starting time, used in Triggered and Continuous Streaming.
 */
static struct timespec tsref;
void timerStart(void)
{
    if (clock_gettime(CLOCK_REALTIME, &tsref) < 0)
    {
        std::cout << "Failed to start timer." << std::endl;
        return;
    }
}

/* Returns time in ms that is spend since the timerStart, used in Triggered and Continuous Streaming.
 */
unsigned int timerSpentTimeMs(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) < 0)
    {
        std::cout << "Failed to get system time." << std::endl;
        return -1;
    }
    return (unsigned int)((int)(ts.tv_sec - tsref.tv_sec) * 1000 + (int)(ts.tv_nsec - tsref.tv_nsec) / 1000000);
}

int ADQAIChannelGroup::allocateBuffers(short* (&daqDataBuffer)[CHANNEL_COUNT_MAX], streamingHeader_t* (&daqStreamHeaders)[CHANNEL_COUNT_MAX], short* (*daqLeftoverSamples)[CHANNEL_COUNT_MAX])
{
    unsigned int bufferSize = m_sampleCnt * m_recordCntCollect * sizeof(short);
    unsigned int stream_chunk_bytes = 512;
    if (m_adqType == 714 || m_adqType == 14)
        stream_chunk_bytes = ADQ14_STREAM_CHUNK_BYTES;
    if (m_adqType == 7)
        stream_chunk_bytes = ADQ7_STREAM_CHUNK_BYTES;
    if (m_adqType == 8)
        stream_chunk_bytes = ADQ7_STREAM_CHUNK_BYTES;
    bufferSize = stream_chunk_bytes * ((bufferSize + stream_chunk_bytes - 1) / stream_chunk_bytes);

    for (unsigned int chan = 0; chan < CHANNEL_COUNT_MAX; ++chan)
    {
        daqDataBuffer[chan] = NULL;
        daqStreamHeaders[chan] = NULL;
        if (daqLeftoverSamples != NULL)
            (*daqLeftoverSamples)[chan] = NULL;
        if (!((1 << chan) & m_chanMask))
            continue;

        daqDataBuffer[chan] = (short*)malloc(bufferSize);
        if (!daqDataBuffer[chan])
        {
            bufferSize = 0;
            ADQNDS_MSG_ERRLOG_PV(bufferSize, "Failed to allocate memory for target buffers.");
        }

        daqStreamHeaders[chan] = (streamingHeader_t*)malloc(sizeof(streamingHeader_t)* m_recordCnt);
        if (!daqStreamHeaders[chan])
        {
            bufferSize = 0;
            ADQNDS_MSG_ERRLOG_PV(bufferSize, "Failed to allocate memory for target headers.");
        }

        if (daqLeftoverSamples != NULL)
        {
            (*daqLeftoverSamples)[chan] = (short int*)malloc(sizeof(short) * m_sampleCnt * 2);
            if (!(*daqLeftoverSamples)[chan])
            {
                bufferSize = 0;
                ADQNDS_MSG_ERRLOG_PV(bufferSize, "Failed to allocate memory for target extradata.");
            }
        }
    }
    return bufferSize;
}

/* Data acquisition method for triggered streaming
 */
void ADQAIChannelGroup::daqTrigStream()
{
    int status = 0;
    int channelCount = 0;
    unsigned int buffersFilled;
    int recordsDoneCnt[CHANNEL_COUNT_MAX];
    unsigned int samplesAddedCnt[CHANNEL_COUNT_MAX];
    unsigned int headersAdded[CHANNEL_COUNT_MAX];
    unsigned int samplesLeftoverSamplesCnt[CHANNEL_COUNT_MAX];
    unsigned int maxLeftoverSamplesCnt = 0;
    bool trigTimeSent = false;
    bool allDataSent = true;
    unsigned int headerStatus[CHANNEL_COUNT_MAX];
    int recordCnt = 0;
    short* daqDataBuffer[CHANNEL_COUNT_MAX];
    short* daqLeftoverSamples[CHANNEL_COUNT_MAX];
    streamingHeader_t* daqStreamHeaders[CHANNEL_COUNT_MAX];
    double PicoSec = (m_adqType == 8) ? PicoSec25 : PicoSec125;

    try
    {
        unsigned int bufferSize = allocateBuffers(daqDataBuffer, daqStreamHeaders, &daqLeftoverSamples);
        if (!bufferSize)
            ADQNDS_MSG_ERRLOG_PV(0, "allocateBuffers failed.");

        for (unsigned int chan = 0; chan < CHANNEL_COUNT_MAX; ++chan)
        {
            recordsDoneCnt[chan] = 0;
            samplesAddedCnt[chan] = 0;
            headersAdded[chan] = 0;
            samplesLeftoverSamplesCnt[chan] = 0;
            headerStatus[chan] = 0;

            if (!((1 << chan) & m_chanMask))
                continue;

            m_AIChannelsPtr[chan]->resize(m_sampleCnt);
            channelCount++;
        }


        {
            std::lock_guard<std::mutex> lock(m_adqDevMutex);
            status = m_adqInterface->StopStreaming();
            ADQNDS_MSG_ERRLOG_PV(status, "StopStreaming failed.");

            status = m_adqInterface->TriggeredStreamingSetup(m_recordCnt, m_sampleCnt, m_preTrigSamp, m_trigHoldOffSamp, m_chanMask);
            ADQNDS_MSG_ERRLOG_PV(status, "TriggeredStreamingSetup failed.");

            status = m_adqInterface->FlushPacketOnRecordStop(1);
            ADQNDS_MSG_ERRLOG_PV(status, "FlushPacketOnRecordStop failed.");

            status = m_adqInterface->SetStreamStatus(1);
            ADQNDS_MSG_ERRLOG_PV(status, "SetStreamStatus to 1 (Stream enabled) failed.");
            if (m_adqType == 714 || m_adqType == 14)
            {
                status = m_adqInterface->SetStreamConfig(2, 0);
                ADQNDS_MSG_ERRLOG_PV(status, "SetStreamConfig 2 (enable packet headers) failed.");
            }

            status = m_adqInterface->SetTransferBuffers(m_chanCnt, bufferSize);
            ADQNDS_MSG_ERRLOG_PV(status, "SetTransferBuffers failed.");

            status = m_adqInterface->StartStreaming();
            ADQNDS_MSG_ERRLOG_PV(status, "StartStreaming failed.");
        }
        if (m_trigMode == 0)
        {
            for (int i = 0; i < m_recordCnt; ++i)
            {
                status = m_adqInterface->SWTrig();
                ADQNDS_MSG_ERRLOG_PV(status, "SWTrig failed.");
            }
        }
        else
        {
            status = armTrigger();
            ADQNDS_MSG_ERRLOG_PV(status, "armTrigger.");
        }


        /* Data acquisition loop.
         * During infinite data collection each record is sent to data PV.
         * When number of records is limited, the whole buffer is sent to data PV.
         */
        do
        {
            // Mutex is used to save digitizer's API library from interruptions during infinite collection
            {
                std::lock_guard<std::mutex> lock(m_adqDevMutex);

                buffersFilled = 0;

                status = m_adqInterface->GetStreamOverflow();

                if (status)
                {
                    status = 0;
                    ADQNDS_MSG_ERRLOG_PV(status, "Streaming overflow detected.");
                }

                buffersFilled = 0;
                int32_t thisTimeout = 1;
                if (int(maxLeftoverSamplesCnt) >= m_sampleCnt)
                    // We already have at least one complete channel record ready to send.
                    thisTimeout = 0;
                else if (allDataSent)
                    // We have no buffered data currently.
                    thisTimeout = m_timeout;
                status = m_adqInterface->WaitForTransferBuffer(&buffersFilled, thisTimeout);
                ADQNDS_MSG_ERRLOG_PV(status, "WaitForTransferBuffer failed.");
                if (buffersFilled)
                {
                    ndsInfoStream(m_node) << "INFO: Receiving data..." << std::endl;
                    status = m_adqInterface->GetDataStreaming((void**)daqDataBuffer, (void**)daqStreamHeaders, m_chanMask, samplesAddedCnt, headersAdded, headerStatus);
                    TraceOutWithTime(m_node, "GetDataStreaming returned %u buffers with %u %u %u %u samples on records %u %u %u %u",
                        buffersFilled, samplesAddedCnt[0], samplesAddedCnt[1], samplesAddedCnt[2], samplesAddedCnt[3],
                        recordsDoneCnt[0], recordsDoneCnt[1], recordsDoneCnt[2], recordsDoneCnt[3]);
                    ADQNDS_MSG_ERRLOG_PV(status, "GetDataStreaming failed.");
                }
                else
                {
                    memset(samplesAddedCnt, 0, sizeof(samplesAddedCnt));
                    memset(headersAdded, 0, sizeof(headersAdded));
                    memset(headerStatus, 0, sizeof(headerStatus));
                }
                nds::state_t localState = m_stateMachine.getLocalState();
                if ((localState != nds::state_t::running) &&
                    (localState != nds::state_t::starting))
                {
                    ndsInfoStream(m_node) << "INFO: Data acquisition was stopped." << std::endl;
                    goto finish;
                }
            }

            // Go through each channel's m_daqDataBuffer data: check on incomplete records, add samples of incomplete records to the next records
            maxLeftoverSamplesCnt = 0;
            for (unsigned int chan = 0; chan < CHANNEL_COUNT_MAX; ++chan)
            {
                if (!((1 << chan) & m_chanMask))
                    continue;

                unsigned int headersDoneCnt = headersAdded[chan];
                if ((headersDoneCnt > 0) && (!headerStatus[chan]))
                    // One incomplete record in the buffer (header is copied to the front of the buffer later)
                    headersDoneCnt--;

                // Go through added samples
                if ((samplesAddedCnt[chan] > 0) || (int(samplesLeftoverSamplesCnt[chan] + samplesAddedCnt[chan]) >= m_sampleCnt))
                {
                    allDataSent = false;
                    // Handle incomplete record at the start of the buffer
                    if (samplesLeftoverSamplesCnt[chan] > 0)
                    {
                        // There is already some data in the daqLeftoverSamples buffer.
                        // Add all the new samples to it.
                        std::memcpy(&(daqLeftoverSamples[chan][samplesLeftoverSamplesCnt[chan]]),
                            daqDataBuffer[chan],
                            sizeof(short) * samplesAddedCnt[chan]);
                        samplesLeftoverSamplesCnt[chan] += samplesAddedCnt[chan];
                        if (int(samplesLeftoverSamplesCnt[chan]) >= m_sampleCnt)
                        {
                            // There is enough data in the transfer buffer to complete the record
                            m_AIChannelsPtr[chan]->readData(daqLeftoverSamples[chan], m_sampleCnt);
                            samplesLeftoverSamplesCnt[chan] -= m_sampleCnt;
                            // If there is at least one complete header
                            recordsDoneCnt[chan]++;
                            if (samplesLeftoverSamplesCnt[chan] > 0)
                                memmove(daqLeftoverSamples[chan], daqLeftoverSamples[chan] + m_sampleCnt, sizeof(short) * samplesLeftoverSamplesCnt[chan]);
                            TraceOutWithTime(m_node, "Sent data samplesLeftoverSamplesCnt[%d]=%d", chan, samplesLeftoverSamplesCnt[chan]);
                        }
                    }
                    else if (int(samplesAddedCnt[chan]) >= m_sampleCnt)
                    {
                        // At least one complete record is available in this new data.
                        recordsDoneCnt[chan]++;
                        m_AIChannelsPtr[chan]->readData(daqDataBuffer[chan], m_sampleCnt);
                        if (int(samplesAddedCnt[chan]) > m_sampleCnt)
                        {
                            // This new data contains more than one complete record. Store it for use later.
                            samplesLeftoverSamplesCnt[chan] = samplesAddedCnt[chan] - m_sampleCnt;
                            std::memcpy(&(daqLeftoverSamples[chan][0]),
                                &(daqDataBuffer[chan][m_sampleCnt]),
                                sizeof(short) * samplesLeftoverSamplesCnt[chan]);
                        }
                    }
                    else
                    {
                        // This new data contains an incomplete record. Store it for use later.
                        std::memcpy(daqLeftoverSamples[chan],
                            daqDataBuffer[chan],
                            sizeof(short) * samplesAddedCnt[chan]);
                        samplesLeftoverSamplesCnt[chan] = samplesAddedCnt[chan];
                    }
                    samplesAddedCnt[chan] = 0;
                    maxLeftoverSamplesCnt = std::max(maxLeftoverSamplesCnt, samplesLeftoverSamplesCnt[chan]);
                }
                if ((!trigTimeSent) && (headersDoneCnt))
                {
                    // Only the first channel contains a valid timestamp.
                    m_trigTimeStamp = daqStreamHeaders[chan]->timeStamp * PicoSec;
                    if (recordCnt == 0)
                        // Timestamp issue: the first time stamp recorded is not reset.
                        if ((m_trigMode == 1) || (m_trigMode == 2))
                            m_trigTimeStamp = 0;
                    trigTimeSent = true;
                    TraceOutWithTime(m_node, "Sent timestamp %f", m_trigTimeStamp);
                }
            }

            int recordsDoneMinCnt = INT_MAX;
            for (unsigned int chan = 0; chan < CHANNEL_COUNT_MAX; ++chan)
            {
                if (!((1 << chan) & m_chanMask))
                    continue;
                recordsDoneMinCnt = std::min(recordsDoneMinCnt, recordsDoneCnt[chan]);
            }
            if (recordCnt < recordsDoneMinCnt)
            {
                struct timespec now;
                clock_gettime(CLOCK_REALTIME, &now);
                recordCnt = recordsDoneMinCnt;
                m_trigTimeStampPV.push(now, m_trigTimeStamp);
                trigTimeSent = false;
                allDataSent = (maxLeftoverSamplesCnt == 0);
                for (unsigned int chan = 0; chan < CHANNEL_COUNT_MAX; ++chan)
                {
                    if (!((1 << chan) & m_chanMask))
                        continue;
                    m_AIChannelsPtr[chan]->pushData(now);
                }
                m_recordCntPV.push(now, recordCnt);
                TraceOutWithTime(m_node, "Sent recordcount %d", recordCnt);
            }
        } while (recordCnt < m_recordCnt);
    }
    catch (nds::NdsError const&) {
    }

finish:
    setTriggerIdleState();
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);

        status = m_adqInterface->SetStreamStatus(0);
        ADQNDS_MSG_WARNLOG_PV(status, "SetStreamStatus to 0 (Stream disabled) failed.");

        status = m_adqInterface->StopStreaming();
        ADQNDS_MSG_WARNLOG_PV(status, "StopStreaming failed.");
    }
    ndsInfoStream(m_node) << "INFO: Acquisition finished." << std::endl;

    for (unsigned int chan = 0; chan < CHANNEL_COUNT_MAX; ++chan)
    {
        if (daqDataBuffer[chan])
            free(daqDataBuffer[chan]);
        if (daqStreamHeaders[chan])
            free(daqStreamHeaders[chan]);
        if (daqLeftoverSamples[chan])
            free(daqLeftoverSamples[chan]);
    }

    try
    {
        if ((m_stateMachine.getLocalState() == nds::state_t::running) ||
            (m_stateMachine.getLocalState() == nds::state_t::stopping))
            m_stateMachine.setState(nds::state_t::on);
    }
    catch (nds::StateMachineNoSuchTransition& /*error*/)
    {
        ADQNDS_MSG_WARNLOG_PV(status, "State transition error.");
    }

    commitChanges(true);
}

int ADQAIChannelGroup::commitDaisyChainTriggerSource(struct timespec const& now)
{
    int status = 1;
    if ((m_masterMode == 1) && (m_adqType == 8))
    {
        if (m_trigMode == 2)
        {
            TraceOutWithTime(m_node, "DaisyChainSetupLevelTrigger %d %d %d", m_levelTrigChan, m_levelTrigLvl, m_levelTrigEdge);
            status = m_adqInterface->DaisyChainSetupLevelTrigger(m_levelTrigChan, m_levelTrigLvl, 1, m_levelTrigEdge);
            if (status)
                // There is no other readback of this valaue when using the daisy chain.
                m_levelTrigEdgePV.push(now, m_levelTrigEdge);

            ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainSetupLevelTrigger failed.");
        }
        TraceOutWithTime(m_node, "DaisyChainSetTriggerSource %d", m_trigMode + 1);
        status = m_adqInterface->DaisyChainSetTriggerSource(m_trigMode + 1);
        ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainSetTriggerSource failed.");
        DaisyChainGetStatus(__LINE__);
    }
    if (status) {
        ndsInfoStream(m_node) << "Daisy chain trigger source set." << std::endl;
    }
    return status;
}

void ADQAIChannelGroup::setTriggerIdleState()
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    // Dont trigger while the daisy chain isn't in operation.
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);
#endif
    int status = m_adqInterface->SetInternalTriggerPeriod(std::numeric_limits<unsigned int>::max() - 4);
    ADQNDS_MSG_WARNLOG_PV(status, "SetInternalTriggerPeriod failed.");
    uint32_t internTrigLowSamp = std::numeric_limits<unsigned int>::max() - 4 - m_internTrigHighSamp;
    status = m_adqInterface->SetInternalTriggerHighLow(m_internTrigHighSamp, internTrigLowSamp);
    ADQNDS_MSG_WARNLOG_PV(status, "SetInternalTriggerHighLow failed.");
    if ((m_adqType == 412) || (m_adqType == 1600) || (m_adqType == 108) || (m_adqType == 14) || (m_adqType == 7) || (m_adqType == 8))
    {
        status = m_adqInterface->SetInternalTriggerSyncMode(0);
        ADQNDS_MSG_WARNLOG_PV(status, "SetInternalTriggerSyncMode failed.");
    }
    status = m_adqInterface->DisarmTimestampSync();
    ADQNDS_MSG_WARNLOG_PV(status, "DisarmTimestampSync failed.");
    status = m_adqInterface->DisarmTrigger();
    ADQNDS_MSG_WARNLOG_PV(status, "DisarmTrigger failed.");
    if (m_adqType == 8)
    {
        status = m_adqInterface->DaisyChainSetOutputState(0);
        ADQNDS_MSG_WARNLOG_PV(status, "DaisyChainSetOutputState failed.");
    }
}

void ADQAIChannelGroup::commitInternTrigEdge(struct timespec const& now)
{
    m_internTrigEdgeChanged = false;

    int status = m_adqInterface->SetTriggerEdge(m_trigMode + 1, m_internTrigEdge);
    ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerEdge failed.");

    if (status)
    {
        unsigned int trigEdge = 0;
        status = m_adqInterface->GetTriggerEdge(m_trigMode + 1, &trigEdge);
        ADQNDS_MSG_WARNLOG_PV(status, "GetTriggerEdge failed.");
        if (status)
        {
            m_internTrigEdge = trigEdge;
            m_internTrigEdgePV.push(now, m_internTrigEdge);
        }
    }
}

void ADQAIChannelGroup::commitLevelTrigEdge(struct timespec const& now)
{
    m_levelTrigEdgeChanged = false;

    TraceOutWithTime(m_node, "SetTriggerEdge %d", m_levelTrigEdge);
    int status = m_adqInterface->SetTriggerEdge(m_trigMode + 1, m_levelTrigEdge);
    ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerEdge failed.");

    if ((status) && (m_masterMode != 1))
    {
        // Does not work for the daisy chain usage.
        unsigned int trigEdge = 0;
        status = m_adqInterface->GetTriggerEdge(m_trigMode + 1, &trigEdge);
        ADQNDS_MSG_WARNLOG_PV(status, "GetTriggerEdge failed.");
        if (status)
        {
            m_levelTrigEdgePV.push(now, int(trigEdge));
        }
    }
}

void ADQAIChannelGroup::commitClockRefOut(struct timespec const& now)
{
    m_clockRefOutChanged = false;

    if (m_clockRefOut < 0)
        m_clockRefOut = 0;
    if (m_clockRefOut > 1)
        m_clockRefOut = 1;
    int status = m_adqInterface->EnableClockRefOut(m_clockRefOut);
    ADQNDS_MSG_WARNLOG_PV(status, "EnableClockRefOut failed.");
    if (status)
        m_clockRefOutPV.push(now, m_clockRefOut);
}

void ADQAIChannelGroup::commitExternTrigEdge(struct timespec const& now)
{
    m_externTrigEdgeChanged = false;

    int status = m_adqInterface->SetTriggerEdge(m_trigMode + 1, m_externTrigEdge);
    ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerEdge failed.");

    if (status)
    {
        unsigned int trigEdge = 0;
        status = m_adqInterface->GetTriggerEdge(m_trigMode + 1, &trigEdge);
        ADQNDS_MSG_WARNLOG_PV(status, "GetTriggerEdge failed.");
        if (status)
        {
            m_externTrigEdge = trigEdge;
            m_externTrigEdgePV.push(now, m_externTrigEdge);
        }
    }
}

void ADQAIChannelGroup::commitExternTrigImpedance(struct timespec const& now)
{   
    m_externTrigInputImpedanceChanged = false;
    int status = m_adqInterface->SetTriggerInputImpedance(1, m_externTrigInputImpedance);
    ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerInputImpedance failed.");
    unsigned int externTrigInputImpedance;
    status = m_adqInterface->GetTriggerInputImpedance(1, &externTrigInputImpedance);
    ADQNDS_MSG_WARNLOG_PV(status, "GetTriggerInputImpedance failed.");
    m_externTrigInputImpedancePV.push(now, int(externTrigInputImpedance));
}

void ADQAIChannelGroup::commitInternTrigHighSamp(struct timespec const& now)
{
    m_internTrigHighSampChanged = false;

    if (m_internTrigHighSamp < 4)
    {
        m_internTrigHighSamp = 4;
        ndsInfoStream(m_node) << "INFO: Sample length can't be less than 4." << std::endl;
    }

    m_internTrigLowSamp = m_internTrigPeriod - m_internTrigHighSamp;
    if (m_masterMode != 1)
    {
        TraceOutWithTime(m_node, "SetInternalTriggerHighLow %u %u %u", m_internTrigPeriod, m_internTrigHighSamp, m_internTrigLowSamp);
        int status = m_adqInterface->SetInternalTriggerHighLow(m_internTrigHighSamp, m_internTrigLowSamp);
        ADQNDS_MSG_WARNLOG_PV(status, "SetInternalTriggerHighLow failed.");
    }

    m_internTrigHighSampPV.push(now, m_internTrigHighSamp);
    m_internTrigLowSampPV.push(now, m_internTrigLowSamp);
}

void ADQAIChannelGroup::commitInternTrigFreqChanged(struct timespec const& now)
{
    m_internTrigFreqChanged = false;

    if (m_internTrigFreq <= 0)
        m_internTrigFreq = 1;

    m_internTrigPeriod = int32_t(0.5 + m_SampleRate / m_internTrigFreq);
    m_internTrigLowSamp = m_internTrigPeriod - m_internTrigHighSamp;

    if (m_masterMode != 1)
    {
        TraceOutWithTime(m_node, "SetInternalTriggerPeriod %u %u %u", m_internTrigPeriod, m_internTrigHighSamp, m_internTrigLowSamp);
        int status = m_adqInterface->SetInternalTriggerPeriod(m_internTrigPeriod);
        ADQNDS_MSG_WARNLOG_PV(status, "SetInternalTriggerPeriod failed.");
    }

    m_internTrigFreqPV.push(now, m_internTrigFreq);
}

void ADQAIChannelGroup::commitExternTrigThreshold(struct timespec const& now)
{
    m_externTrigThresholdChanged = false;
    int status;
    if (m_adqInterface->HasVariableTrigThreshold(EXTERN_TRIG_COUNT))
    {
        if (m_adqType == 714 || m_adqType == 14)
        {
            status = m_adqInterface->SetExtTrigThreshold(EXTERN_TRIG_COUNT, m_externTrigThreshold);
            ADQNDS_MSG_WARNLOG_PV(status, "SetExtTrigThreshold failed.");

            if (status)
                m_externTrigThresholdPV.push(now, m_externTrigThreshold);
        }
        if ((m_adqType == 7) || (m_adqType == 8))
        {
            status = m_adqInterface->SetTriggerThresholdVoltage(m_trigMode + 1, m_externTrigThreshold);
            ADQNDS_MSG_WARNLOG_PV(status, "SetExtTrigThreshold failed.");

            if (status)
                m_externTrigThresholdPV.push(now, m_externTrigThreshold);
        }
    }
    else
    {
        ndsInfoStream(m_node) << "INFO: Device doesn't have a programmable extern trigger threshold." << std::endl;
    }
}

void ADQAIChannelGroup::commitPatternMode(struct timespec const& now)
{
    m_patternModeChanged = false;
    int status = m_adqInterface->SetTestPatternMode(m_patternMode);
    ADQNDS_MSG_WARNLOG_PV(status, "SetTestPatternMode failed.");
    if (status)
    {
        m_patternModePV.push(now, m_patternMode);
        ndsInfoStream(m_node) << "SUCCESS: SetTestPatternMode" << std::endl;
    }
}

void ADQAIChannelGroup::commitLevelTrigLvl(struct timespec const& now)
{
    m_levelTrigLvlChanged = false;

    TraceOutWithTime(m_node, "SetLvlTrigLevel %d", m_levelTrigLvl);
    int status = m_adqInterface->SetLvlTrigLevel(m_levelTrigLvl);
    ADQNDS_MSG_WARNLOG_PV(status, "SetLvlTrigLevel failed.");

    if (m_adqType == 714 || m_adqType == 14)
        m_levelTrigLvl = m_adqInterface->GetLvlTrigLevel();

    m_levelTrigLvlPV.push(now, m_levelTrigLvl);
}

void ADQAIChannelGroup::commitLevelTrigChanMask(struct timespec const& now)
{
    m_levelTrigChanChanged = false;
    m_levelTrigChanMaskChanged = false;
    if (m_levelTrigChan < 0)
    {
        ndsWarningStream(m_node) << "negative level trigger channel is resetted to 0" << std::endl;
        m_levelTrigChan = 0;
    }

    if (m_chanCnt == 1)
    {
        switch (m_levelTrigChan)
        {
        case 0:   // None 
            m_levelTrigChanMask = 0;
            break;
        case 1:   // ch A
            m_levelTrigChanMask = 1;
            break;
        default:
            ADQNDS_MSG_WARNLOG_PV(0, "Device has only one channel -> changed to channel A");
            m_levelTrigChanMask = 1;
            break;
        }
    }

    if (m_chanCnt == 2)
    {
        switch (m_levelTrigChan)
        {
        case 0:   // None 
            m_levelTrigChanMask = 0;
            break;
        case 1:   // ch A
            m_levelTrigChanMask = 1;
            break;
        case 2:   // ch B
            m_levelTrigChanMask = 2;
            break;
        case 3:   // ch A+B
            if (m_adqType == 7)
            {
                ADQNDS_MSG_WARNLOG_PV(0, "ADQ7 allows only one channel to generate the trigger -> "
                    "changed to channel B");
                m_levelTrigChanMask = 2;
            }
            else
            {
                m_levelTrigChanMask = 3;
            }
            break;
        default:
            ADQNDS_MSG_WARNLOG_PV(0, "Device has only two channels -> changed to channel B");
            m_levelTrigChanMask = 2;
            break;
        }
    }

    if (m_chanCnt == 4)
    {
        switch (m_levelTrigChan)
        {
        case 0:   // None 
            m_levelTrigChanMask = 0;
            break;
        case 5:   // ch A+B
            m_levelTrigChanMask = 3;
            break;
        case 6:   // ch C+D
            m_levelTrigChanMask = 12;
            break;
        case 7:   // ch A+B+C+D
            m_levelTrigChanMask = 15;
            break;
        default:
            m_levelTrigChanMask = 1 << (m_levelTrigChan - 1);
            break;
        }
    }

    if (m_chanCnt == 8)
    {
        int channel = 0;
        for (channel = 0; channel < int(m_chanCnt); channel++)
            if ((m_levelTrigChanMask & (1 << channel)) != 0)
                break;
        m_levelTrigChan = channel + 1;
        commitDaisyChainTriggerSource(now);
    }

    m_levelTrigChanPV.push(now, m_levelTrigChan);

    TraceOutWithTime(m_node, "SetLvlTrigChannel %d %d", m_levelTrigChan, m_levelTrigChanMask);
    int status = m_adqInterface->SetLvlTrigChannel(m_levelTrigChanMask);
    ADQNDS_MSG_WARNLOG_PV(status, "SetLvlTrigChannel failed.");

    if (m_adqType == 714 || m_adqType == 14)
        m_levelTrigChanMask = m_adqInterface->GetLvlTrigChannel();

    m_levelTrigChanMaskPV.push(now, m_levelTrigChanMask);
}

void ADQAIChannelGroup::commitExternTrigDelay(struct timespec const& now)
{
    if ((m_adqType == 714) || (m_adqType == 14))
    {
        if (m_externTrigDelay < 0)
            m_externTrigDelay = 0;
        if (m_externTrigDelay > 37)
            m_externTrigDelay = 37;

        ndsInfoStream(m_node) << "INFO: Trigger delay: for ADQ14 valid range is [0, 37]." << std::endl;
    }

    if (m_adqType == 7)
    {
        if (m_externTrigDelay < 1)
            m_externTrigDelay = 1;
        if (m_externTrigDelay > 63)
            m_externTrigDelay = 63;

        ndsInfoStream(m_node) << "INFO: Trigger delay: for ADQ7 valid range is [1, 63]." << std::endl;
    }

    int status = m_adqInterface->SetExternalTriggerDelay(m_externTrigDelay);
    ADQNDS_MSG_WARNLOG_PV(status, "SetExternalTriggerDelay failed.");

    m_externTrigDelayChanged = false;
    m_externTrigDelayPV.push(now, m_externTrigDelay);
}

void ADQAIChannelGroup::commitDaqMode(struct timespec& now)
{
    m_daqModeChanged = false;

    if ((m_daqMode == 2) && ((m_adqType == 714) || (m_adqType == 14)))   // Triggered streaming and ADQ14
    {
        int status = m_adqInterface->HasTriggeredStreamingFunctionality();
        if (!status)
        {
            ADQNDS_MSG_WARNLOG_PV(status, "Device doesn't have triggered streaming functionality.");
            m_daqMode = 1;
        }
    }
    m_daqModePV.push(now, m_daqMode);

    // Trigger sample and records numbers to update
    m_recordCntPV.read(&now, &m_recordCnt);
    m_recordCntPV.push(now, m_recordCnt);
    m_sampleCntPV.read(&now, &m_sampleCnt);
    m_sampleCntPV.push(now, m_sampleCnt);
    // Trigger records to collect to update
    m_recordCntCollectPV.read(&now, &m_recordCntCollect);
    m_recordCntCollectPV.push(now, m_recordCntCollect);
}

void ADQAIChannelGroup::commitClockSource(struct timespec const& now)
{
    m_clockSrcChanged = false;

    if (m_clockSrc < 0)
    {
        m_clockSrc = 0;
    }

    if (m_clockSrc > 6)
    {
        m_clockSrc = 6;
    }

    int status = m_adqInterface->SetClockSource(m_clockSrc);
    ADQNDS_MSG_WARNLOG_PV(status, "SetClockSource failed.");

    if (m_adqType == 714 || m_adqType == 14)
    {
        m_clockSrc = m_adqInterface->GetClockSource();
    }

    if (status)
        m_clockSrcPV.push(now, m_clockSrc);
    if (status) {
        ndsInfoStream(m_node) << "Clock source set." << std::endl;
    }
}

void ADQAIChannelGroup::commitTriggerMode(struct timespec const& now)
{
    m_trigModeChanged = false;

    int32_t trigMode = m_trigMode + 1;
    if (trigMode == 5)
        // Daisy chain trigger
        trigMode = 23;
    if (trigMode == 6)
        // PXIe trigger.
        trigMode = 14;

    int status = m_adqInterface->DisarmTrigger();
    ADQNDS_MSG_ERRLOG_PV(status, "DisarmTrigger failed.");
    if (m_masterMode == 1)
    {
        // Don't start triggering until the daisy chain is ready!
        uint32_t internTrigLowSamp = std::numeric_limits<unsigned int>::max() - 4 - m_internTrigHighSamp;
        TraceOutWithTime(m_node, "SetInternalTriggerPeriod %u %u %u", std::numeric_limits<unsigned int>::max() - 4, m_internTrigHighSamp, internTrigLowSamp);
        status = m_adqInterface->SetInternalTriggerPeriod(std::numeric_limits<unsigned int>::max() - 4);
        ADQNDS_MSG_ERRLOG_PV(status, "SetInternalTriggerPeriod failed.");
        status = m_adqInterface->SetInternalTriggerHighLow(m_internTrigHighSamp, internTrigLowSamp);
        ADQNDS_MSG_ERRLOG_PV(status, "SetInternalTriggerHighLow failed.");
        // Trigger from the daisy chain and NOT the actual trigger source.
        // Or the daisy information will not be collected correctly.
        // The trigger source is set through DaisyChainSetTriggerSource().
        TraceOutWithTime(m_node, "setTriggerMode %d", 23);
        status = m_adqInterface->SetTriggerMode(23);
    }
    else
    {
        TraceOutWithTime(m_node, "setTriggerMode %d", trigMode);
        status = m_adqInterface->SetTriggerMode(trigMode);
    }
    ADQNDS_MSG_ERRLOG_PV(status, "SetTriggerMode failed.");
    if (status)
    {
        int trigModeReadBack = m_adqInterface->GetTriggerMode();
        if (trigModeReadBack == 23)
            // Daisy chain trigger
            trigModeReadBack = m_trigMode;
        else if (trigModeReadBack == 14)
            trigModeReadBack = 6;
        else
            trigModeReadBack -= 1;
        m_trigModePV.push(now, trigModeReadBack);
    }

    if (m_trigMode != 2) // Level trigger cannot be used for timestamp synchronization.
    {
        if (trigMode == 23)
            // Daisy chain trigger sync not work. Use SYNC input instead.
            trigMode = 9;
        status = m_adqInterface->SetupTimestampSync(0, trigMode);
        ADQNDS_MSG_ERRLOG_PV(status, "SetupTimestampSync failed.");
    }

    if (status) {
        ndsInfoStream(m_node) << "INFO: Trigger mode set." << std::endl;
    }
}

void ADQAIChannelGroup::commitTriggerDelayOrHoldoff(struct timespec const& now)
{
    int status = 0;
    m_preTrigSampChanged = false;
    m_trigHoldOffSampChanged = false;
    m_daisyPositionChanged = false;

    int32_t preTrigSamp = m_preTrigSamp;
    int32_t trigHoldOffSamp = m_trigHoldOffSamp;
    if ((m_daisyPosition.size() > 0) && (m_daisyPosition[0] < 0))
    {
        ndsWarningStream(m_node) << "Daisy chain position is resetted to 0" << std::endl;
        m_daisyPosition[0] = 0;
    }
    if (m_daisyPosition.size() > 0)
    {
        if ((m_daisyPosition.size() > 1) && (m_masterMode != 1))
        {
            ndsWarningStream(m_node) << "Daisy chain information only available to master modules" << std::endl;
            m_daisyPosition.resize(1);
        }
        if (m_adqType != 8)
        {
            ndsWarningStream(m_node) << "Daisy chain position only an option for ADQ8" << std::endl;
            m_daisyPosition.clear();
        }
        else
        {
            int DaisyPreTriggerSamples=0;
            if (m_daisyPosition.size() == 0)
                m_daisyPosition.resize(1);
            m_device_info.resize(m_daisyPosition.size());
            for (size_t Position = 0; Position < m_device_info.size(); Position++)
            {
                status = m_adqInterface->DaisyChainGetNofPretriggerSamples(m_daisyPosition[Position], m_SampleRate, &DaisyPreTriggerSamples);
                ADQNDS_MSG_WARNLOG_PV(status, "DaisyChainGetNofPretriggerSamples failed.");
                m_device_info[Position].Position = m_daisyPosition[Position];
                m_device_info[Position].PretriggerSamples = preTrigSamp + DaisyPreTriggerSamples;
                // The requirement to add 4 is only required if DaisyChainGetTriggerInformation is set to use trigger source 3.
                //m_device_info[Position].PretriggerSamples += 4;
                m_device_info[Position].TriggerDelaySamples = trigHoldOffSamp;
                m_device_info[Position].SampleRate = m_SampleRate;
            }
            preTrigSamp = m_device_info[0].PretriggerSamples;
            if (m_masterMode != 1)
                m_device_info.resize(0);
        }
    }

    if (preTrigSamp > m_sampleCnt)
    {
        char Buf[20];
        snprintf(Buf, sizeof(Buf) - 1, "%d", preTrigSamp);
        std::string Warning = "Number of pre-trigger samples (" + std::string(Buf);
        snprintf(Buf, sizeof(Buf) - 1, "%d", m_sampleCnt);
        Warning += ") must be less than number of samples (" + std::string(Buf) + ") per record.";
        m_preTrigSamp = preTrigSamp = m_sampleCnt;
        m_preTrigSampPV.push(now, m_preTrigSamp);
        ADQNDS_MSG_WARNLOG_PV(0, Warning);
    }

    if (preTrigSamp < 0)
    {
        m_preTrigSamp = preTrigSamp = 0;
        m_preTrigSampPV.push(now, 0);
        m_preTrigSampPV.push(now, 0);
        ndsWarningStream(m_node) << "Number of pre-trigger samples is resetted to zero" << std::endl;
    }

    if ((m_daqMode == 0) || (m_daqMode == 2)) // Multi-Record
    {
        status = m_adqInterface->SetPreTrigSamples(preTrigSamp);
        ADQNDS_MSG_WARNLOG_PV(status, "SetPreTrigSamples failed.");
        m_preTrigSampPV.push(now, m_preTrigSamp);
    }
    else
        ADQNDS_MSG_WARNLOG_PV(status, "SetPreTrigSamples only in multirecord mode.");

    if (trigHoldOffSamp > m_sampleCnt)
    {
        trigHoldOffSamp = m_trigHoldOffSamp = 0;
        m_trigHoldOffSampPV.push(now, 0);
        ndsWarningStream(m_node) << "Number of hold-off samples must be less than number of samples per record." << std::endl;
    }

    if (trigHoldOffSamp < 0)
    {
        ndsWarningStream(m_node) << "Trigger hold-off is resetted to zero." << std::endl;
        trigHoldOffSamp = m_trigHoldOffSamp = 0;
        m_trigHoldOffSampPV.push(now, 0);
    }

    if ((m_daqMode == 0) || (m_daqMode == 2)) // Multi-Record
    {
        m_trigHoldOffSampChanged = false;
        status = m_adqInterface->SetTriggerDelay(trigHoldOffSamp);
        ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerDelay failed.");
        m_trigHoldOffSampPV.push(now, trigHoldOffSamp);
    }
    else
        ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerDelay only in multirecord mode.");

    if (status)
    {
        m_daisyPositionPV.push(now, m_daisyPosition);
        ndsInfoStream(m_node) << "INFO: Trigger delay and holdoff set." << std::endl;
    }
}


void ADQAIChannelGroup::DaisyChainGetStatus(long Line)
{
    unsigned int DaisyChainStatus = 0xFF;
    int status = m_adqInterface->DaisyChainGetStatus(&DaisyChainStatus);
    DaisyChainStatus &= 0XE; // We don't need the state of the daisy chain input.
    ADQNDS_MSG_WARNLOG_PV(status, "DaisyChainGetStatus failed.");
    if (DaisyChainStatus != m_DaisyChainStatus)
    {
        ADQNDS_MSG_WARNLOG_PV(0, "DaisyChainGetStatus fault");
        ndsWarningStream(m_node) << "DaisyChainGetStatus fault " << DaisyChainStatus << " on " << m_node.getFullExternalName() << " at " << Line << std::endl;
        m_DaisyChainStatus = DaisyChainStatus;
    }
}

void ADQAIChannelGroup::commitDaisyChain(struct timespec const& now)
{
    int status=0;
    TraceOutWithTime(m_node, "setDaisyChain %d", m_masterMode);
    m_masterModeChanged = false;
    m_sync_immediateChanged = false;
    if (m_adqType == 8)
    {
        status = m_adqInterface->DaisyChainEnableOutput(0);
        ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainEnableOutput failed.");
        status = m_adqInterface->DaisyChainEnable(0);
        ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainEnable failed.");
        status = m_adqInterface->DaisyChainReset();
        ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainReset failed.");
        if (m_masterMode == 1)
        {
            status = m_adqInterface->DaisyChainSetMode(1); // Master mode
            ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainSetMode failed.");
        }
        else if (m_masterMode == 2)
        {
            status = m_adqInterface->DaisyChainSetMode(0); // Slave mode
            ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainSetMode failed.");
        }
        commitDaisyChainTriggerSource(now);
        if (m_masterMode != 0)
        {
            status = m_adqInterface->DaisyChainSetupOutput(1, m_sync_immediate, 12);
            ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainSetupOutput failed.");
            status = m_adqInterface->DaisyChainEnableOutput(1);
            ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainEnableOutput failed.");
            DaisyChainGetStatus(__LINE__);
        }
    }
    m_masterModePV.push(now, m_masterMode);
    if (status)
    {
        ndsInfoStream(m_node) << "INFO: Master mode set." << std::endl;
    }
}

void ADQAIChannelGroup::commitSetupDBS(struct timespec const& now)
{
    unsigned int dbsInstCnt;

    m_dbsBypassChanged = false;
    m_dbsDcChanged = false;
    m_dbsLowSatChanged = false;
    m_dbsUpSatChanged = false;

    int status = m_adqInterface->GetNofDBSInstances(&dbsInstCnt);
    if (status)
    {
        if (m_dbsBypass < 0)
            m_dbsBypass = 0;
        if (m_dbsBypass > 1)
            m_dbsBypass = 1;
        if (m_dbsLowSat < 0)
            m_dbsLowSat = 0;
        if (m_dbsUpSat < 0)
            m_dbsUpSat = 0;

        unsigned int i = 0;
        for (unsigned char dbsInst = 0; dbsInst < dbsInstCnt; ++dbsInst)
        {
            status = m_adqInterface->SetupDBS(dbsInst, m_dbsBypass, m_dbsDc, m_dbsLowSat, m_dbsUpSat);
            if (status)
            {
                ++i;
            }
        }
        SLEEP(1000);

        if (i == dbsInstCnt)
        {
            ndsInfoStream(m_node) << "SUCCESS: SetupDBS is set for all channels." << std::endl;
            m_dbsBypassPV.push(now, m_dbsBypass);
            m_dbsDcPV.push(now, m_dbsDc);
            m_dbsLowSatPV.push(now, m_dbsLowSat);
            m_dbsUpSatPV.push(now, m_dbsUpSat);
        }
        else
        {
            status = 0;
            ADQNDS_MSG_WARNLOG_PV(status, "SetupDBS failed on one or more channels.");
        }
    }
    else
    {
        status = 0;
        ADQNDS_MSG_WARNLOG_PV(status, "GetNofDBSInstances failed.");
    }
}

void ADQAIChannelGroup::commitActiveChan(struct timespec const& now)
{
    m_chanActiveChanged = false;

    if (!m_chanCnt)
    {
        int status = 0;
        ADQNDS_MSG_WARNLOG_PV(status, "No channels are found.");
        return;
    }
    if (m_chanCnt == 1)
    {
        switch (m_chanActive)
        {
        case 0:   // ch A
            m_chanMask = 0x1;
            m_chanInt = 1;
            break;
        case 1:   // ch B
        case 2:   // ch A+B
        case 3:   // ch D
        case 4:   // ch A+B
        case 5:   // ch C+D
        case 6:   // ch A+B+C+D
            ADQNDS_MSG_WARNLOG_PV(0, "Device has only one channel -> changed to channel A.");
            m_chanMask = 0x1;
            m_chanInt = 1;
            break;
        }
    }

    if (m_chanCnt == 2)
    {
        switch (m_chanActive)
        {
        case 0:   // ch A
            m_chanMask = 0x1;
            m_chanInt = 1;
            break;
        case 1:   // ch B
            m_chanMask = 0x2;
            m_chanInt = 2;
            break;
        case 2:   // ch A+B
            if (m_daqMode != 3)
            {
                m_chanMask = 0x3;
                m_chanInt = 3;
            }
            else
            {
                ADQNDS_MSG_WARNLOG_PV(0, "Only one channel can be set for Raw Streaming -> changed to "
                    "channel B.");
                m_chanMask = 0x2;
                m_chanInt = 2;
            }
            break;
        case 3:   // ch D
        case 4:   // ch A+B
        case 5:   // ch C+D
        case 6:   // ch A+B+C+D
            ADQNDS_MSG_WARNLOG_PV(0, "Device has only two channels -> changed to channels A+B.");
            m_chanMask = 0x3;
            m_chanInt = 3;
            break;
        }
    }

    if (m_chanCnt == 4)
    {
        switch (m_chanActive)
        {
        case 0:   // ch A
            m_chanMask = 0x1;
            m_chanInt = 1;
            break;
        case 1:   // ch B
            m_chanMask = 0x2;
            m_chanInt = 2;
            break;
        case 2:   // ch C
            m_chanMask = 0x4;
            m_chanInt = 4;
            break;
        case 3:   // ch D
            m_chanMask = 0x8;
            m_chanInt = 8;
            break;
        case 4:   // ch A+B
            m_chanMask = 0x3;
            m_chanInt = 3;
            break;
        case 5:   // ch C+D
            m_chanMask = 0xC;
            m_chanInt = 12;
            break;
        case 6:   // ch A+B+C+D
            m_chanMask = 0xF;
            m_chanInt = 15;
            break;
        }
        if (m_chanCnt == 8)
        {
            switch (m_chanActive)
            {
            case 0:   // ch A
                m_chanMask = 0x1;
                m_chanInt = 1;
                break;
            case 1:   // ch B
                m_chanMask = 0x2;
                m_chanInt = 2;
                break;
            case 2:   // ch C
                m_chanMask = 0x4;
                m_chanInt = 4;
                break;
            case 3:   // ch D
                m_chanMask = 0x8;
                m_chanInt = 8;
                break;
            case 4:   // ch E
                m_chanMask = 0x10;
                m_chanInt = 16;
                break;
            case 5:   // ch F
                m_chanMask = 0x20;
                m_chanInt = 32;
                break;
            case 6:   // ch G
                m_chanMask = 0x40;
                m_chanInt = 64;
                break;
            case 7:   // ch H
                m_chanMask = 0x80;
                m_chanInt = 128;
                break;
            case 8:   // ch A+B+C+D
                m_chanMask = 0xF;
                m_chanInt = 15;
                break;
            case 9:   // ch E+F+G+H
                m_chanMask = 0xF0;
                m_chanInt = 240;
                break;
            case 10:   // ch A+B+C+D+E+F+G+H
                m_chanMask = 0xFF;
                m_chanInt = 255;
                break;
            }
        }
    }
    if ((m_daqMode == 3) && (m_chanInt > 8))   // Raw streaming
    {
        ADQNDS_MSG_WARNLOG_PV(0, "WARNLOG: Only one channel can be set for Raw Streaming -> changed to channel "
            "D.");
        m_chanMask = 0x8;
        m_chanInt = 8;
    }

    m_chanMaskPV.push(now, m_chanMask);
    m_chanActivePV.push(now, m_chanActive);
}

void ADQAIChannelGroup::commitRecordCount(struct timespec& now)
{
    unsigned int sampleCntMax;
    int status;

    m_recordCntChanged = false;
    m_sampleCntChanged = false;

    if (m_recordCnt < -1)
        m_recordCnt = -1;

    if (m_sampleCnt < 0)
        m_sampleCnt = 0;

    if ((m_recordCnt == -1) && (m_daqMode != 2))
    {
        m_recordCnt = 0;
        status = 0;
        ADQNDS_MSG_WARNLOG_PV(status, "Infinite collection is enabled only in Triggered mode.");
    }

    if (m_recordCnt >= 0)
    {
        m_recordCntPV.push(now, m_recordCnt);

        if (m_daqMode == 0)   // Multi-Record
        {
            status = m_adqInterface->GetMaxNofSamplesFromNofRecords(m_recordCnt, &sampleCntMax);
            ADQNDS_MSG_WARNLOG_PV(status, "Couldn't get the MAX number of samples: set number of records.");
            if (status)
            {
                m_sampleCntMax = sampleCntMax;
                m_sampleCntMaxPV.push(now, m_sampleCntMax);

                if (m_sampleCnt > m_sampleCntMax)
                {
                    status = 0;
                    ADQNDS_MSG_WARNLOG_PV(status, "Chosen number of samples was higher than the maximum number of samples.");
                    m_sampleCnt = m_sampleCntMax;
                }
            }
        }

        m_sampleCntPV.push(now, m_sampleCnt);

        m_sampleCntTotal = m_sampleCnt * m_recordCnt;
        m_sampleCntTotalPV.push(now, m_sampleCntTotal);

        // Trigger records to collect to update
        m_recordCntCollectPV.read(&now, &m_recordCntCollect);
        m_recordCntCollectPV.push(now, m_recordCntCollect);
    }
    else
    {
        m_recordCntPV.push(now, m_recordCnt);
        m_sampleCntPV.push(now, m_sampleCnt);
        m_sampleCntTotalPV.push(now, m_sampleCnt);

        // Trigger records to collect to update
        m_recordCntCollectPV.read(&now, &m_recordCntCollect);
        m_recordCntCollectPV.push(now, m_recordCntCollect);
    }
}

void ADQAIChannelGroup::commitRecordCountCollect(struct timespec const& now)
{
    m_recordCntCollectChanged = false;

    if (m_recordCntCollect > m_recordCnt)
    {
        m_recordCntCollect = m_recordCnt;
        ndsWarningStream(m_node) << "Number of records to collect cannot be higher than total number of records." << std::endl;
    }

    m_recordCntCollectPV.push(now, m_recordCntCollect);

    m_sampleCntTotal = m_sampleCnt * m_recordCntCollect;
    m_sampleCntTotalPV.push(now, m_sampleCntTotal);
}

void ADQAIChannelGroup::commitStreamTime(struct timespec const& now)
{
    m_streamTimeChanged = false;

    if (m_streamTime <= 0)
    {
        m_streamTime = 1.0;
        ADQNDS_MSG_WARNLOG_PV(0, "Streaming time can't be 0 seconds -> changed to 1.0 seconds.");
    }

    m_streamTimePV.push(now, m_streamTime);
}

void ADQAIChannelGroup::commitSWTrigEdge(struct timespec const& now)
{
    m_swTrigEdgeChanged = false;

    int status = m_adqInterface->SetTriggerEdge(m_trigMode + 1, m_swTrigEdge);
    ADQNDS_MSG_WARNLOG_PV(status, "SetTriggerEdge failed.");

    if (status)
    {
        unsigned int trigEdge = 0;
        status = m_adqInterface->GetTriggerEdge(m_trigMode + 1, &trigEdge);
        ADQNDS_MSG_WARNLOG_PV(status, "GetTriggerEdge failed.");
        if (status)
        {
            m_swTrigEdge = trigEdge;
            m_swTrigEdgePV.push(now, m_swTrigEdge);
        }
    }
}

void ADQAIChannelGroup::commitSampleSkip(struct timespec& now)
{
    m_sampleSkipChanged = false;
    std::string adqOption = m_adqInterface->GetCardOption();

    if (m_sampleSkip < 1)
    {
        m_sampleSkip = 1;
        ADQNDS_MSG_WARNLOG_PV(0, "Sample skip can't be less than 1 -> changed to 1.");
    }

    if (m_sampleSkip > 65536)
    {
        m_sampleSkip = 65536;
        ADQNDS_MSG_WARNLOG_PV(0, "Sample skip can't be more than 65536 -> changed to 65536.");
    }

    if ((adqOption.find("-2X") != std::string::npos) || (adqOption.find("-1X") != std::string::npos))
    {
        if (m_sampleSkip == 3)
        {
            m_sampleSkip = 4;
            ADQNDS_MSG_WARNLOG_PV(0, "Sample skip can't be 3 -> changed to 4.");
        }

        if (m_sampleSkip == 5 || m_sampleSkip == 6 || m_sampleSkip == 7)
        {
            m_sampleSkip = 8;
            ADQNDS_MSG_WARNLOG_PV(0, "Sample skip can't be 5, 6 or 7 -> changed to 8.");
        }
    }

    if ((adqOption.find("-4C") != std::string::npos) || (adqOption.find("-2C") != std::string::npos))
    {
        if (m_sampleSkip == 3)
        {
            m_sampleSkip = 4;
            ADQNDS_MSG_WARNLOG_PV(0, "Sample skip can't be 3 -> changed to 4.");
        }
    }

    int status = m_adqInterface->SetSampleSkip(m_sampleSkip);
    ADQNDS_MSG_WARNLOG_PV(status, "SetSampleSkip failed.");

    m_sampleSkip = m_adqInterface->GetSampleSkip();
    m_sampleSkipPV.push(now, m_sampleSkip);

    // Trigger sample rate with decimation to update
    double tmp = 0;
    m_sampRateDecPV.read(&now, &tmp);
    m_sampRateDecPV.push(now, tmp);
}

void ADQAIChannelGroup::commitSampleDec(struct timespec& now)
{
    m_sampleDecChanged = false;
    std::string adqOption = m_adqInterface->GetCardOption();

    if ((m_adqType == 714 || m_adqType == 14) && (adqOption.find("-FWSDR") != std::string::npos))
    {
        if (m_sampleDec < 0)
        {
            m_sampleDec = 0;
        }

        int status = m_adqInterface->SetSampleDecimation(m_sampleDec);
        ADQNDS_MSG_WARNLOG_PV(status, "SetSampleDecimation failed.");

        if (status)
        {
            m_sampleDec = m_adqInterface->GetSampleDecimation();
            m_sampleSkipPV.push(now, m_sampleDec);
        }

        // Trigger sample rate with decimation to update
        double tmp = 0;
        m_sampRateDecPV.read(&now, &tmp);
        m_sampRateDecPV.push(now, tmp);
    }
    else
    {
        ADQNDS_MSG_WARNLOG_PV(0, "Sample decimation is not supported on this device.");
    }
}

int ADQAIChannelGroup::armTrigger()
{
    std::lock_guard<std::mutex> lock(m_adqDevMutex);
    int status;
    if (m_trigMode != 2)
    {
        // Level trigger cannot be used for timestamp synchronization.
        status = m_adqInterface->ArmTimestampSync();
        ADQNDS_MSG_ERRLOG_PV(status, "ArmTimestampSync failed.");
    }
    if (m_masterMode == 2)
    {
        status = m_adqInterface->SetInternalTriggerSyncMode(m_masterMode);
        ADQNDS_MSG_ERRLOG_PV(status, "SetInternalTriggerSyncMode failed");
    }
    if ((m_masterMode != 2) && (m_trigMode == 3))
    {
        // To ensure good synchronisation of multiple modules.
        TraceOutWithTime(m_node, "SetInternalTriggerPeriod %u %u %u", m_internTrigPeriod, m_internTrigHighSamp, m_internTrigLowSamp);
        status = m_adqInterface->SetInternalTriggerPeriod(m_internTrigPeriod);
        ADQNDS_MSG_ERRLOG_PV(status, "SetInternalTriggerPeriod failed.");
        status = m_adqInterface->SetInternalTriggerHighLow(m_internTrigHighSamp, m_internTrigLowSamp);
        ADQNDS_MSG_ERRLOG_PV(status, "SetInternalTriggerHighLow failed.");
    }

    while (m_stateMachine.getLocalState() == nds::state_t::starting)
        SLEEP(1);

#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#endif
    TraceOutWithTime(m_node, "ArmTrigger");
    status = m_adqInterface->ArmTrigger();
    ADQNDS_MSG_ERRLOG_PV(status, "ArmTrigger failed.");
    return status;
}

void ADQAIChannelGroup::RecordTimeStampsAsCSV(std::vector<TimeStamp_t> const& TimeStampRecord, int32_t Record)
{
    std::string CSVFileName = m_node.getFullName();
    size_t ColonPos = CSVFileName.find(":"); // Illegal in a file name.
    if (ColonPos != std::string::npos)
        CSVFileName = CSVFileName.substr(0, ColonPos);
    CSVFileName += ".csv";
    FILE* CSVFile = fopen(CSVFileName.c_str(), "a+");
    if (CSVFile)
    {
        double PRETime = TimeStampRecord[0].m_ClockTimeStamp.tv_sec * 1000 + TimeStampRecord[0].m_ClockTimeStamp.tv_nsec / 1000000.0;
        fprintf(CSVFile, "Clock, Hardware, Start %s\n", ctime(&TimeStampRecord[0].m_ClockTimeStamp.tv_sec));
        for (int32_t CSVRecord = 0; CSVRecord < Record; CSVRecord++)
        {
            // Convert to ms.
            double NowTime = TimeStampRecord[CSVRecord].m_ClockTimeStamp.tv_sec * 1000 + TimeStampRecord[CSVRecord].m_ClockTimeStamp.tv_nsec / 1000000.0;
            fprintf(CSVFile, "%f, %f\n", NowTime - PRETime, TimeStampRecord[CSVRecord].m_TrigTimeStamp);
        }
        fclose(CSVFile);
    }
    else
        ndsWarningStream(m_node) << "CSV file could not be opened" << std::endl;
}

/* Data acquisition method for Multi-Record
*/
void ADQAIChannelGroup::daqMultiRecord()
{
    int acqRecTotal = 0;
    int status;
    streamingHeader_t* daqStreamHeaders[CHANNEL_COUNT_MAX];
    short* daqDataBuffer[CHANNEL_COUNT_MAX];
    double PicoSec = (m_adqType == 8) ? PicoSec25 : PicoSec125;
    std::vector<TimeStamp_t> TimeStampRecord(m_recordCnt);
    int32_t Record = 0;

    try
    {
        if (!allocateBuffers(daqDataBuffer, daqStreamHeaders, NULL))
            ADQNDS_MSG_ERRLOG_PV(0, "allocateBuffers failed.");

        TraceOutWithTime(m_node, "daqMultiRecord started");

        if (m_adqType == 8)
        {
            status = m_adqInterface->DaisyChainEnable(1);
            ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainEnable failed.");
        }

        {
            std::lock_guard<std::mutex> lock(m_adqDevMutex);
            TraceOutWithTime(m_node, "MultiRecordSetChannelMask %d", m_chanMask);
            status = m_adqInterface->MultiRecordSetChannelMask(m_chanMask);
            ADQNDS_MSG_ERRLOG_PV(status, "MultiRecordSetChannelMask failed.");

            status = m_adqInterface->MultiRecordSetup(m_recordCnt, m_sampleCnt);
            ADQNDS_MSG_ERRLOG_PV(status, "MultiRecordSetup failed.");
        }
        status = armTrigger();
        ADQNDS_MSG_ERRLOG_PV(status, "armTrigger failed.");

        for (Record = 0; Record < m_recordCnt; Record += m_recordCntCollect)
        {
            if (m_trigMode == 0)   // SW trigger
            {
                std::lock_guard<std::mutex> lock(m_adqDevMutex);
                for (int i = 0; i < m_recordCntCollect; ++i)
                {
                    status = m_adqInterface->SWTrig();
                    ADQNDS_MSG_ERRLOG_PV(status, "SWTrig failed.");
                    SLEEP(1);
                }
            }
            {
#ifdef _DEBUG
                bool StreamOverflowReported = false;
#endif
                while (true)
                {
                    {
                        std::lock_guard<std::mutex> lock(m_adqDevMutex);
                        std::lock_guard<std::mutex> staticlock(m_StaticMutex);
                        acqRecTotal = m_adqInterface->GetAcquiredRecords();
                    }
                    if (acqRecTotal >= Record + m_recordCntCollect)
                    {
                        break;
                    }
                    if ((m_stateMachine.getLocalState() != nds::state_t::running) &&
                        (m_stateMachine.getLocalState() != nds::state_t::starting))
                    {
                        ndsWarningStream(m_node) << "daqMultiRecord exited on frame " << Record << " after GetAcquiredRecords() with state " << int(m_stateMachine.getLocalState()) << " for " << m_node.getFullName() << std::endl;
                        break;
                    }

                    SLEEP(1);
                    {
#ifdef _DEBUG
                        std::lock_guard<std::mutex> lock(m_adqDevMutex);
                        if (m_adqType == 8)
                            DaisyChainGetStatus(__LINE__);
                        if ((!StreamOverflowReported) && (m_adqInterface->GetStreamOverflow()))
                        {
                            StreamOverflowReported = true;
                            ndsWarningStream(m_node) << "GetStreamOverflow detected for " << m_node.getFullName() << std::endl;
                        }
#endif
                    }
                };
            }

            std::ostringstream textTmp;
            textTmp << "INFO: GetAcquiredRecords: " << acqRecTotal;
            std::string textForPV(textTmp.str());
            ndsInfoStream(m_node) << textForPV << std::endl;

            if ((m_stateMachine.getLocalState() != nds::state_t::running) &&
                (m_stateMachine.getLocalState() != nds::state_t::starting))
            {
                ADQNDS_MSG_WARNLOG_PV(0, "daqMultiRecord requested to exit");
                break;
            }

            // We stop monitoring the hardware temperatures etc., in order to maximimise real-time performance.
            {
                std::lock_guard<std::mutex> lock(m_adqDevMutex);
                status = m_adqInterface->GetDataWHTS((void**)daqDataBuffer, daqStreamHeaders[0], NULL,
                    m_sampleCnt, sizeof(short), Record, m_recordCntCollect,
                    m_chanMask, 0, m_sampleCnt, ADQ_TRANSFER_MODE_NORMAL);
                ADQNDS_MSG_ERRLOG_PV(status, "GetDataWHTS failed.");
            }

            struct timespec now = { 0, 0 };
            clock_gettime(CLOCK_REALTIME, &now);
            for (int32_t Collected = 0; Collected < m_recordCntCollect; Collected++)
            {
                // time in ms.
                double trigTimeStamp = daqStreamHeaders[0][Collected].timeStamp * PicoSec;
                if ((m_masterMode != 2) && (Record == 0))
                {
                    // Timestamp issue: the first time stamp recorded is not reset.
                    if ((m_trigMode == 1) || (m_trigMode == 2))
                        m_PRETime = trigTimeStamp;
                    else
                    {
                        m_PRETime = 0;
                        trigTimeStamp = 0;
                    }
                }
                m_trigTimeStamp = trigTimeStamp - m_PRETime;
                m_trigTimeStampPV.push(now, m_trigTimeStamp);
                TimeStampRecord[Record + Collected].m_ClockTimeStamp = now;
                TimeStampRecord[Record + Collected].m_TrigTimeStamp = m_trigTimeStamp;
                for (unsigned int chan = 0; chan < m_chanCnt; ++chan)
                {
                    if ((m_chanMask & (1 << chan)) != 0)
                        // Read buffers by each channel and send them to DATA PVs
                        m_AIChannelsPtr[chan]->readData(daqDataBuffer[chan], m_sampleCnt*m_recordCntCollect, now);
                }
                TraceOutWithTime(m_node, "%s record %d timestamp %f", (m_masterMode == 1) ? "Master" : "Slave ", Record + Collected, m_trigTimeStamp);
            }
            std::vector<std::vector<int64_t>> RecordStart(m_recordCntCollect);
            std::vector<ADQDaisyChainTriggerInformation> trig_info(m_recordCntCollect);
            std::vector <std::vector<double>> ExtendedPrecision(m_recordCntCollect);
            if ((m_masterMode == 1) && (m_device_info.size() > 0))
            {
                for (int32_t Record = 0; Record < m_recordCntCollect; Record++)
                {
                    RecordStart[Record].resize(m_device_info.size());
                    trig_info[Record].RecordStart = &(RecordStart[Record][0]);
                    ExtendedPrecision[Record].resize(m_device_info.size());
                    trig_info[Record].ExtendedPrecision = &(ExtendedPrecision[Record][0]);
                }
                {
                    std::lock_guard<std::mutex> lock(m_adqDevMutex);
                    status = m_adqInterface->DaisyChainGetTriggerInformation(
                        23, m_levelTrigEdge, m_levelTrigLvl, m_levelTrigChan, Record, m_recordCntCollect, m_sampleCnt,
                        &(m_device_info[0]), int(m_device_info.size()), &(trig_info[0]));
                    ADQNDS_MSG_ERRLOG_PV(status, "DaisyChainGetTriggerInformation failed.");
                }
            }
            for (int32_t Collected = 0; Collected < m_recordCntCollect; Collected++)
                if ((m_adqType == 8) && (m_masterMode == 1) && (m_device_info.size() > 0))
                {
                    m_daisyTimeStamp = (trig_info[Collected].Timestamp * PicoSec) - m_PRETime;
                    m_daisyTimeStampPV.push(now, m_daisyTimeStamp);
                    m_daisyRecordStart.resize(m_device_info.size());
                    for (size_t Module = 0; Module < m_daisyRecordStart.size(); Module++)
                        m_daisyRecordStart[Module] = RecordStart[Collected][Module];
                    m_daisyRecordStartPV.push(now, m_daisyRecordStart);
                }
            m_recordCntPV.push(now, Record + 1);
        }
    }
    catch (nds::NdsError const&) {
    }

    double MaxTimeStampDeviation = 0;
    double PRETime = TimeStampRecord[0].m_ClockTimeStamp.tv_sec * 1000 + TimeStampRecord[0].m_ClockTimeStamp.tv_nsec / 1000000.0;
    for (int32_t CSVRecord = 0; CSVRecord < Record; CSVRecord++)
    {
        double NowTime = TimeStampRecord[CSVRecord].m_ClockTimeStamp.tv_sec * 1000 + TimeStampRecord[CSVRecord].m_ClockTimeStamp.tv_nsec / 1000000.0;
        NowTime = NowTime - PRETime;
        MaxTimeStampDeviation = std::max(MaxTimeStampDeviation, fabs(NowTime - TimeStampRecord[CSVRecord].m_TrigTimeStamp));
    }
    if (MaxTimeStampDeviation > 10) {
        ndsWarningStream(m_node) << "maximum time stamp deviation was " << MaxTimeStampDeviation << " for " << m_node.getFullName() << std::endl;
    }
    if (((Record > 0) && (Record < m_recordCnt)) || (MaxTimeStampDeviation > 10))
        RecordTimeStampsAsCSV(TimeStampRecord, Record);

    setTriggerIdleState();
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        if (m_adqType == 8)
        {
            status = m_adqInterface->DaisyChainEnable(0);
            ADQNDS_MSG_WARNLOG_PV(status, "DaisyChainEnable failed.");
        }
        status = m_adqInterface->MultiRecordClose();
        ADQNDS_MSG_WARNLOG_PV(status, "MultiRecordClose failed.");
    }

    ndsInfoStream(m_node) << "INFO: Acquisition finished." << std::endl;

    try
    {
        if ((m_stateMachine.getLocalState() == nds::state_t::running) ||
            (m_stateMachine.getLocalState() == nds::state_t::stopping))
            m_stateMachine.setState(nds::state_t::on);
    }
    catch (nds::StateMachineNoSuchTransition& /*error*/)
    {
        ADQNDS_MSG_WARNLOG_PV(status, "State transition error.");
    }

    commitChanges(true);
}

/* Data acquisition method for continuous streaming
*/
void ADQAIChannelGroup::daqContinStream()
{
    int status;
    unsigned int bufferSize, buffersFilled;
    double elapsedSeconds;
    int bufferStatusLoops;
    unsigned int samplesAdded[CHANNEL_COUNT_MAX];
    unsigned int headersAdded[CHANNEL_COUNT_MAX];
    unsigned int headerStatus[CHANNEL_COUNT_MAX];
    unsigned int samplesAddedTotal;
    unsigned int streamCompleted = 0;
    streamingHeader_t* daqStreamHeaders[CHANNEL_COUNT_MAX];
    short* daqDataBuffer[CHANNEL_COUNT_MAX];

    try
    {
        if (m_adqType == 714 || m_adqType == 14)
        {
            bufferSize = BUFFERSIZE_ADQ14;
        }
        if (m_adqType == 7)
        {
            bufferSize = BUFFERSIZE_ADQ7;
        }
        if (m_adqType == 8)
        {
            bufferSize = BUFFERSIZE_ADQ8;
        }

        for (unsigned int chan = 0; chan < m_chanCnt; chan++)
        {
            if (m_chanMask & (1 << chan))
            {
                daqDataBuffer[chan] = (short*)malloc((size_t)bufferSize);
                daqStreamHeaders[chan] = (streamingHeader_t*)malloc((size_t)bufferSize);
            }
        }

        while (!(m_stateMachine.getLocalState() == nds::state_t::stopping) && !streamCompleted)
        {
            std::lock_guard<std::mutex> lock(m_adqDevMutex);
            status = m_adqInterface->StopStreaming();
            ADQNDS_MSG_ERRLOG_PV(status, "StopStreaming failed.");

            status = m_adqInterface->ContinuousStreamingSetup(m_chanMask);
            ADQNDS_MSG_WARNLOG_PV(status, "ContinuousStreamingSetup failed.");

            // Start streaming
            samplesAddedTotal = 0;

            status = m_adqInterface->StartStreaming();
            ADQNDS_MSG_ERRLOG_PV(status, "StartStreaming failed.");

            timerStart();

            if (m_trigMode == 0)
            {

                for (int i = 0; i < m_recordCnt; ++i)
                {
                    status = m_adqInterface->SWTrig();
                    ADQNDS_MSG_ERRLOG_PV(status, "SWTrig failed.");
                }
            }

            while (!streamCompleted)
            {
                bufferStatusLoops = 0;
                buffersFilled = 0;

                while (buffersFilled == 0 && status)
                {
                    status = m_adqInterface->GetTransferBufferStatus(&buffersFilled);
                    ADQNDS_MSG_ERRLOG_PV(status, "GetTransferBufferStatus failed.");

                    if (buffersFilled == 0)
                    {
                        SLEEP(10);
                        bufferStatusLoops++;

                        if (bufferStatusLoops > 2000)
                        {
                            // If the DMA transfer is taking too long, we should flush the DMA buffer before it times out. The timeout defaults to 60 seconds.
                            ndsInfoStream(m_node) << "INFO: Timeout, flushing DMA..." << std::endl;
                            status = m_adqInterface->FlushDMA();
                            ADQNDS_MSG_ERRLOG_PV(status, "FlushDMA failed.");
                        }
                    }

                    if (m_stateMachine.getLocalState() == nds::state_t::stopping)
                    {
                        ndsInfoStream(m_node) << "INFO: Data acquisition was stopped." << std::endl;
                        goto finish;
                    }
                }

                for (unsigned int buf = 0; buf < buffersFilled; buf++)
                {
                    ndsInfoStream(m_node) << "INFO: Receiving data..." << std::endl;
                    status = m_adqInterface->GetDataStreaming((void**)daqDataBuffer,
                        (void**)daqStreamHeaders,
                        m_chanMask,
                        samplesAdded,
                        headersAdded,
                        headerStatus);
                    ADQNDS_MSG_ERRLOG_PV(status, "GetDataStreaming failed.");

                    for (unsigned int chan = 0; chan < m_chanCnt; ++chan)
                    {
                        samplesAddedTotal += samplesAdded[chan];
                    }

                    // Read data from each channel's buffer
                    for (unsigned int chan = 0; chan < m_chanCnt; ++chan)
                    {
                        int32_t sampleCnt = samplesAdded[chan];
                        struct timespec now;
                        clock_gettime(CLOCK_REALTIME, &now);
                        m_AIChannelsPtr[chan]->readData(daqDataBuffer[chan], sampleCnt, now);
                    }
                }

                elapsedSeconds = timerSpentTimeMs() / 1000;

                if (elapsedSeconds > m_streamTime)
                {
                    streamCompleted = 1;
                    ndsInfoStream(m_node) << "INFO: Acquisition finished due to achieved target stream time." << std::endl;
                }

                status = m_adqInterface->GetStreamOverflow();
                if (status)
                {
                    streamCompleted = 1;
                    ADQNDS_MSG_WARNLOG_PV(0, "GetStreamOverflow detected.");
                }
            }
        }
    }
    catch (nds::NdsError const&) {
    }
    finish:
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);
        status = m_adqInterface->StopStreaming();
        ADQNDS_MSG_WARNLOG_PV(status, "StopStreaming failed.");
    }

    ndsInfoStream(m_node) << "INFO: Acquisition finished." << std::endl;

    for (unsigned int chan = 0; chan < m_chanCnt; ++chan)
    {
        if (daqDataBuffer[chan])
            free(daqDataBuffer[chan]);
        if (daqStreamHeaders[chan])
            free(daqStreamHeaders[chan]);
    }

    try
    {
        if ((m_stateMachine.getLocalState() == nds::state_t::running) ||
            (m_stateMachine.getLocalState() == nds::state_t::stopping))
            m_stateMachine.setState(nds::state_t::on);
    }
    catch (nds::StateMachineNoSuchTransition& /*error*/)
    {
        ADQNDS_MSG_WARNLOG_PV(status, "State transition error.");
    }

    commitChanges(true);
}

/* Data acquisition method for raw streaming.
 * Works per channel.
 */
void ADQAIChannelGroup::daqRawStream()
{
    int status;
    unsigned int bufferSize, buffersFilled;
    unsigned int m_sampleCntCollect;
    short* daqRawDataBuffer = NULL;

    try
    {
        if (m_adqType == 714 || m_adqType == 14)
        {
            bufferSize = BUFFERSIZE_ADQ14;
        }
        if (m_adqType == 7)
        {
            bufferSize = BUFFERSIZE_ADQ7;
        }
        if (m_adqType == 8)
        {
            bufferSize = BUFFERSIZE_ADQ8;
        }

        {
            std::lock_guard<std::mutex> lock(m_adqDevMutex);
            status = m_adqInterface->SetTransferBuffers(CHANNEL_COUNT_MAX, bufferSize);
            ADQNDS_MSG_ERRLOG_PV(status, "SetTransferBuffers failed.");

            status = m_adqInterface->SetStreamStatus(1);
            ADQNDS_MSG_ERRLOG_PV(status, "SetStreamStatus to 1 (Stream enabled) failed.");
            status = m_adqInterface->SetStreamConfig(2, 1);
            ADQNDS_MSG_ERRLOG_PV(status, "SetStreamConfig to 2-1 (Raw without headers) failed.");
            status = m_adqInterface->SetStreamConfig(3, m_chanInt);
            ADQNDS_MSG_ERRLOG_PV(status, "SetStreamConfig to 3-x (channel mask) failed.");
        }

        ndsInfoStream(m_node) << "INFO: Receving data..." << std::endl;

        // Allocate temporary buffer for streaming data
        daqRawDataBuffer = (signed short*)malloc(bufferSize * sizeof(signed short));

        {
            std::lock_guard<std::mutex> lock(m_adqDevMutex);
            status = m_adqInterface->StopStreaming();
            ADQNDS_MSG_ERRLOG_PV(status, "StopStreaming failed.");

            status = m_adqInterface->StartStreaming();
            ADQNDS_MSG_ERRLOG_PV(status, "StartStreaming failed.");
        }

        m_sampleCntCollect = bufferSize;

        while (!(m_stateMachine.getLocalState() == nds::state_t::stopping) && (m_sampleCntCollect > 0))
        {
            unsigned int bufferSampCnt;
            {
                std::lock_guard<std::mutex> lock(m_adqDevMutex);
                do
                {
                    status = m_adqInterface->GetTransferBufferStatus(&buffersFilled);
                    ADQNDS_MSG_ERRLOG_PV(status, "GetTransferBufferStatus failed.");

                } while ((buffersFilled == 0) && (status));

                status = m_adqInterface->CollectDataNextPage();
                ADQNDS_MSG_ERRLOG_PV(status, "CollectDataNextPage failed.");

                bufferSampCnt = MIN(m_adqInterface->GetSamplesPerPage(), m_sampleCntCollect);
                ndsInfoStream(m_node) << "GetSamplesPerPage " << m_adqInterface->GetSamplesPerPage() << std::endl;

                status = m_adqInterface->GetStreamOverflow();
                if (status)
                {
                    status = 0;
                    ADQNDS_MSG_ERRLOG_PV(status, "Streaming overflow detected.");
                }
            }

            // Buffer all data in RAM before writing to disk
            // Data format is set to 16 bits, so buffer size is Samples*2 bytes
            memcpy((void*)&daqRawDataBuffer[bufferSize - m_sampleCntCollect],
                m_adqInterface->GetPtrStream(),
                bufferSampCnt * sizeof(signed short));
            m_sampleCntCollect -= bufferSampCnt;
        }

        m_sampleCntCollect = bufferSize;

        ndsInfoStream(m_node) << "INFO: Writing data to PVs..." << std::endl;

        // Read buffer and send its data to DATA PV
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        m_AIChannelsPtr[m_chanActive]->readData(daqRawDataBuffer, m_sampleCntCollect, now);
    }
    catch (nds::NdsError const&) {
    }
    {
        std::lock_guard<std::mutex> lock(m_adqDevMutex);

        // Reset settings from Raw Streaming to normal streaming
        status = m_adqInterface->SetStreamStatus(0);
        ADQNDS_MSG_WARNLOG_PV(status, "SetStreamStatus to 0 (Stream disabled) failed.");
        status = m_adqInterface->SetStreamConfig(2, 0);
        ADQNDS_MSG_WARNLOG_PV(status, "SetStreamConfig to 2-1 (With headers) failed.");
        status = m_adqInterface->SetStreamConfig(3, 0);
        ADQNDS_MSG_WARNLOG_PV(status, "SetStreamConfig to 3-x (channel mask) failed.");

        status = m_adqInterface->StopStreaming();
        ADQNDS_MSG_WARNLOG_PV(status, "StopStreaming failed.");
    }

    ndsInfoStream(m_node) << "INFO: Acquisition finished." << std::endl;

    if (daqRawDataBuffer)
        free(daqRawDataBuffer);

    try
    {
        if ((m_stateMachine.getLocalState() == nds::state_t::running) ||
            (m_stateMachine.getLocalState() == nds::state_t::stopping))
            m_stateMachine.setState(nds::state_t::on);
    }
    catch (nds::StateMachineNoSuchTransition& /*error*/)
    {
        ADQNDS_MSG_WARNLOG_PV(status, "State transition error.");
    }

    commitChanges(true);
}

ADQAIChannelGroup::~ADQAIChannelGroup()
{
}

std::string ADQAIChannelGroup::utc_system_timestamp(struct timespec const& now, char sep) const
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


void ADQAIChannelGroup::ThrowException(std::string const& text)
{
    struct timespec now = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &now);
    setLogMsg(now, text);
    ndsErrorStream(m_node) << "ERROR: " << utc_system_timestamp(now, ' ') << " " << m_node.getFullExternalName() << " " << text << std::endl;
    throw nds::NdsError(text);
}

#ifdef TRACEDEBUG
void ADQAIChannelGroup::TraceOutWithTime(nds::Port& node, const char *pszFormat, ...)
{
    std::lock_guard<std::mutex> staticlock(m_StaticMutex);
    static const int TRACE_BUFF_SIZE = 512;
    static char szTraceBuffer1[TRACE_BUFF_SIZE];
    static struct timespec StartTime = { 0, 0 };
    if ((StartTime.tv_sec == 0) && (StartTime.tv_nsec == 0))
        clock_gettime(CLOCK_REALTIME, &StartTime);

    va_list args;

    va_start(args, pszFormat);
    vsnprintf(szTraceBuffer1, TRACE_BUFF_SIZE - 1, pszFormat, args);
    va_end(args);

    struct timespec now = { 0, 0 };
    clock_gettime(CLOCK_REALTIME, &now);
    time_t ElapsedS = now.tv_sec - StartTime.tv_sec;
    long ElapsedNS = now.tv_nsec - StartTime.tv_nsec;
    double ElapsedMS = ElapsedS * 1000 + ElapsedNS / 1000000.0;
    char szTraceBuffer2[TRACE_BUFF_SIZE + 20];
    snprintf(szTraceBuffer2, sizeof(szTraceBuffer2) - 1, "[%6.1f] %s %s",
        ElapsedMS, m_node.getFullName().c_str(), szTraceBuffer1);
#if  defined(_DEBUG) && defined(_WIN32)
    ::OutputDebugString(szTraceBuffer2);
    ::OutputDebugString("\n");
#else
    ndsWarningStream(node) << std::string(szTraceBuffer) << std::endl;
#endif
}
#endif // TRACEDEBUG
