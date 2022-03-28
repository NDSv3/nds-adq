//
// Copyright (c) 2018 Cosylab d.d.
// This software is distributed under the terms found
// in file LICENSE.txt that is included with this distribution.
//

#ifndef ADQAICHANNELGROUP_H
#define ADQAICHANNELGROUP_H

#include "ADQAIChannel.h"
#include "ADQDefinition.h"
#include "ADQInfo.h"

#include <mutex>
#include <nds3/nds.h>

/** @file ADQAIChannelGroup.h
 * @brief This file defines ADQAIChannelGroup class and streamingHeader_t struct.
 */

/** @struct streamingHeader_t
 * @brief This record header structure is used in Triggered streaming DAQ mode.
 */
typedef struct
{
    /** @var recordStatus
    * @brief Record status.
    */
    unsigned char recordStatus;

    /** @var userID
    * @brief User ID.
    */
    unsigned char userID;

    /** @var chan
    * @brief The name/number of the channel from which the record is acquired.
    */
    unsigned char chan;

    /** @var dataFormat
    * @brief Data format of the digitizer.
    */
    unsigned char dataFormat;

    /** @var serialNumber
    * @brief Digitizer's serial number.
    */
    unsigned int serialNumber;

    /** @var recordNumber
    * @brief The number of the passed record.
    */
    unsigned int recordNumber;

    /** @var samplePeriod
    * @brief Sample period (1/rate).
    */
    unsigned int samplePeriod;

    /** @var timeStamp
    * @brief Time when record was passed.
    */
    unsigned long long timeStamp;

    /** @var recordStart
    * @brief Record start.
    */
    unsigned long long recordStart;

    /** @var recordLength
    * @brief Number of samples in the record.
    */
    unsigned int recordLength;

    /** @var reserved
    * @brief Reserved placement.
    */
    unsigned int reserved;
} streamingHeader_t;

/** @class ADQAIChannelGroup
 * @brief This class handles majority of parameters for correct setup of each data acquisition mode.
 * Data acquisition is handled in this class. The state machine of the device is defined here.
 * Each digitizer's channel gets a representation by calling ADQCHannel constructor for N amount of physical channels.
 */
class ADQAIChannelGroup : public ADQInfo
{
public:
    /** @fn ADQAIChannelGroup(const std::string& name, nds::Node& parentNode, ADQInterface*& adqInterface);
     * @brief ADQAIChannelGroup class constructor.
     * @param name a name with which this class will register its child node.
     * @param parentNode a name of a parent node to which the child node will connect to.
     * @param adqInterface a pointer to the ADQ API interface created in the ADQDevice class.
     */
    ADQAIChannelGroup(const std::string& name, nds::Node& parentNode, 
        ADQInterface* adqInterface, int32_t masterEnumeration, int32_t thisEnumeration, int32_t nextEnumeration);
    virtual ~ADQAIChannelGroup();

    /** @var m_node
     * @brief ADQAIChannelGroup class child node that connects to the root node.
     */
    nds::Port m_node;

    /** @var m_stateMachine
    * @brief State machine of this class. Attached to the node.
     */
    nds::StateMachine m_stateMachine;

    /** @var m_AIChannelsPtr
    * @brief Vector of pointers to ADQAIChannel class instances.
    */
    std::vector<std::shared_ptr<ADQAIChannel>> m_AIChannelsPtr;

    /** @brief This function creates the most common type of PV and sets it readback PV to interrupt mode.
     */
    template <typename T>
    void createPv(const std::string& name, nds::PVDelegateIn<T>& pvRb,
                  std::function<void(ADQAIChannelGroup*, const timespec&, const T&)> setter,
                  std::function<void(ADQAIChannelGroup*, timespec*, T*)> getter);

    /** @brief This function creates the Enumeration type of PV and sets it readback PV to interrupt mode.
     */
    template <typename T>
    void createPvEnum(const std::string& name, nds::PVDelegateIn<T>& pvRb, nds::enumerationStrings_t enumList,
                      std::function<void(ADQAIChannelGroup*, const timespec&, const T&)> setter,
                      std::function<void(ADQAIChannelGroup*, timespec*, T*)> getter);

    /** @fn createPvRb
     * @brief This function creates and returns the readback PV.
     */
    template <typename T>
    nds::PVDelegateIn<T> createPvRb(const std::string& name, std::function<void(ADQAIChannelGroup*, timespec*, T*)> getter);

    /** @fn setDaqMode
     * @brief Sets the data acquisition mode.
     */
    void setDaqMode(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getDaqMode
     * @brief Gets the data acquisition mode.
     */
    void getDaqMode(timespec* pTimestamp, int32_t* pValue);

    /** @fn setTrigMode
     * @brief Sets the trigger mode.
     */
    void setTrigMode(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getExternTrigInputImpedance
     * @brief Gets the trig input impedance.
     */
    void getExternTrigInputImpedance(timespec* pTimestamp, int32_t* pValue);

    /** @fn setExternTrigInputImpedance
     * @brief Sets the trig input impedance.
     */
    void setExternTrigInputImpedance(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getTrigMode
     * @brief Gets the trigger mode.
     */
    void getTrigMode(timespec* pTimestamp, int32_t* pValue);

    /** @fn setMasterMode
     * @brief Sets the master mode for daisy-chain triggering.
     */
    void setMasterMode(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn setMasterMode
     * @brief Gets the master mode for daisy-chain triggering.
     */
    void getMasterMode(timespec* pTimestamp, int32_t* pValue);

    /** @fn getTrigTimeStamp
     * @brief Gets the trigger time stamp for the most recent record.
     */
    void getTrigTimeStamp(timespec* pTimestamp, double* pValue);

    /** @fn getDaisyRecordStart
     * @brief Gets the daisy time stamp(s) for the most recent record.
     */
    void getDaisyRecordStart(timespec* pTimestamp, std::vector<int32_t>* pValue);
    void getDaisyTimeStamp(timespec* pTimestamp, double* pValue);

    /** @fn setDbsBypass
     * @brief Sets if the DBS settings is bypassed (1) or not (0).
     */
    void setDbsBypass(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getDbsBypass
     * @brief Gets if the DBS settings is bypassed (1) or not (0).
     */
    void getDbsBypass(timespec* pTimestamp, int32_t* pValue);

    /** @fn setDbsDc
     * @brief Sets the DC target for the DBS.
     */
    void setDbsDc(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getDbsDc
     * @brief Gets the DC target for the DBS.
     */
    void getDbsDc(timespec* pTimestamp, int32_t* pValue);

    /** @fn setDbsLowSat
    * @brief Sets the lower saturation level for the DBS.
    */
    void setDbsLowSat(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getDbsLowSat
     * @brief Gets the lower saturation level for the DBS.
     */
    void getDbsLowSat(timespec* pTimestamp, int32_t* pValue);

    /** @fn setDbsUpSat
     * @brief Sets the upper saturation level for the DBS.
     */
    void setDbsUpSat(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getDbsUpSat
     * @brief Gets the upper saturation level for the DBS.
     */
    void getDbsUpSat(timespec* pTimestamp, int32_t* pValue);

    /** @fn setPatternMode
     * @brief Sets the pattern mode.
     */
    void setPatternMode(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getPatternMode
     * @brief Gets the pattern mode.
     */
    void getPatternMode(timespec* pTimestamp, int32_t* pValue);

    /** @fn setChanActive
     * @brief Sets which channels should be active for data acquisition.
     */
    void setChanActive(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getChanActive
     * @brief Gets which channels should be active for data acquisition.
     */
    void getChanActive(timespec* pTimestamp, int32_t* pValue);

    /** @fn getChanMask
     * @brief Gets the channel mask accordingly to chosen active channels.
     */
    void getChanMask(timespec* pTimestamp, int32_t* pValue);

    /** @fn setRecordCnt
     * @brief Sets the number of records to acquire.
     */
    void setRecordCnt(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getRecordCnt
     * @brief Gets the number of records to acquire.
     */
    void getRecordCnt(timespec* pTimestamp, int32_t* pValue);

    /** @fn setRecordCntCollect
     * @brief Sets the number of records to pass to the device (Multi-Record mode).
     */
    void setRecordCntCollect(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getRecordCntCollect
     * @brief Gets the number of records to pass to the device (Multi-Record mode).
     */
    void getRecordCntCollect(timespec* pTimestamp, int32_t* pValue);

    /** @fn setSampleCnt
     * @brief Sets the number of samples per record.
     */
    void setSampleCnt(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getSampleCnt
     * @brief Gets the number of samples per record.
     */
    void getSampleCnt(timespec* pTimestamp, int32_t* pValue);

    /** @fn getSampleCntMax
     * @brief Gets the maximum number of samples per record accordingly to the number of records (Multi-Record mode).
     */
    void getSampleCntMax(timespec* pTimestamp, int32_t* pValue);

    /** @fn getSamplesTotal
     * @brief Gets a total number of samples to acquire (Number of records * Number of samples).
     */
    void getSamplesTotal(timespec* pTimestamp, int32_t* pValue);

    /** @fn setSampleSkip
     * @brief Sets the sample skip.
     */
    void setSampleSkip(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getSampleSkip
     * @brief Gets the sample skip.
     */
    void getSampleSkip(timespec* pTimestamp, int32_t* pValue);

    /** @fn setSampleDec
     * @brief Gets the data acquisition mode..
     */
    void setSampleDec(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getSampleDec
     * @brief Sets the sample decimation (-FWSDR digitizers only).
     */
    void getSampleDec(timespec* pTimestamp, int32_t* pValue);

    /** @fn setPreTrigSamp
     * @brief Sets the number of pre-trigger samples (Multi-Record and Triggered streaming mode).
     */
    void setPreTrigSamp(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getPreTrigSamp
     * @brief Gets the number of pre-trigger samples (Multi-Record and Triggered streaming mode).
     */
    void getPreTrigSamp(timespec* pTimestamp, int32_t* pValue);

    /** @fn setTrigHoldOffSamp
     * @brief Sets the number of hold-off samples (Multi-Record and Triggered streaming mode).
     */
    void setTrigHoldOffSamp(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getTrigHoldOffSamp
     * @brief Gets the number of hold-off samples (Multi-Record and Triggered streaming mode).
     */
    void getTrigHoldOffSamp(timespec* pTimestamp, int32_t* pValue);

    /** @fn setClockSrc
     * @brief Sets the clock source.
     */
    void setClockSrc(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getClockSrc
     * @brief Gets the clock source.
     */
    void getClockSrc(timespec* pTimestamp, int32_t* pValue);

    /** @fn setClockRefOut
     * @brief Enables (1) or disables (0) clock reference output.
     */
    void setClockRefOut(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getClockRefOut
     * @brief Gets the clock reference output.
     */
    void getClockRefOut(timespec* pTimestamp, int32_t* pValue);

    /** @fn setTimeout
     * @brief Sets the flush timeout (Triggered streaming mode).
     */
    void setTimeout(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getTimeout
     * @brief Gets the flush timeout (Triggered streaming mode).
     */
    void getTimeout(timespec* pTimestamp, int32_t* pValue);

    /** @fn setStreamTime
     * @brief Sets the streaming time (Continuous streaming mode).
     */
    void setStreamTime(const timespec& pTimestamp, const double& pValue);

    /** @fn getStreamTime
     * @brief Gets the streaming time (Continuous streaming mode).
     */
    void getStreamTime(timespec* pTimestamp, double* pValue);

    /** @fn setSWTrigEdge
     * @brief Sets the SW trigger edge.
     */
    void setSWTrigEdge(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getSWTrigEdge
     * @brief Gets the SW trigger edge.
     */
    void getSWTrigEdge(timespec* pTimestamp, int32_t* pValue);

    /** @fn setLevelTrigLvl
     * @brief Sets the trigger level of Level trigger.
     */
    void setLevelTrigLvl(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getLevelTrigLvl
     * @brief Gets the trigger level of Level trigger.
     */
    void getLevelTrigLvl(timespec* pTimestamp, int32_t* pValue);

    /** @fn setLevelTrigEdge
     * @brief Sets the Level trigger edge.
     */
    void setLevelTrigEdge(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getLevelTrigEdge
     * @brief Gets the Level trigger edge.
     */
    void getLevelTrigEdge(timespec* pTimestamp, int32_t* pValue);

    /** @fn setLevelTrigChan
     * @brief Sets the Level trigger channel.
     */
    void setLevelTrigChan(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getLevelTrigChan
     * @brief Gets the Level trigger channel.
     */
    void getLevelTrigChan(timespec* pTimestamp, int32_t* pValue);

    /** @fn setLevelTrigChanMask
     * @brief Sets the Level trigger channel mask directly (bypassing setLevelTrigChan).
     */
    void setLevelTrigChanMask(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getLevelTrigChanMask
     * @brief Gets the Level trigger channel mask.
     */
    void getLevelTrigChanMask(timespec* pTimestamp, int32_t* pValue);

    /** @fn setExternTrigDelay
     * @brief Sets the External trigger delay.
     */
    void setExternTrigDelay(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getExternTrigDelay
     * @brief Gets the External trigger delay.
     */
    void getExternTrigDelay(timespec* pTimestamp, int32_t* pValue);

    /** @fn setExternTrigThreshold
     * @brief Sets the External trigger treshold.
     */
    void setExternTrigThreshold(const timespec& pTimestamp, const double& pValue);

    /** @fn getExternTrigThreshold
     * @brief Gets the External trigger treshold.
     */
    void getExternTrigThreshold(timespec* pTimestamp, double* pValue);

    /** @fn setExternTrigEdge
     * @brief Sets the External trigger edge.
     */
    void setExternTrigEdge(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getExternTrigEdge
     * @brief Gets the External trigger edge.
     */
    void getExternTrigEdge(timespec* pTimestamp, int32_t* pValue);

    /** @fn setInternTrigHighSamp
     * @brief Sets the Internal trigger high sample length.
     */
    void setInternTrigHighSamp(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getInternTrigHighSamp
     * @brief Gets the Internal trigger high sample length.
     */
    void getInternTrigHighSamp(timespec* pTimestamp, int32_t* pValue);

    /** @fn getInternTrigLowSamp
     * @brief Gets the Internal trigger low sample length.
     */
    void getInternTrigLowSamp(timespec* pTimestamp, int32_t* pValue);

    /** @fn setInternTrigFreq
     * @brief Sets the Internal trigger frequency.
     */
    void setInternTrigFreq(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getInternTrigFreq
     * @brief Gets the Internal trigger frequency.
     */
    void getInternTrigFreq(timespec* pTimestamp, int32_t* pValue);

    /** @fn setInternTrigEdge
     * @brief Sets the Internal trigger edge.
     */
    void setInternTrigEdge(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getInternTrigEdge
     * @brief Gets the Internal trigger edge.
     */
    void getInternTrigEdge(timespec* pTimestamp, int32_t* pValue);

    /** @fn getLogMsg
     * @brief Gets the log messages.
     */
    void getLogMsg(timespec* pTimestamp, std::string* pValue);
    void setLogMsg(const timespec& pTimestamp, std::string const& pValue);

    /** @fn getEnumerationOrder
     * @brief Gets the bus enumeration order of this module.
     */
    void getMasterEnumeration(timespec* pTimestamp, int32_t* pValue);
    void getThisEnumeration(timespec* pTimestamp, int32_t* pValue);
    void getNextEnumeration(timespec* pTimestamp, int32_t* pValue);

    /** @fn setDaisyPosition
     * @brief Sets the daisy chain position of this module.
     */
    void setDaisyPosition(const timespec& pTimestamp, const std::vector<int32_t>& pValue);

    /** @fn getDaisyPosition
     * @brief Gets the daisy chain position of this module.
     */
    void getDaisyPosition(timespec* pTimestamp, std::vector<int32_t>* pValue);

    /** @fn setsync_immediate
     * @brief Sets the sync_immidate daisy chain attrribute of this module.
     */
    void setsync_immediate(const timespec& pTimestamp, const int32_t& pValue);

    /** @fn getsync_immediate
     * @brief Gets the sync_immidate daisy chain attrribute of this module.
     */
    void getsync_immediate(timespec* pTimestamp, int32_t* pValue);

    /** @fn onSwitchOn
     * @brief Sets the state machine to state ON.
     */
    void onSwitchOn();

    /** @fn onSwitchOff
     * @brief Sets the state machine to state OFF.
     */
    void onSwitchOff();

    /** @fn onStart
     * @brief Sets the state machine to state RUNNING and starts data acquisition.
     */
    void onStart();

    /** @fn onStop
     * @brief Stops the data acquisition and sets the state machine to state ON.
     */
    void onStop();

    /** @fn recover
     * @brief State machine function. Not supported.
     */
    void recover();

    /** @fn allowChange
     * @brief Allows the state machine to switch to a new state.
     */
    bool allowChange(const nds::state_t currentLocal, const nds::state_t currentGlobal, const nds::state_t nextLocal);
 
    int allocateBuffers(short* (&daqDataBuffer)[CHANNEL_COUNT_MAX], streamingHeader_t* (&daqStreamHeaders)[CHANNEL_COUNT_MAX], short* (*daqLeftoverSamples)[CHANNEL_COUNT_MAX]);

    /** @fn daqTrigStream
     * @brief This method processes Triggered streaming data acquisition.
     */
    void daqTrigStream();

    /** @fn daqMultiRecord
     * @brief This method processes Multi-Record data acquisition.
     */
    void daqMultiRecord();

    /** @fn daqContinStream
     * @brief This method processes Continuous streaming data acquisition.
     */
    void daqContinStream();

    /** @fn daqRawStream
     * @brief This method processes Raw streaming data acquisition.
     */
    void daqRawStream();

private:
    int32_t m_masterEnumeration;
    int32_t m_thisEnumeration;
    int32_t m_nextEnumeration;
    unsigned m_DaisyChainStatus;
    std::vector<int32_t> m_daisyPosition;
    bool m_daisyPositionChanged;
    std::vector<ADQDaisyChainDeviceInformation> m_device_info;
    bool m_sync_immediateChanged;
    int m_sync_immediate;

    unsigned int m_chanCnt;
    int m_adqType;

    int32_t m_daqMode;
    bool m_daqModeChanged;

    int32_t m_patternMode;
    bool m_patternModeChanged;

    int32_t m_dbsBypass;
    bool m_dbsBypassChanged;
    int32_t m_dbsDc;
    bool m_dbsDcChanged;
    int32_t m_dbsLowSat;
    bool m_dbsLowSatChanged;
    int32_t m_dbsUpSat;
    bool m_dbsUpSatChanged;

    int32_t m_recordCnt;
    bool m_recordCntChanged;
    int32_t m_recordCntCollect;
    bool m_recordCntCollectChanged;

    int32_t m_sampleCnt;
    bool m_sampleCntChanged;
    int32_t m_sampleCntMax;
    int32_t m_sampleCntTotal;
    bool m_sampleSkipChanged;
    int32_t m_sampleSkip;
    bool m_sampleDecChanged;
    int32_t m_sampleDec;
    bool m_preTrigSampChanged;
    int32_t m_preTrigSamp;
    bool m_trigHoldOffSampChanged;
    int32_t m_trigHoldOffSamp;

    bool m_clockSrcChanged;
    int32_t m_clockSrc;
    bool m_clockRefOutChanged;
    int32_t m_clockRefOut;

    int32_t m_chanActive;
    bool m_chanActiveChanged;
    unsigned int m_chanInt;
    int32_t m_chanMask;   // Variations: 0x1 (A), 0x2 (B), 0x4 (C), 0x8 (D), 0x3 (AB), 0xC (CD), 0xF (ABCD)

    int32_t m_trigMode;   // 0 - SW, 1 external, 2 level, 3 internal, 4 daisychain, 5 PXIe.
    bool m_trigModeChanged;

    int32_t m_masterMode; // None, Master, Slave
    bool m_masterModeChanged;
    int32_t m_swTrigEdge;
    bool m_swTrigEdgeChanged;
    int32_t m_levelTrigLvl;
    bool m_levelTrigLvlChanged;
    int32_t m_levelTrigEdge;
    bool m_levelTrigEdgeChanged;
    int32_t m_levelTrigChan;
    bool m_levelTrigChanChanged;
    int32_t m_levelTrigChanMask;
    bool m_levelTrigChanMaskChanged;
    int32_t m_externTrigDelay;
    bool m_externTrigDelayChanged;
    double m_externTrigThreshold;
    bool m_externTrigThresholdChanged;
    int32_t m_externTrigEdge;
    bool m_externTrigEdgeChanged;
    int32_t m_externTrigInputImpedance;   // 0 - 50 Ohm, 1 high impedance.
    bool m_externTrigInputImpedanceChanged;
    int32_t m_internTrigHighSamp;
    bool m_internTrigHighSampChanged;
    int32_t m_internTrigLowSamp;
    int64_t m_SampleRate;
    int32_t m_internTrigFreq;
    bool m_internTrigFreqChanged;
    double m_trigTimeStamp;
    std::vector<int32_t> m_daisyRecordStart;
    double m_daisyTimeStamp;
    double m_PRETime;
    int32_t m_internTrigPeriod;
    int32_t m_internTrigEdge;
    bool m_internTrigEdgeChanged;

    bool m_timeoutChanged;
    int32_t m_timeout;

    bool m_streamTimeChanged;
    double m_streamTime;

    /** @brief This method processes changes are applied to parameters.
     *
     * @param calledFromDaqThread a flag that prevents this function to be processed when set to false.
     */
    void commitChanges(bool calledFromDaqThread = false);
    void commitDaqMode(struct timespec& now);
    void commitPatternMode(struct timespec const& now);
    void commitClockSource(struct timespec const& now);
    void commitTriggerMode(struct timespec const& now);
    void commitInternTrigHighSamp(struct timespec const& now);
    void commitInternTrigFreqChanged(struct timespec const& now);
    void commitExternTrigThreshold(struct timespec const& now);
    void commitExternTrigEdge(struct timespec const& now);
    void commitExternTrigImpedance(struct timespec const& now);
    void commitClockRefOut(struct timespec const& now);
    void commitLevelTrigEdge(struct timespec const& now);
    void commitInternTrigEdge(struct timespec const& now);
    void commitStreamTime(struct timespec const& now);
    void commitSampleSkip(struct timespec& now);
    void commitSampleDec(struct timespec& now);
    void commitTriggerDelayOrHoldoff(struct timespec const& now);
    void commitSWTrigEdge(struct timespec const& now);
    void commitExternTrigDelay(struct timespec const& now);
    void commitLevelTrigLvl(struct timespec const& now);
    void commitLevelTrigChanMask(struct timespec const& now);
    void commitDaisyChain(struct timespec const& now);
    void commitSetupDBS(struct timespec const& now);
    void commitActiveChan(struct timespec const& now);
    void commitRecordCount(struct timespec& now);
    void commitRecordCountCollect(struct timespec const& now);

    void setTriggerIdleState();
    int armTrigger();
    int commitDaisyChainTriggerSource(struct timespec const& now);
    void DaisyChainGetStatus(long Line);
#ifdef TRACEDEBUG
    void TraceOutWithTime(nds::Port& node, const char *pszFormat, ...);
#else
    void TraceOutWithTime(nds::Port& /*node*/, const char* /*pszFormat*/, ...) {
    }
#endif
    static std::mutex m_StaticMutex;

    std::string m_logMsg;
    nds::PVDelegateIn<std::string> m_logMsgPV;
    nds::PVDelegateIn<int32_t> m_masterEnumerationPV;
    nds::PVDelegateIn<int32_t> m_thisEnumerationPV;
    nds::PVDelegateIn<int32_t> m_nextEnumerationPV;
    nds::PVDelegateIn<std::vector<int32_t>> m_daisyPositionPV;
    nds::PVDelegateIn<int32_t> m_sync_immediatePV;
    nds::PVDelegateIn<int32_t> m_daqModePV;
    nds::PVDelegateIn<int32_t> m_patternModePV;
    nds::PVDelegateIn<int32_t> m_chanActivePV;
    nds::PVDelegateIn<int32_t> m_chanMaskPV;
    nds::PVDelegateIn<int32_t> m_dbsBypassPV;
    nds::PVDelegateIn<int32_t> m_dbsDcPV;
    nds::PVDelegateIn<int32_t> m_dbsLowSatPV;
    nds::PVDelegateIn<int32_t> m_dbsUpSatPV;
    nds::PVDelegateIn<int32_t> m_recordCntPV;
    nds::PVDelegateIn<int32_t> m_recordCntCollectPV;
    nds::PVDelegateIn<int32_t> m_sampleCntPV;
    nds::PVDelegateIn<int32_t> m_sampleCntMaxPV;
    nds::PVDelegateIn<int32_t> m_sampleCntTotalPV;
    nds::PVDelegateIn<int32_t> m_sampleSkipPV;
    nds::PVDelegateIn<int32_t> m_sampleDecPV;
    nds::PVDelegateIn<int32_t> m_preTrigSampPV;
    nds::PVDelegateIn<int32_t> m_trigHoldOffSampPV;
    nds::PVDelegateIn<int32_t> m_clockSrcPV;
    nds::PVDelegateIn<int32_t> m_clockRefOutPV;
    nds::PVDelegateIn<int32_t> m_trigModePV;
    nds::PVDelegateIn<int32_t> m_masterModePV;
    nds::PVDelegateIn<double>  m_trigTimeStampPV;
    nds::PVDelegateIn<std::vector<int32_t>> m_daisyRecordStartPV;
    nds::PVDelegateIn<double>  m_daisyTimeStampPV;
    nds::PVDelegateIn<int32_t> m_swTrigEdgePV;
    nds::PVDelegateIn<int32_t> m_levelTrigLvlPV;
    nds::PVDelegateIn<int32_t> m_levelTrigEdgePV;
    nds::PVDelegateIn<int32_t> m_levelTrigChanPV;
    nds::PVDelegateIn<int32_t> m_levelTrigChanMaskPV;
    nds::PVDelegateIn<int32_t> m_externTrigDelayPV;
    nds::PVDelegateIn<double> m_externTrigThresholdPV;
    nds::PVDelegateIn<int32_t> m_externTrigEdgePV;
    nds::PVDelegateIn<int32_t> m_externTrigInputImpedancePV;
    nds::PVDelegateIn<int32_t> m_internTrigHighSampPV;
    nds::PVDelegateIn<int32_t> m_internTrigLowSampPV;
    nds::PVDelegateIn<int32_t> m_internTrigFreqPV;
    nds::PVDelegateIn<int32_t> m_internTrigEdgePV;
    nds::PVDelegateIn<int32_t> m_timeoutPV;
    nds::PVDelegateIn<double> m_streamTimePV;

    nds::Thread m_daqThread;
    /** @def ADQNDS_MSG_WARNLOG_PV
     * @brief Macro for warning information in case of minor failures.
     * Used in ADQAIChannelGroup methods.
     * @param status status of the function that calls this macro.
     * @param text input information message.
     */
    void ADQNDS_MSG_WARNLOG_PV(int status, std::string const& text)
    {
        if (!status)
        {
            struct timespec now = { 0, 0 };
            clock_gettime(CLOCK_REALTIME, &now);
            m_logMsgPV.push(now, std::string(text));
            ndsWarningStream(m_node) << "WARNING: " << utc_system_timestamp(now, ' ') << " " << m_node.getFullExternalName() << " " << text << std::endl;
        }
    }
    /** @def ADQNDS_MSG_ERRLOG_PV
     * @brief Macro for informing the user about occured major failures and
     * stopping data acquisition. Used in ADQAIChannelGroup methods.
     * @param status status of the function that calls this macro.
     * @param text input information message.
     */
    struct TimeStamp_t {
        timespec m_ClockTimeStamp;
        double m_TrigTimeStamp;
    };
    void RecordTimeStampsAsCSV(std::vector<TimeStamp_t> const& TimeStampRecord, int32_t Record);
    std::string utc_system_timestamp(struct timespec const& now, char sep) const;
    void ThrowException(std::string const& text);
    void ADQNDS_MSG_ERRLOG_PV(int status, std::string const& text) {
        if (!status)
            ThrowException(text);
    }
};

#endif /* ADQAICHANNELGROUP_H */
