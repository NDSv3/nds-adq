//
// Copyright (c) 2018 Cosylab d.d.
// This software is distributed under the terms found
// in file LICENSE.txt that is included with this distribution.
//

#include <cstdlib>
#include <string.h>
#include <string>
#ifndef _WIN32
#include <unistd.h>
#endif
#include <algorithm>

#include <ADQAPI.h>
#include <nds3/nds.h>

#include "ADQAIChannel.h"
#include "ADQAIChannelGroup.h"
#include "ADQDefinition.h"
#include "ADQDevice.h"

ADQDevice::CADQControl* ADQDevice::m_adqCtrlUnit = NULL;

class ADQDevice::CADQControl
{
    // ADQ Control Unit
    void* m_adqCtrlUnit;
    std::map<std::string, int> m_ADQDeviceMap;  // Mapping to ADQ device index
    std::vector<ADQInterface*> m_adqInterfaces; // The list of ADQ objects
    std::map<int, std::vector<int>> m_device_position;
public:
    void detect_daisy_chain_order();

    std::map<std::string, int>::const_iterator find(std::string const& adqSnRdbk) const {
        std::map<std::string, int>::const_iterator ADQIter = m_ADQDeviceMap.find(adqSnRdbk);
        if (ADQIter == m_ADQDeviceMap.end())
            throw nds::NdsError("Requested ADQ device was not found.");
        if (ADQIter->second >= int(m_adqInterfaces.size()))
            throw nds::NdsError("Requested ADQ device index was not found.");
        return ADQIter;
    }
    size_t size() const {
        return m_ADQDeviceMap.size();
    }
    std::map<std::string, int>::const_iterator end() const {
        return m_ADQDeviceMap.end();
    }
    ADQInterface const* adqInterface(size_t Enumeration) const {
        return m_adqInterfaces[Enumeration];
    }
    ADQInterface* adqInterface(size_t Enumeration) {
        return m_adqInterfaces[Enumeration];
    }
    std::map<int, std::vector<int>> const& device_position() const {
        return m_device_position;
    }
    std::map<int, std::vector<int>>::const_iterator device_position(int Enumeration) const {
        return m_device_position.find(Enumeration);
    }
    void CloseDevice(std::map<std::string, int>::const_iterator ADQIter) {
        ADQControlUnit_CloseDevice(m_adqCtrlUnit, ADQIter->second);
        m_adqInterfaces[ADQIter->second] = NULL;
        m_ADQDeviceMap.erase(ADQIter);
    }
    CADQControl();
    ~CADQControl();
};
// ADQ Control Unit

ADQDevice::ADQDevice(nds::Factory& factory, const std::string& deviceName, const nds::namedParameters_t& parameters) :
    m_node(deviceName)
{
    try
    {
        if (!m_adqCtrlUnit)
            m_adqCtrlUnit = new CADQControl;

        nds::namedParameters_t::const_iterator ParamIter = parameters.find("ADQSN");
        if (ParamIter == parameters.end())
            throw nds::NdsError("No device configured.");

        m_adqSnRdbk = "SPD-" + ParamIter->second;
        std::map<std::string, int>::const_iterator ADQIter = m_adqCtrlUnit->find(m_adqSnRdbk);

        int thisEnumeration = ADQIter->second;

        int32_t masterEnumeration = thisEnumeration;
        int32_t nextEnumeration = -1;
        std::map<int, std::vector<int>>::const_iterator MasterIter = m_adqCtrlUnit->device_position(thisEnumeration);
        if ((MasterIter != m_adqCtrlUnit->device_position().end()) && (MasterIter->second.size() > 0))
        {
            masterEnumeration = thisEnumeration;
            nextEnumeration = MasterIter->second[0];
        }
        else
            for (MasterIter = m_adqCtrlUnit->device_position().begin(); MasterIter != m_adqCtrlUnit->device_position().end(); MasterIter++)
            {
                std::vector<int>::const_iterator SlaveIter = std::find(MasterIter->second.begin(), MasterIter->second.end(), thisEnumeration);
                if (SlaveIter != MasterIter->second.end())
                {
                    masterEnumeration = MasterIter->first;
                    if (SlaveIter + 1 != MasterIter->second.end())
                        nextEnumeration = *(SlaveIter + 1);
                    break;
                }
            }

        m_adqChanGrpPtr = new ADQAIChannelGroup(parameters.at("ADQSN"), m_node,
            m_adqCtrlUnit->adqInterface(thisEnumeration), masterEnumeration, thisEnumeration, nextEnumeration);
        
        // Initialize NDS device after declaration of all its PVs
        m_node.initialize(this, factory);
    }
    catch (const nds::NdsError& e)
    {
        delete m_adqCtrlUnit;
        m_adqCtrlUnit = NULL;
        std::cerr << "Initializing stopped due to error: " << e.what() << std::endl;
    }
}

ADQDevice::CADQControl::CADQControl()
{
    struct ADQInfoListEntry* adqInfoStruct;
    unsigned int adqDevList = 0;
    m_adqCtrlUnit = ::CreateADQControlUnit();   // Creates an ADQControlUnit
    if (!m_adqCtrlUnit)
    {
        throw nds::NdsError("Failed to create ADQ Control Unit (CreateADQControlUnit).");
    }
    int status = ADQControlUnit_EnableErrorTrace(m_adqCtrlUnit, LOG_LEVEL_INFO, ".");

    // Check revisions
    const int adqApiRev = ADQAPI_GetRevision();
    if (!IS_VALID_DLL_REVISION(adqApiRev))
    {
        std::cout << "WARNING: The included header file and the linked library are of different revisions. This "
            "may cause corrupt behavior."
            << std::endl;
    }
    // Find all connected devices
    status = ADQControlUnit_ListDevices(m_adqCtrlUnit, &adqInfoStruct, &adqDevList);
    if (!status)
    {
        throw nds::NdsError("Listing connected devices failed (ADQControlUnit_ListDevices).");
    }

    if (adqDevList <= 0)
    {
        throw nds::NdsError("No ADQ devices found.");
    }
    /* This block searches a device with a requested serial number
     */
    m_adqInterfaces.resize(adqDevList);
    unsigned int adqDevListNum;
    for (adqDevListNum = 0; adqDevListNum < adqDevList; ++adqDevListNum)
    {
        // Opens communication channel to a certain ADQ device
        status = ADQControlUnit_OpenDeviceInterface(m_adqCtrlUnit, adqDevListNum);
        if (!status)
        {
            throw nds::NdsError("Device failure during interface opening (ADQControlUnit_OpenDeviceInterface).");
        }

        // Make this device ready to use
        status = ADQControlUnit_SetupDevice(m_adqCtrlUnit, adqDevListNum);
        if (!status)
        {
            throw nds::NdsError("Device failure during setup (ADQControlUnit_SetupDevice).");
        }

        // Pointer to ADQ device interface
        ADQInterface* adqInterface = ADQControlUnit_GetADQ(m_adqCtrlUnit, adqDevListNum + 1);
        if (!adqInterface)
            throw nds::NdsError("Could not obtain ADQ interface.");

        // Get pointer to interface of the device
        status = IS_VALID_DLL(adqInterface);
        if (!status)
        {
            throw nds::NdsError("DLL validation fail.");
        }
        // Check if this ADQ serial number is the one requested
        const char* adqSnRdbk = adqInterface->GetBoardSerialNumber();

        // Check if ADQ started normally
        status = adqInterface->IsStartedOK();
        if (!status)
        {
            throw nds::NdsError("Device didn't start normally (IsStartedOK).");
        }
        m_ADQDeviceMap[adqSnRdbk] = adqDevListNum;
        m_adqInterfaces[adqDevListNum] = adqInterface;
    }
    detect_daisy_chain_order();
#ifdef _WIN32
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
#endif
}

void ADQDevice::CADQControl::detect_daisy_chain_order()
{
    // First disable all outputs
    int status = 0;
    for (size_t adq_num = 0; adq_num < m_adqInterfaces.size(); adq_num++)
    {
        if (m_adqInterfaces[adq_num]->GetADQType() != 8)
            continue;
        status = m_adqInterfaces[adq_num]->DaisyChainEnableOutput(0);
        if (!status)
            throw nds::NdsError("ERROR: DaisyChainEnableOutput failed");
        status = m_adqInterfaces[adq_num]->DaisyChainSetOutputState(0);
        if (!status)
            throw nds::NdsError("ERROR: DaisyChainSetOutputState failed");
    }
    std::vector<int> next_device(m_adqInterfaces.size(), -1);
    std::vector<int> prev_device(m_adqInterfaces.size(), -1);
    for (size_t adq_out_num = 0; adq_out_num < m_adqInterfaces.size(); adq_out_num++)
    {
        if (m_adqInterfaces[adq_out_num]->GetADQType() != 8)
            continue;
        // Determine which digitizer is not pointed to by any device.
        // Filter out the last slave(next_device == -1)
        status = m_adqInterfaces[adq_out_num]->DaisyChainEnableOutput(1);
        if (!status)
            throw nds::NdsError("ERROR: DaisyChainEnableOutput failed");
        status = m_adqInterfaces[adq_out_num]->DaisyChainSetOutputState(1);
        if (!status)
            throw nds::NdsError("ERROR: DaisyChainSetOutputState failed");
        for (size_t adq_in_num = 0; adq_in_num < m_adqInterfaces.size(); adq_in_num++)
        {
            if (adq_in_num == adq_out_num)
                continue;
            unsigned int InputState = 0;
            status = m_adqInterfaces[adq_in_num]->DaisyChainGetInputState(&InputState);
            if (!status)
                throw nds::NdsError("ERROR: DaisyChainGetInputState failed");
            if (InputState == 1)
            {
                if (next_device[adq_out_num] != -1)
                    throw nds::NdsError("Device input already connected");
                next_device[adq_out_num] = int(adq_in_num);
                if (prev_device[adq_in_num] != -1)
                    throw nds::NdsError("Device output already connected");
                prev_device[adq_in_num] = int(adq_out_num);
            }
        }
        status = m_adqInterfaces[adq_out_num]->DaisyChainSetOutputState(0);
        if (!status)
            throw nds::NdsError("ERROR: DaisyChainSetOutputState failed");
        status = m_adqInterfaces[adq_out_num]->DaisyChainEnableOutput(0);
        if (!status)
            throw nds::NdsError("ERROR: DaisyChainEnableOutput failed");
    }
    m_device_position.clear();
    size_t nof_unconnected_slaves = 0;
    for (size_t adq_num = 0; adq_num < m_adqInterfaces.size(); adq_num++)
    {
        if (next_device[adq_num] == -1)
            nof_unconnected_slaves++;
        if (prev_device[adq_num] == -1)
            m_device_position[int(adq_num)] = std::vector<int32_t>();
    }
    if (nof_unconnected_slaves != m_device_position.size())
        throw nds::NdsError("At least one incomplete chain");

    // Walk through the next_device list to determine the devices' positions in
    // the daisy chain.
    for (std::map<int, std::vector<int32_t>>::iterator DaisyIter = m_device_position.begin();
        DaisyIter != m_device_position.end(); DaisyIter++)
    {
        int next_slave_num = DaisyIter->first;
        while (true)
        {
            next_slave_num = next_device[next_slave_num];
            if (next_slave_num == -1)
                break;
            DaisyIter->second.push_back(next_slave_num);
        }
    }
}

void ADQDevice::CloseDevice()
{
    if (m_adqCtrlUnit == NULL)
        return;
    if (m_adqCtrlUnit->size() > 0)
    {
        if (m_adqChanGrpPtr)
            m_adqChanGrpPtr->onSwitchOff();
        std::map<std::string, int>::const_iterator ADQIter = m_adqCtrlUnit->find(m_adqSnRdbk);
        delete m_adqChanGrpPtr;
        if (ADQIter != m_adqCtrlUnit->end())
        {
            m_adqCtrlUnit->CloseDevice(ADQIter);
        }
    }
    if ((m_adqCtrlUnit->size() == 0) && (m_adqCtrlUnit))
    {
        delete m_adqCtrlUnit;
        ndsInfoStream(m_node) << "ADQ Control Unit was deleted." << std::endl;
        m_adqCtrlUnit = NULL;
    }
}

ADQDevice::CADQControl::~CADQControl()
{
    if (m_adqCtrlUnit)
    {
        ::DeleteADQControlUnit(m_adqCtrlUnit);
        m_adqCtrlUnit = NULL;
    }
}

ADQDevice::~ADQDevice()
{
    CloseDevice();
    ndsInfoStream(m_node) << "ADQ Device class was destructed." << std::endl;
}

/* This macro defines the driver name and the name of the class that implements the driver.
 * Name is provided by the shared module for other NDS3 functions (e.g. ndsCreateDevice).
 */
NDS_DEFINE_DRIVER(tspd_adq, ADQDevice)
