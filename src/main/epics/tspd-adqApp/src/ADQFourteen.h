#ifndef ADQFOURTEEN_H
#define ADQFOURTEEN_H

#include <nds3/nds.h>
#include "ADQAIChannelGroup.h"

class ADQFourteen : public ADQAIChannelGroup
{
public:
    ADQFourteen(const std::string& name, nds::Node& parentNode, ADQInterface *& adqDev);

    // Pointer to channel group class
    std::vector<std::shared_ptr<ADQAIChannelGroup> > m_AIChannelGroupPtr;

    void setChanActive(const timespec &pTimestamp, const std::int32_t &pValue);
    void getChanActive(timespec* pTimestamp, std::int32_t* pValue);
    void setChanMask(const timespec &pTimestamp, const std::string &pValue);
    void getChanMask(timespec* pTimestamp, std::string* pValue);

    void setTrigLvl(const timespec &pTimestamp, const std::int32_t &pValue);
    void getTrigLvl(timespec* pTimestamp, std::int32_t* pValue);
    void setTrigEdge(const timespec &pTimestamp, const std::int32_t &pValue);
    void getTrigEdge(timespec* pTimestamp, std::int32_t* pValue);
    void setTrigChan(const timespec &pTimestamp, const std::int32_t &pValue);
    void getTrigChan(timespec* pTimestamp, std::int32_t* pValue);

    void commitChangesSpec(bool calledFromDaqThread = false);

private:
    ADQInterface * m_adqDevPtr;
    nds::Port m_node;

    nds::PVDelegateIn<std::int32_t> m_chanActivePV;
    nds::PVDelegateIn<std::string> m_chanMaskPV;
    nds::PVDelegateIn<std::int32_t> m_trigLvlPV;
    nds::PVDelegateIn<std::int32_t> m_trigEdgePV;
    nds::PVDelegateIn<std::int32_t> m_trigChanPV;
};

#endif /* ADQFOURTEEN_H */