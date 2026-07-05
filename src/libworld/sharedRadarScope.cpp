// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#include "sharedRadarScope.hpp"

using namespace std;

namespace world
{
    SharedRadarScope::SharedRadarScope(shared_ptr<HostServices> _host)
        : m_host(_host),
          m_dataFreshnessTimeoutMs(chrono::seconds(30)),
          m_visibilityTimeoutMs(chrono::minutes(5))
    {
    }

    SharedRadarScope::~SharedRadarScope()
    {
    }

    void SharedRadarScope::updateFlightPosition(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> controller,
        const GeoPoint& position,
        float altitudeFeet,
        float groundSpeedKt,
        float headingDegrees,
        chrono::microseconds timestamp
    )
    {
        if (!flight || !controller)
        {
            return;
        }

        FlightScopeData& data = m_flightScopeData[flight->id()];
        data.flight = flight;
        data.owningController = controller;
        data.lastKnownPosition = position;
        data.lastKnownAltitudeFeet = altitudeFeet;
        data.lastKnownGroundSpeedKt = groundSpeedKt;
        data.lastKnownHeadingDegrees = headingDegrees;
        data.lastUpdateTimestamp = timestamp;
        data.isRadarContact = true;

        // Update controller flight mapping
        addFlightToController(flight, controller);
    }

    void SharedRadarScope::setFlightEmergency(shared_ptr<Flight> flight, bool isEmergency)
    {
        auto data = findFlightScopeData(flight);
        if (data)
        {
            data->isEmergency = isEmergency;
        }
    }

    void SharedRadarScope::setRadarContact(shared_ptr<Flight> flight, bool hasContact)
    {
        auto data = findFlightScopeData(flight);
        if (data)
        {
            data->isRadarContact = hasContact;
        }
    }

    void SharedRadarScope::setHandoffInProgress(shared_ptr<Flight> flight, bool inProgress)
    {
        auto data = findFlightScopeData(flight);
        if (data)
        {
            data->isHandoffInProgress = inProgress;
        }
    }

    void SharedRadarScope::setFlightRemarks(shared_ptr<Flight> flight, const string& remarks)
    {
        auto data = findFlightScopeData(flight);
        if (data)
        {
            data->remarks = remarks;
        }
    }

    bool SharedRadarScope::establishScopeVisibility(
        shared_ptr<ControllerPosition> controller,
        shared_ptr<ControllerPosition> targetController,
        VisibilityRule rule,
        bool bidirectional,
        chrono::microseconds timestamp
    )
    {
        if (!controller || !targetController)
        {
            return false;
        }

        // Check if visibility already exists
        if (hasScopeVisibility(controller, targetController))
        {
            return true;
        }

        ScopeVisibility visibility;
        visibility.controller = controller;
        visibility.targetController = targetController;
        visibility.rule = rule;
        visibility.establishedTimestamp = timestamp;
        visibility.isBidirectional = bidirectional;

        m_scopeVisibilities.push_back(visibility);

        if (bidirectional)
        {
            ScopeVisibility reverseVisibility;
            reverseVisibility.controller = targetController;
            reverseVisibility.targetController = controller;
            reverseVisibility.rule = rule;
            reverseVisibility.establishedTimestamp = timestamp;
            reverseVisibility.isBidirectional = bidirectional;
            m_scopeVisibilities.push_back(reverseVisibility);
        }

        logScopeEvent("VISIBILITY_ESTABLISHED", nullptr, controller);
        return true;
    }

    bool SharedRadarScope::removeScopeVisibility(
        shared_ptr<ControllerPosition> controller,
        shared_ptr<ControllerPosition> targetController
    )
    {
        if (!controller || !targetController)
        {
            return false;
        }

        auto it = remove_if(m_scopeVisibilities.begin(), m_scopeVisibilities.end(),
            [controller, targetController](const ScopeVisibility& v) {
                return v.controller == controller && v.targetController == targetController;
            });

        bool removed = (it != m_scopeVisibilities.end());
        m_scopeVisibilities.erase(it, m_scopeVisibilities.end());

        // Also remove reverse if bidirectional
        auto reverseIt = remove_if(m_scopeVisibilities.begin(), m_scopeVisibilities.end(),
            [controller, targetController](const ScopeVisibility& v) {
                return v.controller == targetController && v.targetController == controller;
            });

        if (reverseIt != m_scopeVisibilities.end())
        {
            m_scopeVisibilities.erase(reverseIt, m_scopeVisibilities.end());
        }

        if (removed)
        {
            logScopeEvent("VISIBILITY_REMOVED", nullptr, controller);
        }

        return removed;
    }

    bool SharedRadarScope::hasFlightData(shared_ptr<Flight> flight) const
    {
        return findFlightScopeData(flight) != nullptr;
    }

    const SharedRadarScope::FlightScopeData* SharedRadarScope::getFlightData(shared_ptr<Flight> flight) const
    {
        return findFlightScopeData(flight);
    }

    vector<shared_ptr<Flight>> SharedRadarScope::getVisibleFlights(shared_ptr<ControllerPosition> controller) const
    {
        vector<shared_ptr<Flight>> visibleFlights;

        if (!controller)
        {
            return visibleFlights;
        }

        for (const auto& pair : m_flightScopeData)
        {
            const auto& data = pair.second;
            if (isFlightVisibleTo(data.flight, controller))
            {
                visibleFlights.push_back(data.flight);
            }
        }

        return visibleFlights;
    }

    vector<shared_ptr<Flight>> SharedRadarScope::getFlightsInSector(shared_ptr<ControllerPosition> controller) const
    {
        vector<shared_ptr<Flight>> sectorFlights;

        if (!controller)
        {
            return sectorFlights;
        }

        auto it = m_controllerFlights.find(controller);
        if (it != m_controllerFlights.end())
        {
            for (const auto& flight : it->second)
            {
                sectorFlights.push_back(flight);
            }
        }

        return sectorFlights;
    }

    bool SharedRadarScope::isFlightVisibleTo(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const
    {
        if (!flight || !controller)
        {
            return false;
        }

        const auto data = findFlightScopeData(flight);
        if (!data)
        {
            return false;
        }

        // Check if there's a scope visibility between the owning controller and the target
        if (data->owningController == controller)
        {
            return true;
        }

        if (hasScopeVisibility(data->owningController, controller))
        {
            return true;
        }

        return false;
    }

    void SharedRadarScope::transferFlightScope(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> fromController,
        shared_ptr<ControllerPosition> toController,
        chrono::microseconds timestamp
    )
    {
        if (!flight || !fromController || !toController)
        {
            return;
        }

        auto data = findFlightScopeData(flight);
        if (data)
        {
            data->previousController = data->owningController;
            data->owningController = toController;
            data->lastUpdateTimestamp = timestamp;
        }

        removeFlightFromController(flight, fromController);
        addFlightToController(flight, toController);

        logScopeEvent("SCOPE_TRANSFERRED", flight, toController);
    }

    void SharedRadarScope::initiateHandoffScopeTransfer(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> fromController,
        shared_ptr<ControllerPosition> toController,
        chrono::microseconds timestamp
    )
    {
        if (!flight || !fromController || !toController)
        {
            return;
        }

        auto data = findFlightScopeData(flight);
        if (data)
        {
            data->isHandoffInProgress = true;
            data->lastUpdateTimestamp = timestamp;
        }

        // Establish temporary visibility for handoff
        establishScopeVisibility(fromController, toController, VisibilityRule::Full, true, timestamp);

        logScopeEvent("HANDOFF_SCOPE_INITIATED", flight, fromController);
    }

    void SharedRadarScope::completeHandoffScopeTransfer(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> toController,
        chrono::microseconds timestamp
    )
    {
        if (!flight || !toController)
        {
            return;
        }

        auto data = findFlightScopeData(flight);
        if (data)
        {
            data->isHandoffInProgress = false;
            data->lastUpdateTimestamp = timestamp;
        }

        logScopeEvent("HANDOFF_SCOPE_COMPLETED", flight, toController);
    }

    void SharedRadarScope::processTimeouts(chrono::microseconds timestamp)
    {
        clearStaleData(timestamp);

        // Remove expired scope visibilities
        auto it = remove_if(m_scopeVisibilities.begin(), m_scopeVisibilities.end(),
            [this, timestamp](const ScopeVisibility& v) {
                return (timestamp - v.establishedTimestamp) > chrono::duration_cast<chrono::microseconds>(m_visibilityTimeoutMs);
            });

        if (it != m_scopeVisibilities.end())
        {
            m_scopeVisibilities.erase(it, m_scopeVisibilities.end());
        }
    }

    void SharedRadarScope::clearStaleData(chrono::microseconds timestamp)
    {
        vector<int> staleIds;

        for (const auto& pair : m_flightScopeData)
        {
            if (isDataStale(pair.second, timestamp))
            {
                staleIds.push_back(pair.first);
            }
        }

        for (int id : staleIds)
        {
            auto it = m_flightScopeData.find(id);
            if (it != m_flightScopeData.end())
            {
                if (it->second.owningController)
                {
                    removeFlightFromController(it->second.flight, it->second.owningController);
                }
                m_flightScopeData.erase(id);
            }
        }
    }

    int SharedRadarScope::visibleFlightCount(shared_ptr<ControllerPosition> controller) const
    {
        if (!controller)
        {
            return 0;
        }

        int count = 0;
        for (const auto& pair : m_flightScopeData)
        {
            if (isFlightVisibleTo(pair.second.flight, controller))
            {
                count++;
            }
        }
        return count;
    }

    SharedRadarScope::FlightScopeData* SharedRadarScope::findFlightScopeData(shared_ptr<Flight> flight)
    {
        if (!flight)
        {
            return nullptr;
        }

        auto it = m_flightScopeData.find(flight->id());
        if (it != m_flightScopeData.end())
        {
            return &it->second;
        }

        return nullptr;
    }

    const SharedRadarScope::FlightScopeData* SharedRadarScope::findFlightScopeData(shared_ptr<Flight> flight) const
    {
        if (!flight)
        {
            return nullptr;
        }

        auto it = m_flightScopeData.find(flight->id());
        if (it != m_flightScopeData.end())
        {
            return &it->second;
        }

        return nullptr;
    }

    bool SharedRadarScope::isDataStale(const FlightScopeData& data, chrono::microseconds timestamp) const
    {
        return (timestamp - data.lastUpdateTimestamp) > chrono::duration_cast<chrono::microseconds>(m_dataFreshnessTimeoutMs);
    }

    bool SharedRadarScope::hasScopeVisibility(shared_ptr<ControllerPosition> controller, shared_ptr<ControllerPosition> target) const
    {
        for (const auto& v : m_scopeVisibilities)
        {
            if (v.controller == controller && v.targetController == target)
            {
                return true;
            }
        }
        return false;
    }

    void SharedRadarScope::removeFlightFromController(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller)
    {
        auto it = m_controllerFlights.find(controller);
        if (it != m_controllerFlights.end())
        {
            it->second.erase(flight);
        }
    }

    void SharedRadarScope::addFlightToController(shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller)
    {
        m_controllerFlights[controller].insert(flight);
    }

    void SharedRadarScope::logScopeEvent(const string& event, shared_ptr<Flight> flight, shared_ptr<ControllerPosition> controller) const
    {
        if (!m_host)
        {
            return;
        }

        m_host->writeLog(
            "SCOPE|%s flight[%s] controller[%s]",
            event.c_str(),
            flight ? flight->callSign().c_str() : "N/A",
            controller ? controller->callSign().c_str() : "N/A"
        );
    }

    // SharedRadarScopeFactory implementation

    SharedRadarScopeFactory::SharedRadarScopeFactory(
        shared_ptr<HostServices> _host,
        shared_ptr<SharedRadarScope> _scope
    ) : m_host(_host),
        m_scope(_scope)
    {
    }

    void SharedRadarScopeFactory::updatePosition(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> controller,
        const GeoPoint& position,
        float altitudeFeet,
        float groundSpeedKt,
        float headingDegrees,
        chrono::microseconds timestamp
    )
    {
        if (m_scope)
        {
            m_scope->updateFlightPosition(flight, controller, position, altitudeFeet, groundSpeedKt, headingDegrees, timestamp);
        }
    }

    void SharedRadarScopeFactory::transferScope(
        shared_ptr<Flight> flight,
        shared_ptr<ControllerPosition> fromController,
        shared_ptr<ControllerPosition> toController,
        chrono::microseconds timestamp
    )
    {
        if (m_scope)
        {
            m_scope->transferFlightScope(flight, fromController, toController, timestamp);
        }
    }

    void SharedRadarScopeFactory::setEmergency(shared_ptr<Flight> flight, bool isEmergency)
    {
        if (m_scope)
        {
            m_scope->setFlightEmergency(flight, isEmergency);
        }
    }

    void SharedRadarScopeFactory::setRadarContact(shared_ptr<Flight> flight, bool hasContact)
    {
        if (m_scope)
        {
            m_scope->setRadarContact(flight, hasContact);
        }
    }
}
