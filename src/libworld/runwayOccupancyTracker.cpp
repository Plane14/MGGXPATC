//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "runwayOccupancyTracker.hpp"
#include "separationManager.hpp"

using namespace std;

namespace world
{
    RunwayOccupancyTracker::RunwayOccupancyTracker()
    {
    }

    void RunwayOccupancyTracker::recordRunwayEntry(shared_ptr<Flight> flight, const string& runwayName, bool isDeparture)
    {
        if (!flight || runwayName.empty())
        {
            return;
        }

        auto event = make_shared<RunwayOccupancyEvent>(flight, runwayName, isDeparture);
        m_occupancyEvents[runwayName].push_back(*event);
        m_activeOccupancies[runwayName] = event;
    }

    void RunwayOccupancyTracker::recordRunwayExit(shared_ptr<Flight> flight, const string& runwayName)
    {
        if (!flight || runwayName.empty())
        {
            return;
        }

        auto it = m_activeOccupancies.find(runwayName);
        if (it != m_activeOccupancies.end() && it->second && it->second->flight == flight)
        {
            it->second->endOccupancy();

            // Update statistics
            auto& stats = m_occupancyStats[runwayName];
            stats.runwayName = runwayName;
            stats.updateStats(it->second->occupancyDurationSeconds);

            m_activeOccupancies.erase(it);
        }
    }

    float RunwayOccupancyTracker::getReleaseIntervalSeconds(const string& runwayName) const
    {
        auto it = m_occupancyStats.find(runwayName);
        if (it != m_occupancyStats.end() && it->second.totalOperations > 0)
        {
            return it->second.averageOccupancySeconds + SAFETY_MARGIN_SECONDS;
        }

        // Default values based on operation type
        return 60.0f; // Default 60 seconds
    }

    const RunwayOccupancyStats* RunwayOccupancyTracker::getStats(const string& runwayName) const
    {
        auto it = m_occupancyStats.find(runwayName);
        if (it != m_occupancyStats.end())
        {
            return &it->second;
        }
        return nullptr;
    }

    vector<RunwayOccupancyEvent> RunwayOccupancyTracker::getOccupancyEvents(const string& runwayName) const
    {
        auto it = m_occupancyEvents.find(runwayName);
        if (it != m_occupancyEvents.end())
        {
            return it->second;
        }
        return {};
    }

    bool RunwayOccupancyTracker::isRunwayOccupied(const string& runwayName) const
    {
        auto it = m_activeOccupancies.find(runwayName);
        return it != m_activeOccupancies.end() && it->second && it->second->isActive();
    }

    shared_ptr<Flight> RunwayOccupancyTracker::getCurrentOccupyingFlight(const string& runwayName) const
    {
        auto it = m_activeOccupancies.find(runwayName);
        if (it != m_activeOccupancies.end() && it->second)
        {
            return it->second->flight;
        }
        return nullptr;
    }

    int RunwayOccupancyTracker::getWakeTurbulenceSeparationSeconds(
        WakeTurbulenceCategory leader,
        WakeTurbulenceCategory follower)
    {
        // ICAO Doc 4444 Table 8-1 separation in seconds
        // Heavy aircraft: 2 minutes (120 seconds)
        // Medium aircraft: 2 minutes (120 seconds)
        // Light aircraft: 2 minutes (120 seconds)
        // Following Heavy: additional 2 minutes

        int baseSeparation = 120; // Base 2 minutes

        // Add extra separation if following aircraft is Heavy
        if (follower == WakeTurbulenceCategory::Heavy || follower == WakeTurbulenceCategory::Super)
        {
            baseSeparation += 120; // Additional 2 minutes
        }

        return baseSeparation;
    }

    chrono::steady_clock::time_point RunwayOccupancyTracker::getEstimatedAvailableTime(
        const string& runwayName,
        chrono::steady_clock::time_point currentTime) const
    {
        if (isRunwayOccupied(runwayName))
        {
            auto it = m_activeOccupancies.find(runwayName);
            if (it != m_activeOccupancies.end() && it->second)
            {
                // Calculate based on average occupancy + safety margin
                float releaseInterval = getReleaseIntervalSeconds(runwayName);
                return it->second->occupancyStart + chrono::seconds(static_cast<int>(releaseInterval));
            }
        }

        return currentTime;
    }
}