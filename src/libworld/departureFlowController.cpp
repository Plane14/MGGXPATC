//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "departureFlowController.hpp"

using namespace std;

namespace world
{
    DepartureFlowController::DepartureFlowController()
    {
    }

    void DepartureFlowController::setAirport(const string& airportIcao)
    {
        m_airportIcao = airportIcao;
        m_config.airportIcao = airportIcao;
    }

    void DepartureFlowController::configure(const DepartureFlowConfig& config)
    {
        m_config = config;
    }

    bool DepartureFlowController::canReleaseDeparture(chrono::steady_clock::time_point currentTime) const
    {
        if (m_config.currentRatePerHour <= 0)
        {
            return false;
        }

        // Check if we have capacity in the last hour
        int departuresInLastHour = 0;
        auto oneHourAgo = currentTime - chrono::hours(1);

        for (const auto& departureTime : m_departureTimes)
        {
            if (departureTime >= oneHourAgo)
            {
                departuresInLastHour++;
            }
        }

        // Check against rate limit
        float maxDeparturesInLastHour = m_config.currentRatePerHour;
        return static_cast<float>(departuresInLastHour) < maxDeparturesInLastHour;
    }

    void DepartureFlowController::recordDeparture(shared_ptr<Flight> flight, chrono::steady_clock::time_point currentTime)
    {
        if (!flight)
        {
            return;
        }

        m_departureTimes.push_back(currentTime);
    }

    chrono::steady_clock::time_point DepartureFlowController::getNextAvailableTime(
        chrono::steady_clock::time_point currentTime) const
    {
        if (m_config.currentRatePerHour <= 0)
        {
            return currentTime + chrono::hours(1); // No capacity, return 1 hour
        }

        float minInterval = m_config.getMinimumIntervalSeconds();
        if (minInterval <= 0)
        {
            return currentTime;
        }

        // Find the earliest time when we have capacity
        auto oneHourAgo = currentTime - chrono::hours(1);
        int departuresInLastHour = 0;
        chrono::steady_clock::time_point oldestDepartureInWindow;

        for (const auto& departureTime : m_departureTimes)
        {
            if (departureTime >= oneHourAgo)
            {
                departuresInLastHour++;
                if (oldestDepartureInWindow == chrono::steady_clock::time_point() || departureTime < oldestDepartureInWindow)
                {
                    oldestDepartureInWindow = departureTime;
                }
            }
        }

        // If we have capacity, return current time
        if (static_cast<float>(departuresInLastHour) < m_config.currentRatePerHour)
        {
            return currentTime;
        }

        // Otherwise, return time when oldest departure exits the 1-hour window
        return oldestDepartureInWindow + chrono::hours(1);
    }

    void DepartureFlowController::reduceRate(float reductionFactor)
    {
        if (reductionFactor < 0.0f)
        {
            reductionFactor = 0.0f;
        }
        if (reductionFactor > 1.0f)
        {
            reductionFactor = 1.0f;
        }

        m_config.currentRatePerHour = m_config.maxDeparturesPerHour * reductionFactor;
    }

    void DepartureFlowController::restoreRate()
    {
        m_config.currentRatePerHour = m_config.maxDeparturesPerHour;
    }

    int DepartureFlowController::getDeparturesInLastHour(chrono::steady_clock::time_point currentTime) const
    {
        int count = 0;
        auto oneHourAgo = currentTime - chrono::hours(1);

        for (const auto& departureTime : m_departureTimes)
        {
            if (departureTime >= oneHourAgo)
            {
                count++;
            }
        }

        return count;
    }
}