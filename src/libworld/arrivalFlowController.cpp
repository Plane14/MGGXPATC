//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "arrivalFlowController.hpp"

using namespace std;

namespace world
{
    ArrivalFlowController::ArrivalFlowController()
    {
    }

    void ArrivalFlowController::setAirport(const string& airportIcao)
    {
        m_airportIcao = airportIcao;
        m_config.airportIcao = airportIcao;
    }

    void ArrivalFlowController::configure(const ArrivalFlowConfig& config)
    {
        m_config = config;
    }

    bool ArrivalFlowController::canAcceptArrival(chrono::steady_clock::time_point currentTime) const
    {
        if (m_config.currentRatePerHour <= 0)
        {
            return false;
        }

        // Check if we have capacity in the last hour
        int arrivalsInLastHour = 0;
        auto oneHourAgo = currentTime - chrono::hours(1);

        for (const auto& arrivalTime : m_arrivalTimes)
        {
            if (arrivalTime >= oneHourAgo)
            {
                arrivalsInLastHour++;
            }
        }

        // Check against rate limit
        float maxArrivalsInLastHour = m_config.currentRatePerHour;
        return static_cast<float>(arrivalsInLastHour) < maxArrivalsInLastHour;
    }

    void ArrivalFlowController::recordArrival(shared_ptr<Flight> flight, chrono::steady_clock::time_point currentTime)
    {
        if (!flight)
        {
            return;
        }

        m_arrivalTimes.push_back(currentTime);
    }

    chrono::steady_clock::time_point ArrivalFlowController::getNextAvailableTime(
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
        int arrivalsInLastHour = 0;
        chrono::steady_clock::time_point oldestArrivalInWindow;

        for (const auto& arrivalTime : m_arrivalTimes)
        {
            if (arrivalTime >= oneHourAgo)
            {
                arrivalsInLastHour++;
                if (oldestArrivalInWindow == chrono::steady_clock::time_point() || arrivalTime < oldestArrivalInWindow)
                {
                    oldestArrivalInWindow = arrivalTime;
                }
            }
        }

        // If we have capacity, return current time
        if (static_cast<float>(arrivalsInLastHour) < m_config.currentRatePerHour)
        {
            return currentTime;
        }

        // Otherwise, return time when oldest arrival exits the 1-hour window
        return oldestArrivalInWindow + chrono::hours(1);
    }

    void ArrivalFlowController::reduceRate(float reductionFactor)
    {
        if (reductionFactor < 0.0f)
        {
            reductionFactor = 0.0f;
        }
        if (reductionFactor > 1.0f)
        {
            reductionFactor = 1.0f;
        }

        m_config.currentRatePerHour = m_config.maxArrivalsPerHour * reductionFactor;
    }

    void ArrivalFlowController::restoreRate()
    {
        m_config.currentRatePerHour = m_config.maxArrivalsPerHour;
    }

    int ArrivalFlowController::getArrivalsInLastHour(chrono::steady_clock::time_point currentTime) const
    {
        int count = 0;
        auto oneHourAgo = currentTime - chrono::hours(1);

        for (const auto& arrivalTime : m_arrivalTimes)
        {
            if (arrivalTime >= oneHourAgo)
            {
                count++;
            }
        }

        return count;
    }
}