//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "groundStopManager.hpp"

using namespace std;

namespace world
{
    GroundStopManager::GroundStopManager()
    {
    }

    void GroundStopManager::setAirport(const string& airportIcao)
    {
        m_airportIcao = airportIcao;
    }

    void GroundStopManager::initiateGroundStop(
        const string& destinationIcao,
        int durationSeconds,
        const string& reason)
    {
        if (destinationIcao.empty())
        {
            return;
        }

        auto resumeTime = chrono::steady_clock::now() + chrono::seconds(durationSeconds);
        m_groundStops[destinationIcao] = GroundStopEntry(destinationIcao, resumeTime, reason);
    }

    void GroundStopManager::cancelGroundStop(const string& destinationIcao)
    {
        auto it = m_groundStops.find(destinationIcao);
        if (it != m_groundStops.end())
        {
            it->second.isActive = false;
        }
    }

    bool GroundStopManager::isFlightUnderGroundStop(
        shared_ptr<Flight> flight,
        chrono::steady_clock::time_point currentTime) const
    {
        if (!flight)
        {
            return false;
        }

        // Check if flight's destination has an active ground stop
        // Note: This would need access to flight plan to get destination
        // For now, we check all ground stops
        for (const auto& pair : m_groundStops)
        {
            const auto& entry = pair.second;
            if (entry.isActive && !entry.isExpired(currentTime))
            {
                // In a real implementation, we would check if flight's destination matches
                return true;
            }
        }

        return false;
    }

    chrono::steady_clock::time_point GroundStopManager::getResumeTime(
        const string& destinationIcao) const
    {
        auto it = m_groundStops.find(destinationIcao);
        if (it != m_groundStops.end() && it->second.isActive)
        {
            return it->second.resumeTime;
        }
        return chrono::steady_clock::time_point::max();
    }

    vector<GroundStopEntry> GroundStopManager::getActiveGroundStops(
        chrono::steady_clock::time_point currentTime) const
    {
        vector<GroundStopEntry> result;
        for (const auto& pair : m_groundStops)
        {
            const auto& entry = pair.second;
            if (entry.isActive && !entry.isExpired(currentTime))
            {
                result.push_back(entry);
            }
        }
        return result;
    }

    int GroundStopManager::getWaitingFlightsCount(
        chrono::steady_clock::time_point currentTime) const
    {
        return static_cast<int>(m_waitingFlights.size());
    }

    void GroundStopManager::addWaitingFlight(shared_ptr<Flight> flight)
    {
        if (!flight)
        {
            return;
        }

        // Check if flight is already in the queue
        for (const auto& f : m_waitingFlights)
        {
            if (f == flight)
            {
                return; // Already in queue
            }
        }

        m_waitingFlights.push_back(flight);
    }

    void GroundStopManager::removeWaitingFlight(shared_ptr<Flight> flight)
    {
        m_waitingFlights.erase(
            remove_if(m_waitingFlights.begin(), m_waitingFlights.end(),
                [&flight](const shared_ptr<Flight>& f) { return f == flight; }),
            m_waitingFlights.end()
        );
    }

    shared_ptr<Flight> GroundStopManager::getNextFlightToRelease(
        chrono::steady_clock::time_point currentTime) const
    {
        if (m_waitingFlights.empty())
        {
            return nullptr;
        }

        // Return the first flight in the queue
        // In a real implementation, this would check ground stop status
        return m_waitingFlights.front();
    }
}