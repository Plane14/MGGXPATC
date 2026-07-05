//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <unordered_map>
#include "libworld.h"

namespace world
{
    /**
     * @brief Ground stop entry for a specific destination
     */
    struct GroundStopEntry
    {
        std::string destinationIcao;
        std::chrono::steady_clock::time_point stopTime;
        std::chrono::steady_clock::time_point resumeTime;
        std::string reason;
        bool isActive = true;

        GroundStopEntry() = default;
        GroundStopEntry(
            const std::string& dest,
            std::chrono::steady_clock::time_point resume,
            const std::string& r)
            : destinationIcao(dest), resumeTime(resume), reason(r)
        {
            stopTime = std::chrono::steady_clock::now();
        }

        /**
         * @brief Check if the ground stop has expired
         */
        bool isExpired(std::chrono::steady_clock::time_point currentTime) const
        {
            return currentTime >= resumeTime;
        }
    };

    /**
     * @brief Ground stop manager for taxi operations
     * Manages ground stop/resume logic for departing aircraft
     */
    class GroundStopManager
    {
    public:
        GroundStopManager();

        /**
         * @brief Set the airport for this manager
         */
        void setAirport(const std::string& airportIcao);

        /**
         * @brief Initiate a ground stop for a specific destination
         * @param destinationIcao Destination airport ICAO code
         * @param durationSeconds Duration of the ground stop in seconds
         * @param reason Reason for the ground stop
         */
        void initiateGroundStop(
            const std::string& destinationIcao,
            int durationSeconds,
            const std::string& reason);

        /**
         * @brief Cancel a ground stop
         * @param destinationIcao Destination airport ICAO code
         */
        void cancelGroundStop(const std::string& destinationIcao);

        /**
         * @brief Check if a flight is affected by a ground stop
         * @param flight The flight to check
         * @param currentTime Current time
         * @return true if flight is under ground stop
         */
        bool isFlightUnderGroundStop(
            std::shared_ptr<Flight> flight,
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Get the estimated resume time for a destination
         * @param destinationIcao Destination airport ICAO code
         * @return Resume time, or time_point::max if no ground stop
         */
        std::chrono::steady_clock::time_point getResumeTime(
            const std::string& destinationIcao) const;

        /**
         * @brief Get all active ground stops
         */
        std::vector<GroundStopEntry> getActiveGroundStops(
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Get the number of flights waiting due to ground stops
         */
        int getWaitingFlightsCount(
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Add a flight to the waiting queue
         * @param flight The flight to add
         */
        void addWaitingFlight(std::shared_ptr<Flight> flight);

        /**
         * @brief Remove a flight from the waiting queue
         * @param flight The flight to remove
         */
        void removeWaitingFlight(std::shared_ptr<Flight> flight);

        /**
         * @brief Get the next flight that can be released
         * @param currentTime Current time
         * @return Next flight to release, or nullptr
         */
        std::shared_ptr<Flight> getNextFlightToRelease(
            std::chrono::steady_clock::time_point currentTime) const;

    private:
        std::string m_airportIcao;
        std::unordered_map<std::string, GroundStopEntry> m_groundStops;
        std::vector<std::shared_ptr<Flight>> m_waitingFlights;
    };
}