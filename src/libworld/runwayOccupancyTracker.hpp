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
    using WakeTurbulenceCategory = Aircraft::WakeTurbulenceCategory;
    /**
     * @brief Runway occupancy event for tracking aircraft on runways
     */
    struct RunwayOccupancyEvent
    {
        std::shared_ptr<Flight> flight;
        std::string runwayName;
        std::chrono::steady_clock::time_point occupancyStart;
        std::chrono::steady_clock::time_point occupancyEnd;
        bool isDeparture;
        float occupancyDurationSeconds;

        RunwayOccupancyEvent(
            std::shared_ptr<Flight> f,
            const std::string& rwy,
            bool departure)
            : flight(f), runwayName(rwy), isDeparture(departure)
        {
            occupancyStart = std::chrono::steady_clock::now();
            occupancyEnd = std::chrono::steady_clock::time_point();
            occupancyDurationSeconds = 0.0f;
        }

        void endOccupancy()
        {
            occupancyEnd = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(occupancyEnd - occupancyStart);
            occupancyDurationSeconds = static_cast<float>(duration.count()) / 1000000.0f;
        }

        bool isActive() const
        {
            return occupancyEnd == std::chrono::steady_clock::time_point();
        }
    };

    /**
     * @brief Runway occupancy statistics
     */
    struct RunwayOccupancyStats
    {
        std::string runwayName;
        int totalOperations = 0;
        float averageOccupancySeconds = 0.0f;
        float minOccupancySeconds = 0.0f;
        float maxOccupancySeconds = 0.0f;
        float lastOccupancySeconds = 0.0f;

        void updateStats(float occupancySeconds)
        {
            totalOperations++;
            lastOccupancySeconds = occupancySeconds;

            if (totalOperations == 1)
            {
                averageOccupancySeconds = occupancySeconds;
                minOccupancySeconds = occupancySeconds;
                maxOccupancySeconds = occupancySeconds;
            }
            else
            {
                // Update running average
                averageOccupancySeconds = (averageOccupancySeconds * (totalOperations - 1) + occupancySeconds) / totalOperations;
                minOccupancySeconds = std::min(minOccupancySeconds, occupancySeconds);
                maxOccupancySeconds = std::max(maxOccupancySeconds, occupancySeconds);
            }
        }
    };

    /**
     * @brief Runway occupancy time tracking system
     * Tracks actual runway occupancy times and calculates release intervals
     */
    class RunwayOccupancyTracker
    {
    public:
        RunwayOccupancyTracker();

        /**
         * @brief Record when a flight enters a runway
         * @param flight The flight entering the runway
         * @param runwayName Name of the runway
         * @param isDeparture true for departure, false for arrival
         */
        void recordRunwayEntry(std::shared_ptr<Flight> flight, const std::string& runwayName, bool isDeparture);

        /**
         * @brief Record when a flight exits a runway
         * @param flight The flight exiting the runway
         * @param runwayName Name of the runway
         */
        void recordRunwayExit(std::shared_ptr<Flight> flight, const std::string& runwayName);

        /**
         * @brief Get the estimated release interval in seconds for a runway
         * Based on average occupancy time plus safety margin
         * @param runwayName Name of the runway
         * @return Estimated release interval in seconds
         */
        float getReleaseIntervalSeconds(const std::string& runwayName) const;

        /**
         * @brief Get occupancy statistics for a runway
         * @param runwayName Name of the runway
         * @return Occupancy statistics
         */
        const RunwayOccupancyStats* getStats(const std::string& runwayName) const;

        /**
         * @brief Get all occupancy events for a runway
         * @param runwayName Name of the runway
         * @return Vector of occupancy events
         */
        std::vector<RunwayOccupancyEvent> getOccupancyEvents(const std::string& runwayName) const;

        /**
         * @brief Check if a runway is currently occupied
         * @param runwayName Name of the runway
         * @return true if runway is occupied
         */
        bool isRunwayOccupied(const std::string& runwayName) const;

        /**
         * @brief Get the currently occupying flight (if any)
         * @param runwayName Name of the runway
         * @return Flight occupying the runway, or nullptr
         */
        std::shared_ptr<Flight> getCurrentOccupyingFlight(const std::string& runwayName) const;

        /**
         * @brief Calculate required separation time for wake turbulence
         * Based on ICAO Doc 4444 Table 8-1
         * @param leader Wake turbulence category of leading aircraft
         * @param follower Wake turbulence category of following aircraft
         * @return Required separation in seconds
         */
        static int getWakeTurbulenceSeparationSeconds(
            WakeTurbulenceCategory leader,
            WakeTurbulenceCategory follower);

        /**
         * @brief Get the time when a runway will be available
         * @param runwayName Name of the runway
         * @param currentTime Current time
         * @return Estimated available time
         */
        std::chrono::steady_clock::time_point getEstimatedAvailableTime(
            const std::string& runwayName,
            std::chrono::steady_clock::time_point currentTime) const;

    private:
        std::unordered_map<std::string, std::vector<RunwayOccupancyEvent>> m_occupancyEvents;
        std::unordered_map<std::string, RunwayOccupancyStats> m_occupancyStats;
        std::unordered_map<std::string, std::shared_ptr<RunwayOccupancyEvent>> m_activeOccupancies;

        // Safety margin in seconds to add to occupancy time
        static constexpr float SAFETY_MARGIN_SECONDS = 5.0f;
    };
}