//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "libworld.h"

namespace world
{
    /**
     * @brief Arrival flow rate configuration
     */
    struct ArrivalFlowConfig
    {
        std::string airportIcao;
        float maxArrivalsPerHour = 30;  // Default 30 arrivals per hour
        float currentRatePerHour = 30;  // Current rate (may be reduced)
        std::vector<std::string> arrivalRunways;

        ArrivalFlowConfig() = default;
        ArrivalFlowConfig(const std::string& icao)
            : airportIcao(icao)
        {
        }

        /**
         * @brief Get the minimum interval between arrivals in seconds
         */
        float getMinimumIntervalSeconds() const
        {
            if (currentRatePerHour <= 0)
            {
                return 0;
            }
            return 3600.0f / currentRatePerHour;
        }
    };

    /**
     * @brief Arrival flow controller for managing arrival rates
     * Implements arrival rate limiting and flow metering
     */
    class ArrivalFlowController
    {
    public:
        ArrivalFlowController();

        /**
         * @brief Set the airport for this controller
         */
        void setAirport(const std::string& airportIcao);

        /**
         * @brief Configure arrival flow parameters
         */
        void configure(const ArrivalFlowConfig& config);

        /**
         * @brief Get the current configuration
         */
        const ArrivalFlowConfig& getConfig() const { return m_config; }

        /**
         * @brief Check if an arrival can be accepted
         * @param currentTime Current time
         * @return true if arrival can be accepted
         */
        bool canAcceptArrival(std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Record an accepted arrival
         * @param flight The arriving flight
         * @param currentTime Current time
         */
        void recordArrival(std::shared_ptr<Flight> flight, std::chrono::steady_clock::time_point currentTime);

        /**
         * @brief Get the estimated time when next arrival can be accepted
         * @param currentTime Current time
         * @return Estimated available time
         */
        std::chrono::steady_clock::time_point getNextAvailableTime(
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Reduce arrival rate (e.g., due to weather)
         * @param reductionFactor Factor to multiply current rate by (0.0 to 1.0)
         */
        void reduceRate(float reductionFactor);

        /**
         * @brief Restore arrival rate to maximum
         */
        void restoreRate();

        /**
         * @brief Get the number of arrivals in the last hour
         */
        int getArrivalsInLastHour(
            std::chrono::steady_clock::time_point currentTime) const;

    private:
        ArrivalFlowConfig m_config;
        std::vector<std::chrono::steady_clock::time_point> m_arrivalTimes;
        std::string m_airportIcao;
    };
}