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
     * @brief Departure flow rate configuration
     */
    struct DepartureFlowConfig
    {
        std::string airportIcao;
        float maxDeparturesPerHour = 40;  // Default 40 departures per hour
        float currentRatePerHour = 40;    // Current rate (may be reduced)
        std::vector<std::string> departureRunways;

        DepartureFlowConfig() = default;
        DepartureFlowConfig(const std::string& icao)
            : airportIcao(icao)
        {
        }

        /**
         * @brief Get the minimum interval between departures in seconds
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
     * @brief Departure flow controller for managing departure rates
     * Implements departure rate limiting and flow metering
     */
    class DepartureFlowController
    {
    public:
        DepartureFlowController();

        /**
         * @brief Set the airport for this controller
         */
        void setAirport(const std::string& airportIcao);

        /**
         * @brief Configure departure flow parameters
         */
        void configure(const DepartureFlowConfig& config);

        /**
         * @brief Get the current configuration
         */
        const DepartureFlowConfig& getConfig() const { return m_config; }

        /**
         * @brief Check if a departure can be released
         * @param currentTime Current time
         * @return true if departure can be released
         */
        bool canReleaseDeparture(std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Record a released departure
         * @param flight The departing flight
         * @param currentTime Current time
         */
        void recordDeparture(std::shared_ptr<Flight> flight, std::chrono::steady_clock::time_point currentTime);

        /**
         * @brief Get the estimated time when next departure can be released
         * @param currentTime Current time
         * @return Estimated available time
         */
        std::chrono::steady_clock::time_point getNextAvailableTime(
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Reduce departure rate (e.g., due to ground stop)
         * @param reductionFactor Factor to multiply current rate by (0.0 to 1.0)
         */
        void reduceRate(float reductionFactor);

        /**
         * @brief Restore departure rate to maximum
         */
        void restoreRate();

        /**
         * @brief Get the number of departures in the last hour
         */
        int getDeparturesInLastHour(
            std::chrono::steady_clock::time_point currentTime) const;

    private:
        DepartureFlowConfig m_config;
        std::vector<std::chrono::steady_clock::time_point> m_departureTimes;
        std::string m_airportIcao;
    };
}