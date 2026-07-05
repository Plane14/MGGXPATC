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
    // Forward declarations
    class FlowStateMachine;
    class RunwayOccupancyTracker;
    class ArrivalFlowController;
    class DepartureFlowController;
    class GroundStopManager;

    /**
     * @brief Flow manager configuration
     */
    struct FlowManagerConfig
    {
        std::string airportIcao;
        float arrivalRatePerHour = 30.0f;
        float departureRatePerHour = 40.0f;
        bool enableFlowControl = true;
        bool enableGroundStop = true;

        FlowManagerConfig() = default;
        FlowManagerConfig(const std::string& icao)
            : airportIcao(icao)
        {
        }
    };

    /**
     * @brief Central flow management system
     * Coordinates arrival/departure flow rates, runway occupancy, and ground stop logic
     */
    class FlowManager
    {
    public:
        FlowManager();

        /**
         * @brief Set the airport for this flow manager
         */
        void setAirport(std::shared_ptr<Airport> airport);

        /**
         * @brief Configure flow parameters
         */
        void configure(const FlowManagerConfig& config);

        /**
         * @brief Get the current configuration
         */
        const FlowManagerConfig& getConfig() const { return m_config; }

        /**
         * @brief Check if an arrival can be accepted
         * @param flight The arriving flight
         * @param currentTime Current time
         * @return true if arrival can be accepted
         */
        bool canAcceptArrival(
            std::shared_ptr<Flight> flight,
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Check if a departure can be released
         * @param flight The departing flight
         * @param currentTime Current time
         * @return true if departure can be released
         */
        bool canReleaseDeparture(
            std::shared_ptr<Flight> flight,
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Record a runway entry for flow tracking
         * @param flight The flight entering the runway
         * @param runwayName Name of the runway
         * @param isDeparture true for departure, false for arrival
         */
        void recordRunwayEntry(
            std::shared_ptr<Flight> flight,
            const std::string& runwayName,
            bool isDeparture);

        /**
         * @brief Record a runway exit for flow tracking
         * @param flight The flight exiting the runway
         * @param runwayName Name of the runway
         */
        void recordRunwayExit(
            std::shared_ptr<Flight> flight,
            const std::string& runwayName);

        /**
         * @brief Initiate a ground stop
         * @param destinationIcao Destination airport ICAO
         * @param durationSeconds Duration in seconds
         * @param reason Reason for ground stop
         */
        void initiateGroundStop(
            const std::string& destinationIcao,
            int durationSeconds,
            const std::string& reason);

        /**
         * @brief Cancel a ground stop
         * @param destinationIcao Destination airport ICAO
         */
        void cancelGroundStop(const std::string& destinationIcao);

        /**
         * @brief Update flow state based on conditions
         * @param currentTime Current time
         */
        void updateFlowState(std::chrono::steady_clock::time_point currentTime);

        /**
         * @brief Get the current flow state
         */
        FlowState getFlowState() const;

        /**
         * @brief Get the estimated time when next arrival can be accepted
         */
        std::chrono::steady_clock::time_point getNextArrivalTime(
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Get the estimated time when next departure can be released
         */
        std::chrono::steady_clock::time_point getNextDepartureTime(
            std::chrono::steady_clock::time_point currentTime) const;

        /**
         * @brief Get the arrival flow controller
         */
        std::shared_ptr<ArrivalFlowController> arrivalController() const { return m_arrivalController; }

        /**
         * @brief Get the departure flow controller
         */
        std::shared_ptr<DepartureFlowController> departureController() const { return m_departureController; }

        /**
         * @brief Get the runway occupancy tracker
         */
        std::shared_ptr<RunwayOccupancyTracker> occupancyTracker() const { return m_occupancyTracker; }

        /**
         * @brief Get the ground stop manager
         */
        std::shared_ptr<GroundStopManager> groundStopManager() const { return m_groundStopManager; }

    private:
        FlowManagerConfig m_config;
        std::shared_ptr<Airport> m_airport;
        std::shared_ptr<FlowStateMachine> m_stateMachine;
        std::shared_ptr<RunwayOccupancyTracker> m_occupancyTracker;
        std::shared_ptr<ArrivalFlowController> m_arrivalController;
        std::shared_ptr<DepartureFlowController> m_departureController;
        std::shared_ptr<GroundStopManager> m_groundStopManager;
    };
}