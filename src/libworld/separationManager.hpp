//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include "libworld.h"

namespace world
{
    // Forward declarations
    class WakeTurbulenceCalculator;
    class VerticalSeparationRules;
    class RadarSeparationMinima;
    class ConflictDetector;

    using WakeTurbulenceCategory = Aircraft::WakeTurbulenceCategory;

    /**
     * @brief Controller facility type for radar separation rules
     */
    enum class FacilityType
    {
        Tower = 0,
        Approach = 1,
        Center = 2,
        Unknown = 3
    };

    /**
     * @brief Separation result containing all separation calculations
     */
    struct SeparationResult
    {
        float wakeTurbulenceSeconds = 0.0f;
        float verticalSeparationFeet = 0.0f;
        float radarSeparationNm = 0.0f;
        bool isConflict = false;
        std::chrono::seconds timeToConflictSeconds = std::chrono::seconds(0);
    };

    /**
     * @brief Central separation management system
     * Coordinates wake turbulence, vertical, and radar separation calculations
     */
    class SeparationManager
    {
    public:
        SeparationManager();

        /**
         * @brief Calculate required separation between two flights
         * @param leader Leading aircraft
         * @param follower Following aircraft
         * @param facilityType Type of controlling facility
         * @return Separation requirements
         */
        SeparationResult calculateSeparation(
            std::shared_ptr<Flight> leader,
            std::shared_ptr<Flight> follower,
            FacilityType facilityType = FacilityType::Unknown) const;

        /**
         * @brief Check if two flights are in conflict
         * @param flight1 First flight
         * @param flight2 Second flight
         * @param facilityType Type of controlling facility
         * @return true if conflict detected
         */
        bool checkConflict(
            std::shared_ptr<Flight> flight1,
            std::shared_ptr<Flight> flight2,
            FacilityType facilityType = FacilityType::Unknown) const;

        /**
         * @brief Get wake turbulence category for a flight
         */
        WakeTurbulenceCategory getWakeTurbulenceCategory(std::shared_ptr<Flight> flight) const;

        /**
         * @brief Get time-based wake turbulence separation in seconds
         * Heavy: 2 minutes, Large: 2 minutes, Small: 2 minutes, following Heavy: 2 minutes
         */
        int getWakeTurbulenceSeparationSeconds(WakeTurbulenceCategory leader, WakeTurbulenceCategory follower) const;

    private:
        std::unique_ptr<WakeTurbulenceCalculator> m_wakeTurbulenceCalculator;
        std::unique_ptr<VerticalSeparationRules> m_verticalSeparationRules;
        std::unique_ptr<RadarSeparationMinima> m_radarSeparationMinima;
    };
}