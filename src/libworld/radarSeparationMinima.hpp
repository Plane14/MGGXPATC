//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <memory>
#include "libworld.h"

namespace world
{
    /**
     * @brief Radar separation minima rules
     * Implements 3NM for approach control, 5NM for enroute
     */
    class RadarSeparationMinima
    {
    public:
        RadarSeparationMinima();

        /**
         * @brief Controller facility type
         */
        enum class FacilityType
        {
            Tower = 0,
            Approach = 1,
            Center = 2,
            Unknown = 3
        };

        /**
         * @brief Get required radar separation in nautical miles
         * @param facilityType Type of controlling facility
         * @return Required separation in NM
         */
        float getRequiredSeparation(FacilityType facilityType) const;

        /**
         * @brief Check if radar separation is adequate
         * @param distanceNm Distance between aircraft in NM
         * @param facilityType Type of controlling facility
         * @return true if separation is adequate
         */
        bool isSeparationAdequate(float distanceNm, FacilityType facilityType) const;

        /**
         * @brief Get radar separation for two flights
         */
        float getRadarSeparation(std::shared_ptr<Flight> flight1, std::shared_ptr<Flight> flight2) const;

        /**
         * @brief Get distance between two flights in NM
         */
        float getDistanceNm(std::shared_ptr<Flight> flight1, std::shared_ptr<Flight> flight2) const;

        /**
         * @brief Radar separation minima constants
         */
        static constexpr float SEPARATION_APPROACH_NM() { return 3.0f; }
        static constexpr float SEPARATION_ENROUTE_NM() { return 5.0f; }
        static constexpr float SEPARATION_TOWER_NM() { return 3.0f; }
    };
}