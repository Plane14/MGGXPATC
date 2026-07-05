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
     * @brief Vertical separation minima rules
     * Implements 1000ft below FL290, 2000ft at/above FL290
     */
    class VerticalSeparationRules
    {
    public:
        VerticalSeparationRules();

        /**
         * @brief Get required vertical separation in feet
         * @param altitude1 Altitude of first aircraft (MSL feet)
         * @param altitude2 Altitude of second aircraft (MSL feet)
         * @return Required vertical separation in feet
         */
        float getRequiredVerticalSeparation(float altitude1, float altitude2) const;

        /**
         * @brief Check if vertical separation is adequate
         * @param altitude1 Altitude of first aircraft (MSL feet)
         * @param altitude2 Altitude of second aircraft (MSL feet)
         * @return true if separation is adequate
         */
        bool isVerticalSeparationAdequate(float altitude1, float altitude2) const;

        /**
         * @brief Get minimum safe altitude for RVSM operations
         * @return 29000.0f (FL290)
         */
        static constexpr float RVSM_MINIMUM_ALTITUDE_FEET() { return 29000.0f; }

        /**
         * @brief Get vertical separation below FL290
         * @return 1000.0f feet
         */
        static constexpr float SEPARATION_BELOW_FL290_FEET() { return 1000.0f; }

        /**
         * @brief Get vertical separation at/above FL290
         * @return 2000.0f feet
         */
        static constexpr float SEPARATION_AT_OR_ABOVE_FL290_FEET() { return 2000.0f; }

        /**
         * @brief Check if altitude is at or above FL290
         */
        bool isAtOrAboveFL290(float altitudeFeet) const;

        /**
         * @brief Check RVSM compliance for two aircraft
         */
        bool checkRVSMCompliance(float altitude1, float altitude2) const;

        /**
         * @brief Get vertical separation for two flights
         */
        float getVerticalSeparation(std::shared_ptr<Flight> flight1, std::shared_ptr<Flight> flight2) const;
    };
}