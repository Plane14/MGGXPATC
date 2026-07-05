//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <string>
#include <vector>
#include <memory>
#include "libworld.h"

namespace world
{
    /**
     * @brief Wake turbulence time-based separation calculator
     * Implements ICAO Doc 4444 Table 8-1 standards
     */
    class WakeTurbulenceCalculator
    {
    public:
        WakeTurbulenceCalculator();

        /**
         * @brief Wake turbulence category enumeration
         */
        enum class WakeClass
        {
            Light = 0,
            Medium = 1,
            Heavy = 2,
            Super = 3
        };

        /**
         * @brief Separation profile for an aircraft
         */
        struct SeparationProfile
        {
            WakeClass wakeClass = WakeClass::Medium;
            float referenceArrivalSpeedKt = 145.0f;
            float departureRollSeconds = 35.0f;
            float lineupSeconds = 8.0f;
            float crossingSeconds = 18.0f;
            bool rotorcraft = false;
        };

        /**
         * @brief Infer wake class from aircraft model ICAO code
         */
        WakeClass inferWakeClass(std::shared_ptr<Flight> flight) const;

        /**
         * @brief Get separation profile for an aircraft
         */
        SeparationProfile getSeparationProfile(std::shared_ptr<Flight> flight) const;

        /**
         * @brief Get time-based wake turbulence separation in seconds
         * Heavy: 2 minutes, Large: 2 minutes, Small: 2 minutes, following Heavy: 2 minutes
         */
        int getWakeTurbulenceSeparationSeconds(WakeClass leader, WakeClass follower) const;

        /**
         * @brief Get required takeoff gap in seconds
         */
        float requiredTakeoffGapSeconds(
            std::shared_ptr<Flight> departure,
            std::shared_ptr<Flight> arrival) const;

        /**
         * @brief Get required LUAW (Line Up And Wait) gap in seconds
         */
        float requiredLuawGapSeconds(
            std::shared_ptr<Flight> departure,
            std::shared_ptr<Flight> arrival) const;

        /**
         * @brief Get required crossing gap in seconds
         */
        float requiredCrossingGapSeconds(
            std::shared_ptr<Flight> crossing,
            std::shared_ptr<Flight> arrival) const;

    private:
        static std::string uppercaseCopy(const std::string& value);
    };
}