//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include "libworld.h"
#include "conflictDetector.hpp"

namespace world
{
    /**
     * @brief Conflict resolution strategies
     * Provides resolution recommendations for detected conflicts
     */
    class ConflictResolver
    {
    public:
        ConflictResolver();

        /**
         * @brief Resolution strategy types
         */
        enum class ResolutionStrategy
        {
            None = 0,
            Vertical = 1,      // Altitude change
            Horizontal = 2,    // Heading change
            Speed = 3,         // Speed change
            Vectoring = 4,     // Combined vectoring
            Hold = 5,          // Hold pattern
            Reroute = 6        // Reroute to different path
        };

        /**
         * @brief Resolution recommendation
         */
        struct Resolution
        {
            ResolutionStrategy strategy = ResolutionStrategy::None;
            bool isRequired = false;
            std::shared_ptr<Flight> affectedFlight;
            std::shared_ptr<Flight> conflictingFlight;
            
            // Vertical resolution
            float targetAltitudeFeet = 0.0f;
            bool climb = false;
            bool descend = false;
            
            // Horizontal resolution
            float targetHeadingDegrees = 0.0f;
            float targetSpeedKt = 0.0f;
            
            // Timing
            std::chrono::seconds timeToConflictSeconds = std::chrono::seconds(0);
            
            // Priority
            int priority = 0;
        };

        /**
         * @brief Resolve a detected conflict
         * @param conflict Conflict information from ConflictDetector
         * @return Resolution recommendation
         */
        Resolution resolveConflict(const ConflictDetector::ConflictInfo& conflict) const;

        /**
         * @brief Get resolution priority for a flight
         * Higher priority flights get precedence in conflict resolution
         * @param flight Flight to evaluate
         * @return Priority value (higher = more important)
         */
        int getFlightPriority(std::shared_ptr<Flight> flight) const;

        /**
         * @brief Select best resolution strategy
         * @param conflict Conflict information
         * @return Recommended strategy
         */
        ResolutionStrategy selectResolutionStrategy(const ConflictDetector::ConflictInfo& conflict) const;

        /**
         * @brief Calculate vertical resolution
         * @param flight1 First flight
         * @param flight2 Second flight
         * @return Resolution with altitude change
         */
        Resolution calculateVerticalResolution(
            std::shared_ptr<Flight> flight1,
            std::shared_ptr<Flight> flight2) const;

        /**
         * @brief Calculate horizontal resolution
         * @param flight1 First flight
         * @param flight2 Second flight
         * @return Resolution with heading change
         */
        Resolution calculateHorizontalResolution(
            std::shared_ptr<Flight> flight1,
            std::shared_ptr<Flight> flight2) const;

        /**
         * @brief Calculate speed resolution
         * @param flight1 First flight
         * @param flight2 Second flight
         * @return Resolution with speed change
         */
        Resolution calculateSpeedResolution(
            std::shared_ptr<Flight> flight1,
            std::shared_ptr<Flight> flight2) const;
    };
}