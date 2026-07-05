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
    /**
     * @brief Conflict detection algorithms
     * Predicts conflicts 1-5 minutes ahead and calculates CPA (Closest Point of Approach)
     */
    class ConflictDetector
    {
    public:
        ConflictDetector();

        /**
         * @brief Conflict detection parameters
         */
        struct DetectionParams
        {
            // Look-ahead time window in seconds (default: 300 = 5 minutes)
            int lookAheadSeconds;
            // Horizontal separation threshold in NM (default: 5 NM)
            float horizontalThresholdNm;
            // Vertical separation threshold in feet (default: 1000 ft)
            float verticalThresholdFeet;
            // Minimum altitude for vertical separation (FL290 = 29000 ft)
            float rvsmTransitionAltitudeFeet;

            DetectionParams()
                : lookAheadSeconds(300),
                  horizontalThresholdNm(5.0f),
                  verticalThresholdFeet(1000.0f),
                  rvsmTransitionAltitudeFeet(29000.0f)
            {
            }

            DetectionParams(int look, float horiz, float vert, float rvsm)
                : lookAheadSeconds(look),
                  horizontalThresholdNm(horiz),
                  verticalThresholdFeet(vert),
                  rvsmTransitionAltitudeFeet(rvsm)
            {
            }
        };

        /**
         * @brief Conflict information
         */
        struct ConflictInfo
        {
            bool hasConflict = false;
            std::shared_ptr<Flight> flight1;
            std::shared_ptr<Flight> flight2;
            float predictedDistanceNm = 0.0f;
            float predictedVerticalSeparationFt = 0.0f;
            std::chrono::seconds timeToCPASeconds = std::chrono::seconds(0);
            GeoPoint predictedCPAPosition;
        };

        /**
         * @brief CPA (Closest Point of Approach) calculation result
         */
        struct CPAResult
        {
            bool isValid = false;
            float distanceNm = 0.0f;
            float verticalSeparationFt = 0.0f;
            std::chrono::seconds timeToCPASeconds = std::chrono::seconds(0);
            GeoPoint cpaPosition;
        };

        /**
         * @brief Check for conflict between two flights
         * @param flight1 First flight
         * @param flight2 Second flight
         * @param params Detection parameters
         * @return Conflict information
         */
        ConflictInfo checkConflict(
            std::shared_ptr<Flight> flight1,
            std::shared_ptr<Flight> flight2,
            const DetectionParams& params = DetectionParams()) const;

        /**
         * @brief Calculate CPA between two flights
         * @param flight1 First flight
         * @param flight2 Second flight
         * @param lookAheadSeconds Maximum look-ahead time
         * @return CPA result
         */
        CPAResult calculateCPA(
            std::shared_ptr<Flight> flight1,
            std::shared_ptr<Flight> flight2,
            int lookAheadSeconds = 300) const;

        /**
         * @brief Check for loss of separation
         * @param flight1 First flight
         * @param flight2 Second flight
         * @return true if separation is lost
         */
        bool checkLossOfSeparation(std::shared_ptr<Flight> flight1, std::shared_ptr<Flight> flight2) const;

        /**
         * @brief Check for potential conflict in time window
         * @param flight1 First flight
         * @param flight2 Second flight
         * @param timeWindowSeconds Time window to check
         * @return true if potential conflict exists
         */
        bool checkPotentialConflict(
            std::shared_ptr<Flight> flight1,
            std::shared_ptr<Flight> flight2,
            int timeWindowSeconds) const;

        /**
         * @brief Find all conflicts in a set of flights
         * @param flights Vector of flights to check
         * @param params Detection parameters
         * @return Vector of conflict pairs
         */
        std::vector<std::pair<std::shared_ptr<Flight>, std::shared_ptr<Flight>>>
        findAllConflicts(
            const std::vector<std::shared_ptr<Flight>>& flights,
            const DetectionParams& params = DetectionParams()) const;
    };
}