//
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
//
#include "conflictDetector.hpp"
#include "verticalSeparationRules.hpp"
#include "radarSeparationMinima.hpp"
#include <cmath>
#include <algorithm>

using namespace std;

namespace world
{
    ConflictDetector::ConflictDetector()
    {
    }

    ConflictDetector::ConflictInfo ConflictDetector::checkConflict(
        shared_ptr<Flight> flight1,
        shared_ptr<Flight> flight2,
        const DetectionParams& params) const
    {
        ConflictInfo result;
        result.flight1 = flight1;
        result.flight2 = flight2;

        if (!flight1 || !flight2)
        {
            return result;
        }

        // Calculate CPA
        const auto cpa = calculateCPA(flight1, flight2, params.lookAheadSeconds);
        
        if (!cpa.isValid)
        {
            return result;
        }

        result.predictedDistanceNm = cpa.distanceNm;
        result.predictedVerticalSeparationFt = cpa.verticalSeparationFt;
        result.timeToCPASeconds = cpa.timeToCPASeconds;
        result.predictedCPAPosition = cpa.cpaPosition;

        // Check if separation is violated
        const VerticalSeparationRules vertRules;
        const float requiredVerticalSep = vertRules.getRequiredVerticalSeparation(
            flight1->aircraft() ? flight1->aircraft()->altitude().feet() : 0.0f,
            flight2->aircraft() ? flight2->aircraft()->altitude().feet() : 0.0f
        );

        // Conflict if horizontal distance is below threshold OR vertical separation is below required
        if (cpa.distanceNm < params.horizontalThresholdNm ||
            cpa.verticalSeparationFt < requiredVerticalSep)
        {
            result.hasConflict = true;
        }

        return result;
    }

    ConflictDetector::CPAResult ConflictDetector::calculateCPA(
        shared_ptr<Flight> flight1,
        shared_ptr<Flight> flight2,
        int lookAheadSeconds) const
    {
        CPAResult result;

        if (!flight1 || !flight2)
        {
            return result;
        }

        const auto pos1 = flight1->position();
        const auto pos2 = flight2->position();

        if (pos1 == GeoPoint::empty || pos2 == GeoPoint::empty)
        {
            return result;
        }

        // Get current positions and velocities
        const double lat1 = pos1.latitude;
        const double lon1 = pos1.longitude;
        const double lat2 = pos2.latitude;
        const double lon2 = pos2.longitude;

        const float alt1 = flight1->aircraft() ? flight1->aircraft()->altitude().feet() : 0.0f;
        const float alt2 = flight2->aircraft() ? flight2->aircraft()->altitude().feet() : 0.0f;

        // Get ground speeds and headings
        const float speed1Kt = flight1->aircraft() ? flight1->aircraft()->groundSpeedKt() : 0.0f;
        const float speed2Kt = flight2->aircraft() ? flight2->aircraft()->groundSpeedKt() : 0.0f;
        const float heading1 = flight1->aircraft() ? flight1->aircraft()->heading() : 0.0f;
        const float heading2 = flight2->aircraft() ? flight2->aircraft()->heading() : 0.0f;

        // Convert to m/s
        const double speed1Ms = speed1Kt * 0.514444f;
        const double speed2Ms = speed2Kt * 0.514444f;

        // Calculate relative velocity
        const double dlon = lon2 - lon1;
        const double dlat = lat2 - lat1;

        // Simple CPA calculation (assuming constant velocity)
        // This is a simplified 2D calculation
        const double distanceMeters = GeoMath::getDistanceMeters(pos1, pos2);
        const double distanceNm = distanceMeters / METERS_IN_1_NAUTICAL_MILE;

        // Calculate time to CPA based on closing speed
        const double closingSpeedMs = abs(speed1Ms - speed2Ms);
        if (closingSpeedMs > 0.1)
        {
            const double timeToCPASec = distanceMeters / closingSpeedMs;
            result.timeToCPASeconds = chrono::seconds(static_cast<int>(timeToCPASec));
        }
        else
        {
            // If speeds are similar, check if on collision course
            result.timeToCPASeconds = chrono::seconds(lookAheadSeconds);
        }

        result.isValid = true;
        result.distanceNm = static_cast<float>(distanceNm);
        result.verticalSeparationFt = abs(alt1 - alt2);
        result.cpaPosition = pos1; // Simplified - would need interpolation for actual CPA

        return result;
    }

    bool ConflictDetector::checkLossOfSeparation(shared_ptr<Flight> flight1, shared_ptr<Flight> flight2) const
    {
        if (!flight1 || !flight2)
        {
            return false;
        }

        const auto pos1 = flight1->position();
        const auto pos2 = flight2->position();

        if (pos1 == GeoPoint::empty || pos2 == GeoPoint::empty)
        {
            return false;
        }

        // Check horizontal separation
        const double distanceMeters = GeoMath::getDistanceMeters(pos1, pos2);
        const float distanceNm = static_cast<float>(distanceMeters / METERS_IN_1_NAUTICAL_MILE);

        // Check vertical separation
        const float alt1 = flight1->aircraft() ? flight1->aircraft()->altitude().feet() : 0.0f;
        const float alt2 = flight2->aircraft() ? flight2->aircraft()->altitude().feet() : 0.0f;
        const float verticalSep = abs(alt1 - alt2);

        // Use vertical separation rules
        VerticalSeparationRules vertRules;
        const float requiredVerticalSep = vertRules.getRequiredVerticalSeparation(alt1, alt2);

        // Loss of separation if either horizontal or vertical is below threshold
        return distanceNm < 5.0f || verticalSep < requiredVerticalSep;
    }

    bool ConflictDetector::checkPotentialConflict(
        shared_ptr<Flight> flight1,
        shared_ptr<Flight> flight2,
        int timeWindowSeconds) const
    {
        DetectionParams params;
        params.lookAheadSeconds = timeWindowSeconds;
        params.horizontalThresholdNm = 5.0f;
        params.verticalThresholdFeet = 1000.0f;

        const auto conflict = checkConflict(flight1, flight2, params);
        return conflict.hasConflict;
    }

    vector<pair<shared_ptr<Flight>, shared_ptr<Flight>>>
    ConflictDetector::findAllConflicts(
        const vector<shared_ptr<Flight>>& flights,
        const DetectionParams& params) const
    {
        vector<pair<shared_ptr<Flight>, shared_ptr<Flight>>> conflicts;

        for (size_t i = 0; i < flights.size(); ++i)
        {
            for (size_t j = i + 1; j < flights.size(); ++j)
            {
                const auto conflict = checkConflict(flights[i], flights[j], params);
                if (conflict.hasConflict)
                {
                    conflicts.push_back({ flights[i], flights[j] });
                }
            }
        }

        return conflicts;
    }
}