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
        static const double PI = 3.14159265358979323846;
        static const double EARTH_RADIUS_METERS = 6371000.0;
        static const double KT_TO_MPS = 0.514444;

        CPAResult result;
        result.timeToCPASeconds = chrono::seconds(lookAheadSeconds);

        if (!flight1 || !flight2 ||
            !flight1->aircraft() || !flight2->aircraft())
        {
            return result;
        }

        const auto pos1 = flight1->position();
        const auto pos2 = flight2->position();

        if (pos1 == GeoPoint::empty || pos2 == GeoPoint::empty)
        {
            return result;
        }

        // Ground speeds (kt) and ground tracks (degrees true).
        const float speed1Kt = static_cast<float>(flight1->aircraft()->groundSpeedKt());
        const float speed2Kt = static_cast<float>(flight2->aircraft()->groundSpeedKt());
        const float track1 = static_cast<float>(flight1->aircraft()->track());
        const float track2 = static_cast<float>(flight2->aircraft()->track());
        const float alt1 = flight1->aircraft()->altitude().feet();
        const float alt2 = flight2->aircraft()->altitude().feet();
        const float vs1Fpm = static_cast<float>(flight1->aircraft()->verticalSpeedFpm());
        const float vs2Fpm = static_cast<float>(flight2->aircraft()->verticalSpeedFpm());

        // Local north-east tangent plane centered on pos1 (meters).
        const double lat1Rad = pos1.latitude * PI / 180.0;
        const double lon1Rad = pos1.longitude * PI / 180.0;
        const double lat2Rad = pos2.latitude * PI / 180.0;
        const double lon2Rad = pos2.longitude * PI / 180.0;

        // Relative position of aircraft 2 with respect to aircraft 1.
        const double rNorth = EARTH_RADIUS_METERS * (lat2Rad - lat1Rad);
        const double rEast = EARTH_RADIUS_METERS * (lon2Rad - lon1Rad) * cos((lat1Rad + lat2Rad) / 2.0);

        // Velocity of each aircraft in the local north-east frame (m/s).
        const double t1Rad = track1 * PI / 180.0;
        const double t2Rad = track2 * PI / 180.0;
        const double v1North = speed1Kt * KT_TO_MPS * cos(t1Rad);
        const double v1East = speed1Kt * KT_TO_MPS * sin(t1Rad);
        const double v2North = speed2Kt * KT_TO_MPS * cos(t2Rad);
        const double v2East = speed2Kt * KT_TO_MPS * sin(t2Rad);

        // Relative velocity of aircraft 2 with respect to aircraft 1.
        const double vRelNorth = v2North - v1North;
        const double vRelEast = v2East - v1East;
        const double vRelSquared = vRelNorth * vRelNorth + vRelEast * vRelEast;

        // Time of closest point of approach within the look-ahead window.
        // t* = -dot(r, vRel) / |vRel|^2, clamped to [0, lookAheadSeconds].
        double tOfCpa = 0.0;
        if (vRelSquared > 1e-9)
        {
            const double tStar = -(rNorth * vRelNorth + rEast * vRelEast) / vRelSquared;
            tOfCpa = std::max(0.0, std::min(static_cast<double>(lookAheadSeconds), tStar));
        }
        else
        {
            // No relative horizontal motion: separation is constant; CPA is now.
            tOfCpa = 0.0;
        }

        // Projected separation at tOfCpa.
        const double sepNorth = rNorth + vRelNorth * tOfCpa;
        const double sepEast = rEast + vRelEast * tOfCpa;
        const double sepMeters = std::sqrt(sepNorth * sepNorth + sepEast * sepEast);
        const float distanceNm = static_cast<float>(sepMeters / METERS_IN_1_NAUTICAL_MILE);

        // CPA position projected back to lat/lon from pos1.
        const double cpaLatDeg = pos1.latitude + (sepNorth / EARTH_RADIUS_METERS) * 180.0 / PI;
        const double cpaLonDeg = pos1.longitude +
            (sepEast / (EARTH_RADIUS_METERS * std::cos(lat1Rad))) * 180.0 / PI;

        // Project vertical separation to the same CPA time (vsFpm -> feet/sec via /60).
        const double alt1Proj = alt1 + vs1Fpm * tOfCpa / 60.0;
        const double alt2Proj = alt2 + vs2Fpm * tOfCpa / 60.0;
        const float verticalSeparationFt = static_cast<float>(std::abs(alt1Proj - alt2Proj));

        result.isValid = true;
        result.distanceNm = distanceNm;
        result.verticalSeparationFt = verticalSeparationFt;
        result.timeToCPASeconds = chrono::seconds(static_cast<int>(tOfCpa));
        result.cpaPosition = GeoPoint(cpaLatDeg, cpaLonDeg, pos1.altitude);

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