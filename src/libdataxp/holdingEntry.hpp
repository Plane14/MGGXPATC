// 
// This file is part of AT&C project which simulates virtual world of air traffic and ATC.
// Code licensing terms are available at https://github.com/felix-b/atc/blob/master/LICENSE
// 
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../libworld/libworld.h"

using namespace std;
using namespace world;

class XPHoldingEntryCalculator
{
public:
    enum class EntryType
    {
        Direct,      // Direct entry - turn to inbound course
        Parallel,    // Parallel entry - fly outbound, turn 180, intercept inbound
        Teardrop,    // Teardrop entry - fly outbound at 30 degrees offset, turn to intercept
        Unknown
    };

    struct EntryProcedure
    {
        EntryType type = EntryType::Unknown;
        float outboundCourse = 0.0f;     // Degrees true (for teardrop/parallel)
        float teardropAngle = 30.0f;     // Degrees offset from outbound course
        float turnDirection = 1.0f;      // 1.0 = right, -1.0 = left
        string description;

        string toString() const
        {
            switch (type)
            {
            case EntryType::Direct:
                return "Direct entry";
            case EntryType::Parallel:
                return "Parallel entry";
            case EntryType::Teardrop:
                return "Teardrop entry";
            default:
                return "Unknown entry";
            }
        }
    };

    // Calculate the holding entry type based on the angle between the aircraft's
    // current track and the holding pattern's inbound course.
    // 
    // Standard sector definitions (ICAO Doc 9432):
    // - Sector 1 (Direct): 0 to 70 degrees on the right side of inbound course
    // - Sector 2 (Teardrop): 70 to 180 degrees on the right side of inbound course
    // - Sector 3 (Parallel): 180 to 110 degrees on the left side of inbound course
    // - Sector 4 (Direct): 110 to 0 degrees on the left side of inbound course
    //
    // Simplified: 
    // - Angle < 70 degrees: Direct entry
    // - Angle 70-180 degrees on right: Teardrop entry
    // - Angle 180-110 degrees on left: Parallel entry
    // - Angle < 110 degrees on left: Direct entry
    static EntryType calculateEntryType(float inboundCourse, float currentTrack)
    {
        // Calculate the angle from inbound course to current track
        float angle = normalizeAngle(currentTrack - inboundCourse);

        // Sector definitions based on angle from inbound course
        // 0 degrees = flying inbound, 180 degrees = flying outbound
        if (angle < 70.0f)
        {
            // Sector 1: Direct entry (right side, 0-70 degrees)
            return EntryType::Direct;
        }
        else if (angle < 180.0f)
        {
            // Sector 2: Teardrop entry (right side, 70-180 degrees)
            return EntryType::Teardrop;
        }
        else if (angle > 290.0f)  // 360 - 70 = 290
        {
            // Sector 4: Direct entry (left side, 290-360 degrees)
            return EntryType::Direct;
        }
        else
        {
            // Sector 3: Parallel entry (left side, 180-290 degrees)
            return EntryType::Parallel;
        }
    }

    static EntryProcedure calculateEntryProcedure(
        float inboundCourse,
        float currentTrack,
        bool isRightTurnHolding)
    {
        EntryProcedure procedure;
        procedure.turnDirection = isRightTurnHolding ? 1.0f : -1.0f;

        const EntryType entryType = calculateEntryType(inboundCourse, currentTrack);
        procedure.type = entryType;

        switch (entryType)
        {
        case EntryType::Direct:
            procedure.description = "Direct entry: Turn to intercept inbound course";
            break;

        case EntryType::Parallel:
            procedure.outboundCourse = inboundCourse;  // Fly outbound on inbound course
            procedure.description = "Parallel entry: Fly outbound, turn 180, intercept inbound";
            break;

        case EntryType::Teardrop:
        {
            // Teardrop outbound course is 30 degrees from outbound course
            // For right turn holding: offset to the right (add 30)
            // For left turn holding: offset to the left (subtract 30)
            const float outboundCourse = inboundCourse;  // Outbound is reciprocal of inbound
            const float offset = isRightTurnHolding ? 30.0f : -30.0f;
            procedure.outboundCourse = normalizeAngle(outboundCourse + offset);
            procedure.teardropAngle = 30.0f;
            procedure.description = "Teardrop entry: Fly outbound at 30 degrees offset, turn to intercept";
            break;
        }

        default:
            procedure.description = "Unknown entry type";
            break;
        }

        return procedure;
    }

    // Calculate the outbound leg length for timed entries
    // Standard: 1 minute at or below 14,000 feet, 1.5 minutes above 14,000 feet
    static float calculateOutboundLegTime(float altitudeFeet)
    {
        if (altitudeFeet >= 14000.0f)
        {
            return 1.5f;
        }
        return 1.0f;
    }

    // Calculate the outbound leg distance for distance-based entries
    // Standard: 1 NM per 1000 feet of altitude (minimum 1 NM)
    static float calculateOutboundLegDistance(float altitudeFeet)
    {
        return max(1.0f, altitudeFeet / 1000.0f);
    }

    // Calculate the turn radius for a given speed and turn rate
    static float calculateTurnRadius(float speedKnots, float turnRateDegreesPerSecond = 3.0f)
    {
        if (speedKnots <= 0.0f || turnRateDegreesPerSecond <= 0.0f)
        {
            return 0.0f;
        }

        // Radius (NM) = speed (knots) / (turnRate * 10.2)
        return speedKnots / (turnRateDegreesPerSecond * 10.2f);
    }

    // Calculate the lead distance needed to start the turn to intercept inbound course
    // This accounts for the turn radius and the angle of interception
    static float calculateLeadDistance(
        float speedKnots,
        float inboundCourse,
        float outboundCourse,
        float turnRateDegreesPerSecond = 3.0f)
    {
        const float turnRadius = calculateTurnRadius(speedKnots, turnRateDegreesPerSecond);
        if (turnRadius <= 0.0f)
        {
            return 0.0f;
        }

        // Angle between outbound and inbound courses
        float angle = normalizeAngle(inboundCourse - outboundCourse);
        if (angle > 180.0f)
        {
            angle = 360.0f - angle;
        }

        // Lead distance = turn radius * tan(angle/2)
        // This is the distance from the end of the outbound leg to start the turn
        const float leadDistance = turnRadius * tan(angle * 0.5f * M_PI / 180.0f);
        return max(0.0f, leadDistance);
    }

private:
    static float normalizeAngle(float angle)
    {
        while (angle < 0.0f)
        {
            angle += 360.0f;
        }
        while (angle >= 360.0f)
        {
            angle -= 360.0f;
        }
        return angle;
    }
};
